"""
web_control_node.py
-------------------
기존 PyQt5 GUI(gui_node.py)가 담당하던 "물체 선택 + 로봇/그리퍼 제어 + 파라미터
튜닝" 기능을 화면(Qt) 없이 SQLite 데이터베이스와 HTTP 서버로 대체하는 헤드리스 노드.

설계 개요
=========
기존 GUI는 Qt 메인 루프와 ROS spin을 한 스레드에서 돌렸지만, 여기서는 스레드 안전을
위해 SQLite를 "명령 버스 + 상태 저장소"로 사용한다.

  HTTP 스레드 (요청마다 1개)         ROS 스레드 (rclpy.spin, 단일)
  ─────────────────────────         ─────────────────────────────
  POST /api/command  ──INSERT──▶  command_queue (status=pending)
                                       │  50ms 타이머가 drain
                                       ▼
                                  ROS 서비스/파라미터 호출 실행
                                       │  done_callback
                                       ▼
                                  command_queue (status=done/failed)
  GET  /api/command/{id} ◀─SELECT─────┘

  ROS 콜백(상태/검출물체) ──UPSERT──▶  state / detected_objects 테이블
  GET  /api/state        ◀──SELECT────┘

이렇게 하면 rclpy 객체는 항상 ROS 스레드에서만 만지고, HTTP 스레드는 DB만 읽고 쓴다.
SQLite는 WAL 모드 + busy_timeout 으로 교차 연결 동시 접근을 처리한다.

DB 테이블
=========
  settings          (key, value)            - 최신 설정값(JSON). DB가 설정의 단일 출처.
  command_queue     (id, ts, action, ...)   - 명령 큐 겸 감사 로그.
  state             (key, value, ts)         - 실시간 상태 스냅샷.
  detected_objects  (label, pose, ...)       - 검출 물체(매 갱신마다 교체).

HTTP 엔드포인트
==============
  GET  /                      제어용 웹 페이지(브라우저 "창")
  GET  /api/state             실시간 상태 + 검출 물체 + 시스템 상태점
  GET  /api/settings          저장된 설정값
  GET  /api/commands          최근 명령 로그
  GET  /api/command/{id}      단일 명령 결과(폴링)
  GET  /api/image.jpg         디버그 영상 1프레임(JPEG)
  POST /api/command           {action, payload} → 명령 큐에 적재

기존 gui_node.py 는 그대로 두고(런치에서 주석 처리), 이 노드가 동일한 ROS 인터페이스
(서비스/토픽/파라미터 이름)를 그대로 사용한다.
"""

import json
import os
import re
import socket as _socket
import sqlite3
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from rcl_interfaces.msg import Parameter as RclParameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from sensor_msgs.msg import Image, JointState, Range
from std_msgs.msg import Int32, String
from std_srvs.srv import SetBool, Trigger
from dsr_gripper_tcp_interfaces.msg import GripperState
from dsr_realsense_pick_place.task_repository import HybridRepository, OrderStatus, ItemStatus

try:
    from cv_bridge import CvBridge
    import cv2
    _CV_OK = True
    _CV_ERR = None
except Exception as e:  # pragma: no cover - 런타임 환경 의존
    CvBridge = None
    cv2 = None
    _CV_OK = False
    _CV_ERR = e


# ─────────────────────────────────────────────────────────────────────────────
# 명령(action) → ROS Trigger 서비스 클라이언트 매핑.
# 기존 GUI의 버튼들이 호출하던 서비스를 그대로 사용한다.
# ─────────────────────────────────────────────────────────────────────────────
TRIGGER_ACTIONS = {
    'run_once':         'cli_run_once',
    'sort_all':         'cli_sort_all',
    'run_once_package': 'cli_run_once_package',
    'go_home':          'cli_go_home',
    'recover_to_home':  'cli_recover_to_home',
    'clear_error':      'cli_clear_error',
    'e_stop':           'cli_e_stop',
    'cancel':           'cli_cancel',
    'e_stop_reset':     'cli_e_stop_reset',
    'speed_normal':     'cli_speed_normal',
    'speed_reduced':    'cli_speed_reduced',
    'servo_off':        'cli_servo_off',
    'servo_on':         'cli_servo_on',
    'safety_normal':    'cli_safety_normal',
    'safety_backdrive': 'cli_safety_backdrive',
    'gripper_open':     'cli_gripper_open',
    'gripper_close':    'cli_gripper_close',
    'gripper_reinit':   'cli_gripper_reinit',
}

# DB가 비어 있을 때 시드(seed)할 기본 설정값. config/pick_place_params.yaml 의 값과 동일.
DEFAULT_SETTINGS = {
    'confidence_threshold': 0.5,
    'calib_x_mm': -195.0,
    'calib_y_mm': 10.0,
    'calib_z_mm': 125.0,
    'yolo_model_path': '',
    'camera_auto_exposure': True,
    'camera_exposure': 1500,
    'gripper_open_current': 400,
    'gripper_close_current': 300,
    'gripper_transport_current': 150,
    'gripper_profile_velocity': 1500,
    'gripper_profile_acceleration': 1000,
    'min_safe_z': 0.0,
    'grip_current_default': 200,
    'grip_class_names': ["ramen", "pack", "ssnack", "bsnack", "water",
                         "jelly", "can", "boxsnack", "wafers"],
    'grip_class_currents': [150, 150, 150, 120, 180, 140, 200, 170, 150],
}


# ─────────────────────────────────────────────────────────────────────────────
# DB 설정 ↔ config/pick_place_params.yaml 키 매핑.
# 기존 GUI(_grip_strength_save / _gripper_ctrl_save / _min_safe_z_save)가 하던
# "yaml 해당 라인만 값 교체(인라인 주석 보존)" 영구 저장을 그대로 재현한다.
# (DB설정키, yaml키, 값 포맷터)
def _fmt_str_list(v):
    return '[' + ', '.join(f'"{x}"' for x in v) + ']'


def _fmt_int_list(v):
    return '[' + ', '.join(str(int(x)) for x in v) + ']'


YAML_MAP = [
    ('confidence_threshold',         'confidence_threshold',  lambda v: f'{float(v):.3f}'),
    ('calib_x_mm',                   'absolute_calib_x_mm',   lambda v: f'{float(v):.1f}'),
    ('calib_y_mm',                   'absolute_calib_y_mm',   lambda v: f'{float(v):.1f}'),
    ('calib_z_mm',                   'absolute_calib_z_mm',   lambda v: f'{float(v):.1f}'),
    ('gripper_open_current',         'open_current',          lambda v: str(int(v))),
    ('gripper_close_current',        'close_current',         lambda v: str(int(v))),
    ('gripper_transport_current',    'transport_current',     lambda v: str(int(v))),
    ('gripper_profile_velocity',     'profile_velocity',      lambda v: str(int(v))),
    ('gripper_profile_acceleration', 'profile_acceleration',  lambda v: str(int(v))),
    ('min_safe_z',                   'min_safe_z',            lambda v: f'{float(v):.3f}'),
    ('grip_current_default',         'grip_current_default',  lambda v: str(int(v))),
    ('grip_class_names',             'grip_class_names',      _fmt_str_list),
    ('grip_class_currents',          'grip_class_currents',   _fmt_int_list),
]

# yaml에 라인이 없을 수 있는(=누락 허용) 키. transport_current는 노드 기본값만 있고
# yaml에는 라인이 없을 수 있다. known_classes는 grip_class_names를 미러링한다.
OPTIONAL_YAML_KEYS = {'transport_current', 'known_classes'}

# 이 action들이 처리되면 변경값을 yaml에도 반영한다(카메라/모델은 yaml 미저장 — 기존 GUI와 동일).
YAML_SYNC_ACTIONS = {
    'set_confidence', 'set_calibration', 'set_gripper_params',
    'set_min_safe_z', 'set_grip_strength',
}


class WebControlNode(Node):
    def __init__(self):
        super().__init__('web_control_node')

        # ── 파라미터 ────────────────────────────────────────────────────────
        self.declare_parameter('http_host', '0.0.0.0')
        self.declare_parameter('http_port', 8080)
        self.declare_parameter('db_path', '')   # 빈 값이면 ~/.config 아래 기본 경로 사용
        self.declare_parameter('serve_image', True)

        # 도달 가능 영역 필터(기존 GUI와 동일) — 범위 밖 물체는 reachable=false 로 표시.
        self.declare_parameter('workspace_x_min', 0.15)
        self.declare_parameter('workspace_x_max', 0.80)
        self.declare_parameter('workspace_y_min', -0.60)
        self.declare_parameter('workspace_y_max', 0.60)
        self.declare_parameter('workspace_z_min', 0.0)
        self.declare_parameter('workspace_z_max', 0.60)
        self.declare_parameter('reach_radius_max', 0.65)
        self.workspace_x_min = float(self.get_parameter('workspace_x_min').value)
        self.workspace_x_max = float(self.get_parameter('workspace_x_max').value)
        self.workspace_y_min = float(self.get_parameter('workspace_y_min').value)
        self.workspace_y_max = float(self.get_parameter('workspace_y_max').value)
        self.workspace_z_min = float(self.get_parameter('workspace_z_min').value)
        self.workspace_z_max = float(self.get_parameter('workspace_z_max').value)
        self.reach_radius_max = float(self.get_parameter('reach_radius_max').value)

        self.serve_image = bool(self.get_parameter('serve_image').value) and _CV_OK
        self.bridge = CvBridge() if (self.serve_image and CvBridge is not None) else None

        # ── 상태 캐시(ROS 스레드 전용) ──────────────────────────────────────
        self.pick_place_state = 'IDLE'
        self._pending_calib = None  # {'x', 'y', 'z'} — DETECTING 진입 시 적용 대기
        self.last_error_text = ''
        self.hw_state = -1
        self.speed_mode = 0
        self.gripper_hw_ready = False
        self.gripper_present_position = 0.0
        self.gripper_present_current = 0.0
        self.gripper_init_progress = ''
        self.gripper_init_progress_t = 0.0
        self.ultrasonic_range_m = None
        self.detected_objects = []
        self._last_nonempty_objects = []
        self._last_nonempty_objects_time = 0.0

        self.last_image_time = 0.0
        self.last_objects_time = 0.0
        self.last_state_time = 0.0
        self.last_hw_state_time = 0.0
        self.last_speed_mode_time = 0.0
        self.last_ultrasonic_time = 0.0
        self.current_model_path = ''  # set_model 성공 시 갱신

        # HTTP 스레드가 읽는 최신 JPEG 프레임(불변 bytes, GIL 하에서 참조 교체는 원자적).
        self.latest_jpeg = None

        # 시스템 리셋 / 그리퍼 브릿지 재시작 subprocess 추적.
        self._system_reset_proc = None
        self._system_restart_proc = None
        self._system_reset_phase = ''
        self._gripper_bridge_proc = None

        # ── ROS 인터페이스(기존 gui_node.py 와 동일한 이름) ──────────────────
        self.pub_selected = self.create_publisher(String, '/selected_object_label', 10)

        self.cli_run_once      = self.create_client(Trigger, '/pick_place/run_once')
        self.cli_sort_all          = self.create_client(Trigger, '/pick_place/sort_all')
        self.cli_run_once_package  = self.create_client(Trigger, '/pick_place/run_once_package')
        self.cli_go_home       = self.create_client(Trigger, '/pick_place/go_home')
        self.cli_gripper_open  = self.create_client(Trigger, '/gripper/open')
        self.cli_gripper_close = self.create_client(Trigger, '/gripper/close')
        self.cli_gripper_reinit = self.create_client(Trigger, '/gripper_service/reinitialize')
        self.cli_gripper_enable = self.create_client(SetBool, '/gripper/enable')
        self.cli_recover_to_home = self.create_client(Trigger, '/pick_place/recover_to_home')
        self.cli_clear_error     = self.create_client(Trigger, '/pick_place/clear_error')
        self.cli_e_stop        = self.create_client(Trigger, '/pick_place/e_stop')
        self.cli_cancel        = self.create_client(Trigger, '/pick_place/cancel')
        self.cli_e_stop_reset  = self.create_client(Trigger, '/pick_place/e_stop_reset')
        self.cli_speed_normal     = self.create_client(Trigger, '/pick_place/speed_normal')
        self.cli_speed_reduced    = self.create_client(Trigger, '/pick_place/speed_reduced')
        self.cli_servo_off        = self.create_client(Trigger, '/pick_place/servo_off')
        self.cli_servo_on         = self.create_client(Trigger, '/pick_place/servo_on')
        self.cli_safety_normal    = self.create_client(Trigger, '/pick_place/safety_normal')
        self.cli_safety_backdrive = self.create_client(Trigger, '/pick_place/safety_backdrive')

        self.cli_object_get_parameters = self.create_client(GetParameters, '/object_detector/get_parameters')
        self.cli_object_set_parameters = self.create_client(SetParameters, '/object_detector/set_parameters')
        self.cli_pickplace_set_parameters = self.create_client(SetParameters, '/pick_place_node/set_parameters')
        self.cli_gripper_set_parameters = self.create_client(SetParameters, '/rh_p12_rna_gripper/set_parameters')
        self.cli_camera_set_parameters = self.create_client(SetParameters, '/camera/camera/set_parameters')

        self.create_subscription(Int32, '/robot_hw_state',  self._cb_hw_state, 10)
        self.create_subscription(Int32, '/robot_speed_mode', self._cb_speed_mode, 10)
        self.create_subscription(GripperState, '/gripper_service/state', self._cb_gripper_service_state, 10)
        self.create_subscription(JointState, '/gripper/state', self._cb_gripper_joint_state, 10)
        self.create_subscription(String, '/gripper_service/init_progress', self._cb_gripper_init_progress, 10)
        self.create_subscription(String, '/detected_objects', self._cb_objects, 10)
        self.create_subscription(String, '/pick_place_state', self._cb_state, 10)
        self.create_subscription(String, '/pick_place_error', self._cb_error, 10)
        self.create_subscription(Range, '/ultrasonic_range', self._cb_ultrasonic, 10)
        if self.serve_image and self.bridge is not None:
            self.create_subscription(Image, '/detection_debug_image', self._cb_image, qos_profile_sensor_data)

        # ── SQLite 초기화 ───────────────────────────────────────────────────
        self.db_path = self._resolve_db_path()
        self.db = self._open_db(self.db_path)   # ROS 스레드 전용 연결
        self._init_schema()
        self._seed_settings()

        # ── 유저 주문 큐 저장소(같은 패키지의 task_repository) ─────────────────
        # 관리자 웹이 유저 주문 큐를 읽기 위해 동일한 JSON DB를 공유한다(읽기 전용 위주).
        try:
            self._order_repo = HybridRepository()   # 영속=SQLite(store.db 공유), 휘발=JSON
        except Exception:
            self._order_repo = None

        # ── HTTP 서버 스레드 시작 ───────────────────────────────────────────
        self._start_http_server()

        # ── 주기 타이머 ─────────────────────────────────────────────────────
        # 50ms: 명령 큐 drain / 200ms: 상태 스냅샷 기록 / 1s: DB 설정을 노드에 적용 시도.
        self._startup_settings_applied = False
        self.create_timer(0.05, self._drain_command_queue)
        self.create_timer(0.20, self._write_state_snapshot)
        self.create_timer(1.00, self._maybe_apply_startup_settings)

        self.get_logger().info(
            f'web_control_node 시작: http://{self.http_host}:{self.http_port}  (db={self.db_path})')

    # ========================================================================
    # SQLite
    # ========================================================================
    def _resolve_db_path(self) -> Path:
        configured = str(self.get_parameter('db_path').value).strip()
        if configured:
            p = Path(configured).expanduser()
        else:
            p = Path.home() / '.config' / 'dsr_realsense_pick_place' / 'web_control.db'
        p.parent.mkdir(parents=True, exist_ok=True)
        return p

    @staticmethod
    def _open_db(path: Path) -> sqlite3.Connection:
        conn = sqlite3.connect(str(path), check_same_thread=False, timeout=5.0)
        conn.row_factory = sqlite3.Row
        conn.execute('PRAGMA journal_mode=WAL;')
        conn.execute('PRAGMA busy_timeout=5000;')
        conn.execute('PRAGMA synchronous=NORMAL;')
        return conn

    def _init_schema(self):
        cur = self.db.cursor()
        cur.executescript(
            """
            CREATE TABLE IF NOT EXISTS settings (
                key   TEXT PRIMARY KEY,
                value TEXT NOT NULL
            );
            CREATE TABLE IF NOT EXISTS command_queue (
                id      INTEGER PRIMARY KEY AUTOINCREMENT,
                ts      REAL NOT NULL,
                action  TEXT NOT NULL,
                payload TEXT NOT NULL DEFAULT '{}',
                status  TEXT NOT NULL DEFAULT 'pending',  -- pending|running|done|failed
                result  TEXT NOT NULL DEFAULT '',
                done_ts REAL
            );
            CREATE TABLE IF NOT EXISTS state (
                key   TEXT PRIMARY KEY,
                value TEXT NOT NULL,
                ts    REAL NOT NULL
            );
            CREATE TABLE IF NOT EXISTS detected_objects (
                id         INTEGER PRIMARY KEY AUTOINCREMENT,
                label      TEXT,
                confidence REAL,
                depth_m    REAL,
                pixel_u    INTEGER,
                pixel_v    INTEGER,
                pose_x     REAL,
                pose_y     REAL,
                pose_z     REAL,
                reachable  INTEGER,
                ts         REAL
            );
            CREATE TABLE IF NOT EXISTS error_log (
                id        INTEGER PRIMARY KEY AUTOINCREMENT,
                ts        REAL NOT NULL,
                level     TEXT NOT NULL DEFAULT 'WAR',
                text      TEXT NOT NULL,
                ticket_no TEXT NOT NULL DEFAULT ''
            );
            """
        )
        self.db.commit()

    def _read_yaml_calib(self):
        """config/pick_place_params.yaml 의 object_detector.absolute_calib_* 실측값을 읽는다.

        반환: {'calib_x_mm': float, 'calib_y_mm': float, 'calib_z_mm': float} 또는 None.
        DB의 플레이스홀더가 아니라 yaml이 캘리브의 진실(single source)이다.
        """
        path = None
        # 1) 노드 params_file 파라미터(있다면) 우선.
        try:
            pf = str(self.get_parameter('params_file').value).strip()
            if pf and Path(pf).expanduser().is_file():
                path = Path(pf).expanduser()
        except Exception:
            path = None
        # 2) 소스 트리의 yaml(인라인 저장 대상과 동일 경로).
        if path is None:
            path = self._find_source_params_yaml()
        # 3) 패키지 share 의 yaml 폴백.
        if path is None:
            try:
                from ament_index_python.packages import get_package_share_directory
                cand = Path(get_package_share_directory('dsr_realsense_pick_place')) \
                    / 'config' / 'pick_place_params.yaml'
                if cand.is_file():
                    path = cand
            except Exception:
                path = None
        if path is None:
            return None
        try:
            import yaml
            with open(path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
            params = (data.get('object_detector', {}) or {}).get('ros__parameters', {}) or {}
            out = {}
            for skey, ykey in (('calib_x_mm', 'absolute_calib_x_mm'),
                               ('calib_y_mm', 'absolute_calib_y_mm'),
                               ('calib_z_mm', 'absolute_calib_z_mm')):
                if ykey in params and params[ykey] is not None:
                    out[skey] = float(params[ykey])
            return out if len(out) == 3 else None
        except Exception as e:
            self.get_logger().warn(f'yaml 캘리브 읽기 실패(기본값 폴백): {e}')
            return None

    def _seed_settings(self):
        """DB에 없는 설정 키만 기본값으로 채운다(기존 값 보존).

        단 calib_x/y/z 3키는 yaml 실측값을 진실로 보고 항상 갱신(UPSERT)한다.
        사용자가 대시보드에서 바꾸기 전까진 yaml(launch/실측)이 우선.
        """
        cur = self.db.cursor()
        for key, val in DEFAULT_SETTINGS.items():
            cur.execute('INSERT OR IGNORE INTO settings(key, value) VALUES (?, ?)',
                        (key, json.dumps(val)))
        # calib 은 yaml 실측값으로 강제 갱신(DB 플레이스홀더가 yaml을 덮어쓰지 못하게).
        yaml_calib = self._read_yaml_calib()
        if yaml_calib:
            for key, val in yaml_calib.items():
                cur.execute(
                    'INSERT INTO settings(key, value) VALUES (?, ?) '
                    'ON CONFLICT(key) DO UPDATE SET value=excluded.value',
                    (key, json.dumps(val)))
            self.get_logger().info(f'캘리브를 yaml 실측값으로 시드: {yaml_calib}')
        self.db.commit()

    def _get_setting(self, key, default=None):
        row = self.db.execute('SELECT value FROM settings WHERE key=?', (key,)).fetchone()
        if row is None:
            return default
        try:
            return json.loads(row['value'])
        except (json.JSONDecodeError, TypeError):
            return default

    def _set_setting(self, key, value):
        self.db.execute(
            'INSERT INTO settings(key, value) VALUES (?, ?) '
            'ON CONFLICT(key) DO UPDATE SET value=excluded.value',
            (key, json.dumps(value)))
        self.db.commit()

    def _put_state(self, key, value):
        self.db.execute(
            'INSERT INTO state(key, value, ts) VALUES (?, ?, ?) '
            'ON CONFLICT(key) DO UPDATE SET value=excluded.value, ts=excluded.ts',
            (key, json.dumps(value), time.time()))

    # ── config/pick_place_params.yaml 영구 저장 (기존 GUI의 yaml 저장 재현) ───
    def _find_source_params_yaml(self):
        """config/pick_place_params.yaml 의 '소스' 경로를 찾는다.

        install/share 의 yaml은 build 디렉터리로 향하는 심볼릭 링크이므로,
        IDE에서 보는 src 트리의 원본이 갱신되도록 경로에 'src'가 포함된 후보를
        우선한다.
        """
        rel = Path('config') / 'pick_place_params.yaml'
        candidates = []
        here = Path(__file__).resolve()
        for parent in here.parents:
            candidates.append(parent / rel)
        candidates.append(Path.cwd() / rel)

        existing = []
        seen = set()
        for c in candidates:
            try:
                rp = c.resolve()
            except OSError:
                continue
            if rp.is_file() and rp not in seen:
                seen.add(rp)
                existing.append(rp)
        if not existing:
            return None
        # 'src' 트리 우선.
        existing.sort(key=lambda p: 0 if 'src' in p.parts else 1)
        return existing[0]

    def _sync_yaml(self):
        """현재 DB 설정값을 yaml의 해당 라인만 교체해 저장(인라인 주석 보존).

        반환: (저장한 경로 or None, 누락된 필수 키 목록).
        """
        path = self._find_source_params_yaml()
        if path is None:
            self.get_logger().warn('yaml 동기화 실패: config 파일을 찾지 못함')
            return None, ['<yaml-not-found>']

        # yaml키 → 새 값 문자열.
        patterns = {}
        for skey, ykey, fmt in YAML_MAP:
            val = self._get_setting(skey)
            if val is None:
                continue
            try:
                patterns[ykey] = fmt(val)
            except (TypeError, ValueError):
                continue
        # known_classes 는 grip_class_names 를 미러링(라벨↔강도 일관성).
        names = self._get_setting('grip_class_names')
        if names is not None:
            patterns['known_classes'] = _fmt_str_list(names)

        try:
            with open(path, 'r', encoding='utf-8') as f:
                lines = f.readlines()
            replaced = {k: False for k in patterns}
            for i, line in enumerate(lines):
                for key, new_val in patterns.items():
                    if replaced[key]:
                        continue
                    # 값만 교체하고 인라인 주석(#...)은 보존.
                    m = re.match(rf'^(\s*){re.escape(key)}\s*:\s*[^#\n]*?(\s*#.*)?$', line)
                    if m:
                        comment = m.group(2) or ''
                        lines[i] = f'{m.group(1)}{key}: {new_val}{comment}\n'
                        replaced[key] = True
            missing = [k for k, v in replaced.items()
                       if not v and k not in OPTIONAL_YAML_KEYS]
            with open(path, 'w', encoding='utf-8') as f:
                f.writelines(lines)
            self.get_logger().info(f'💾 yaml 저장: {path}'
                                   + (f' (누락 키: {missing})' if missing else ''))
            return path, missing
        except Exception as e:
            self.get_logger().error(f'yaml 저장 실패: {e}')
            return None, [str(e)]

    # ========================================================================
    # ROS 콜백 (모두 ROS 스레드에서 실행)
    # ========================================================================
    def _cb_hw_state(self, msg: Int32):
        self.hw_state = msg.data
        self.last_hw_state_time = time.monotonic()

    def _cb_speed_mode(self, msg: Int32):
        self.speed_mode = msg.data
        self.last_speed_mode_time = time.monotonic()

    def _cb_gripper_service_state(self, msg: GripperState):
        self.gripper_hw_ready = msg.ready

    def _cb_gripper_joint_state(self, msg: JointState):
        target = None
        for name in msg.name:
            if 'gripper_joint' in name or 'rh_p12_rn' in name:
                target = name
                break
        if target is not None:
            idx = msg.name.index(target)
            self.gripper_present_position = msg.position[idx]
            self.gripper_present_current = msg.effort[idx]

    def _cb_gripper_init_progress(self, msg: String):
        self.gripper_init_progress = msg.data.strip()
        self.gripper_init_progress_t = time.monotonic()

    def _cb_state(self, msg: String):
        prev = self.pick_place_state
        self.pick_place_state = msg.data
        self.last_state_time = time.monotonic()
        if self.pick_place_state == 'DETECTING' and prev != 'DETECTING' and self._pending_calib:
            p = self._pending_calib
            self._pending_calib = None
            if self.cli_object_set_parameters.service_is_ready():
                req = SetParameters.Request()
                req.parameters = [
                    self._make_param('absolute_calib_x_mm', ParameterType.PARAMETER_DOUBLE, p['x']),
                    self._make_param('absolute_calib_y_mm', ParameterType.PARAMETER_DOUBLE, p['y']),
                    self._make_param('absolute_calib_z_mm', ParameterType.PARAMETER_DOUBLE, p['z']),
                ]
                self.cli_object_set_parameters.call_async(req)
                self.get_logger().info(
                    f'pending calib 적용: x={p["x"]} y={p["y"]} z={p["z"]}mm')

    def _cb_error(self, msg: String):
        self.last_error_text = msg.data
        text = msg.data
        level = 'ERR' if text.startswith('[ERR]') else 'WAR'
        ticket = ''
        try:
            repo = self._order_repo
            if repo is not None:
                repo.reload()
                for o in repo.list_orders({OrderStatus.RUNNING}):
                    ticket = o.ticket_no or ''
                    break
        except Exception:
            pass
        try:
            self.db.execute(
                'INSERT INTO error_log(ts, level, text, ticket_no) VALUES (?, ?, ?, ?)',
                (time.time(), level, text, ticket))
            self.db.commit()
        except Exception:
            pass

    def _cb_ultrasonic(self, msg: Range):
        if msg.range is not None and msg.range > 0.0:
            self.ultrasonic_range_m = float(msg.range)
            self.last_ultrasonic_time = time.monotonic()

    def _cb_objects(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('detected_objects JSON 파싱 실패')
            return
        objects = payload.get('objects', [])
        now = time.monotonic()
        if objects:
            self._last_nonempty_objects = objects
            self._last_nonempty_objects_time = now
            self.detected_objects = objects
        elif now - self._last_nonempty_objects_time < 1.0:
            self.detected_objects = list(self._last_nonempty_objects)
        else:
            self.detected_objects = []
        self.last_objects_time = now

    def _cb_image(self, msg: Image):
        if self.bridge is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            ok, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            if ok:
                self.latest_jpeg = buf.tobytes()
                self.last_image_time = time.monotonic()
        except Exception as e:
            self.get_logger().warn(f'이미지 인코딩 실패: {e}')

    # ========================================================================
    # 도달 가능 영역 필터 (기존 GUI와 동일)
    # ========================================================================
    def _is_reachable(self, pose: dict) -> bool:
        try:
            x = float(pose.get('x'))
            y = float(pose.get('y'))
            z = float(pose.get('z'))
        except (TypeError, ValueError):
            return False
        if not (self.workspace_x_min <= x <= self.workspace_x_max):
            return False
        if not (self.workspace_y_min <= y <= self.workspace_y_max):
            return False
        if not (self.workspace_z_min <= z <= self.workspace_z_max):
            return False
        if (x * x + y * y) ** 0.5 > self.reach_radius_max:
            return False
        return True

    # ========================================================================
    # 상태 스냅샷 기록 (200ms 타이머)
    # ========================================================================
    def _write_state_snapshot(self):
        now = time.monotonic()

        def fresh(stamp, max_age=3.0):
            return stamp > 0.0 and now - stamp <= max_age

        # 시스템 상태점(기존 GUI의 status bar 와 동일 규칙).
        grip_state = (
            'ok' if self.gripper_hw_ready
            else ('warn' if fresh(self.gripper_init_progress_t, 30.0) else 'bad'))
        try:
            _s = _socket.socket(_socket.AF_INET, _socket.SOCK_STREAM)
            _s.settimeout(0.1)
            _kiosk_up = _s.connect_ex(('127.0.0.1', 8000)) == 0
            _s.close()
        except Exception:
            _kiosk_up = False
        system_status = {
            'HW':   'ok' if fresh(self.last_hw_state_time) else 'warn',
            'GRIP': grip_state,
            'CAM':  'ok' if fresh(self.last_image_time) else 'bad',
            'DET':  'ok' if fresh(self.last_objects_time) else 'bad',
            'PICK': 'ok' if (self.cli_run_once.service_is_ready() and fresh(self.last_state_time)) else 'bad',
            'ARD':  'ok' if fresh(self.last_ultrasonic_time) else 'bad',
            'SPD':  'ok' if fresh(self.last_speed_mode_time) else 'warn',
            'USER': 'ok' if _kiosk_up else 'bad',
        }

        self._put_state('pick_place_state', self.pick_place_state)
        self._put_state('error_text', self.last_error_text)
        self._put_state('hw_state', self.hw_state)
        self._put_state('speed_mode', self.speed_mode)
        self._put_state('gripper_ready', bool(self.gripper_hw_ready))
        self._put_state('gripper_current_ma', round(float(self.gripper_present_current), 1))
        self._put_state('gripper_position_raw', round(float(self.gripper_present_position), 1))
        self._put_state('gripper_init_progress', self.gripper_init_progress)
        self._put_state('ultrasonic_mm',
                        None if self.ultrasonic_range_m is None else round(self.ultrasonic_range_m * 1000.0, 1))
        self._put_state('system_status', system_status)
        self._put_state('system_reset_phase', self._system_reset_phase)
        self._put_state('current_model_path', self.current_model_path)

        # 검출 물체 테이블 교체(매 갱신마다 비우고 다시 채움).
        self.db.execute('DELETE FROM detected_objects')
        ts = time.time()
        for obj in self.detected_objects:
            pose = obj.get('pose', {}) or {}
            self.db.execute(
                'INSERT INTO detected_objects'
                '(label, confidence, depth_m, pixel_u, pixel_v, pose_x, pose_y, pose_z, reachable, ts) '
                'VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)',
                (obj.get('label', ''), obj.get('confidence'), obj.get('depth_m'),
                 obj.get('pixel_u'), obj.get('pixel_v'),
                 pose.get('x'), pose.get('y'), pose.get('z'),
                 1 if self._is_reachable(pose) else 0, ts))
        self.db.commit()

    # ========================================================================
    # 시작 시 DB 설정을 각 노드에 적용 (1s 타이머, 한 번)
    # ========================================================================
    def _maybe_apply_startup_settings(self):
        if self._startup_settings_applied:
            self._poll_subprocesses()
            return
        # 핵심 set_parameters 서비스가 준비됐을 때 한 번에 적용.
        if not (self.cli_object_set_parameters.service_is_ready()
                and self.cli_pickplace_set_parameters.service_is_ready()):
            return
        self._startup_settings_applied = True
        self.get_logger().info('DB에 저장된 설정을 각 노드에 적용합니다...')
        # 명령 큐를 통해 일관된 경로로 적용(결과는 로그로 남음).
        for action, payload in (
            ('set_confidence',  {'value': self._get_setting('confidence_threshold')}),
            ('set_calibration', {'x': self._get_setting('calib_x_mm'),
                                 'y': self._get_setting('calib_y_mm'),
                                 'z': self._get_setting('calib_z_mm')}),
            ('set_grip_strength', {'names': self._get_setting('grip_class_names'),
                                   'currents': self._get_setting('grip_class_currents'),
                                   'default': self._get_setting('grip_current_default')}),
            ('set_min_safe_z',  {'value': self._get_setting('min_safe_z')}),
            ('set_gripper_params', {
                'open_current': self._get_setting('gripper_open_current'),
                'close_current': self._get_setting('gripper_close_current'),
                'transport_current': self._get_setting('gripper_transport_current'),
                'profile_velocity': self._get_setting('gripper_profile_velocity'),
                'profile_acceleration': self._get_setting('gripper_profile_acceleration'),
            }),
        ):
            payload['_startup'] = True   # 부팅 재적용 → yaml은 건드리지 않음
            self.db.execute(
                'INSERT INTO command_queue(ts, action, payload, status) VALUES (?, ?, ?, ?)',
                (time.time(), action, json.dumps(payload), 'pending'))
        # 빈/공백/따옴표만 있는 깨진 값('""', "''")은 적용하지 않는다 → launch가 주입한
        # 모델(models/proto_v3.pt 절대경로)을 그대로 유지. DB 플레이스홀더가 덮어쓰지 못하게.
        model = str(self._get_setting('yolo_model_path') or '').strip().strip('\'"').strip()
        if model:
            self.db.execute(
                'INSERT INTO command_queue(ts, action, payload, status) VALUES (?, ?, ?, ?)',
                (time.time(), 'set_model', json.dumps({'path': model}), 'pending'))
        self.db.commit()

    # ========================================================================
    # 명령 큐 drain (50ms 타이머)
    # ========================================================================
    def _drain_command_queue(self):
        rows = self.db.execute(
            "SELECT id, action, payload FROM command_queue WHERE status='pending' ORDER BY id ASC"
        ).fetchall()
        for row in rows:
            cmd_id = row['id']
            action = row['action']
            try:
                payload = json.loads(row['payload'])
            except (json.JSONDecodeError, TypeError):
                payload = {}
            self._mark_command(cmd_id, 'running', '')
            try:
                self._dispatch(cmd_id, action, payload)
            except Exception as e:
                self.get_logger().error(f'명령 {action} 실행 실패: {e}')
                self._mark_command(cmd_id, 'failed', str(e))
                continue
            # 설정 변경 명령은 DB뿐 아니라 yaml에도 영구 반영(기존 GUI의 yaml 저장 재현).
            # 부팅 시 DB→노드 재적용 명령(_startup)은 yaml을 건드리지 않는다.
            if action in YAML_SYNC_ACTIONS and not payload.get('_startup'):
                try:
                    self._sync_yaml()
                except Exception as e:
                    self.get_logger().warn(f'yaml 동기화 경고: {e}')

    def _mark_command(self, cmd_id, status, result):
        done_ts = time.time() if status in ('done', 'failed') else None
        self.db.execute(
            'UPDATE command_queue SET status=?, result=?, done_ts=? WHERE id=?',
            (status, str(result)[:500], done_ts, cmd_id))
        self.db.commit()

    def _dispatch(self, cmd_id, action, payload):
        """action 을 실제 ROS 동작으로 변환. 비동기 호출은 done_callback에서 마감."""
        # 1) 단순 Trigger 서비스
        if action in TRIGGER_ACTIONS:
            client = getattr(self, TRIGGER_ACTIONS[action])
            self._call_trigger(cmd_id, client, action)
            return

        # 2) 물체 선택(publish). 빈 라벨 = 자동 선택.
        if action == 'select_object':
            label = str(payload.get('label', '') or '')
            msg = String()
            msg.data = label
            self.pub_selected.publish(msg)
            self._mark_command(cmd_id, 'done', f'selected="{label or "(auto)"}"')
            return

        # 3) 그리퍼 토크 on/off (SetBool)
        if action == 'gripper_enable':
            enable = bool(payload.get('enable', True))
            client = self.cli_gripper_enable
            if not client.service_is_ready():
                self._mark_command(cmd_id, 'failed', '서비스 미연결: /gripper/enable')
                return
            req = SetBool.Request()
            req.enable = enable
            fut = client.call_async(req)
            fut.add_done_callback(lambda f: self._on_setbool_done(cmd_id, f))
            return

        # 4) 파라미터 set 계열
        if action == 'set_confidence':
            self._set_params(cmd_id, self.cli_object_set_parameters,
                             [('confidence_threshold', ParameterType.PARAMETER_DOUBLE,
                               float(payload.get('value', 0.5)))])
            self._set_setting('confidence_threshold', float(payload.get('value', 0.5)))
            return

        if action == 'set_calibration':
            x = float(payload.get('x', 0.0)); y = float(payload.get('y', 0.0)); z = float(payload.get('z', 0.0))
            self._set_setting('calib_x_mm', x); self._set_setting('calib_y_mm', y); self._set_setting('calib_z_mm', z)
            if self.pick_place_state == 'IDLE':
                self._set_params(cmd_id, self.cli_object_set_parameters, [
                    ('absolute_calib_x_mm', ParameterType.PARAMETER_DOUBLE, x),
                    ('absolute_calib_y_mm', ParameterType.PARAMETER_DOUBLE, y),
                    ('absolute_calib_z_mm', ParameterType.PARAMETER_DOUBLE, z),
                ])
            else:
                self._pending_calib = {'x': x, 'y': y, 'z': z, 'cmd_id': cmd_id}
                self._mark_command(cmd_id, 'done', f'다음 DETECTING 시 적용 예약 (현재 {self.pick_place_state})')
            return

        if action == 'set_model':
            path = str(payload.get('path', '')).strip()
            if not path:
                self._mark_command(cmd_id, 'failed', '모델 경로가 비어 있음')
                return
            self._set_params(cmd_id, self.cli_object_set_parameters,
                             [('yolo_model', ParameterType.PARAMETER_STRING, path)])
            self._set_setting('yolo_model_path', path)
            self.current_model_path = path
            return

        if action == 'set_camera_auto_exposure':
            enable = bool(payload.get('enable', True))
            self._set_params(cmd_id, self.cli_camera_set_parameters,
                             [('rgb_camera.enable_auto_exposure', ParameterType.PARAMETER_BOOL, enable)])
            self._set_setting('camera_auto_exposure', enable)
            return

        if action == 'set_camera_exposure':
            val = int(payload.get('value', 1500))
            self._set_params(cmd_id, self.cli_camera_set_parameters,
                             [('rgb_camera.exposure', ParameterType.PARAMETER_INTEGER, val)])
            self._set_setting('camera_exposure', val)
            return

        if action == 'set_gripper_params':
            spec = [
                ('open_current', 'gripper_open_current'),
                ('close_current', 'gripper_close_current'),
                ('transport_current', 'gripper_transport_current'),
                ('profile_velocity', 'gripper_profile_velocity'),
                ('profile_acceleration', 'gripper_profile_acceleration'),
            ]
            params = []
            for pname, skey in spec:
                if pname in payload and payload[pname] is not None:
                    v = int(payload[pname])
                    params.append((pname, ParameterType.PARAMETER_INTEGER, v))
                    self._set_setting(skey, v)
            if not params:
                self._mark_command(cmd_id, 'failed', '적용할 그리퍼 파라미터 없음')
                return
            self._set_params(cmd_id, self.cli_gripper_set_parameters, params)
            return

        if action == 'set_min_safe_z':
            if self.pick_place_state != 'IDLE':
                self._mark_command(cmd_id, 'failed', 'min_safe_z 적용은 IDLE 상태에서만 가능')
                return
            val = float(payload.get('value', 0.0))
            self._set_params(cmd_id, self.cli_pickplace_set_parameters,
                             [('min_safe_z', ParameterType.PARAMETER_DOUBLE, val)])
            self._set_setting('min_safe_z', val)
            return

        if action == 'set_grip_strength':
            names = list(payload.get('names', []))
            currents = [int(c) for c in payload.get('currents', [])]
            default = int(payload.get('default', 200))
            if len(names) != len(currents):
                self._mark_command(cmd_id, 'failed', 'names 와 currents 길이가 다름')
                return
            # pick_place_node 에 grip_* 적용 + object_detector.known_classes 동기화(기존 GUI 동작).
            self._set_params(cmd_id, self.cli_pickplace_set_parameters, [
                ('grip_class_names', ParameterType.PARAMETER_STRING_ARRAY, names),
                ('grip_class_currents', ParameterType.PARAMETER_INTEGER_ARRAY, currents),
                ('grip_current_default', ParameterType.PARAMETER_INTEGER, default),
            ])
            if self.cli_object_set_parameters.service_is_ready():
                req = SetParameters.Request()
                req.parameters = [self._make_param(
                    'known_classes', ParameterType.PARAMETER_STRING_ARRAY, names)]
                self.cli_object_set_parameters.call_async(req)
            self._set_setting('grip_class_names', names)
            self._set_setting('grip_class_currents', currents)
            self._set_setting('grip_current_default', default)
            return

        # 5) 캘리브레이션/모델 경로 읽어오기(object_detector get_parameters)
        if action == 'load_calibration':
            client = self.cli_object_get_parameters
            if not client.service_is_ready():
                self._mark_command(cmd_id, 'failed', '서비스 미연결: object_detector/get_parameters')
                return
            req = GetParameters.Request()
            req.names = ['absolute_calib_x_mm', 'absolute_calib_y_mm', 'absolute_calib_z_mm', 'yolo_model']
            fut = client.call_async(req)
            fut.add_done_callback(lambda f: self._on_calib_loaded(cmd_id, f))
            return

        # 6) subprocess 동작(시스템 리셋 / 그리퍼 브릿지 재시작)
        if action == 'restart_gripper_bridge':
            self._run_gripper_bridge_restart(cmd_id)
            return

        if action == 'system_reset':
            self._run_system_reset(cmd_id, payload)
            return

        # 7) 현재 DB 설정 전체를 yaml에 영구 저장(수동 트리거).
        if action == 'save_yaml':
            path, missing = self._sync_yaml()
            if path is None:
                self._mark_command(cmd_id, 'failed', f'yaml 저장 실패: {missing}')
            elif missing:
                self._mark_command(cmd_id, 'done', f'저장: {path.name} (누락 키: {missing})')
            else:
                self._mark_command(cmd_id, 'done', f'저장 완료: {path}')
            return

        self._mark_command(cmd_id, 'failed', f'알 수 없는 action: {action}')

    # ── ROS 호출 헬퍼 ───────────────────────────────────────────────────────
    @staticmethod
    def _make_param(name, ptype, value):
        rp = RclParameter()
        rp.name = name
        pv = ParameterValue()
        pv.type = ptype
        if ptype == ParameterType.PARAMETER_BOOL:
            pv.bool_value = bool(value)
        elif ptype == ParameterType.PARAMETER_INTEGER:
            pv.integer_value = int(value)
        elif ptype == ParameterType.PARAMETER_DOUBLE:
            pv.double_value = float(value)
        elif ptype == ParameterType.PARAMETER_STRING:
            pv.string_value = str(value)
        elif ptype == ParameterType.PARAMETER_STRING_ARRAY:
            pv.string_array_value = [str(v) for v in value]
        elif ptype == ParameterType.PARAMETER_INTEGER_ARRAY:
            pv.integer_array_value = [int(v) for v in value]
        rp.value = pv
        return rp

    def _call_trigger(self, cmd_id, client, label):
        if not client.service_is_ready():
            self._mark_command(cmd_id, 'failed', f'서비스 미연결: {label}')
            return
        fut = client.call_async(Trigger.Request())

        def _done(f):
            try:
                res = f.result()
            except Exception as e:
                self._mark_command(cmd_id, 'failed', f'{label} 호출 실패: {e}')
                return
            status = 'done' if res.success else 'failed'
            self._mark_command(cmd_id, status, res.message)
        fut.add_done_callback(_done)

    def _on_setbool_done(self, cmd_id, fut):
        try:
            res = fut.result()
        except Exception as e:
            self._mark_command(cmd_id, 'failed', str(e))
            return
        self._mark_command(cmd_id, 'done' if res.success else 'failed', res.message)

    def _set_params(self, cmd_id, client, params):
        """params: [(name, ParameterType, value), ...] 를 set_parameters 로 호출."""
        if not client.service_is_ready():
            self._mark_command(cmd_id, 'failed', 'set_parameters 서비스 미연결')
            return
        req = SetParameters.Request()
        req.parameters = [self._make_param(n, t, v) for (n, t, v) in params]
        fut = client.call_async(req)

        def _done(f):
            try:
                results = f.result().results
                ok = bool(results) and all(r.successful for r in results)
            except Exception as e:
                self._mark_command(cmd_id, 'failed', str(e))
                return
            if ok:
                self._mark_command(cmd_id, 'done', '적용 완료')
            else:
                reason = next((r.reason for r in results if not r.successful), '거절')
                self._mark_command(cmd_id, 'failed', f'거절: {reason}')
        fut.add_done_callback(_done)

    def _on_calib_loaded(self, cmd_id, fut):
        try:
            res = fut.result()
        except Exception as e:
            self._mark_command(cmd_id, 'failed', str(e))
            return
        vals = [v.double_value for v in res.values[:3]]
        if len(vals) >= 3:
            self._set_setting('calib_x_mm', vals[0])
            self._set_setting('calib_y_mm', vals[1])
            self._set_setting('calib_z_mm', vals[2])
        if len(res.values) >= 4:
            self._set_setting('yolo_model_path', res.values[3].string_value)
        self._mark_command(cmd_id, 'done', f'calib={vals}')

    # ── subprocess 동작(기존 GUI의 시스템 리셋 / 그리퍼 브릿지 재시작) ────────
    def _run_gripper_bridge_restart(self, cmd_id):
        from ament_index_python.packages import get_package_share_directory
        if self._gripper_bridge_proc is not None and self._gripper_bridge_proc.poll() is None:
            self._mark_command(cmd_id, 'failed', '이미 진행 중')
            return
        try:
            pkg_share = get_package_share_directory('dsr_realsense_pick_place')
            script = os.path.join(pkg_share, 'scripts', 'restart_gripper_bridge.sh')
            self._gripper_bridge_proc = subprocess.Popen(
                ['bash', script], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.get_logger().warn('🔧 그리퍼 브릿지 재기동 스크립트 실행 (status3 복구)')
            self._mark_command(cmd_id, 'done', '재기동 시작(~5-40초, 그리퍼 ready 확인)')
        except Exception as e:
            self._mark_command(cmd_id, 'failed', str(e))

    def _run_system_reset(self, cmd_id, payload):
        """GUI를 제외한 모든 노드를 종료 후 재시작(기존 GUI _system_reset 과 동일 흐름)."""
        from ament_index_python.packages import get_package_share_directory
        if self._system_reset_phase:
            self._mark_command(cmd_id, 'failed', '이미 진행 중')
            return
        try:
            pkg_share = get_package_share_directory('dsr_realsense_pick_place')
            shutdown_script = os.path.join(pkg_share, 'scripts', 'shutdown_nodes.sh')
            self._system_reset_proc = subprocess.Popen(
                ['bash', shutdown_script], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self._system_reset_phase = 'shutting_down'
            # 재시작 시 전달할 launch 인자(payload로 덮어쓸 수 있음).
            self._system_reset_args = {
                'mode': payload.get('mode', 'real'),
                'host': payload.get('host', '110.120.1.50'),
            }
            self.get_logger().warn('🔄 시스템 리셋: 노드 종료 중...')
            self._mark_command(cmd_id, 'done', '시스템 리셋 시작(노드 종료 → 재시작)')
        except Exception as e:
            self._mark_command(cmd_id, 'failed', str(e))

    def _poll_subprocesses(self):
        """시스템 리셋 단계 폴링(기존 GUI _poll_system_reset 과 동일)."""
        if self._system_reset_phase == 'shutting_down':
            if self._system_reset_proc and self._system_reset_proc.poll() is not None:
                self._system_reset_phase = 'restarting'
                args = getattr(self, '_system_reset_args', {'mode': 'real', 'host': '110.120.1.50'})
                try:
                    # web:=true 로 재시작 — 이 노드는 별도 프로세스로 살아있으므로 웹은 유지.
                    self._system_restart_proc = subprocess.Popen(
                        ['ros2', 'launch', 'dsr_realsense_pick_place', 'pick_place.launch.py',
                         f"mode:={args['mode']}", f"host:={args['host']}",
                         'gui:=false', 'web:=false'],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    self.get_logger().warn('🚀 시스템 리셋: 노드 재시작 중...')
                except Exception as e:
                    self.get_logger().error(f'재시작 실패: {e}')
                self._system_reset_phase = ''

    # ========================================================================
    # HTTP 서버
    # ========================================================================
    def _start_http_server(self):
        self.http_host = str(self.get_parameter('http_host').value)
        self.http_port = int(self.get_parameter('http_port').value)
        node = self

        class Handler(BaseHTTPRequestHandler):
            # HTTP 요청마다 별도 sqlite 연결을 연다(스레드 안전).
            def _db(self):
                conn = sqlite3.connect(str(node.db_path), check_same_thread=False, timeout=5.0)
                conn.row_factory = sqlite3.Row
                conn.execute('PRAGMA busy_timeout=5000;')
                return conn

            def log_message(self, fmt, *args):
                pass  # 콘솔 스팸 방지

            def _send_json(self, obj, code=200):
                body = json.dumps(obj, ensure_ascii=False).encode('utf-8')
                self.send_response(code)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(body)

            def _send_bytes(self, data, ctype):
                self.send_response(200)
                self.send_header('Content-Type', ctype)
                self.send_header('Content-Length', str(len(data)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(data)

            # ---- GET ----
            def do_GET(self):
                path = self.path.split('?', 1)[0]
                if path == '/' or path == '/index.html':
                    self._send_bytes(INDEX_HTML.encode('utf-8'), 'text/html; charset=utf-8')
                    return
                if path == '/api/image.jpg':
                    data = node.latest_jpeg
                    if data is None:
                        self._send_json({'error': 'no image'}, 404)
                    else:
                        self._send_bytes(data, 'image/jpeg')
                    return
                if path == '/api/state':
                    self._send_json(self._read_state())
                    return
                if path == '/api/settings':
                    self._send_json(self._read_settings())
                    return
                if path == '/api/commands':
                    self._send_json(self._read_commands())
                    return
                if path == '/api/orders':
                    qs = self.path.split('?', 1)[1] if '?' in self.path else ''
                    show_all = 'all=1' in qs
                    self._send_json(self._read_orders(show_all=show_all))
                    return
                if path == '/api/lockers':
                    self._send_json(self._read_lockers())
                    return
                if path == '/api/log':
                    self._send_json(self._read_log())
                    return
                if path == '/api/models':
                    self._send_json(self._scan_models())
                    return
                if path.startswith('/api/command/'):
                    try:
                        cid = int(path.rsplit('/', 1)[1])
                    except ValueError:
                        self._send_json({'error': 'bad id'}, 400)
                        return
                    self._send_json(self._read_command(cid))
                    return
                self._send_json({'error': 'not found'}, 404)

            # ---- POST ----
            def do_POST(self):
                path = self.path.split('?', 1)[0]
                if path not in ('/api/command', '/api/orders/cancel', '/api/orders/pause',
                                '/api/orders/cancel_item', '/api/lockers/release',
                                '/api/lockers/reset', '/api/lockers/pickup'):
                    self._send_json({'error': 'not found'}, 404)
                    return
                length = int(self.headers.get('Content-Length', 0))
                raw = self.rfile.read(length) if length else b'{}'
                try:
                    body = json.loads(raw or b'{}')
                except json.JSONDecodeError:
                    self._send_json({'error': 'bad json'}, 400)
                    return
                # 유저 주문 큐 조작(task_repository 직접 호출 — command_queue를 거치지 않음).
                if path == '/api/orders/cancel':
                    if node._order_repo is None:
                        self._send_json({'ok': False, 'error': 'DB 없음'}, 503)
                        return
                    order_id = str(body.get('order_id', '') or '')
                    if not order_id:
                        self._send_json({'ok': False, 'error': 'order_id 누락'}, 400)
                        return
                    try:
                        node._order_repo.reload()
                        ok = node._order_repo.cancel_order(order_id, protect_running=True)
                        self._send_json({'ok': bool(ok)})
                    except Exception as e:
                        self._send_json({'ok': False, 'error': str(e)}, 500)
                    return
                if path == '/api/orders/pause':
                    if node._order_repo is None:
                        self._send_json({'ok': False, 'error': 'DB 없음'}, 503)
                        return
                    try:
                        node._order_repo.reload()
                        if 'paused' in body:
                            paused = bool(body.get('paused'))
                        else:
                            paused = not node._order_repo.is_queue_paused()
                        node._order_repo.set_queue_paused(paused)
                        self._send_json({'ok': True, 'paused': paused})
                    except Exception as e:
                        self._send_json({'ok': False, 'error': str(e)}, 500)
                    return
                # 락커 관리(강제해제 / 전체리셋 / 대리수령).
                if path in ('/api/lockers/release', '/api/lockers/reset',
                            '/api/lockers/pickup'):
                    if node._order_repo is None:
                        self._send_json({'ok': False, 'error': 'DB 없음'}, 503)
                        return
                    try:
                        node._order_repo.reload()
                        if path == '/api/lockers/release':
                            try:
                                lid = int(body.get('id', 0))
                            except (TypeError, ValueError):
                                self._send_json({'ok': False, 'error': 'id는 정수여야 함'}, 400)
                                return
                            self._send_json({'ok': bool(node._order_repo.release_locker(lid))})
                        elif path == '/api/lockers/reset':
                            node._order_repo.reset_lockers()
                            self._send_json({'ok': True})
                        else:
                            res = node._order_repo.confirm_pickup(
                                str(body.get('token', '') or ''))
                            self._send_json(
                                {'ok': bool(res and res.get('ok')), 'detail': res})
                    except Exception as e:
                        self._send_json({'ok': False, 'error': str(e)}, 500)
                    return
                if path == '/api/orders/cancel_item':
                    if node._order_repo is None:
                        self._send_json({'ok': False, 'error': 'DB 없음'}, 503)
                        return
                    item_id = str(body.get('item_id', '') or '')
                    if not item_id:
                        self._send_json({'ok': False, 'error': 'item_id 누락'}, 400)
                        return
                    try:
                        node._order_repo.reload()
                        node._order_repo.cancel_item(item_id)
                        self._send_json({'ok': True})
                    except Exception as e:
                        self._send_json({'ok': False, 'error': str(e)}, 500)
                    return
                action = str(body.get('action', '')).strip()
                if not action:
                    self._send_json({'error': 'action 누락'}, 400)
                    return
                payload = body.get('payload', {}) or {}
                conn = self._db()
                cur = conn.execute(
                    'INSERT INTO command_queue(ts, action, payload, status) VALUES (?, ?, ?, ?)',
                    (time.time(), action, json.dumps(payload), 'pending'))
                conn.commit()
                cid = cur.lastrowid
                conn.close()
                self._send_json({'id': cid, 'action': action, 'status': 'pending'})

            # ---- DB 읽기 헬퍼 ----
            def _read_state(self):
                conn = self._db()
                state = {}
                for row in conn.execute('SELECT key, value FROM state'):
                    try:
                        state[row['key']] = json.loads(row['value'])
                    except (json.JSONDecodeError, TypeError):
                        state[row['key']] = row['value']
                objs = []
                for row in conn.execute('SELECT * FROM detected_objects ORDER BY id'):
                    objs.append({
                        'label': row['label'], 'confidence': row['confidence'],
                        'depth_m': row['depth_m'], 'pixel_u': row['pixel_u'], 'pixel_v': row['pixel_v'],
                        'pose': {'x': row['pose_x'], 'y': row['pose_y'], 'z': row['pose_z']},
                        'reachable': bool(row['reachable']),
                    })
                conn.close()
                state['detected_objects'] = objs
                state['has_image'] = node.latest_jpeg is not None
                return state

            def _read_settings(self):
                conn = self._db()
                out = {}
                for row in conn.execute('SELECT key, value FROM settings'):
                    try:
                        out[row['key']] = json.loads(row['value'])
                    except (json.JSONDecodeError, TypeError):
                        out[row['key']] = row['value']
                conn.close()
                return out

            def _read_commands(self):
                conn = self._db()
                rows = conn.execute(
                    'SELECT id, ts, action, status, result, done_ts FROM command_queue '
                    'ORDER BY id DESC LIMIT 30').fetchall()
                conn.close()
                return [dict(r) for r in rows]

            def _read_orders(self, show_all=False):
                repo = node._order_repo
                if repo is None:
                    return {'orders': [], 'paused': False, 'error': 'DB 없음'}
                try:
                    repo.reload()
                    if show_all:
                        statuses = {OrderStatus.QUEUED, OrderStatus.RUNNING, OrderStatus.PAUSED,
                                    OrderStatus.DONE, OrderStatus.FAILED, OrderStatus.CANCELED}
                    else:
                        # DONE(배달완료=수령대기) 포함 — 락커별 수령 상태 표시용.
                        statuses = {OrderStatus.QUEUED, OrderStatus.RUNNING,
                                    OrderStatus.PAUSED, OrderStatus.DONE}
                    orders = []
                    for o in repo.list_orders(statuses):
                        items = []
                        for iid in o.item_ids:
                            it = repo.get_item(iid)
                            if it is None:
                                continue
                            items.append({
                                'class_name': it.class_name,
                                'status': it.status,
                                'item_id': iid,
                            })
                        orders.append({
                            'order_id': o.order_id,
                            'ticket_no': o.ticket_no,
                            'status': o.status,
                            'items': items,
                            'created_at': o.created_at,
                            'locker_id': o.locker_id,
                            'qr_token': o.qr_token,
                        })
                    return {'orders': orders, 'paused': bool(repo.is_queue_paused())}
                except Exception as e:
                    return {'orders': [], 'paused': False, 'error': str(e)}

            def _read_lockers(self):
                repo = node._order_repo
                if repo is None:
                    return {'lockers': [], 'error': 'DB 없음'}
                try:
                    repo.reload()
                    return {'lockers': repo.list_lockers()}
                except Exception as e:
                    return {'lockers': [], 'error': str(e)}

            def _read_command(self, cid):
                conn = self._db()
                row = conn.execute(
                    'SELECT id, ts, action, payload, status, result, done_ts '
                    'FROM command_queue WHERE id=?', (cid,)).fetchone()
                conn.close()
                if row is None:
                    return {'error': 'not found'}
                return dict(row)

            def _read_log(self):
                conn = self._db()
                try:
                    cmd_rows = conn.execute(
                        'SELECT ts, action as text, status, result, done_ts, id '
                        'FROM command_queue ORDER BY ts DESC LIMIT 200').fetchall()
                    err_rows = conn.execute(
                        'SELECT ts, text, level, ticket_no, id '
                        'FROM error_log ORDER BY ts DESC LIMIT 200').fetchall()
                finally:
                    conn.close()
                import datetime
                def fmt(ts):
                    if ts is None: return ''
                    try: return datetime.datetime.fromtimestamp(ts).strftime('%H:%M:%S')
                    except: return str(ts)
                entries = []
                for r in cmd_rows:
                    entries.append({
                        'ts': r['ts'], 'ts_str': fmt(r['ts']),
                        'type': 'cmd', 'source': '관리자',
                        'text': r['text'], 'status': r['status'],
                        'detail': r['result'] or '', 'done_ts_str': fmt(r['done_ts']),
                    })
                for r in err_rows:
                    ticket = r['ticket_no'] or ''
                    src = f'#{ticket} (유저 큐)' if ticket else '시스템'
                    entries.append({
                        'ts': r['ts'], 'ts_str': fmt(r['ts']),
                        'type': r['level'].lower(),  # 'err' or 'war'
                        'source': src,
                        'text': r['text'], 'status': r['level'],
                        'detail': '', 'done_ts_str': '',
                    })
                entries.sort(key=lambda x: x['ts'] or 0, reverse=True)
                return entries[:300]

            def _scan_models(self):
                import glob, os
                search_dirs = [
                    os.path.expanduser('~/Downloads'),
                    os.path.expanduser('~/'),
                    os.path.expanduser('~/models'),
                ]
                try:
                    from ament_index_python.packages import get_package_share_directory
                    search_dirs.append(
                        os.path.join(get_package_share_directory('dsr_realsense_pick_place'), 'models'))
                except Exception:
                    pass
                found = []
                seen = set()
                for d in search_dirs:
                    for p in glob.glob(os.path.join(d, '*.pt')):
                        ap = os.path.abspath(p)
                        if ap not in seen:
                            seen.add(ap)
                            found.append(ap)
                found.sort()
                current = node.current_model_path
                if current and os.path.isfile(current) and current not in seen:
                    found.insert(0, current)
                return {'models': found, 'current': current}

        self._httpd = ThreadingHTTPServer((self.http_host, self.http_port), Handler)
        self._http_thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        self._http_thread.start()

    def shutdown(self):
        try:
            if getattr(self, '_httpd', None) is not None:
                self._httpd.shutdown()
        except Exception:
            pass
        try:
            self.db.close()
        except Exception:
            pass


# ─────────────────────────────────────────────────────────────────────────────
# 브라우저 제어 페이지("창"). 외부 의존성 없이 단일 HTML 로 모든 동작을 제공.
# ─────────────────────────────────────────────────────────────────────────────
INDEX_HTML = r"""<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>DSR Pick &amp; Place 웹 제어</title>
<style>
  :root{color-scheme:dark;}
  *{box-sizing:border-box;}
  body{font-family:system-ui,sans-serif;margin:0;background:#1a1a1a;color:#e0e0e0;}
  header{background:#0d1520;padding:8px 14px;display:flex;gap:8px;align-items:center;flex-wrap:wrap;border-bottom:1px solid #2a3a50;}
  .dot{padding:2px 7px;border-radius:4px;font-size:11px;font-weight:bold;background:#555;}
  .ok{background:#1a7a1a;} .warn{background:#8a6000;} .bad{background:#a00000;}
  .dot-sep{border-left:1px solid #444;margin:0 4px;height:16px;display:inline-block;vertical-align:middle;}
  nav{background:#111820;padding:0 14px;display:flex;gap:2px;border-bottom:1px solid #2a3a50;}
  .tab{background:none;border:none;color:#8ab;padding:10px 18px;cursor:pointer;font-size:13px;border-bottom:3px solid transparent;border-radius:0;margin:0;}
  .tab:hover{color:#def;}
  .tab.active{color:#9fd0ff;border-bottom-color:#9fd0ff;}
  .page{display:none;}
  .page.active{display:block;}
  .cols{display:flex;gap:12px;padding:12px;flex-wrap:wrap;align-items:flex-start;}
  .col{flex:1;min-width:300px;}
  .col-wide{flex:1.6;min-width:360px;}
  .card{background:#252525;border-radius:10px;padding:11px 13px;margin-bottom:11px;border:1px solid #333;}
  .card h3{margin:0 0 8px;font-size:13px;color:#9fd0ff;font-weight:600;}
  button{background:#374151;color:#fff;border:none;border-radius:6px;padding:7px 10px;font-size:12px;cursor:pointer;margin:2px;}
  button:hover{background:#4b5a6e;}
  button.danger{background:#8b0000;} button.danger:hover{background:#c00000;}
  button.warn{background:#7a3200;} button.warn:hover{background:#b04800;}
  button.go{background:#1a5e1a;} button.go:hover{background:#1e8a1e;}
  button.active-sel{background:#1a7a1a;outline:2px solid #6fffaf;outline-offset:1px;}
  input,select{background:#1a1a1a;color:#fff;border:1px solid #555;border-radius:4px;padding:4px 6px;font-size:12px;}
  #activity{font-weight:bold;color:#cfe8ff;font-size:12px;}
  #banner{color:#ff8888;font-weight:bold;padding:6px 14px;display:none;background:#2a0808;border-bottom:1px solid #a00;}
  .row{display:flex;gap:6px;align-items:center;margin:4px 0;flex-wrap:wrap;}
  label{font-size:12px;color:#bbb;}
  small{color:#888;font-size:11px;}
  /* camera — D455 16:9 aspect ratio */
  #cam-wrap{position:relative;background:#111;border-radius:8px;overflow:hidden;aspect-ratio:16/9;width:100%;}
  #cam{position:absolute;inset:0;width:100%;height:100%;display:block;border-radius:8px;object-fit:contain;}
  #cam-no{position:absolute;inset:0;display:none;align-items:center;justify-content:center;color:#ff3333;font-size:22px;font-weight:bold;letter-spacing:3px;background:#0a0a0a;}
  /* detect objects */
  .obj-info{background:#2a2a2a;border-radius:5px;padding:5px 7px;margin:3px 0;font-size:12px;border-left:3px solid #555;}
  .obj-info.reach{border-left-color:#2a6a2a;}
  .obj-info.unreach{opacity:.5;}
  .obj-row-wrap{display:flex;align-items:center;gap:4px;margin:6px 0 2px;height:34px;}
  .obj-row-label{font-size:10px;color:#666;min-width:44px;text-align:right;flex-shrink:0;}
  .obj-row-btns{display:flex;gap:4px;overflow-x:auto;overflow-y:hidden;flex:1;height:34px;align-items:center;}
  .obj-row-btns::-webkit-scrollbar{height:3px;}.obj-row-btns::-webkit-scrollbar-thumb{background:#444;}
  /* orders */
  .order{border-radius:7px;padding:7px 9px;margin:5px 0;border-left:5px solid #555;background:#2a2a2a;}
  .order.running,.order.run{border-left-color:#1a8a1a;background:#1a2e1a;}
  .order.pause{border-left-color:#c89000;background:#2a2610;}
  .order.queued{border-left-color:#666;background:#2a2a2a;}
  .order.done{border-left-color:#2255aa;background:#1a1e2a;opacity:.75;}
  .order.failed{border-left-color:#993300;background:#2a1a10;opacity:.85;}
  .order.canceled{border-left-color:#553300;opacity:.6;}
  .order .tk{font-weight:bold;font-size:13px;}
  .badge{font-size:10px;padding:1px 6px;border-radius:4px;background:#555;display:inline-block;margin:1px;}
  .badge.run,.badge.running{background:#1a7a1a;}
  .badge.pause{background:#c89000;color:#111;}
  .badge.queued{background:#555;}
  .locker-grid{display:grid;grid-template-columns:repeat(4,1fr);gap:6px;}
  .lk{border-radius:6px;padding:6px;font-size:11px;background:#2a2a2a;border-left:5px solid #555;}
  .lk.free{border-left-color:#555;opacity:.55;}
  .lk.occupied{border-left-color:#c89000;background:#2a2610;}
  .lk.ready{border-left-color:#1a8a1a;background:#1a2e1a;}
  .lk .lkid{font-weight:bold;font-size:13px;}
  .lk .lkcode{font-family:monospace;font-size:14px;font-weight:bold;letter-spacing:2px;color:#ffd24d;margin-top:2px;}
  .lk button{font-size:10px;padding:2px 5px;margin:3px 2px 0 0;}
  .badge.done{background:#2255aa;}
  .badge.failed{background:#993300;}
  .badge.canceled{background:#553300;}
  .item-row{display:flex;align-items:center;gap:5px;margin:2px 0;}
  .item-cancel{background:#6a2020;padding:1px 6px;font-size:11px;}
  /* robot action grid */
  .act-grid{display:grid;grid-template-columns:1fr 1fr 1fr;gap:4px;margin-bottom:4px;}
  .act-grid button{margin:0;width:100%;}
  .act-grid span{display:block;}
  /* gripper chart */
  #gchart{width:100%;height:90px;border-radius:6px;background:#111;display:block;}
  /* ultrasonic bar */
  .us-wrap{background:#1a1a1a;border-radius:5px;height:24px;position:relative;overflow:hidden;border:1px solid #444;margin:6px 0;}
  .us-fill{height:100%;transition:width .3s,background .5s;border-radius:4px;}
  .us-mark{position:absolute;top:0;bottom:0;width:2px;background:#fff8;pointer-events:none;}
  .us-lbl{position:absolute;right:6px;top:50%;transform:translateY(-50%);font-size:11px;font-weight:bold;color:#fff;}
  /* grip table */
  .grip-tbl{width:100%;font-size:12px;border-collapse:collapse;margin-top:5px;}
  .grip-tbl td,.grip-tbl th{padding:3px 5px;border-bottom:1px solid #333;text-align:left;}
  .grip-tbl th{color:#9fd0ff;font-weight:600;}
  .grip-tbl input{width:65px;}
  /* log */
  .log-filter{display:flex;gap:4px;margin-bottom:8px;}
  .log-filter button{font-size:11px;padding:4px 10px;}
  .log-filter button.active-sel{background:#1a5ea0;}
  .log-entry{border-radius:6px;padding:6px 9px;margin:4px 0;border-left:4px solid #555;background:#252525;}
  .log-entry.cmd{border-left-color:#4466aa;}
  .log-entry.err{border-left-color:#cc2222;background:#280a0a;}
  .log-entry.war{border-left-color:#cc7700;background:#251800;}
  .log-entry.user-src{background:#1c1c24!important;}
  .log-row1{display:flex;gap:10px;font-size:11px;color:#888;margin-bottom:2px;}
  .log-row2{font-size:12px;}
  .log-badge{padding:1px 6px;border-radius:3px;font-size:10px;font-weight:bold;}
  .log-badge.cmd{background:#1a3a6a;} .log-badge.err{background:#8a0000;} .log-badge.war{background:#7a4000;}
  .log-badge.done{background:#1a5a1a;} .log-badge.failed{background:#6a2200;} .log-badge.pending{background:#555;} .log-badge.running{background:#1a4a1a;}
  #loglist{max-height:calc(100vh - 160px);overflow-y:auto;}
  /* user tab */
  #user-stats{display:flex;gap:6px;margin-bottom:10px;flex-wrap:wrap;}
  .stat-box{background:#1e2a3a;border-radius:7px;padding:6px 14px;text-align:center;min-width:72px;}
  .stat-box .sval{font-size:22px;font-weight:bold;color:#9fd0ff;}
  .stat-box .slbl{font-size:10px;color:#888;}
</style>
</head>
<body>
<header>
  <strong style="font-size:14px;color:#9fd0ff;">DSR Pick &amp; Place</strong>
  <span id="status-dots"></span>
  <span id="activity" style="margin-left:auto;">연결 중...</span>
</header>
<div id="banner"></div>
<nav>
  <button class="tab active" id="tab-main" onclick="showTab('main')">🤖 메인</button>
  <button class="tab" id="tab-user" onclick="showTab('user')">👤 유저 큐</button>
  <button class="tab" id="tab-gripper" onclick="showTab('gripper')">🦾 그리퍼</button>
  <button class="tab" id="tab-log" onclick="showTab('log')">📋 로그</button>
</nav>

<!-- ═══════════════ 메인 탭 ═══════════════ -->
<div class="page active" id="page-main">
<div class="cols">
  <!-- Col 1: Queue + Camera -->
  <div class="col col-wide">
    <div class="card">
      <h3>작업 중인 유저 주문</h3>
      <div id="orders"><small>불러오는 중...</small></div>
    </div>
    <div class="card">
      <h3>락커 현황 <button onclick="resetLockers()" style="float:right;font-size:11px;padding:2px 8px">전체 리셋</button></h3>
      <div id="lockers" class="locker-grid"><small>불러오는 중...</small></div>
    </div>
    <div class="card">
      <h3>카메라 / 검출 &nbsp;<small id="sel-label" style="color:#9fd0ff;">선택: 자동</small></h3>
      <div id="cam-wrap">
        <img id="cam" alt="카메라 영상" src="">
        <div id="cam-no">NO CAMERA</div>
      </div>
      <div class="obj-row-wrap">
        <span class="obj-row-label">Known</span>
        <div id="known-row" class="obj-row-btns"></div>
      </div>
      <div class="obj-row-wrap">
        <span class="obj-row-label">Unknown</span>
        <div id="unknown-row" class="obj-row-btns"></div>
      </div>
    </div>
  </div>

  <!-- Col 2: Controls + Tuning -->
  <div class="col">
    <div class="card">
      <h3>긴급 제어</h3>
      <button id="estop-btn" class="danger" style="width:100%;padding:14px;font-size:15px;font-weight:bold;margin-bottom:6px;" onclick="toggleEStop()">⛔ 긴급정지</button>
      <button class="warn" onclick="cmd('cancel')">🚫 태스크 중단</button>
      <button onclick="cmd('clear_error')">🔄 에러 해제</button>
      <button onclick="cmd('go_home')">🏠 홈 복귀</button>
    </div>
    <div class="card">
      <h3>로봇 동작</h3>
      <div class="act-grid">
        <button onclick="cmd('run_once')">▶ 한번실행</button>
        <button class="go" onclick="cmd('sort_all')">🗂 자동 분류</button>
        <button onclick="cmd('run_once_package')">📦 패키지 픽</button>
        <button onclick="cmd('speed_normal')">🟢 정상속도</button>
        <button onclick="cmd('speed_reduced')">🟡 저속</button>
        <span></span>
        <button id="servo-btn" onclick="toggleServo()">서보 ON/OFF</button>
        <button onclick="cmd('safety_normal')">정상운전</button>
        <button onclick="cmd('safety_backdrive')">역구동</button>
      </div>
    </div>
    <div class="card">
      <h3>검출 / 카메라 튜닝</h3>
      <div class="row"><label>신뢰도</label>
        <input id="conf" type="number" step="0.01" min="0.05" max="0.95" style="width:65px">
        <button onclick="cmd('set_confidence',{value:+v('conf')})">적용</button></div>
      <div class="row"><label>자동노출</label>
        <button onclick="cmd('set_camera_auto_exposure',{enable:true})">ON</button>
        <button onclick="cmd('set_camera_auto_exposure',{enable:false})">OFF</button>
        <input id="exp" type="number" min="20" max="5000" style="width:70px">
        <button onclick="cmd('set_camera_exposure',{value:+v('exp')})">적용</button></div>
    </div>
    <div class="card">
      <h3>캘리브레이션 (mm, IDLE 전용)</h3>
      <div class="row">
        X<input id="cx" type="number" step="1" style="width:65px">
        Y<input id="cy" type="number" step="1" style="width:65px">
        Z<input id="cz" type="number" step="1" style="width:65px">
      </div>
      <button onclick="cmd('load_calibration')">불러오기</button>
      <button onclick="cmd('set_calibration',{x:+v('cx'),y:+v('cy'),z:+v('cz')})">적용</button>
    </div>
  </div>

  <!-- Col 3: Model + 전류 그래프 + 초음파 -->
  <div class="col">
    <div class="card">
      <h3>모델 / 시스템</h3>
      <div style="margin-bottom:6px;"><small>현재 모델:</small><br>
        <code id="model-path" style="font-size:11px;color:#adf;word-break:break-all;">-</code>
      </div>
      <div class="row">
        <select id="model-sel" style="flex:1;min-width:0;" onchange="if(this.value)$('model').value=this.value">
          <option value="">-- 스캔된 파일 --</option>
        </select>
        <button onclick="loadModels()">🔍</button>
      </div>
      <div class="row">
        <input id="model" type="text" placeholder=".pt 경로 직접 입력" style="flex:1;min-width:0;">
        <button onclick="cmd('set_model',{path:v('model')})">적용</button>
      </div>
      <div style="margin-top:6px;">
        <button class="go" onclick="cmd('save_yaml')">💾 yaml 저장</button>
        <button class="warn" onclick="if(confirm('노드 재시작?'))cmd('system_reset')">🔄 시스템 리셋</button>
      </div>
    </div>
    <div class="card">
      <h3>그리퍼 전류 &nbsp;<small><span id="g_curr2">-</span> mA &nbsp;위치: <span id="g_pos2">-</span></small></h3>
      <canvas id="gchart"></canvas>
    </div>
    <div class="card">
      <h3>초음파 거리 &nbsp;<small>grasp 기준: <span id="us-thresh">40</span> mm</small></h3>
      <div class="us-wrap">
        <div class="us-fill" id="us-fill" style="width:0%;background:#555;"></div>
        <div class="us-mark" id="us-mark" style="left:20%;"></div>
        <span class="us-lbl" id="us-lbl">- mm</span>
      </div>
      <small>0mm ←──────────────────────── 400mm</small>
    </div>
  </div>
</div>
</div>

<!-- ═══════════════ 유저 큐 탭 ═══════════════ -->
<div class="page" id="page-user">
<div style="padding:12px;">
  <div style="display:flex;align-items:center;gap:8px;margin-bottom:10px;flex-wrap:wrap;">
    <strong style="font-size:14px;">유저 주문 관리</strong>
    <span id="user-q-badge"></span>
    <button id="user-pause-btn" onclick="toggleQueue();setTimeout(pollUser,400)">⏸ 보류/재개</button>
    <label style="margin-left:8px;display:flex;align-items:center;gap:4px;">
      <input type="checkbox" id="show-history" onchange="pollUser()"> 완료/취소 이력 포함
    </label>
    <button onclick="pollUser()" style="margin-left:auto;">🔄 새로고침</button>
  </div>
  <div id="user-stats"></div>
  <div id="user-orders"><small>로딩 중...</small></div>
</div>
</div>

<!-- ═══════════════ 그리퍼 탭 ═══════════════ -->
<div class="page" id="page-gripper">
<div class="cols">
  <div class="col">
    <div class="card">
      <h3>그리퍼 제어</h3>
      <button onclick="cmd('gripper_open')">OPEN</button>
      <button onclick="cmd('gripper_close')">CLOSE</button>
      <button onclick="cmd('gripper_enable',{enable:true})">토크 ON</button>
      <button class="warn" onclick="cmd('gripper_enable',{enable:false})">토크 OFF</button>
      <button onclick="cmd('gripper_reinit')">🔁 재초기화</button>
      <button class="warn" onclick="cmd('restart_gripper_bridge')">🔧 브릿지 재시작</button>
    </div>
  </div>
  <div class="col">
    <div class="card">
      <h3>기본 파라미터 (mA / step) <small style="float:right;color:#5a9;font-size:10px;">적용 시 yaml 자동 저장</small></h3>
      <div class="row">열기<input id="goc" type="number" style="width:65px">
        닫기<input id="gcc" type="number" style="width:65px">
        이송<input id="gtc" type="number" style="width:65px"></div>
      <div class="row">속도<input id="gpv" type="number" style="width:65px">
        가속<input id="gpa" type="number" style="width:65px">
        <button onclick="applyGripper()">적용</button></div>
      <div class="row">Min Safe Z(m)<input id="msz" type="number" step="0.005" style="width:75px">
        <button onclick="cmd('set_min_safe_z',{value:+v('msz')})">적용</button></div>
    </div>
    <div class="card">
      <h3>물체별 파지 강도 (mA) <small style="float:right;color:#5a9;font-size:10px;">적용 시 yaml 자동 저장</small></h3>
      <div class="row">기본 전류<input id="gcd" type="number" style="width:65px">
        <button onclick="applyGripStrength()">전체 적용</button></div>
      <table class="grip-tbl">
        <thead><tr><th>물체</th><th>파지 전류(mA)</th></tr></thead>
        <tbody id="grip-tbl-body"></tbody>
      </table>
    </div>
  </div>
</div>
</div>

<!-- ═══════════════ 로그 탭 ═══════════════ -->
<div class="page" id="page-log">
<div style="padding:12px;">
  <div class="log-filter">
    <button class="active-sel" id="lf-all" onclick="setLogFilter('all')">전체</button>
    <button id="lf-cmd" onclick="setLogFilter('cmd')">CMD</button>
    <button id="lf-err" onclick="setLogFilter('err')">🔴 ERR</button>
    <button id="lf-war" onclick="setLogFilter('war')">🟠 WAR</button>
    <button onclick="pollLog()" style="margin-left:auto;">🔄 새로고침</button>
  </div>
  <div id="loglist"><small>로그 탭 클릭 시 로드됩니다.</small></div>
</div>
</div>

<script>
const $ = id => document.getElementById(id);
const v = id => $(id) ? $(id).value : '';

// ── 탭 전환 ─────────────────────────────────────────────────────────────────
function showTab(t){
  ['main','user','gripper','log'].forEach(n=>{
    const p=$('page-'+n), tb=$('tab-'+n);
    if(p) p.className='page'+(n===t?' active':'');
    if(tb) tb.className='tab'+(n===t?' active':'');
  });
  if(t==='log') pollLog();
  if(t==='user') pollUser();
}

// ── cmd() ────────────────────────────────────────────────────────────────────
async function cmd(action, payload={}){
  try{
    const r = await fetch('/api/command',{method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify({action,payload})});
    const j = await r.json();
    $('activity').textContent='명령 #'+(j.id||'?')+' '+action+' 전송';
    return j;
  }catch(e){
    $('activity').textContent='오류: '+e;
    return {};
  }
}

function applyGripper(){
  cmd('set_gripper_params',{
    open_current:+v('goc'),close_current:+v('gcc'),transport_current:+v('gtc'),
    profile_velocity:+v('gpv'),profile_acceleration:+v('gpa')});
}

// ── 설정 로드 ────────────────────────────────────────────────────────────────
let settingsLoaded=false, gripNames=[], gripCurrents=[];
async function loadSettings(){
  try{
    const s=await(await fetch('/api/settings')).json();
    const set=(id,k)=>{if($(id)&&s[k]!=null&&$(id).value==='')$(id).value=s[k];};
    set('conf','confidence_threshold'); set('exp','camera_exposure');
    set('cx','calib_x_mm'); set('cy','calib_y_mm'); set('cz','calib_z_mm');
    set('goc','gripper_open_current'); set('gcc','gripper_close_current');
    set('gtc','gripper_transport_current'); set('gpv','gripper_profile_velocity');
    set('gpa','gripper_profile_acceleration'); set('msz','min_safe_z');
    set('gcd','grip_current_default');
    gripNames=s.grip_class_names||[]; gripCurrents=s.grip_class_currents||[];
    renderGripTable();
    settingsLoaded=true;
  }catch(e){}
}

function renderGripTable(){
  const tb=$('grip-tbl-body'); if(!tb)return;
  tb.innerHTML=gripNames.map((n,i)=>
    `<tr><td>${n}</td><td><input type="number" id="gc_${i}" value="${gripCurrents[i]||200}" style="width:65px"></td></tr>`
  ).join('');
}

function applyGripStrength(){
  const names=[...gripNames];
  const currents=gripNames.map((_,i)=>+v('gc_'+i)||200);
  const def=+v('gcd')||200;
  cmd('set_grip_strength',{names,currents,default:def});
}

// ── 모델 스캔 ────────────────────────────────────────────────────────────────
async function loadModels(){
  try{
    const d=await(await fetch('/api/models')).json();
    const sel=$('model-sel');
    sel.innerHTML='<option value="">-- 스캔된 파일 선택 --</option>'+
      (d.models||[]).map(p=>{
        const name=p.split('/').pop();
        const cur=p===(d.current||'');
        return `<option value="${p}"${cur?' selected':''}>${name}</option>`;
      }).join('');
  }catch(e){}
}

// ── 상태 코드 매핑 ────────────────────────────────────────────────────────────
const HW={0:'INIT',1:'STANDBY',2:'MOVING',3:'SAFE_OFF',4:'TEACH',5:'SAFE_STOP',
  6:'E-STOP',7:'HOMING',8:'RECOVERY',15:'NOT_READY','-1':'?'};

// ── 서보 토글 ─────────────────────────────────────────────────────────────────
let lastHwState=-1;
function updateServoBtn(hwState){
  lastHwState=hwState;
  const btn=$('servo-btn');
  if(!btn)return;
  if(hwState===3){
    btn.textContent='⚡ 서보 ON';
    btn.className='go';
  }else{
    btn.textContent='서보 OFF';
    btn.className='warn';
  }
}
function toggleServo(){
  if(lastHwState===3){cmd('servo_on');}
  else{if(confirm('서보 OFF? 로봇 낙하 위험'))cmd('servo_off');}
}

// ── 긴급정지 토글 ─────────────────────────────────────────────────────────────
function updateEStopBtn(hwState){
  const btn=$('estop-btn');
  if(!btn)return;
  if(hwState===6){ // E-STOP
    btn.textContent='✅ 긴급정지 해제';
    btn.className='go';
    btn.style.cssText='width:100%;padding:14px;font-size:15px;font-weight:bold;margin-bottom:6px;';
  }else{
    btn.textContent='⛔ 긴급정지';
    btn.className='danger';
    btn.style.cssText='width:100%;padding:14px;font-size:15px;font-weight:bold;margin-bottom:6px;';
  }
}
function toggleEStop(){
  if(lastHwState===6){cmd('e_stop_reset');}
  else{cmd('e_stop');}
}

// ── 그리퍼 전류 sparkline (0 중앙, ±300mA, close=빨강/open=파랑) ─────────────
const GC_LEN=60;
const gcBuf=new Array(GC_LEN).fill(0);
const GC_RANGE=300; // ±300 mA
function gcPush(val){gcBuf.shift();gcBuf.push(Math.max(-GC_RANGE,Math.min(GC_RANGE,+val||0)));}
function gcDraw(){
  const c=$('gchart'); if(!c)return;
  const W=c.offsetWidth||300, H=90;
  c.width=W; c.height=H;
  const ctx=c.getContext('2d');
  ctx.clearRect(0,0,W,H);
  ctx.fillStyle='#111'; ctx.fillRect(0,0,W,H);
  const mid=H/2;
  const valToY=v=>mid-((v/GC_RANGE)*mid);
  // grid
  ctx.strokeStyle='#2a2a2a'; ctx.lineWidth=1;
  [-300,-150,0,150,300].forEach(v=>{
    const y=valToY(v);
    ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(W,y);ctx.stroke();
  });
  // zero line (brighter)
  ctx.strokeStyle='#444'; ctx.lineWidth=1;
  ctx.beginPath();ctx.moveTo(0,mid);ctx.lineTo(W,mid);ctx.stroke();
  // labels
  ctx.font='10px monospace';ctx.textAlign='left';
  [[-300,'#5577ff'],[0,'#888'],[300,'#ff4444']].forEach(([v,col])=>{
    const y=valToY(v);
    ctx.fillStyle=col;
    ctx.fillText((v>0?'+':'')+v,3,y<10?12:(y>H-10?H-2:y+10));
  });
  // draw line in colored segments
  ctx.lineWidth=2;
  gcBuf.forEach((val,i)=>{
    if(i===0)return;
    const x1=W*((i-1)/(GC_LEN-1)), y1=valToY(gcBuf[i-1]);
    const x2=W*(i/(GC_LEN-1)),     y2=valToY(val);
    ctx.strokeStyle=val>=0?'#ff4444':'#4488ff';
    ctx.beginPath();ctx.moveTo(x1,y1);ctx.lineTo(x2,y2);ctx.stroke();
  });
}

// ── 초음파 바 ────────────────────────────────────────────────────────────────
let graspThresh=40;
function usColor(mm){
  if(mm>300)return'#555';
  if(mm>200)return'#888800';
  if(mm>100)return'#cc7700';
  if(mm>graspThresh)return'#ee4400';
  return'#ff1111';
}
function updateUs(mm){
  const fill=$('us-fill'),lbl=$('us-lbl'),mark=$('us-mark');
  if(mm==null||mm<0){
    if(fill)fill.style.width='0%';
    if(lbl)lbl.textContent='- mm';
    return;
  }
  const MAX=400, pct=Math.min(100,mm/MAX*100);
  if(fill){fill.style.width=pct+'%';fill.style.background=usColor(mm);}
  if(lbl)lbl.textContent=mm+' mm';
  if(mark)mark.style.left=(graspThresh/MAX*100)+'%';
}

// ── 검출 물체 + 선택 ─────────────────────────────────────────────────────────
let selectedLabel='';
function selectObject(label){
  selectedLabel=label||'';
  cmd('select_object',{label:selectedLabel});
  updateSelLabel();
}
function updateSelLabel(){
  const el=$('sel-label');
  if(el)el.textContent='선택: '+(selectedLabel||'자동');
}
function renderObjArea(objects){
  const selBtns=objects.filter(o=>o.label!=='box'&&o.label!=='source_zone'&&o.reachable);
  const seen={},knownLabels=[],unknownLabels=[];
  selBtns.forEach(o=>{
    if(!seen[o.label]){
      seen[o.label]=1;
      if(o.label.startsWith('unknown')) unknownLabels.push(o.label);
      else knownLabels.push(o.label);
    }
  });
  const mkBtn=lb=>{
    const act=lb===selectedLabel?'active-sel':'';
    return `<button class="${act}" onclick="selectObject('${lb}')">${lb}</button>`;
  };
  const kRow=$('known-row');
  if(kRow) kRow.innerHTML=knownLabels.map(mkBtn).join('');
  const uRow=$('unknown-row');
  if(uRow) uRow.innerHTML=unknownLabels.map(mkBtn).join('');
  updateSelLabel();
}

// ── 메인 poll ────────────────────────────────────────────────────────────────
async function poll(){
  try{
    const s=await(await fetch('/api/state')).json();
    const ss=s.system_status||{};
    $('status-dots').innerHTML=Object.entries(ss).map(([k,st])=>{
      const sep=k==='USER'?'<span class="dot-sep"></span>':'';
      return sep+`<span class="dot ${st}">${k}</span>`;
    }).join(' ');
    const stt=s.pick_place_state||'?';
    $('activity').textContent=`상태: ${stt} · HW: ${HW[s.hw_state]||s.hw_state} · 속도: ${s.speed_mode===1?'감속':'정상'}`;
    const err=(stt==='ERROR')?(s.error_text||'ERROR'):'';
    $('banner').style.display=err?'block':'none';
    $('banner').textContent=err?('🔴 '+err):'';
    updateServoBtn(s.hw_state);
    updateEStopBtn(s.hw_state);
    if(s.has_image){
      $('cam').src='/api/image.jpg?t='+Date.now();
      $('cam').style.display='block'; $('cam-no').style.display='none';
    }else{
      $('cam').style.display='none'; $('cam-no').style.display='flex';
    }
    renderObjArea(s.detected_objects||[]);
    const gc=s.gripper_current_ma;
    const gp=s.gripper_position_raw;
    if($('g_curr2'))$('g_curr2').textContent=gc;
    if($('g_pos2'))$('g_pos2').textContent=gp;
    gcPush(gc); gcDraw();
    updateUs(s.ultrasonic_mm);
    const mp=s.current_model_path||'';
    if(mp&&$('model-path'))$('model-path').textContent=mp;
  }catch(e){$('activity').textContent='연결 끊김: '+e;}
  if(!settingsLoaded)loadSettings();
}
setInterval(poll,200); poll();

// ── 공통 주문 헬퍼 ───────────────────────────────────────────────────────────
const ST_CLS={RUNNING:'running',PAUSED:'pause',QUEUED:'queued',DONE:'done',FAILED:'failed',CANCELED:'canceled'};
function statusKo(s){return{RUNNING:'처리중',PAUSED:'보류',QUEUED:'대기',DONE:'완료',FAILED:'실패',CANCELED:'취소'}[s]||s;}
async function cancelOrder(id){
  if(!confirm('주문 취소? (처리중 품목 보호)'))return;
  const r=await fetch('/api/orders/cancel',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({order_id:id})});
  const j=await r.json();
  if(!j.ok)alert('취소 실패: '+(j.error||'처리중 품목 보호'));
  pollOrders(); pollUser();
}
async function cancelItem(itemId){
  if(!confirm('이 품목을 취소?'))return;
  const r=await fetch('/api/orders/cancel_item',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({item_id:itemId})});
  const j=await r.json();
  if(!j.ok)alert('취소 실패: '+(j.error||''));
  pollOrders(); pollUser();
}
async function toggleQueue(){
  await fetch('/api/orders/pause',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}'});
  pollOrders();
}
function renderOrderCard(o){
  const cls=ST_CLS[o.status]||'queued';
  const items=(o.items||[]).map(it=>{
    const ic=ST_CLS[it.status]||'queued';
    const cb=it.status==='QUEUED'?`<button class="item-cancel" onclick="cancelItem('${it.item_id||''}')">✕</button>`:'';
    return `<div class="item-row"><span class="badge ${ic}">${it.class_name}</span>`+
           `<span class="badge ${ic}" style="font-size:9px;">${statusKo(it.status)}</span>${cb}</div>`;
  }).join('');
  const isDone=o.status==='DONE'||o.status==='CANCELED'||o.status==='FAILED';
  const lk=o.locker_id?`<span class="badge done" style="margin-left:6px;">🔒${o.locker_id}번</span>`:'';
  // 수령대기(DONE)는 대리수령, 진행 중이면 취소.
  const actBtn = o.status==='DONE' && o.qr_token
    ? `<button class="item-cancel" style="margin-left:auto;background:#1a7a1a;" onclick="pickupLocker('${o.qr_token}')">수령</button>`
    : (isDone?'':`<button class="item-cancel" style="margin-left:auto;" onclick="cancelOrder('${o.order_id}')">취소</button>`);
  return `<div class="order ${cls}">`+
    `<div style="display:flex;align-items:center;margin-bottom:4px;">`+
    `<span class="tk">${o.ticket_no||o.order_id}</span>`+
    `<span class="badge ${cls}" style="margin-left:6px;">${statusKo(o.status)}</span>`+
    lk+actBtn+`</div>`+
    `${items||'<small>품목 없음</small>'}`+
    `<div style="margin-top:3px;"><small>${o.created_at||''}</small></div></div>`;
}

// ── 메인 탭: RUNNING 주문 하나만 표시 ─────────────────────────────────────────
async function pollOrders(){
  try{
    const d=await(await fetch('/api/orders')).json();
    const orders=d.orders||[];
    const running=orders.filter(o=>o.status==='RUNNING');
    const el=$('orders');
    if(!el)return;
    if(d.error){el.innerHTML=`<small>${d.error}</small>`;}
    else if(running.length){el.innerHTML=running.map(o=>renderOrderCard(o)).join('');}
    else{el.innerHTML='<small style="color:#666;">작업 중인 주문 없음</small>';}
  }catch(e){const el=$('orders');if(el)el.innerHTML='<small>큐 로드 실패: '+e+'</small>';}
}
setInterval(pollOrders,1500); pollOrders();

// ── 락커 현황 ───────────────────────────────────────────────────────────────
const LK_KO={free:'비어있음',occupied:'배달중',ready:'수령대기'};
async function releaseLocker(id){
  if(!confirm(id+'번 락커를 강제 해제할까요?'))return;
  await fetch('/api/lockers/release',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({id})});
  pollLockers(); pollOrders();
}
async function pickupLocker(token){
  if(!confirm('대리 수령 처리할까요? (락커 해제)'))return;
  const j=await(await fetch('/api/lockers/pickup',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({token})})).json();
  if(!j.ok)alert('수령 처리 실패: '+(j.error||(j.detail&&j.detail.status)||'아직 준비 안 됨'));
  pollLockers(); pollOrders();
}
async function resetLockers(){
  if(!confirm('모든 락커를 초기화할까요?'))return;
  await fetch('/api/lockers/reset',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}'});
  pollLockers(); pollOrders();
}
async function pollLockers(){
  try{
    const d=await(await fetch('/api/lockers')).json();
    const el=$('lockers'); if(!el)return;
    el.innerHTML=(d.lockers||[]).map(l=>{
      const st=l.status||'free', tk=l.ticket_no?(' '+l.ticket_no):'';
      // 수령 번호(qr_token) — 점유 칸만. 관리자 확인/대리수령용.
      const code=(st!=='free'&&l.qr_token)?`<div class="lkcode">🔑 ${l.qr_token}</div>`:'';
      let b=''; if(st==='ready')b+=`<button onclick="pickupLocker('${l.qr_token}')">수령처리</button>`;
      if(st!=='free')b+=`<button onclick="releaseLocker(${l.id})">해제</button>`;
      return `<div class="lk ${st}"><div class="lkid">#${l.id}</div><div>${LK_KO[st]||st}${tk}</div>${code}${b}</div>`;
    }).join('')||'<small>락커 없음</small>';
  }catch(e){const el=$('lockers');if(el)el.innerHTML='<small>락커 로드 실패: '+e+'</small>';}
}
setInterval(pollLockers,1500); pollLockers();

// ── 유저 큐 탭 전체 관리 ─────────────────────────────────────────────────────
async function pollUser(){
  const showAll=$('show-history')&&$('show-history').checked;
  try{
    const d=await(await fetch('/api/orders'+(showAll?'?all=1':''))).json();
    const pb=$('user-q-badge');
    if(pb){
      if(d.error)pb.innerHTML=`<span class="badge failed">${d.error}</span>`;
      else if(d.paused)pb.innerHTML='<span class="badge pause">⏸ 큐 보류중</span>';
      else pb.innerHTML='<span class="badge run">▶ 동작중</span>';
    }
    const pBtn=$('user-pause-btn');
    if(pBtn)pBtn.textContent=d.paused?'▶ 큐 재개':'⏸ 큐 보류';
    const orders=d.orders||[];
    const cnt={RUNNING:0,QUEUED:0,PAUSED:0,DONE:0,FAILED:0,CANCELED:0};
    orders.forEach(o=>{if(cnt[o.status]!==undefined)cnt[o.status]++;});
    const stats=$('user-stats');
    if(stats)stats.innerHTML=['RUNNING','QUEUED','PAUSED','DONE','FAILED','CANCELED'].map(k=>{
      const ko={RUNNING:'처리중',QUEUED:'대기',PAUSED:'보류',DONE:'완료',FAILED:'실패',CANCELED:'취소'}[k];
      return `<div class="stat-box"><div class="sval">${cnt[k]}</div><div class="slbl">${ko}</div></div>`;
    }).join('');
    const uo=$('user-orders');
    if(uo)uo.innerHTML=orders.length?orders.map(o=>renderOrderCard(o)).join(''):'<small>주문 없음</small>';
  }catch(e){const uo=$('user-orders');if(uo)uo.innerHTML='<small>로드 실패: '+e+'</small>';}
}

// ── 로그 탭 ──────────────────────────────────────────────────────────────────
let logFilter='all';
function setLogFilter(f){
  logFilter=f;
  ['all','cmd','err','war'].forEach(k=>{
    const b=$('lf-'+k);
    if(b)b.className=k===f?'active-sel':'';
  });
  pollLog();
}
(function(){const h=location.hash.replace('#','');if(h&&['main','user','gripper','log'].includes(h))showTab(h);})();

async function pollLog(){
  try{
    const data=await(await fetch('/api/log')).json();
    const list=$('loglist');
    if(!list)return;
    const filtered=logFilter==='all'?data:data.filter(e=>e.type===logFilter);
    if(!filtered.length){list.innerHTML='<small>로그 없음</small>';return;}
    list.innerHTML=filtered.map(e=>{
      const isUser=e.source&&e.source.includes('유저');
      const userCls=isUser?' user-src':'';
      const typeBadge=`<span class="log-badge ${e.type}">${e.type.toUpperCase()}</span>`;
      const stBadge=e.status?`<span class="log-badge ${e.status.toLowerCase()}">${e.status}</span>`:'';
      const done=e.done_ts_str?` → ${e.done_ts_str}`:'';
      return `<div class="log-entry ${e.type}${userCls}">`+
        `<div class="log-row1"><span>${e.ts_str}${done}</span>`+
        `<span style="color:${isUser?'#999':'#ccc'}">${e.source}</span></div>`+
        `<div class="log-row2">${typeBadge} ${stBadge} ${e.text}${e.detail?` <small>· ${e.detail}</small>`:''}</div>`+
        `</div>`;
    }).join('');
  }catch(e){const l=$('loglist');if(l)l.innerHTML='<small>로그 로드 실패: '+e+'</small>';}
}
</script>
</body>
</html>
"""


def main(args=None):
    rclpy.init(args=args)
    node = WebControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
