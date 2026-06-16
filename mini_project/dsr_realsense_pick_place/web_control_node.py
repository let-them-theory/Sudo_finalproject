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
                         "jelly", "box", "can", "boxsnack", "wafers"],
    'grip_class_currents': [150, 150, 150, 120, 180, 140, 180, 200, 170, 150],
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
            """
        )
        self.db.commit()

    def _seed_settings(self):
        """DB에 없는 설정 키만 기본값으로 채운다(기존 값 보존)."""
        cur = self.db.cursor()
        for key, val in DEFAULT_SETTINGS.items():
            cur.execute('INSERT OR IGNORE INTO settings(key, value) VALUES (?, ?)',
                        (key, json.dumps(val)))
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
            candidates.append(parent / 'mini_project' / rel)
            candidates.append(parent / 'src' / 'mini_project' / rel)
        candidates.append(Path.cwd() / rel)
        candidates.append(Path.cwd() / 'src' / 'mini_project' / rel)

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
        self.pick_place_state = msg.data
        self.last_state_time = time.monotonic()

    def _cb_error(self, msg: String):
        self.last_error_text = msg.data

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
        system_status = {
            'HW':   'ok' if fresh(self.last_hw_state_time) else 'warn',
            'GRIP': grip_state,
            'CAM':  'ok' if fresh(self.last_image_time) else 'bad',
            'DET':  'ok' if fresh(self.last_objects_time) else 'bad',
            'PICK': 'ok' if (self.cli_run_once.service_is_ready() and fresh(self.last_state_time)) else 'bad',
            'ARD':  'ok' if fresh(self.last_ultrasonic_time) else 'bad',
            'SPD':  'ok' if fresh(self.last_speed_mode_time) else 'warn',
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
        model = str(self._get_setting('yolo_model_path') or '').strip()
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
            # 기존 GUI와 동일하게 IDLE 상태에서만 허용.
            if self.pick_place_state != 'IDLE':
                self._mark_command(cmd_id, 'failed', '캘리브레이션 적용은 IDLE 상태에서만 가능')
                return
            x = float(payload.get('x', 0.0)); y = float(payload.get('y', 0.0)); z = float(payload.get('z', 0.0))
            self._set_params(cmd_id, self.cli_object_set_parameters, [
                ('absolute_calib_x_mm', ParameterType.PARAMETER_DOUBLE, x),
                ('absolute_calib_y_mm', ParameterType.PARAMETER_DOUBLE, y),
                ('absolute_calib_z_mm', ParameterType.PARAMETER_DOUBLE, z),
            ])
            self._set_setting('calib_x_mm', x); self._set_setting('calib_y_mm', y); self._set_setting('calib_z_mm', z)
            return

        if action == 'set_model':
            path = str(payload.get('path', '')).strip()
            if not path:
                self._mark_command(cmd_id, 'failed', '모델 경로가 비어 있음')
                return
            self._set_params(cmd_id, self.cli_object_set_parameters,
                             [('yolo_model', ParameterType.PARAMETER_STRING, path)])
            self._set_setting('yolo_model_path', path)
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
                if path != '/api/command':
                    self._send_json({'error': 'not found'}, 404)
                    return
                length = int(self.headers.get('Content-Length', 0))
                raw = self.rfile.read(length) if length else b'{}'
                try:
                    body = json.loads(raw or b'{}')
                except json.JSONDecodeError:
                    self._send_json({'error': 'bad json'}, 400)
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

            def _read_command(self, cid):
                conn = self._db()
                row = conn.execute(
                    'SELECT id, ts, action, payload, status, result, done_ts '
                    'FROM command_queue WHERE id=?', (cid,)).fetchone()
                conn.close()
                if row is None:
                    return {'error': 'not found'}
                return dict(row)

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
  :root { color-scheme: dark; }
  body { font-family: system-ui, sans-serif; margin: 0; background:#1e1e1e; color:#e0e0e0; }
  header { background:#101820; padding:8px 14px; display:flex; gap:10px; align-items:center; flex-wrap:wrap; }
  .dot { padding:2px 8px; border-radius:4px; font-size:12px; font-weight:bold; background:#666; }
  .ok{background:#1a7a1a;} .warn{background:#b38600;} .bad{background:#cc0000;}
  main { display:flex; gap:14px; padding:14px; flex-wrap:wrap; align-items:flex-start; }
  .col { flex:1; min-width:320px; }
  .card { background:#2a2a2a; border-radius:10px; padding:12px; margin-bottom:12px; }
  .card h3 { margin:0 0 8px; font-size:14px; color:#9fd0ff; }
  button { background:#374151; color:#fff; border:none; border-radius:6px; padding:8px 10px;
           font-size:13px; cursor:pointer; margin:3px; }
  button:hover { background:#4b5563; }
  button.danger { background:#cc0000; } button.danger:hover{background:#ff1a1a;}
  button.warn   { background:#e65c00; } button.go{background:#1a7a1a;}
  input, select { background:#1e1e1e; color:#fff; border:1px solid #555; border-radius:4px; padding:4px; }
  #activity { font-weight:bold; color:#cfe8ff; }
  #banner { color:#ff6666; font-weight:bold; padding:6px; display:none; }
  table { width:100%; border-collapse:collapse; font-size:12px; }
  td,th { border-bottom:1px solid #444; padding:3px 4px; text-align:left; }
  .obj { background:#333; border-radius:6px; padding:6px; margin:4px 0; cursor:pointer; }
  .obj:hover { background:#3d4f63; }
  .obj.unreach { opacity:.4; cursor:not-allowed; }
  img { max-width:100%; border-radius:8px; background:#111; }
  .row { display:flex; gap:6px; align-items:center; margin:4px 0; flex-wrap:wrap; }
  label { font-size:12px; }
  small { color:#999; }
</style>
</head>
<body>
<header>
  <strong>DSR Pick &amp; Place</strong>
  <span id="status-dots"></span>
  <span id="activity">연결 중...</span>
</header>
<div id="banner"></div>
<main>
  <div class="col">
    <div class="card">
      <h3>카메라 / 검출</h3>
      <img id="cam" alt="카메라 영상" src="">
      <div id="objects"></div>
      <button onclick="cmd('select_object',{label:''})">자동 선택</button>
    </div>
  </div>

  <div class="col">
    <div class="card">
      <h3>긴급 제어</h3>
      <button class="danger" onclick="cmd('e_stop')">⛔ 긴급정지</button>
      <button class="warn" onclick="cmd('cancel')">🚫 태스크 중단</button>
      <button class="go" onclick="cmd('e_stop_reset')">✅ 긴급정지 해제</button>
      <button onclick="cmd('clear_error')">에러 해제</button>
    </div>
    <div class="card">
      <h3>로봇 동작</h3>
      <button onclick="cmd('run_once')">▶ 한 번 실행</button>
      <button onclick="cmd('go_home')">🏠 HOME 이동</button>
      <button onclick="cmd('recover_to_home')">에러복구 &amp; HOME</button>
      <div class="row">
        <button onclick="cmd('speed_normal')">🟢 정상속도</button>
        <button onclick="cmd('speed_reduced')">🟡 감속</button>
        <button onclick="cmd('servo_on')">서보 ON</button>
        <button class="warn" onclick="if(confirm('서보 OFF? 중력으로 로봇이 떨어질 수 있습니다'))cmd('servo_off')">서보 OFF</button>
        <button onclick="cmd('safety_normal')">정상운전</button>
        <button onclick="cmd('safety_backdrive')">역구동</button>
      </div>
    </div>
    <div class="card">
      <h3>그리퍼</h3>
      <button onclick="cmd('gripper_open')">OPEN</button>
      <button onclick="cmd('gripper_close')">CLOSE</button>
      <button onclick="cmd('gripper_enable',{enable:true})">토크 ON</button>
      <button onclick="cmd('gripper_enable',{enable:false})">토크 OFF</button>
      <button onclick="cmd('restart_gripper_bridge')">🔧 브릿지 재시작</button>
      <div><small>전류: <span id="g_curr">-</span> mA · 위치: <span id="g_pos">-</span> · 초음파: <span id="us">-</span> mm</small></div>
    </div>
  </div>

  <div class="col">
    <div class="card">
      <h3>검출 / 카메라 튜닝</h3>
      <div class="row"><label>신뢰도</label>
        <input id="conf" type="number" step="0.01" min="0.05" max="0.95" style="width:70px">
        <button onclick="cmd('set_confidence',{value:+v('conf')})">적용</button></div>
      <div class="row"><label>자동노출</label>
        <button onclick="cmd('set_camera_auto_exposure',{enable:true})">ON</button>
        <button onclick="cmd('set_camera_auto_exposure',{enable:false})">OFF</button>
        <input id="exp" type="number" min="20" max="5000" style="width:80px">
        <button onclick="cmd('set_camera_exposure',{value:+v('exp')})">노출적용</button></div>
    </div>
    <div class="card">
      <h3>캘리브레이션 (mm, IDLE 전용)</h3>
      <div class="row">
        X<input id="cx" type="number" step="1" style="width:70px">
        Y<input id="cy" type="number" step="1" style="width:70px">
        Z<input id="cz" type="number" step="1" style="width:70px">
      </div>
      <button onclick="cmd('load_calibration')">불러오기</button>
      <button onclick="cmd('set_calibration',{x:+v('cx'),y:+v('cy'),z:+v('cz')})">적용</button>
    </div>
    <div class="card">
      <h3>그리퍼 정밀 / 안전</h3>
      <div class="row">열기<input id="goc" type="number" style="width:70px">
        닫기<input id="gcc" type="number" style="width:70px">
        이송<input id="gtc" type="number" style="width:70px"></div>
      <div class="row">속도<input id="gpv" type="number" style="width:70px">
        가속<input id="gpa" type="number" style="width:70px">
        <button onclick="applyGripper()">적용</button></div>
      <div class="row">Min Safe Z(m)<input id="msz" type="number" step="0.005" style="width:80px">
        <button onclick="cmd('set_min_safe_z',{value:+v('msz')})">적용</button></div>
    </div>
    <div class="card">
      <h3>모델 / 시스템</h3>
      <div class="row"><input id="model" type="text" placeholder="YOLO .pt 경로" style="flex:1">
        <button onclick="cmd('set_model',{path:v('model')})">적용</button></div>
      <button class="go" onclick="cmd('save_yaml')">💾 전체 yaml 저장</button>
      <button class="warn" onclick="if(confirm('GUI 제외 전 노드 재시작?'))cmd('system_reset')">🔄 시스템 리셋</button>
      <div><small>설정 변경은 DB에 즉시 저장되고, 위 튜닝값은 yaml에도 자동 기록됩니다.</small></div>
    </div>
    <div class="card">
      <h3>최근 명령</h3>
      <table id="log"><tbody></tbody></table>
    </div>
  </div>
</main>
<script>
const $ = id => document.getElementById(id);
const v = id => $(id).value;
async function cmd(action, payload={}) {
  const r = await fetch('/api/command', {method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({action, payload})});
  const j = await r.json();
  $('activity').textContent = '명령 #' + (j.id||'?') + ' ' + action + ' 전송';
  return j;
}
function applyGripper(){ cmd('set_gripper_params', {
  open_current:+v('goc'), close_current:+v('gcc'), transport_current:+v('gtc'),
  profile_velocity:+v('gpv'), profile_acceleration:+v('gpa')}); }

let settingsLoaded = false;
async function loadSettings(){
  const s = await (await fetch('/api/settings')).json();
  const set=(id,k)=>{ if($(id) && s[k]!=null && $(id).value==='') $(id).value=s[k]; };
  set('conf','confidence_threshold'); set('exp','camera_exposure');
  set('cx','calib_x_mm'); set('cy','calib_y_mm'); set('cz','calib_z_mm');
  set('goc','gripper_open_current'); set('gcc','gripper_close_current');
  set('gtc','gripper_transport_current'); set('gpv','gripper_profile_velocity');
  set('gpa','gripper_profile_acceleration'); set('msz','min_safe_z'); set('model','yolo_model_path');
  settingsLoaded = true;
}
const HW={0:'INIT',1:'STANDBY',2:'MOVING',3:'SAFE_OFF',4:'TEACH',5:'SAFE_STOP',
  6:'E-STOP',7:'HOMING',8:'RECOVERY',15:'NOT_READY','-1':'?'};
async function poll(){
  try{
    const s = await (await fetch('/api/state')).json();
    // 상태점
    const ss = s.system_status||{};
    $('status-dots').innerHTML = Object.entries(ss).map(([k,st])=>
      `<span class="dot ${st}">${k}</span>`).join(' ');
    const stt = s.pick_place_state||'?';
    $('activity').textContent = `상태: ${stt} · HW: ${HW[s.hw_state]||s.hw_state} · 속도: ${s.speed_mode===1?'감속':'정상'}`;
    const err = (stt==='ERROR') ? (s.error_text||'ERROR') : '';
    $('banner').style.display = err?'block':'none'; $('banner').textContent = err?('🔴 '+err):'';
    $('g_curr').textContent = s.gripper_current_ma; $('g_pos').textContent = s.gripper_position_raw;
    $('us').textContent = s.ultrasonic_mm==null?'-':s.ultrasonic_mm;
    // 검출 물체
    $('objects').innerHTML = (s.detected_objects||[]).map(o=>{
      const cls = o.reachable?'obj':'obj unreach';
      const onclick = o.reachable?`onclick="cmd('select_object',{label:'${o.label}'}).then(()=>cmd('run_once'))"`:'';
      const p=o.pose||{};
      return `<div class="${cls}" ${onclick}>${o.label} `+
             `<small>conf ${(o.confidence||0).toFixed(2)} · [${(p.x||0).toFixed(2)},${(p.y||0).toFixed(2)},${(p.z||0).toFixed(2)}]m`+
             `${o.reachable?'':' · 도달불가'}</small></div>`;
    }).join('') || '<small>검출된 물체 없음</small>';
    if(s.has_image) $('cam').src = '/api/image.jpg?t=' + Date.now();
    // 명령 로그
    const logs = await (await fetch('/api/commands')).json();
    $('log').querySelector('tbody').innerHTML = logs.map(l=>
      `<tr><td>#${l.id}</td><td>${l.action}</td><td>${l.status}</td><td><small>${l.result||''}</small></td></tr>`).join('');
  }catch(e){ $('activity').textContent='연결 끊김: '+e; }
  if(!settingsLoaded) loadSettings();
}
setInterval(poll, 500); poll();
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
