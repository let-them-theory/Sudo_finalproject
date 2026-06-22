# user GUI 주문/큐/재고를 영속 저장하는 리포지토리 (지금 JSON, 나중 sqlite로 교체)
"""
User GUI 데이터 계층. ROS 비의존 순수 모듈 — User_gui_node가 import해서 씀.

설계 결정 (User_gui_context-notes.md 참조)
- task 계층: Order(부모) + OrderItem(자식). 수량 N = 자식 N개 펼침.
- 큐: 자식 item의 실행 순서 리스트(FIFO + 재정렬). 새 주문은 뒤에 append.
- 상태: order(QUEUED/RUNNING/DONE/CANCELED/PAUSED), item(QUEUED/RUNNING/DONE/FAILED/CANCELED).
- 재고: catalog stock. item DONE 시 차감(검출 기반 자동갱신은 후속).
- 교체점: Repository 인터페이스 1개 → JsonRepository(now) / SqliteRepository(later).
"""

from __future__ import annotations

import json
import random
import sqlite3
import threading
from abc import ABC, abstractmethod
from dataclasses import dataclass, field, asdict
from datetime import datetime
from pathlib import Path


# ─── 상태 상수 ──────────────────────────────────────────────
class OrderStatus:
    QUEUED = 'QUEUED'
    RUNNING = 'RUNNING'
    DONE = 'DONE'          # 전 품목 배달 완료 — 락커 ready, 수령 대기
    CANCELED = 'CANCELED'
    PAUSED = 'PAUSED'
    PICKED = 'PICKED'      # 유저 수령 완료 — 락커 free (DONE 이후 단말 상태)


class ItemStatus:
    QUEUED = 'QUEUED'
    RUNNING = 'RUNNING'
    DONE = 'DONE'
    FAILED = 'FAILED'
    CANCELED = 'CANCELED'


# 큐(실행 대상)로 보는 item 상태 — 아직 안 끝난 것.
_ACTIVE_ITEM = {ItemStatus.QUEUED, ItemStatus.RUNNING}

# 락커 개수. 주문 시 빈 락커 배정, 수령 완료 시 해제.
LOCKER_COUNT = 8

# 락커 상태.
class LockerStatus:
    FREE = 'free'          # 비어 있음
    OCCUPIED = 'occupied'  # 주문 배정됨(배달 중)
    READY = 'ready'        # 배달 완료 — 수령 대기


# ─── 데이터 모델 ────────────────────────────────────────────
@dataclass
class CatalogItem:
    class_name: str            # known_classes 값 (예: "can")
    display_name: str          # 화면 표시명
    default_grip: int = 200    # 기본 파지 전류(mA)
    stock: int = 0             # 재고 수량 (지금 수동 튜닝)


@dataclass
class OrderItem:
    item_id: str
    order_id: str
    class_name: str
    status: str = ItemStatus.QUEUED


@dataclass
class Order:
    order_id: str
    ticket_no: str             # 번호표 (예: "A-017")
    status: str = OrderStatus.QUEUED
    created_at: str = ''
    item_ids: list = field(default_factory=list)
    locker_id: int = 0         # 배정 락커(1~N), 0=미배정
    qr_token: str = ''         # 수령 QR 토큰
    picked_at: str = ''        # 수령 완료 시각


# ─── 리포지토리 인터페이스 ──────────────────────────────────
class TaskRepository(ABC):
    """user GUI가 의존하는 데이터 계층. JSON·sqlite 구현이 이 인터페이스를 만족한다."""

    # catalog
    @abstractmethod
    def seed_catalog(self, items: list[CatalogItem]) -> None: ...
    @abstractmethod
    def list_catalog(self) -> list[CatalogItem]: ...
    @abstractmethod
    def get_catalog_item(self, class_name: str) -> CatalogItem | None: ...
    @abstractmethod
    def adjust_stock(self, class_name: str, delta: int) -> None: ...

    # order
    @abstractmethod
    def create_order(self, lines: list[tuple[str, int]]) -> Order:
        """lines = [(class_name, qty), ...] → 자식 item 펼쳐 생성 + 큐 뒤에 append."""
    @abstractmethod
    def get_order(self, order_id: str) -> Order | None: ...
    @abstractmethod
    def list_orders(self, statuses: set[str] | None = None) -> list[Order]: ...
    @abstractmethod
    def cancel_order(self, order_id: str) -> None: ...

    # item / queue
    @abstractmethod
    def get_item(self, item_id: str) -> OrderItem | None: ...
    @abstractmethod
    def get_queue(self) -> list[OrderItem]:
        """실행 순서대로 아직 안 끝난(QUEUED/RUNNING) item."""
    @abstractmethod
    def next_queued_item(self) -> OrderItem | None: ...
    @abstractmethod
    def set_item_status(self, item_id: str, status: str) -> None: ...
    @abstractmethod
    def cancel_item(self, item_id: str) -> None: ...
    @abstractmethod
    def reorder_item(self, item_id: str, new_index: int) -> None:
        """큐(QUEUED item) 안에서 실행 순서 이동."""

    # history
    @abstractmethod
    def list_history(self) -> list[dict]: ...

    # pick 통계 / 재고 (admin)
    @abstractmethod
    def record_pick_result(self, class_name: str, success: bool, reason: str = '') -> None: ...
    @abstractmethod
    def get_pick_stats(self) -> list[dict]: ...
    @abstractmethod
    def set_stock(self, class_name: str, qty: int) -> None: ...


# ─── JSON 구현 ──────────────────────────────────────────────
_DEFAULT_DB_PATH = Path.home() / '.config' / 'dsr_realsense_pick_place' / 'user_gui_db.json'


class JsonRepository(TaskRepository):
    """단일 JSON 파일에 전체 상태를 저장. 변경마다 atomic 저장. 스레드 락 보호."""

    def __init__(self, path: Path | str = _DEFAULT_DB_PATH):
        self._path = Path(path)
        self._lock = threading.RLock()
        self._data = {
            'version': 1,
            'ticket_counter': 0,
            'order_counter': 0,
            'catalog': {},   # class_name -> CatalogItem dict
            'orders': {},    # order_id   -> Order dict
            'items': {},     # item_id    -> OrderItem dict
            'queue': [],     # [item_id, ...] 실행 순서
            'history': [],   # [dict, ...]
            'lockers': {},   # "1".."N" -> {status, order_id, qr_token, assigned_at, ready_at}
        }
        self._load()
        self._ensure_schema()

    # ── 영속 ──
    def _load(self) -> None:
        if self._path.exists():
            with self._path.open('r', encoding='utf-8') as f:
                self._data = json.load(f)

    def reload(self) -> None:
        """디스크에서 다시 읽어 메모리 동기화.
        다중 프로세스(web 백엔드 + 관리자 GUI)가 같은 파일을 공유할 때,
        읽기/쓰기 직전에 호출해 상대 프로세스의 변경을 반영한다."""
        with self._lock:
            self._load()
            self._ensure_schema()

    def _ensure_schema(self) -> None:
        """구버전 JSON 호환 — 누락 키 보강 + 락커 N칸 초기화(락 비보유 호출 가정: __init__/reload 내부)."""
        self._data.setdefault('lockers', {})
        lk = self._data['lockers']
        changed = False
        for i in range(1, LOCKER_COUNT + 1):
            key = str(i)
            if key not in lk:
                lk[key] = {'status': LockerStatus.FREE, 'order_id': '',
                           'qr_token': '', 'assigned_at': '', 'ready_at': ''}
                changed = True
        if changed and self._path.exists():
            self._save()

    def _save(self) -> None:
        self._path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self._path.with_suffix('.json.tmp')
        with tmp.open('w', encoding='utf-8') as f:
            json.dump(self._data, f, ensure_ascii=False, indent=2)
        tmp.replace(self._path)   # atomic

    @staticmethod
    def _now() -> str:
        return datetime.now().isoformat(timespec='seconds')

    # ── catalog ──
    def seed_catalog(self, items: list[CatalogItem]) -> None:
        with self._lock:
            cat = self._data['catalog']
            for it in items:
                # 이미 있으면 stock·표시명 보존(수동 튜닝 덮어쓰지 않음), 없으면 추가.
                if it.class_name not in cat:
                    cat[it.class_name] = asdict(it)
            self._save()

    def list_catalog(self) -> list[CatalogItem]:
        with self._lock:
            return [CatalogItem(**d) for d in self._data['catalog'].values()]

    def get_catalog_item(self, class_name: str) -> CatalogItem | None:
        with self._lock:
            d = self._data['catalog'].get(class_name)
            return CatalogItem(**d) if d else None

    def adjust_stock(self, class_name: str, delta: int) -> None:
        with self._lock:
            d = self._data['catalog'].get(class_name)
            if d is None:
                return
            d['stock'] = max(0, int(d.get('stock', 0)) + delta)
            self._save()

    def set_stock(self, class_name: str, qty: int) -> None:
        with self._lock:
            d = self._data['catalog'].get(class_name)
            if d is None:
                return
            d['stock'] = max(0, int(qty))
            self._save()

    # ── pick 통계 (admin: 품목별 성공/실패) ──
    def record_pick_result(self, class_name: str, success: bool, reason: str = '') -> None:
        with self._lock:
            stats = self._data.setdefault('pick_stats', {})
            s = stats.setdefault(class_name, {'success': 0, 'fail': 0,
                                              'last_fail_reason': '', 'last_fail_at': ''})
            if success:
                s['success'] = int(s.get('success', 0)) + 1
            else:
                s['fail'] = int(s.get('fail', 0)) + 1
                s['last_fail_reason'] = reason
                s['last_fail_at'] = self._now()
            self._save()

    def get_pick_stats(self) -> list[dict]:
        with self._lock:
            return [{'class_name': cn, **s}
                    for cn, s in self._data.get('pick_stats', {}).items()]

    # ── order ──
    def create_order(self, lines: list[tuple[str, int]]) -> Order:
        with self._lock:
            self._data['order_counter'] += 1
            self._data['ticket_counter'] += 1
            oid = f"order{self._data['order_counter']}"
            ticket = f"A-{self._data['ticket_counter']:03d}"
            order = Order(order_id=oid, ticket_no=ticket,
                          status=OrderStatus.QUEUED, created_at=self._now())
            k = 0
            for class_name, qty in lines:
                for _ in range(max(0, int(qty))):
                    k += 1
                    iid = f"{oid}_{k}"
                    item = OrderItem(item_id=iid, order_id=oid, class_name=class_name)
                    self._data['items'][iid] = asdict(item)
                    self._data['queue'].append(iid)
                    order.item_ids.append(iid)
            self._data['orders'][oid] = asdict(order)
            self._save()
            return order

    def get_order(self, order_id: str) -> Order | None:
        with self._lock:
            d = self._data['orders'].get(order_id)
            return Order(**d) if d else None

    def list_orders(self, statuses: set[str] | None = None) -> list[Order]:
        with self._lock:
            out = [Order(**d) for d in self._data['orders'].values()]
        if statuses is not None:
            out = [o for o in out if o.status in statuses]
        return out

    def cancel_order(self, order_id: str, protect_running: bool = False) -> bool:
        """주문 취소. protect_running=True면 RUNNING(이미 처리 중) 품목이 있으면
        아무것도 취소하지 않고 False 반환(로봇이 집는 중인 주문 보호). 성공 시 True."""
        with self._lock:
            order = self._data['orders'].get(order_id)
            if order is None:
                return False
            if protect_running and any(
                self._data['items'].get(iid, {}).get('status') == ItemStatus.RUNNING
                for iid in order['item_ids']):
                return False
            for iid in order['item_ids']:
                it = self._data['items'].get(iid)
                if it and it['status'] in _ACTIVE_ITEM:
                    it['status'] = ItemStatus.CANCELED
                    self._append_history_locked(it, ItemStatus.CANCELED)
                    self._remove_from_queue(iid)
            self._refresh_order_status(order_id)
            self._save()
            return True

    # ── 큐 일시정지(보류) 플래그 — 관리자가 토글, 백엔드 tick이 존중 ──
    def is_queue_paused(self) -> bool:
        with self._lock:
            return bool(self._data.get('queue_paused', False))

    def set_queue_paused(self, paused: bool) -> None:
        with self._lock:
            self._data['queue_paused'] = bool(paused)
            self._save()

    # ── 락커 (QR 수령) ──────────────────────────────────────────
    def list_lockers(self) -> list[dict]:
        """[{id, status, order_id, ticket_no, qr_token, assigned_at, ready_at}, ...] (id 오름차순)."""
        with self._lock:
            out = []
            for i in range(1, LOCKER_COUNT + 1):
                lk = dict(self._data['lockers'].get(str(i), {}))
                lk['id'] = i
                oid = lk.get('order_id', '')
                od = self._data['orders'].get(oid) if oid else None
                lk['ticket_no'] = od.get('ticket_no', '') if od else ''
                out.append(lk)
            return out

    def free_locker_count(self) -> int:
        with self._lock:
            return sum(1 for v in self._data['lockers'].values()
                       if v.get('status') == LockerStatus.FREE)

    def assign_locker(self, order_id: str, code: str | None = None) -> dict | None:
        """주문에 빈 락커 1칸 배정 + 수령 코드 발급. 만석/주문없음이면 None.
        code(4자리 숫자)를 주면 유저 지정 코드 사용, 없으면 자동 생성.
        code 형식 오류/중복이면 ValueError. 반환: {locker_id, qr_token}."""
        with self._lock:
            order = self._data['orders'].get(order_id)
            if order is None:
                return None
            if int(order.get('locker_id', 0) or 0) > 0:
                # 이미 배정됨 — 멱등 반환.
                return {'locker_id': order['locker_id'], 'qr_token': order.get('qr_token', '')}
            # 현재 점유 중 락커의 코드(중복 회피용).
            used = {self._data['lockers'][str(j)].get('qr_token')
                    for j in range(1, LOCKER_COUNT + 1)
                    if self._data['lockers'][str(j)].get('status') != LockerStatus.FREE}
            if code is not None:
                code = str(code).strip()
                if not (code.isdigit() and len(code) == 4):
                    raise ValueError('수령 코드는 4자리 숫자여야 합니다')
                if code in used:
                    raise ValueError('이미 사용 중인 코드입니다 — 다른 코드를 입력하세요')
                token = code
            else:
                token = f'{random.randint(0, 9999):04d}'
                for _ in range(100):
                    if token not in used:
                        break
                    token = f'{random.randint(0, 9999):04d}'
            for i in range(1, LOCKER_COUNT + 1):
                lk = self._data['lockers'].get(str(i))
                if lk and lk.get('status') == LockerStatus.FREE:
                    lk.update({'status': LockerStatus.OCCUPIED, 'order_id': order_id,
                               'qr_token': token, 'assigned_at': self._now(), 'ready_at': ''})
                    order['locker_id'] = i
                    order['qr_token'] = token
                    self._save()
                    return {'locker_id': i, 'qr_token': token}
            return None   # 만석

    def locker_info_by_token(self, token: str) -> dict | None:
        """수령 QR 조회. {locker_id, status, order_id, ticket_no, order_status, ready} 또는 None."""
        token = (token or '').strip()
        if not token:
            return None
        with self._lock:
            for i in range(1, LOCKER_COUNT + 1):
                lk = self._data['lockers'].get(str(i), {})
                if lk.get('qr_token') == token:
                    oid = lk.get('order_id', '')
                    od = self._data['orders'].get(oid, {})
                    return {
                        'locker_id': i,
                        'status': lk.get('status'),
                        'order_id': oid,
                        'ticket_no': od.get('ticket_no', ''),
                        'order_status': od.get('status', ''),
                        'ready': lk.get('status') == LockerStatus.READY,
                    }
            return None

    def confirm_pickup(self, token: str) -> dict | None:
        """유저 수령 완료. ready 상태일 때만 처리 — order PICKED, 락커 free. 결과 dict 또는 None."""
        token = (token or '').strip()
        with self._lock:
            for i in range(1, LOCKER_COUNT + 1):
                lk = self._data['lockers'].get(str(i), {})
                if lk.get('qr_token') != token or not token:
                    continue
                if lk.get('status') != LockerStatus.READY:
                    return {'ok': False, 'reason': 'not_ready', 'locker_id': i,
                            'status': lk.get('status')}
                oid = lk.get('order_id', '')
                od = self._data['orders'].get(oid)
                if od is not None:
                    od['status'] = OrderStatus.PICKED
                    od['picked_at'] = self._now()
                self._free_locker_locked(i)
                self._save()
                return {'ok': True, 'locker_id': i, 'order_id': oid}
            return None   # 토큰 매칭 없음

    def release_locker(self, locker_id: int) -> bool:
        """관리자 강제 해제 — 락커 free + 주문 바인딩 해제(주문 상태는 그대로)."""
        with self._lock:
            if str(locker_id) not in self._data['lockers']:
                return False
            self._free_locker_locked(int(locker_id))
            self._save()
            return True

    def reset_lockers(self) -> None:
        """관리자 전체 초기화 — 모든 락커 free."""
        with self._lock:
            for i in range(1, LOCKER_COUNT + 1):
                self._free_locker_locked(i)
            self._save()

    def _free_locker_locked(self, locker_id: int) -> None:
        self._data['lockers'][str(locker_id)] = {
            'status': LockerStatus.FREE, 'order_id': '', 'qr_token': '',
            'assigned_at': '', 'ready_at': ''}

    def _sync_locker_to_order_locked(self, order: dict) -> None:
        """주문 상태 변화에 락커 동기화 — DONE→ready, CANCELED→free. (락 보유 중 호출)"""
        lid = int(order.get('locker_id', 0) or 0)
        if lid <= 0:
            return
        lk = self._data['lockers'].get(str(lid))
        if lk is None or lk.get('order_id') != order['order_id']:
            return   # 이 락커가 다른 주문 소유면 건드리지 않음
        if order['status'] == OrderStatus.DONE:
            lk['status'] = LockerStatus.READY
            lk['ready_at'] = self._now()
        elif order['status'] == OrderStatus.CANCELED:
            self._free_locker_locked(lid)

    # ── item / queue ──
    def get_item(self, item_id: str) -> OrderItem | None:
        with self._lock:
            d = self._data['items'].get(item_id)
            return OrderItem(**d) if d else None

    def get_queue(self) -> list[OrderItem]:
        with self._lock:
            out = []
            for iid in self._data['queue']:
                d = self._data['items'].get(iid)
                if d and d['status'] in _ACTIVE_ITEM:
                    out.append(OrderItem(**d))
            return out

    def next_queued_item(self) -> OrderItem | None:
        with self._lock:
            for iid in self._data['queue']:
                d = self._data['items'].get(iid)
                if d and d['status'] == ItemStatus.QUEUED:
                    return OrderItem(**d)
            return None

    def set_item_status(self, item_id: str, status: str) -> None:
        with self._lock:
            it = self._data['items'].get(item_id)
            if it is None:
                return
            it['status'] = status
            # 종료 상태면 큐에서 빼고 stock·history 반영.
            if status == ItemStatus.DONE:
                self._adjust_stock_locked(it['class_name'], -1)
                self._append_history_locked(it, status)
                self._remove_from_queue(item_id)
            elif status in (ItemStatus.FAILED, ItemStatus.CANCELED):
                self._append_history_locked(it, status)
                self._remove_from_queue(item_id)
            self._refresh_order_status(it['order_id'])
            self._save()

    def cancel_item(self, item_id: str) -> None:
        self.set_item_status(item_id, ItemStatus.CANCELED)

    def reorder_item(self, item_id: str, new_index: int) -> None:
        with self._lock:
            q = self._data['queue']
            if item_id not in q:
                return
            q.remove(item_id)
            new_index = max(0, min(new_index, len(q)))
            q.insert(new_index, item_id)
            self._save()

    # ── history ──
    def list_history(self) -> list[dict]:
        with self._lock:
            return list(self._data['history'])

    # ── 내부 헬퍼 (락 보유 중 호출) ──
    def _remove_from_queue(self, item_id: str) -> None:
        if item_id in self._data['queue']:
            self._data['queue'].remove(item_id)

    def _adjust_stock_locked(self, class_name: str, delta: int) -> None:
        d = self._data['catalog'].get(class_name)
        if d is not None:
            d['stock'] = max(0, int(d.get('stock', 0)) + delta)

    def _append_history_locked(self, item: dict, status: str) -> None:
        self._data['history'].append({
            'item_id': item['item_id'],
            'order_id': item['order_id'],
            'class_name': item['class_name'],
            'status': status,
            'at': self._now(),
        })

    def _refresh_order_status(self, order_id: str) -> None:
        order = self._data['orders'].get(order_id)
        if order is None:
            return
        if order['status'] == OrderStatus.PICKED:
            return   # 수령 완료는 단말 상태 — item 재계산이 덮어쓰지 않도록.
        statuses = [self._data['items'][i]['status']
                    for i in order['item_ids'] if i in self._data['items']]
        if not statuses:
            return
        if any(s == ItemStatus.RUNNING for s in statuses):
            order['status'] = OrderStatus.RUNNING
        elif all(s in (ItemStatus.DONE, ItemStatus.FAILED, ItemStatus.CANCELED) for s in statuses):
            # 전부 종료 → 하나라도 DONE 있으면 DONE, 아니면 CANCELED.
            order['status'] = (OrderStatus.DONE
                               if any(s == ItemStatus.DONE for s in statuses)
                               else OrderStatus.CANCELED)
            self._sync_locker_to_order_locked(order)   # DONE→락커 ready, CANCELED→free
        else:
            order['status'] = OrderStatus.QUEUED


# ─── 하이브리드 구현 (영속 catalog/pick_stats/history = SQLite, 휘발 orders/queue/lockers = JSON) ───
class HybridRepository(JsonRepository):
    """영속 데이터(재고/통계/이력)는 SQLite, 휘발(주문/큐/락커)은 JSON.
    JsonRepository 상속 — 휘발 로직 그대로 쓰고 영속 메서드만 SQLite로 override.
    영속=SQLite 이유: 동시쓰기 안전·집계·누적 보존. 휘발=JSON 이유: 빈번갱신·세션성·가벼움."""
    _PERSIST_DB = Path.home() / '.config' / 'dsr_realsense_pick_place' / 'store.db'

    def __init__(self, path: Path | str = _DEFAULT_DB_PATH, db_path: Path | str | None = None):
        super().__init__(path)                       # 휘발(JSON) 초기화
        self._db_path = Path(db_path) if db_path else self._PERSIST_DB
        self._db_path.parent.mkdir(parents=True, exist_ok=True)
        self._db = sqlite3.connect(str(self._db_path), check_same_thread=False, timeout=5.0)
        self._db.row_factory = sqlite3.Row
        self._db.execute('PRAGMA journal_mode=WAL;')      # 동시 접근(web_kiosk+web_control)
        self._db.execute('PRAGMA busy_timeout=5000;')
        self._init_persist_schema()

    def _init_persist_schema(self) -> None:
        c = self._db
        c.execute('''CREATE TABLE IF NOT EXISTS catalog(
            class_name TEXT PRIMARY KEY, display_name TEXT,
            default_grip INTEGER DEFAULT 200, stock INTEGER DEFAULT 0)''')
        c.execute('''CREATE TABLE IF NOT EXISTS pick_stats(
            class_name TEXT PRIMARY KEY, success INTEGER DEFAULT 0, fail INTEGER DEFAULT 0,
            last_fail_reason TEXT DEFAULT '', last_fail_at TEXT DEFAULT '')''')
        c.execute('''CREATE TABLE IF NOT EXISTS history(
            id INTEGER PRIMARY KEY AUTOINCREMENT, ticket_no TEXT, order_id TEXT,
            class_name TEXT, status TEXT, at TEXT)''')
        c.execute('CREATE INDEX IF NOT EXISTS idx_history_at ON history(at)')  # 시간대별 통계 확장용
        c.commit()

    # ── catalog (SQLite) — box는 판매/재고 대상 아니므로 제외 ──
    def seed_catalog(self, items: list[CatalogItem]) -> None:
        with self._lock:
            for it in items:
                if it.class_name == 'box':
                    continue
                self._db.execute(
                    'INSERT OR IGNORE INTO catalog(class_name,display_name,default_grip,stock) '
                    'VALUES(?,?,?,?)',
                    (it.class_name, it.display_name, it.default_grip, it.stock))
            self._db.commit()

    def list_catalog(self) -> list[CatalogItem]:
        with self._lock:
            return [CatalogItem(r['class_name'], r['display_name'], r['default_grip'], r['stock'])
                    for r in self._db.execute('SELECT * FROM catalog ORDER BY class_name')]

    def get_catalog_item(self, class_name: str) -> CatalogItem | None:
        with self._lock:
            r = self._db.execute('SELECT * FROM catalog WHERE class_name=?', (class_name,)).fetchone()
            return CatalogItem(r['class_name'], r['display_name'],
                               r['default_grip'], r['stock']) if r else None

    def adjust_stock(self, class_name: str, delta: int) -> None:
        with self._lock:
            self._db.execute('UPDATE catalog SET stock=MAX(0,stock+?) WHERE class_name=?',
                             (int(delta), class_name))
            self._db.commit()

    def set_stock(self, class_name: str, qty: int) -> None:
        with self._lock:
            self._db.execute('UPDATE catalog SET stock=MAX(0,?) WHERE class_name=?',
                             (int(qty), class_name))
            self._db.commit()

    def _adjust_stock_locked(self, class_name: str, delta: int) -> None:
        # super가 _lock(RLock) 보유 중 호출 — SQLite는 자체 락이라 중첩 안전.
        self._db.execute('UPDATE catalog SET stock=MAX(0,stock+?) WHERE class_name=?',
                         (int(delta), class_name))
        self._db.commit()

    # ── pick 통계 (SQLite) ──
    def record_pick_result(self, class_name: str, success: bool, reason: str = '') -> None:
        with self._lock:
            self._db.execute('INSERT OR IGNORE INTO pick_stats(class_name) VALUES(?)', (class_name,))
            if success:
                self._db.execute('UPDATE pick_stats SET success=success+1 WHERE class_name=?',
                                 (class_name,))
            else:
                self._db.execute('UPDATE pick_stats SET fail=fail+1,last_fail_reason=?,'
                                 'last_fail_at=? WHERE class_name=?',
                                 (reason, self._now(), class_name))
            self._db.commit()

    def get_pick_stats(self) -> list[dict]:
        with self._lock:
            return [dict(r) for r in self._db.execute('SELECT * FROM pick_stats ORDER BY class_name')]

    # ── 처리 이력 (SQLite) — ticket_no(번호표) 함께 저장 ──
    def _append_history_locked(self, item: dict, status: str) -> None:
        # super가 _lock 보유 중 호출. ticket_no는 JSON orders에서 조회.
        oid = item.get('order_id', '')
        ticket = self._data['orders'].get(oid, {}).get('ticket_no', '')
        self._db.execute(
            'INSERT INTO history(ticket_no,order_id,class_name,status,at) VALUES(?,?,?,?,?)',
            (ticket, oid, item.get('class_name', ''), status, self._now()))
        self._db.commit()

    def list_history(self) -> list[dict]:
        with self._lock:
            return [dict(r) for r in self._db.execute(
                'SELECT ticket_no,order_id,class_name,status,at FROM history ORDER BY id')]


# ─── 스모크 테스트 ──────────────────────────────────────────
if __name__ == '__main__':
    import tempfile, os

    tmp = Path(tempfile.mkdtemp()) / 'db.json'
    repo = JsonRepository(tmp)

    # catalog seed
    repo.seed_catalog([
        CatalogItem('can', '캔', 200, stock=5),
        CatalogItem('ramen', '라면', 150, stock=3),
    ])
    assert {c.class_name for c in repo.list_catalog()} == {'can', 'ramen'}
    assert repo.get_catalog_item('can').stock == 5

    # 주문: 캔2 + 라면1 → item 3개 펼침
    order = repo.create_order([('can', 2), ('ramen', 1)])
    assert order.ticket_no == 'A-001'
    assert len(order.item_ids) == 3
    q = repo.get_queue()
    assert [i.class_name for i in q] == ['can', 'can', 'ramen']

    # 재정렬: 라면을 맨 앞으로
    repo.reorder_item(q[2].item_id, 0)
    assert [i.class_name for i in repo.get_queue()] == ['ramen', 'can', 'can']

    # 실행: 첫 item RUNNING → DONE (stock 차감)
    first = repo.next_queued_item()
    assert first.class_name == 'ramen'
    repo.set_item_status(first.item_id, ItemStatus.RUNNING)
    assert repo.get_order(order.order_id).status == OrderStatus.RUNNING
    repo.set_item_status(first.item_id, ItemStatus.DONE)
    assert repo.get_catalog_item('ramen').stock == 2
    assert len(repo.get_queue()) == 2

    # item 1개 취소
    rest = repo.get_queue()
    repo.cancel_item(rest[0].item_id)
    assert len(repo.get_queue()) == 1

    # order 취소 → 남은 item CANCELED
    repo.cancel_order(order.order_id)
    assert repo.get_queue() == []
    assert repo.get_order(order.order_id).status == OrderStatus.DONE  # DONE 1개 있어 DONE

    # 영속 왕복: 새 인스턴스로 다시 로드
    repo2 = JsonRepository(tmp)
    assert repo2.get_order(order.order_id).ticket_no == 'A-001'
    assert len(repo2.list_history()) == 3  # done + canceled + canceled

    os.remove(tmp)
    print('OK — task_repository 스모크 테스트 통과')
