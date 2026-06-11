# 키오스크 웹 백엔드 — FastAPI + rclpy. 주문 큐 처리 + pick_place 연동(User_gui_node 로직 이전).
from __future__ import annotations

import threading
import time
import json
import asyncio
from contextlib import asynccontextmanager

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

from dsr_realsense_pick_place.task_repository import (
    JsonRepository, CatalogItem, ItemStatus, OrderStatus,
)

# 상품 표시명 (class → 한글). 프론트도 자체 매핑 가능하지만 카탈로그 seed에 필요.
PRODUCT_DISPLAY = {
    'ramen': '라면', 'pack': '팩음료', 'ssnack': '스낵', 'bsnack': '봉지과자',
    'water': '생수', 'jelly': '젤리', 'box': '박스', 'can': '캔',
    'boxsnack': '박스과자', 'wafers': '웨하스',
}
KNOWN_CLASSES = list(PRODUCT_DISPLAY.keys())

ABNORMAL_STATES = {'ERROR', 'EMERGENCY_STOP'}
GRASPED_STATES = {'LIFT', 'MOVE_TO_PLACE', 'PLACE', 'POST_PLACE'}


class KioskBackend(Node):
    """주문 큐를 들고 pick_place에 순차 투입. User_gui_node의 헤드리스 버전."""

    def __init__(self, on_event=None):
        super().__init__('kiosk_backend')
        self._on_event = on_event   # 상태 변화 → WebSocket 브로드캐스트 콜백
        self.repo = JsonRepository()
        self.repo.seed_catalog([
            CatalogItem(c, PRODUCT_DISPLAY.get(c, c)) for c in KNOWN_CLASSES
        ])
        # 시작 시 이전 세션 미완료(QUEUED/RUNNING) 주문 취소 — 잔여 자동 실행 방지(새 세션).
        stale = self.repo.get_queue()
        for it in stale:
            self.repo.cancel_item(it.item_id)
        if stale:
            self.get_logger().info(f'이전 세션 미완료 주문 {len(stale)}건 취소(새 세션)')

        self.pub_selected = self.create_publisher(String, '/selected_object_label', 10)
        self.cli_run_once = self.create_client(Trigger, '/pick_place/run_once')
        self.cli_cancel = self.create_client(Trigger, '/pick_place/cancel')
        self.create_subscription(String, '/detected_objects', self._cb_objects, 10)
        self.create_subscription(String, '/pick_place_state', self._cb_state, 10)
        self.create_subscription(String, '/pick_place_error', self._cb_error, 10)

        self.pick_place_state = ''
        self.detected_classes: set[str] = set()
        self.last_error_text = ''
        self.paused = False
        self._injected_item_id = None
        self._item_grasped = False
        self._inject_inflight = False
        self._inject_cooldown_until = 0.0
        self._last_queue_json = None       # WS queue 변경 감지(중복 전송 억제)
        self._last_paused = None           # WS paused 변경 감지

        self.create_timer(0.3, self.tick_queue)

    # ── 구독 콜백 ──
    def _cb_objects(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.detected_classes = {
            o.get('class_name', o.get('label', '')) for o in payload.get('objects', [])
        }
        self._emit('detected', {'classes': sorted(self.detected_classes)})

    def _cb_state(self, msg: String):
        if msg.data != self.pick_place_state:
            self.pick_place_state = msg.data
            self._emit('state', {'value': msg.data})

    def _cb_error(self, msg: String):
        self.last_error_text = msg.data
        self._emit('error', {'msg': msg.data})

    def _emit(self, kind: str, data: dict):
        if self._on_event:
            self._on_event({'type': kind, **data})

    # ── 큐 실행 루프 (timer 0.3s) ──
    def tick_queue(self):
        # 관리자 GUI 등 다른 프로세스의 DB 변경(주문 취소·보류 등) 반영.
        self.repo.reload()
        self._emit_queue_if_changed()      # 외부(관리자) 취소도 web에 즉시 반영
        state = self.pick_place_state
        qp = self.repo.is_queue_paused()   # 관리자 보류 플래그
        abnormal = state in ABNORMAL_STATES
        self.paused = abnormal             # 로봇 레벨 (정상 복귀 시 자동 해제)
        self._emit_paused_if_changed(self.paused or qp, state)

        if abnormal:
            return
        if self._inject_inflight:
            return
        if self._injected_item_id is not None:
            if state in GRASPED_STATES:
                self._item_grasped = True
            elif state == 'IDLE':
                done = self._item_grasped
                iid = self._injected_item_id
                self._injected_item_id = None
                self._item_grasped = False
                self.repo.set_item_status(
                    iid, ItemStatus.DONE if done else ItemStatus.FAILED)
                self.get_logger().info(f'item {iid} {"완료" if done else "미파지→FAILED"}')
                self._emit_queue_if_changed()
            return
        if state != 'IDLE':
            return
        if qp:                             # 관리자 보류 중 → 새 item 투입 안 함
            return
        if time.monotonic() < self._inject_cooldown_until:
            return
        nxt = self.repo.next_queued_item()
        if nxt is None:
            return
        self._inject(nxt)

    def _emit_queue_if_changed(self):
        items = self._queue_dump()
        key = json.dumps(items, sort_keys=True)
        if key != self._last_queue_json:
            self._last_queue_json = key
            self._emit('queue', {'items': items})

    def _emit_paused_if_changed(self, eff: bool, state: str):
        if eff != self._last_paused:
            self._last_paused = eff
            self._emit('paused', {'paused': eff, 'state': state})

    def _inject(self, item):
        if not self.cli_run_once.service_is_ready():
            return
        self._inject_inflight = True
        self.repo.set_item_status(item.item_id, ItemStatus.RUNNING)
        label = String(); label.data = item.class_name
        self.pub_selected.publish(label)
        future = self.cli_run_once.call_async(Trigger.Request())

        def _done(fut, iid=item.item_id):
            self._inject_inflight = False
            try:
                res = fut.result()
            except Exception as e:
                self.get_logger().error(f'run_once 호출 실패: {e}')
                self.repo.set_item_status(iid, ItemStatus.QUEUED)
                return
            if res.success:
                self._injected_item_id = iid
                self._item_grasped = False
                self.get_logger().info(f'item {iid} 투입: {res.message}')
            else:
                self.repo.set_item_status(iid, ItemStatus.QUEUED)
                self._inject_cooldown_until = time.monotonic() + 2.0
                self.get_logger().info(
                    f'run_once 거절 → 2초 후 재시도: {res.message}',
                    throttle_duration_sec=5.0)
            self._emit_queue_if_changed()

        future.add_done_callback(_done)

    def _call_cancel(self):
        if self.cli_cancel.service_is_ready():
            self.cli_cancel.call_async(Trigger.Request())

    # ── 주문 명령 (REST에서 호출) ──
    def submit_order(self, lines):
        self.repo.reload()
        order = self.repo.create_order(lines)
        self._emit_queue_if_changed()
        return order

    def cancel_order(self, order_id):
        self.repo.reload()
        # 실행 중 item이 이 주문 소속이면 Main도 취소 (User_gui_node와 동일 로직).
        order = self.repo.get_order(order_id)
        if order and self._injected_item_id in (order.item_ids if order else []):
            self._call_cancel()
            self._injected_item_id = None
            self._item_grasped = False
        self.repo.cancel_order(order_id)
        self._emit_queue_if_changed()

    def _queue_dump(self):
        # 대기보드용으로 ticket_no·주문상태도 포함 (프론트가 주문 단위로 묶어 표시).
        out = []
        for it in self.repo.get_queue():
            order = self.repo.get_order(it.order_id)
            out.append({'item_id': it.item_id, 'order_id': it.order_id,
                        'class_name': it.class_name, 'status': it.status,
                        'ticket_no': order.ticket_no if order else '',
                        'order_status': order.status if order else ''})
        return out


# ─────────────────────────────────────────────────────────────
# FastAPI
# ─────────────────────────────────────────────────────────────
node: KioskBackend | None = None
_ws_clients: set[WebSocket] = set()
_loop: asyncio.AbstractEventLoop | None = None


def _broadcast(event: dict):
    # rclpy 스레드 → asyncio 루프로 안전 전달.
    if _loop is None:
        return
    asyncio.run_coroutine_threadsafe(_push(event), _loop)


async def _push(event: dict):
    dead = []
    for ws in _ws_clients:
        try:
            await ws.send_json(event)
        except Exception:
            dead.append(ws)
    for ws in dead:
        _ws_clients.discard(ws)


@asynccontextmanager
async def lifespan(app: FastAPI):
    global node, _loop
    _loop = asyncio.get_running_loop()
    rclpy.init()
    node = KioskBackend(on_event=_broadcast)
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()
    yield
    rclpy.shutdown()


app = FastAPI(lifespan=lifespan)
app.add_middleware(CORSMiddleware, allow_origins=['*'],
                   allow_methods=['*'], allow_headers=['*'])


class OrderBody(BaseModel):
    lines: list[tuple[str, int]]   # [(class_name, qty)]


@app.get('/api/catalog')
def get_catalog():
    avail = node.detected_classes if node else set()
    return [{'class_name': c.class_name, 'display_name': c.display_name,
             'stock': c.stock, 'available': c.class_name in avail}
            for c in node.repo.list_catalog()]


@app.post('/api/orders')
def create_order(body: OrderBody):
    if not body.lines:
        raise HTTPException(400, '빈 주문')
    order = node.submit_order(body.lines)
    return {'order_id': order.order_id, 'ticket_no': order.ticket_no}


@app.get('/api/orders/{order_id}')
def get_order(order_id: str):
    order = node.repo.get_order(order_id)
    if order is None:
        raise HTTPException(404, '주문 없음')
    items = [node.repo.get_item(i) for i in order.item_ids]
    return {
        'order_id': order.order_id, 'ticket_no': order.ticket_no,
        'status': order.status,
        'items': [{'item_id': it.item_id, 'class_name': it.class_name,
                   'status': it.status} for it in items if it],
    }


@app.post('/api/orders/{order_id}/cancel')
def cancel_order(order_id: str):
    if node.repo.get_order(order_id) is None:
        raise HTTPException(404, '주문 없음')
    node.cancel_order(order_id)
    return {'ok': True}


@app.get('/api/queue')
def get_queue():
    return node._queue_dump()


@app.get('/api/state')
def get_state():
    return {'state': node.pick_place_state,
            'available': sorted(node.detected_classes),
            'error': node.last_error_text}


@app.websocket('/ws')
async def ws_endpoint(ws: WebSocket):
    await ws.accept()
    _ws_clients.add(ws)
    # 접속 즉시 현재 상태 1회 전송.
    await ws.send_json({'type': 'state', 'value': node.pick_place_state})
    await ws.send_json({'type': 'paused', 'paused': node.paused, 'state': node.pick_place_state})
    await ws.send_json({'type': 'queue', 'items': node._queue_dump()})
    try:
        while True:
            await ws.receive_text()   # 클라 핑 등 — 무시
    except WebSocketDisconnect:
        _ws_clients.discard(ws)


# 프론트 빌드물 정적 서빙 — /api·/ws 명시 라우트 뒤에 mount(catch-all)해야 충돌 없음.
# 빌드 전(dist 없음)이면 건너뛴다(개발 중엔 vite dev 서버 사용).
import os
from fastapi.staticfiles import StaticFiles


def _find_dist():
    """dist 위치 탐색 — 설치본(share) 우선, 없으면 이 파일 기준 상대경로."""
    cands = []
    try:
        from ament_index_python.packages import get_package_share_directory
        cands.append(os.path.join(
            get_package_share_directory('dsr_realsense_pick_place'),
            'web_kiosk', 'frontend', 'dist'))
    except Exception:
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    cands.append(os.path.join(here, '..', 'frontend', 'dist'))
    # index.html을 realpath로 풀어 '실제 파일이 있는' 디렉터리를 반환.
    # (colcon --symlink-install은 dist를 심볼릭링크로 깔고, StaticFiles는 디렉터리
    #  밖을 가리키는 링크를 거부해 404 → 실제 경로로 마운트해야 한다.)
    for c in cands:
        idx = os.path.join(c, 'index.html')
        if os.path.isfile(idx):
            return os.path.dirname(os.path.realpath(idx))
    return None


_DIST = _find_dist()
print(f'[kiosk] dist 경로: {_DIST}', flush=True)
if _DIST:
    app.mount('/', StaticFiles(directory=_DIST, html=True), name='spa')
else:
    print('[kiosk] ⚠ dist 없음 — 프론트 미서빙 (npm run build 필요)', flush=True)


def main():
    import uvicorn
    port = int(os.environ.get('KIOSK_PORT', '8000'))
    uvicorn.run(app, host='0.0.0.0', port=port)


if __name__ == '__main__':
    main()
