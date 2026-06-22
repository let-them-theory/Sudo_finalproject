# 키오스크 웹 백엔드 — FastAPI + rclpy. 주문 큐 처리 + pick_place 연동(User_gui_node 로직 이전).
from __future__ import annotations

import os
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
from fastapi.responses import HTMLResponse
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
# 성공(파지+이송) 판정 — MOVE_TO_PLACE 도달 = LIFT 파지판정 통과 후 이송 시작 = 진짜 파지 성공.
# LIFT는 파지판정 단계라(여기서 실패하면 HOME行) 성공으로 보면 false-DONE 발생 → 제외.
GRASPED_STATES = {'MOVE_TO_PLACE', 'PLACE', 'POST_PLACE'}
INJECT_REPLY_TIMEOUT = 5.0   # run_once 응답 대기 한계(s). 초과 시 큐 영구정지 방지 복구.
MAX_RETRIES = 3              # 픽 실패 시 자동 재시도 최대 횟수.
MAX_ZONES = int(os.environ.get('KIOSK_MAX_ZONES', '6'))  # 배치 가능 zone 수.


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
        # user 주문은 항상 package 영역으로 place — sort(box zone)와 분리된 전용 서비스.
        self.cli_run_once = self.create_client(Trigger, '/pick_place/run_once_package')
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
        self._inflight_item_id = None      # 응답 대기 중 item (워치독 복구용)
        self._inject_sent_t = 0.0          # run_once 보낸 시각
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
            # 워치독 — run_once 응답이 끝내 안 오면(서비스 행/사망) 큐가 영구 정지한다.
            # 타임아웃 초과 시 복구: item 되돌리고 백오프 후 재시도.
            if time.monotonic() - self._inject_sent_t > INJECT_REPLY_TIMEOUT:
                self.get_logger().warn('run_once 응답 타임아웃 → 큐 복구(재시도 예약)')
                if self._inflight_item_id:
                    self.repo.set_item_status(self._inflight_item_id, ItemStatus.QUEUED)
                self._inflight_item_id = None
                self._inject_inflight = False
                self._inject_cooldown_until = time.monotonic() + 2.0
            return
        if self._injected_item_id is not None:
            if state in GRASPED_STATES:
                self._item_grasped = True
            elif state == 'IDLE':
                done = self._item_grasped
                iid = self._injected_item_id
                self._injected_item_id = None
                self._item_grasped = False
                _it = self.repo.get_item(iid)
                _class_name = _it.class_name if _it else ''
                _order_id = _it.order_id if _it else ''
                if done:
                    self.repo.set_item_status(iid, ItemStatus.DONE)
                    self.repo.record_pick_result(_class_name, True)
                    zone_id = self.repo.occupy_next_zone(_class_name, _order_id, MAX_ZONES)
                    if zone_id is not None:
                        self.get_logger().info(f'item {iid} 완료(DONE) → zone {zone_id}')
                    else:
                        self.get_logger().warn(f'item {iid} 완료 — zone 전체 점유(고객 수령 대기)')
                    self._emit('zones', {'zones': self.repo.get_zones(MAX_ZONES)})
                    _order = self.repo.get_order(_order_id)
                    if _order and _order.status == OrderStatus.DONE:
                        self._emit('order_done', {
                            'order_id': _order.order_id,
                            'ticket_no': _order.ticket_no,
                            'pickup_code': _order.pickup_code,
                        })
                else:
                    retried = self.repo.retry_or_fail(iid, MAX_RETRIES)
                    if retried:
                        it = self.repo.get_item(iid)
                        cnt = it.retry_count if it else '?'
                        self.get_logger().warn(
                            f'item {iid} 픽 실패 → 재시도 {cnt}/{MAX_RETRIES} '
                            f'— 사유: {self.last_error_text or "미상"}')
                    else:
                        self.repo.record_pick_result(
                            _class_name, False, self.last_error_text or '미상')
                        self.get_logger().error(
                            f'item {iid} FAILED(재시도 {MAX_RETRIES}회 초과) '
                            f'— 사유: {self.last_error_text or "미상"}')
                self._emit_queue_if_changed()
            return
        if state != 'IDLE':
            return
        if qp:                             # 관리자 보류 중 → 새 item 투입 안 함
            return
        if self.repo.is_all_zones_full(MAX_ZONES):
            return                         # zone 전체 점유 → 고객 수령 대기
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
        self._inflight_item_id = item.item_id
        self._inject_sent_t = time.monotonic()
        self.repo.set_item_status(item.item_id, ItemStatus.RUNNING)
        label = String(); label.data = item.class_name
        self.pub_selected.publish(label)
        future = self.cli_run_once.call_async(Trigger.Request())

        def _done(fut, iid=item.item_id):
            self._inject_inflight = False
            self._inflight_item_id = None
            try:
                res = fut.result()
            except Exception as e:
                self.get_logger().error(f'run_once 호출 실패: {e}')
                self.repo.set_item_status(iid, ItemStatus.QUEUED)
                self._inject_cooldown_until = time.monotonic() + 2.0   # 예외 시도 폭주 방지
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
                        'retry_count': it.retry_count,
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


def _node():
    # ROS 노드 초기화 전(lifespan 시작 틈)엔 503 — None 역참조 크래시 방지.
    if node is None:
        raise HTTPException(503, '서버 준비 중 — 잠시 후 다시 시도하세요')
    return node


@app.get('/api/catalog')
def get_catalog():
    n = _node()
    avail = n.detected_classes
    return [{'class_name': c.class_name, 'display_name': c.display_name,
             'stock': c.stock,
             'in_stock': c.stock > 0,
             'available': c.class_name in avail}
            for c in n.repo.list_catalog()]


@app.get('/api/orders')
def list_orders(status: str | None = None):
    n = _node()
    statuses = {status} if status else None
    orders = n.repo.list_orders(statuses)
    orders.sort(key=lambda o: o.order_id, reverse=True)  # 최신순
    return [{'order_id': o.order_id, 'ticket_no': o.ticket_no,
             'status': o.status, 'pickup_code': o.pickup_code,
             'created_at': o.created_at, 'item_count': len(o.item_ids)}
            for o in orders]


@app.post('/api/orders')
def create_order(body: OrderBody):
    n = _node()
    if not body.lines:
        raise HTTPException(400, '빈 주문')
    try:
        order = n.submit_order(body.lines)
    except ValueError as e:
        raise HTTPException(400, str(e))
    return {'order_id': order.order_id, 'ticket_no': order.ticket_no,
            'pickup_code': order.pickup_code}


@app.get('/api/orders/{order_id}')
def get_order(order_id: str):
    n = _node()
    order = n.repo.get_order(order_id)
    if order is None:
        raise HTTPException(404, '주문 없음')
    items = [n.repo.get_item(i) for i in order.item_ids]
    return {
        'order_id': order.order_id, 'ticket_no': order.ticket_no,
        'status': order.status,
        'pickup_code': order.pickup_code,
        'items': [{'item_id': it.item_id, 'class_name': it.class_name,
                   'status': it.status, 'retry_count': it.retry_count}
                  for it in items if it],
    }


@app.post('/api/orders/{order_id}/cancel')
def cancel_order(order_id: str):
    n = _node()
    if n.repo.get_order(order_id) is None:
        raise HTTPException(404, '주문 없음')
    n.cancel_order(order_id)
    return {'ok': True}


@app.get('/api/queue')
def get_queue():
    return _node()._queue_dump()


@app.get('/api/state')
def get_state():
    n = _node()
    return {'state': n.pick_place_state,
            'available': sorted(n.detected_classes),
            'error': n.last_error_text}


@app.get('/api/stats')
def get_stats():
    return _node().repo.get_pick_stats()


@app.get('/api/history')
def get_history():
    return list(reversed(_node().repo.list_history()))  # 최신순


@app.get('/api/zones')
def get_zones():
    return _node().repo.get_zones(MAX_ZONES)


@app.post('/api/zones/{zone_id}/clear')
def clear_zone(zone_id: int):
    if zone_id < 0 or zone_id >= MAX_ZONES:
        raise HTTPException(400, f'zone_id는 0~{MAX_ZONES - 1} 범위여야 합니다')
    n = _node()
    n.repo.clear_zone(zone_id)
    n._emit('zones', {'zones': n.repo.get_zones(MAX_ZONES)})
    return {'ok': True}


class StockBody(BaseModel):
    stock: int


@app.put('/api/admin/stock/{class_name}')
def set_stock(class_name: str, body: StockBody):
    n = _node()
    if n.repo.get_catalog_item(class_name) is None:
        raise HTTPException(404, f"품목 '{class_name}' 없음")
    if body.stock < 0:
        raise HTTPException(400, '재고는 0 이상이어야 합니다')
    n.repo.set_stock(class_name, body.stock)
    return {'ok': True, 'class_name': class_name, 'stock': body.stock}


_ADMIN_HTML = """<!doctype html>
<html lang="ko">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>관리자 패널</title>
<style>
  * { box-sizing: border-box; }
  body { font-family: -apple-system, sans-serif; margin: 0; background: #f0f2f5; color: #222; }
  header { background: #1a1a2e; color: #fff; padding: 16px 24px; display: flex; align-items: center; gap: 16px; }
  header h1 { margin: 0; font-size: 20px; flex: 1; }
  header span { font-size: 12px; color: #aaa; }
  main { max-width: 1200px; margin: 24px auto; padding: 0 16px; }
  section { background: #fff; border-radius: 10px; padding: 20px 24px; margin-bottom: 24px;
            box-shadow: 0 1px 4px rgba(0,0,0,.08); }
  h2 { margin: 0 0 16px; font-size: 15px; color: #444; border-bottom: 2px solid #e0e7ff; padding-bottom: 8px; }
  table { border-collapse: collapse; width: 100%; font-size: 13px; }
  th, td { padding: 7px 11px; border-bottom: 1px solid #eee; text-align: left; white-space: nowrap; }
  th { background: #f7f8fc; font-weight: 600; color: #555; }
  tr:last-child td { border-bottom: none; }
  tr:hover td { background: #fafbff; }
  .zone-grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(130px, 1fr)); gap: 12px; }
  .zone { border-radius: 8px; padding: 14px 10px; text-align: center; font-size: 13px; }
  .zone.free { background: #e8f5e9; border: 1px solid #a5d6a7; }
  .zone.occupied { background: #fde8e8; border: 1px solid #ef9a9a; }
  .zone b { display: block; margin-bottom: 4px; font-size: 14px; }
  .zone small { color: #666; line-height: 1.5; display: block; }
  .btn { padding: 3px 10px; border: none; border-radius: 5px; cursor: pointer; font-size: 12px; margin-top: 6px; }
  .btn-clear { background: #e53935; color: #fff; }
  .btn-save  { background: #43a047; color: #fff; }
  input[type=number] { width: 68px; padding: 3px 6px; border: 1px solid #ccc; border-radius: 4px; font-size: 13px; }
  .chip { display: inline-block; padding: 2px 8px; border-radius: 10px; font-size: 11px; font-weight: 700; }
  .chip-queued   { background: #e3f2fd; color: #1565c0; }
  .chip-running  { background: #fff3e0; color: #e65100; }
  .chip-done     { background: #e8f5e9; color: #2e7d32; }
  .chip-failed   { background: #fce4ec; color: #c62828; }
  .chip-canceled { background: #f5f5f5; color: #757575; }
  .chip-stock-ok { background: #e8f5e9; color: #2e7d32; }
  .chip-stock-no { background: #fafafa; color: #999; border: 1px solid #eee; }
  .retry-warn { color: #e65100; font-weight: 700; }
  .code { font-family: monospace; font-size: 13px; background: #f5f5f5; padding: 1px 5px; border-radius: 3px; }
  .empty { text-align: center; color: #aaa; padding: 20px !important; }
</style>
</head>
<body>
<header>
  <h1>관리자 패널</h1>
  <span id="ts"></span>
</header>
<main>

  <section>
    <h2>재고 관리</h2>
    <table>
      <thead><tr><th>품목</th><th>표시명</th><th>현재 재고</th><th>감지됨</th><th>재고 설정</th></tr></thead>
      <tbody id="stock-body"></tbody>
    </table>
  </section>

  <section>
    <h2>현재 대기 큐</h2>
    <table>
      <thead><tr><th>번호표</th><th>품목</th><th>상태</th><th>재시도</th></tr></thead>
      <tbody id="queue-body"></tbody>
    </table>
  </section>

  <section>
    <h2>주문 현황</h2>
    <table>
      <thead><tr><th>번호표</th><th>상태</th><th>픽업코드</th><th>품목 수</th><th>생성 시각</th></tr></thead>
      <tbody id="orders-body"></tbody>
    </table>
  </section>

  <section>
    <h2>Zone 점유 현황 (총 """ + str(MAX_ZONES) + """개)</h2>
    <div id="zone-grid" class="zone-grid"></div>
  </section>

  <section>
    <h2>픽 통계</h2>
    <table>
      <thead><tr><th>품목</th><th>성공</th><th>실패</th><th>마지막 실패 사유</th><th>마지막 실패 시각</th></tr></thead>
      <tbody id="stats-body"></tbody>
    </table>
  </section>

  <section>
    <h2>처리 이력 (최신순)</h2>
    <table>
      <thead><tr><th>품목</th><th>결과</th><th>주문 ID</th><th>처리 시각</th></tr></thead>
      <tbody id="history-body"></tbody>
    </table>
  </section>

</main>
<script>
const STATUS_CHIP = {
  QUEUED:   '<span class="chip chip-queued">대기</span>',
  RUNNING:  '<span class="chip chip-running">진행중</span>',
  DONE:     '<span class="chip chip-done">완료</span>',
  FAILED:   '<span class="chip chip-failed">실패</span>',
  CANCELED: '<span class="chip chip-canceled">취소</span>',
};
function chip(s) { return STATUS_CHIP[s] || `<span class="chip">${s}</span>`; }
function dt(s) { return s ? s.slice(0, 19).replace('T', ' ') : '—'; }

async function load() {
  const [catalog, queue, orders, zones, stats, history] = await Promise.all([
    fetch('/api/catalog').then(r => r.json()),
    fetch('/api/queue').then(r => r.json()),
    fetch('/api/orders').then(r => r.json()),
    fetch('/api/zones').then(r => r.json()),
    fetch('/api/stats').then(r => r.json()),
    fetch('/api/history').then(r => r.json()),
  ]);

  // ── 재고
  document.getElementById('stock-body').innerHTML = catalog.map(c => `
    <tr>
      <td><span class="code">${c.class_name}</span></td>
      <td>${c.display_name}</td>
      <td>${c.stock > 0
        ? `<span class="chip chip-stock-ok">${c.stock}</span>`
        : `<span class="chip chip-stock-no">0</span>`}</td>
      <td>${c.available ? '✅' : '—'}</td>
      <td>
        <input type="number" id="s_${c.class_name}" value="${c.stock}" min="0">
        <button class="btn btn-save" onclick="setStock('${c.class_name}')">저장</button>
      </td>
    </tr>`).join('');

  // ── 큐
  document.getElementById('queue-body').innerHTML = queue.length === 0
    ? `<tr><td colspan="4" class="empty">대기 중인 항목 없음</td></tr>`
    : queue.map(q => `
      <tr>
        <td>${q.ticket_no}</td>
        <td><span class="code">${q.class_name}</span></td>
        <td>${chip(q.status)}</td>
        <td>${q.retry_count > 0
          ? `<span class="retry-warn">${q.retry_count}회</span>`
          : '—'}</td>
      </tr>`).join('');

  // ── 주문
  document.getElementById('orders-body').innerHTML = orders.length === 0
    ? `<tr><td colspan="5" class="empty">주문 없음</td></tr>`
    : orders.map(o => `
      <tr>
        <td><b>${o.ticket_no}</b></td>
        <td>${chip(o.status)}</td>
        <td>${o.pickup_code ? `<span class="code">${o.pickup_code}</span>` : '—'}</td>
        <td>${o.item_count}</td>
        <td>${dt(o.created_at)}</td>
      </tr>`).join('');

  // ── Zone
  document.getElementById('zone-grid').innerHTML = zones.map(z => `
    <div class="zone ${z.occupied ? 'occupied' : 'free'}">
      <b>Zone ${z.zone_id}</b>
      ${z.occupied
        ? `<small><span class="code">${z.class_name}</span><br>${(z.placed_at||'').slice(11,19)}</small>
           <button class="btn btn-clear" onclick="clearZone(${z.zone_id})">비우기</button>`
        : '<small>비어 있음</small>'}
    </div>`).join('');

  // ── 픽 통계
  document.getElementById('stats-body').innerHTML = stats.length === 0
    ? `<tr><td colspan="5" class="empty">아직 픽 기록 없음</td></tr>`
    : stats.map(s => `
      <tr>
        <td><span class="code">${s.class_name}</span></td>
        <td>${s.success}</td>
        <td>${s.fail > 0 ? `<span class="retry-warn">${s.fail}</span>` : '0'}</td>
        <td>${s.last_fail_reason || '—'}</td>
        <td>${dt(s.last_fail_at)}</td>
      </tr>`).join('');

  // ── 이력
  document.getElementById('history-body').innerHTML = history.length === 0
    ? `<tr><td colspan="4" class="empty">처리 이력 없음</td></tr>`
    : history.slice(0, 100).map(h => `
      <tr>
        <td><span class="code">${h.class_name}</span></td>
        <td>${chip(h.status)}</td>
        <td><span class="code">${h.order_id}</span></td>
        <td>${dt(h.at)}</td>
      </tr>`).join('');

  document.getElementById('ts').textContent = '갱신: ' + new Date().toLocaleTimeString();
}

async function setStock(cn) {
  const val = parseInt(document.getElementById('s_' + cn).value, 10);
  if (isNaN(val) || val < 0) { alert('0 이상의 숫자를 입력하세요'); return; }
  const res = await fetch('/api/admin/stock/' + cn, {
    method: 'PUT', headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({stock: val}),
  });
  if (!res.ok) { alert('저장 실패'); return; }
  load();
}

async function clearZone(id) {
  await fetch('/api/zones/' + id + '/clear', {method: 'POST'});
  load();
}

load();
setInterval(load, 5000);
</script>
</body>
</html>"""


@app.get('/admin', response_class=HTMLResponse)
def admin_page():
    return _ADMIN_HTML


@app.websocket('/ws')
async def ws_endpoint(ws: WebSocket):
    await ws.accept()
    if node is None:
        await ws.close(code=1013)   # try again later
        return
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
