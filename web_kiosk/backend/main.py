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
from fastapi.responses import Response, HTMLResponse
from pydantic import BaseModel

from dsr_realsense_pick_place.task_repository import (
    HybridRepository, CatalogItem, ItemStatus, OrderStatus,
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


class KioskBackend(Node):
    """주문 큐를 들고 pick_place에 순차 투입. User_gui_node의 헤드리스 버전."""

    def __init__(self, on_event=None):
        super().__init__('kiosk_backend')
        self._on_event = on_event   # 상태 변화 → WebSocket 브로드캐스트 콜백
        self.repo = HybridRepository()   # 영속(재고/통계/이력)=SQLite, 휘발(주문/큐/락커)=JSON
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
        # 사이클 결과(success/dropped/failed) — 상태추론 대신 이걸로 DONE/FAILED 판정.
        self.create_subscription(String, '/pick_place/cycle_result', self._cb_cycle_result, 10)

        self.pick_place_state = ''
        self.detected_classes: set[str] = set()
        self.last_error_text = ''
        self.paused = False
        self._injected_item_id = None
        self._item_grasped = False
        self._last_cycle_result = None   # pick_place가 발행한 마지막 사이클 결과
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

    def _cb_cycle_result(self, msg: String):
        # pick_place가 사이클 끝에 발행. 'success'만 배달완료, 'dropped'/'failed'는 실패.
        self._last_cycle_result = msg.data

    def _maybe_emit_order_done(self, iid):
        # 이 item으로 주문이 종료(DONE/CANCELED)됐으면 ready/실패 알림 발행(QR 락커 + #3 실패피드백).
        it = self.repo.get_item(iid)
        if it is None:
            return
        order = self.repo.get_order(it.order_id)
        if order is None or order.status not in (OrderStatus.DONE, OrderStatus.CANCELED):
            return
        items = [self.repo.get_item(i) for i in order.item_ids]
        failed = [x.class_name for x in items if x and x.status == ItemStatus.FAILED]
        self._emit('order_done', {
            'order_id': order.order_id,
            'ticket_no': order.ticket_no,
            'status': order.status,
            'locker_id': order.locker_id,
            'failed': failed,
            'all_ok': (not failed and order.status == OrderStatus.DONE),
        })

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
                # 완료 판정: pick_place 사이클 결과가 우선('success'만 배달완료).
                # 결과 토픽 유실 시에만 기존 grasp-도달 추론으로 폴백(무회귀).
                res = self._last_cycle_result
                done = (res == 'success') if res is not None else self._item_grasped
                iid = self._injected_item_id
                self._injected_item_id = None
                self._item_grasped = False
                self._last_cycle_result = None
                self.repo.set_item_status(
                    iid, ItemStatus.DONE if done else ItemStatus.FAILED)
                # 픽 통계 기록(admin 품목별 성공/실패).
                _it = self.repo.get_item(iid)
                if _it is not None:
                    self.repo.record_pick_result(
                        _it.class_name, done,
                        '' if done else (self.last_error_text or '미상'))
                if done:
                    self.get_logger().info(f'item {iid} 완료(DONE)')
                else:
                    # DONE으로 두지 않고 실패 사유를 남긴다(결과/에러 텍스트).
                    self.get_logger().error(
                        f'item {iid} FAILED (result={res!r}) — '
                        f'사유: {self.last_error_text or "미상(파지/이송 미완)"}')
                self._emit_queue_if_changed()
                self._maybe_emit_order_done(iid)   # 주문 종료 시 ready/실패 알림(QR 락커)
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
        self._inflight_item_id = item.item_id
        self._inject_sent_t = time.monotonic()
        self._last_cycle_result = None   # 직전 사이클 결과 잔존 방지(이번 item 결과만 본다)
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
    def submit_order(self, lines, code=None):
        self.repo.reload()
        order = self.repo.create_order(lines)
        # 빈 락커 배정 + 수령 코드(유저 지정 또는 자동). 실패 시 주문 롤백 후 거절.
        try:
            assigned = self.repo.assign_locker(order.order_id, code)
        except ValueError as e:           # 코드 형식 오류/중복
            self.repo.cancel_order(order.order_id)
            self._emit_queue_if_changed()
            raise RuntimeError(str(e))
        if assigned is None:              # 만석
            self.repo.cancel_order(order.order_id)
            self._emit_queue_if_changed()
            raise RuntimeError('모든 락커가 사용 중입니다. 잠시 후 다시 시도하세요.')
        self._emit_queue_if_changed()
        return self.repo.get_order(order.order_id)   # 락커 배정 반영본

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


# 관리자 패널 HTML — 백엔드 직접 서빙(SPA 아님). 재고/큐/주문/락커/픽통계/이력.
_PANEL_CSS = """<style>
  *{box-sizing:border-box}
  body{font-family:-apple-system,'Pretendard',sans-serif;margin:0;background:#f0f2f5;color:#222}
  header{background:#1a1a2e;color:#fff;padding:16px 24px;display:flex;align-items:center;gap:16px}
  header h1{margin:0;font-size:20px;flex:1} header span{font-size:12px;color:#aaa}
  header a{color:#9fd0ff;font-size:13px;text-decoration:none;font-weight:600}
  main{max-width:1100px;margin:24px auto;padding:0 16px}
  section{background:#fff;border-radius:10px;padding:18px 22px;margin-bottom:22px;box-shadow:0 1px 4px rgba(0,0,0,.08)}
  h2{margin:0 0 14px;font-size:15px;color:#444;border-bottom:2px solid #e0e7ff;padding-bottom:8px}
  table{border-collapse:collapse;width:100%;font-size:13px}
  th,td{padding:7px 11px;border-bottom:1px solid #eee;text-align:left;white-space:nowrap}
  th{background:#f7f8fc;font-weight:600;color:#555} tr:last-child td{border-bottom:none}
  tr:hover td{background:#fafbff}
  .lk-grid{display:grid;grid-template-columns:repeat(4,1fr);gap:10px}
  .lk{border-radius:8px;padding:12px 10px;text-align:center;font-size:13px;border:1px solid #ddd}
  .lk.free{background:#f5f5f5;color:#999} .lk.occupied{background:#fff3e0;border-color:#ffcc80}
  .lk.ready{background:#e8f5e9;border-color:#a5d6a7}
  .lk b{display:block;margin-bottom:4px;font-size:14px} .lk small{display:block;color:#666;line-height:1.5}
  .code{font-family:monospace;font-weight:700;color:#d97706}
  .btn{padding:3px 10px;border:none;border-radius:5px;cursor:pointer;font-size:12px}
  .btn-save{background:#43a047;color:#fff}
  input[type=number]{width:64px;padding:3px 6px;border:1px solid #ccc;border-radius:4px;font-size:13px}
  .chip{display:inline-block;padding:2px 8px;border-radius:10px;font-size:11px;font-weight:700}
  .c-QUEUED{background:#e3f2fd;color:#1565c0} .c-RUNNING{background:#fff3e0;color:#e65100}
  .c-DONE{background:#e8f5e9;color:#2e7d32} .c-FAILED{background:#fce4ec;color:#c62828}
  .c-CANCELED{background:#f5f5f5;color:#757575} .c-PICKED{background:#ede7f6;color:#5e35b1}
  .empty{text-align:center;color:#aaa;padding:18px!important}
</style>"""

# 운영 패널(/admin) — 락커/대기큐/주문현황 (휘발 JSON, 실시간)
# 관리자 패널(/admin) — 재고/락커/주문/큐/통계/이력 통합. 영속은 SQLite, 휘발은 JSON(repo가 분기).
_ADMIN_HTML = """<!doctype html>
<html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>관리자 패널</title>""" + _PANEL_CSS + """</head><body>
<header><h1>관리자 패널</h1><span id="ts"></span></header>
<main>
  <section><h2>재고 관리 (box 제외)</h2>
    <table><thead><tr><th>품목</th><th>표시명</th><th>재고</th><th>감지</th><th>설정</th></tr></thead>
    <tbody id="stock"></tbody></table></section>
  <section><h2>락커 현황</h2><div id="lockers" class="lk-grid"></div></section>
  <section><h2>대기 큐</h2>
    <table><thead><tr><th>품목</th><th>주문</th><th>상태</th></tr></thead><tbody id="queue"></tbody></table></section>
  <section><h2>주문 현황</h2>
    <table><thead><tr><th>번호표</th><th>상태</th><th>수령코드</th><th>락커</th><th>품목수</th><th>시각</th></tr></thead>
    <tbody id="orders"></tbody></table></section>
  <section><h2>픽 통계 (누적)</h2>
    <table><thead><tr><th>품목</th><th>성공</th><th>실패</th><th>최근 실패</th></tr></thead><tbody id="stats"></tbody></table></section>
  <section><h2>처리 이력 (최신)</h2>
    <table><thead><tr><th>번호표</th><th>품목</th><th>상태</th><th>시각</th></tr></thead><tbody id="hist"></tbody></table></section>
</main>
<script>
const $=id=>document.getElementById(id);
const chip=s=>`<span class="chip c-${s}">${s}</span>`;
const LK={free:'비어있음',occupied:'배달중',ready:'수령대기'};
async function j(u){try{const r=await fetch(u);return r.ok?await r.json():null}catch{return null}}
function emptyRow(tb,cols,msg){tb.innerHTML=`<tr><td class="empty" colspan="${cols}">${msg}</td></tr>`}
async function setStock(cn){
  const v=parseInt($('s-'+cn).value);if(isNaN(v))return;
  await fetch('/api/admin/stock/'+cn,{method:'PUT',headers:{'Content-Type':'application/json'},body:JSON.stringify({stock:v})});
  load();
}
async function load(){
  $('ts').textContent=new Date().toLocaleTimeString('ko-KR');
  const [cat,lk,q,od,st,hi]=await Promise.all([
    j('/api/catalog'),j('/api/lockers'),j('/api/queue'),j('/api/orders'),j('/api/stats'),j('/api/history')]);
  const cb=$('stock');
  if(cat&&cat.length)cb.innerHTML=cat.map(c=>`<tr><td>${c.class_name}</td><td>${c.display_name}</td>
    <td>${c.stock}</td><td>${c.available?'✅':'—'}</td>
    <td><input type=number id="s-${c.class_name}" value="${c.stock}" min=0>
    <button class="btn btn-save" onclick="setStock('${c.class_name}')">저장</button></td></tr>`).join('');
  else emptyRow(cb,5,'품목 없음');
  const lb=$('lockers'); const lks=(lk&&lk.lockers)||[];
  lb.innerHTML=Array.from({length:8},(_,i)=>{
    const l=lks.find(x=>x.id===i+1)||{id:i+1,status:'free',ticket_no:'',qr_token:''};
    const code=(l.status!=='free'&&l.qr_token)?`<small class="code">🔑 ${l.qr_token}</small>`:'';
    return `<div class="lk ${l.status}"><b>${l.id}번</b><small>${LK[l.status]||l.status}</small>
      <small>${l.ticket_no||'—'}</small>${code}</div>`}).join('');
  const qb=$('queue');
  if(q&&q.length)qb.innerHTML=q.map(i=>`<tr><td>${i.class_name}</td><td>${i.order_id}</td><td>${chip(i.status)}</td></tr>`).join('');
  else emptyRow(qb,3,'대기 없음');
  const ob=$('orders');
  if(od&&od.length)ob.innerHTML=od.map(o=>`<tr><td>${o.ticket_no}</td><td>${chip(o.status)}</td>
    <td class="code">${o.qr_token||'—'}</td><td>${o.locker_id?o.locker_id+'번':'—'}</td>
    <td>${o.item_count}</td><td>${(o.created_at||'').replace('T',' ').slice(0,19)}</td></tr>`).join('');
  else emptyRow(ob,6,'주문 없음');
  const sb=$('stats');
  if(st&&st.length)sb.innerHTML=st.map(s=>`<tr><td>${s.class_name}</td><td>${s.success||0}</td>
    <td>${s.fail||0}</td><td>${s.last_fail_reason||'—'}</td></tr>`).join('');
  else emptyRow(sb,4,'기록 없음');
  const hb=$('hist');
  if(hi&&hi.length)hb.innerHTML=hi.slice(0,80).map(h=>`<tr><td>${h.ticket_no||'—'}</td><td>${h.class_name}</td>
    <td>${chip(h.status)}</td><td>${(h.at||'').replace('T',' ').slice(0,19)}</td></tr>`).join('');
  else emptyRow(hb,4,'이력 없음');
}
load(); setInterval(load,2000);
</script></body></html>"""


class OrderBody(BaseModel):
    lines: list[tuple[str, int]]   # [(class_name, qty)]
    code: str | None = None        # 유저 지정 4자리 수령 코드(없으면 자동 생성)


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
             'stock': c.stock, 'available': c.class_name in avail}
            for c in n.repo.list_catalog()]


@app.post('/api/orders')
def create_order(body: OrderBody):
    n = _node()
    if not body.lines:
        raise HTTPException(400, '빈 주문')
    try:
        order = n.submit_order(body.lines, body.code)
    except RuntimeError as e:        # 락커 만석 / 코드 오류·중복
        raise HTTPException(409, str(e))
    return {'order_id': order.order_id, 'ticket_no': order.ticket_no,
            'locker_id': order.locker_id, 'qr_token': order.qr_token}


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
        'items': [{'item_id': it.item_id, 'class_name': it.class_name,
                   'status': it.status} for it in items if it],
    }


@app.post('/api/orders/{order_id}/cancel')
def cancel_order(order_id: str):
    n = _node()
    if n.repo.get_order(order_id) is None:
        raise HTTPException(404, '주문 없음')
    n.cancel_order(order_id)
    return {'ok': True}


# ── 수령(QR/락커) ───────────────────────────────────────────────
class TokenBody(BaseModel):
    token: str


class StockBody(BaseModel):
    stock: int   # 재고 수량(0 이상)


@app.get('/api/pickup/qr')
def pickup_qr(token: str):
    """수령 QR PNG. qrcode 미설치 시 503 → 프론트는 코드 텍스트 입력으로 폴백."""
    if not token:
        raise HTTPException(400, '토큰 없음')
    try:
        import io
        import qrcode
        buf = io.BytesIO()
        qrcode.make(token).save(buf, format='PNG')
        return Response(content=buf.getvalue(), media_type='image/png')
    except ImportError:
        raise HTTPException(503, 'qrcode 미설치 — 코드 입력 사용')


@app.post('/api/pickup/scan')
def pickup_scan(body: TokenBody):
    """QR/코드 조회 → 락커 안내 정보."""
    n = _node()
    n.repo.reload()
    info = n.repo.locker_info_by_token(body.token)
    if info is None:
        raise HTTPException(404, '유효하지 않은 QR/코드')
    return info


@app.post('/api/pickup/confirm')
def pickup_confirm(body: TokenBody):
    """유저 수령 완료 → 락커 해제."""
    n = _node()
    n.repo.reload()
    res = n.repo.confirm_pickup(body.token)
    if res is None:
        raise HTTPException(404, '유효하지 않은 QR/코드')
    if not res.get('ok'):
        raise HTTPException(409, f"아직 수령할 수 없습니다 (상태={res.get('status')})")
    n._emit_queue_if_changed()
    return res


@app.get('/api/lockers')
def get_lockers():
    """락커 현황(유저·관리자 공용)."""
    n = _node()
    n.repo.reload()   # 타 프로세스(관리자 해제/리셋/수령) 변경 반영
    return {'lockers': n.repo.list_lockers()}


@app.get('/api/queue')
def get_queue():
    return _node()._queue_dump()


@app.get('/api/state')
def get_state():
    n = _node()
    return {'state': n.pick_place_state,
            'available': sorted(n.detected_classes),
            'error': n.last_error_text}


# ── 관리자 패널(/admin) — 재고/큐/주문/락커/픽통계/이력 ──
@app.get('/api/orders')
def list_all_orders(status: str | None = None):
    n = _node()
    statuses = {status} if status else None
    orders = n.repo.list_orders(statuses)
    orders.sort(key=lambda o: o.order_id, reverse=True)   # 최신순
    return [{'order_id': o.order_id, 'ticket_no': o.ticket_no, 'status': o.status,
             'qr_token': o.qr_token, 'locker_id': o.locker_id,
             'created_at': o.created_at, 'item_count': len(o.item_ids)}
            for o in orders]


@app.get('/api/history')
def get_history():
    return list(reversed(_node().repo.list_history()))   # 최신순


@app.get('/api/stats')
def get_stats():
    return _node().repo.get_pick_stats()


@app.put('/api/admin/stock/{class_name}')
def set_stock(class_name: str, body: StockBody):
    n = _node()
    if n.repo.get_catalog_item(class_name) is None:
        raise HTTPException(404, f"품목 '{class_name}' 없음")
    if body.stock < 0:
        raise HTTPException(400, '재고는 0 이상이어야 합니다')
    n.repo.set_stock(class_name, body.stock)
    return {'ok': True}


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
