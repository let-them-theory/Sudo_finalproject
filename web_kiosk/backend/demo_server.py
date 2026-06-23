"""Standalone 데모 서버 — ROS/로봇 없이 QR/락커 수령 흐름을 브라우저에서 확인하는 검증용.

실행:
    cd web_kiosk/frontend && npm install && npm run build   # dist 생성(최초 1회)
    python3 web_kiosk/backend/demo_server.py                # http://localhost:8000

production main.py와 '동일한' task_repository 로직을 쓴다(엔드포인트 본문도 main.py와 동일).
차이는 ROS 노드 대신 모듈 전역 repo를 직접 쓰고, 로봇 배달을 흉내내는 /api/demo/deliver를
추가한 것뿐. 흐름: 주문 → QR/락커 발급 → (배달 시뮬) → 스캔 → 수령완료 → 락커 free.
"""
import io
import sys
from pathlib import Path

from fastapi import FastAPI, HTTPException
from fastapi.responses import Response
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

# 실제 task_repository 로직 재사용(코드 중복 없이 동일 동작 보장).
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'dsr_realsense_pick_place'))
from task_repository import HybridRepository, CatalogItem, ItemStatus  # noqa: E402

DB = Path('/tmp/kiosk_demo.json')
repo = HybridRepository(DB)
repo.seed_catalog([
    CatalogItem('jelly', '젤리', stock=10), CatalogItem('can', '캔', stock=10),
    CatalogItem('water', '생수', stock=10), CatalogItem('ramen', '라면', stock=10),
])

app = FastAPI(title='Kiosk QR/Locker Demo')


class OrderBody(BaseModel):
    lines: list[tuple[str, int]]
    code: str | None = None


class TokenBody(BaseModel):
    token: str


@app.get('/api/catalog')
def catalog():
    return [{'class_name': c.class_name, 'display_name': c.display_name,
             'stock': c.stock, 'available': True} for c in repo.list_catalog()]


@app.post('/api/orders')
def create(body: OrderBody):
    if not body.lines:
        raise HTTPException(400, '빈 주문')
    repo.reload()
    o = repo.create_order(body.lines)
    try:
        a = repo.assign_locker(o.order_id, body.code)
    except ValueError as e:
        repo.cancel_order(o.order_id)
        raise HTTPException(409, str(e))
    if a is None:
        repo.cancel_order(o.order_id)
        raise HTTPException(409, '모든 락커가 사용 중입니다')
    return {'order_id': o.order_id, 'ticket_no': o.ticket_no,
            'locker_id': a['locker_id'], 'qr_token': a['qr_token']}


@app.post('/api/orders/{oid}/cancel')
def cancel(oid: str):
    repo.reload()
    repo.cancel_order(oid)
    return {'ok': True}


@app.get('/api/pickup/qr')
def qr(token: str):
    if not token:
        raise HTTPException(400, '토큰 없음')
    import qrcode
    buf = io.BytesIO()
    qrcode.make(token).save(buf, format='PNG')
    return Response(content=buf.getvalue(), media_type='image/png')


@app.post('/api/pickup/scan')
def scan(b: TokenBody):
    repo.reload()
    info = repo.locker_info_by_token(b.token)
    if info is None:
        raise HTTPException(404, '유효하지 않은 QR/코드')
    return info


@app.post('/api/pickup/confirm')
def confirm(b: TokenBody):
    repo.reload()
    res = repo.confirm_pickup(b.token)
    if res is None:
        raise HTTPException(404, '유효하지 않은 QR/코드')
    if not res.get('ok'):
        raise HTTPException(409, f"아직 수령할 수 없습니다 (상태={res.get('status')})")
    return res


@app.get('/api/lockers')
def lockers():
    repo.reload()
    return {'lockers': repo.list_lockers()}


@app.get('/api/queue')
def queue():
    return []


@app.get('/api/state')
def state():
    return {'state': 'IDLE', 'available': [], 'error': ''}


@app.post('/api/demo/deliver/{oid}')
def demo_deliver(oid: str):
    """로봇 배달 흉내 — 주문의 전 품목을 DONE으로 → 락커 ready (수령 가능)."""
    repo.reload()
    o = repo.get_order(oid)
    if o is None:
        raise HTTPException(404, '주문 없음')
    for iid in o.item_ids:
        repo.set_item_status(iid, ItemStatus.DONE)
    return {'ok': True, 'order_status': repo.get_order(oid).status}


# 빌드된 프론트(dist) 정적 서빙 — 없으면 API만.
_DIST = Path(__file__).resolve().parents[1] / 'frontend' / 'dist'
if (_DIST / 'index.html').is_file():
    app.mount('/', StaticFiles(directory=str(_DIST), html=True), name='spa')


if __name__ == '__main__':
    import uvicorn
    print('데모 서버 → http://localhost:8000')
    print('  흐름: 브라우저서 주문 → QR/락커 확인 →')
    print('  배달 시뮬: curl -X POST http://localhost:8000/api/demo/deliver/<order_id>')
    print('  → 수령하기(QR 스캔/코드입력) → 수령완료')
    if not (_DIST / 'index.html').is_file():
        print('  [경고] 프론트 dist 없음 — `cd ../frontend && npm run build` 먼저')
    uvicorn.run(app, host='0.0.0.0', port=8000)
