// 백엔드(FastAPI) REST/WebSocket 클라이언트.
export type Catalog = {
  class_name: string
  display_name: string
  stock: number
  available: boolean
}
export type OrderResp = {
  order_id: string
  ticket_no: string
  locker_id?: number
  qr_token?: string
}
export type LockerInfo = {
  locker_id: number
  status: string
  order_id: string
  ticket_no: string
  order_status: string
  ready: boolean
}
// /api/lockers 한 칸. status: 'free' | 'occupied'(배달중) | 'ready'(수령가능)
export type LockerSlot = {
  id: number
  status: string
  order_id: string
  ticket_no: string
  qr_token?: string
}
export async function getLockers(): Promise<LockerSlot[]> {
  try {
    const r = await fetch('/api/lockers')
    if (!r.ok) return []
    return (await r.json()).lockers ?? []
  } catch {
    return []
  }
}
export type OrderItem = { item_id: string; class_name: string; status: string }
export type OrderDetail = {
  order_id: string
  ticket_no: string
  status: string
  items: OrderItem[]
}

export async function getCatalog(): Promise<Catalog[]> {
  const r = await fetch('/api/catalog')
  return r.json()
}

export async function createOrder(lines: [string, number][], code?: string): Promise<OrderResp> {
  const r = await fetch('/api/orders', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ lines, code: code ?? null }),
  })
  if (!r.ok) throw new Error((await r.json().catch(() => ({}))).detail || '주문 생성 실패')
  return r.json()
}

export async function getOrder(id: string): Promise<OrderDetail> {
  const r = await fetch(`/api/orders/${id}`)
  if (!r.ok) throw new Error('주문 조회 실패')
  return r.json()
}

export async function cancelOrder(id: string): Promise<void> {
  await fetch(`/api/orders/${id}/cancel`, { method: 'POST' })
}

// ── 수령(QR/락커) ──
export async function scanPickup(token: string): Promise<LockerInfo> {
  const r = await fetch('/api/pickup/scan', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ token }),
  })
  if (!r.ok) throw new Error((await r.json().catch(() => ({}))).detail || '유효하지 않은 QR/코드')
  return r.json()
}

export async function confirmPickup(token: string): Promise<void> {
  const r = await fetch('/api/pickup/confirm', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ token }),
  })
  if (!r.ok) throw new Error((await r.json().catch(() => ({}))).detail || '수령 처리 실패')
}

export function pickupQrUrl(token: string): string {
  return `/api/pickup/qr?token=${encodeURIComponent(token)}`
}

export type QueueItem = OrderItem & {
  order_id: string
  ticket_no?: string
  order_status?: string
}
export async function getQueue(): Promise<QueueItem[]> {
  const r = await fetch('/api/queue')
  return r.json()
}

// WebSocket 이벤트: {type:'state'|'detected'|'queue'|'error'|'paused'|'order_done', ...}
export type WsEvent =
  | { type: 'state'; value: string }
  | { type: 'detected'; classes: string[] }
  | { type: 'queue'; items: QueueItem[] }
  | { type: 'error'; msg: string }
  | { type: 'paused'; paused: boolean; state: string }
  | {
      type: 'order_done'
      order_id: string
      ticket_no: string
      status: string
      locker_id: number
      failed: string[]
      all_ok: boolean
    }

export function connectWs(onEvent: (e: WsEvent) => void): WebSocket {
  const proto = location.protocol === 'https:' ? 'wss' : 'ws'
  const ws = new WebSocket(`${proto}://${location.host}/ws`)
  ws.onmessage = (m) => {
    try {
      onEvent(JSON.parse(m.data))
    } catch {
      /* ignore */
    }
  }
  return ws
}
