// 백엔드(FastAPI) REST/WebSocket 클라이언트.
export type Catalog = {
  class_name: string
  display_name: string
  stock: number
  available: boolean
}
export type OrderResp = { order_id: string; ticket_no: string }
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

export async function createOrder(lines: [string, number][]): Promise<OrderResp> {
  const r = await fetch('/api/orders', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ lines }),
  })
  if (!r.ok) throw new Error('주문 생성 실패')
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

export type QueueItem = OrderItem & {
  order_id: string
  ticket_no?: string
  order_status?: string
}
export async function getQueue(): Promise<QueueItem[]> {
  const r = await fetch('/api/queue')
  return r.json()
}

// WebSocket 이벤트: {type:'state'|'detected'|'queue'|'error'|'paused', ...}
export type WsEvent =
  | { type: 'state'; value: string }
  | { type: 'detected'; classes: string[] }
  | { type: 'queue'; items: QueueItem[] }
  | { type: 'error'; msg: string }
  | { type: 'paused'; paused: boolean; state: string }

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
