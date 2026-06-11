// 원격 주문 키오스크 — 환영(대기현황) → 상품선택 → 주문확인 → 주문완료(영수증). 토스풍.
import { useEffect, useState } from 'react'
import {
  getCatalog, createOrder, connectWs,
  type Catalog, type OrderResp, type QueueItem,
} from './api'
import { nameOf, emojiOf } from './products'
import {
  ChevronUp, ChevronDown, Plus, Minus, Check, AlertTriangle, Clock,
} from 'lucide-react'

type Page = 'welcome' | 'select' | 'confirm' | 'done'
type ReceiptItem = { class_name: string; qty: number }

// 품절/미검출 표시 — 로직만 배선, 지금은 끔(큐→픽플레이스 검증 우선). 나중에 true.
const SHOW_SOLDOUT = false
const IDLE_MS = 60000               // 무동작 시 처음 화면 복귀
const ORDER_KEY = 'kiosk_receipt'   // 새로고침 후 영수증 복원

const STEPS: { key: Page; label: string }[] = [
  { key: 'select', label: '선택' },
  { key: 'confirm', label: '확인' },
  { key: 'done', label: '완료' },
]

const ORDER_STATUS_KR: Record<string, string> = {
  RUNNING: '처리 중', QUEUED: '대기 중', PAUSED: '멈춤',
}

type BoardRow = { ticket: string; status: string; order_id: string }
function buildBoard(queue: QueueItem[]): BoardRow[] {
  const rows: BoardRow[] = []
  const seen = new Set<string>()
  for (const q of queue) {
    if (!q.order_id || seen.has(q.order_id)) continue
    seen.add(q.order_id)
    rows.push({
      ticket: q.ticket_no || '?',
      status: ORDER_STATUS_KR[q.order_status || ''] || '대기 중',
      order_id: q.order_id,
    })
  }
  return rows
}

export default function App() {
  const [page, setPage] = useState<Page>('welcome')
  const [catalog, setCatalog] = useState<Catalog[]>([])
  const [cart, setCart] = useState<Record<string, number>>({})
  const [order, setOrder] = useState<OrderResp | null>(null)
  const [receipt, setReceipt] = useState<ReceiptItem[]>([])
  const [state, setState] = useState('')
  const [available, setAvailable] = useState<string[]>([])
  const [paused, setPaused] = useState(false)
  const [queue, setQueue] = useState<QueueItem[]>([])
  const [cartOpen, setCartOpen] = useState(false)
  const [err, setErr] = useState('')

  useEffect(() => {
    getCatalog().then(setCatalog).catch(() => {})
  }, [])

  // 새로고침 시 영수증 복원.
  useEffect(() => {
    const saved = localStorage.getItem(ORDER_KEY)
    if (!saved) return
    try {
      const r = JSON.parse(saved) as { order: OrderResp; items: ReceiptItem[] }
      setOrder(r.order); setReceipt(r.items); setPage('done')
    } catch { localStorage.removeItem(ORDER_KEY) }
  }, [])

  useEffect(() => {
    const ws = connectWs((e) => {
      if (e.type === 'state') setState(e.value)
      else if (e.type === 'detected') setAvailable(e.classes)
      else if (e.type === 'queue') setQueue(e.items)
      else if (e.type === 'paused') setPaused(e.paused)
    })
    return () => ws.close()
  }, [])

  // 유휴 자동 리셋 (홈 제외).
  useEffect(() => {
    if (page === 'welcome') return
    let t = window.setTimeout(toWelcome, IDLE_MS)
    const bump = () => { clearTimeout(t); t = window.setTimeout(toWelcome, IDLE_MS) }
    window.addEventListener('pointerdown', bump)
    window.addEventListener('keydown', bump)
    return () => {
      clearTimeout(t)
      window.removeEventListener('pointerdown', bump)
      window.removeEventListener('keydown', bump)
    }
  }, [page])

  const add = (c: string) => setCart((p) => ({ ...p, [c]: (p[c] || 0) + 1 }))
  const sub = (c: string) =>
    setCart((p) => {
      const n = { ...p }
      if ((n[c] || 0) > 1) n[c]--
      else delete n[c]
      return n
    })
  const total = Object.values(cart).reduce((a, b) => a + b, 0)
  const kinds = Object.keys(cart).length

  const submit = async () => {
    try {
      const lines = Object.entries(cart) as [string, number][]
      const o = await createOrder(lines)
      const items: ReceiptItem[] = lines.map(([c, q]) => ({ class_name: c, qty: q }))
      localStorage.setItem(ORDER_KEY, JSON.stringify({ order: o, items }))
      setOrder(o); setReceipt(items); setErr(''); setCart({}); setPage('done')
    } catch {
      setErr('주문 접수에 실패했어요. 다시 시도해주세요.')
    }
  }
  function toWelcome() {
    localStorage.removeItem(ORDER_KEY)
    setCart({}); setOrder(null); setReceipt([]); setCartOpen(false); setErr(''); setPage('welcome')
  }

  const hardError = state === 'ERROR' || state === 'EMERGENCY_STOP'
  const alertLevel: 'error' | 'warn' | null = hardError ? 'error' : paused ? 'warn' : null
  const board = buildBoard(queue)

  return (
    <div className={`mx-auto flex h-full w-full max-w-2xl flex-col transition-colors ${
      alertLevel === 'error' ? 'bg-red-50' : alertLevel === 'warn' ? 'bg-amber-50' : 'bg-white'
    }`}>
      <Header page={page} />
      {alertLevel && (
        <div className={`flex items-center justify-center gap-2 px-4 py-2.5 text-[15px] font-bold text-white ${
          alertLevel === 'error' ? 'bg-red-500' : 'bg-amber-400 text-amber-950'
        }`}>
          <AlertTriangle size={17} />
          {alertLevel === 'error' ? '시스템 점검 중이에요 — 잠시 후 처리됩니다' : '잠시 지연되고 있어요'}
        </div>
      )}
      <main className="flex-1 overflow-hidden">
        {page === 'welcome' && <Welcome board={board} onStart={() => setPage('select')} />}
        {page === 'select' && (
          <SelectPage
            catalog={catalog} cart={cart} total={total} kinds={kinds} cartOpen={cartOpen}
            available={available}
            onToggleCart={() => setCartOpen((v) => !v)}
            onAdd={add} onSub={sub} onClear={() => setCart({})}
            onNext={() => total > 0 && setPage('confirm')}
          />
        )}
        {page === 'confirm' && (
          <ConfirmPage cart={cart} total={total} err={err}
            onBack={() => setPage('select')} onPlace={submit} />
        )}
        {page === 'done' && (
          <DonePage order={order} receipt={receipt} board={board} onHome={toWelcome} />
        )}
      </main>
    </div>
  )
}

function Header({ page }: { page: Page }) {
  const activeIdx = STEPS.findIndex((s) => s.key === page)
  return (
    <header className="flex items-center justify-between border-b border-line px-6 py-4">
      <div className="text-xl font-bold text-ink">무인 스토어</div>
      {page !== 'welcome' && (
        <div className="flex items-center gap-2">
          {STEPS.map((s, i) => {
            const on = i === activeIdx
            const done = i < activeIdx
            return (
              <div key={s.key} className="flex items-center gap-2">
                <span
                  className={`flex h-6 items-center gap-1.5 rounded-full px-2.5 text-[13px] font-bold transition-all ${
                    on ? 'bg-brand text-white' : done ? 'text-brand' : 'text-muted'
                  }`}
                >
                  {done ? <Check size={13} /> : <span>{i + 1}</span>}
                  {s.label}
                </span>
                {i < STEPS.length - 1 && <span className="text-line">›</span>}
              </div>
            )
          })}
        </div>
      )}
    </header>
  )
}

function CTA(props: { onClick: () => void; disabled?: boolean; children: React.ReactNode }) {
  return (
    <button
      onClick={props.onClick}
      disabled={props.disabled}
      className="w-full rounded-2xl bg-brand py-4 text-[17px] font-bold text-white transition-all hover:bg-brand-dark active:scale-[0.98] disabled:bg-line disabled:text-muted"
    >
      {props.children}
    </button>
  )
}

function QueueBoard({ board, myId }: { board: BoardRow[]; myId?: string }) {
  if (board.length === 0) {
    return (
      <div className="rounded-2xl bg-surface px-4 py-3 text-center text-[15px] font-semibold text-muted">
        대기 중인 주문이 없어요 · 바로 주문 가능
      </div>
    )
  }
  return (
    <div className="rounded-2xl bg-surface px-4 py-3">
      <div className="mb-2 flex items-center gap-1.5 text-[13px] font-bold text-muted">
        <Clock size={13} /> 현재 대기 {board.length}건
      </div>
      <div className="flex flex-wrap items-center gap-x-2 gap-y-1 text-[15px]">
        {board.map((b, i) => (
          <span key={b.order_id} className="flex items-center gap-2">
            {i > 0 && <span className="text-line">|</span>}
            <span className={b.order_id === myId ? 'font-bold text-brand' : 'font-semibold text-sub'}>
              {b.ticket} {b.status}
            </span>
          </span>
        ))}
      </div>
    </div>
  )
}

function Welcome({ board, onStart }: { board: BoardRow[]; onStart: () => void }) {
  return (
    <div className="page-fade flex h-full flex-col px-6">
      <div className="flex flex-1 flex-col items-center justify-center gap-6">
        <div className="text-8xl">🛒</div>
        <div className="text-center">
          <div className="text-[30px] font-bold leading-tight text-ink">
            무엇을<br />담아드릴까요?
          </div>
          <div className="mt-3 text-[17px] text-muted">로봇이 직접 담아드려요</div>
        </div>
      </div>
      <div className="space-y-3 pb-8">
        <QueueBoard board={board} />
        <CTA onClick={onStart}>주문 시작하기</CTA>
      </div>
    </div>
  )
}

function SelectPage(props: {
  catalog: Catalog[]
  cart: Record<string, number>
  total: number
  kinds: number
  cartOpen: boolean
  available: string[]
  onToggleCart: () => void
  onAdd: (c: string) => void
  onSub: (c: string) => void
  onClear: () => void
  onNext: () => void
}) {
  const { catalog, cart, total, kinds, cartOpen, available } = props
  return (
    <div className="page-fade flex h-full flex-col">
      <div className="flex-1 overflow-y-auto px-6 py-5">
        <div className="mb-4 text-[22px] font-bold text-ink">상품을 골라주세요</div>
        <div className="grid grid-cols-3 gap-3">
          {catalog.map((c) => {
            const qty = cart[c.class_name] || 0
            const sold = SHOW_SOLDOUT && !available.includes(c.class_name)
            return (
              <button
                key={c.class_name}
                disabled={sold}
                onClick={() => props.onAdd(c.class_name)}
                className={`relative flex flex-col items-center gap-2 rounded-2xl border bg-white py-5 transition-all active:scale-95 ${
                  qty ? 'border-brand bg-brand-light' : 'border-line'
                } ${sold ? 'opacity-40' : ''}`}
              >
                {qty > 0 && (
                  <span className="pop absolute right-2 top-2 flex h-6 min-w-6 items-center justify-center rounded-full bg-brand px-1.5 text-[13px] font-bold text-white">
                    {qty}
                  </span>
                )}
                {sold && (
                  <span className="absolute left-2 top-2 rounded-full bg-muted px-1.5 text-[11px] font-bold text-white">
                    품절
                  </span>
                )}
                <span className="text-[42px]">{emojiOf(c.class_name)}</span>
                <span className="text-[15px] font-semibold text-ink">{nameOf(c.class_name)}</span>
              </button>
            )
          })}
        </div>
      </div>

      <div className="border-t border-line px-6 pb-6 pt-4">
        {cartOpen && total > 0 && (
          <div className="page-fade mb-3 max-h-44 space-y-2 overflow-y-auto">
            {Object.entries(cart).map(([c, q]) => (
              <div key={c} className="flex items-center gap-3 rounded-xl bg-surface px-3 py-2.5">
                <span className="text-2xl">{emojiOf(c)}</span>
                <span className="flex-1 font-semibold text-ink">{nameOf(c)}</span>
                <button onClick={() => props.onSub(c)}
                  className="flex h-8 w-8 items-center justify-center rounded-lg bg-white text-sub active:scale-90">
                  <Minus size={16} />
                </button>
                <span className="w-5 text-center font-bold text-ink">{q}</span>
                <button onClick={() => props.onAdd(c)}
                  className="flex h-8 w-8 items-center justify-center rounded-lg bg-white text-brand active:scale-90">
                  <Plus size={16} />
                </button>
              </div>
            ))}
          </div>
        )}
        <div className="mb-3 flex items-center">
          <div className="flex-1 text-[15px] font-semibold text-sub">
            {total ? `${kinds}종 · ${total}개 담음` : '담은 상품 없음'}
          </div>
          {total > 0 && (
            <button onClick={props.onToggleCart}
              className="flex items-center gap-1 rounded-lg px-2 py-1 text-[13px] font-semibold text-muted active:scale-95">
              {cartOpen ? <ChevronDown size={15} /> : <ChevronUp size={15} />}
              {cartOpen ? '접기' : '담은 상품'}
            </button>
          )}
          {total > 0 && (
            <button onClick={props.onClear}
              className="ml-2 rounded-lg px-2 py-1 text-[13px] font-semibold text-muted active:scale-95">
              비우기
            </button>
          )}
        </div>
        <CTA onClick={props.onNext} disabled={total === 0}>
          {total ? `${total}개 담기 · 다음` : '상품을 담아주세요'}
        </CTA>
      </div>
    </div>
  )
}

function ConfirmPage(props: {
  cart: Record<string, number>
  total: number
  err: string
  onBack: () => void
  onPlace: () => void
}) {
  const { cart, total, err } = props
  return (
    <div className="page-fade flex h-full flex-col">
      <div className="flex-1 overflow-y-auto px-6 py-5">
        <div className="mb-4 text-[22px] font-bold text-ink">주문을 확인해주세요</div>
        <div className="space-y-2.5">
          {Object.entries(cart).map(([c, q]) => (
            <div key={c}
              className="flex items-center gap-4 rounded-2xl border border-line bg-white p-4">
              <span className="text-3xl">{emojiOf(c)}</span>
              <span className="flex-1 text-[17px] font-semibold text-ink">{nameOf(c)}</span>
              <span className="text-[17px] font-bold text-brand">{q}개</span>
            </div>
          ))}
        </div>
      </div>
      <div className="border-t border-line px-6 pb-6 pt-4">
        {err && (
          <div className="mb-3 rounded-xl bg-red-50 px-4 py-2.5 text-[14px] font-semibold text-red-600">
            {err}
          </div>
        )}
        <div className="mb-3 flex items-center justify-between">
          <span className="text-[15px] font-semibold text-sub">총 수량</span>
          <span className="text-[17px] font-bold text-ink">{total}개</span>
        </div>
        <div className="flex gap-2">
          <button onClick={props.onBack}
            className="rounded-2xl border border-line px-6 py-4 text-[16px] font-bold text-sub active:scale-95">
            뒤로
          </button>
          <div className="flex-1">
            <CTA onClick={props.onPlace}>주문하기</CTA>
          </div>
        </div>
      </div>
    </div>
  )
}

// 원격 주문 — 주문 완료 영수증. 실시간 로봇 추적 없음(주문은 큐에 올라가 처리됨).
function DonePage(props: {
  order: OrderResp | null
  receipt: ReceiptItem[]
  board: BoardRow[]
  onHome: () => void
}) {
  const { order, receipt, board } = props
  const ahead = order
    ? board.findIndex((b) => b.order_id === order.order_id)
    : -1
  const total = receipt.reduce((a, b) => a + b.qty, 0)

  return (
    <div className="page-fade flex h-full flex-col px-6">
      <div className="flex flex-1 flex-col items-center overflow-y-auto pt-8">
        <span className="pop flex h-16 w-16 items-center justify-center rounded-full bg-brand text-white">
          <Check size={36} />
        </span>
        <div className="mt-4 text-[26px] font-bold text-ink">주문 완료!</div>
        <div className="mt-1 text-[16px] text-muted">주문이 큐에 등록되었어요</div>

        {/* 영수증 카드 */}
        <div className="mt-6 w-full rounded-2xl border-2 border-dashed border-line bg-white p-5">
          <div className="flex items-center justify-between border-b border-line pb-3">
            <span className="text-[15px] font-semibold text-muted">대기 번호</span>
            <span className="text-[28px] font-bold text-brand">{order?.ticket_no ?? '—'}</span>
          </div>
          <div className="space-y-2.5 py-3">
            {receipt.map((it) => (
              <div key={it.class_name} className="flex items-center gap-3">
                <span className="text-2xl">{emojiOf(it.class_name)}</span>
                <span className="flex-1 font-semibold text-ink">{nameOf(it.class_name)}</span>
                <span className="font-bold text-sub">{it.qty}개</span>
              </div>
            ))}
          </div>
          <div className="flex items-center justify-between border-t border-line pt-3 text-[15px]">
            <span className="font-semibold text-muted">총 수량</span>
            <span className="font-bold text-ink">{total}개</span>
          </div>
        </div>

        {ahead > 0 && (
          <div className="mt-4 flex items-center gap-1.5 rounded-full bg-brand-light px-4 py-1.5 text-[14px] font-bold text-brand">
            <Clock size={14} /> 앞에 {ahead}건 대기 중
          </div>
        )}
      </div>

      <div className="pb-6 pt-4">
        <CTA onClick={props.onHome}>홈으로</CTA>
      </div>
    </div>
  )
}
