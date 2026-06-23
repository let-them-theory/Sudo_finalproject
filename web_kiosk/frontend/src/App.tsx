// 원격 주문 키오스크 — 환영(대기현황) → 상품선택 → 주문확인 → 주문완료(영수증). 토스풍.
import { useEffect, useRef, useState } from 'react'
import {
  getCatalog, createOrder, connectWs, scanPickup, confirmPickup, pickupQrUrl, getLockers,
  type Catalog, type OrderResp, type QueueItem, type LockerInfo, type LockerSlot,
} from './api'
import { nameOf, emojiOf } from './products'
import jsQR from 'jsqr'   // 데스크톱 크롬/파폭엔 BarcodeDetector가 없어 QR 디코드 폴백으로 사용
import {
  ChevronUp, ChevronDown, ChevronLeft, RefreshCw, Plus, Minus, Check, AlertTriangle, Clock, Package,
} from 'lucide-react'

type Page = 'welcome' | 'select' | 'confirm' | 'pay' | 'setcode' | 'done' | 'pickup'
type ReceiptItem = { class_name: string; qty: number }

const IDLE_MS = 60000               // 무동작 시 처음 화면 복귀
const ORDER_KEY = 'kiosk_receipt'   // 새로고침 후 영수증 복원

const STEPS: { key: Page; label: string }[] = [
  { key: 'select', label: '주문' },
  { key: 'pay', label: '결제' },
  { key: 'setcode', label: '코드' },
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
  const [paused, setPaused] = useState(false)
  const [queue, setQueue] = useState<QueueItem[]>([])
  const [lockers, setLockers] = useState<LockerSlot[]>([])
  const [cartOpen, setCartOpen] = useState(false)
  const [err, setErr] = useState('')
  // 주문 완료(배달끝) 알림 — order_done WS 이벤트.
  const [doneNotice, setDoneNotice] = useState<
    { order_id: string; status: string; locker_id: number; failed: string[]; all_ok: boolean } | null
  >(null)

  useEffect(() => {
    getCatalog().then(setCatalog).catch(() => {})
    // 재고 실시간 반영 — admin/sort_all로 바뀐 재고를 주기적으로 다시 받아 품절/수량 갱신.
    const t = setInterval(() => getCatalog().then(setCatalog).catch(() => {}), 3000)
    return () => clearInterval(t)
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
      else if (e.type === 'queue') setQueue(e.items)
      else if (e.type === 'paused') setPaused(e.paused)
      else if (e.type === 'order_done')
        setDoneNotice({ order_id: e.order_id, status: e.status, locker_id: e.locker_id,
          failed: e.failed, all_ok: e.all_ok })
    })
    return () => ws.close()
  }, [])

  // HOME 락커(보관함) 현황 폴링 — 어느 주문이 어느 칸에 있는지 표시.
  useEffect(() => {
    if (page !== 'welcome') return
    let alive = true
    const tick = () => getLockers().then((l) => { if (alive) setLockers(l) }).catch(() => {})
    tick()
    const id = window.setInterval(tick, 1500)
    return () => { alive = false; clearInterval(id) }
  }, [page])

  // 유휴 자동 리셋 (홈 제외).
  useEffect(() => {
    if (page === 'welcome' || page === 'done') return   // done은 자체 15초 타이머 사용
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

  const submit = async (code: string) => {
    try {
      const lines = Object.entries(cart) as [string, number][]
      const o = await createOrder(lines, code)
      const items: ReceiptItem[] = lines.map(([c, q]) => ({ class_name: c, qty: q }))
      localStorage.setItem(ORDER_KEY, JSON.stringify({ order: o, items }))
      setOrder(o); setReceipt(items); setErr(''); setCart({}); setPage('done')
    } catch (e) {
      setErr(e instanceof Error ? e.message : '주문 접수에 실패했어요. 다시 시도해주세요.')
    }
  }
  function toWelcome() {
    localStorage.removeItem(ORDER_KEY)
    setCart({}); setOrder(null); setReceipt([]); setCartOpen(false); setErr('')
    setDoneNotice(null); setPage('welcome')
  }

  const hardError = state === 'ERROR' || state === 'EMERGENCY_STOP'
  const alertLevel: 'error' | 'warn' | null = hardError ? 'error' : paused ? 'warn' : null
  const board = buildBoard(queue)

  return (
    <div className="flex h-screen w-screen items-center justify-center overflow-hidden bg-neutral-200">
      <div className={`relative flex flex-col overflow-hidden rounded-[20px] shadow-2xl transition-colors ${
        alertLevel === 'error' ? 'bg-red-50' : alertLevel === 'warn' ? 'bg-amber-50' : 'bg-white'
      }`} style={{ width: 'var(--kiosk-w)', height: 'var(--kiosk-h)' }}>
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
        {page === 'welcome' && (
          <Welcome board={board} lockers={lockers} onStart={() => setPage('select')}
            onPickup={() => setPage('pickup')}
            onRefresh={() => getCatalog().then(setCatalog).catch(() => {})} />
        )}
        {page === 'pickup' && (
          <PickupPage onBack={() => setPage('welcome')} />
        )}
        {page === 'select' && (
          <SelectPage
            catalog={catalog} cart={cart} total={total} kinds={kinds} cartOpen={cartOpen}
            onToggleCart={() => setCartOpen((v) => !v)}
            onAdd={add} onSub={sub} onClear={() => setCart({})}
            onNext={() => total > 0 && setPage('confirm')}
            onBack={() => { setCart({}); setCartOpen(false); setPage('welcome') }}
          />
        )}
        {page === 'confirm' && (
          <ConfirmPage cart={cart} total={total} err={err}
            onBack={() => { setErr(''); setPage('select') }}
            onPlace={async () => {
              // DB 재고와 주문 대조 — 부족하면 다음(결제)으로 안 넘기고 막는다.
              setErr('')
              const cat = await getCatalog().catch(() => null)
              if (cat) {
                setCatalog(cat)   // 선택 화면 재고/품절도 최신으로
                const short = Object.entries(cart)
                  .filter(([c, q]) => { const it = cat.find((x) => x.class_name === c); return !it || it.stock < q })
                  .map(([c]) => nameOf(c))
                if (short.length) { setErr(`재고 부족: ${short.join(', ')} — 수량을 줄여주세요`); return }
              }
              setPage('pay')
            }} />
        )}
        {page === 'pay' && (
          <PayPage total={total} onBack={() => setPage('confirm')}
            onPaid={() => setPage('setcode')} />
        )}
        {page === 'setcode' && (
          <SetCodePage err={err} onBack={() => setPage('pay')} onSubmit={submit} />
        )}
        {page === 'done' && (
          <DonePage order={order} receipt={receipt} board={board}
            notice={doneNotice && order && doneNotice.order_id === order.order_id ? doneNotice : null}
            onHome={toWelcome} />
        )}
      </main>
      </div>
    </div>
  )
}

function Header({ page }: { page: Page }) {
  // confirm은 주문(select) 단계로 묶어 표시.
  const stepKey = page === 'confirm' ? 'select' : page
  const activeIdx = STEPS.findIndex((s) => s.key === stepKey)
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

// HOME 보관함 현황 — 8칸 전부 표시. 완료=초록/진행중=노랑/대기(빈칸)=흰색. 번호+주문번호+상태.
function LockerBoard({ lockers }: { lockers: LockerSlot[] }) {
  // 항상 8칸 그리드. lockers 비었으면 id만 채운 빈칸으로.
  const slots: LockerSlot[] = Array.from({ length: 8 }, (_, i) => {
    const id = i + 1
    return lockers.find((l) => l.id === id) ?? { id, status: 'free', order_id: '', ticket_no: '' }
  })
  return (
    <div className="rounded-xl border border-line bg-white/70 p-3">
      <div className="mb-2 flex items-center gap-1.5 text-[14px] font-bold text-ink">
        <Package size={15} /> 보관함 현황
        <span className="ml-auto flex items-center gap-2 text-[11px] font-semibold text-muted">
          <span className="inline-flex items-center gap-1"><i className="inline-block h-2.5 w-2.5 rounded-sm bg-amber-300" />진행중</span>
          <span className="inline-flex items-center gap-1"><i className="inline-block h-2.5 w-2.5 rounded-sm bg-green-400" />수령가능</span>
        </span>
      </div>
      <div className="grid grid-cols-4 gap-2">
        {slots.map((l) => {
          const ready = l.status === 'ready'
          const busy = l.status === 'occupied'
          const cls = ready
            ? 'bg-green-100 border-green-300 text-green-800'
            : busy
            ? 'bg-amber-100 border-amber-300 text-amber-900'
            : 'bg-white border-line text-muted'
          return (
            <div key={l.id}
              className={`flex flex-col items-center justify-center rounded-lg border py-2 ${cls}`}>
              <span className="text-[15px] font-bold">{l.id}</span>
              <span className="font-mono text-[12px] leading-tight">{l.ticket_no || '—'}</span>
              <span className="text-[11px] font-semibold leading-tight">
                {ready ? '수령가능' : busy ? '배달중' : '대기'}
              </span>
            </div>
          )
        })}
      </div>
    </div>
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
        {board.map((b, i) => {
          const running = b.status === ORDER_STATUS_KR.RUNNING   // 처리 중 → 초록 강조
          return (
            <span key={b.order_id} className="flex items-center gap-2">
              {i > 0 && <span className="text-line">|</span>}
              <span className={
                b.order_id === myId ? 'font-bold text-brand'
                : running ? 'font-bold text-emerald-600'
                : 'font-semibold text-sub'}>
                {b.ticket} {b.status}
              </span>
            </span>
          )
        })}
      </div>
    </div>
  )
}

function Welcome({ board, lockers, onStart, onPickup, onRefresh }: {
  board: BoardRow[]; lockers: LockerSlot[]
  onStart: () => void; onPickup: () => void; onRefresh: () => void
}) {
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
        {/* 진행중인 큐 새로고침 (대기현황 갱신) */}
        <div className="flex justify-end">
          <button onClick={onRefresh}
            className="flex items-center gap-1 rounded-lg px-3 py-1.5 text-[14px] font-semibold text-muted active:scale-95">
            <RefreshCw size={16} /> 새로고침
          </button>
        </div>
        <LockerBoard lockers={lockers} />
        <QueueBoard board={board} />
        <CTA onClick={onStart}>주문 시작하기</CTA>
        <button onClick={onPickup}
          className="flex w-full items-center justify-center gap-2 rounded-xl border-2 border-brand py-3 text-[16px] font-bold text-brand active:scale-95">
          📦 QR로 수령하기
        </button>
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
  onToggleCart: () => void
  onAdd: (c: string) => void
  onSub: (c: string) => void
  onClear: () => void
  onNext: () => void
  onBack: () => void
}) {
  const { cart, total, kinds, cartOpen } = props
  // box(박스)는 판매/이동 대상 아님 — 선택 버튼에서 제외(검출은 백엔드서 계속).
  const catalog = props.catalog.filter((c) => c.class_name !== 'box')
  return (
    <div className="page-fade flex h-full flex-col">
      <div className="flex-1 overflow-y-auto px-6 py-5">
        {/* 상단: 이전 / 제목 (이전 누르면 선택 초기화) */}
        <div className="mb-4 flex items-center gap-3">
          <button onClick={props.onBack}
            className="flex items-center gap-1 rounded-lg px-2 py-1.5 text-[14px] font-semibold text-muted active:scale-95">
            <ChevronLeft size={18} /> 이전
          </button>
          <div className="text-[22px] font-bold text-ink">상품을 골라주세요</div>
        </div>
        <div className="grid grid-cols-3 gap-3">
          {catalog.map((c) => {
            const qty = cart[c.class_name] || 0
            const sold = c.stock <= 0          // 재고 0 = 품절(주문 불가)
            const maxed = qty >= c.stock       // 재고만큼 다 담음 → 추가 차단
            return (
              <button
                key={c.class_name}
                disabled={sold}
                onClick={() => { if (!maxed) props.onAdd(c.class_name) }}
                className={`relative flex flex-col items-center gap-2 rounded-2xl border bg-white py-5 transition-all active:scale-95 ${
                  qty ? 'border-brand bg-brand-light' : 'border-line'
                } ${sold ? 'opacity-40' : ''}`}
              >
                {qty > 0 && (
                  <>
                    <span className="pop absolute right-2 top-2 flex h-6 min-w-6 items-center justify-center rounded-full bg-brand px-1.5 text-[13px] font-bold text-white">
                      {qty}
                    </span>
                    {/* 카드에서 바로 감소 (장바구니 안 열어도). 카드 본체 탭은 +. */}
                    <span
                      role="button"
                      onClick={(e) => { e.stopPropagation(); props.onSub(c.class_name) }}
                      className="absolute left-2 top-2 flex h-6 w-6 items-center justify-center rounded-full bg-amber-400 text-white shadow active:scale-90">
                      <Minus size={14} />
                    </span>
                  </>
                )}
                {sold && (
                  <span className="absolute left-2 top-2 rounded-full bg-muted px-1.5 text-[11px] font-bold text-white">
                    품절
                  </span>
                )}
                <span className="text-[42px]">{emojiOf(c.class_name)}</span>
                <span className="text-[15px] font-semibold text-ink">{nameOf(c.class_name)}</span>
                <span className={`text-[12px] font-medium ${sold ? 'text-muted' : 'text-sub'}`}>
                  {sold ? '품절' : `재고 ${c.stock}`}
                </span>
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
function PayPage({ total, onBack, onPaid }: {
  total: number; onBack: () => void; onPaid: () => void
}) {
  const [paid, setPaid] = useState(false)
  const [method, setMethod] = useState('')
  useEffect(() => {
    if (!paid) return
    const t = window.setTimeout(onPaid, 1500)   // 결제완료 보여주고 자동으로 다음
    return () => clearTimeout(t)
  }, [paid])

  const PAY_METHODS = [
    { key: 'card', label: '카드', emoji: '💳' },
    { key: 'kakao', label: '카카오페이', emoji: '🟡' },
    { key: 'naver', label: '네이버페이', emoji: '🟢' },
  ]

  if (paid) return (
    <div className="page-fade flex h-full flex-col items-center justify-center px-6">
      <span className="pop flex h-20 w-20 items-center justify-center rounded-full bg-brand text-white">
        <Check size={44} />
      </span>
      <div className="mt-5 text-[26px] font-bold text-ink">결제 완료!</div>
      <div className="mt-1 text-[16px] text-muted">잠시만 기다려주세요...</div>
    </div>
  )
  return (
    <div className="page-fade flex h-full flex-col">
      <div className="flex-1 overflow-y-auto px-6 py-5">
        <div className="mb-1 text-[22px] font-bold text-ink">결제 수단 선택</div>
        <div className="text-[15px] text-muted">총 {total}개 상품</div>
        <div className="mt-5 space-y-3">
          {PAY_METHODS.map((m) => {
            const on = method === m.key
            return (
              <button key={m.key} onClick={() => setMethod(m.key)}
                className={`flex w-full items-center gap-3 rounded-2xl border-2 px-5 py-4 text-[17px] font-bold transition-all active:scale-95 ${
                  on ? 'border-brand bg-brand-light text-brand' : 'border-line bg-white text-ink'
                }`}>
                <span className="text-2xl">{m.emoji}</span>
                <span className="flex-1 text-left">{m.label}</span>
                {on && <Check size={20} />}
              </button>
            )
          })}
        </div>
      </div>
      <div className="flex gap-2 border-t border-line px-6 pb-6 pt-4">
        <button onClick={onBack}
          className="rounded-2xl border border-line px-6 py-4 text-[16px] font-bold text-sub active:scale-95">뒤로</button>
        <div className="flex-1">
          <CTA onClick={() => method && setPaid(true)}>
            {method ? '결제하기' : '결제 수단을 선택하세요'}
          </CTA>
        </div>
      </div>
    </div>
  )
}

// 코드 표시 — _ _ _ _ 칸이 한 글자씩 채워짐. 현재 입력 위치 강조.
function CodeDisplay({ code, len = 4 }: { code: string; len?: number }) {
  return (
    <div className="flex justify-center gap-3">
      {Array.from({ length: len }).map((_, i) => {
        const filled = i < code.length
        const current = i === code.length
        return (
          <div key={i}
            className={`flex h-16 w-14 items-center justify-center rounded-2xl border-2 font-mono text-[34px] font-bold ${
              filled ? 'border-brand bg-brand-light text-ink'
                : current ? 'border-brand' : 'border-line'
            }`}>
            {filled ? <span className="code-pop">{code[i]}</span> : <span className="text-line">_</span>}
          </div>
        )
      })}
    </div>
  )
}

// 터치 숫자 키패드 — 키오스크용. 1~9, 0, 삭제(⌫).
function Keypad({ onDigit, onDelete }: { onDigit: (d: string) => void; onDelete: () => void }) {
  const KEY = 'rounded-xl border border-line bg-white py-3 text-[24px] font-bold active:scale-95 active:bg-gray-100'
  return (
    <div className="mt-3 grid grid-cols-3 gap-2">
      {['1', '2', '3', '4', '5', '6', '7', '8', '9'].map((k) => (
        <button key={k} onClick={() => onDigit(k)} className={`${KEY} text-ink`}>{k}</button>
      ))}
      <div />
      <button onClick={() => onDigit('0')} className={`${KEY} text-ink`}>0</button>
      <button onClick={onDelete} className={`${KEY} text-red-500`}>⌫</button>
    </div>
  )
}

function SetCodePage({ err, onBack, onSubmit }: {
  err: string; onBack: () => void; onSubmit: (code: string) => void
}) {
  const [code, setCode] = useState('')
  const ok = code.length === 4
  return (
    <div className="page-fade flex h-full flex-col">
      <div className="flex-1 overflow-y-auto px-6 py-5">
        <div className="mb-2 text-[22px] font-bold text-ink">수령 코드 설정</div>
        <div className="text-[15px] text-muted">수령할 때 입력할 4자리 숫자를 정해주세요</div>
        <div className="mt-7"><CodeDisplay code={code} /></div>
        <Keypad onDigit={(d) => setCode((c) => (c + d).slice(0, 4))}
          onDelete={() => setCode((c) => c.slice(0, -1))} />
        {err && (
          <div className="mt-3 rounded-xl bg-red-50 px-4 py-2.5 text-[14px] font-semibold text-red-600">{err}</div>
        )}
      </div>
      <div className="flex gap-2 border-t border-line px-6 pb-6 pt-4">
        <button onClick={onBack}
          className="rounded-2xl border border-line px-6 py-4 text-[16px] font-bold text-sub active:scale-95">뒤로</button>
        <div className="flex-1"><CTA onClick={() => ok && onSubmit(code)}>{ok ? '주문 완료' : '4자리를 입력하세요'}</CTA></div>
      </div>
    </div>
  )
}

function DonePage(props: {
  order: OrderResp | null
  receipt: ReceiptItem[]
  board: BoardRow[]
  notice: { order_id: string; status: string; locker_id: number; failed: string[]; all_ok: boolean } | null
  onHome: () => void
}) {
  const { order, receipt, board, notice } = props
  const ahead = order
    ? board.findIndex((b) => b.order_id === order.order_id)
    : -1
  const total = receipt.reduce((a, b) => a + b.qty, 0)

  // 15초 미동작 시 홈 복귀. 마지막 5초(10초 경과)부터 우측 상단 카운트다운 표시.
  const [left, setLeft] = useState(15)
  useEffect(() => {
    let n = 15
    setLeft(n)
    const reset = () => { n = 15; setLeft(n) }
    const iv = window.setInterval(() => {
      n -= 1
      setLeft(n)
      if (n <= 0) { clearInterval(iv); props.onHome() }
    }, 1000)
    window.addEventListener('pointerdown', reset)
    window.addEventListener('keydown', reset)
    return () => {
      clearInterval(iv)
      window.removeEventListener('pointerdown', reset)
      window.removeEventListener('keydown', reset)
    }
  }, [])

  return (
    <div className="page-fade relative flex h-full flex-col px-6">
      {left <= 5 && (
        <div className="absolute right-4 top-4 flex h-9 w-9 items-center justify-center rounded-full bg-red-500 text-[15px] font-bold text-white">
          {left}
        </div>
      )}
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

        {/* 수령 락커 + QR */}
        {order?.locker_id ? (
          <div className="mt-6 w-full rounded-2xl border-2 border-brand bg-brand-light p-5 text-center">
            <div className="text-[15px] font-semibold text-muted">수령 락커</div>
            <div className="text-[40px] font-bold leading-tight text-brand">{order.locker_id}번</div>
            {order.qr_token && (
              <>
                <img src={pickupQrUrl(order.qr_token)} alt="수령 QR" width={160} height={160}
                  className="mx-auto mt-3 rounded-lg bg-white p-1"
                  onError={(e) => { (e.currentTarget as HTMLImageElement).style.display = 'none' }} />
                <div className="mt-2 text-[13px] text-muted">수령 코드</div>
                <div className="font-mono text-[18px] font-bold tracking-widest text-ink">{order.qr_token}</div>
              </>
            )}
            <div className="mt-2 text-[14px] text-sub">상품 준비가 끝나면 이 QR로 수령하세요</div>
          </div>
        ) : null}

        {/* 배달 완료 알림 (order_done) — 준비완료 / 일부 실패 / 전체 실패 피드백 */}
        {notice && (
          <div className={`mt-4 w-full rounded-2xl p-4 text-center font-bold ${
            notice.all_ok ? 'bg-green-100 text-green-800'
              : notice.status === 'CANCELED' ? 'bg-red-100 text-red-800'
              : 'bg-amber-100 text-amber-900'
          }`}>
            {notice.all_ok
              ? `✅ 준비 완료! ${notice.locker_id}번 락커에서 수령하세요`
              : notice.status === 'CANCELED'
                ? '😢 주문을 처리하지 못했어요 (전 품목 실패) — 다시 주문해 주세요'
                : `⚠️ 일부 품목 실패: ${notice.failed.map(nameOf).join(', ')} — 나머지는 ${notice.locker_id}번 락커에서 수령하세요`}
          </div>
        )}

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

function PickupPage({ onBack }: { onBack: () => void }) {
  const videoRef = useRef<HTMLVideoElement | null>(null)
  const [info, setInfo] = useState<LockerInfo | null>(null)
  const [token, setToken] = useState('')   // 스캔/입력에 쓴 토큰 — 수령확인에 사용
  const [code, setCode] = useState('')
  const [msg, setMsg] = useState('')
  const [busy, setBusy] = useState(false)
  const handledRef = useRef(false)   // 중복 스캔 방지

  const lookup = async (raw: string) => {
    const t = raw.trim()
    if (!t || busy) return
    setBusy(true); setMsg('')
    try {
      const i = await scanPickup(t)
      setToken(t); setInfo(i)
    } catch (e) {
      setMsg(e instanceof Error ? e.message : '조회 실패')
      handledRef.current = false   // 실패 시 재스캔 허용
    } finally { setBusy(false) }
  }

  // 카메라 + 네이티브 BarcodeDetector(있으면). 없으면 코드 입력만.
  useEffect(() => {
    if (info) return   // 결과 화면에선 스캔 중지
    const BD = (window as unknown as { BarcodeDetector?: new (o: object) => {
      detect: (s: CanvasImageSource) => Promise<{ rawValue: string }[]> } }).BarcodeDetector
    // getUserMedia 자체가 없으면(=http로 IP 원격 접속 등 non-secure context) 카메라 불가.
    if (!navigator.mediaDevices?.getUserMedia) {
      setMsg('이 접속에선 카메라를 쓸 수 없어요 (http 원격 접속은 보안상 차단) — 코드를 직접 입력하세요')
      return
    }
    let stream: MediaStream | null = null
    let raf = 0
    // BarcodeDetector 없는 브라우저(Firefox/Safari)에서도 카메라는 띄운다(스캔만 비활성, 코드 입력 보조).
    const det = BD ? new BD({ formats: ['qr_code'] }) : null
    const run = async () => {
      try {
        // 권한 확보 후 디바이스 열거 → 원하는 카메라(deviceId)로 연결.
        // RealSense(depth) 카메라는 QR 인식에 부적합 → 제외하고 노트북 내장/일반 웹캠 우선.
        let constraint: MediaStreamConstraints = { video: { facingMode: 'environment' } }
        try {
          const probe = await navigator.mediaDevices.getUserMedia({ video: true })
          probe.getTracks().forEach((t) => t.stop())
          const cams = (await navigator.mediaDevices.enumerateDevices())
            .filter((d) => d.kind === 'videoinput')
          const isDepth = (c: MediaDeviceInfo) => /realsense|depth|intel|stereo|infrared/i.test(c.label)
          const webcams = cams.filter((c) => !isDepth(c))   // realsense 등 제외
          const pick = webcams[0] || cams[0]
          if (pick?.deviceId) constraint = { video: { deviceId: { exact: pick.deviceId } } }
        } catch { /* 열거 실패 시 기본 video */ }
        stream = await navigator.mediaDevices.getUserMedia(constraint)
        const v = videoRef.current
        if (!v) return
        v.srcObject = stream; await v.play()
        // 스캔 루프: BarcodeDetector(주로 안드로이드 크롬) 있으면 그걸로, 없으면(데스크톱
        // 크롬/파폭/Safari) jsQR로 canvas 디코드 — 브라우저 무관 QR 인식.
        const canvas = document.createElement('canvas')
        const ctx = canvas.getContext('2d', { willReadFrequently: true })
        const tick = async () => {
          if (handledRef.current || !videoRef.current) return
          const vid = videoRef.current
          try {
            let raw = ''
            if (det) {
              const codes = await det.detect(vid)
              raw = codes[0]?.rawValue || ''
            } else if (ctx && vid.videoWidth > 0) {
              canvas.width = vid.videoWidth
              canvas.height = vid.videoHeight
              ctx.drawImage(vid, 0, 0, canvas.width, canvas.height)
              const img = ctx.getImageData(0, 0, canvas.width, canvas.height)
              const r = jsQR(img.data, img.width, img.height, { inversionAttempts: 'dontInvert' })
              raw = r?.data || ''
            }
            if (raw && !handledRef.current) {
              handledRef.current = true
              lookup(raw)
              return
            }
          } catch { /* 프레임 스킵 */ }
          raf = requestAnimationFrame(tick)
        }
        raf = requestAnimationFrame(tick)
      } catch { setMsg('카메라를 열 수 없어요 — 코드를 직접 입력하세요') }
    }
    run()
    return () => {
      cancelAnimationFrame(raf)
      stream?.getTracks().forEach((t) => t.stop())
    }
  }, [info])

  return (
    <div className="page-fade flex h-full flex-col">
      <div className="flex-1 overflow-y-auto px-6 py-5">
        {/* 상단: 이전 / 제목 (주문 페이지와 동일) */}
        <div className="mb-4 flex items-center gap-3">
          <button onClick={onBack}
            className="flex items-center gap-1 rounded-lg px-2 py-1.5 text-[14px] font-semibold text-muted active:scale-95">
            <ChevronLeft size={18} /> 이전
          </button>
          <div className="text-[22px] font-bold text-ink">QR 수령</div>
        </div>
        <div className="flex flex-col items-center">
          {!info ? (
            <>
              <div className="text-[15px] text-muted">QR을 카메라에 비추거나 코드를 입력하세요</div>
              <div className="relative mt-3 aspect-[4/3] w-full max-w-[220px] overflow-hidden rounded-2xl border-2 border-line bg-black">
                <video ref={videoRef} className="h-full w-full object-cover" muted playsInline />
                <div className="pointer-events-none absolute inset-6 rounded-xl border-2 border-white/70" />
                <div className="absolute left-1/2 top-2 -translate-x-1/2 text-[13px] text-white/60">📷 QR 스캔</div>
              </div>
              <div className="mt-3 w-full max-w-xs">
                <div className="mb-2 text-center text-[13px] font-semibold text-muted">수령 코드 4자리 입력</div>
                <CodeDisplay code={code} />
                <Keypad onDigit={(d) => setCode((c) => (c + d).slice(0, 4))}
                  onDelete={() => setCode((c) => c.slice(0, -1))} />
                <button onClick={() => lookup(code)} disabled={busy || code.length !== 4}
                  className="mt-2 w-full rounded-xl bg-brand py-3 font-bold text-white active:scale-95 disabled:opacity-50">조회</button>
              </div>
              {msg && <div className="mt-3 text-[14px] font-semibold text-red-500">{msg}</div>}
            </>
          ) : (
            <PickupResult info={info} token={token} onDone={onBack}
              onRescan={() => { handledRef.current = false; setInfo(null); setToken(''); setCode(''); setMsg('') }} />
          )}
        </div>
      </div>
    </div>
  )
}

function PickupResult({ info, token, onDone, onRescan }: {
  info: LockerInfo; token: string; onDone: () => void; onRescan: () => void
}) {
  const [busy, setBusy] = useState(false)
  const [err, setErr] = useState('')
  const [done, setDone] = useState(false)

  const confirm = async () => {
    setBusy(true); setErr('')
    try {
      await confirmPickup(token)   // 스캔/입력에 쓴 토큰으로 수령 확정
      setDone(true)
    } catch (e) {
      setErr(e instanceof Error ? e.message : '수령 처리 실패')
    } finally { setBusy(false) }
  }

  if (done) return (
    <div className="flex flex-col items-center pt-10">
      <span className="pop flex h-16 w-16 items-center justify-center rounded-full bg-brand text-white"><Check size={36} /></span>
      <div className="mt-4 text-[24px] font-bold text-ink">수령 완료!</div>
      <div className="mt-1 text-[15px] text-muted">이용해 주셔서 감사합니다</div>
      <button onClick={onDone} className="mt-8 rounded-xl bg-brand px-8 py-3 font-bold text-white">홈으로</button>
    </div>
  )

  return (
    <div className="flex w-full flex-col items-center pt-4">
      <div className={`flex h-16 w-16 items-center justify-center rounded-full text-[30px] ${info.ready ? 'bg-brand' : 'bg-amber-400'}`}>
        📦
      </div>
      <div className="mt-4 text-[15px] font-semibold text-muted">수령 락커</div>
      <div className="text-[48px] font-bold leading-none text-brand">{info.locker_id}번</div>
      <div className={`mt-3 rounded-full px-4 py-1.5 text-[14px] font-bold ${
        info.ready ? 'bg-green-100 text-green-800' : 'bg-amber-100 text-amber-900'
      }`}>
        {info.ready ? '수령 가능 — 락커에서 꺼내세요' : '아직 준비 중이에요'}
      </div>
      {err && <div className="mt-3 text-[14px] font-semibold text-red-500">{err}</div>}
      <div className="mt-8 w-full max-w-xs space-y-2">
        {info.ready && (
          <button onClick={confirm} disabled={busy}
            className="w-full rounded-xl bg-brand py-3.5 text-[17px] font-bold text-white active:scale-95 disabled:opacity-50">
            수령 완료
          </button>
        )}
        <button onClick={onRescan} className="w-full rounded-xl border-2 border-line py-3 text-[15px] font-semibold text-muted">
          다시 스캔
        </button>
      </div>
    </div>
  )
}
