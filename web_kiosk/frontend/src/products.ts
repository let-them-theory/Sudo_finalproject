// 상품 class → 표시명·이모지. 백엔드는 class_name만 다루고, 표시는 프론트가 담당.
export const PRODUCTS: Record<string, { name: string; emoji: string }> = {
  ramen: { name: '라면', emoji: '🍜' },
  pack: { name: '팩음료', emoji: '🧃' },
  ssnack: { name: '스낵', emoji: '🍪' },
  bsnack: { name: '봉지과자', emoji: '🍿' },
  water: { name: '생수', emoji: '💧' },
  jelly: { name: '젤리', emoji: '🍮' },
  box: { name: '박스', emoji: '📦' },
  can: { name: '캔', emoji: '🥫' },
  boxsnack: { name: '박스과자', emoji: '🍫' },
  wafers: { name: '웨하스', emoji: '🧇' },
}

export const nameOf = (c: string) => PRODUCTS[c]?.name ?? c
export const emojiOf = (c: string) => PRODUCTS[c]?.emoji ?? '🛒'
