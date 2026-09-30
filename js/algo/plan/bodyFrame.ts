// ==================== 算法：本体坐标系（近场判据的基准） ====================
//
// 【为什么近场要用本体坐标系，而不是 Frenet】
// Frenet（弧长 s / 横向 l）是**道路**坐标系：它依赖地图参考线与投影口径，远处/弯道上任何
// 参考线误差都会串进"这算不算我的障碍"这个判断（本会话的幽灵刹车正是这么来的 ✗）。
// 而"它离我多近、偏我多少"在**本体坐标系**里是**定义即精确**的：x 前、y 左，用自车航向角旋转即可，
// 不依赖地图、不依赖投影 —— 这是业界近场判断的标准做法（远场/道路级才用 Frenet）。
//
// 【安全方向】近场用途是**加约束**（"它就在我车头侧前方 ⇒ 当障碍处理"），
// **不用它解除**任何已判定的阻塞 —— 所以它不可能让车撞上东西 ✓。
//
// 【量纲】全部为仿真单位（px）；角度为 rad。

import { rad } from '../../shared/dimensions.js';

/** 本体坐标系下的偏移（px）：ahead = 沿自车前进方向（正=前），lat = 左正右负 */
export interface BodyOffset {
  ahead: number;
  lat: number;
}

/** 世界坐标点 → 本体坐标系（自车 x/y/angle 为基准） */
export function bodyFrameOffset(
  car: { x: number; y: number; angle: number },
  x: number, y: number,
): BodyOffset {
  const c = Math.cos(car.angle), s = Math.sin(car.angle);
  const dx = x - car.x, dy = y - car.y;
  return {
    ahead: dx * c + dy * s,      // 前向分量
    lat: -dx * s + dy * c,       // 左向分量（逆时针 90°）
  };
}

/** 近场判据：本体坐标系下"是否在我车头正前方的一条窄带里"（用于加约束） */
export function nearFieldBlocking(
  car: { x: number; y: number; angle: number },
  x: number, y: number,
  carHalfWidth: number, bandMargin: number, nearField: number,
): boolean {
  const o = bodyFrameOffset(car, x, y);
  if (o.ahead <= 0 || o.ahead > nearField) return false;       // 只算车头前方、近场之内
  return Math.abs(o.lat) <= carHalfWidth + bandMargin;
}

/** 近场距离（px）：超过它就用道路坐标系（Frenet）判断 */
export const NEAR_FIELD = 15 * 4;
/** 近场窄带的横向余量（px）：0.3m */
export const NEAR_BAND_MARGIN = 0.3 * 4;
