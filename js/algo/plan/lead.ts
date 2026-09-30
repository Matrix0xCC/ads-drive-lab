// ==================== 算法：跟驰目标选取（从物体层挑"前车"） ====================
//
// 【为什么单独成模块】规划器里"挑哪辆车当跟随目标"这段逻辑必须**可单测**：
// 它是 C 方案的入口，选错了（比如把相邻车道的车、或后方的车当成前车）会直接影响纵向决策。
//
// 【规则】我的车道上、前方、已确认、且在动的物体里**最近**的那个。
// 【安全门·相对速度形式】跟驰要问的不是"能不能停下"（那是对静止障碍），而是
//   **"能不能在追上前把速度降到前车的速度"**：需要 (v² − v_lead²)/(2a) 的距离。
//   前车静止时它退化成制动距离 ✓（所以静态场景行为不变）。留 SAFETY_MARGIN 余量。
//   门不过 ⇒ 不返回跟随目标 ⇒ 调用方按静态障碍处理（停车线/急刹照旧）—— C 不能削弱安全。

import { m, mps } from '../../shared/dimensions.js';

/** 跟驰安全门余量（px） */
export const FOLLOW_SAFETY_MARGIN = m(10) as number;
/** 认为"在动"的最小速度（px/s）；低于它按静态障碍走停车线 */
export const FOLLOW_MIN_SPEED = mps(1.5) as number;

export interface LeadCandidate {
  /** Frenet 纵向位置（px） */
  s: number;
  /** Frenet 横向位置（px） */
  l: number;
  /** 沿路速度（px/s） */
  vS: number;
  /** 是否已确认（连续命中） */
  confirmed: boolean;
}

/**
 * 选出跟驰目标。
 *
 * @param tracks   物体层的轨迹（Frenet）
 * @param egoS     自车弧长（px）
 * @param egoL     自车横向（px）
 * @param egoV     自车速度（px/s）
 * @param aMax     自车减速能力（px/s²）
 * @param laneCenter 本车道中心（px）
 * @param laneHalf   本车道半宽 + 余量（px）
 * @param closedLoop 参考线是否闭环（决定 s 是否需要绕接）
 * @param totalS     参考线总长（px；闭环用）
 * @returns 前车（gap / leadSpeed）或 null
 */
export function selectFollowTarget(
  tracks: LeadCandidate[],
  egoS: number, egoL: number, egoV: number, aMax: number,
  laneCenter: number, laneHalf: number,
  closedLoop: boolean, totalS: number,
): { gap: number; leadSpeed: number } | null {
  let best: { gap: number; leadSpeed: number } | null = null;
  for (const t of tracks) {
    if (!t.confirmed) continue;
    let gap = t.s - egoS;
    if (closedLoop && gap < -totalS / 2) gap += totalS;      // 闭环：绕回来的也算"前面"
    if (gap <= 0) continue;                                   // 在我后方
    if (Math.abs(t.l - laneCenter) > laneHalf) continue;       // 不在我这条车道
    if (t.vS < FOLLOW_MIN_SPEED) continue;                     // 基本静止 → 按静态障碍
    const need = Math.max(0, (egoV * egoV - t.vS * t.vS) / (2 * Math.max(1e-6, aMax))) + FOLLOW_SAFETY_MARGIN;
    if (gap <= need) continue;                                 // 降不到前车速度 → 不许跟
    if (!best || gap < best.gap) best = { gap, leadSpeed: t.vS };
  }
  return best;
}
