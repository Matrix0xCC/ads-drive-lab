// ==================== 算法：误差感知裕度（margin，物理走廊膨胀用） ====================
//
// 【职责】给出"车半宽之外还需要多少额外余量"——横向跟踪误差的实测分布决定物理
// 走廊该膨胀多少（也即"紧间隙还敢不敢试"的总旋钮）。
//
// 【形式：速度分段表（数据驱动，2026-09-25 标定）】原设计假设 margin = m₀ + k·κ·v²
// （随曲率×速度²增长），**标定否决了该形式**（k 拟合为 0）：实测横向偏差与 κ·v²
// 无关，而是**随速度下降**——低速段（2.5~7.5m/s）偏差最大（e95 ≈ 0.8~1.2m），
// 高速稳态段最小（e95 ≈ 0.3~0.5m）。低速段恰是机动段（避障横移/紧间隙慢行/
// 规划目标切换），高速段是稳态跟踪。
//   标定数据与分箱表见 docs/margin-calibration.md（js/headless/calibrate-margin.ts 可复跑）。
//
// 表（速度分段线性插值，v 单位 m/s；裕度单位 m）：取"达到留出集覆盖率 ≥95% 的
// 最小表"（逐候选表在留出集上评估，95.7% 达标；更小的表 94.2% 不达标）：
//     v:      0     5     10    15    25    35
//     margin: 1.15  1.00  0.85  0.75  0.65  0.60
//
// 【取舍】该表偏保守：低速段 1.15m 会把大量"紧间隙慢行"判成封死（更多干净停车、
// 零接触）；"敢不敢试紧线"的裁决交给阶段 C 的路径-速度联合决策。

import { FrenetReference } from '../../shared/types.js';
import { m } from '../../shared/dimensions.js';

/** 分段表节点（速度 m/s → 裕度 m） */
const MARGIN_TABLE: { v: number; m: number }[] = [
  { v: 0, m: 1.15 },
  { v: 5, m: 1.0 },
  { v: 10, m: 0.85 },
  { v: 15, m: 0.75 },
  { v: 25, m: 0.65 },
  { v: 35, m: 0.6 },
];
export const MARGIN_MIN = 0.4;
export const MARGIN_MAX = 1.2;

/** 误差感知裕度（m）：速度分段线性插值，clamp 到 [0.4, 1.2] */
/**
 * 【A 方案·侧向间隙政策】**移动**物体（物体层给出的、有速度的物体）的侧向裕度附加量。
 *
 * 【为什么与静态障碍不同】"超越/被超越一辆车"本来就是贴着过去的：真实驾驶里两车中心距 3.75m
 * （相邻车道）、车体间隙 ~1.9m 属于常态。而静态障碍（墙、锥桶、抛锚车）你并不预期它会动，
 * 必须留足裕度。所以两者用不同政策：
 *   · 静态障碍：车半宽 + `marginFor(v)`（标定速度表，≈1.5m @高速）
 *   · 移动物体：车半宽 + `MARGIN_MOVING_EXTRA`（0.6m）
 * 用户实测的"旁边超车就横移"就是被前者逼出来的（相邻车道的车把物理走廊压到本车道里 ✗）。
 *
 * 【边界】这**不是**取消安全裕度：0.6m 仍大于零，且物体层的速度是**有物理门与硬夹**的估计。
 * 判据只看"物体层说它在动（|v| > 1.5m/s）"，静止/未知的一律按静态障碍处理（保守）。
 */
export const MARGIN_MOVING_EXTRA = m(0.6) as number;

/** A 方案总开关（A/B 用）：关掉 = 移动物体也用静态障碍的裕度（等价于 A 之前的行为）。 */
export const MARGIN_MOVING_ENABLED = true;

export function marginFor(vMps: number): number {
  const v = Math.max(0, vMps);
  const last = MARGIN_TABLE[MARGIN_TABLE.length - 1];
  if (v >= last.v) return Math.max(MARGIN_MIN, Math.min(MARGIN_MAX, last.m));
  for (let i = 0; i < MARGIN_TABLE.length - 1; i++) {
    const a = MARGIN_TABLE[i], b = MARGIN_TABLE[i + 1];
    if (v >= a.v && v <= b.v) {
      const t = (v - a.v) / ((b.v - a.v) || 1);
      return Math.max(MARGIN_MIN, Math.min(MARGIN_MAX, a.m + t * (b.m - a.m)));
    }
  }
  return MARGIN_MIN;
}

/**
 * 参考线在弧长 s 处的曲率（1/u）：取 s±2.5m 两个参考点，三点外接圆近似。
 * 标定工具（calibrate-margin.ts）用于 (v,κ) 分箱（κ 维度最终被数据否决，
 * 但保留该口径以便复现"κ 与偏差无关"的结论）。
 */
export function refCurvatureAt(ref: FrenetReference, s: number): number {
  const D = 10;   // ±2.5m（仿真单位）
  const n = ref.points.length;
  const at = (target: number) => {
    // 二分查找 s 所在段（ref.s 单调）
    let lo = 0, hi = n - 1;
    while (lo < hi) {
      const mid = (lo + hi) >> 1;
      if (ref.s[mid] < target) lo = mid + 1; else hi = mid;
    }
    const i = Math.max(1, Math.min(n - 1, lo));
    const t = (target - ref.s[i - 1]) / ((ref.s[i] - ref.s[i - 1]) || 1);
    return {
      x: ref.points[i - 1].x + t * (ref.points[i].x - ref.points[i - 1].x),
      y: ref.points[i - 1].y + t * (ref.points[i].y - ref.points[i - 1].y),
    };
  };
  const a = at(s - D), b = at(s), c = at(s + D);
  // 三点外接圆：κ = 1/R = 2·Area2 / (|ab|·|bc|·|ca|)（Area2 = 叉积绝对值）
  const ab = Math.hypot(b.x - a.x, b.y - a.y);
  const bc = Math.hypot(c.x - b.x, c.y - b.y);
  const ca = Math.hypot(a.x - c.x, a.y - c.y);
  const area2 = Math.abs((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x));
  const denom = ab * bc * ca;
  return denom > 1e-9 ? (2 * area2) / denom : 0;
}
