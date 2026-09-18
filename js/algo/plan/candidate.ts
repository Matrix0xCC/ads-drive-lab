// ==================== 候选轨迹生成 + 代价评估 ====================
//
// 【轨迹生成】
//   对每组 (targetL, targetV)：
//     1. 纵向五次多项式 s(t)：从 v0 到 targetV，边界 s'(0)=v0, s''(0)=0, s'(Tf)=vf, s''(Tf)=0
//     2. 横向五次多项式 l(s_rel)：从 l0 到 targetL，边界 l'(0)=dl0, l''(0)=0, l'(Sf)=0, l''(Sf)=0
//     3. 离散采样：t_i → s(t_i) → l(s_rel_i) → FrenetToCartesian → 笛卡尔点
//
// 【代价评估】
//   Cost = W_collision × 碰撞 + W_center × 中心偏离² + W_latAcc × 横向加速度
//        + W_jerk × 加加速度 + W_progress × 进度奖励
//
//   碰撞代价 1e6 是硬约束（有碰撞的轨迹绝对不可选）。
//   其他项是舒适性和效率的软约束。

import {
  FrenetReference, TrajectoryPoint, LidarScan,
} from '../../shared/types.js';
import {
  Px, PxPerSec, PxPerSec2, DimLess, PerPx, Seconds,
  pxps, dimless, perpx, secs,
} from '../../shared/dimensions.js';
import { solveQuintic, solveSpeedProfile } from './polynomial.js';
import { frenetToCartesian, cartesianToFrenet } from './frenet.js';
import { isLidarCollision, SAFETY_MARGIN } from './collision.js';

/**
 * 代价权重
 *
 * W_COLLISION = 1e6 — 硬约束。任何碰撞都不可接受，直接淘汰。
 * W_CENTER = 0.3    — 偏离中心线每 10px 损失约 30 分
 * W_LAT_ACC = 1.0   — 横向加速度每 100px/s² 损失约 100 分
 * W_JERK = 0.1      — 加加速度惩罚（方向盘突变）
 * W_PROGRESS = -0.5 — 负奖励：走得越远代价越低，鼓励高速前进
 *
 * 典型代价分配（无障碍平坦直道，v=180, l=0）：
 *   center≈0 + latAcc≈0 + jerk≈0 + progress≈-180 ≈ -180（正常巡航）
 * 典型代价分配（避障 l=±40, v=100）：
 *   center≈480 + latAcc≈50 + jerk≈10 + progress≈-100 ≈ 440（避障代价）
 */
const W_COLLISION = 1e6;
const W_JERK      = 0.1;
const W_LAT_ACC   = 1.0;
const W_PROGRESS  = -0.5;
const W_CENTER    = 0.3;

// ═══════════════════════════════════════════════════
// 轨迹生成
// ═══════════════════════════════════════════════════

/**
 * 生成一条候选轨迹
 *
 * 【生成管线】
 *   1. solveSpeedProfile(v0, 0, targetV, T) → 纵向多项式 s(t)
 *      s(0)=0, s'(0)=v0, s''(0)=0
 *      s(T)=sf_guess, s'(T)=targetV, s''(T)=0
 *   2. latSf = max(50, s(T)) → 横向规划总弧长
 *      （至少 50px，防止极低速时 Sf 太小导致多项式系数爆炸）
 *   3. solveQuintic(l0, dl0, 0, targetL, 0, 0, latSf) → 横向多项式 l(s_rel)
 *      l(0)=l0, l'(0)=dl0, l''(0)=0
 *      l(Sf)=targetL, l'(Sf)=0, l''(Sf)=0
 *   4. 离散采样：t_i → s(t_i) → l(s_rel_i) → frenetToCartesian
 *
 * 【关键参数 dl0】
 *   dl0 = dl/ds = tan(headingDiff)，无量纲斜率。
 *   如果车辆正朝外漂，dl0 ≠ 0，多项式从正确初始斜率开始，
 *   生成更响应的轨迹（避免初始段过于平坦）。
 */
export function generateCandidate(
  l0: Px, dl0: DimLess, ddl0: PerPx,
  s0: number, ds0: PxPerSec, dds0: PxPerSec2,
  targetL: Px, targetV: PxPerSec,
  ref: FrenetReference,
  duration: Seconds, steps: number,
): TrajectoryPoint[] {
  // 纵向：s(t)
  const lonPoly = solveSpeedProfile(ds0, dds0, targetV, duration);
  // 横向规划距离 ≥ 50px（保证多项式数值稳定）
  const latSf = Math.max(50, lonPoly.eval(duration as number));
  // 横向：l(s_rel)
  const latPoly = solveQuintic(l0, dl0, ddl0, targetL, dimless(0), perpx(0), latSf);

  const points: TrajectoryPoint[] = [];
  for (let i = 0; i <= steps; i++) {
    const t = (i / steps) * (duration as number);
    const siRel = lonPoly.eval(t);        // 相对弧长（从 0 开始）
    const siAbs = s0 + siRel;             // 绝对弧长（从车辆当前 s 开始）
    const li = latPoly.eval(siRel);       // 横向偏移
    const cart = frenetToCartesian(siAbs, li, ref);
    points.push({ t: secs(t), x: cart.x, y: cart.y, yaw: cart.yaw, v: pxps(lonPoly.deriv1(t)), kappa: perpx(0) });
  }
  return points;
}

// ═══════════════════════════════════════════════════
// 代价评估
// ═══════════════════════════════════════════════════

/**
 * 评估轨迹代价
 *
 * 【逐点计算】
 *   - collision: 检查每个轨迹点是否在 LiDAR 检测到的障碍物后方
 *   - center:    l²（横向偏移平方）沿轨迹积分
 *   - latAcc:    v² × κ（曲率由相邻点方向变化估算）
 *   - jerk:      曲率变化率（κ 对时间的一阶导数）
 *   - progress:  -1 × 轨迹端点时间 × 端点速度（负值为奖励）
 *
 * 【碰撞检测与剪枝】
 *   原始 isLidarCollision 返回 true 时，轨迹已确定不安全。
 *   第一次碰撞后立即 break，后续点不评估（cost += 1e6，轨迹被淘汰）。
 *
 * 【横向加速度估算】
 *   κ_i ≈ Δyaw_i / (v_i × Δt_i)  — 方向变化率除以速度
 *   a_lat_i = v_i² × κ_i          — 横向加速度 ∝ 速度平方 × 曲率
 *
 *   这是近似值——真实横向加速度应使用轨迹点 Frenet l 的二阶导数。
 *   但对于代价排序来说精度足够：更弯曲的轨迹得到更高的 latAcc 惩罚。
 */
export function evaluateTrajectory(
  points: TrajectoryPoint[],
  ref: FrenetReference,
  carX: number, carY: number,
  lidarScan: LidarScan,
  laneCenter: number = 0,
): number {
  let cost = 0, jerkSum = 0, latAccSum = 0, centerSum = 0;

  for (let i = 0; i < points.length; i++) {
    const pt = points[i];
    // 硬约束：碰撞检测
    if (isLidarCollision(pt.x as number, pt.y as number, carX, carY, lidarScan, SAFETY_MARGIN as number)) {
      cost += W_COLLISION; break;
    }
    // 中心偏离
    const fl = cartesianToFrenet(pt.x, pt.y, ref);
    const dl = (fl.l as number) - laneCenter; centerSum += dl * dl;

    // 舒适性（需要 3 个连续点计算二阶变化）
    if (i > 1) {
      const prev = points[i - 1];
      const dyaw = (pt.yaw as number) - (prev.yaw as number);
      const dt = (pt.t as number) - (prev.t as number);
      if (dt > 1e-6) {
        const kappa = dyaw / ((prev.v as number) * dt + 1e-6);
        latAccSum += Math.abs((pt.v as number) * (pt.v as number) * kappa);
        jerkSum += Math.abs((kappa - dyaw / ((prev.v as number) * dt + 1e-6)) / (dt + 1e-6));
      }
    }
  }

  const n = points.length || 1;
  cost += W_JERK     * jerkSum / n;
  cost += W_LAT_ACC  * latAccSum / n;
  // 进度奖励：越长越好，越快越好
  cost += W_PROGRESS * (points[points.length - 1]?.t as number ?? 0) * (points[points.length - 1]?.v as number ?? 0);
  cost += W_CENTER   * centerSum / n;
  return cost;
}
