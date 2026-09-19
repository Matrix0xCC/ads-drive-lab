// ==================== 算法：速度层（ST 空间） ====================
//
// 【职责】给定路径层的阻塞信息（ST 占据条目），生成纵向速度剖面 s(t)：
//   cruise    — 无阻塞：收敛到巡航目标 targetV
//   stop      — 有阻塞且物理可停：在障碍前 STOP_MARGIN 处减速到 0（停车线）
//   emergency — 物理停不住（距离 < v0²/2A_MAX）：最大减速兜底（接受接触）
//
// 【ST 图】接口按"障碍在 s-t 平面占据区间"通用设计（可承载动态障碍），
// v1 静态实现：只处理 tMin===0 的条目（占据现在起的所有时间）；tMin>0 忽略。
//
// 【停车线整定】T 取最小单调解（自然时长 2·sStop/v0 起 ×1.15 递增至速度非负）。
// 峰值减速度由距离决定（该多项式族的最小峰值即在最小单调解处），不通过延长 T 下调。
//
// 【职责分工】路径层判"几何上过得去"（小间隙）；速度层管"安全距离"
// （刹车/停车线）——解耦架构的核心分工（见 spec 2026-09-19-decoupled-planner）。

import { m, mps2, px, pxps, pxps2, dimless, perpx, secs } from '../../shared/dimensions.js';
import { solveQuintic, solveSpeedProfile } from './polynomial.js';

export interface STObstacle { sMin: number; sMax: number; tMin: number; tMax: number; }
export type SpeedMode = 'cruise' | 'stop' | 'emergency';

export interface SpeedProfile {
  eval(t: number): number;
  deriv1(t: number): number;
  duration: number;
}

export interface SpeedLayerResult { profile: SpeedProfile; mode: SpeedMode; }

export const SPEED_HORIZON = 2.0;
export const STOP_MARGIN = m(4);
export const A_COMFORT = mps2(3.5);
export const A_MAX = mps2(6.0);
export const T_STOP_MIN = 0.8;

export function solveSpeedLayer(o: { stObstacles: STObstacle[]; v0: number; targetV: number }): SpeedLayerResult {
  // 第一个站在路径正前方的静态障碍（sMin ≥ 0；tMin 必须为 0 = 现在起持续占据）
  const ahead = o.stObstacles
    .filter(ob => ob.tMin === 0 && ob.sMin >= 0)
    .sort((a, b) => a.sMin - b.sMin)[0];

  if (!ahead) {
    const poly = solveSpeedProfile(pxps(o.v0), pxps2(0), pxps(o.targetV), secs(SPEED_HORIZON));
    return {
      mode: 'cruise',
      profile: { eval: t => poly.eval(t), deriv1: t => poly.deriv1(t), duration: SPEED_HORIZON },
    };
  }

  const sStop = ahead.sMin - (STOP_MARGIN as number);
  const v0 = o.v0;

  // 物理可行性：停车距离不足 → 最大减速兜底。
  // 五次多项式族的形状关系：D ≈ v0·T/2、peak ≈ 1.875·v0/T ⇒ 峰值恰为 A_MAX 时
  // 所需距离 D_min = 1.875·v0²/(2·A_MAX)（比线性制动的 v0²/2A_MAX 大 1.875 倍）。
  if (sStop <= 0 || sStop < 1.875 * (v0 * v0) / (2 * (A_MAX as number))) {
    const T = Math.max(T_STOP_MIN, v0 / (A_MAX as number));
    const poly = solveSpeedProfile(pxps(v0), pxps2(0), pxps(0), secs(T));
    return {
      mode: 'emergency',
      profile: { eval: t => poly.eval(t), deriv1: t => poly.deriv1(t), duration: T },
    };
  }

  // 停车线剖面：s(0)=0, s'(0)=v0, s''(0)=0, s(T)=sStop, s'(T)=0, s''(T)=0
  // T 取"最小单调解"：自然时长略短时五次多项式会过冲 sStop 再折返（非单调），
  // 从自然时长起 ×1.15 递增直到速度全程非负。
  // 注：该族的最小峰值减速度即在最小单调解处（继续延长 T 会前载加剧峰值，
  // 与直觉相反）——"是否舒适"由距离决定，不通过延长 T 调整。
  let T = Math.max((2 * sStop) / v0, T_STOP_MIN);
  let poly = solveQuintic(px(0), dimless(v0), perpx(0), px(sStop), dimless(0), perpx(0), T);
  for (let i = 0; i < 12; i++) {
    let minVel = Infinity;
    for (let k = 0; k <= 24; k++) {
      minVel = Math.min(minVel, poly.deriv1((k / 24) * T) as number);
    }
    if (minVel >= -1) break;   // 允许 1u/s 数值容差
    T *= 1.15;
    poly = solveQuintic(px(0), dimless(v0), perpx(0), px(sStop), dimless(0), perpx(0), T);
  }

  return {
    mode: 'stop',
    profile: { eval: t => poly.eval(t) as number, deriv1: t => poly.deriv1(t) as number, duration: T },
  };
}
