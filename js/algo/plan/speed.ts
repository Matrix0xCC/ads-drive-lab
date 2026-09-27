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

import { m, mps, mps2, px, pxps, pxps2, dimless, perpx, secs } from '../../shared/dimensions.js';
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
/** 跟驰（方案 C）参数：期望车头时距、静止车距、判定"前车确实在动"的速度阈值 */
export const HEADWAY_T = 1.4;                                  // s
export const FOLLOW_MIN_GAP = m(6);                            // 车头到前车尾的最小间距
/** 低于它按"静止障碍"处理 —— 这条是**逐位不变**的保险：无交通流的场景 v_obs≈0 → 走老逻辑 */
export const FOLLOW_MIN_LEAD_SPEED = mps(1.5);
export const STOP_MARGIN = m(4);
export const A_COMFORT = mps2(3.5);
export const A_MAX = mps2(6.0);
export const T_STOP_MIN = 0.8;

export function solveSpeedLayer(o: {
  stObstacles: STObstacle[];
  v0: number;
  targetV: number;
  /** 前车估计（见 algo/plan/closing.ts）。缺省 undefined = 纯静态世界（历史行为逐位一致） */
  follow?: { gap: number; leadSpeed: number };
}): SpeedLayerResult {
  // 第一个站在路径正前方的静态障碍（sMin ≥ 0；tMin 必须为 0 = 现在起持续占据）
  const ahead = o.stObstacles
    .filter(ob => ob.tMin === 0 && ob.sMin >= 0)
    .sort((a, b) => a.sMin - b.sMin)[0];

  // ── 跟驰（方案 C）：前车在动，且路径层的阻塞点**就是这辆前车** → 不停车，改为"跟到它的速度" ──
  // 判据用"阻塞点与雷达最近障碍的距离是否吻合"：吻合说明阻塞来自这辆会走的车；
  // 不吻合说明前面还有**静态**障碍（它才该决定停车线）→ 走老逻辑。
  const lead = o.follow;
  if (ahead && lead && lead.leadSpeed > (FOLLOW_MIN_LEAD_SPEED as number)
      && Math.abs(ahead.sMin - lead.gap) < (m(8) as number)) {
    const gap = Math.max(0, lead.gap);
    // 安全速度 = 前车速度 + (富余间距 / 时距)：间距 = 最小车距时恰好等于前车速度（稳态跟车），
    // 间距更大时允许略快地接近（受 targetV 上限约束），更小时自动降到前车速度之下（拉开距离）。
    const vSafe = Math.max(0, lead.leadSpeed + (gap - (FOLLOW_MIN_GAP as number)) / HEADWAY_T);
    const vTarget = Math.max(0, Math.min(o.targetV, vSafe));
    const poly = solveSpeedProfile(pxps(o.v0), pxps2(0), pxps(vTarget), secs(SPEED_HORIZON));
    return {
      mode: 'cruise',
      profile: { eval: t => poly.eval(t), deriv1: t => poly.deriv1(t), duration: SPEED_HORIZON },
    };
  }

  if (!ahead) {
    const poly = solveSpeedProfile(pxps(o.v0), pxps2(0), pxps(o.targetV), secs(SPEED_HORIZON));
    return {
      mode: 'cruise',
      profile: { eval: t => poly.eval(t), deriv1: t => poly.deriv1(t), duration: SPEED_HORIZON },
    };
  }

  const sStop = ahead.sMin - (STOP_MARGIN as number);
  const v0 = o.v0;

  // 已停住（v0≈0，与控制的"停车"阈值一致）且前方有停车线 → 静止剖面。
  // 必须单独处理：T = 2·sStop/v0 在 v0=0 时为 Infinity → 五次多项式系数 NaN →
  // 停车后每帧产出 NaN 轨迹（归档为 null、回放 HUD 崩溃，2026-09-24 实测）。
  if (v0 < 1) {
    return {
      mode: 'stop',
      profile: { eval: () => 0, deriv1: () => 0, duration: T_STOP_MIN },
    };
  }

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
