// ==================== 五次多项式求解器（Frenet 轨迹生成核心） ====================
//
// 【为什么是五次多项式？】
// 三次多项式：4 个系数，只能满足 4 个边界条件（起点位置+速度，终点位置+速度）。
//   会导致加速度不连续——方向盘突然跳动，对真实车辆不可接受。
// 五次多项式：6 个系数，额外满足加速度边界条件，保证曲率 C² 连续。
//   这是 Apollo/ Waymo 等 L4 系统的标准做法。
//
// 【解析解（闭式公式）】
//   l(s) = c₀ + c₁s + c₂s² + c₃s³ + c₄s⁴ + c₅s⁵
//
//   6 个边界条件 → 6 个线性方程 → 6 个系数的解析解：
//     起点 (s=0): l=l0,  l'=dl0,  l''=ddl0
//     终点 (s=Sf): l=lf,  l'=dlf,  l''=ddlf
//
//   c₀ = l0
//   c₁ = dl0
//   c₂ = ddl0 / 2
//   c₃ = -(10l₀-10lf + 6dl₀S + 4dlfS + 1.5ddl₀S² - 0.5ddlfS²) / S³
//   c₄ =  (15l₀-15lf + 8dl₀S + 7dlfS + 1.5ddl₀S² -      ddlfS²) / S⁴
//   c₅ = -( 6l₀- 6lf + 3dl₀S + 3dlfS + 0.5ddl₀S² - 0.5ddlfS²) / S⁵
//
// 【量纲】（由 branded types 在编译时保证）
//   eval(s)    → l(s)   → Px     （横向偏移量）
//   deriv1(s)  → l'(s)  → DimLess（dl/ds，无量纲斜率）
//   deriv2(s)  → l''(s) → PerPx  （d²l/ds²，曲率的空间变化率）

import {
  Px, PxPerSec, PxPerSec2, DimLess, PerPx, Seconds,
  px, dimless, perpx,
} from '../../shared/dimensions.js';

/**
 * 求解五次多项式 l(s) — 横向轨迹规划
 *
 * 【使用场景】
 * 横向轨迹：l(s_rel) 从当前偏移 l0 到目标偏移 targetL
 * 纵向速度：s(t) 从当前速度 v0 到目标速度 targetV
 * 两种多项式使用相同的求解器，只是在调用时传入不同的参数。
 *
 * 【边界条件典型值】
 *   横向 l(s)：l'(0)=dl0 (tan(headingDiff)), l''(0)=0, l'(Sf)=0, l''(Sf)=0
 *   纵向 s(t)：s(0)=0, s'(0)=v0, s''(0)=0, s'(Tf)=vf, s''(Tf)=0
 *
 * 【退化保护】
 *   Sf < 1e-6 时自动设为 1，防止除零导致的 NaN/Infinity。
 */
export function solveQuintic(
  l0: Px, dl0: DimLess, ddl0: PerPx,
  lf: Px, dlf: DimLess, ddlf: PerPx,
  Sf: number,
) {
  const L0 = l0 as number, D0 = dl0 as number, DD0 = ddl0 as number;
  const Lf = lf as number, Df = dlf as number, DDf = ddlf as number;

  let s = Sf;
  if (Math.abs(s) < 1e-6) s = 1;  // 退化保护：Sf≈0 → 退化为单位长度
  const S2 = s * s, S3 = S2 * s, S4 = S3 * s, S5 = S4 * s;
  const c0 = L0;
  const c1 = D0;
  const c2 = DD0 / 2;
  const c3 = -(10 * L0 - 10 * Lf + 6 * D0 * s + 4 * Df * s + 1.5 * DD0 * S2 - 0.5 * DDf * S2) / S3;
  const c4 =  (15 * L0 - 15 * Lf + 8 * D0 * s + 7 * Df * s + 1.5 * DD0 * S2 -       DDf * S2) / S4;
  const c5 = -( 6 * L0 -  6 * Lf + 3 * D0 * s + 3 * Df * s + 0.5 * DD0 * S2 - 0.5 * DDf * S2) / S5;

  // 用 Horner 方法求值（避免 pow 调用，O(5) 乘法）
  return {
    coefs: [c0, c1, c2, c3, c4, c5],
    /** l(s)：返回横向偏移 (Px) */
    eval(sv: number): Px { return px(c0 + sv*(c1 + sv*(c2 + sv*(c3 + sv*(c4 + sv*c5))))); },
    /** l'(s)：返回 dl/ds (DimLess) */
    deriv1(sv: number): DimLess { return dimless(c1 + sv*(2*c2 + sv*(3*c3 + sv*(4*c4 + sv*5*c5)))); },
    /** l''(s)：返回 d²l/ds² (PerPx) */
    deriv2(sv: number): PerPx { return perpx(2*c2 + sv*(6*c3 + sv*(12*c4 + sv*20*c5))); },
  };
}

/**
 * 五次多项式速度规划 s(t) — 纵向轨迹
 *
 * 【使用场景】
 * 规划车辆在时间 Tf 内从当前速度 v0 到目标速度 targetV 的纵向运动。
 *
 * 【边界条件】
 *   s(0)=0, s'(0)=v0, s''(0)=0
 *   s(Tf)=sf_guess, s'(Tf)=vf, s''(Tf)=0
 *
 * sf_guess = (v0+vf)/2 × Tf（平均速度估算总行程）。
 * 实际行程由五次多项式自动调整，此估算只是给多项式一个合理的终点位置。
 *
 * 【调用者】candidate.ts 的 generateCandidate()
 */
export function solveSpeedProfile(v0: PxPerSec, a0: PxPerSec2, vf: PxPerSec, Tf: Seconds) {
  const sf = ((v0 as number) + (vf as number)) / 2 * (Tf as number);
  const poly = solveQuintic(
    px(0), dimless(v0 as number), perpx(a0 as number),
    px(sf), dimless(vf as number), perpx(0),
    Tf as number,
  );
  return {
    coefs: poly.coefs,
    eval(t: number): number { return (poly.eval(t) as number); },
    deriv1(t: number): number { return (poly.deriv1(t) as number); },
  };
}
