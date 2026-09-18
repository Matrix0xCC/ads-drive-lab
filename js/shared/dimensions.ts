// ==================== 量纲类型系统 ====================
//
// 【为什么需要量纲类型？】
// 在这个项目中，我们之前遇到了两次量纲错误：
//   1. car.angle += steerAngle * dt
//      Radians += Radians * Seconds → 实际应该是 RadPerSec * Seconds
//      steerAngle 是角度（rad），被错误地当成了角速度（rad/s）
//   2. fs.dl = speed * sin(headingDiff)
//      DimLess = PxPerSec * DimLess → 实际应该是 tan(headingDiff)
//      dl/ds（无量纲斜率）被错误地赋值为速度（px/s）
//
// 【设计方案：Branded Types（品牌类型）】
// 使用 TypeScript 的 intersection type 给 number 附加编译时的"量纲标签"。
// 这些标签在编译后完全擦除，运行时零开销。
//
//   角度：  type Radians   = number & { brand: 'radians' }
//   角速度：type RadPerSec = number & { brand: 'rad_per_sec' }
//   距离：  type Px        = number & { brand: 'px' }
//   速度：  type PxPerSec  = number & { brand: 'px_per_sec' }
//   无量纲：type DimLess   = number & { brand: 'dimensionless' }
//
// 【使用原则】
//   - 模块边界（函数签名、接口字段）使用品牌类型 — 编译时检查
//   - 函数内部中间计算可用普通 number — 灵活高效
//   - 工厂函数是 unsafe cast 的唯一位置 — 集中审计点

declare const _brand: unique symbol;
type Branded<T, B> = T & { readonly [_brand]: B };

// ═══════════════════════════════════════════════════
// 量纲定义
// ═══════════════════════════════════════════════════

/** 角度（弧度）— 如车头朝向、转向角、LiDAR 射线角度 */
export type Radians = Branded<number, 'radians'>;

/** 角速度（弧度/秒）— 横摆角速度 ω = v/L × tan(δ) */
export type RadPerSec = Branded<number, 'rad_per_sec'>;

/** 像素（位置、距离）— 如坐标、预瞄距离、轴距 */
export type Px = Branded<number, 'px'>;

/** 像素/秒（速度）— 如车速、纵向速度 ds/dt */
export type PxPerSec = Branded<number, 'px_per_sec'>;

/** 像素/秒²（加速度）— 如车辆加速度 */
export type PxPerSec2 = Branded<number, 'px_per_sec2'>;

/** 无量纲（比例、三角函数结果、dl/ds 斜率） */
export type DimLess = Branded<number, 'dimensionless'>;

/** 1/像素（曲率 κ）— κ = 1/R = 2×sin(α)/ld */
export type PerPx = Branded<number, 'per_px'>;

/** 秒（时间）— 时间步长 dt、轨迹时长 */
export type Seconds = Branded<number, 'seconds'>;

// ═══════════════════════════════════════════════════
// 单位系统：物理量 → 仿真单位
// ═══════════════════════════════════════════════════
//
// 仿真内部统一使用 Px/PxPerSec 等无量纲"仿真单位"。
// 物理量通过单一比例尺转换为仿真单位：
//
//   SIM_UNITS_PER_METER = 4  →  1m = 4 仿真单位
//
// 示例：车宽 2m → m(2) → px(8)
//       速度 25m/s → mps(25) → pxps(100)
//
// 修改比例尺只需改这一个常量。渲染层不需要额外转换
// （Three.js 使用任意单位，仿真单位直接映射到 3D 场景）。

/** 1 米 = N 仿真单位（也是渲染时的 px 映射） */
export const SIM_UNITS_PER_METER = 4;

/** 物理长度 → 仿真单位：m(2.0) = 2 米 */
export const m = (v: number): Px => px(v * SIM_UNITS_PER_METER);

/** 物理速度 → 仿真单位：mps(25) = 25 m/s = 90 km/h */
export const mps = (v: number): PxPerSec => pxps(v * SIM_UNITS_PER_METER);

/** 物理加速度 → 仿真单位：mps2(5) = 5 m/s² */
export const mps2 = (v: number): PxPerSec2 => pxps2(v * SIM_UNITS_PER_METER);

// ═══════════════════════════════════════════════════
// 仿真单位工厂函数 — 唯一的 unsafe cast 点
// ═══════════════════════════════════════════════════

export const rad     = (v: number): Radians   => v as Radians;
export const radps   = (v: number): RadPerSec => v as RadPerSec;
export const px      = (v: number): Px        => v as Px;
export const pxps    = (v: number): PxPerSec  => v as PxPerSec;
export const pxps2   = (v: number): PxPerSec2 => v as PxPerSec2;
export const dimless = (v: number): DimLess   => v as DimLess;
export const perpx   = (v: number): PerPx     => v as PerPx;
export const secs    = (v: number): Seconds   => v as Seconds;

// ═══════════════════════════════════════════════════
// 量纲感知的数学运算
// ═══════════════════════════════════════════════════
// 这些辅助函数封装了常见的跨量纲运算，确保输入输出类型正确。

/**
 * 横摆角速度：ω = v / L × tan(δ)
 *
 * 【自行车模型核心公式】
 * 前轮转角 δ（Radians）通过轴距 L（Px）和速度 v（PxPerSec）产生横摆角速度。
 * 量纲推导：PxPerSec / Px × DimLess = 1/s × 1 = RadPerSec ✓
 *
 * @param speed     车速 (PxPerSec)
 * @param wheelbase 轴距 (Px)
 * @param steerAngle 前轮转角 (Radians)
 * @returns 横摆角速度 (RadPerSec)
 */
export function yawRate(speed: PxPerSec, wheelbase: Px, steerAngle: Radians): RadPerSec {
  return radps((speed as number) / (wheelbase as number) * Math.tan(steerAngle as number));
}

/**
 * 三角函数封装 — 输入 Radians，输出 DimLess
 *
 * 普通 Math.sin/cos/tan 接受 number 返回 number，没有量纲信息。
 * 这些封装确保输入必须是角度类型，输出标记为无量纲。
 */
export function sin(a: Radians): DimLess { return dimless(Math.sin(a as number)); }
export function cos(a: Radians): DimLess { return dimless(Math.cos(a as number)); }
export function tan(a: Radians): DimLess { return dimless(Math.tan(a as number)); }

/** atan2 — 输入 number（坐标差），输出 Radians */
export function atan2(y: number, x: number): Radians { return rad(Math.atan2(y, x)); }

/**
 * 角度归一化到 [-π, π]
 *
 * 【为什么需要归一化？】
 * 角度相减后可能超出 [-π, π] 范围（如 3.1 - (-3.1) = 6.2），
 * 但物理上这等价于 -0.08。不归一化会导致转向指令错误。
 * 用 atan2(sin, cos) 技巧实现，比用 while 循环更高效。
 */
export function normalizeAngle(a: Radians): Radians {
  return rad(Math.atan2(Math.sin(a as number), Math.cos(a as number)));
}

/**
 * 计算 Frenet 横向导数 dl/ds
 *
 * 【关键公式】
 * dl/dt = speed × sin(headingDiff)        — 横向速度（px/s）
 * ds/dt = speed × cos(headingDiff)        — 纵向速度（px/s）
 * dl/ds = (dl/dt) / (ds/dt) = tan(headingDiff) — 无量纲斜率
 *
 * 这就是之前 bug 的修复点：dl/ds 是 tan(Δ)，不是 speed × sin(Δ)！
 * TypeScript 的类型系统保证了这里返回值是 DimLess，防止量纲错误。
 */
export function lateralSlope(speed: PxPerSec, headingDiff: Radians): { ds: PxPerSec; dl_ds: DimLess } {
  const hd = headingDiff as number;
  return {
    ds: pxps((speed as number) * Math.cos(hd)),
    dl_ds: dimless(Math.tan(hd)),
  };
}

/** 两点距离 hypot(dx, dy) — 输入输出都是 Px */
export function hypot(dx: Px, dy: Px): Px { return px(Math.hypot(dx as number, dy as number)); }
