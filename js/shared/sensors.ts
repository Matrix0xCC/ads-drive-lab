// ==================== 传感器配置（多向雷达的骨架） ====================
//
// 【为什么要有这一层】现在的感知把"视场/分辨率/量程"写死成**单个前向扇面**（±60°、2°、125m），
// 规划器又直接对**射线距离**做决策（`vCap = 最近射线/2`）。业界的长期形态是**多向传感器套件**：
// 每个传感器有自己的**安装位姿**（位置 + 朝向）、视场、量程、测量频率，各自出测量帧；
// 融合/跟踪层把它们变成"以自车为中心的 360° 物体列表"，规划器只吃物体列表（传感器无关）。
//
// 【本步（S0）只做抽象，不改行为】把参数抽成数据 + 支持安装位姿；**默认 rig 与今天逐位一致**
// （单个前向传感器、位姿为零）。这样后续加后向/侧向传感器只是往 `SENSOR_RIG` 里加一项，
// 而"规划器改吃物体"（S1）必须在加传感器**之前**完成——否则后方目标会被当成前方障碍限速。
//
// 【量纲】位姿用 px / rad（品牌类型在模块边界转换），与仿真一致。

import { px, rad, m, Px, Radians } from './dimensions.js';

export interface SensorSpec {
  /** 稳定标识：归档/调试/融合按它区分（换 rig 会改变归档，别随意改） */
  id: string;
  /** 安装位姿（**车辆坐标系**：x 前、y 左；yaw 相对车头） */
  pose: { x: Px; y: Px; yaw: Radians };
  /** 总视场（rad） */
  fov: number;
  /** 角分辨率（rad） */
  resolution: number;
  /** 量程（px） */
  maxRange: Px;
  /** 测量频率（Hz） */
  hz: number;
}

/** 今天用的前向单传感器（±60°、2°、125m、20Hz）——默认 rig 就是它，行为与历史逐位一致 */
export const FRONT_SENSOR: SensorSpec = {
  id: 'front',
  pose: { x: px(0), y: px(0), yaw: rad(0) },
  fov: (Math.PI * 2) / 3,
  resolution: (Math.PI * 2) / 180,
  maxRange: m(125),
  hz: 20,
};

/** 只看前向的套件（A/B 对照用：S2 之前的行为） */
export const FRONT_ONLY_RIG: SensorSpec[] = [FRONT_SENSOR];

/** 后向中距：看见后车（配合责任划分：超车不理会、切入才让）。装在车尾、朝后 */
export const REAR_SENSOR: SensorSpec = {
  id: 'rear',
  pose: { x: m(-2.2), y: px(0), yaw: rad(Math.PI) },
  fov: (Math.PI * 2) / 3,                 // ±60°
  resolution: (Math.PI * 4) / 180,        // 4°（中距，够用）
  maxRange: m(50),
  hz: 20,
};

/** 左右角雷达：补侧后方盲区（变道/并线时"突然出现"的车）。装在车头两角、朝侧向 */
export const CORNER_SENSORS: SensorSpec[] = [
  { id: 'corner-l', pose: { x: m(2.0), y: m(0.9), yaw: rad(Math.PI / 2) },
    fov: (Math.PI * 5) / 6, resolution: (Math.PI * 6) / 180, maxRange: m(30), hz: 20 },   // ±75°、6°、30m
  { id: 'corner-r', pose: { x: m(2.0), y: m(-0.9), yaw: rad(-Math.PI / 2) },
    fov: (Math.PI * 5) / 6, resolution: (Math.PI * 6) / 180, maxRange: m(30), hz: 20 },
];

/**
 * 默认传感器套件 = **多向**（S2）：前 ±60°/125m + 后 ±60°/50m + 左右角各 ±75°/30m。
 * 覆盖 360° 且有重叠区（重叠区是 S3 融合的难点：同一物体被两个传感器看到 → 必须先去重再跟踪）。
 *
 * 【已知约束，别忽略】
 * · 每根射线的"看不见"哨兵 = **它自己传感器的量程**（角雷达 30m 外是**盲区**，不是"空"）——
 *   真正的"已知自由 / 未知"记账是 S4；在 S4 之前，未知区一律按保守处理。
 * · 各传感器目前都用感知频率（20Hz）；**每传感器独立频率**（固定步长累加器）留到 S5。
 * · 规划侧只吃"本车道 + 前方"的命中（S1①），所以后向/侧向命中**不会**限速；
 *   后向传感器接入的前提是 S1①（已完成 ✓）。
 * · 射线数 61+31+26+26 = **144**：渲染层射线池按 `rigRayCount()` 扩容，感知耗时约 2.4×。
 */
export const SURROUND_SENSOR_RIG: SensorSpec[] = [FRONT_SENSOR, REAR_SENSOR, ...CORNER_SENSORS];

/**
 * **默认套件 = 前向单个**（与历史行为一致）。
 *
 * 【为什么多向 rig 还不设默认】S2 只把**基础设施**做好（位姿、每射线传感器归属、每射线量程）。
 * 实测：直接开多向会改变既有场景的行为——角雷达看到的路缘/旁边车进入规划走廊，放大了 S1② 的
 * "无辜横移"问题（三条路缘测试 + 径向黄金基线同时变化）。按计划，多向套件要在 **S3（融合/跟踪
 * 出物体列表 → 物体归属 + 责任划分）** 之后再设为默认；届时用"只看得到后方的物体不得改变轨迹"
 * 这条回归断言守住安全（见 js/test/perception/rig.test.ts）。
 */
export const DEFAULT_SENSOR_RIG: SensorSpec[] = FRONT_ONLY_RIG;

/** 单个传感器的射线数（含两端：与历史 `SCAN_RAY_COUNT` 同一算法，保证默认 rig 逐位一致） */
export function sensorRayCount(s: SensorSpec): number {
  // 用 round 而不是 floor：120°/4° 在弧度里会算成 29.999…，floor 会少一条射线
  // （前向 ±60°/2° 恰好整除，两者都是 61，所以历史行为不变）。
  return Math.round(s.fov / s.resolution) + 1;
}

/** 整个 rig 的射线总数（渲染层射线池按它扩容） */
export function rigRayCount(rig: SensorSpec[]): number {
  return rig.reduce((n, s) => n + sensorRayCount(s), 0);
}

/** rig 里的最大量程（背景减除/未命中的哨兵值用） */
export function rigMaxRange(rig: SensorSpec[]): Px {
  return rig.reduce((mx, s) => (s.maxRange > mx ? s.maxRange : mx), px(0));
}
