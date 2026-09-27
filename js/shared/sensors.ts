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

/**
 * 默认传感器套件。**目前只有前向一个**——扩成多向（S2）时在这里加：
 *   · 后向中距（±60°、50m）：看见后车，配合"责任划分"（超车不理会、切入才让）
 *   · 左右角雷达（各 ±75°、30m，位姿在车角）：补侧后方盲区，消除变道/并线的"突然出现"
 * 加之前请确认 S1（规划吃物体）已完成，否则后方命中会限速。
 */
export const DEFAULT_SENSOR_RIG: SensorSpec[] = [FRONT_SENSOR];

/** 单个传感器的射线数（含两端：与历史 `SCAN_RAY_COUNT` 同一算法，保证默认 rig 逐位一致） */
export function sensorRayCount(s: SensorSpec): number {
  return Math.floor(s.fov / s.resolution) + 1;
}

/** 整个 rig 的射线总数（渲染层射线池按它扩容） */
export function rigRayCount(rig: SensorSpec[]): number {
  return rig.reduce((n, s) => n + sensorRayCount(s), 0);
}

/** rig 里的最大量程（背景减除/未命中的哨兵值用） */
export function rigMaxRange(rig: SensorSpec[]): Px {
  return rig.reduce((mx, s) => (s.maxRange > mx ? s.maxRange : mx), px(0));
}
