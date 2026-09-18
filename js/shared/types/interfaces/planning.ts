// ==================== 规划接口：Frenet、轨迹、路径 ====================
//
// 规划模块使用的数据类型。

import { Radians, Px, PxPerSec, PxPerSec2, DimLess, PerPx, Seconds } from '../../../shared/dimensions.js';
import { Point2D } from './geometry.js';

// ═══════════════════════════════════════════════════
// Frenet 坐标系
// ═══════════════════════════════════════════════════

/** Frenet 状态 — 车辆在 S-L 框架中的瞬时状态 */
export interface FrenetState {
  s: Px;            // 沿参考线的弧长（纵向位置）
  ds: PxPerSec;     // ds/dt（纵向速度）
  dds: PxPerSec2;   // d²s/dt²（纵向加速度）
  l: Px;            // 横向偏移（正=内侧，负=外侧）
  dl: DimLess;      // dl/ds — 无量纲斜率（关键量纲约束）
  ddl: PerPx;       // d²l/ds²（横向曲率）
}

/** Frenet 参考线 — 从赛道中心线构建的 S-L 坐标系 */
export interface FrenetReference {
  points: Point2D[];                       // 参考线采样点
  s: number[];                             // 累计弧长
  tangents: { x: number; y: number }[];    // 单位切向量
  normals: { x: number; y: number }[];     // 单位法向量（指向赛道内侧）
  totalS: number;                          // 参考线总弧长
  closedLoop: boolean;                     // 闭环取模 / 开环 clamp
}

// ═══════════════════════════════════════════════════
// 轨迹
// ═══════════════════════════════════════════════════

/** 轨迹点 — 时空中的一个采样点 */
export interface TrajectoryPoint {
  t: Seconds;       // 相对当前时刻的时间偏移
  x: Px;            // 世界坐标
  y: Px;
  yaw: Radians;     // 期望朝向
  v: PxPerSec;      // 目标速度
  kappa: PerPx;     // 路径曲率（1/半径）
}

/** 轨迹 — 规划模块产出的完整时空路径 */
export interface Trajectory {
  points: TrajectoryPoint[];
  cost: number;       // 越小越好
  type: string;       // lane_keep / avoid_left / avoid_right / emergency / arrived
  timestamp: number;
}

export function createTrajectory(points: TrajectoryPoint[], cost: number, type: string): Trajectory {
  return { points, cost, type, timestamp: performance.now() };
}

// ═══════════════════════════════════════════════════
// 全局路径规划
// ═══════════════════════════════════════════════════

/** 全局路径 — 从起点到终点的有序路网点序列 */
export interface Route {
  waypoints: Point2D[];
  waypointIndices: number[];
  startIndex: number;
  goalIndex: number;
  length: Px;
}

/** GPS 定位读数（真值 + 噪声） */
export interface GPSReading {
  x: Px;
  y: Px;
  uncertainty: Px;    // 2σ 误差半径
  timestamp: number;
}

export function createRoute(
  waypoints: Point2D[], indices: number[],
  startIdx: number, goalIdx: number, length: Px,
): Route {
  return { waypoints, waypointIndices: indices, startIndex: startIdx, goalIndex: goalIdx, length };
}

export function createGPSReading(x: Px, y: Px, uncertainty: Px): GPSReading {
  return { x, y, uncertainty, timestamp: performance.now() };
}
