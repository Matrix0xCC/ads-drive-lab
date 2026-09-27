// ==================== 流水线接口：env → algo 数据契约 ====================
//
// 定义感知-规划-控制各模块间的消息格式。

import { Radians, Px, DimLess } from '../../../shared/dimensions.js';
import { CarState, TrackGeometry, Obstacle } from './geometry.js';
import type { Route } from './planning.js';

/**
 * WorldSnapshot — 环境在每个感知周期打包给算法的"世界快照"
 *
 * 包含感知模块需要的所有原始数据。算法模块不直接访问 env 内部状态，
 * 只通过 WorldSnapshot 获取信息。
 */
export interface WorldSnapshot {
  car: CarState;
  track: TrackGeometry;
  obstacles: Obstacle[];
  timestamp: number;
  /** 当前路线（可选）：**路网地图的"道路"是路线**——感知要按它取路缘边界（见 perception.ts） */
  route?: Route | null;
  /** 交通流当帧的车辆矩形（**只给渲染/记录分流用**）。
   *  算法侧一律只看 `obstacles`（那里已经**含**交通流）——保持"障碍物只有一个概念"。 */
  traffic?: Obstacle[];
}

/**
 * LidarScan — 感知模块的输出、规划模块的输入
 *
 * angles[i] / distances[i] / rawDistances[i] / confidences[i] 一一对应。
 *
 * distances:   过滤后距离（规划层使用）。已知边界 → MAX_SENSOR_DIST
 * rawDistances: 过滤前距离（渲染层使用）。保留真实命中位置
 * confidences:  0=边界(已过滤) 1=高置信障碍物 0.5=低置信(边缘模糊)（规划层用更低安全余量）
 */
export interface LidarScan {
  angles: Radians[];
  distances: Px[];
  rawDistances: Px[];
  confidences: number[];
  /** 每条射线的**真障碍**命中点 Frenet 位置（px；无命中/命中路缘时 = NaN）。
   *  规划层用它判断"这个命中在不在我这条车道、在不在我前方"——旧的 `min(distances)` 口径
   *  会把相邻车道/侧后方的命中当限速源（实测：旁边超车就把巡航目标钉到 17.5m/s）。 */
  hitS?: number[];
  hitL?: number[];
  /** 每条射线来自哪个传感器（下标；多向套件的融合要用）。**不进归档**（可由 rig 重建） */
  sensorOf?: number[];
  /** 每条射线的**传感器量程**（px）：超过它就是该传感器的**盲区**（未知），不是"空"。
   *  与 `distances`（只回答"有没有障碍"）分开，供 S4 的"已知自由/未知"记账用。 */
  sensorMax?: number[];
  /** 本帧用了几个传感器 */
  sensorCount?: number;
  timestamp: number;
}

/** 规划模块的输出（兼容旧接口，暂未使用） */
export interface PlanningOutput {
  steerInput: DimLess;
  throttle: DimLess;
  state: 'NORMAL' | 'AVOID';
  timestamp: number;
}

/**
 * ControlCommand — 控制模块的输出、车辆物理模型的输入
 *
 * steerAngle 是物理前轮转角（弧度），直接作用于 stepPhysics()。
 */
export interface ControlCommand {
  steerAngle: Radians;
  throttle: DimLess;
  timestamp: number;
}

// ═══════════════════════════════════════════════════
// 工厂函数
// ═══════════════════════════════════════════════════

export function createWorldSnapshot(
  car: CarState, track: TrackGeometry, obstacles: Obstacle[], route?: Route | null,
  traffic?: Obstacle[],
): WorldSnapshot {
  return { car, track, obstacles, route: route ?? null, traffic, timestamp: performance.now() };
}

export function createLidarScan(
  angles: Radians[], distances: Px[], rawDistances: Px[], confidences: number[],
  hitS?: number[], hitL?: number[], sensorOf?: number[], sensorMax?: number[], sensorCount?: number,
): LidarScan {
  return {
    angles: [...angles], distances: [...distances], rawDistances: [...rawDistances], confidences: [...confidences],
    hitS: hitS ? [...hitS] : undefined, hitL: hitL ? [...hitL] : undefined,
    sensorOf: sensorOf ? [...sensorOf] : undefined,
    sensorMax: sensorMax ? [...sensorMax] : undefined, sensorCount,
    timestamp: performance.now(),
  };
}

export function createControlCommand(steerAngle: Radians, throttle: DimLess): ControlCommand {
  return { steerAngle, throttle, timestamp: performance.now() };
}
