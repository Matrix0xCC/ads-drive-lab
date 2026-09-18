// ==================== 流水线接口：env → algo 数据契约 ====================
//
// 定义感知-规划-控制各模块间的消息格式。

import { Radians, Px, DimLess } from '../../../shared/dimensions.js';
import { CarState, TrackGeometry, Obstacle } from './geometry.js';

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

export function createWorldSnapshot(car: CarState, track: TrackGeometry, obstacles: Obstacle[]): WorldSnapshot {
  return { car, track, obstacles, timestamp: performance.now() };
}

export function createLidarScan(angles: Radians[], distances: Px[], rawDistances: Px[], confidences: number[]): LidarScan {
  return { angles: [...angles], distances: [...distances], rawDistances: [...rawDistances], confidences: [...confidences], timestamp: performance.now() };
}

export function createControlCommand(steerAngle: Radians, throttle: DimLess): ControlCommand {
  return { steerAngle, throttle, timestamp: performance.now() };
}
