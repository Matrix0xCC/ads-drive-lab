// ==================== 调试状态（跨模块共享） ====================
//
// 各模块在运行时将内部状态写入此对象。
// inspect 面板和 HUD 读取这些数据展示。

import { LidarScan, Trajectory, FrenetReference } from './types.js';

/** 感知模块内部状态 */
export interface PerceptionDebug {
  rayCount: number;
  frontDist: number;           // 正前方距离
  minDist: number;             // 全程最近命中
  minDistAngle: number;        // 最近命中方向（相对车头）
  rawHits: number;             // 未过滤命中数
  filteredHits: number;        // 背景减除后保留数
}

/** 规划模块内部状态（解耦架构：路径层候选 + 速度层模式） */
export interface PlanningDebug {
  state: string;               // 当前状态机状态
  frenetS: number;             // 车辆 Frenet s
  frenetL: number;             // 车辆 Frenet l
  dl_ds: number;               // dl/ds 斜率
  headingDiff: number;         // 车头与切线夹角
  paths: {
    targetL: number;
    blocked: boolean;
    cost: number;
  }[];
  bestCost: number;
  bestTargetL: number;
  /** 巡航速度目标（仿真单位） */
  speedTarget: number;
  /** 速度层模式：cruise | stop | emergency */
  speedMode: string;
  remainingDist: number;
}

/** 控制模块内部状态 */
export interface ControlDebug {
  lookaheadDist: number;       // 预瞄距离
  lookaheadX: number;          // 预瞄点坐标
  lookaheadY: number;
  alpha: number;               // 预瞄角度差
  steerAngle: number;          // 输出转向角
  throttle: number;            // 输出油门
}

/** 全局调试状态 */
export interface DebugState {
  paused: boolean;
  perception: PerceptionDebug | null;
  planning: PlanningDebug | null;
  control: ControlDebug | null;
}

export const debug: DebugState = {
  paused: false,
  perception: null,
  planning: null,
  control: null,
};
