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

/** 走廊可视化载荷（渲染层读取；直接引用路径层已有对象，零拷贝。
 *  shared 不依赖 algo，故此处结构化声明（字段与 corridor.ts 的 Corridor/PathCandidate 对齐）。 */
export interface PlanningViz {
  /** 自由带四边形（笛卡尔；每对相邻网格步的自由段交集） */
  bands: { x: number; y: number }[][];
  /** 瓶颈横断线（笛卡尔；全封死时一条 ±lMax 满宽标记） */
  pinchSegments: { x: number; y: number }[][];
  /** 瓶颈摘要（sealed = 无任何自由位置） */
  pinch: { s: number; width: number; sealed: boolean } | null;
  /** 每条自由带是否被本帧雷达真正扫到（与 bands 一一对应；false = 遮挡阴影/视锥外，属"未知"） */
  bandObserved: boolean[];
  /** 观测极限横断线（笛卡尔；空数组 = 全观测，不画） */
  obsLimitSegments: { x: number; y: number }[][];
  /** 未观测区外边界线段（笛卡尔；共享边已去重，供渲染层描边） */
  unobsBoundary: { x: number; y: number }[][];
  /** 可视化网格步长（仿真单位；= 规划网格的一半，带子边界更贴曲率） */
  stepU: number;
  /** 候选路径（targetL/develop/阻塞标记/首次越界弧长/路径点——路径点带 s 供画发展段与截断） */
  candidates: {
    targetL: number; develop: number; blocked: boolean; blockedHard: boolean;
    sBlock: number; sBlockHard: number; cost: number;
    points: { s: number; x: number; y: number }[];
  }[];
  /** 最优候选目标横向偏移（渲染层高亮用） */
  bestTargetL: number;
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
  /** 巡航速度目标（仿真单位；**已被路径几何压低**过） */
  speedTarget: number;
  /** **路径层之前**的巡航目标（地图限速 + 本车道前方最近命中，见 plan/inlane.ts） */
  cruiseV: number;
  /** 速度层模式：cruise | stop | emergency */
  speedMode: string;
  /** 地图限速（仿真单位/s；null = 该地图未声明）——让"地图语义在起作用"可见 */
  mapSpeedLimitMps?: number | null;
  /** 名义车道中心（仿真单位，Frenet l）——来自地图 */
  laneOffsetU?: number;
  /** 本图全部车道中心（仿真单位，升序） */
  laneCentersU?: number[];
  /** 紧间隙迟滞状态（tight.ts）：NONE | CREEPING | SEALED（回放无此数据） */
  tight?: string;
  /** 数据来源：'replay' = 回放（用归档扫描重算走廊，迭代/耗时等实时字段缺省） */
  source?: 'live' | 'replay';
  /** 路径-速度收敛迭代的路径层求值次数（2..3；spec 2026-09-25 §5；回放无此数据） */
  rounds?: number;
  /** 种子假设不自洽（速度上限 ≠ 假设）→ 本帧确实需要迭代修正（回放无此数据） */
  revised?: boolean;
  /** 本帧 plan() 总耗时（ms；只记录，不参与决策；回放无此数据） */
  planMs?: number;
  /** 迭代末轮的速度假设（仿真单位/s；几何评估速度；回放无此数据） */
  vAssume?: number;
  /** 走廊可视化载荷（渲染层用；未接线时为 null） */
  viz: PlanningViz | null;
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
