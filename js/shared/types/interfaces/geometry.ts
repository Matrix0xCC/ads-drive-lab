// ==================== 几何接口：点、赛道、障碍物、车辆 ====================
//
// env 模块产出，algo 模块只读消费。所有字段带量纲约束。

import { Radians, Px, PxPerSec, PxPerSec2, DimLess, rad } from '../../../shared/dimensions.js';

/** 2D 点 — 赛道、车辆、障碍物的基本坐标单元 */
export interface Point2D {
  x: Px;
  y: Px;
}

/** 赛道几何 — 中心线 + 内外边界 */
export interface TrackGeometry {
  centerPoints: Point2D[];
  innerBoundary: Point2D[];
  outerBoundary: Point2D[];
}

/** 障碍物 — 矩形，带位置、尺寸和朝向角 */
export interface Obstacle {
  x: Px;
  y: Px;
  width: Px;
  height: Px;
  angle: Radians;
  zHeight?: number;  // 仅 3D 渲染使用
}

/**
 * 车辆状态 — "被控对象"的完整运动学状态
 *
 * angle (Radians): 车头朝向，0=朝右，正=逆时针
 * steerAngle (Radians): 前轮转角，正=右转
 * friction (DimLess): 线性空气阻力系数
 */
export interface CarState {
  x: Px;
  y: Px;
  angle: Radians;
  speed: PxPerSec;
  steerAngle: Radians;
  maxSpeed: PxPerSec;
  acceleration: PxPerSec2;
  friction: DimLess;
  width: Px;
  length: Px;
  /** 碰撞半径（圆形近似，对角线一半） */
  collisionRadius: Px;
}

/** 最大前轮转角（弧度） */
export const VEHICLE_MAX_STEER: Radians = rad(2.8);

// ═══════════════════════════════════════════════════
// 障碍物 → 线段转换
// ═══════════════════════════════════════════════════

interface Segment {
  x1: number; y1: number;
  x2: number; y2: number;
}

/**
 * 将矩形障碍物转换为 4 条线段（用于 LiDAR 射线求交检测）
 *
 * 1. 在本地坐标系定义 4 个角点
 * 2. 用旋转矩阵变换到世界坐标系
 * 3. 按顺序连接角点 → 闭合四边形 4 条边
 */
export function obstacleToSegments(obs: Obstacle): Segment[] {
  const hw = (obs.width as number) / 2;
  const hh = (obs.height as number) / 2;
  const corners = [
    { x: -hw, y: -hh }, { x: hw, y: -hh },
    { x:  hw, y:  hh }, { x: -hw, y:  hh },
  ];
  const c = Math.cos(obs.angle as number);
  const s = Math.sin(obs.angle as number);
  const ox = obs.x as number;
  const oy = obs.y as number;
  const world = corners.map(pt => ({
    x: ox + pt.x * c - pt.y * s,
    y: oy + pt.x * s + pt.y * c,
  }));
  return [
    { x1: world[0].x, y1: world[0].y, x2: world[1].x, y2: world[1].y },
    { x1: world[1].x, y1: world[1].y, x2: world[2].x, y2: world[2].y },
    { x1: world[2].x, y1: world[2].y, x2: world[3].x, y2: world[3].y },
    { x1: world[3].x, y1: world[3].y, x2: world[0].x, y2: world[0].y },
  ];
}
