// ==================== 几何接口：点、赛道、障碍物、车辆 ====================
//
// env 模块产出，algo 模块只读消费。所有字段带量纲约束。

import { Radians, Px, PxPerSec, PxPerSec2, DimLess, rad } from '../../../shared/dimensions.js';

/** 2D 点 — 赛道、车辆、障碍物的基本坐标单元 */
export interface Point2D {
  x: Px;
  y: Px;
}

/** 赛道几何 — 中心线 + 内外边界
 *
 * 【开环（OSM 高速）语义】`closed === false` 时：
 *   - `centerPoints` 是**开环点列**（世界坐标，已含边距，见 mapgen）；
 *   - `innerBoundary` 为**空**（开环道路没有"内场"）；
 *   - `outerBoundary` 是**闭合的路面带多边形**（左缘 → 末端帽 → 右缘 → 起点帽），
 *     因此 `car.ts` 的 `pointInPolygon(outer) && !pointInPolygon(inner)` 判定继续成立，
 *     且两端帽自然成为"不能冲出地图"的端墙。
 * 【缺省】`closed` 缺省视为 `true`、`halfWidth` 缺省 `m(7.5)`——保持历史径向地图不变。 */
export interface TrackGeometry {
  centerPoints: Point2D[];
  innerBoundary: Point2D[];
  outerBoundary: Point2D[];
  /** 是否闭环；缺省 true */
  closed?: boolean;
  /** 道路半宽（px）；缺省 m(7.5) */
  halfWidth?: Px;
  /** 车道宽（px）；缺省 m(3.75)。渲染车道分隔线与车道中心推导共用 */
  laneWidth?: Px;
  /** 车道中心横向偏移（px，Frenet l，升序）——**车道语义来自地图** */
  laneCenters?: Px[];
  /** 名义行驶车道中心（px）：路径代价的中心偏好与 AVOID 判据都用它 */
  laneOffset?: Px;
  /** 该地图的限速（px/s，来自 OSM maxspeed）：规划器的巡航速度硬上限。
   *  缺省 undefined = 该地图不限速（用规划器默认上限） */
  speedLimitMps?: PxPerSec;
  /** 路网地图（`chains` 存在即视为路网）：全部可行驶链（世界坐标）。
   *  与"单中心线"地图的区别：可行驶域 = 到**最近链**的距离 ≤ 半宽（路口天然由并集覆盖），
   *  而不是"在某条闭合多边形内"。`centerPoints` 此时取最长链，作为无路线时的回退参考线。 */
  chains?: Point2D[][];
  /** 路网节点（世界坐标）；路由图的端点 */
  netNodes?: Point2D[];
  /** 路网边（节点对 → 链序号） */
  netEdges?: { from: number; to: number; chain: number; lengthM?: number; oneway?: boolean }[];
  /** 逐链道路属性（与 chains 下标对齐）；缺省 = 全图用地图层标量 */
  chainAttrs?: RoadAttrs[];
  /** 参考线上的逐段属性（弧长 px，沿 `centerPoints`）：无路线时规划域/限速就查它 */
  attrSpans?: AttrSpan[];
}

/** 逐段道路属性（"这段路几车道、限速多少、多宽"）——**地图起作用**的第二层：
 *  地图级标量（TrackGeometry.halfWidth/speedLimitMps）是全场默认，本类型是逐段真值。
 *  米制字段保持米（渲染宽度、限速用）；弧长另用 px（与运行时的 s 一致）。 */
export interface RoadAttrs {
  /** OSM highway 值（网络地图逐链有；链式地图的区间也有） */
  highway: string;
  laneCount: number;
  /** 单车道宽（米） */
  laneWidthM: number;
  /** 半宽（米）= laneCount × laneWidthM / 2 */
  halfWidthM: number;
  maxspeedKph: number;
}

/** 沿参考线的一段属性 */
export interface AttrSpan extends RoadAttrs {
  /** 区间起点弧长（px） */
  s0: Px;
  /** 区间终点弧长（px） */
  s1: Px;
}

/** 取参考线弧长 s 处的属性（线性查表；超界取首/末段；无表返回 null） */
export function attrsAtS(spans: AttrSpan[] | undefined, s: number): AttrSpan | null {
  if (!spans || spans.length === 0) return null;
  if (s <= (spans[0].s0 as number)) return spans[0];
  for (const sp of spans) if (s < (sp.s1 as number)) return sp;
  return spans[spans.length - 1];
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
