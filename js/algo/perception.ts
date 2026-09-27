// ==================== 算法：感知模块（Perception） ====================
//
// 【职责】接收 WorldSnapshot，执行 LiDAR 扫描 + HD Map 背景减除，输出 LidarScan
//
// 【数据流】
//   WorldSnapshot → 收集线段（赛道边界 + 障碍物）→ 61 条射线求交 →
//   HD Map 背景减除（Frenet |l| 分类）→ LidarScan { distances, rawDistances, confidences }
//
// 【LiDAR 参数】
//   单线 2D 扫描仪（对标 SICK LMS 系列工业激光雷达）
//   - 水平 FOV：±60°（120° 总视场）
//   - 角分辨率：2° → 约 61 条射线
//   - 最大探测距离：125m（覆盖 140 m/s × 2s = 280m 轨迹 + 余量，比例 4px/m）
//   - 更新频率：20 Hz（外部定时器控制）
//
// 【背景减除（HD Map Background Subtraction）】
//   核心设计决策：只用传感器原始数据 + HD Map（赛道边界位置），不使用障碍物列表。
//   对标 Waymo/Apollo 的 map-based point cloud filtering。
//
//   每条 LiDAR 射线的处理管线：
//     1. 射线-线段求交 → 原始距离 rawDist
//     2. 命中点 → 世界坐标 → Frenet 投影 → |l|
//     3. |l| 三级分类：
//        |l| < 4.5m   → confidence=1（高置信障碍物：命中点在赛道中间 60% 区域）
//        4.5m ≤ |l| < 6.0m → confidence=0.5（边缘模糊：障碍物靠近路缘或 Frenet 误差）
//        |l| ≥ 6.0m   → confidence=0（边界：半宽的 80%，1.5m 缓冲吸收 Frenet 投影误差）
//     4. confidence=0 → filteredDist = MAX_SENSOR_DIST（规划层看不到）
//        confidence=0.5/1 → filteredDist = rawDist（规划层保留）
//
//   【为什么不查障碍物列表做交叉验证？】
//   那等价于上帝视角——规划层直接知道障碍物位置。真实系统中感知模块
//   只有传感器数据，无法区分"围墙边的障碍物"和"围墙本身"。
//   2D 单帧 LiDAR 在这个问题上存在物理极限——我们用置信度接受这个模糊性。
//
//   规划层 `collision.ts` 对低置信命中使用更大的安全余量（+10px）。
//
//   【为什么用闭环 Frenet 参考线？】
//   感知层的背景减除使用赛道闭环中心线（track.centerPoints）。
//   这与规划器在 route 模式下使用的开环参考线（route.waypoints）不同。
//   背景减除判断"这是围墙吗"用的是全局赛道几何——围墙在整个环道上存在。
//
// 【输出给规划层的数据】
//   LidarScan {
//     angles:      每条射线的世界绝对角度（弧度）
//     distances:   过滤后距离（规划层使用，边界命中已替换为 MAX）
//     rawDistances: 过滤前距离（渲染层使用，显示真实射线截止位置）
//     confidences:  置信度标记（0=边界 0.5=低置信 1=高置信）
//   }
//
// 【复杂度】
//   每次扫描：61 条射线 × (赛道边界约 400 段 + 障碍物约 20 段) ≈ 25,000 次求交
//   20 Hz 运行 = 每秒 50 万次求交。每次求交只有几十次浮点运算，
//   对现代浏览器 < 1ms。

import { WorldSnapshot, LidarScan, TrackGeometry, Point2D, AttrSpan, attrsAtS, createLidarScan, obstacleToSegments, FrenetReference, Route } from '../shared/types.js';
import { Radians, Px, SIM_UNITS_PER_METER, m, rad, px, normalizeAngle } from '../shared/dimensions.js';
import { buildFrenetReference, cartesianToFrenet } from './plan/frenet.js';
import { SensorSpec, DEFAULT_SENSOR_RIG, FRONT_SENSOR, rigMaxRange, sensorRayCount } from '../shared/sensors.js';
import { TRACK_HALF_WIDTH, ribbonPolygon, ribbonPolygonVar, halfWidthsAlongPoints, taperedHalfWidthAt } from '../env/track.js';
import { debug } from '../shared/debug.js';

// ═══════════════════════════════════════════════════
// LiDAR 硬件参数
// ═══════════════════════════════════════════════════

/** 水平视场角：±60°（共 120°），对标 SICK LMS 系列 */
// 【单一真值】视场/分辨率/量程都来自传感器套件（js/shared/sensors.ts）；这两个导出保留给既有调用方，
// 值由**默认 rig** 派生（与历史逐位一致，见 sensors.test.ts）
const SCAN_FOV = FRONT_SENSOR.fov;

/** 角分辨率：2°（≈ 0.0349 rad），每条射线间距约 12px @ 350px 距离 */
const SCAN_RESOLUTION = 2 * Math.PI / 180;

/** 最大探测距离 125m */
export const MAX_SENSOR_DIST: Px = FRONT_SENSOR.maxRange;

/** 射线总数 ≈ 61 */
export const SCAN_RAY_COUNT = sensorRayCount(FRONT_SENSOR);

// ═══════════════════════════════════════════════════
// 背景减除阈值
// ═══════════════════════════════════════════════════
//
// 边界（墙/护栏）在 |l| ≈ 道路半宽处。阈值按**半宽的比例**取，而不是写死的米数——
// OSM 高速图半宽 5.625m（3 车道）、径向图 7.5m，同一个绝对阈值在两者上的含义完全不同。
// 取 0.6 / 0.8：给 Frenet 投影误差（最近邻搜索 ±0.5m）与曲率影响留缓冲。
// 注：径向图 0.6×7.5=4.5m、0.8×7.5=6.0m —— 与历史写死值完全一致，行为零变化。

/** |l| < 0.6×半宽：明确在道路内部 → 高置信障碍物 */
const CONF_HIGH_RATIO = 0.6;

/** |l| > 0.8×半宽：明确在边界附近（缓冲吸收投影误差）→ 确认是边界 */
const CONF_LOW_RATIO = 0.8;

// ═══════════════════════════════════════════════════
// 线段类型
// ═══════════════════════════════════════════════════

interface Segment {
  x1: number; y1: number; x2: number; y2: number;
  /** 该线段是否来自 HD Map 的**道路边界**（路缘/端帽）。
   *  【为什么要这个标记】射线本来就是分别对"边界线段"与"障碍物矩形"求交的——**命中的是哪一类
   *  自己就知道**，比事后用 |l| 比值反推可靠得多。|l| 比值受最近邻投影误差影响：在宽度过渡段的
   *  斜边上，命中点会投影到几米外的 s，局部半宽对不上 → 真实路缘被判成 0.5 模糊带 → 规划器看成
   *  "横在路中间的一堵墙" → 停车（实测三环场景停在 s=2823m，那里正是匝道收窄的过渡段）。
   *  现在的规则：命中边界 → 一定是背景（conf 0）；命中障碍物 → 才用 |l| 比值做"贴路缘的模糊带"细化。 */
  boundary: boolean;
  /** 该线段来自**交通流车辆**（不是静态障碍/路缘）。车辆无论 |l| 多大都算高置信障碍——
   *  见下面命中分支的说明（|l| 降置信是为静态路缘设计的）。 */
  vehicle?: boolean;
}

// ═══════════════════════════════════════════════════
// 射线-线段求交（2D 参数方程法 / Cramer's Rule）
// ═══════════════════════════════════════════════════
//
// 【算法】
//   射线：P = O + t·D  (t ≥ 0)
//   线段：P = A + u·(B-A)  (0 ≤ u ≤ 1)
//   联立解 2×2 线性方程组：
//     t = det([A-O, segDir]) / det(rayDir, segDir)
//     u = det([A-O, rayDir]) / det(rayDir, segDir)
//   行列式 = 0 → 平行 → 无交点
//   t > 0 且 0 ≤ u ≤ 1 → 有效交点，返回 t（射线原点到交点的距离）
//
// 【为什么用参数方程而非几何法？】
//   代码紧凑，易于扩展到 3D，计算量相同（都是 2 次叉积 + 1 次除法）。

/**
 * 2D 射线-线段求交
 *
 * @param ox, oy  射线原点（车辆位置）
 * @param dx, dy  射线单位方向向量
 * @param x1-x2   线段端点
 * @returns 交点到射线原点的距离 t，或 null（无有效交点）
 */
function raySegmentIntersect(
  ox: number, oy: number, dx: number, dy: number,
  x1: number, y1: number, x2: number, y2: number,
): number | null {
  const segX = x2 - x1, segY = y2 - y1;            // 线段方向向量
  const denom = dx * segY - dy * segX;              // 行列式 = rayDir × segDir
  if (Math.abs(denom) < 1e-10) return null;        // 平行或共线 → 无交点
  const t = ((x1 - ox) * segY - (y1 - oy) * segX) / denom;  // Cramer's Rule: t
  const u = ((x1 - ox) * dy - (y1 - oy) * dx) / denom;      // Cramer's Rule: u
  if (t > 0 && u >= 0 && u <= 1) return t;          // 交点在射线前方且在线段上
  return null;
}

// ═══════════════════════════════════════════════════
// HD Map 背景参考线（懒缓存）
// ═══════════════════════════════════════════════════

/**
 * 背景减除用的 Frenet 参考线。
 * 按轨道对象身份缓存（`buildFrenetReference` O(n) ≈ 200 点）。
 * 始终使用闭环参考线——围墙在整个环道上存在。
 *
 * 【必须按对象失效】浏览器在同一进程内 rebuildWorld 切换地图/场景（headless
 * 每次新进程，不会遇到）：若只缓存一次，切图后背景减除会继续用旧地图参考线，
 * 障碍命中被 |l| 误判为边界并过滤 → 规划器"看不见"障碍 → 小车直穿障碍物
 * （batch-hairpin-s4 浏览器实测，回归测试见 test/algo/perception-ref.test.ts）。
 */
let _bgFrenetRef: FrenetReference | null = null;
/** 缓存键 = 实际用作参考线的那份点列（赛道中心线 **或** 路线） */
let _bgRefKey: Point2D[] | null = null;

// ═══════════════════════════════════════════════════
// 边界线段缓存（射线求交用；按 track/route 对象身份失效）
// ═══════════════════════════════════════════════════

/** 本帧的逐段属性：路网用**路线**的、链式用**赛道**的（几何与分类阈值必须同源） */
function boundarySpansFor(track: TrackGeometry, route?: Route | null): AttrSpan[] | undefined {
  if (track.chains && track.chains.length > 0) {
    return route?.waypoints && route.waypoints.length >= 2 ? route.attrSpans : track.attrSpans;
  }
  return track.attrSpans;
}

interface BoundedSeg { x1: number; y1: number; x2: number; y2: number; minX: number; minY: number; maxX: number; maxY: number }
let _bdSegs: BoundedSeg[] = [];
let _bdTrack: TrackGeometry | null = null;
let _bdRoute: unknown = null;

/**
 * 供射线求交的"道路边界"线段（世界坐标，带包围盒供每帧粗筛）。
 *
 * 【为什么路网要单独处理】径向/开环地图的边界就是 inner/outer 多边形；**路网地图的
 * `outerBoundary` 是空的**（可行驶域由"到最近链的距离"判定，不是一个多边形）。照抄的话
 * 路网上**没有任何可撞的边界**：实测 60/60 条射线全部飞到 125m 上限——界面上就是那圈
 * 穿透路缘的扇形，而且**墙/护栏不再遮挡**（射线穿过去能打到后面的东西）。
 *
 * 【为什么只取"路线"那一条 ribbon】不能把全部 24 条链的 ribbon 都当边界：路口处**别的链的
 * 端帽**会横在自车前方，且它们相对路线参考线的 |l| 很小 → 会被判成**高置信障碍物**
 * （幻影障碍 → 规划器假停车）。语义上正确的说法是"LiDAR 只该被**我要走的那条路**的路缘挡住"。
 * 无路线时退回最长链（与 planner 的回退一致）。
 */
function getBoundarySegments(track: TrackGeometry, route: Route | null | undefined, spans: AttrSpan[] | undefined): BoundedSeg[] {
  const rw = route?.waypoints;
  if (_bdTrack === track && _bdRoute === (rw ?? null)) return _bdSegs;
  const polys: { x: Px; y: Px }[][] = [];
  if (track.chains && track.chains.length > 0) {
    const halfW = (track.halfWidth ?? TRACK_HALF_WIDTH) as number;
    const src = rw && rw.length >= 2 ? rw : track.centerPoints;
    // **逐段宽度**：1 车道匝道（半宽 1.875m）的路缘必须在 1.875m 处挡住射线，
    // 用地图级半宽（5.625m）会让 LiDAR"看到"一条并不存在的宽路面。
    polys.push(spans && spans.length > 0
      ? ribbonPolygonVar(src as Point2D[], halfWidthsAlongPoints(src as Point2D[], spans, halfW)).poly
      : ribbonPolygon(src, halfW));
  } else {
    polys.push(track.innerBoundary, track.outerBoundary);
  }
  const out: BoundedSeg[] = [];
  for (const poly of polys) {
    for (let i = 0; i + 1 < poly.length; i++) {
      const x1 = poly[i].x as number, y1 = poly[i].y as number;
      const x2 = poly[i + 1].x as number, y2 = poly[i + 1].y as number;
      out.push({ x1, y1, x2, y2, minX: Math.min(x1, x2), minY: Math.min(y1, y2), maxX: Math.max(x1, x2), maxY: Math.max(y1, y2) });
    }
  }
  _bdSegs = out; _bdTrack = track; _bdRoute = rw ?? null;
  return out;
}

/**
 * 背景减除用的参考线：**路网地图必须用"我要走的那条路"（路线）**。
 *
 * 路网地图的 `centerPoints` 只是"最长链"（无路线时的回退），它可能**不是自车所在的链**——
 * 用它算 |l| 会把路线 ribbon 的路缘判成高置信障碍物（实测 phantom_hit 870 帧），可能假停车。
 * 径向/开环地图不受影响（它们的 centerPoints 就是那条路）。缓存按点列**对象身份**失效（P6 教训）。
 */
function getBackgroundFrenetRef(track: TrackGeometry, route?: Route | null): FrenetReference {
  const rw = track.chains && route?.waypoints && route.waypoints.length >= 2 ? route.waypoints : null;
  const key = (rw ?? track.centerPoints) as Point2D[];
  if (!_bgFrenetRef || _bgRefKey !== key) {
    _bgFrenetRef = buildFrenetReference(key, rw ? false : (track.closed ?? true));
    _bgRefKey = key;
  }
  return _bgFrenetRef;
}

// ═══════════════════════════════════════════════════
// 命中点 Frenet 投影
// ═══════════════════════════════════════════════════

/**
 * 计算 LiDAR 射线命中点在 Frenet 坐标系中的 |l| 值
 *
 * 命中点世界坐标 = carPos + rayDir × dist
 * → cartesianToFrenet → 返回 |l|（横向偏移绝对值）
 *
 * 如果射线未命中（dist ≈ MAX），返回 null。
 * 只用传感器原始数据 + HD Map 参考线，不查障碍物列表。
 */
function hitFrenetAbsL(
  carX: number, carY: number,
  rayAngle: number, dist: number,
  ref: FrenetReference,
  maxDist: number,
): { absL: number; l: number; s: number } | null {
  if (dist >= maxDist - 1) return null;  // 未命中 → 无投影

  const hitX = carX + Math.cos(rayAngle) * dist;
  const hitY = carY + Math.sin(rayAngle) * dist;
  const fl = cartesianToFrenet(px(hitX), px(hitY), ref);
  // 同时给出**带符号**的 l：规划层要靠它判断"这个命中在不在我这条车道"
  return { absL: Math.abs(fl.l as number), l: fl.l as number, s: fl.s as number };
}

// ═══════════════════════════════════════════════════
// 主扫描
// ═══════════════════════════════════════════════════

/**
 * 执行一次完整的 LiDAR 扫描 + 背景减除
 *
 * 【管线（每条射线）】
 *   1. 对全部线段（赛道边界 + 障碍物）做射线求交 → 最近命中距离 minDist
 *   2. rawDistances.push(minDist) — 保留原始值给渲染层
 *   3. 对命中点做 Frenet 投影 → |l|
 *   4. 三级分类 → confidence ∈ {0, 0.5, 1}
 *   5. confidence=0 → distances.push(MAX)（过滤），否则 push(minDist)
 *
 * 【输出】
 *   - distances:   过滤后（规划层），只含障碍物命中
 *   - rawDistances: 过滤前（渲染层），含边界命中
 *   - confidences:  每条射线的置信度
 */
export function sense(snapshot: WorldSnapshot, rig: SensorSpec[] = DEFAULT_SENSOR_RIG): LidarScan {
  const { car, track, obstacles } = snapshot;
  const angles: Radians[] = [], distances: Px[] = [], rawDistances: Px[] = [], confidences: number[] = [];

  // ── 1. 收集所有待检测线段 ──
  const allSegments: Segment[] = [];
  // 本帧的半宽（地图级兜底值）与**逐段属性**：边界几何与分类阈值必须同源，
  // 否则匝道路缘（|l|=1.875m）会被按地图级 5.625m 的门槛当成高置信障碍。
  const halfWidthU = (track.halfWidth ?? TRACK_HALF_WIDTH) as number;
  const bdSpans = boundarySpansFor(track, snapshot.route);
  // 道路边界（径向/开环 = 内外多边形；路网 = 路线 ribbon，见 getBoundarySegments）。
  // 用包围盒粗筛掉 125m 之外的段——射线最长 125m，过滤是**等价变换**，不动结果只省算力。
  const RANGE = MAX_SENSOR_DIST as number;
  const cx0 = car.x as number, cy0 = car.y as number;
  for (const b of getBoundarySegments(track, snapshot.route, bdSpans)) {
    if (cx0 < b.minX - RANGE || cx0 > b.maxX + RANGE || cy0 < b.minY - RANGE || cy0 > b.maxY + RANGE) continue;
    allSegments.push({ x1: b.x1, y1: b.y1, x2: b.x2, y2: b.y2, boundary: true });
  }
  // 障碍物矩形 → 4 条线段/个
  for (const obs of obstacles) {
    for (const sgm of obstacleToSegments(obs)) {
      allSegments.push({ ...sgm, boundary: false, vehicle: obs.isVehicle === true });
    }
  }

  // ── 2. HD Map 参考线（懒缓存）──
  const bgRef = getBackgroundFrenetRef(track, snapshot.route);
  /**
   * **逐段半宽**：分类阈值必须用命中点所在路段的半宽，而不是地图级标量。
   *
   * 【为什么】路面与边界改成逐段宽度后，匝道上真实路缘在 |l|=1.875m 处；若仍按地图级
   * 5.625m 判（高置信门槛 3.375m），整条匝道路缘都会被当成**高置信障碍物**——
   * 实测车在匝道口（s=2528m）永久停住。这与"域/限速/复位崖都逐段"是同一件事：
   * 背景减除的判据也必须逐段，否则 HD Map 与感知口径不一致。
   */
  // 与边界几何同一个过渡函数（几何、分类、规划域三处同源）
  const localHalfWidthAt = taperedHalfWidthAt(bdSpans, halfWidthU);

  // ── 3. 扫描主循环 ──
  const carX = car.x as number, carY = car.y as number;
  const carAngle = car.angle as number;
  const maxDist = rigMaxRange(rig) as number;          // rig 级（背景减除参考线等仍按最大量程）

  // 按传感器套件铺射线（默认 rig = 单个前向 → 与历史逐位一致）
  const rayPlan: { sen: SensorSpec; offset: number }[] = [];
  for (const sen of rig) {
    for (let offset = -sen.fov / 2; offset <= sen.fov / 2 + 1e-9; offset += sen.resolution) {
      rayPlan.push({ sen, offset });
    }
  }

  const sensorOf: number[] = new Array(rayPlan.length).fill(0);
  const sensorMax: number[] = new Array(rayPlan.length).fill(maxDist as number);
  const hitS: number[] = new Array(rayPlan.length).fill(NaN);
  const hitL: number[] = new Array(rayPlan.length).fill(NaN);
  let ri = 0;

  for (const ray of rayPlan) {
    const { sen, offset } = ray;
    const i = ri++;
    sensorOf[i] = rig.indexOf(sen);
    sensorMax[i] = sen.maxRange as number;      // 本射线的传感器量程（之外是**盲区**，不是"空"）
    // 射线世界方向 = 车头朝向 + 传感器安装朝向 + 扇面偏移角；原点 = 车体原点 + 位姿（旋转到世界系）
    const worldAngle = carAngle + sen.pose.yaw + offset;
    const dx = Math.cos(worldAngle), dy = Math.sin(worldAngle);
    const ox = carX + Math.cos(carAngle) * (sen.pose.x as number) - Math.sin(carAngle) * (sen.pose.y as number);
    const oy = carY + Math.sin(carAngle) * (sen.pose.x as number) + Math.cos(carAngle) * (sen.pose.y as number);

    // 对全部线段求交，取最近距离。**哨兵用本传感器量程**：角雷达 30m 外是**盲区**而非"空"，
    // 用 rig 最大量程会让"看不见"伪装成"看到 125m 的空旷"（S4 之前也要保持诚实）。
    const senMax = sen.maxRange as number;
    let minDist = senMax;
    let hitBoundary = false;
    let hitVehicle = false;
    for (const seg of allSegments) {
      const t = raySegmentIntersect(ox, oy, dx, dy, seg.x1, seg.y1, seg.x2, seg.y2);
      if (t !== null && t < minDist) { minDist = t; hitBoundary = seg.boundary; hitVehicle = seg.vehicle === true; }
    }

    // 原始距离（渲染层使用）
    rawDistances.push(px(minDist));

    // ── 4. 背景减除 ──
    let confidence = 1;           // 默认：无命中 → 空旷 → 高置信
    let hitFrenet: { absL: number; l: number; s: number } | null = null;
    let filteredDist = minDist;

    if (minDist < senMax) {
      // 有命中 → Frenet 投影判断
      const hit = hitFrenetAbsL(ox, oy, worldAngle, minDist, bgRef, senMax);
      hitFrenet = hit;
      if (hit !== null) {
        const absL = hit.absL;
        // 该命中点所在路段的半宽 → 逐段阈值（宽路仍是地图级值 → 行为零变化）
        const hwLocal = localHalfWidthAt(hit.s);
        const chLocal = hwLocal * CONF_HIGH_RATIO;
        const clLocal = hwLocal * CONF_LOW_RATIO;
        // 【开环地图的端帽】端帽是"地图尽头"的已知结构（track 生成时补的），它横跨路面 →
        // |l| 很小，会被当成高置信障碍物（检测器报 phantom_hit）。判据：命中点落在
        // 参考线终点附近 ⇒ 已知边界（与侧墙同等对待）。
        const atMapEnd = !(track.closed ?? true)
          && hit.s >= (bgRef.totalS as number) - (m(2) as number);
        if (hitVehicle) {
          // 命中的是**车辆**：无论 |l| 多大都保持高置信（车辆不可能是道路边界）。
          // 否则外车道的真车会落进 0.6~0.8/0.8+ 两个模糊带被降成 0.5 → 检测器报 phantom_hit、
          // 规划器对真车不设防（实测踩过）。
          confidence = 1;
        } else if (hitBoundary || atMapEnd) {
          // 命中的就是 HD Map 的路缘/端帽 → 一定是背景（不依赖 |l| 投影比值，见 Segment.boundary）
          filteredDist = maxDist;   // "没有障碍"的哨兵用 **rig 级**：`distances` 只回答
                                    // "前方有没有东西"，不回答"我的传感器能看多远"（后者见 sensorMax）
          confidence = 0;
        } else if (absL > clLocal) {
          // 障碍物命中但贴得比 0.8×半宽还靠外 → 模糊带（可能是路缘/地图误差，低置信）
          confidence = 0.5;
        } else if (absL > chLocal) {
          // 0.6~0.8×半宽：边缘模糊区 → 低置信（可能是障碍物靠近路缘）
          confidence = 0.5;
        }
        // < 0.6×半宽：明确在道路内 → 高置信障碍物（confidence=1 不变）
      }
    }

    // 【S1① 用】真障碍命中（不是路缘/端帽）→ 记下 Frenet 位置：规划层据此判断
    // "在我这条车道、且在我前方"。旧口径是"全视野最近射线"，会把相邻车道（甚至侧后方）的车当限速源。
    if (filteredDist < maxDist && hitFrenet !== null) {
      hitS[i] = hitFrenet.s; hitL[i] = hitFrenet.l;
    }

    confidences.push(confidence);
    angles.push(normalizeAngle(rad(worldAngle)));   // 归一化到 [-π,π]，下游无需再处理环绕
    distances.push(px(filteredDist));
  }

  // ── 5. Debug 信息 ──
  const mid = Math.floor(distances.length / 2);
  let minD = maxDist, rawHits = 0;
  for (let i = 0; i < distances.length; i++) {
    if ((distances[i] as number) < minD) minD = distances[i] as number;
    if ((rawDistances[i] as number) < maxDist - 1) rawHits++;
  }
  debug.perception = {
    rayCount: distances.length,
    frontDist: distances[mid] as number,
    minDist: minD,
    minDistAngle: 0,
    rawHits,
    filteredHits: confidences.filter(c => c > 0).length,
  };

  return createLidarScan(angles, distances, rawDistances, confidences, hitS, hitL, sensorOf, sensorMax, rig.length);
}
