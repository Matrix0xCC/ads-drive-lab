// ==================== 环境：车辆模型 + 物理模拟 + 安全约束 ====================
//
// 【职责】
//   1. createCar() — 创建车辆状态对象
//   2. stepPhysics() — 接收 ControlCommand，执行一帧的物理积分
//   3. checkBounds() — 越界检测与复位
//
// 在真实自动驾驶架构中，这些属于"被控对象模型"（Plant Model），
// 属于仿真环境的一部分，不包含任何决策逻辑。
//
// 【运动学自行车模型（Kinematic Bicycle Model）】
//
//        前轮
//         ○
//        /|\
//       / | \  ← 前轮转角 δ
//      /  |  \
//     /   |   \
//    ○────○────○  ← 后轴（非转向轮）
//    ←—— L ——→     L = 轴距 (Wheelbase)
//
// 核心方程：
//   横摆角速度：ω = v / L × tan(δ)
//   转弯半径：  R = L / tan(δ)
//
// 【为什么不用更复杂的动力学模型？】
// 动力学模型需要轮胎模型（Pacejka）、载荷转移等参数，
// 对于 2D 俯视 Demo 来说过于复杂。自行车模型在低速下精度足够，
// 且计算量极小（每帧只需几次浮点运算）。

import { CarState, ControlCommand, TrackGeometry, Point2D } from '../shared/types.js';
import { Radians, Px, PxPerSec, PxPerSec2, DimLess, SIM_UNITS_PER_METER, rad, m, mps, mps2, px, pxps, pxps2, dimless, yawRate, normalizeAngle } from '../shared/dimensions.js';

/** 轴距 3m（与 algo/control.ts 保持一致） */
const WHEELBASE: Px = m(3.0);

// ═══════════════════════════════════════════════════
// 车辆工厂
// ═══════════════════════════════════════════════════

/**
 * 创建车辆状态对象
 *
 * @param startPoint 起点（赛道中心线第一个点）
 * @param nextPoint  第二点（用于确定初始朝向）
 * @returns CarState（所有字段带量纲标注）
 */
export function createCar(startPoint: Point2D, nextPoint: Point2D): CarState {
  const sx = startPoint.x as number;
  const sy = startPoint.y as number;
  const nx = nextPoint.x as number;
  const ny = nextPoint.y as number;
  return {
    x: startPoint.x,
    y: startPoint.y,
    angle: rad(Math.atan2(ny - sy, nx - sx)), // 车头朝向 = 赛道起点方向
    speed: mps(25),              // 初始速度 25 m/s = 90 km/h
    steerAngle: rad(0),
    maxSpeed: mps(42.5),         // 最高速度 42.5 m/s ≈ 153 km/h
    acceleration: mps2(8),       // 加速度 8 m/s²（0-100km/h ≈ 3.5s）
    friction: dimless(0.12),     // 线性阻力系数（终端速度 8/0.12=67m/s ≫ maxSpeed 42.5）
    width: m(2.0),               // 车宽 2.0m
    length: m(4.5),              // 车长 4.5m
    collisionRadius: px(Math.hypot(m(2.0) as number, m(4.5) as number) / 2),
  };
}

// ═══════════════════════════════════════════════════
// 物理模拟（运动学自行车模型 + 中点法积分）
// ═══════════════════════════════════════════════════

/**
 * 执行一帧的车辆物理积分
 *
 * 【物理流程】
 * 1. 前轮转角 ← ControlCommand.steerAngle
 * 2. 速度 → 向 targetSpeed 收敛 → 施加线性空气阻力
 * 3. 横摆角速度：ω = v/L × tan(δ)（yawRate 函数保证量纲正确）
 * 4. 位置：中点法积分（用旋转前后的平均角度分解速度）
 *
 * 【中点法 vs 前向欧拉】
 * 前向欧拉：用旋转后的角度更新位置 → 轻微外向偏差
 * 中点法：  用旋转前后的平均角度 → 二阶精度，无系统偏差
 *
 * 【量纲安全检查】
 * car.angle (Radians) += yawRate (RadPerSec) × dt (number)
 * → TypeScript 通过 yawRate() 返回 RadPerSec 保证量纲正确 ✓
 *
 * @param car     CarState（原地修改）
 * @param command ControlCommand { steerAngle, throttle }
 * @param dt      时间步长（秒）
 */
export function stepPhysics(car: CarState, command: ControlCommand, dt: number): void {
  car.steerAngle = command.steerAngle;

  // —— 速度控制 ——
  // 向目标速度加速或减速
  const targetSpeed = pxps((command.throttle as number) * (car.maxSpeed as number));
  const ts = targetSpeed as number;
  const cs = car.speed as number;
  const acc = car.acceleration as number;

  if (cs < ts) {
    car.speed = pxps(cs + acc * dt > ts ? ts : cs + acc * dt);
  } else {
    car.speed = pxps(cs - acc * dt < ts ? ts : cs - acc * dt);
  }

  // 线性空气阻力：F_drag ∝ speed，a_drag = friction × speed
  const friction = car.friction as number;
  const newSpeed = (car.speed as number) * (1 - friction * dt);
  car.speed = pxps(newSpeed < 0 ? 0 : newSpeed);

  // —— 运动学自行车模型：横摆角速度 = v/L × tan(δ) ——
  const prevAngle = car.angle;
  if ((car.speed as number) > 0.1) {
    // yawRate() 是量纲感知的辅助函数：PxPerSec / Px × DimLess → RadPerSec ✓
    const yr = yawRate(car.speed, WHEELBASE, car.steerAngle);
    car.angle = rad((car.angle as number) + (yr as number) * dt);
  }

  // —— 中点法：用旋转前后的平均角度更新位置 ——
  // 相比前向欧拉法（用旋转后的角度），中点法消除了一阶积分偏差
  const avgAngle = ((prevAngle as number) + (car.angle as number)) / 2;
  const step = (car.speed as number) * dt;
  car.x = px((car.x as number) + Math.cos(avgAngle) * step);
  car.y = px((car.y as number) + Math.sin(avgAngle) * step);

  // 角度归一化到 [-π, π]（放在位置更新之后：中点法需要未折叠的前后角均值）
  car.angle = normalizeAngle(car.angle);
}

// ═══════════════════════════════════════════════════
// 越界检测与复位
// ═══════════════════════════════════════════════════

/**
 * 射线法判断点是否在多边形内
 *
 * 【算法：Ray Casting】
 * 从点向右发射水平射线，统计与多边形边的交点数量。
 * 奇数次 = 内部，偶数次 = 外部。
 */
function pointInPolygon(px: number, py: number, polygon: Point2D[]): boolean {
  let inside = false;
  for (let i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
    const xi = polygon[i].x as number, yi = polygon[i].y as number;
    const xj = polygon[j].x as number, yj = polygon[j].y as number;
    if ((yi > py) !== (yj > py) && (px < (xj - xi) * (py - yi) / (yj - yi) + xi)) {
      inside = !inside;
    }
  }
  return inside;
}

/** 判断点是否在赛道范围内（在外边界内且不在内边界内） */
function isInsideTrack(px: number, py: number, inner: Point2D[], outer: Point2D[]): boolean {
  return pointInPolygon(px, py, outer) && !pointInPolygon(px, py, inner);
}

/**
 * 越界复位：将车辆重置到最近的赛道中心点
 *
 * 复位时保持最低速度 60 px/s，防止停车在边界上再次触发越界。
 */
function resetToTrack(car: CarState, centerPoints: Point2D[], closed = true, chains?: Point2D[][]): void {
  // 路网：先把车挪到**最近链上最近的点**，再按该链的局部切向定朝向
  if (chains && chains.length > 0) {
    let bestD = Infinity, bestChain = chains[0], bestSeg = 0, bestT = 0;
    for (const ch of chains) {
      for (let i = 0; i + 1 < ch.length; i++) {
        const a = ch[i], b = ch[i + 1];
        const dx = (b.x as number) - (a.x as number), dy = (b.y as number) - (a.y as number);
        const l2 = dx * dx + dy * dy || 1;
        let t = (((car.x as number) - (a.x as number)) * dx + ((car.y as number) - (a.y as number)) * dy) / l2;
        t = Math.max(0, Math.min(1, t));
        const d = Math.hypot((car.x as number) - ((a.x as number) + t * dx), (car.y as number) - ((a.y as number) + t * dy));
        if (d < bestD) { bestD = d; bestChain = ch; bestSeg = i; bestT = t; }
      }
    }
    const a = bestChain[bestSeg], b = bestChain[Math.min(bestSeg + 1, bestChain.length - 1)];
    car.x = px((a.x as number) + bestT * ((b.x as number) - (a.x as number)));
    car.y = px((a.y as number) + bestT * ((b.y as number) - (a.y as number)));
    car.angle = rad(Math.atan2((b.y as number) - (a.y as number), (b.x as number) - (a.x as number)));
    car.speed = pxps(Math.max(car.speed as number, mps(10) as number));
    return;
  }
  let minDist = Infinity, bestIdx = 0;
  const n = closed ? centerPoints.length - 1 : centerPoints.length;
  const cx = car.x as number, cy = car.y as number;
  for (let i = 0; i < n; i++) {
    const d = Math.hypot(cx - (centerPoints[i].x as number), cy - (centerPoints[i].y as number));
    if (d < minDist) { minDist = d; bestIdx = i; }
  }
  const pt = centerPoints[bestIdx];
  // 开环：末端没有"下一个点"，用前一个点定朝向（否则会绕回起点，方向反了）
  const nextPt = closed ? centerPoints[(bestIdx + 1) % n]
    : centerPoints[Math.min(bestIdx + 1, n - 1)] === pt
      ? centerPoints[Math.max(0, bestIdx - 1)]
      : centerPoints[Math.min(bestIdx + 1, n - 1)];
  car.x = pt.x;
  car.y = pt.y;
  car.angle = rad(Math.atan2(
    (nextPt.y as number) - (pt.y as number),
    (nextPt.x as number) - (pt.x as number)
  ));
  car.speed = pxps(Math.max(car.speed as number, mps(10) as number)); // 不低于 10 m/s
}

/** 点到线段的最短距离 */
function distToSegment(px: number, py: number, x1: number, y1: number, x2: number, y2: number): number {
  const dx = x2 - x1, dy = y2 - y1;
  const len2 = dx * dx + dy * dy;
  if (len2 === 0) return Math.hypot(px - x1, py - y1);
  let t = ((px - x1) * dx + (py - y1) * dy) / len2;
  t = Math.max(0, Math.min(1, t));
  return Math.hypot(px - (x1 + t * dx), py - (y1 + t * dy));
}

/**
 * 路网地图的可行驶域：**到最近链的距离 + 碰撞半径 ≤ 半宽**。
 *
 * 【为什么不用多边形】路网是若干条链的**并集**（路口处重叠），无法表达成"外多边形挖内多边形"；
 * 距离判据天然处理并集与路口。端部也自动成立：越过链末端后"到最近链的距离"会增大 → 判越界
 * （等价于端墙）。
 */
function isInsideNetwork(cx: number, cy: number, r: number, track: TrackGeometry): boolean {
  const chains = track.chains!;
  const mapHalfW = (track.halfWidth ?? m(7.5)) as number;
  let best = Infinity;
  for (let ci = 0; ci < chains.length; ci++) {
    const ch = chains[ci];
    // **逐链半宽**：1 车道匝道（半宽 1.875m）与 4 车道主路（7.5m）不是同一条边界。
    // 缺属性 → 回退地图级标量（径向图与旧地图行为逐位不变）。
    const attrs = track.chainAttrs?.[ci];
    const halfW = attrs ? attrs.halfWidthM * SIM_UNITS_PER_METER : mapHalfW;
    // 复位缓冲按该链宽度缩放（窄匝道不能扣掉固定 0.5m，见 boundsBufferFor）
    const rLocal = (r - (BOUNDS_BUFFER as number)) + boundsBufferFor(halfW, (r - (BOUNDS_BUFFER as number)));
    for (let i = 0; i + 1 < ch.length; i++) {
      const a = ch[i], b = ch[i + 1];
      best = Math.min(best, distToSegment(cx, cy, a.x as number, a.y as number, b.x as number, b.y as number));
      // 【提前退出条件必须是"已经确定在内"】不能写成 `best <= halfW`：那只要**任一**链段
      // 落在半宽内就下结论，而 24 条链里先遍历到的邻近链会把结果定成越界（实测：车精确在链上、
      // 距离 0.0m，却连续复位 3400+ 次）。正确判据是"到**最近链**的距离 + 车半径 ≤ 该链半宽"。
      if (best + rLocal <= halfW + 1e-9) return true;
    }
  }
  return false;
}

/** 检查车体（中心+碰撞半径）是否在赛道内 */
function isCarInsideTrack(cx: number, cy: number, r: number, track: TrackGeometry): boolean {
  if (track.chains && track.chains.length > 0) return isInsideNetwork(cx, cy, r, track);
  // 距离外边界太近 → 越界
  for (let i = 0; i < track.outerBoundary.length - 1; i++) {
    const p = track.outerBoundary[i], n = track.outerBoundary[i + 1];
    if (distToSegment(cx, cy, p.x as number, p.y as number, n.x as number, n.y as number) < r) return false;
  }
  // 进入内边界 → 越界
  for (let i = 0; i < track.innerBoundary.length - 1; i++) {
    const p = track.innerBoundary[i], n = track.innerBoundary[i + 1];
    if (distToSegment(cx, cy, p.x as number, p.y as number, n.x as number, n.y as number) < r) return false;
  }
  // 用射线法确认中心点在外边界内、内边界外
  if (!isInsideTrack(cx, cy, track.innerBoundary, track.outerBoundary)) return false;
  return true;
}

/** 复位缓冲（车半宽之外的额外余量）：车到赛道边界近于此值即复位到中心线。
 *  它同时决定规划域上限（paths.ts 的 L_MAX = TRACK_HALF_WIDTH − (车半宽 + 本值) − 0.6m 过冲预留），
 *  两者必须同步改——早期只放宽规划域会撞上复位崖（P8 贴边棘轮家族）。
 *  A/B：1.5m（旧）→ 0.5m 让车能多用两侧各约 1m 路面（复位崖 5.0 → 6.0m）。 */
export const BOUNDS_BUFFER: Px = m(0.5);

/**
 * 复位缓冲**按路段宽度缩放**：0.5m 在 7.5m 半宽的路上是 7% 的余量，但在 1.875m 半宽的
 * 单车道匝道上会吃掉近 60% 的车中心可用空间（3.75m 车道 − 2m 车宽 = ±0.875m，再减 0.5m
 * 只剩 ±0.375m）——而最近邻投影误差就有几厘米，于是"规划域 = 复位崖 = 比误差还窄"→
 * 出现"候选路径起点就在走廊外 → 全判碰撞 → 永久停车"的死锁（实测三环场景卡死在匝道入口）。
 *
 * 规则：缓冲 = min(0.5m, 40% × 车中心物理余量)。宽路（径向 6.5m 余量）→ 仍是 0.5m（零回归）；
 * 窄路按比例缩到 ~0.35m，让车有可用的中心行程，同时保持"域边 = 复位崖"这条不变量。
 */
export function boundsBufferFor(halfWidthPx: number, carHalfPx: number): number {
  const free = Math.max(0, halfWidthPx - carHalfPx);
  return Math.min(BOUNDS_BUFFER as number, 0.4 * free);
}

export function checkBounds(car: CarState, track: TrackGeometry): boolean {
  // 单中心线地图：缓冲按地图半宽缩放（窄图同理，见 boundsBufferFor）
  const hw = (track.halfWidth ?? m(7.5)) as number;
  const carHalf = (car.width as number) / 2;
  const r = carHalf + boundsBufferFor(hw, carHalf);
  if (!isCarInsideTrack(car.x as number, car.y as number, r, track)) {
    resetToTrack(car, track.centerPoints, track.closed ?? true, track.chains);
    return true;
  }
  return false;
}
