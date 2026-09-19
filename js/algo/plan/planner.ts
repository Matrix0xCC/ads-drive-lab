// ==================== 算法：规划器编排层（状态机驱动） ====================
//
// 【职责】接收 WorldSnapshot + LidarScan，生成最优 Trajectory。
//
// 【状态机】（详见 states.ts）
//   CRUISE ──障碍物──→ AVOID ──清除──→ CRUISE/FOLLOW
//   FOLLOW ──障碍物──→ AVOID
//   FOLLOW ──近终点──→ DECEL ──到达──→ ARRIVED
//   任意 ──全碰撞──→ EMERGENCY（单帧，下一帧重评估）
//
// 【每帧流程】
//   1. 选择 Frenet 参考线（route 存在→开环路线，不存在→闭环赛道）
//   2. 车辆笛卡尔→Frenet 投影，计算 dl/ds
//   3. ARRIVED 检测 → 提前返回空轨迹
//   4. 按状态配置速度目标 + 轨迹时长（DECEL 时减速+缩短时长）
//   5. 遍历候选 (targetL 横向目标已 clamp 到可行驶域, targetV 三档速度)：
//      a. generateCandidate → 轨迹点序列
//      b. 快速剪枝：前 4 点是否碰撞
//      c. evaluateTrajectory → 代价
//   6. 兜底：无安全候选 → EMERGENCY（直行减速 1s）
//   7. 状态转移 + 轨迹类型标签

import {
  WorldSnapshot, LidarScan, Route, FrenetReference,
  Trajectory, createTrajectory,
} from '../../shared/types.js';
import {
  Px, PxPerSec, DimLess, Seconds,
  m, mps, px, pxps, pxps2, dimless, perpx, secs,
} from '../../shared/dimensions.js';
import { buildFrenetReference, cartesianToFrenet } from './frenet.js';
import { TRACK_HALF_WIDTH } from '../../env/track.js';
import { PlanState, nextState, stateTrajectoryType } from './states.js';
import { debug } from '../../shared/debug.js';
import { detectObstacle, isLidarCollision, SAFETY_MARGIN } from './collision.js';
import { generateCandidate, evaluateTrajectory } from './candidate.js';

// ==================== 参数 ====================

const TRAJECTORY_DURATION: Seconds = secs(2.0);
const TRAJECTORY_STEPS = 20;

const SPEED_TARGETS: PxPerSec[] = [mps(17.5), mps(25), mps(35)];    // 63, 90, 126 km/h

const W_COLLISION = 1e6;
const W_SWITCH    = 100;
const DECEL_ZONE = m(25) as number;  // 25m 减速区

/** 车道中心偏移量（Lane 3 = +1.875m）。切换车道改此值即可。 */
let laneOffset: Px = m(1.875);  // 默认 Lane 3（中心偏右）

// ==================== 状态 ====================

let _loopFrenetRef: FrenetReference | null = null;
let _routeFrenetRef: FrenetReference | null = null;
let _lastRouteKey: string | null = null;
let _currentState: PlanState = PlanState.CRUISE;
let _prevBestTargetL: number | null = null;

// ==================== 主入口 ====================

export function plan(snapshot: WorldSnapshot, lidarScan: LidarScan, route?: Route): Trajectory {
  const { car, track } = snapshot;

  // ─── 1. Frenet 参考线 ───
  let frenetRef: FrenetReference;
  let routeActive = false;

  if (route && route.waypoints.length >= 2) {
    const key = `${route.startIndex}-${route.goalIndex}`;
    if (key !== _lastRouteKey) {
      _routeFrenetRef = buildFrenetReference(route.waypoints, false);
      _lastRouteKey = key;
      _prevBestTargetL = null;
    }
    frenetRef = _routeFrenetRef!; routeActive = true;
  } else {
    if (!_loopFrenetRef) _loopFrenetRef = buildFrenetReference(track.centerPoints, true);
    frenetRef = _loopFrenetRef;
  }

  // ─── 2. Frenet 状态 ───
  const fl = cartesianToFrenet(car.x, car.y, frenetRef);
  const tangent = frenetRef.tangents[fl.idx];
  const ta = Math.atan2(tangent.y, tangent.x);
  const hd = Math.atan2(Math.sin((car.angle as number) - ta), Math.cos((car.angle as number) - ta));
  const dl0 = dimless(Math.tan(hd));
  const cosHD = Math.cos(hd);
  const remainingDist = routeActive ? frenetRef.totalS - fl.s : Infinity;

  // ─── 3. ARRIVED ───
  if (routeActive && remainingDist <= 10) {
    _currentState = PlanState.ARRIVED;
    return createTrajectory([], 0, 'arrived');
  }

  // ─── 4. 状态配置 ───
  let speedTargets = SPEED_TARGETS;
  let duration = TRAJECTORY_DURATION;

  if (routeActive && remainingDist < DECEL_ZONE) {
    const s = Math.max(0.1, remainingDist / DECEL_ZONE);
    speedTargets = SPEED_TARGETS.map(v => pxps((v as number) * s));
    const cs = Math.max(car.speed as number, 30);
    duration = secs(Math.max(0.5, Math.min(TRAJECTORY_DURATION as number, remainingDist / cs * 1.2)));
  }

  // ─── 5. 候选评估 ───
  const cx = car.x as number, cy = car.y as number;
  let best: Trajectory | null = null;          // 安全候选中的最优
  let bestCost = Infinity;
  let bestTL = 0, bestTV = 0;
  // 全碰撞降级候选：优先最低速度（最小化碰撞能量），同速比代价。
  // 域内不可避让（如高速近距障碍）时用它替代"最便宜的碰撞轨迹"
  let fallback: Trajectory | null = null;
  let fallbackCost = Infinity;
  let fallbackTL = 0, fallbackTV = 0;
  const margin = (SAFETY_MARGIN as number) + (car.collisionRadius as number);
  const candidates: { targetL: number; targetV: number; cost: number; collision: boolean }[] = [];

  // 横向候选：以当前车道中心为基准，±7m/±14m 偏移（用于避障），
  // 但必须 clamp 到可行驶域内——域边界与 checkBounds 复位阈值一致
  //（半宽 − 车半宽 − 1.5m 复位缓冲），再留 0.25m 跟踪超调余量。
  // 越界目标会把车带向界外 → 复位传送 → 常在障碍正前方"复活"引发碰撞
  //（见 ISSUE_HISTORY 2026-09-19"横向候选超出可行驶域"）。clamp 后去重，
  // 该赛道上典型结果为 3 个横向目标（当前车道 / 左缘 / 右缘）。
  const lo = laneOffset as number;
  const L_MAX = (TRACK_HALF_WIDTH as number)
    - ((car.width as number) / 2 + (m(1.5) as number))
    - (m(0.25) as number);
  const latTargets: Px[] = [];
  for (const v of [lo, lo - (m(7) as number), lo + (m(7) as number), lo - (m(14) as number), lo + (m(14) as number)]) {
    const c = Math.max(-L_MAX, Math.min(L_MAX, v));
    if (!latTargets.some(t => Math.abs((t as number) - c) < 0.5)) latTargets.push(px(c));
  }

  for (const tL of latTargets) {
    for (const tV of speedTargets) {
      const points = generateCandidate(
        fl.l, dl0, perpx(0), fl.s,
        pxps((car.speed as number) * cosHD), pxps2(0),
        tL, tV, frenetRef, duration, TRAJECTORY_STEPS,
      );

      let skip = false;
      for (let i = 0; i <= 3 && i < points.length && !skip; i++) {
        if (isLidarCollision(points[i].x as number, points[i].y as number, cx, cy, lidarScan, margin)) skip = true;
      }
      if (skip) continue;

      const ev = evaluateTrajectory(points, frenetRef, cx, cy, lidarScan, lo, margin);
      let cost = ev.cost;
      if (_prevBestTargetL !== null && Math.abs((tL as number) - _prevBestTargetL) > 5 && (tL as number) !== 0) cost += W_SWITCH;
      if (!ev.collided) {
        if (cost < bestCost) { bestCost = cost; best = createTrajectory(points, cost, ''); bestTL = tL as number; bestTV = tV as number; }
      } else if (fallback === null || (tV as number) < fallbackTV - 1e-9
                 || (Math.abs((tV as number) - fallbackTV) < 1e-9 && cost < fallbackCost)) {
        fallback = createTrajectory(points, cost, ''); fallbackCost = cost;
        fallbackTL = tL as number; fallbackTV = tV as number;
      }
      candidates.push({ targetL: tL as number, targetV: tV as number, cost, collision: ev.collided });
    }
  }

  // ─── 5b. 选择：安全候选优先；无安全候选时用降级候选（最低速度） ───
  if (!best && fallback) {
    best = fallback;
    bestCost = fallbackCost;
    bestTL = fallbackTL;
    bestTV = fallbackTV;
  }

  // ─── 6. EMERGENCY ───
  if (!best) {
    _currentState = PlanState.EMERGENCY;
    const pts = generateCandidate(fl.l, dimless(0), perpx(0), fl.s, mps(10), pxps2(0), fl.l, mps(10), frenetRef, secs(1.0), TRAJECTORY_STEPS);
    return createTrajectory(pts, W_COLLISION, 'emergency');
  }

  // ─── 7. 状态转移 ───
  _currentState = nextState(_currentState, {
    routeActive, nearEnd: routeActive && remainingDist < DECEL_ZONE,
    atEnd: false, hasObstacle: detectObstacle(lidarScan),
    allBlocked: false, wasEmergency: false,
  });

  if (_currentState === PlanState.AVOID && best.points.length > 0) {
    const lp = best.points[best.points.length - 1];
    const lfl = cartesianToFrenet(lp.x, lp.y, frenetRef);
    best.type = stateTrajectoryType(_currentState, lfl.l as number, fl.l as number);
  } else {
    best.type = stateTrajectoryType(_currentState);
  }

  // debug
  debug.planning = {
    state: _currentState,
    frenetS: fl.s,
    frenetL: fl.l as number,
    dl_ds: dl0 as number,
    headingDiff: hd,
    candidates,
    bestCost,
    bestTargetL: bestTL,
    bestTargetV: bestTV,
    remainingDist: routeActive ? remainingDist : -1,
    speedScale: routeActive && remainingDist < DECEL_ZONE ? Math.max(0.1, remainingDist / DECEL_ZONE) : 1,
  };

  // ─── 8. 帧间平滑 ───
  if (best.points.length > 0) {
    const lp = best.points[best.points.length - 1];
    _prevBestTargetL = (cartesianToFrenet(lp.x, lp.y, frenetRef).l as number);
  }
  return best;
}

export function getPlannerState(): string {
  return _currentState;
}
