// ==================== 算法：规划器编排层（纵向-横向解耦） ====================
//
// 【架构】SL 路径层（paths.ts）+ ST 速度层（speed.ts），单轮协调后合成 Trajectory。
//   路径层决定"走哪条路"（横向，120m 视程）；速度层决定"以什么速度走"（纵向，
//   含停车线——横向无解时停在障碍前）。合成 l(s(t))，输出契约不变
//   （控制/回放/归档/检测零改动）。详见 spec：2026-09-19-decoupled-planner-design.md
//
// 【每帧流程】
//   1. Frenet 参考线（route 开环 / 赛道闭环）
//   2. 车辆投影 + dl/ds
//   3. ARRIVED 检测
//   4. 纵向巡航目标（vCap 单值版 + 终点 DECEL 缩放）
//   5. solvePathLayer → 最优路径（窗口内排名；全程阻塞时给出停车基准）
//   6. solveSpeedLayer → 巡航 / 停车线（全程阻塞触发）/ 兜底剖面
//   7. 合成 l(s(t)) → Trajectory；状态机 + 兼容类型标签

import {
  WorldSnapshot, LidarScan, Route, FrenetReference,
  Trajectory, TrajectoryPoint, createTrajectory,
} from '../../shared/types.js';
import {
  Px, m, mps, pxps, secs, dimless, perpx,
} from '../../shared/dimensions.js';
import { buildFrenetReference, cartesianToFrenet, frenetToCartesian } from './frenet.js';
import { PlanState, nextState } from './states.js';
import { debug } from '../../shared/debug.js';
import { solvePathLayer, PATH_STEPS, PATH_HORIZON } from './paths.js';
import { solveSpeedLayer, STObstacle } from './speed.js';

// ==================== 参数 ====================

const DECEL_ZONE = m(25) as number;        // 路线终点减速区
const STOP_ASSUMED_LEN = m(8) as number;   // 阻塞条目假定的障碍长度（仅影响 ST 条目 sMax）

/** 车道中心偏移量（Lane 3 = +1.875m）。切换车道改此值即可。 */
let laneOffset: Px = m(1.875);

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

  // ─── 4. 纵向巡航目标（单值）───
  const minObsDist = Math.min(...lidarScan.distances);
  let targetV = Math.min(mps(35) as number, Math.max(mps(17.5) as number, minObsDist / 2));
  if (routeActive && remainingDist < DECEL_ZONE) {
    targetV *= Math.max(0.1, remainingDist / DECEL_ZONE);
  }

  // ─── 5. 路径层 ───
  const pathLayer = solvePathLayer({
    ref: frenetRef, l0: fl.l, dl0, s0: fl.s,
    carX: car.x as number, carY: car.y as number, lidarScan,
    carWidth: car.width, collisionRadius: car.collisionRadius,
    laneOffset, prevTargetL: _prevBestTargetL,
  });

  // ─── 6. 速度层 ───
  // 停车线由"全程阻塞"（allBlockedFar）触发：所有候选在整段视程内都被堵（全宽墙/
  // 死口袋）才刹车；只要还有候选远端畅通，速度交给 vCap（横向耐心、纵向保守）。
  const stObstacles: STObstacle[] = pathLayer.allBlockedFar
    ? [{ sMin: pathLayer.sBlockFar - fl.s, sMax: pathLayer.sBlockFar - fl.s + (STOP_ASSUMED_LEN as number), tMin: 0, tMax: Infinity }]
    : [];
  const speedLayer = solveSpeedLayer({ stObstacles, v0: (car.speed as number) * cosHD, targetV });

  // ─── 7. 合成 l(s(t)) ───
  const duration = speedLayer.profile.duration;
  const points: TrajectoryPoint[] = [];
  for (let i = 0; i <= PATH_STEPS; i++) {
    const t = (i / PATH_STEPS) * duration;
    const sRel = Math.max(0, Math.min(PATH_HORIZON as number, speedLayer.profile.eval(t)));
    const li = pathLayer.best.evalL(sRel);
    const cart = frenetToCartesian(fl.s + sRel, li, frenetRef);
    points.push({ t: secs(t), x: cart.x, y: cart.y, yaw: cart.yaw,
      v: pxps(speedLayer.profile.deriv1(t)), kappa: perpx(0) });
  }

  // ─── 8. 状态机 ───
  _currentState = nextState({
    routeActive,
    nearEnd: routeActive && remainingDist < DECEL_ZONE,
    allBlockedFar: pathLayer.allBlockedFar,
    stopInfeasible: speedLayer.mode === 'emergency',
    avoiding: Math.abs((pathLayer.best.targetL as number) - (laneOffset as number)) > 3,
  });

  // ─── 9. 兼容类型标签（overlays 颜色 / HUD 方向 / 检测器 / 测试依赖）───
  let type: string;
  if (speedLayer.mode === 'emergency') type = 'emergency';
  else if (Math.abs((pathLayer.best.targetL as number) - (fl.l as number)) > 3) {
    type = (pathLayer.best.targetL as number) > (fl.l as number) ? 'avoid_left' : 'avoid_right';
  } else type = 'lane_keep';

  // ─── 10. debug（inspect 面板 / 归档记录）───
  debug.planning = {
    state: _currentState,
    frenetS: fl.s, frenetL: fl.l as number,
    dl_ds: dl0 as number, headingDiff: hd,
    paths: pathLayer.ranked.map(p => ({ targetL: p.targetL as number, blocked: p.blocked, cost: p.cost })),
    bestCost: pathLayer.best.cost,
    bestTargetL: pathLayer.best.targetL as number,
    speedTarget: targetV,
    speedMode: speedLayer.mode,
    remainingDist: routeActive ? remainingDist : -1,
  };

  // ─── 11. 帧间平滑 ───
  _prevBestTargetL = pathLayer.best.targetL as number;

  return createTrajectory(points, pathLayer.best.cost, type);
}

export function getPlannerState(): string {
  return _currentState;
}
