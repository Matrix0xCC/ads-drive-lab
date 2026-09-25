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
//   5. solvePathLayer → 最优路径（窗口内排名；物理层全阻塞时给出停车基准）
//   6. solveSpeedLayer → 巡航 / 停车线（物理层全阻塞触发）/ 兜底剖面
//   7. 合成 l(s(t)) → Trajectory；状态机 + 兼容类型标签

import {
  WorldSnapshot, LidarScan, Route, FrenetReference, TrackGeometry,
  Trajectory, TrajectoryPoint, createTrajectory,
} from '../../shared/types.js';
import {
  Px, m, mps, mps2, pxps, secs, dimless, perpx,
} from '../../shared/dimensions.js';
import { buildFrenetReference, cartesianToFrenet, frenetToCartesian } from './frenet.js';
import { PlanState, nextState } from './states.js';
import { debug } from '../../shared/debug.js';
import { solvePathLayer, PATH_STEPS, PATH_HORIZON } from './paths.js';
import { solveSpeedLayer, STObstacle } from './speed.js';

// ==================== 参数 ====================

const DECEL_ZONE = m(25) as number;        // 路线终点减速区
const STOP_ASSUMED_LEN = m(8) as number;   // 阻塞条目假定的障碍长度（仅影响 ST 条目 sMax）
const TIGHT_SPEED = mps(8);                // 紧间隙慢行限速（双阈值：物理可过但低于舒适间隙时）
const A_BRAKE = mps2(6);                   // 保守制动能力（与速度层 A_MAX 一致；紧间隙可达性评估用）

/** 车道中心偏移量（Lane 3 = +1.875m）。切换车道改此值即可。 */
let laneOffset: Px = m(1.875);

// ==================== 状态 ====================

// 参考线缓存按"世界对象身份"失效（track/route 对象变化 → 重建）。
// 浏览器在同一进程内 rebuildWorld 切换地图/场景（headless 每次新进程不会遇到），
// 若缓存不失效，换图后会继续用旧地图几何投影 → 规划/避障全错。
let _loopFrenetRef: FrenetReference | null = null;
let _loopRefTrack: TrackGeometry | null = null;
let _routeFrenetRef: FrenetReference | null = null;
let _lastRoute: Route | null = null;
let _currentState: PlanState = PlanState.CRUISE;
let _prevBestTargetL: number | null = null;

// ==================== 主入口 ====================

export function plan(snapshot: WorldSnapshot, lidarScan: LidarScan, route?: Route): Trajectory {
  const { car, track } = snapshot;

  // ─── 1. Frenet 参考线 ───
  let frenetRef: FrenetReference;
  let routeActive = false;
  if (route && route.waypoints.length >= 2) {
    if (route !== _lastRoute) {
      _routeFrenetRef = buildFrenetReference(route.waypoints, false);
      _lastRoute = route;
      _prevBestTargetL = null;
    }
    frenetRef = _routeFrenetRef!; routeActive = true;
  } else {
    if (!_loopFrenetRef || _loopRefTrack !== track) {
      _loopFrenetRef = buildFrenetReference(track.centerPoints, true);
      _loopRefTrack = track as TrackGeometry;
      _prevBestTargetL = null;
    }
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

  // 双阈值：走的路径低于舒适间隙（舒适阻塞但物理可过）→ 慢行通过
  if (pathLayer.best.blocked) targetV = Math.min(targetV, TIGHT_SPEED as number);

  // ─── 6. 速度层 ───
  // 停车线触发条件（二选一）：
  //   ① 物理层全阻塞（allBlockedHard）——所有候选连硬阈值都过不去（全宽墙/真死口袋）；
  //   ② 紧间隙候选不可达——慢行通过需在紧段起点前降到 TIGHT_SPEED，制动距离不够
  //      （30m/s 时需 ~70m；20m 外才发现只能减速通行的紧口 = 物理上停不住也绕不开）。
  // 只要存在"可达的物理可过"候选，就不停车（横向耐心、纵向保守、双阈值通行）。
  let stopEngaged = pathLayer.allBlockedHard;
  let stopLineS = pathLayer.sBlockHard;
  if (!stopEngaged && pathLayer.best.blocked) {
    const v0u = (car.speed as number) * cosHD;
    const dReq = (v0u * v0u - (TIGHT_SPEED as number) ** 2) / (2 * (A_BRAKE as number));
    if ((pathLayer.best.sBlock as number) - (fl.s as number) < dReq) {
      stopEngaged = true;
      stopLineS = pathLayer.best.sBlock;
    }
  }
  const stObstacles: STObstacle[] = stopEngaged
    ? [{ sMin: stopLineS - fl.s, sMax: stopLineS - fl.s + (STOP_ASSUMED_LEN as number), tMin: 0, tMax: Infinity }]
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
    allBlockedHard: stopEngaged,
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
