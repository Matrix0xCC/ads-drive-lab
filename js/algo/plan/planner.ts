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
//   5. 路径层 × 速度上限收敛迭代（≤3 轮）→ 最优路径（窗口内排名；物理层全阻塞时给出停车基准）
//      与迭代末轮速度上限（横向机动/紧间隙/封死）
//   6. solveSpeedLayer → 巡航 / 停车线（物理层全阻塞触发）/ 兜底剖面
//   7. 合成 l(s(t)) → Trajectory；状态机 + 兼容类型标签

import {
  WorldSnapshot, LidarScan, Route, FrenetReference, TrackGeometry, AttrSpan,
  Trajectory, TrajectoryPoint, createTrajectory, attrsAtS,
} from '../../shared/types.js';
import {
  Px, PxPerSec, SIM_UNITS_PER_METER, m, mps, mps2, px, pxps, secs, dimless, perpx,
} from '../../shared/dimensions.js';
import { TRACK_HALF_WIDTH } from '../../env/track.js';
import { buildFrenetReference, cartesianToFrenet, frenetToCartesian } from './frenet.js';
import { PlanState, nextState } from './states.js';
import { debug } from '../../shared/debug.js';
import type { PlanningViz } from '../../shared/debug.js';
import { solvePathLayer, lateralSpeedCap, PATH_STEPS, PATH_HORIZON, TIGHT_SPEED, A_LAT_MAX } from './paths.js';
import type { PathLayerResult } from './paths.js';
import { solveSpeedLayer, STObstacle } from './speed.js';
import { nextTight, INITIAL_TIGHT, TightMemory } from './tight.js';
import { refCurvatureAt } from './margin.js';
import { corridorBandPolygons, corridorBandObserved, corridorObservedLimitS, corridorPinchSegments, corridorCrossSectionSegments, coverageFromScan, bandBoundary as unobservedBoundarySegments } from './corridor.js';

// ==================== 参数 ====================

/** 道路曲率限速开关（A/B；**默认关 = 旧行为**）：开 = 按 A_LAT_MAX 对**路弯**限速。
 *  【为什么默认关】它改变**所有**地图的速度剖面（径向图弯道半径 80~100m → 限速 17~19m/s），
 *  属语义变更：必须单独 A/B + 基线重取，不能在 M1 里顺手打开（见 spec 2026-09-26 执行记录）。
 *  OSM 高速图（最急 R=141m）在关闭时会以 vCap 上限 35m/s 跑完含弯全程。 */
export let CURVATURE_SPEED_LIMIT = false;
/** 运行时开关（headless `--curvature-limit` / 浏览器调试）：见常量注释的 A/B 说明 */
export function setCurvatureSpeedLimit(on: boolean): void { CURVATURE_SPEED_LIMIT = on; }

const DECEL_ZONE = m(25) as number;        // 路线终点减速区
const STOP_ASSUMED_LEN = m(8) as number;   // 阻塞条目假定的障碍长度（仅影响 ST 条目 sMax）
const A_BRAKE = mps2(6);                   // 保守制动能力（与速度层 A_MAX 一致；紧间隙可达性评估用）
/** 收敛迭代上限（spec §5.1：最多 3 轮） */
const MAX_ROUNDS = 3;
/** 横向目标收敛阈（0.2m） */
const CONV_TARGET_L = m(0.2) as number;
/** 速度上限收敛阈（0.5m/s） */
const CONV_SPEED = mps(0.5) as number;

/** 回放可视化：用归档的扫描 + 车位姿重建走廊载荷（不跑规划/速度层，仅供渲染）。
 *  回放不跑规划器，debug.planning.viz 是实时残留 → 回放模式下走廊可视化走这里。
 *  参考线由调用方缓存传入（ref 按路线/赛道建一次即可）。 */
export function corridorVizForReplay(
  ref: FrenetReference, lidarScan: LidarScan, carX: number, carY: number, sEnd?: number,
  laneOffsetU?: number,
): PlanningViz {
  const fl = cartesianToFrenet(px(carX), px(carY), ref);
  const pathLayer = solvePathLayer({
    ref, l0: fl.l, dl0: dimless(0), s0: fl.s, v0: 0,
    carX, carY, lidarScan,
    carWidth: m(2.0), collisionRadius: px(Math.hypot(m(2.0) as number, m(4.5) as number) / 2),
    laneOffset: px(laneOffsetU ?? (LEGACY_LANE_OFFSET as number)), prevTargetL: null,
    sEnd,
  });
  return buildVizPayload(pathLayer, ref, lidarScan, carX, carY);
}

/** 走廊可视化载荷：带子（2× 采样，边界更贴曲率）+ 观测覆盖 + 瓶颈线 + 候选（引用路径层对象）。
 *  观测覆盖用 **rawDistances**（真实命中含边界）：边界命中被 HD Map 背景减除从规划输入里剔除了，
 *  墙后的阴影区在走廊看来是"自由"，可视化必须标成"未观测"，否则图会撒谎。 */
function buildVizPayload(
  pl: PathLayerResult, ref: FrenetReference, scan: LidarScan, carX: number, carY: number,
) {
  const bands = corridorBandPolygons(pl.vizCorridor, ref);
  const cov = coverageFromScan(scan as { angles: number[]; distances: number[]; rawDistances?: number[] }, carX, carY);
  const bandObserved = corridorBandObserved(bands, cov.rays, carX, carY, cov.halfSpacing);
  const limitS = corridorObservedLimitS(pl.vizCorridor, ref, cov.rays, carX, carY, cov.halfSpacing);
  const unseenQuads = bands.filter((q, i) => !bandObserved[i] && q.length >= 4);
  return {
    bands,
    bandObserved,
    unobsBoundary: unobservedBoundarySegments(unseenQuads),
    obsLimitSegments: limitS === null ? [] : corridorCrossSectionSegments(pl.vizCorridor, ref, limitS, pl.lMaxU),
    pinchSegments: corridorPinchSegments(pl.vizCorridor, ref, pl.lMaxU),
    pinch: pl.vizCorridor.pinch
      ? { s: pl.vizCorridor.pinch.s, width: pl.vizCorridor.pinch.width,
          sealed: pl.vizCorridor.pinch.seg === null }
      : null,
    stepU: (PATH_HORIZON as number) / (PATH_STEPS * 2),
    candidates: pl.ranked,
    bestTargetL: pl.best.targetL as number,
  };
}

/** 路径几何 → 速度上限（spec §5.1 的"速度上限 u_k"）：物理封死 → 0；紧间隙 → 慢行档；
 *  横向机动 → lateralSpeedCap（发展的横向加速度不超舒适上限）；否则巡航目标 vCap。 */
function pathSpeedCap(pl: PathLayerResult, l0: number, vCap: number): number {
  if (pl.allBlockedHard) return 0;
  const lateral = lateralSpeedCap(Math.abs((pl.best.targetL as number) - l0), pl.best.develop);
  const tight = pl.best.blocked ? (TIGHT_SPEED as number) : Infinity;
  return Math.max(0, Math.min(vCap, lateral, tight));
}

/** 名义车道的**历史回退值**（径向图 4 车道的内侧右车道 +1.875m）。
 *  现在名义车道来自**地图**（`track.laneOffset`，由半宽/车道宽推导——见 track.ts 的 laneCentersOf）；
 *  这里只用于"地图未声明"的场景（例如回放可视化无 track）。 */
const LEGACY_LANE_OFFSET: Px = m(1.875);

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
/** 紧间隙迟滞状态（tight.ts；世界切换时重置） */
let _tight: TightMemory = INITIAL_TIGHT;

/**
 * 道路曲率限速：`v ≤ √(A_LAT_MAX/κ(s))`，并叠加"前视制动可行"约束
 * `v² ≤ v_lim(s)² + 2·a_brake·(s−s0)` → 取全程最小值（越远越松）。
 *
 * 【为什么需要】原有 `lateralSpeedCap` 管的是**变道机动**的横向加速度，与**路弯**无关。
 * OSM 实测（京港澳 G4 杜家坎段，最急 R=141m）：不限速时车以 vCap 上限 35m/s 跑完含弯全程
 * （横向加速度 8.7m/s² ≫ A_LAT_MAX=3.5），与真实限速语义脱节。
 * 【口径】κ 用 `refCurvatureAt`（±2.5m 三点外接圆），对 3m 采样稳健。
 * 【注意】本项目车辆是运动学模型（无侧向附着极限），该限速**只作用于规划器**。
 */
/** 当前参考线上的逐段属性（有路线用路线的，否则用赛道的） */
function attrSpans(track: TrackGeometry, route?: Route): AttrSpan[] | undefined {
  return route?.attrSpans ?? track.attrSpans;
}

/** 半宽随 s（px）：逐段属性存在时给出查表函数，否则 undefined（用常量） */
function halfWidthAtFn(track: TrackGeometry, route?: Route): ((s: number) => number) | undefined {
  const spans = attrSpans(track, route);
  if (!spans || spans.length === 0) return undefined;
  const fallback = (track.halfWidth as number) ?? TRACK_HALF_WIDTH;
  return (s: number) => {
    const a = attrsAtS(spans, s);
    return a ? a.halfWidthM * SIM_UNITS_PER_METER : fallback;
  };
}

/**
 * 逐段限速的**制动可行**上限（**px/s**，与规划器其余部分同一量纲）：地图限速沿路变化，
 * 必须在到达"更低限速段"之前就把速度降下来。判据：`v² ≤ v_lim(s')² + 2·a_brake·(s'−s0)`
 * 对前方所有 s' 成立（`ds` 用 px、`a_brake` 用 px/s²、`v` 用 px/s，量纲自洽）。
 *
 * 【量纲坑】逐段属性里的限速是 km/h（人读），必须 ×SIM_UNITS_PER_METER 转成 px/s 再比较；
 * 实测写成 m/s 时 `min(133.3, 33.3)` 会把巡航目标砍成 33.3px/s = 8.3m/s，车全程慢行档。
 *
 * 采样：固定 10m 网格 **加上每一段的边界**——限速是台阶式下降，网格采样可能整个跨过台阶起点，
 * 必须把边界本身作为约束点采到（取 Δs 最小的那个下降沿）。
 * 无逐段属性时退化成"常量上限"（只看 s0，不看前方）——与旧行为逐位一致。
 */
function speedLimitCap(
  spans: AttrSpan[] | undefined, fallbackMps: PxPerSec | undefined,
  s0: number, horizon: number,
): number | undefined {
  const aBrake = A_BRAKE as number;
  const kphToPxPerSec = (kph: number) => mps(kph / 3.6) as number;
  let cap = Infinity;
  const consider = (sAbs: number, vPx: number) => {
    const ds = sAbs - s0;
    if (ds < 0 || ds > horizon || !Number.isFinite(vPx)) return;
    cap = Math.min(cap, Math.sqrt(vPx * vPx + 2 * aBrake * ds));
  };
  if (spans && spans.length > 0) {
    const STEP = m(10) as number;
    for (let ds = 0; ds <= horizon; ds += STEP) {
      const a = attrsAtS(spans, s0 + ds);
      consider(s0 + ds, a ? kphToPxPerSec(a.maxspeedKph) : (fallbackMps !== undefined ? (fallbackMps as number) : Infinity));
    }
    for (const sp of spans) consider(sp.s0 as number, kphToPxPerSec(sp.maxspeedKph));
  } else if (fallbackMps !== undefined) {
    consider(s0, fallbackMps as number);
  }
  return Number.isFinite(cap) ? cap : undefined;
}

function curvatureSpeedCap(ref: FrenetReference, s0: number, horizon: number): number {
  const aBrake = A_BRAKE as number;
  const STEP = m(10) as number;
  let cap = Infinity;
  for (let ds = 0; ds <= horizon; ds += STEP) {
    const k = refCurvatureAt(ref, s0 + ds);
    if (k <= 1e-6) continue;
    const vLim = Math.sqrt((A_LAT_MAX as number) / k);
    cap = Math.min(cap, Math.sqrt(vLim * vLim + 2 * aBrake * ds));
  }
  return cap;
}

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
      _tight = INITIAL_TIGHT;
    }
    frenetRef = _routeFrenetRef!; routeActive = true;
  } else {
    if (!_loopFrenetRef || _loopRefTrack !== track) {
      _loopFrenetRef = buildFrenetReference(track.centerPoints, track.closed ?? true);
      _loopRefTrack = track as TrackGeometry;
      _prevBestTargetL = null;
      _tight = INITIAL_TIGHT;
    }
    frenetRef = _loopFrenetRef;
  }

  // ─── 2. Frenet 状态 ───
  // 名义车道来自**地图语义**（车道数 × 车道宽 → 车道中心，见 track.ts 的 laneCentersOf）：
  // OSM 3 车道 → 中间车道 l=0；径向 4 车道 → +1.875m（与历史硬编码一致）。路径代价的中心
  // 偏好、候选里的"车道中心"、AVOID 判据都用它 —— 换地图时车的巡航横向位置随之改变。
  const laneOffset = (track.laneOffset ?? LEGACY_LANE_OFFSET) as Px;
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
  // 地图限速（OSM maxspeed → vCap）：**语义来自地图**，与"前方障碍限速"取最小。
  // 位置必须在 17.5m/s 下限**之后**——地图限速是硬上限，低限速图（60km/h=16.7m/s）
  // 不应被"防呆下限"抬上去。
  // 【逐段限速 + 制动可行剖面】地图限速现在是"沿路逐段"的（`attrSpans`）：匝道 40~60km/h、
  // 主路 100~120km/h。不是简单地"取当前段的限速"，而是**提前减速**：
  //   v_allowed(s0) = min over 前方各点 s' 的 √(v_lim(s')² + 2·a_brake·(s'−s0))
  // 否则车会在限速台阶处"到了才发现要减速"——规划速度合理但车刹不住（与曲率限速同一套道理）。
  // 没有逐段属性时退化成"常量硬上限"，与旧行为逐位一致。
  const vLimit = speedLimitCap(attrSpans(track, route), track.speedLimitMps, fl.s as number, PATH_HORIZON as number);
  if (vLimit !== undefined) targetV = Math.min(targetV, vLimit);
  if (routeActive && remainingDist < DECEL_ZONE) {
    targetV *= Math.max(0.1, remainingDist / DECEL_ZONE);
  }

  // ─── 4b. 道路曲率限速 ───
  // 弯道限速 v_lim(s)=√(A_LAT_MAX/κ(s))，并检查"到该点前能否制动降到 v_lim"
  // （v² ≤ v_lim² + 2·a_brake·Δs）。否则会"进弯才发现要减速"——规划速度合理但车刹不住。
  if (CURVATURE_SPEED_LIMIT) {
    targetV = Math.min(targetV, curvatureSpeedCap(frenetRef, fl.s as number, PATH_HORIZON as number));
  }

  // ─── 5. 路径层 × 速度上限：收敛迭代（spec 2026-09-25 §5.1，EM 风格交替）───
  // 为什么迭代：路径几何（物理走廊裕度 margin(v)、发展段 S(v)）依赖"通过时的速度"，
  // 而"能通过多快"又由路径几何决定（横向机动上限 / 紧间隙慢行档 / 停车线）——单轮
  // 顺序传递丢掉这层反馈：车停下时按 v=0 取几何（标定裕度最大 1.15m）→ 窄口判死 →
  // 规划速度 0 → 下一帧依旧 v=0（自锁，批量实证 4 场景中途停车）。故把 v 当变量求解：
  // 初值 = max(当前速度, 慢行档)，几何评估速度下限取慢行档（裕度差异 ≤0.1m，换不自锁；
  // 紧间隙通过一律限速 ≤TIGHT_SPEED）。收敛阈 0.2m / 0.5m·s⁻¹；变化不降反增则提前停
  // （防极限环）。兜底 = 末轮结果（确定性）。
  const t0 = performance.now();
  const vCap = targetV;                                          // 巡航目标（含终点减速缩放）
  const vLo = TIGHT_SPEED as number;
  const v0u = (car.speed as number) * cosHD;
  let vAssume = Math.min(Math.max(v0u, vLo), Math.max(vCap, vLo));
  let pathLayer!: PathLayerResult;   // MAX_ROUNDS ≥ 1 → 循环体至少执行一次
  let uCap = Infinity;
  let prevA: number | null = null, prevU: number | null = null, prevDelta = Infinity;
  let rounds = 0;                                   // 路径层求值次数（首次必然发生）
  let revised = false;                              // 种子假设不自洽（速度上限 ≠ 假设）
  let seedBlocked = false, seedBlockS = Infinity;   // 种子档=当前速度档几何（可达性闸门用）
  for (; rounds < MAX_ROUNDS; ) {
    pathLayer = solvePathLayer({
      ref: frenetRef, l0: fl.l, dl0, s0: fl.s, v0: vAssume,
      carX: car.x as number, carY: car.y as number, lidarScan,
      carWidth: car.width, collisionRadius: car.collisionRadius,
      laneOffset, prevTargetL: _prevBestTargetL,
      sEnd: routeActive ? (frenetRef.totalS as number) : undefined,
      trackHalfWidth: snapshot.track.halfWidth,     // 规划域随地图半宽（OSM 3 车道比径向图窄）
      halfWidthAt: halfWidthAtFn(track, route),     // 逐段：匝道窄、主路宽
    });
    rounds += 1;
    const a = pathLayer.best.targetL as number;
    if (rounds === 1) {
      seedBlocked = pathLayer.best.blocked; seedBlockS = pathLayer.best.sBlock;
    }
    uCap = pathSpeedCap(pathLayer, fl.l as number, vCap);
    if (rounds === 1 && uCap < vAssume) revised = true;         // 路径几何压低种子假设（需修正）
    const converged = prevA !== null
      && Math.abs(a - prevA) < CONV_TARGET_L
      && Math.abs(uCap - (prevU as number)) < CONV_SPEED;
    const diverging = prevA !== null && Math.abs(a - prevA) > prevDelta;   // 变化不降反增 → 防极限环
    if (converged || diverging) break;
    prevDelta = prevA === null ? Infinity : Math.abs(a - prevA);
    prevA = a; prevU = uCap;
    vAssume = Math.max(vLo, Math.min(uCap, vCap));                // 下一轮速度假设
  }
  // 规划速度目标：迭代末轮的几何上限（0 = 物理封死 → 交给停车线，不把 0 当巡航目标）
  targetV = Math.min(vCap, uCap > 0 ? uCap : vLo);

  // 未观测区口径 'speed'（见-即-停）：按"可见距离内能刹停"限速——不得超出视程行驶。
  // 不裁几何、不产生停止线，因此不会像 'blocked' 那样因"规划视程 > 可见距离"而永久停车。
  if (pathLayer.unknownHandling === 'speed') {
    const cov = coverageFromScan(lidarScan as { angles: number[]; distances: number[]; rawDistances?: number[] },
      car.x as number, car.y as number);
    const limitS = corridorObservedLimitS(pathLayer.corridor, frenetRef, cov.rays, car.x as number, car.y as number, cov.halfSpacing);
    if (limitS !== null) {
      const dVis = Math.max(m(5) as number, limitS - (fl.s as number));
      targetV = Math.min(targetV, Math.sqrt(2 * (A_BRAKE as number) * dVis));
    }
  }

  // ─── 6. 速度层 ───
  // 停车线触发条件（二选一）：
  //   ① 迟滞状态机 SEALED（tight.ts）——连"更松走廊"（物理 −0.15m）都无候选：
  //      进入需确实过不去、退出需确实过得去（施密特带 + 3 帧防抖），消除临界抖动；
  //   ② 紧间隙候选不可达——慢行通过需在紧段起点前降到 TIGHT_SPEED，制动距离不够
  //      （30m/s 时需 ~70m；20m 外才发现只能减速通行的紧口 = 物理上停不住也绕不开）。
  // 只要存在"可达的物理可过"候选，就不停车（横向耐心、纵向保守、双阈值通行）。
  _tight = nextTight(_tight, {
    hasComfortClear: !pathLayer.allBlocked,
    hasHardClear: !pathLayer.allBlockedHard,
    hasDeepClear: pathLayer.deepClear,
    hasDeepBlocked: !pathLayer.permissiveClear,
  });
  let stopEngaged = _tight.state === 'SEALED';
  let stopLineS = pathLayer.sBlockHard;
  if (stopEngaged && !Number.isFinite(stopLineS)) {
    // 单帧内 SEALED ⟹ 无候选通过"更松"走廊 ⟹ 无候选通过物理走廊 ⟹ sBlockHard 有限。
    // 走到这里只可能是"记忆滞后窗口"：上帧封死已记忆，本帧已有硬可过候选，但恢复还需
    // 连续 3 帧确证（或本帧 deepClear 仍为否）。此时 sBlockHard=Infinity 会喂给停车
    // 剖面 → NaN 轨迹污染（"停车后 NaN"同类缺陷）。兜底：按舒适制动距离设虚拟停车点。
    const v0u = (car.speed as number) * cosHD;
    stopLineS = (fl.s as number) + Math.max(v0u * v0u / (2 * (A_BRAKE as number)), m(5) as number);
  }
  {
    // 可达性闸门：迭代把速度当变量后，"车能不能在到达绑定几何点之前降到速度前提"必须单独查
    // （规划出的低速绕行方案，车可能根本没时间减速过去）。
    // 判据用**车辆自身减速能力**（car.acceleration；物理层加速/减速共用同一上限，另有利阻力
    // 加成），不用规划侧保守的 A_BRAKE=6——A_BRAKE 是"舒适制动剖面"的设计值，而"能不能做到"
    // 问的是能力。定格实测（30m/s 前方 50m 障碍、绕行需降到 16m/s）：需 (30²−16²)/(2×50) =
    // 6.3m/s² ≤ 8 ✓ 可做到 → 正常绕行；同一几何 45m/s 需 17.6m/s² ✗ → 绕行/制动都做不到
    // → 必须急刹兜底。
    // 绑定点 = 三者最早：种子档（当前速度档）几何的阻塞点、末轮最优的阻塞点、
    // **全部候选里最早的舒适阻塞点**——最后一项兜住"域放宽后大角度绕行候选变舒适畅通、
    // 判据消失"的情形（45m/s 同几何绕行需峰值横向 5.77×4.6×45²/40² ≈ 33m/s² ≫ A_LAT_MAX）。
    // targetV ≥ 当前速度时 dReq ≤ 0 → 闸门自动不触发。
    let bindS = Math.min(
      seedBlocked ? seedBlockS : Infinity,
      pathLayer.best.blocked ? (pathLayer.best.sBlock as number) : Infinity,
    );
    for (const c of pathLayer.ranked) {
      if (c.blocked) bindS = Math.min(bindS, c.sBlock as number);
    }
    if (!stopEngaged && Number.isFinite(bindS)) {
      const dReq = (v0u * v0u - targetV * targetV) / (2 * (car.acceleration as number));
      if (bindS - (fl.s as number) < dReq) {
        stopEngaged = true; stopLineS = bindS;
      }
    }
  }
  const stObstacles: STObstacle[] = stopEngaged
    ? [{ sMin: stopLineS - fl.s, sMax: stopLineS - fl.s + (STOP_ASSUMED_LEN as number), tMin: 0, tMax: Infinity }]
    : [];
  const speedLayer = solveSpeedLayer({ stObstacles, v0: v0u, targetV });

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
    mapSpeedLimitMps: (track.speedLimitMps as number | undefined) ?? null,
    laneOffsetU: laneOffset as number,
    laneCentersU: track.laneCenters?.map(v => v as number),
    tight: _tight.state,
    rounds,
    revised,
    planMs: performance.now() - t0,
    vAssume,
    // 走廊可视化载荷：直接引用末轮路径层的走廊/候选对象（零拷贝）；带子/横断线在此换算成笛卡尔
    viz: buildVizPayload(pathLayer, frenetRef, lidarScan, car.x as number, car.y as number),
    remainingDist: routeActive ? remainingDist : -1,
  };

  // ─── 11. 帧间平滑 ───
  _prevBestTargetL = pathLayer.best.targetL as number;

  return createTrajectory(points, pathLayer.best.cost, type);
}

export function getPlannerState(): string {
  return _currentState;
}
