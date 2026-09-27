// ==================== 场景加载：JSON → 世界配置 ====================
//
// 【用途】headless 模式的输入：场景 JSON（地图 + 障碍物 + 路线 + 种子），
// 校验并转换为 SimEngine 的 WorldConfig。
//
// 【单位】场景文件中所有长度/速度一律物理单位（米 / m/s），
// 在本模块边界统一转换为仿真单位（m()/mps() 工厂是唯一转换点）。
//
// 【坐标约定】障碍物用赛道 Frenet 坐标 (s, l) 声明（l>0 内侧），
// 由 frenetToCartesian 换算到世界坐标；yaw 为相对赛道切线的偏角。
// 路线用弧长 s 声明起终点，内部转中心线索引后复用 Dijkstra。
//
// 【注意】无 route 时车辆保持静止（与浏览器语义一致），
// 行驶类场景必须带 route。

import { AttrSpan, Obstacle, Point2D, Route, TrackGeometry, createRoute } from '../shared/types.js';
import { Px, rad, m, mps, px, SIM_UNITS_PER_METER } from '../shared/dimensions.js';
import { MAP_PRESETS, MapDefinition } from '../env/mapDefinitions.js';
import { buildTrack, TRACK_HALF_WIDTH } from '../env/track.js';
import { generateObstacles } from '../env/obstacle.js';
import { buildGraphFromEdges, buildNetworkRoute, buildRoadGraph, findRoute } from '../algo/routePlanner.js';
import { buildFrenetReference, frenetToCartesian } from '../algo/plan/frenet.js';
import { Rng, createRng, deriveSeed } from '../shared/rng.js';
import { WorldConfig } from './engine.js';

export class ScenarioError extends Error {}

export interface ScenarioObstacle { s: number; l: number; w: number; len: number; yaw?: number; }

export interface Scenario {
  version: 1;
  name: string;
  map: string;
  seed: number;
  duration: number;
  car?: { speed?: number };
  obstacles: { mode: 'explicit'; list: ScenarioObstacle[] } | { mode: 'random'; count: number };
  /** 路线：链式/径向地图用弧长 `startS/goalS`；**路网地图用节点** `fromNode/toNode` */
  route?: { startS?: number; goalS?: number; fromNode?: number; toNode?: number };
  checks?: Record<string, number>;
}

// ═══════════════════════════════════════════════════
// 解析与校验
// ═══════════════════════════════════════════════════

function fail(msg: string): never { throw new ScenarioError(msg); }
function isNum(v: unknown): v is number { return typeof v === 'number' && Number.isFinite(v); }

/** 校验并规整场景 JSON；任何非法输入抛 ScenarioError（含字段名） */
export function parseScenario(json: unknown): Scenario {
  if (typeof json !== 'object' || json === null) fail('场景必须是 JSON 对象');
  const o = json as Record<string, any>;

  if (o.version !== 1) fail(`version 必须为 1（实际 ${o.version}）`);
  if (typeof o.name !== 'string' || o.name.length === 0) fail('name 必须是非空字符串');
  if (typeof o.map !== 'string' || !(o.map in MAP_PRESETS)) fail(`未知地图: ${o.map}`);
  if (!isNum(o.seed) || !Number.isInteger(o.seed) || o.seed < 0) fail('seed 必须是非负整数');
  if (!isNum(o.duration) || o.duration <= 0 || o.duration > 3600) fail('duration 必须在 (0, 3600] 秒');

  if (o.car !== undefined) {
    if (typeof o.car !== 'object' || o.car === null) fail('car 必须是对象');
    if (o.car.speed !== undefined && (!isNum(o.car.speed) || o.car.speed <= 0)) {
      fail('car.speed 必须是正数（m/s）');
    }
  }

  // 校验用的横向上限要按**该地图**的半宽（parseScenario 阶段还没有 track 对象）
  const maxL = ((MAP_PRESETS[o.map].halfWidth ?? TRACK_HALF_WIDTH) as number) / SIM_UNITS_PER_METER;
  const obs = o.obstacles;
  if (typeof obs !== 'object' || obs === null) fail('缺少 obstacles 配置');
  if (obs.mode === 'explicit') {
    if (!Array.isArray(obs.list) || obs.list.length === 0) fail('obstacles.list 必须是非空数组');
    obs.list.forEach((it: any, i: number) => {
      if (!isNum(it?.s) || it.s < 0 || it.s >= 3600) fail(`obstacles.list[${i}].s 必须在 [0, 3600) 米`);
      if (!isNum(it?.l) || Math.abs(it.l) >= maxL) fail(`obstacles.list[${i}].l 必须在 (-${maxL}, ${maxL}) 米`);
      if (!isNum(it?.w) || it.w <= 0 || it.w > 20) fail(`obstacles.list[${i}].w 必须在 (0, 20] 米`);
      if (!isNum(it?.len) || it.len <= 0 || it.len > 20) fail(`obstacles.list[${i}].len 必须在 (0, 20] 米`);
      if (it.yaw !== undefined && !isNum(it.yaw)) fail(`obstacles.list[${i}].yaw 必须是数字（弧度）`);
    });
  } else if (obs.mode === 'random') {
    if (!Number.isInteger(obs.count) || obs.count < 0 || obs.count > 50) {
      fail('obstacles.count 必须是 0..50 的整数');
    }
  } else {
    fail(`obstacles.mode 必须是 explicit 或 random（实际 ${obs.mode}）`);
  }

  if (o.route !== undefined) {
    if (typeof o.route !== 'object' || o.route === null) fail('route 必须是对象');
    const byS = isNum(o.route.startS) && isNum(o.route.goalS);
    const byNode = Number.isInteger(o.route.fromNode) && Number.isInteger(o.route.toNode);
    if (!byS && !byNode) fail('route 需要 (startS, goalS)（链式地图）或 (fromNode, toNode)（路网地图）');
    if (byS && (o.route.startS < 0 || o.route.goalS < 0)) fail('route.startS/goalS 必须是非负数（米）');
    if (o.route.fromNode !== undefined && byNode && (o.route.fromNode < 0 || o.route.toNode < 0)) {
      fail('route.fromNode/toNode 必须是非负整数（路网节点序号）');
    }
  }

  if (o.checks !== undefined) {
    if (typeof o.checks !== 'object' || o.checks === null) fail('checks 必须是对象');
    for (const [k, v] of Object.entries(o.checks)) {
      if (!isNum(v)) fail(`checks.${k} 必须是数字`);
    }
  }

  return o as Scenario;
}

// ═══════════════════════════════════════════════════
// 浏览器默认场景（与 headless 同源：同一套 Scenario → engine 管线）
// ═══════════════════════════════════════════════════

/**
 * 构建浏览器默认场景：地图 + 随机种子 + 随机 3-5 障碍、无路线
 *（路线由用户右键设置；带路线的场景文件加载后会自动开跑）
 */
export function defaultScenario(map: string, rng: Rng = Math.random): Scenario {
  return {
    version: 1,
    name: `browser-${map}`,
    map,
    seed: Math.floor(rng() * 1000000),
    duration: 3600,        // 浏览器不按时长终止
    obstacles: { mode: 'random', count: 3 + Math.floor(rng() * 3) },
  };
}

// ═══════════════════════════════════════════════════
// 世界构建
// ═══════════════════════════════════════════════════

/** 累计弧长 → 最近中心线索引（闭环绕接；开环不绕接） */
function sToIndex(centerPoints: Point2D[], s: number, closed = true): number {
  let acc = 0, best = 0, bestErr = Infinity;
  const n = closed ? centerPoints.length - 1 : centerPoints.length;
  for (let i = 0; i < n; i++) {
    const err = Math.abs(acc - s);
    if (err < bestErr) { bestErr = err; best = i; }
    const j = closed ? (i + 1) % n : Math.min(i + 1, n - 1);
    acc += Math.hypot(
      (centerPoints[j].x as number) - (centerPoints[i].x as number),
      (centerPoints[j].y as number) - (centerPoints[i].y as number),
    );
  }
  return best;
}

/**
 * 场景 → 世界配置
 *
 * @returns worldCfg 传给 SimEngine；track/route 供归档与检测器使用
 * @throws ScenarioError 路线不可达等构建期错误
 */
export function buildScenarioWorld(scenario: Scenario, width: number, height: number): {
  worldCfg: WorldConfig; track: TrackGeometry; route: Route | null;
} {
  const mapDef: MapDefinition = MAP_PRESETS[scenario.map];
  const track = buildTrack(mapDef, width, height);
  const closed = track.closed ?? true;
  const ref = buildFrenetReference(track.centerPoints, closed);

  // 路线起点索引先算（随机障碍的出生区净空需要）
  const isNetwork = !!(track.chains && track.netNodes && track.netEdges);
  // ── 路网地图：路线必须先算（障碍要沿**路线**布置，而不是沿某条链）──
  let netWaypoints: Point2D[] | null = null;
  let netStartNode = -1;
  let lastChainSeq: number[] = [];
  let netAttrSpans: AttrSpan[] | undefined;
  if (isNetwork && scenario.route) {
    if (scenario.route.fromNode === undefined || scenario.route.toNode === undefined) {
      throw new ScenarioError('路网地图的 route 需要 fromNode/toNode');
    }
    // 【合法性】节点序号是**地图专属**的：切换地图时旧场景的 fromNode/toNode 会落空。
    // 不先检查会在建图内部炸出 `Invalid array length`（不可读；实测踩过）。
    const nodeCount = track.netNodes!.length;
    if (scenario.route.fromNode >= nodeCount || scenario.route.toNode >= nodeCount) {
      throw new ScenarioError(
        `路网节点序号超出范围: fromNode=${scenario.route.fromNode} toNode=${scenario.route.toNode}`
        + `（本图共 ${nodeCount} 个节点，合法 0..${nodeCount - 1}）—— 路线声明可能来自别的地图`);
    }
    const graph = buildGraphFromEdges(track.netNodes!.length, track.netEdges!, track.chains!);
    const r = buildNetworkRoute(graph, track.netEdges!, track.chains!, scenario.route.fromNode, scenario.route.toNode,
      track.chainAttrs);
    if (!r) throw new ScenarioError(`路网路线不可达: node ${scenario.route.fromNode} → ${scenario.route.toNode}`);
    netWaypoints = r.waypoints;
    netStartNode = scenario.route.fromNode;
    lastChainSeq = r.chainSeq;
    netAttrSpans = r.attrSpans;
  }
  const routeStartIdx = !isNetwork && scenario.route
    ? sToIndex(track.centerPoints, m(scenario.route.startS ?? 0) as number, closed)
    : null;

  // 【显式障碍的参考线】路网地图上"道路"就是路线：必须沿**路线**的 s 放置，
  // 用 track.centerPoints（= 最长链）会把障碍放到另一条路上去。
  const obsRef = netWaypoints ? buildFrenetReference(netWaypoints, false) : ref;
  let obstacles: Obstacle[];
  if (scenario.obstacles.mode === 'explicit') {
    obstacles = scenario.obstacles.list.map(o => {
      const cart = frenetToCartesian(m(o.s) as number, m(o.l), obsRef);
      return {
        x: cart.x, y: cart.y,
        width: m(o.w), height: m(o.len),
        angle: rad((cart.yaw as number) + (o.yaw ?? 0)),
        zHeight: 8,
      };
    });
  } else {
    const rng = createRng(deriveSeed(scenario.seed, 'obstacles'));
    // 路网：障碍沿**路线**布置（出生区净空 = 路线起点后 80m）；链式：沿赛道，按起点索引净空
    const obsTrack = netWaypoints
      ? { centerPoints: netWaypoints, innerBoundary: [], outerBoundary: [], closed: false, halfWidth: track.halfWidth }
      : track;
    obstacles = generateObstacles(obsTrack, scenario.obstacles.count, 8, 14, rng,
      netWaypoints ? { fromIndex: 0, minGap: m(80) as number }
        : routeStartIdx !== null ? { fromIndex: routeStartIdx, minGap: m(80) as number } : undefined);
  }

  let route: Route | null = null;
  let carStart: WorldConfig['carStart'] = undefined;
  if (netWaypoints) {
    // 路网：waypoints 已由 Dijkstra 装配；waypointIndices 记途经**链序号**（归档用）
    const nodes = netWaypoints;
    let totalLen = 0;
    for (let i = 1; i < nodes.length; i++) {
      totalLen += Math.hypot(
        (nodes[i].x as number) - (nodes[i - 1].x as number),
        (nodes[i].y as number) - (nodes[i - 1].y as number),
      );
    }
    const np = nodes[Math.min(1, nodes.length - 1)];
    route = createRoute(nodes, lastChainSeq, netStartNode, scenario.route!.toNode ?? -1, px(totalLen), netAttrSpans);
    carStart = {
      x: nodes[0].x, y: nodes[0].y,
      angle: rad(Math.atan2(
        (np.y as number) - (nodes[0].y as number),
        (np.x as number) - (nodes[0].x as number),
      )),
      speed: scenario.car?.speed !== undefined ? mps(scenario.car.speed) : mps(5),
    };
  } else if (scenario.route && routeStartIdx !== null) {
    const graph = buildRoadGraph(track.centerPoints, closed);
    const startIdx = routeStartIdx;
    const goalIdx = sToIndex(track.centerPoints, m(scenario.route.goalS ?? 0) as number, closed);
    const pathIndices = findRoute(graph, startIdx, goalIdx);
    if (!pathIndices || pathIndices.length < 2) {
      throw new ScenarioError(`路线不可达: startS=${scenario.route.startS} goalS=${scenario.route.goalS}`);
    }
    const waypoints = pathIndices.map(i => track.centerPoints[i]);
    // 【合法性】弧长同样是**地图专属**的：切换地图后旧场景的 startS/goalS 可能落到本图之外，
    // 此时 centerPoints[i] 是 undefined，路线会带着坏点继续（长度算成 NaN）。
    if (waypoints.length < 2 || waypoints.some(p => !p
      || !Number.isFinite(p.x as number) || !Number.isFinite(p.y as number))) {
      throw new ScenarioError(
        `路线不可达: startS=${scenario.route.startS} goalS=${scenario.route.goalS}`
        + `（本图中心线共 ${track.centerPoints.length} 点）—— 路线声明可能来自别的地图`);
    }
    let totalLen = 0;
    for (let i = 1; i < waypoints.length; i++) {
      totalLen += Math.hypot(
        (waypoints[i].x as number) - (waypoints[i - 1].x as number),
        (waypoints[i].y as number) - (waypoints[i - 1].y as number),
      );
    }
    // 逐段属性：路线是中心线的一段 → 把赛道 s 轴的区间切到 [起点,终点] 并整体平移
    let routeSpans: AttrSpan[] | undefined;
    if (track.attrSpans && track.attrSpans.length > 0) {
      let sStart = 0;
      for (let i = 1; i <= startIdx; i++) {
        sStart += Math.hypot(
          (track.centerPoints[i].x as number) - (track.centerPoints[i - 1].x as number),
          (track.centerPoints[i].y as number) - (track.centerPoints[i - 1].y as number),
        );
      }
      const sGoal = sStart + (totalLen as unknown as number);
      routeSpans = track.attrSpans
        .filter(sp => (sp.s1 as number) > sStart && (sp.s0 as number) < sGoal)
        .map(sp => ({
          ...sp,
          s0: px(Math.max(0, (sp.s0 as number) - sStart)),
          s1: px(Math.min(sGoal - sStart, (sp.s1 as number) - sStart)),
        }))
        .filter(sp => (sp.s1 as number) > (sp.s0 as number));
    }
    route = createRoute(waypoints, pathIndices, startIdx, goalIdx, px(totalLen), routeSpans);
    // 车放路线起点，朝向第二点（与 engine.setRoute 一致的语义）
    const np = waypoints[Math.min(1, waypoints.length - 1)];
    carStart = {
      x: waypoints[0].x, y: waypoints[0].y,
      angle: rad(Math.atan2(
        (np.y as number) - (waypoints[0].y as number),
        (np.x as number) - (waypoints[0].x as number),
      )),
      // 默认起步速度 5 m/s（18 km/h）：障碍按随机散布，可能落在出生点紧邻处；
      // 25m/s 起步时制动距离 ~52m，出生点前十几米的障碍物理上不可避
      // （oval-s3 实测：障碍在出生点前 13.8m → 任何规划器都撞）。可用 car.speed 覆盖。
      speed: scenario.car?.speed !== undefined ? mps(scenario.car.speed) : mps(5),
    };
  }

  // 【兜底】长度必须是有限值：任何未来的 NaN 都要变成可读错误，而不是静默的坏路线
  if (route && !Number.isFinite(route.length as unknown as number)) {
    throw new ScenarioError('路线数据非法（长度为 NaN）');
  }
  return {
    worldCfg: { map: mapDef, obstacles, seed: scenario.seed, carStart },
    track, route,
  };
}
