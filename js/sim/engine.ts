// ==================== 仿真核心引擎（浏览器 / headless 共用） ====================
//
// 【职责】持有世界状态与全部管线订阅者（感知20Hz / 规划15Hz / GPS10Hz /
// 控制物理每 tick），浏览器与 headless 使用同一份装配——保证 headless
// 测的就是浏览器实际运行的管线。
//
// 【固定步长】对外只提供 step(dt)，浏览器用真实时间累积器按 1/60s 调用，
// headless 直接循环调用；两者逐帧数值一致。
//
// 【消息主题】engine 发布（订阅方按需消费）：
//   tick             每步        {dt, t, step}
//   perception:result 20Hz       {snapshot, scan, stats}
//   planning:result   15Hz       {snapshot, scan, trajectory, state, route, planning}
//   gps:result        10Hz       {reading, truth}
//   control:state     每步        {snapshot, lidarScan, trajectory, state, control}
//   sim:event         离散        {kind, step, t, payload}
//     kind: state_change / emergency_brake / bounds_reset / arrived

import { MessageBus } from '../messageBus.js';
import {
  LidarScan, Trajectory, Route, GPSReading, TrackGeometry, CarState, Obstacle,
  createWorldSnapshot,
} from '../shared/types.js';
import { Radians, Px, PxPerSec, rad, m, mps, mps2, pxps } from '../shared/dimensions.js';
import { debug } from '../shared/debug.js';
import { buildTrack } from '../env/track.js';
import { generateObstacles } from '../env/obstacle.js';
import { createCar, stepPhysics, checkBounds } from '../env/car.js';
import { MapDefinition } from '../env/mapDefinitions.js';
import { localize, LocalizationConfig } from '../env/localization.js';
import { sense, MAX_SENSOR_DIST, SCAN_RAY_COUNT } from '../algo/perception.js';
import { plan, getPlannerState } from '../algo/plan/index.js';
import { control } from '../algo/control.js';
import { buildRoadGraph, buildGraphFromEdges, RoadGraph } from '../algo/routePlanner.js';
import { Rng, createRng, deriveSeed } from '../shared/rng.js';

export const SIM_W = 1800, SIM_H = 800;
export const FIXED_DT = 1 / 60;
export const PERCEPTION_HZ = 20, PLANNING_HZ = 15, GPS_HZ = 10;
const DEFAULT_GPS: Partial<LocalizationConfig> = { noiseStdDev: 1.25 };

export interface WorldConfig {
  map: MapDefinition;
  /** 显式障碍物布局（优先于随机） */
  obstacles?: Obstacle[];
  /** 随机模式数量；缺省 3 + rng()*3 */
  obstacleCount?: number;
  /** 随机根种子；缺省 undefined = Math.random（浏览器） */
  seed?: number;
  carStart?: { x: Px; y: Px; angle?: Radians; speed?: PxPerSec };
  gpsConfig?: Partial<LocalizationConfig>;
}

export type RouteState = 'NONE' | 'ACTIVE' | 'ARRIVED';

function emptyScan(): LidarScan {
  const d = new Array(SCAN_RAY_COUNT).fill(MAX_SENSOR_DIST) as Px[];
  const a = new Array(SCAN_RAY_COUNT).fill(rad(0)) as Radians[];
  return { angles: a, distances: d, rawDistances: d, confidences: [], timestamp: 0 };
}

export class SimEngine {
  readonly bus: MessageBus;
  mapDef: MapDefinition;
  track!: TrackGeometry;
  roadGraph!: RoadGraph;
  car!: CarState;
  obstacles: Obstacle[] = [];
  route: Route | null = null;
  routeState: RouteState = 'NONE';
  routeStartIdx: number | null = null;
  latestLidarScan: LidarScan;
  latestTrajectory: Trajectory;
  latestGPS: GPSReading | null = null;
  /** 世界宽（px）：按地图定义；OSM 长条图远大于默认 SIM_W */
  get worldW(): number { return this.mapDef.worldW ?? SIM_W; }
  /** 世界高（px） */
  get worldH(): number { return this.mapDef.worldH ?? SIM_H; }
  /** 单帧规划耗时样本（ms；只记录，不参与决策；summary/报告用） */
  planMsSamples: number[] = [];
  simTime = 0;
  stepCount = 0;

  private _cfg: WorldConfig;
  private _perceptionTimer = 0;
  private _planningTimer = 0;
  private _gpsTimer = 0;
  private _prevPlanState: string | null = null;
  private _braking = false;
  private _gpsRng: Rng = Math.random;

  constructor(bus: MessageBus, cfg: WorldConfig) {
    this.bus = bus;
    this._cfg = cfg;
    this.mapDef = cfg.map;
    this.latestLidarScan = emptyScan();
    this.latestTrajectory = { points: [], cost: 0, type: 'lane_keep', timestamp: 0 };
    this.rebuildWorld(cfg);
    this._wireSubscribers();
  }

  /** 重建世界（初始 / 地图切换）：赛道、路网、障碍物、车辆、路线全部重置 */
  rebuildWorld(cfg: WorldConfig): void {
    this._cfg = cfg;
    this.mapDef = cfg.map;
    this.track = buildTrack(cfg.map, SIM_W, SIM_H);
    this.roadGraph = this.track.chains && this.track.netNodes && this.track.netEdges
      ? buildGraphFromEdges(this.track.netNodes.length, this.track.netEdges)
      : buildRoadGraph(this.track.centerPoints, this.track.closed ?? true);

    const seed = cfg.seed;
    const obsRng: Rng = seed === undefined ? Math.random : createRng(deriveSeed(seed, 'obstacles'));
    this._gpsRng = seed === undefined ? Math.random : createRng(deriveSeed(seed, 'gps'));
    const count = cfg.obstacleCount ?? (3 + Math.floor(obsRng() * 3));
    this.obstacles = cfg.obstacles ?? generateObstacles(this.track, count, 8, 14, obsRng);

    this.car = createCar(this.track.centerPoints[0], this.track.centerPoints[1]);
    if (cfg.carStart) {
      this.car.x = cfg.carStart.x;
      this.car.y = cfg.carStart.y;
      if (cfg.carStart.angle !== undefined) this.car.angle = cfg.carStart.angle;
      if (cfg.carStart.speed !== undefined) this.car.speed = cfg.carStart.speed;
    }
    this.route = null;
    this.routeState = 'NONE';
    this.routeStartIdx = null;
    this.latestGPS = null;
    this.simTime = 0;
    this.stepCount = 0;
    this._perceptionTimer = this._planningTimer = this._gpsTimer = 0;
    this._prevPlanState = null;
    this._braking = false;
  }

  /** 与浏览器右键设路线的语义一致：车放到路线起点、朝向第二点 */
  setRoute(route: Route, startIdx: number, opts?: { speed?: PxPerSec }): void {
    this.route = route;
    this.routeStartIdx = startIdx;
    this.routeState = 'ACTIVE';
    this.car.x = route.waypoints[0].x;
    this.car.y = route.waypoints[0].y;
    const np = route.waypoints[Math.min(1, route.waypoints.length - 1)];
    this.car.angle = rad(Math.atan2(
      (np.y as number) - (this.car.y as number),
      (np.x as number) - (this.car.x as number),
    ));
    this.car.speed = opts?.speed ?? mps(7.5);
  }

  /** 推进一个固定步长：发布 tick，全部订阅者按各自频率节流执行 */
  step(dt: number = FIXED_DT): void {
    this.simTime += dt;
    this.stepCount += 1;
    this.bus.publish('tick', { dt, t: this.simTime, step: this.stepCount });
  }

  private _emit(kind: string, payload: Record<string, unknown>): void {
    this.bus.publish('sim:event', { kind, step: this.stepCount, t: this.simTime, payload });
  }

  private _wireSubscribers(): void {
    // ── 感知 20Hz ──
    this.bus.subscribe('tick', ({ dt }: { dt: number }) => {
      this._perceptionTimer += dt;
      if (this._perceptionTimer >= 1 / PERCEPTION_HZ) {
        const snapshot = createWorldSnapshot(this.car, this.track, this.obstacles);
        this.latestLidarScan = sense(snapshot);
        this.bus.publish('perception:result', {
          snapshot, scan: this.latestLidarScan,
          stats: debug.perception ? { ...debug.perception } : null,
          t: this.simTime, step: this.stepCount,
        });
        this._perceptionTimer = 0;
      }
    });

    // ── 规划 15Hz ──
    this.bus.subscribe('tick', ({ dt }: { dt: number }) => {
      this._planningTimer += dt;
      if (this._planningTimer >= 1 / PLANNING_HZ) {
        const snapshot = createWorldSnapshot(this.car, this.track, this.obstacles);
        this.latestTrajectory = plan(snapshot, this.latestLidarScan, this.route ?? undefined);
        if (debug.planning?.planMs !== undefined) this.planMsSamples.push(debug.planning.planMs);
        const state = getPlannerState();
        if (this._prevPlanState !== null && state !== this._prevPlanState) {
          this._emit('state_change', { from: this._prevPlanState, to: state });
        }
        this._prevPlanState = state;
        if (this.latestTrajectory.type === 'arrived' && this.routeState === 'ACTIVE') {
          this.routeState = 'ARRIVED';
          this._emit('arrived', {});
        }
        this.bus.publish('planning:result', {
          snapshot, scan: this.latestLidarScan, trajectory: this.latestTrajectory,
          state, route: this.route, planning: debug.planning ? { ...debug.planning } : null,
          t: this.simTime, step: this.stepCount,
        });
        this._planningTimer = 0;
      }
    });

    // ── GPS 10Hz ──
    this.bus.subscribe('tick', ({ dt }: { dt: number }) => {
      this._gpsTimer += dt;
      if (this._gpsTimer >= 1 / GPS_HZ) {
        const truth = { x: this.car.x, y: this.car.y };
        this.latestGPS = localize(this.car, this._cfg.gpsConfig ?? DEFAULT_GPS, this._gpsRng);
        this.bus.publish('gps:result', {
          reading: this.latestGPS, truth, t: this.simTime, step: this.stepCount,
        });
        this._gpsTimer = 0;
      }
    });

    // ── 控制 + 物理（每 tick） ──
    this.bus.subscribe('tick', ({ dt }: { dt: number }) => {
      if (this.routeState === 'ACTIVE') {
        const mid = Math.floor(this.latestLidarScan.distances.length / 2);
        const frontDist = this.latestLidarScan.distances[mid] as number;
        // 紧急制动：LiDAR 前方 < 5m 且速度 > 3m/s 时减 5m/s²，最低 2m/s
        const braking = frontDist < (m(5) as number) && (this.car.speed as number) > (mps(3) as number);
        if (braking !== this._braking) {
          this._braking = braking;
          if (braking) this._emit('emergency_brake', { frontDist });
        }
        if (braking) {
          this.car.speed = pxps(Math.max(
            mps(2) as number,
            (this.car.speed as number) - (mps2(5) as number) * dt,
          ));
        }
        stepPhysics(this.car, control(this.latestTrajectory, this.car), dt);
        if (checkBounds(this.car, this.track)) this._emit('bounds_reset', {});
      } else {
        this.car.speed = pxps(0);
      }
      const snapshot = createWorldSnapshot(this.car, this.track, this.obstacles);
      this.bus.publish('control:state', {
        snapshot, lidarScan: this.latestLidarScan, trajectory: this.latestTrajectory,
        state: getPlannerState(), control: debug.control ? { ...debug.control } : null,
        t: this.simTime, step: this.stepCount,
      });
    });
  }
}
