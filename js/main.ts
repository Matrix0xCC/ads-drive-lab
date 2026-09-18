// ==================== 主入口：管线编排 ====================
import * as THREE from 'three';
import { MessageBus } from './messageBus.js';
import { createWorldSnapshot, LidarScan, Trajectory, Route, GPSReading, TrackGeometry, CarState } from './shared/types.js';
import { Recorder } from './shared/recorder.js';
import { debug } from './shared/debug.js';
import { Radians, Px, PxPerSec, rad, m, mps, mps2, px, pxps } from './shared/dimensions.js';

// env/
import { generateTrack } from './env/track.js';
import { generateObstacles } from './env/obstacle.js';
import { createCar, stepPhysics, checkBounds } from './env/car.js';
import { initScene, updateScene, resetSceneCache } from './env/renderer.js';
import { MAP_PRESETS, DEFAULT_MAP, MapDefinition } from './env/mapDefinitions.js';
import { localize, LocalizationConfig } from './env/localization.js';

// algo/
import { sense, MAX_SENSOR_DIST, SCAN_RAY_COUNT } from './algo/perception.js';
import { plan, getPlannerState } from './algo/plan/index.js';
import { control } from './algo/control.js';
import { buildRoadGraph, RoadGraph } from './algo/routePlanner.js';

// ui/
import { setupUI, updateInspectPanel, UIContext } from './ui.js';

// ==================== 状态 ====================

const W = 1800, H = 800;

let mapDef: MapDefinition = MAP_PRESETS[DEFAULT_MAP];
let track: TrackGeometry;
let roadGraph: RoadGraph;
let car: CarState = createCar({ x: px(0), y: px(0) }, { x: px(1), y: px(0) });
let obstacles: ReturnType<typeof generateObstacles> = [];

let routeState: 'NONE' | 'SETTING_START' | 'SETTING_GOAL' | 'ACTIVE' | 'ARRIVED' = 'NONE';
let route: Route | null = null;
let clickStartIdx: number | null = null;
let latestGPS: GPSReading | null = null;

// ==================== 初始化 ====================

const canvas = document.getElementById('canvas') as HTMLCanvasElement;
canvas.width = W; canvas.height = H;

const { renderer, scene, camera, controls, lidarLines } = initScene(canvas, W, H);
const bus = new MessageBus('autopilot', false);
const recorder = new Recorder();
recorder.start(mapDef.name);

// ==================== 地图 ====================

function buildWorld() {
  track = generateTrack(W, H, mapDef.controlRadii);
  roadGraph = buildRoadGraph(track.centerPoints);
  obstacles = generateObstacles(track, 3 + Math.floor(Math.random() * 3));
  route = null; routeState = 'NONE'; clickStartIdx = null;
  resetSceneCache(scene);
  recorder.reset(); recorder.start(mapDef.name);
  car = createCar(track.centerPoints[0], track.centerPoints[1]);
  document.getElementById('hud-map')!.textContent = mapDef.name;
}

buildWorld();

// ==================== UI ====================

setupUI({
  W, H, canvas, camera, scene, controls, recorder,
  get mapDef() { return mapDef; },
  get track() { return track; },
  get roadGraph() { return roadGraph; },
  get car() { return car; },
  get route() { return route; },
  get routeState() { return routeState; },
  get latestGPS() { return latestGPS; },
  onMapChange(name: string) { mapDef = MAP_PRESETS[name] || MAP_PRESETS[DEFAULT_MAP]; buildWorld(); },
  onRouteSet(r: Route, startIdx: number) {
    route = r; clickStartIdx = startIdx; routeState = 'ACTIVE';
    car.x = r.waypoints[0].x; car.y = r.waypoints[0].y;
    const np = r.waypoints[Math.min(1, r.waypoints.length - 1)];
    car.angle = rad(Math.atan2((np.y as number) - (car.y as number), (np.x as number) - (car.x as number)));
    car.speed = mps(7.5);  // 7.5 m/s = 27 km/h 起步
  },
  setRouteState(s: string) { routeState = s as typeof routeState; },
  setClickStartIdx(idx: number | null) { clickStartIdx = idx; },
  getClickStartIdx: () => clickStartIdx,
  getRouteState: () => routeState,
});

// ==================== 频率配置 ====================

const PERCEPTION_HZ = 20, PLANNING_HZ = 15, GPS_HZ = 10;
const gpsConfig: LocalizationConfig = { noiseStdDev: 1.25 };  // 1.25m GPS 噪声

// ==================== 缓存 ====================

const emptyDistances: Px[] = new Array(SCAN_RAY_COUNT).fill(MAX_SENSOR_DIST);
const emptyAngles: Radians[] = new Array(SCAN_RAY_COUNT).fill(rad(0));
let latestLidarScan: LidarScan = { angles: emptyAngles, distances: emptyDistances, rawDistances: emptyDistances, confidences: [], timestamp: 0 };
let latestTrajectory: Trajectory = { points: [], cost: 0, type: 'lane_keep', timestamp: 0 };
let perceptionTimer = 0, planningTimer = 0, gpsTimer = 0;

// ==================== 流水线 ====================

bus.subscribe('tick', ({ dt }: { dt: number }) => {
  perceptionTimer += dt;
  if (perceptionTimer >= 1 / PERCEPTION_HZ) { latestLidarScan = sense(createWorldSnapshot(car, track, obstacles)); perceptionTimer = 0; }
});

bus.subscribe('tick', ({ dt }: { dt: number }) => {
  planningTimer += dt;
  if (planningTimer >= 1 / PLANNING_HZ) {
    latestTrajectory = plan(createWorldSnapshot(car, track, obstacles), latestLidarScan, route ?? undefined);
    recorder.recordFrame(car, latestLidarScan, latestTrajectory, getPlannerState(), route);
    if (latestTrajectory.type === 'arrived' && routeState === 'ACTIVE') routeState = 'ARRIVED';
    planningTimer = 0;
  }
});

bus.subscribe('tick', ({ dt }: { dt: number }) => {
  gpsTimer += dt;
  if (gpsTimer >= 1 / GPS_HZ) { latestGPS = localize(car, gpsConfig); gpsTimer = 0; }
});

bus.subscribe('tick', ({ dt }: { dt: number }) => {
  if (routeState === 'ACTIVE') {
    const mid = Math.floor(latestLidarScan.distances.length / 2);
    const frontDist = latestLidarScan.distances[mid] as number;
    // 紧急制动：LiDAR 前方 < 5m 且速度 > 3m/s 时减速 5m/s²，最低 2m/s
    if (frontDist < (m(5) as number) && (car.speed as number) > (mps(3) as number))
      car.speed = pxps(Math.max(mps(2) as number, (car.speed as number) - (mps2(5) as number) * dt));
    stepPhysics(car, control(latestTrajectory, car), dt);
    checkBounds(car, track);
  } else {
    car.speed = pxps(0);
  }
  bus.publish('control:state', { snapshot: createWorldSnapshot(car, track, obstacles), lidarScan: latestLidarScan, trajectory: latestTrajectory, state: getPlannerState() });
});

bus.subscribe('control:state', ({ snapshot, lidarScan, trajectory, state }: any) => {
  updateScene(scene, snapshot, lidarScan, state, lidarLines, trajectory, route, latestGPS);
});

// ==================== 主循环 ====================

let lastTime = performance.now();
function gameLoop(now: number) {
  const rawDt = (now - lastTime) / 1000;
  const dt = rawDt > 0.1 ? 0.016 : rawDt;
  if (!debug.paused) bus.publish('tick', { dt });
  lastTime = now;
  controls.update();
  renderer.render(scene, camera);
  updateInspectPanel(dt);
  requestAnimationFrame(gameLoop);
}
requestAnimationFrame(gameLoop);
