// ==================== 主入口：engine 装配 + 渲染 ====================
import { MessageBus } from './messageBus.js';
import { Recorder } from './shared/recorder.js';
import { debug } from './shared/debug.js';
import { SIM_W, SIM_H, FIXED_DT, SimEngine } from './sim/engine.js';
import { initScene, updateScene, resetSceneCache } from './env/renderer.js';
import { MAP_PRESETS, DEFAULT_MAP } from './env/mapDefinitions.js';
import { setupUI, updateInspectPanel } from './ui.js';
import { ReplayController } from './replay.js';

const W = SIM_W, H = SIM_H;
const canvas = document.getElementById('canvas') as HTMLCanvasElement;
canvas.width = W; canvas.height = H;

const { renderer, scene, camera, controls, lidarLines } = initScene(canvas, W, H);
const engine = new SimEngine(new MessageBus('autopilot', false), { map: MAP_PRESETS[DEFAULT_MAP] });

// 浏览器侧数据记录（headless 用 ArchiveWriter 订阅同样的主题）
const recorder = new Recorder();
recorder.start(engine.mapDef.name);
engine.bus.subscribe('planning:result', (p: any) => {
  recorder.recordFrame(p.snapshot.car, p.scan, p.trajectory, p.state, p.route);
});

document.getElementById('hud-map')!.textContent = engine.mapDef.name;

// ui.ts 右键路由状态机（SETTING_START/SETTING_GOAL）的起点暂存
let clickStartIdx: number | null = null;

const replay = new ReplayController(scene, lidarLines);

setupUI({
  W, H, canvas, camera, scene, controls, recorder,
  get mapDef() { return engine.mapDef; },
  get track() { return engine.track; },
  get roadGraph() { return engine.roadGraph; },
  get car() { return engine.car; },
  get route() { return engine.route; },
  get routeState() { return engine.routeState; },
  get latestGPS() { return engine.latestGPS; },
  onMapChange(name: string) {
    const mapDef = MAP_PRESETS[name] || MAP_PRESETS[DEFAULT_MAP];
    engine.rebuildWorld({ map: mapDef });
    resetSceneCache(scene);
    recorder.reset(); recorder.start(mapDef.name);
    document.getElementById('hud-map')!.textContent = mapDef.name;
  },
  onRouteSet(r, startIdx) { engine.setRoute(r, startIdx); },
  setRouteState(s: string) { engine.routeState = s as typeof engine.routeState; },
  setClickStartIdx(idx: number | null) { clickStartIdx = idx; },
  getClickStartIdx: () => clickStartIdx,
  getRouteState: () => engine.routeState,
  replay,
});

engine.bus.subscribe('control:state', ({ snapshot, lidarScan, trajectory, state }: any) => {
  updateScene(scene, snapshot, lidarScan, state, lidarLines, trajectory, engine.route, engine.latestGPS);
});

// ==================== 主循环（固定步长累积器） ====================
//
// 真实帧率波动 → 累积时间 → 每满 1/60s 调用一次 engine.step。
// 与 headless 直接固定步进逐帧一致（复现性要求）。
let lastTime = performance.now();
let acc = 0;
const MAX_CATCHUP = 5;

function gameLoop(now: number) {
  const realDt = Math.min(0.1, (now - lastTime) / 1000);
  lastTime = now;

  if (replay.state.active) {
    replay.update(realDt);            // 回放模式：只推进回放时钟，不步进 engine
  } else if (!debug.paused) {
    acc += realDt;
    let n = 0;
    while (acc >= FIXED_DT && n < MAX_CATCHUP) {
      engine.step(FIXED_DT);
      acc -= FIXED_DT;
      n++;
    }
    if (n === MAX_CATCHUP) acc = 0; // 防止追帧螺旋
  }

  controls.update();
  renderer.render(scene, camera);
  updateInspectPanel(realDt);
  requestAnimationFrame(gameLoop);
}
requestAnimationFrame(gameLoop);
