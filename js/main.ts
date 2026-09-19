// ==================== 主入口：engine 装配 + 渲染 ====================
import { MessageBus } from './messageBus.js';
import { FrameRecord } from './shared/types.js';
import { debug } from './shared/debug.js';
import { SIM_W, SIM_H, FIXED_DT, SimEngine } from './sim/engine.js';
import { Scenario, parseScenario, buildScenarioWorld, defaultScenario } from './sim/scenario.js';
import { ARCHIVE_TOPICS, recordFrom } from './sim/records.js';
import { assembleBundle } from './headless/bundle.js';
import { initScene, updateScene, resetSceneCache } from './env/renderer.js';
import { MAP_PRESETS, DEFAULT_MAP } from './env/mapDefinitions.js';
import { setupUI, updateInspectPanel } from './ui/index.js';
import { ReplayController } from './replay.js';

const W = SIM_W, H = SIM_H;
const canvas = document.getElementById('canvas') as HTMLCanvasElement;
canvas.width = W; canvas.height = H;

const { renderer, scene, camera, controls, lidarLines } = initScene(canvas, W, H);
const engine = new SimEngine(new MessageBus('autopilot', false), { map: MAP_PRESETS[DEFAULT_MAP] });

// 运行归档收集（与 headless 同源：同一主题清单、同一记录映射）
const frames: FrameRecord[] = [];
for (const topic of ARCHIVE_TOPICS) {
  engine.bus.subscribe(topic, (p: any) => frames.push(recordFrom(topic, p)));
}

// ==================== 场景（与 headless 完全同源） ====================
//
// 两条模式共用同一管线：Scenario → parseScenario → buildScenarioWorld → engine。
// 浏览器默认场景 = 随机障碍、无路线（右键设路线）；加载 scenarios/ 文件后
// 带路线的场景自动开跑，且同场景同种子与 headless 逐帧一致。

let currentScenario: Scenario | null = null;

function applyScenario(json: unknown): boolean {
  let scenario: Scenario;
  let built: ReturnType<typeof buildScenarioWorld>;
  try {
    scenario = parseScenario(json);
    built = buildScenarioWorld(scenario, SIM_W, SIM_H);
  } catch (e) {
    alert(`场景加载失败: ${(e as Error).message}`);
    return false;
  }
  engine.rebuildWorld(built.worldCfg);
  if (built.route) {
    engine.setRoute(built.route, built.route.startIndex, { speed: built.worldCfg.carStart?.speed });
  }
  currentScenario = scenario;
  resetSceneCache(scene);
  frames.length = 0;   // 归档从本次场景加载起重新收集
  document.getElementById('hud-map')!.textContent = engine.mapDef.name;
  document.getElementById('hud-scenario')!.textContent = `${scenario.name}（种子 ${scenario.seed}）`;
  const mapSelect = document.getElementById('map-dropdown') as HTMLSelectElement | null;
  if (mapSelect) mapSelect.value = scenario.map;
  return true;
}

// 初始世界：浏览器默认场景
applyScenario(defaultScenario(DEFAULT_MAP));

/** 导出当前运行归档：与 headless 同一套检测/报告（assembleBundle），下载 .run.json */
function exportRun(): void {
  if (!currentScenario) return;
  const bundle = assembleBundle({
    scenario: currentScenario, track: engine.track, obstacles: engine.obstacles,
    route: engine.route, frames, simTime: engine.simTime,
    id: `${new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19)}_${currentScenario.name}_browser`,
    createdAt: new Date().toISOString(),
    gitCommit: null, nodeVersion: 'browser',
  });
  const blob = new Blob([JSON.stringify(bundle)], { type: 'application/json' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = `${bundle.meta.id}.run.json`;
  a.click();
  URL.revokeObjectURL(a.href);
}

// ui.ts 右键路由状态机（SETTING_START/SETTING_GOAL）的起点暂存
let clickStartIdx: number | null = null;

const replay = new ReplayController(scene, lidarLines);

setupUI({
  W, H, canvas, camera, scene, controls,
  onExportRun: exportRun,
  get mapDef() { return engine.mapDef; },
  get track() { return engine.track; },
  get roadGraph() { return engine.roadGraph; },
  get car() { return engine.car; },
  get route() { return engine.route; },
  get routeState() { return engine.routeState; },
  get latestGPS() { return engine.latestGPS; },
  onMapChange(name: string) {
    const map = MAP_PRESETS[name] ? name : DEFAULT_MAP;
    const base = currentScenario ?? defaultScenario(map);
    applyScenario({ ...base, map });
  },
  onRouteSet(r, startIdx) { engine.setRoute(r, startIdx); },
  setRouteState(s: string) { engine.routeState = s as typeof engine.routeState; },
  setClickStartIdx(idx: number | null) { clickStartIdx = idx; },
  getClickStartIdx: () => clickStartIdx,
  getRouteState: () => engine.routeState,
  replay,
  onScenarioLoad(json: unknown) { applyScenario(json); },
  onScenarioReset() { applyScenario(defaultScenario(engine.mapDef.name)); },
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
