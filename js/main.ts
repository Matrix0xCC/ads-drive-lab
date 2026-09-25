// ==================== 主入口：engine 装配 + 渲染 + UI ====================
import { MessageBus } from './messageBus.js';
import { FrameRecord } from './shared/types.js';
import { debug } from './shared/debug.js';
import { SIM_W, SIM_H, FIXED_DT, SimEngine } from './sim/engine.js';
import { Scenario, parseScenario, buildScenarioWorld, defaultScenario } from './sim/scenario.js';
import { ARCHIVE_TOPICS, recordFrom } from './sim/records.js';
import { assembleBundle } from './headless/bundle.js';
import { initScene, updateScene, resetSceneCache } from './env/renderer.js';
import { MAP_PRESETS, DEFAULT_MAP } from './env/mapDefinitions.js';
import { setupUI } from './ui/index.js';
import { ReplayController } from './replay.js';

const W = SIM_W, H = SIM_H;
const canvas = document.getElementById('canvas') as HTMLCanvasElement;
const container = canvas.parentElement as HTMLElement;

/** 画布占满容器的视口尺寸（布局未就绪时回退到世界尺寸） */
function viewportSize(): { w: number; h: number } {
  return { w: container.clientWidth || W, h: container.clientHeight || H };
}

const { renderer, scene, camera, controls, lidarLines } = initScene(
  canvas, viewportSize().w, viewportSize().h, W, H,
);

// 窗口尺寸变化：渲染分辨率与相机宽高比跟随（相机位置保持，R 键可重新取景）
window.addEventListener('resize', () => {
  const { w, h } = viewportSize();
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
});
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

function applyScenario(json: unknown): string | null {
  let scenario: Scenario;
  let built: ReturnType<typeof buildScenarioWorld>;
  try {
    scenario = parseScenario(json);
    built = buildScenarioWorld(scenario, SIM_W, SIM_H);
  } catch (e) {
    alert(`场景加载失败: ${(e as Error).message}`);
    return null;
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
  return scenario.name;
}

/** 导出当前运行归档：与 headless 同一套检测/报告（assembleBundle），下载 .run.json */
function exportRun(): void {
  if (!currentScenario) return;
  const bundle = assembleBundle({
    scenario: currentScenario, track: engine.track, obstacles: engine.obstacles,
    route: engine.route, frames, simTime: engine.simTime, planMs: engine.planMsSamples,
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

// 初始世界：浏览器默认场景
applyScenario(defaultScenario(DEFAULT_MAP));

const replay = new ReplayController(scene, lidarLines);

const ui = setupUI({
  canvas, camera, scene, controls, replay,
  engine: {
    get mapDef() { return engine.mapDef; },
    get track() { return engine.track; },
    get roadGraph() { return engine.roadGraph; },
    get route() { return engine.route; },
    get routeState() { return engine.routeState; },
  },
  actions: {
    loadScenario(json: unknown) { return applyScenario(json); },
    loadDefaultScenario(map: string) { applyScenario(defaultScenario(map)); },
    switchMap(name: string) {
      const map = MAP_PRESETS[name] ? name : DEFAULT_MAP;
      const base = currentScenario ?? defaultScenario(map);
      applyScenario({ ...base, map });
    },
    exportRun,
    setRoute(r, startIdx) { engine.setRoute(r, startIdx); },
    setLivePaused(paused: boolean) { debug.paused = paused; },
  },
});

engine.bus.subscribe('control:state', ({ snapshot, lidarScan, trajectory, state }: any) => {
  updateScene(scene, snapshot, lidarScan, state, lidarLines, trajectory, engine.route, engine.latestGPS,
    ui.isCorridorVisible() ? (debug.planning?.viz ?? null) : null, ui.isFrenetTicksVisible());
});

// ==================== 主循环（固定步长累积器） ====================
//
// 真实帧率波动 → 累积时间 → 每满 1/60s 调用一次 engine.step。
// 与 headless 直接固定步进逐帧一致（复现性要求）。
// 回放模式由 replay.state.active 分支接管，不步进 engine。
let lastTime = performance.now();
let acc = 0;
const MAX_CATCHUP = 5;

function gameLoop(now: number) {
  const realDt = Math.min(0.1, (now - lastTime) / 1000);
  lastTime = now;

  if (replay.state.active) {
    replay.update(realDt);
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
  ui.tick(realDt);
  requestAnimationFrame(gameLoop);
}
requestAnimationFrame(gameLoop);
