// ==================== 主入口：engine 装配 + 渲染 + UI ====================
import { MessageBus } from './messageBus.js';
import { FrameRecord } from './shared/types.js';
import { debug } from './shared/debug.js';
import { SIM_W, SIM_H, FIXED_DT, SimEngine } from './sim/engine.js';
import { Scenario, parseScenario, buildScenarioWorld, defaultScenario } from './sim/scenario.js';
import { ARCHIVE_TOPICS, recordFrom } from './sim/records.js';
import { assembleBundle } from './headless/bundle.js';
import { initScene, updateScene, resetSceneCache, frameSmart, followCar, setWorldExtent, applyBasemap, getBasemapAttribution } from './env/renderer.js';
import { MAP_PRESETS, DEFAULT_MAP } from './env/mapDefinitions.js';
import { SIM_UNITS_PER_METER } from './shared/dimensions.js';
import type { RouteSource } from './ui/state.js';
import { setupUI, populateMapDropdown } from './ui/index.js';
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

/** 底图署名（许可义务）：只在底图打开且确实有署名时显示。
 *  放在 applyScenario **之前**（虽然函数声明会提升，但放前面能避免"变量在下面声明"的 TDZ 陷阱）。 */
function renderBasemapAttribution(): void {
  const el = document.getElementById('hud-basemap-attribution');
  if (!el) return;
  const attr = getBasemapAttribution();
  const show = attr !== '' && ui.isBasemapVisible();
  el.style.display = show ? 'block' : 'none';
  if (show) el.textContent = attr;
}

/** 一次世界变化的"对外说明"：路线来源与摘要 + 给用户看的一行提示。
 *  由运行期的 actions 转交给界面（**applyScenario 自己不能碰 `ui`**——它在模块初始化阶段就会被调用，
 *  而 `const ui` 那时还没初始化，会踩 TDZ，见 js/test/ui/main-init-order.test.ts）。 */
export interface WorldAnnounce {
  routeInfo: { source: RouteSource; summary: string };
  notice: string;
}

/** 路线摘要（给人读的一行）：声明里的"节点/弧长" + 实际长度 */
function routeSummary(scenario: Scenario, lenPx: number | null): { source: RouteSource; summary: string } {
  const r = scenario.route;
  if (!r || lenPx === null || !Number.isFinite(lenPx)) return { source: 'none', summary: '无路线（车辆静止）' };
  const km = (lenPx / SIM_UNITS_PER_METER / 1000).toFixed(2);
  const what = r.fromNode !== undefined ? `节点 ${r.fromNode}→${r.toNode}` : `S=${r.startS}→${r.goalS}m`;
  return { source: 'scenario', summary: `场景自带 · ${what} · ${km}km` };
}

function applyScenario(json: unknown, out?: WorldAnnounce): string | null {
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
  // 世界尺寸随地图（OSM 长条图 900×1840m ≫ 默认 450×200m）：地面/网格重建 + 智能取景
  setWorldExtent(scene, engine.worldW, engine.worldH);
  // 底图（真实地图瓦片）：**异步、可失败**——没有瓦片缓存时静默降级为无底图，不影响仿真。
  // ① 用 `scenario.map`（**产物 key**）而不是 `engine.mapDef.name`（对合成地图是显示名，如 `Oval`/`Tech Circuit`）；
  // ② 径向图是**合成**地图（没有经纬度）→ 找瓦片没有意义，直接跳过（否则控制台一条 404 噪音）。
  // 署名（许可义务）由 renderer 的底图句柄提供，每帧读取 → 异步加载完成后自动出现在 HUD。
  if (engine.mapDef.kind !== 'radial') void applyBasemap(scene, scenario.map);
  frameSmart(camera, controls, canvas.clientWidth || W, canvas.clientHeight || H,
    engine.worldW, engine.worldH, engine.car.x as number, engine.car.y as number);
  frames.length = 0;   // 归档从本次场景加载起重新收集
  document.getElementById('hud-map')!.textContent = engine.mapDef.name;
  document.getElementById('hud-scenario')!.textContent = `${scenario.name}（种子 ${scenario.seed}）`;
  if (out) {
    out.routeInfo = routeSummary(scenario, built.route ? (built.route.length as unknown as number) : null);
    out.notice = `已加载场景「${scenario.name}」：地点 → ${engine.mapDef.name}`
      + (built.route ? `、路线 → ${out.routeInfo.summary.replace('场景自带 · ', '')}` : '（无路线：车辆静止）');
  }
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

// 初始世界：浏览器默认场景（先填充地图下拉：applyScenario 会按 scenario.map 设 value）
populateMapDropdown();
applyScenario(defaultScenario(DEFAULT_MAP));

const replay = new ReplayController(scene, lidarLines);

const ui = setupUI({
  canvas, camera, scene, controls, replay,
  engine: {
    get mapDef() { return engine.mapDef; },
    get track() { return engine.track; },
    get worldW() { return engine.worldW; },
    get worldH() { return engine.worldH; },
    get carX() { return latestCar.x; },
    get carY() { return latestCar.y; },
    get roadGraph() { return engine.roadGraph; },
    get route() { return engine.route; },
    get routeState() { return engine.routeState; },
  },
  actions: {
    loadScenario(json: unknown) {
      const out = {} as WorldAnnounce;
      const name = applyScenario(json, out);
      if (!name) return null;
      ui.setRouteInfo(out.routeInfo.source, out.routeInfo.summary);
      ui.setNotice(out.notice);
      return name;
    },
    loadDefaultScenario(map: string): boolean {
      const out = {} as WorldAnnounce;
      const name = applyScenario(defaultScenario(map), out);
      if (!name) return false;
      ui.setRouteInfo(out.routeInfo.source, out.routeInfo.summary);
      ui.setNotice(out.notice);
      return true;
    },
    /** 当前地图的**产物 key**（下拉回滚、"随机场景"都用它；显示名对合成地图是 Oval 之类，不能当 key） */
    currentMapKey(): string { return currentScenario?.map ?? DEFAULT_MAP; },
    /** 换地点：**丢弃当前路线**——路线用节点号/弧长表达，只对原地图有意义（硬套会报错或落错位置） */
    switchMap(name: string): boolean {
      const map = MAP_PRESETS[name] ? name : DEFAULT_MAP;
      const base = currentScenario ?? defaultScenario(map);
      const hadRoute = base.route !== undefined;
      const { route: _dropped, ...rest } = base;          // 关键：不把旧路线的声明带过去
      const out = {} as WorldAnnounce;
      if (!applyScenario({ ...rest, map }, out)) return false;
      out.routeInfo = { source: 'none', summary: '无路线（车辆静止）' };
      out.notice = `已切换到「${engine.mapDef.name}」`
        + (hadRoute ? '；原路线属于上一张地图，已清除 —— 要跑就点「手工设点」或选一个带路线的场景' : '');
      ui.setRouteInfo(out.routeInfo.source, out.routeInfo.summary);
      ui.setNotice(out.notice);
      return true;
    },
    /** 清除路线：车辆随之保持静止（与 headless 的"无 route 不动"一致） */
    clearRoute() {
      engine.clearRoute();
      ui.setRouteInfo('none', '无路线（车辆静止）');
      ui.setNotice('已清除路线：车辆将保持静止（要恢复就重新加载场景，或点「手工设点」）');
    },
    exportRun,
    setRoute(r, startIdx) {
      engine.setRoute(r, startIdx);
      // 手工点的路线（T / toolbar「路线」/ 侧栏「手工设点」）——来源标记为"手工"
      const km = ((r.length as unknown as number) / SIM_UNITS_PER_METER / 1000).toFixed(2);
      ui.setRouteInfo('manual', `手工设点 · ${km}km`);
      ui.setNotice('');
    },
    setLivePaused(paused: boolean) { debug.paused = paused; },
  },
});

engine.bus.subscribe('control:state', ({ snapshot, lidarScan, trajectory, state }: any) => {
  latestCar = { x: snapshot.car.x as number, y: snapshot.car.y as number };
  renderBasemapAttribution();
  updateScene(scene, snapshot, lidarScan, state, lidarLines, trajectory, engine.route, engine.latestGPS,
    ui.isCorridorVisible() ? (debug.planning?.viz ?? null) : null, ui.isFrenetTicksVisible(),
    ui.isLidarVisible(), ui.isPaused(), ui.isBasemapVisible());
});

// ==================== 主循环（固定步长累积器） ====================
//
// 真实帧率波动 → 累积时间 → 每满 1/60s 调用一次 engine.step。
// 与 headless 直接固定步进逐帧一致（复现性要求）。
// 回放模式由 replay.state.active 分支接管，不步进 engine。
/** 最新车位（相机跟随用；跟随默认开，用户仍可用 WASD/鼠标调整相对视角） */
let latestCar = { x: 0, y: 0 };

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

  // 相机跟随：由 toolbar/F 键的开关控制（换地图时按世界尺寸给默认值，见 SCENARIO_CHANGED）
  if (ui.isFollowCamEnabled()) {
    followCar(camera, controls, latestCar.x, latestCar.y, realDt);
  }
  controls.update();
  renderer.render(scene, camera);
  ui.tick(realDt);
  requestAnimationFrame(gameLoop);
}
requestAnimationFrame(gameLoop);
