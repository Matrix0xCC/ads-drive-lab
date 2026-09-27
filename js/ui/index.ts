// ==================== UI 编排层 ====================
//
// 组装：UiController（状态机）+ 各面板接线。所有 UI 行为经 dispatch 驱动，
// DOM 是状态的渲染投影；main.ts 只通过 UiDeps.actions 改世界。

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { Route, TrackGeometry } from '../shared/types.js';
import { MapDefinition } from '../env/mapDefinitions.js';
import { RoadGraph } from '../algo/routePlanner.js';
import { ReplayController } from '../replay.js';
import type { RouteState } from '../sim/engine.js';
import { frameSmart } from '../env/renderer.js';
import {
  compassBearingDeg, viewPitchDeg, bearingLabel, snapView, stepViewSnap,
  pitchModeOf, toggledPitchMode, VIEW_PITCH_2D_DEG, VIEW_PITCH_3D_DEG,
} from '../env/renderer/scene.js';
import { panDelta } from './cameraKeys.js';
import { UiController } from './controller.js';
import type { RouteSource } from './state.js';
import { wireToolbar } from './toolbar.js';
import { setupMeasure } from './measure.js';
import { metersPerPixel, niceScaleBar } from '../env/renderer/scale.js';
import { wireRunPicker, setRunPickerOpen } from './runList.js';
import { wireInteractions } from './interactions.js';
import { setupReplayPanel } from './replayPanel.js';
import { renderHint, setInfoOpen } from './hud.js';
import { setInspectorOpen, updateInspectPanel } from './inspector.js';
import { display } from '../shared/display.js';
import { MAP_PRESETS } from '../env/mapDefinitions.js';

/** 用 MAP_PRESETS 填充地图下拉（在首次 applyScenario **之前**调用：
 *  applyScenario 会按 scenario.map 设 `select.value`，选项必须先存在）。
 *  这样新增地图只需在 MAP_PRESETS 登记，不必再改 index.html。 */
export function populateMapDropdown(): void {
  const sel = document.getElementById('map-dropdown') as HTMLSelectElement | null;
  if (!sel) return;
  const keep = sel.value;
  sel.innerHTML = '';
  for (const [key, def] of Object.entries(MAP_PRESETS)) {
    const opt = document.createElement('option');
    opt.value = key;
    opt.textContent = def.name;
    sel.appendChild(opt);
  }
  if (keep && [...sel.options].some(o => o.value === keep)) sel.value = keep;
}

// ═══════════════════════════════════════════════════
// 依赖接口（main.ts 提供）
// ═══════════════════════════════════════════════════

export interface UiDeps {
  canvas: HTMLCanvasElement;
  camera: THREE.PerspectiveCamera;
  scene: THREE.Scene;
  controls: OrbitControls;
  replay: ReplayController;
  /** 引擎只读视图 */
  engine: {
    readonly mapDef: MapDefinition;
    readonly track: TrackGeometry;
    readonly worldW: number;
    readonly worldH: number;
    readonly carX: number;
    readonly carY: number;
    readonly roadGraph: RoadGraph;
    readonly route: Route | null;
    readonly routeState: RouteState;
  };
  /** UI 改世界的唯一通道 */
  actions: {
    /** 加载场景 JSON；成功返回场景名，失败（已 alert）返回 null */
    loadScenario(json: unknown): string | null;
    /** 当前地图的浏览器默认随机场景 */
    loadDefaultScenario(map: string): boolean;
    /** 换地点（地图）：**清除当前路线**（节点号/弧长只对原地图有意义）。返回是否成功 */
    switchMap(name: string): boolean;
    /** 当前地图的**产物 key**（不是显示名）——下拉回滚与"随机场景"要用它 */
    currentMapKey(): string;
    /** 清除路线（车辆将保持静止） */
    clearRoute(): void;
    exportRun(): void;
    setRoute(route: Route, startIdx: number): void;
    setLivePaused(paused: boolean): void;
  };
}

// ═══════════════════════════════════════════════════
// 注册所有 UI 事件
// ═══════════════════════════════════════════════════

export function setupUI(deps: UiDeps): {
  tick(dt: number): void; isCorridorVisible(): boolean; isFrenetTicksVisible(): boolean;
  isLidarVisible(): boolean; isBasemapVisible(): boolean; isPaused(): boolean; isFollowCamEnabled(): boolean;
  /** 告诉界面"当前路线是什么、从哪来"（摘要文本由 main 生成：它知道场景/地图/路线） */
  setRouteInfo(source: RouteSource, summary: string): void;
  /** 世界变化的一次性说明（切地点清除了什么 / 场景带来了什么）；传空串即清除 */
  setNotice(text: string): void;
} {
  const { canvas, camera, controls } = deps;
  const measure = setupMeasure({ canvas, camera, scene: deps.scene, controls });

  /** 画布左缘 toolbar：开关类按钮用 active 类表示"开"（不再改文案，保持两字标签） */
  function renderToggleButton(id: string, on: boolean) {
    document.getElementById(id)?.classList.toggle('active', on);
  }
  function renderFrenetTicksButton(visible: boolean) {
    renderToggleButton('btn-frenet-ticks', visible);
  }
  function renderLidarButton(visible: boolean) {
    renderToggleButton('btn-lidar', visible);
  }
  function renderBasemapButton(visible: boolean) {
    renderToggleButton('btn-basemap', visible);
  }
  function renderCorridorButton(visible: boolean) {
    renderToggleButton('btn-corridor', visible);
  }
  function renderFollowCamButton(on: boolean) {
    renderToggleButton('btn-follow-cam', on);
  }

  /** 暂停/播放图标：LIVE 看 livePaused、REPLAY 看回放播放态（与 Space 语义一致）。
   *  只改图标 span，保留按钮文案（toolbar 里是"暂停"两字标签）。 */
  let _pauseIcon: string | null = null;
  function renderPauseButton(): void {
    const paused = controller.state.mode === 'REPLAY'
      ? !deps.replay.state.playing
      : controller.state.livePaused;
    const want = paused ? '▶' : '⏸';
    if (want === _pauseIcon) return;
    _pauseIcon = want;
    const icon = document.getElementById('pause-icon');
    if (icon) icon.textContent = want;
  }

  // ── 状态机 + 副作用 ──（replayPanel 在下方定义，副作用仅在运行期触发，无 TDZ 问题）
  const controller = new UiController({
    onLivePaused: (p) => { deps.actions.setLivePaused(p); renderPauseButton(); },
    onMode: (mode) => (mode === 'REPLAY' ? replayPanel.show() : replayPanel.hide()),
    onInfoPanel: (open) => setInfoOpen(open),
    onDebugPanel: (open) => setInspectorOpen(open),
    onRunPicker: (open) => setRunPickerOpen(open),
    onHint: (hint) => renderHint(hint),
    onCorridorVisible: (v) => { renderCorridorButton(v); deps.replay.showCorridor = v; deps.replay.requestRedraw(); },
    onFrenetTicksVisible: (v) => { renderFrenetTicksButton(v); deps.replay.showFrenetTicks = v; deps.replay.requestRedraw(); },
    onLidarVisible: (v) => { renderLidarButton(v); deps.replay.showLidar = v; deps.replay.requestRedraw(); },
    onBasemapVisible: (v) => { renderBasemapButton(v); },
    onFollowCam: (on) => { renderFollowCamButton(on); },
    onRoutePickPhase: (ph) => {
      renderToggleButton('btn-route', ph !== 'idle');
      document.getElementById('btn-route-pick')?.classList.toggle('active', ph !== 'idle');
    },
    onRouteInfo: (info) => {
      const el = document.getElementById('route-source');
      if (el) el.textContent = info.summary;      // 摘要文本由 main 提供（它知道场景/地图/路线）
    },
    onNotice: (text) => {
      const el = document.getElementById('world-notice');
      if (el) el.textContent = text;
    },
    onMeasureMode: (on) => {
      renderToggleButton('btn-measure', on);
      measure.setActive(on);
      if (!on) measure.reset();          // 退出模式即清掉上次的尺子（避免留下一条看不懂的线）
    },
    onSpeedUnit: (unit) => { display.speedUnit = unit; },
  }, () => deps.engine.routeState);

  /** 进入回放：加载数据成功后切模式（回放栏显隐由 onMode 副作用处理） */
  async function enterReplay(load: () => Promise<void> | void) {
    await load();
    controller.dispatch({ type: 'ENTER_REPLAY' });
    replayPanel.renderMarkers();
  }

  /** 退出回放：恢复实时（livePaused 从未被改动，自动恢复原暂停状态） */
  function exitReplay() {
    deps.replay.close();
    controller.dispatch({ type: 'EXIT_REPLAY' });
  }

  /** 场景/地图切换前：回放中先退出回放（避免回放渲染叠加到新世界） */
  function exitReplayIfActive() {
    if (controller.state.mode === 'REPLAY') exitReplay();
  }

  const replayPanel = setupReplayPanel(deps, enterReplay, exitReplay);
  wireToolbar(deps, controller, { enterReplay, exitReplayIfActive });
  wireRunPicker(controller, (id) => { void enterReplay(() => deps.replay.load(id)); });
  wireInteractions(deps, controller);

  // 右侧面板折叠按钮
  document.getElementById('btn-info-toggle')?.addEventListener('click', () => {
    controller.dispatch({ type: 'TOGGLE_INFO' });
  });
  document.getElementById('btn-debug-toggle')?.addEventListener('click', () => {
    controller.dispatch({ type: 'TOGGLE_DEBUG' });
  });

  // 左侧栏「仿真」：走廊可视化开关（快捷键 C）
  document.getElementById('btn-corridor')?.addEventListener('click', () => {
    controller.dispatch({ type: 'TOGGLE_CORRIDOR' });
  });
  // 相机跟随开关（toolbar / F 键）：开启时视点随车移动，关闭后视野固定
  document.getElementById('btn-follow-cam')?.addEventListener('click', () => {
    controller.dispatch({ type: 'TOGGLE_FOLLOW_CAM' });
  });
  document.getElementById('btn-frenet-ticks')?.addEventListener('click', () => {
    controller.dispatch({ type: 'TOGGLE_FRENET_TICKS' });
  });
  document.getElementById('btn-basemap')?.addEventListener('click', () => {
    controller.dispatch({ type: 'TOGGLE_BASEMAP' });
  });
  document.getElementById('btn-lidar')?.addEventListener('click', () => {
    controller.dispatch({ type: 'TOGGLE_LIDAR' });
  });
  document.getElementById('btn-route')?.addEventListener('click', () => {
    controller.dispatch({ type: 'TOGGLE_ROUTE_SET' });
  });
  // 路线行：手工设点（与 toolbar「路线」/ T 同一动作）与清除（清掉后车辆保持静止）
  document.getElementById('btn-measure')?.addEventListener('click', () => {
    controller.dispatch({ type: 'TOGGLE_MEASURE' });
  });
  document.getElementById('btn-route-pick')?.addEventListener('click', () => {
    controller.dispatch({ type: 'TOGGLE_ROUTE_SET' });
  });
  document.getElementById('btn-route-clear')?.addEventListener('click', () => {
    deps.actions.clearRoute();
  });

  // 点击 HUD 速度 → 切换 km/h ↔ m/s（渲染层下一帧自动按新单位显示）
  document.getElementById('hud-speed')?.addEventListener('click', () => {
    controller.dispatch({ type: 'TOGGLE_SPEED_UNIT' });
  });

  // ── 统一键盘分发器（Space 双语义 / Esc 三义）──
  function resetCamera() {
    // 智能取景：小世界整图、大世界取车周围局部（见 renderer/scene.ts 的 frameSmart）
    frameSmart(camera, controls, canvas.clientWidth, canvas.clientHeight,
      deps.engine.worldW, deps.engine.worldH, deps.engine.carX, deps.engine.carY);
  }
  // ── 罗盘：一次性视角命令（与"视角/R"同类），以及每帧对齐的界面状态 ──
  // 注释：方位角 0 = 正北、顺时针为正（见 renderer/scene.ts 的推导）；这里只做"命令 → 动画"。
  const BEARINGS: Record<string, number> = { N: 0, E: 90, S: 180, W: 270 };
  function snapViewToBearing(deg: number): void {
    snapView(camera, controls, { bearingDeg: deg });
  }
  function snapViewToPitch(mode: '2d' | '3d'): void {
    snapView(camera, controls, { pitchDeg: mode === '2d' ? VIEW_PITCH_2D_DEG : VIEW_PITCH_3D_DEG });
  }
  // 圆环上的 N/E/S/W：点哪个转到哪个（字母随圆环移动、文字保持正立，见 index.html 注释）
  document.querySelectorAll<HTMLElement>('.compass-dir').forEach((btn) => {
    btn.addEventListener('click', (e) => {
      e.stopPropagation();                       // 别冒泡到圆心（圆心 = 正北）
      const b = BEARINGS[btn.textContent?.trim() ?? ''];
      if (b !== undefined) snapViewToBearing(b);
    });
  });
  document.getElementById('compass-dial')?.addEventListener('click', () => snapViewToBearing(0));
  // 2D/3D 合成一个按钮：点一下切到另一边（当前是 2D → 去 3D；否则回 2D）
  document.getElementById('btn-view-pitch')?.addEventListener('click', () => {
    snapViewToPitch(toggledPitchMode(viewPitchDeg(camera, controls.target)));
  });

  /* 罗盘界面 = 相机状态的**投影**（不是 UI 状态）：每帧读相机、变化才写 DOM。
     圆环转 −bearing（让 N 指向屏幕上的北），字母各自转 +bearing 抵消父级旋转 → 文字正立。 */
  const compassEls = {
    ring: document.getElementById('compass-ring'),
    dirs: Array.from(document.querySelectorAll<HTMLElement>('.compass-dir')),
    bearing: document.getElementById('compass-bearing'),
    btnPitch: document.getElementById('btn-view-pitch'),
  };
  let lastBearing = NaN;
  let lastPitchMode = '';
  function renderCompass(): void {
    const bearing = compassBearingDeg(camera, controls.target);
    if (Math.abs(bearing - lastBearing) > 0.05) {
      lastBearing = bearing;
      if (compassEls.ring) compassEls.ring.style.transform = `rotate(${-bearing}deg)`;
      for (const d of compassEls.dirs) d.style.transform = `rotate(${bearing}deg)`;
      if (compassEls.bearing) compassEls.bearing.textContent = bearingLabel(bearing);
    }
    // 单按钮高亮：由**实际俯仰角**决定（手拖到中间角度不高亮，如实反映；阈值见 scene.ts 的 pitchModeOf）
    const mode = pitchModeOf(viewPitchDeg(camera, controls.target));
    if (mode !== lastPitchMode) {
      lastPitchMode = mode;
      compassEls.btnPitch?.classList.toggle('active', mode === '2d');
    }
  }

  /* 底部比例尺 = 相机状态的投影：随缩放更新，变化才写 DOM。
     用**横向**尺度（屏幕水平方向在目标点深度处的米数）——纵向会被俯仰的透视压缩扭曲，
     地图类比例尺一律用横向（见 env/renderer/scale.ts 文件头）。 */
  const scaleEls = {
    fill: document.getElementById('scale-bar-fill'),
    label: document.getElementById('scale-bar-label'),
  };
  let lastScaleLabel = '';
  let lastScalePx = -1;
  function renderScaleBar(): void {
    const h = canvas.clientHeight;
    if (!h) return;
    const { widthPx, label } = niceScaleBar(metersPerPixel(camera, controls.target, h));
    const px = Math.round(widthPx);
    // 数值与**条宽**都要比：数值没变但相机缩放了 → 条宽必须跟着变，否则"20 m"会画成旧宽度（比例就错了）
    if (label === lastScaleLabel && px === lastScalePx) return;
    lastScaleLabel = label; lastScalePx = px;
    if (scaleEls.fill) scaleEls.fill.style.width = `${px}px`;
    if (scaleEls.label) scaleEls.label.textContent = label;
  }

  // ── WASD 相机平移：按下集合由 window 维护，实际位移在 tick 里按 dt 应用 ──
  const heldKeys = new Set<string>();
  window.addEventListener('keydown', (e) => {
    if (e.target instanceof HTMLInputElement) return;
    heldKeys.add(e.key.toLowerCase());
  });
  window.addEventListener('keyup', (e) => heldKeys.delete(e.key.toLowerCase()));
  window.addEventListener('blur', () => heldKeys.clear());   // 失焦清空，防"卡键"一直漂移
  function moveCameraByKeys(dt: number): void {
    const f = (heldKeys.has('w') ? 1 : 0) - (heldKeys.has('s') ? 1 : 0);
    const r = (heldKeys.has('d') ? 1 : 0) - (heldKeys.has('a') ? 1 : 0);
    const d = panDelta(
      { cam: [camera.position.x, camera.position.y, camera.position.z],
        target: [controls.target.x, controls.target.y, controls.target.z] },
      f, r, dt,
    );
    if (d[0] === 0 && d[2] === 0) return;
    camera.position.x += d[0]; camera.position.z += d[2];
    controls.target.x += d[0]; controls.target.z += d[2];
  }

  window.addEventListener('keydown', (e) => {
    if (e.target instanceof HTMLInputElement) return;
    const st = controller.state;
    switch (e.key) {
      case ' ':
        e.preventDefault();
        if (st.mode === 'REPLAY') deps.replay.toggle();
        else controller.dispatch({ type: 'TOGGLE_PAUSE' });
        break;
      case 'Escape':
        if (st.runPickerOpen) controller.dispatch({ type: 'CLOSE_RUN_PICKER' });
        else if (st.mode === 'REPLAY') exitReplay();
        else if (st.measureMode) controller.dispatch({ type: 'TOGGLE_MEASURE' });
        else controller.dispatch({ type: 'CANCEL' });
        break;
      case 'r': case 'R':
        resetCamera();
        break;
      case 'm': case 'M':                       // M = Measure（量距）
        controller.dispatch({ type: 'TOGGLE_MEASURE' });
        break;
      case 'n': case 'N':                       // N = 正北（navigation 习惯）
        snapViewToBearing(0);
        break;
      case '2':
        snapViewToPitch('2d');
        break;
      case '3':
        snapViewToPitch('3d');
        break;
      case 'i': case 'I':
        controller.dispatch({ type: 'TOGGLE_DEBUG' });
        break;
      case 'c': case 'C':
        controller.dispatch({ type: 'TOGGLE_CORRIDOR' });
        break;
      case 'g': case 'G':
        controller.dispatch({ type: 'TOGGLE_FRENET_TICKS' });
        break;
      case 'l': case 'L':
        controller.dispatch({ type: 'TOGGLE_LIDAR' });
        break;
      case 'b': case 'B':                       // B = Basemap（底图）
        controller.dispatch({ type: 'TOGGLE_BASEMAP' });
        break;
      case 'f': case 'F':
        controller.dispatch({ type: 'TOGGLE_FOLLOW_CAM' });
        break;
      case 't': case 'T':
        controller.dispatch({ type: 'TOGGLE_ROUTE_SET' });
        break;
      case 'ArrowLeft':
        if (st.mode === 'REPLAY') deps.replay.seekStep(deps.replay.state.currentStep - 1);
        break;
      case 'ArrowRight':
        if (st.mode === 'REPLAY') deps.replay.seekStep(deps.replay.state.currentStep + 1);
        break;
    }
  });

  controller.refresh();   // 初始化：同步暂停按钮/回放栏/inspect/提示条

  return {
    setRouteInfo(source, summary) { controller.dispatch({ type: 'ROUTE_INFO', source, summary }); },
    setNotice(text) { controller.dispatch({ type: 'NOTICE', text }); },
    tick(dt: number) {
      if (controller.state.debugOpen) updateInspectPanel(dt);
      renderPauseButton();   // 回放播放态可能由 Space/回放栏改变 → 每帧对齐图标（变化才写 DOM）
      moveCameraByKeys(dt);  // WASD 平移相机（主循环每帧调用；controls.update 在其后应用）
      stepViewSnap(camera, controls, dt * 1000);   // 罗盘的平滑转动（dt 秒 → 毫秒）
      renderCompass();                             // 圆环旋转/方位文字/2D-3D 高亮（变化才写 DOM）
      renderScaleBar();                            // 底部比例尺（随缩放；变化才写 DOM）
      measure.update();                            // 量距读数跟随尺子中点（相机移动/缩放时也要跟）
    },
    /** 走廊可视化是否生效（main 的渲染订阅用）：开关打开且处于实时模式——
     *  回放不跑规划器，debug.planning.viz 是上一帧实时数据，画出来会是陈旧走廊。 */
    isCorridorVisible(): boolean {
      return controller.state.corridorVisible && controller.state.mode === 'LIVE';
    },
    /** Frenet 刻度显隐（回放模式也显示——刻度只依赖路线/赛道，不依赖规划器） */
    isFrenetTicksVisible(): boolean {
      return controller.state.frenetTicksVisible;
    },
    /** LiDAR 射线显隐（回放模式同样生效） */
    isBasemapVisible(): boolean {
      return controller.state.basemapVisible;
    },
    isLidarVisible(): boolean {
      return controller.state.lidarVisible;
    },
    /** 相机跟随是否开启（main 的主循环据此调用 followCar） */
    isFollowCamEnabled(): boolean {
      return controller.state.followCam;
    },
    /** 世界是否处于暂停（实时=暂停中；回放=未播放）——候选线编号只在暂停时画 */
    isPaused(): boolean {
      return controller.state.mode === 'REPLAY'
        ? !deps.replay.state.playing
        : controller.state.livePaused;
    },
  };
}
