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
import { panDelta } from './cameraKeys.js';
import { UiController } from './controller.js';
import { wireToolbar } from './toolbar.js';
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
    loadDefaultScenario(map: string): void;
    /** 保留当前场景配置，仅换地图 */
    switchMap(name: string): void;
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
  isLidarVisible(): boolean; isPaused(): boolean; isFollowCamEnabled(): boolean;
} {
  const { canvas, camera, controls } = deps;

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
    onFollowCam: (on) => { renderFollowCamButton(on); },
    onRoutePickPhase: (ph) => renderToggleButton('btn-route', ph !== 'idle'),
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
  document.getElementById('btn-lidar')?.addEventListener('click', () => {
    controller.dispatch({ type: 'TOGGLE_LIDAR' });
  });
  document.getElementById('btn-route')?.addEventListener('click', () => {
    controller.dispatch({ type: 'TOGGLE_ROUTE_SET' });
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
        else controller.dispatch({ type: 'CANCEL' });
        break;
      case 'r': case 'R':
        resetCamera();
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
    tick(dt: number) {
      if (controller.state.debugOpen) updateInspectPanel(dt);
      renderPauseButton();   // 回放播放态可能由 Space/回放栏改变 → 每帧对齐图标（变化才写 DOM）
      moveCameraByKeys(dt);  // WASD 平移相机（主循环每帧调用；controls.update 在其后应用）
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
