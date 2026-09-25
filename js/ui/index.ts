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
import { SIM_W, SIM_H } from '../sim/engine.js';
import { frameCamera } from '../env/renderer.js';
import { UiController } from './controller.js';
import { wireToolbar } from './toolbar.js';
import { wireSidebar, setSidebarSection } from './sidebar.js';
import { wireRunPicker, setRunPickerOpen } from './runList.js';
import { wireInteractions } from './interactions.js';
import { setupReplayPanel } from './replayPanel.js';
import { renderHint, setInfoOpen } from './hud.js';
import { setInspectorOpen, updateInspectPanel } from './inspector.js';
import { display } from '../shared/display.js';

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
} {
  const { canvas, camera, controls } = deps;

  /** Frenet 刻度按钮文案随状态同步 */
  function renderFrenetTicksButton(visible: boolean) {
    const btn = document.getElementById('btn-frenet-ticks');
    if (btn) btn.textContent = visible ? 'Frenet 刻度：开' : 'Frenet 刻度：关';
  }

  /** 走廊可视化按钮文案随状态同步 */
  function renderCorridorButton(visible: boolean) {
    const btn = document.getElementById('btn-corridor');
    if (btn) btn.textContent = visible ? '走廊可视化：开' : '走廊可视化：关';
  }

  /** 暂停按钮图标随状态同步（只改图标 span，保留按钮文案） */
  function renderPauseButton(paused: boolean) {
    const icon = document.getElementById('pause-icon');
    if (icon) icon.textContent = paused ? '▶' : '⏸';
  }

  // ── 状态机 + 副作用 ──（replayPanel 在下方定义，副作用仅在运行期触发，无 TDZ 问题）
  const controller = new UiController({
    onLivePaused: (p) => { deps.actions.setLivePaused(p); renderPauseButton(p); },
    onMode: (mode) => (mode === 'REPLAY' ? replayPanel.show() : replayPanel.hide()),
    onSidebarSection: (section) => setSidebarSection(section),
    onInfoPanel: (open) => setInfoOpen(open),
    onDebugPanel: (open) => setInspectorOpen(open),
    onRunPicker: (open) => setRunPickerOpen(open),
    onHint: (hint) => renderHint(hint),
    onCorridorVisible: (v) => renderCorridorButton(v),
    onFrenetTicksVisible: (v) => renderFrenetTicksButton(v),
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
  wireSidebar(controller);
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
  document.getElementById('btn-frenet-ticks')?.addEventListener('click', () => {
    controller.dispatch({ type: 'TOGGLE_FRENET_TICKS' });
  });

  // 点击 HUD 速度 → 切换 km/h ↔ m/s（渲染层下一帧自动按新单位显示）
  document.getElementById('hud-speed')?.addEventListener('click', () => {
    controller.dispatch({ type: 'TOGGLE_SPEED_UNIT' });
  });

  // ── 统一键盘分发器（Space 双语义 / Esc 三义）──
  function resetCamera() {
    frameCamera(camera, controls, canvas.clientWidth, canvas.clientHeight, SIM_W, SIM_H);
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
  };
}
