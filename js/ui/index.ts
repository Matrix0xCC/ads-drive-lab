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
import { UiController } from './controller.js';
import { wireToolbar } from './toolbar.js';
import { wireInteractions } from './interactions.js';
import { setupReplayPanel } from './replayPanel.js';
import { renderHint } from './hud.js';
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

export function setupUI(deps: UiDeps): { tick(dt: number): void } {
  const { canvas, camera, controls } = deps;

  /** 暂停按钮图标随状态同步 */
  function renderPauseButton(paused: boolean) {
    const btn = document.getElementById('btn-pause');
    if (btn) btn.textContent = paused ? '▶' : '⏸';
  }

  // ── 状态机 + 副作用 ──（replayPanel 在下方定义，副作用仅在运行期触发，无 TDZ 问题）
  const controller = new UiController({
    onLivePaused: (p) => { deps.actions.setLivePaused(p); renderPauseButton(p); },
    onMode: (mode) => (mode === 'REPLAY' ? replayPanel.show() : replayPanel.hide()),
    onInspector: (open) => setInspectorOpen(open),
    onHint: (hint) => renderHint(hint),
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
  wireInteractions(deps, controller);

  // inspect 折叠按钮
  document.getElementById('btn-inspect-toggle')?.addEventListener('click', () => {
    controller.dispatch({ type: 'TOGGLE_INSPECTOR' });
  });

  // 点击 HUD 速度 → 切换 km/h ↔ m/s（渲染层下一帧自动按新单位显示）
  document.getElementById('hud-speed')?.addEventListener('click', () => {
    controller.dispatch({ type: 'TOGGLE_SPEED_UNIT' });
  });

  // ── 统一键盘分发器（Space 双语义 / Esc 三义）──
  function resetCamera() {
    camera.position.set(canvas.width / 2, 600, canvas.height / 2 + 400);
    controls.target.set(canvas.width / 2, 0, canvas.height / 2);
    controls.update();
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
        if (st.mode === 'REPLAY') exitReplay();
        else controller.dispatch({ type: 'CANCEL' });
        break;
      case 'r': case 'R':
        resetCamera();
        break;
      case 'i': case 'I':
        controller.dispatch({ type: 'TOGGLE_INSPECTOR' });
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
      if (controller.state.inspectorOpen) updateInspectPanel(dt);
    },
  };
}
