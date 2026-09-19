// ==================== UI 编排层 ====================
//
// setupUI 组装各面板接线（工具栏/交互/回放栏/键盘）。
// 状态逻辑见 state.ts/controller.ts（T5 接线后本层只做组装）。

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { Route, GPSReading, TrackGeometry, CarState } from '../shared/types.js';
import { MapDefinition } from '../env/mapDefinitions.js';
import { RoadGraph } from '../algo/routePlanner.js';
import { ReplayController } from '../replay.js';
import { debug } from '../shared/debug.js';
import { wireToolbar } from './toolbar.js';
import { wireInteractions } from './interactions.js';
import { setupReplayPanel } from './replayPanel.js';

// ═══════════════════════════════════════════════════
// 上下文接口
// ═══════════════════════════════════════════════════

export interface UIContext {
  W: number; H: number;
  canvas: HTMLCanvasElement;
  camera: THREE.PerspectiveCamera;
  scene: THREE.Scene;
  controls: OrbitControls;
  /** 导出当前运行归档（main.ts 组装 RunBundle 并下载） */
  onExportRun: () => void;
  // 可变状态（UI 只读或通过回调修改）
  mapDef: MapDefinition;
  track: TrackGeometry;
  roadGraph: RoadGraph;
  car: CarState;
  route: Route | null;
  routeState: string;
  latestGPS: GPSReading | null;
  // 回调
  onMapChange: (name: string) => void;
  onRouteSet: (route: Route, startIdx: number) => void;
  setRouteState: (s: string) => void;
  setClickStartIdx: (idx: number | null) => void;
  getClickStartIdx: () => number | null;
  getRouteState: () => string;
  /** 回放控制器（main.ts 创建并接入渲染循环） */
  replay: ReplayController;
  /** 场景加载（与 headless 同源）：ui 层负责 fetch 场景文件，main 负责解析应用 */
  onScenarioLoad: (json: unknown) => void;
  /** 恢复浏览器默认随机场景（当前地图） */
  onScenarioReset: () => void;
}

export { updateInspectPanel } from './inspector.js';

// ═══════════════════════════════════════════════════
// 注册所有 UI 事件
// ═══════════════════════════════════════════════════

export function setupUI(ctx: UIContext): void {
  const { W, H, camera, controls } = ctx;

  // ── 视角重置（按钮在 toolbar 接线，键盘 R 在此） ──
  function resetCamera() {
    camera.position.set(W / 2, 600, H / 2 + 400);
    controls.target.set(W / 2, 0, H / 2);
    controls.update();
  }

  // ── 暂停（按钮在 toolbar 接线，键盘 Space 在此） ──
  function togglePause() {
    debug.paused = !debug.paused;
    const btn = document.getElementById('btn-pause');
    if (btn) btn.textContent = debug.paused ? '▶' : '⏸';
  }

  const replayPanel = setupReplayPanel(ctx, enterReplay);

  /** 进入回放：暂停实时仿真 → 加载数据（服务器运行或导入归档）→ 显示回放栏 */
  async function enterReplay(load: () => Promise<void> | void) {
    debug.paused = true;
    await load();
    replayPanel.show();
    replayPanel.renderMarkers();
  }

  wireToolbar(ctx, { enterReplay, resetCamera, togglePause });
  wireInteractions(ctx);

  window.addEventListener('keydown', (e) => {
    if (e.target instanceof HTMLInputElement) return;
    if (e.key === ' ') { e.preventDefault(); togglePause(); }
    if (e.key === 'r') resetCamera();
  });
}
