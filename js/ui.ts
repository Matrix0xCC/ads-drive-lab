// ==================== UI 交互层 ====================
//
// 所有 DOM 事件处理和 UI 逻辑的集中管理。
// 通过 `setupUI(ctx)` 注册，与 main.ts 的管线逻辑解耦。

import * as THREE from 'three';
import { MapDefinition, MAP_PRESETS, DEFAULT_MAP } from './env/mapDefinitions.js';
import { debug } from './shared/debug.js';
import { Route, GPSReading, createRoute, TrackGeometry } from './shared/types.js';
import { parseBundle } from './headless/bundle.js';
import { rad, px, pxps, SIM_UNITS_PER_METER } from './shared/dimensions.js';
import { buildRoadGraph, findRoute, findClosestPoint, RoadGraph } from './algo/routePlanner.js';
import { CarState } from './shared/types.js';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { ReplayController } from './replay.js';

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
  /** 场景加载（与 headless 同源）：ui.ts 负责 fetch 场景文件，main 负责解析应用 */
  onScenarioLoad: (json: unknown) => void;
  /** 恢复浏览器默认随机场景（当前地图） */
  onScenarioReset: () => void;
}

// ═══════════════════════════════════════════════════
// Inspect 面板
// ═══════════════════════════════════════════════════

let _inspectTimer = 0;

export function updateInspectPanel(dt: number) {
  _inspectTimer += dt;
  if (_inspectTimer < 0.1) return;
  _inspectTimer = 0;

  const panel = document.getElementById('inspect-panel');
  if (!panel) return;

  const toM = (v: number, d = 1) => (v / SIM_UNITS_PER_METER).toFixed(d);

  const p = debug.perception;
  const pl = debug.planning;
  const c = debug.control;

  let html = '<b>Perception</b>\n';
  if (p) {
    html += `  射线: ${p.rayCount} | 前方: ${toM(p.frontDist)}m\n`;
    html += `  最近命中: ${toM(p.minDist)}m\n`;
    html += `  过滤: ${p.rawHits}→${p.filteredHits} (背景减除)\n`;
  }
  html += '\n<b>Planning</b>\n';
  if (pl) {
    html += `  状态: ${pl.state} | S=${toM(pl.frenetS)}m L=${toM(pl.frenetL, 2)}m\n`;
    html += `  dl/ds=${pl.dl_ds.toFixed(4)} | Δheading=${(pl.headingDiff*180/Math.PI).toFixed(1)}°\n`;
    if (pl.remainingDist > 0) html += `  剩余: ${toM(pl.remainingDist)}m | 速度比: ${pl.speedScale.toFixed(2)}\n`;
    html += `  最优: L=${toM(pl.bestTargetL, 1)}m V=${toM(pl.bestTargetV, 1)}m/s cost=${pl.bestCost.toFixed(1)}\n`;
    html += '  候选:\n';
    for (const cnd of pl.candidates) {
      const mark = cnd.targetL === pl.bestTargetL && cnd.targetV === pl.bestTargetV ? '★' : ' ';
      html += `    ${mark} L=${toM(cnd.targetL, 1).padStart(5)}m V=${toM(cnd.targetV, 1).padStart(5)}m/s cost=${cnd.cost.toFixed(1)}${cnd.collision?' COL':''}\n`;
    }
  }
  html += '\n<b>Control</b>\n';
  if (c) {
    html += `  预瞄: ${toM(c.lookaheadDist)}m → (${toM(c.lookaheadX)},${toM(c.lookaheadY)})\n`;
    html += `  α=${(c.alpha*180/Math.PI).toFixed(1)}° | δ=${(c.steerAngle*180/Math.PI).toFixed(2)}° | 油门=${c.throttle.toFixed(3)}\n`;
  }
  panel.innerHTML = `<pre style="margin:0">${html}</pre>`;
}

// ═══════════════════════════════════════════════════
// 注册所有 UI 事件
// ═══════════════════════════════════════════════════

export function setupUI(ctx: UIContext) {
  const { W, H, canvas, camera, controls } = ctx;

  // ── 地图选择 ──
  const mapSelect = document.getElementById('map-dropdown') as HTMLSelectElement;
  if (mapSelect) {
    mapSelect.value = DEFAULT_MAP;
    mapSelect.addEventListener('change', () => {
      ctx.onMapChange(mapSelect.value);
    });
  }

  // ── 场景加载（与 headless 同源：scenarios/index.json 列表） ──
  const scenarioSelect = document.getElementById('scenario-dropdown') as HTMLSelectElement | null;
  if (scenarioSelect) {
    fetch('scenarios/index.json')
      .then(r => (r.ok ? r.json() : { scenarios: [] }))
      .then((idx: any) => {
        for (const entry of idx.scenarios ?? []) {
          const opt = document.createElement('option');
          opt.value = entry.file;
          opt.textContent = `${entry.name}（${entry.map}）`;
          scenarioSelect.appendChild(opt);
        }
      })
      .catch(() => { /* 索引缺失不阻塞页面 */ });
    scenarioSelect.addEventListener('change', async () => {
      const file = scenarioSelect.value;
      if (!file) { ctx.onScenarioReset(); return; }
      try {
        const res = await fetch(`scenarios/${file}`);
        if (!res.ok) throw new Error(`HTTP ${res.status}`);
        ctx.onScenarioLoad(await res.json());
      } catch (e) {
        alert(`场景加载失败: ${(e as Error).message}`);
      }
    });
  }

  // ── 运行归档导出/导入（与 headless 同格式：.run.json 归档包） ──
  document.getElementById('btn-export')?.addEventListener('click', () => {
    ctx.onExportRun();
  });

  document.getElementById('btn-import')?.addEventListener('click', () => {
    const input = document.createElement('input');
    input.type = 'file'; input.accept = '.json';
    input.onchange = (e: any) => {
      const file = e.target?.files?.[0];
      if (!file) return;
      const reader = new FileReader();
      reader.onload = () => {
        try {
          const bundle = parseBundle(JSON.parse(reader.result as string));
          void enterReplay(() => replay.loadBundle(bundle));
        } catch (err) {
          alert(`归档导入失败: ${(err as Error).message}`);
        }
      };
      reader.readAsText(file);
    };
    input.click();
  });

  // ── 暂停 ──
  function togglePause() {
    debug.paused = !debug.paused;
    const btn = document.getElementById('btn-pause');
    if (btn) btn.textContent = debug.paused ? '▶' : '⏸';
  }
  document.getElementById('btn-pause')?.addEventListener('click', togglePause);
  window.addEventListener('keydown', (e) => {
    if (e.key === ' ' && !(e.target instanceof HTMLInputElement)) { e.preventDefault(); togglePause(); }
  });

  // ── 视角重置 ──
  function resetCamera() {
    camera.position.set(W / 2, 600, H / 2 + 400);
    controls.target.set(W / 2, 0, H / 2);
    controls.update();
  }
  document.getElementById('btn-reset-view')?.addEventListener('click', resetCamera);
  window.addEventListener('keydown', (e) => {
    if (e.key === 'r' && !(e.target instanceof HTMLInputElement)) resetCamera();
  });

  // ── 左键点击 → 物体检查 ──
  canvas.addEventListener('click', (e: MouseEvent) => {
    const rect = canvas.getBoundingClientRect();
    const mouse = new THREE.Vector2(
      ((e.clientX - rect.left) / rect.width) * 2 - 1,
      -((e.clientY - rect.top) / rect.height) * 2 + 1,
    );
    const raycaster = new THREE.Raycaster();
    raycaster.setFromCamera(mouse, camera);

    const hits = raycaster.intersectObjects(ctx.scene.children, true);
    const objHit = hits.find(h => h.object.userData?.type);
    if (objHit) {
      const d = objHit.object.userData;
      const tip = document.getElementById('object-tooltip');
      if (tip) {
        const w = typeof d.width === 'number' ? (d.width as number).toFixed(1) : d.width;
        const h = typeof d.height === 'number' ? (d.height as number).toFixed(1) : d.height;
        const l = typeof d.length === 'number' ? (d.length as number).toFixed(1) : d.length;
        tip.innerHTML = `${d.label}<br>宽 ${w}m × 高 ${h}m × 长 ${l}m`;
        tip.style.display = 'block';
        tip.style.left = (e.clientX + 15) + 'px';
        tip.style.top = (e.clientY - 30) + 'px';
        setTimeout(() => { if (tip) tip.style.display = 'none'; }, 3000);
      }
    }
  });

  // ── 右键点击 → 路线设置 ──
  canvas.addEventListener('contextmenu', (e: MouseEvent) => {
    e.preventDefault();
    const rect = canvas.getBoundingClientRect();
    const mouse = new THREE.Vector2(
      ((e.clientX - rect.left) / rect.width) * 2 - 1,
      -((e.clientY - rect.top) / rect.height) * 2 + 1,
    );
    const raycaster = new THREE.Raycaster();
    raycaster.setFromCamera(mouse, camera);
    const target = new THREE.Vector3();
    raycaster.ray.intersectPlane(new THREE.Plane(new THREE.Vector3(0, 1, 0), 0), target);
    if (!target) return;

    const idx = findClosestPoint(target.x, target.z, ctx.track.centerPoints);
    const rs = ctx.getRouteState();

    if (rs === 'NONE' || rs === 'SETTING_START' || rs === 'ARRIVED') {
      ctx.setClickStartIdx(idx);
      ctx.setRouteState('SETTING_GOAL');
    } else if (rs === 'SETTING_GOAL') {
      const startIdx = ctx.getClickStartIdx()!;
      const pathIndices = findRoute(ctx.roadGraph, startIdx, idx);
      if (pathIndices && pathIndices.length >= 2) {
        const waypoints = pathIndices.map(i => ctx.track.centerPoints[i]);
        let totalLen = 0;
        for (let i = 1; i < waypoints.length; i++) {
          totalLen += Math.hypot(
            (waypoints[i].x as number) - (waypoints[i-1].x as number),
            (waypoints[i].y as number) - (waypoints[i-1].y as number),
          );
        }
        const r = createRoute(waypoints, pathIndices, startIdx, idx, px(totalLen));
        ctx.onRouteSet(r, startIdx);
      }
    }
  });

  // ── 回放 ──
  const replayBar = document.getElementById('replay-bar') as HTMLDivElement | null;
  const timeline = document.getElementById('replay-timeline') as HTMLInputElement | null;
  const stepLabel = document.getElementById('replay-step-label');
  const markersBox = document.getElementById('replay-markers');
  const replay = ctx.replay;

  function renderMarkers() {
    if (!markersBox) return;
    markersBox.innerHTML = '';
    const total = replay.state.totalSteps || 1;
    for (const step of replay.state.markers) {
      const dot = document.createElement('div');
      dot.className = 'replay-marker';
      dot.style.left = `${(step / total) * 100}%`;
      dot.title = `问题帧 ${step}`;
      dot.addEventListener('click', () => replay.seekStep(step));
      markersBox.appendChild(dot);
    }
  }

  /** 进入回放：暂停实时仿真 → 加载数据（服务器运行或导入归档）→ 显示回放栏 */
  async function enterReplay(load: () => Promise<void> | void) {
    debug.paused = true;
    await load();
    if (replayBar) replayBar.style.display = 'flex';
    renderMarkers();
  }

  document.getElementById('btn-replay-mode')?.addEventListener('click', async () => {
    const runs = await ReplayController.listRuns();
    if (runs.length === 0) {
      alert('没有可回放的运行。先执行: npm run sim -- --scenario scenarios/oval-cruise.json');
      return;
    }
    const choice = prompt('选择运行（输入编号）:\n' + runs.map((r, i) => `${i}: ${r.label}`).join('\n'), '0');
    if (choice === null) return;
    const idx = Number(choice);
    if (Number.isNaN(idx) || !runs[idx]) return;
    await enterReplay(() => replay.load(runs[idx].id));
  });

  document.getElementById('btn-replay-play')?.addEventListener('click', () => replay.toggle());
  document.getElementById('btn-replay-prev')?.addEventListener('click', () => replay.seekStep(replay.state.currentStep - 1));
  document.getElementById('btn-replay-next')?.addEventListener('click', () => replay.seekStep(replay.state.currentStep + 1));
  (document.getElementById('replay-speed') as HTMLSelectElement | null)
    ?.addEventListener('change', (e: any) => replay.setSpeed(Number(e.target.value)));
  document.getElementById('btn-replay-close')?.addEventListener('click', () => {
    replay.close();
    if (replayBar) replayBar.style.display = 'none';
    debug.paused = false;                      // 恢复实时仿真
  });
  timeline?.addEventListener('input', () => replay.seekStep(Number(timeline.value)));
  replay.onChange = () => {
    if (timeline) {
      timeline.value = String(Math.floor(replay.state.currentStep));
      timeline.max = String(replay.state.totalSteps);
    }
    if (stepLabel) stepLabel.textContent = `${Math.floor(replay.state.currentStep)} / ${replay.state.totalSteps}`;
  };
  window.addEventListener('keydown', (e) => {
    if (!replay.state.active || e.target instanceof HTMLInputElement) return;
    if (e.key === 'ArrowLeft') replay.seekStep(replay.state.currentStep - 1);
    if (e.key === 'ArrowRight') replay.seekStep(replay.state.currentStep + 1);
  });
}
