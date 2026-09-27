// ==================== UI：量距（可拖动的尺） ====================
//
// 【为什么是"模式"】左键拖动默认归 OrbitControls 做**相机旋转**（未覆盖 mouseButtons，用默认映射）
// → 直接拿左键量距会和转视角抢手势。所以走与「路线」同样的套路：toolbar 开一个模式，
// 模式内才接收拖动（并把 controls 暂时关掉），Esc / 再点按钮退出。
//
// 【尺子长什么样】与底部那条常驻比例尺**同一套视觉语言**：一条线 + 每"整数好看"间距一道横杠（可数格子）。
// 距离用**地理平面上的真值**（把指针射线打到 y=0 平面），所以俯视/斜视都准；
// 若按屏幕长度折算，斜视时越远越会被透视压缩（那是错的）。
//
// 尺子画在**场景里**（随地图一起转/缩放），读数用 DOM 跟随尺子中点。

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { SIM_UNITS_PER_METER } from '../shared/dimensions.js';
import { formatMeters, niceRulerInterval, rulerTicks } from '../env/renderer/scale.js';
import { bearingLabel, worldZToSim, simToWorldZ } from '../env/renderer/scene.js';

export interface MeasureDeps {
  canvas: HTMLCanvasElement;
  camera: THREE.PerspectiveCamera;
  scene: THREE.Scene;
  controls: OrbitControls;
}

export interface MeasureTool {
  setActive(on: boolean): void;
  isActive(): boolean;
  /** 每帧调用：让读数跟随尺子中点（相机移动/缩放时也要跟） */
  update(): void;
  /** 清掉当前量距结果 */
  reset(): void;
}

const GROUND_Y = 0.06;      // 尺子略高于路面（路面 0.01），避免 z-fighting
const TICK_HALF = 6;        // 刻度半长（世界单位 = 1.5m）

export function setupMeasure(deps: MeasureDeps): MeasureTool {
  const { canvas, camera, scene, controls } = deps;

  const group = new THREE.Group();
  group.visible = false;
  scene.add(group);
  const lineGeom = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]);
  const ruler = new THREE.Line(lineGeom, new THREE.LineBasicMaterial({ color: 0xffdd55 }));
  const tickGeom = new THREE.BufferGeometry();
  const ticks = new THREE.LineSegments(tickGeom, new THREE.LineBasicMaterial({ color: 0xffdd55 }));
  group.add(ruler, ticks);

  const readout = document.getElementById('measure-readout');
  const raycaster = new THREE.Raycaster();
  const ground = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
  const ndc = new THREE.Vector2();
  let active = false;
  let dragging = false;
  /** 仿真坐标下的两点（尺子是场景子节点 → 必须用仿真坐标，场景根的 z 镜像会把它们摆正） */
  let from: { x: number; y: number } | null = null;
  let to: { x: number; y: number } | null = null;

  /** 屏幕指针 → 地面上的仿真坐标（射线打 y=0 平面） */
  function pick(ev: PointerEvent): { x: number; y: number } | null {
    const r = canvas.getBoundingClientRect();
    if (r.width <= 0 || r.height <= 0) return null;
    ndc.set(((ev.clientX - r.left) / r.width) * 2 - 1, -((ev.clientY - r.top) / r.height) * 2 + 1);
    raycaster.setFromCamera(ndc, camera);
    const hit = new THREE.Vector3();
    if (!raycaster.ray.intersectPlane(ground, hit)) return null;   // 与地面平行/背向时不产生点
    return { x: hit.x, y: worldZToSim(hit.z) };
  }

  /** 仿真坐标 → 世界坐标（场景根镜像后它真正所在的位置；用于把读数投到屏幕上） */
  const toWorld = (p: { x: number; y: number }) => new THREE.Vector3(p.x, GROUND_Y, simToWorldZ(p.y));

  function redraw(): void {
    if (!from || !to) { group.visible = false; if (readout) readout.style.display = 'none'; return; }
    group.visible = true;
    const a = toWorld(from), b = toWorld(to);
    lineGeom.setFromPoints([a, b]);
    // 刻度间距随尺子长度取"整数好看"值（20m 尺子 5m 一格、2km 尺子 200m 一格）
    const lenM = Math.hypot(to.x - from.x, to.y - from.y) / SIM_UNITS_PER_METER;
    const interval = niceRulerInterval(lenM);
    const segs = rulerTicks(a, b, interval, SIM_UNITS_PER_METER, TICK_HALF);
    tickGeom.setFromPoints(segs.flat());
    if (readout) {
      readout.style.display = '';
      readout.textContent = `${formatMeters(lenM)}　${bearingLabel(bearingOf(from, to))}`;
    }
    update();
  }

  /** 方位角（0=正北、顺时针）：与罗盘同一套定义（北 = 仿真 +y） */
  function bearingOf(p: { x: number; y: number }, q: { x: number; y: number }): number {
    return (Math.atan2(q.x - p.x, q.y - p.y) * 180 / Math.PI + 360) % 360;
  }

  /** 把读数贴到尺子中点的屏幕位置 */
  function update(): void {
    if (!from || !to || !readout) return;
    const mid = toWorld(from).add(toWorld(to)).multiplyScalar(0.5).project(camera);
    const w = canvas.clientWidth, h = canvas.clientHeight;
    readout.style.left = `${(mid.x * 0.5 + 0.5) * w}px`;
    readout.style.top = `${(-mid.y * 0.5 + 0.5) * h}px`;
    readout.style.display = mid.z < 1 ? '' : 'none';       // 跑到相机背后时不显示
  }

  canvas.addEventListener('pointerdown', (ev) => {
    if (!active || ev.button !== 0) return;
    const p = pick(ev);
    if (!p) return;
    dragging = true;
    from = p; to = p;
    controls.enabled = false;                              // 模式内不让相机跟着转
    canvas.setPointerCapture(ev.pointerId);
    redraw();
  });
  canvas.addEventListener('pointermove', (ev) => {
    if (!dragging) return;
    const p = pick(ev);
    if (!p) return;
    to = p;
    redraw();
  });
  const finish = (ev: PointerEvent) => {
    if (!dragging) return;
    dragging = false;
    controls.enabled = true;
    try { canvas.releasePointerCapture(ev.pointerId); } catch { /* 已释放 */ }
    redraw();
  };
  canvas.addEventListener('pointerup', finish);
  canvas.addEventListener('pointercancel', finish);

  return {
    setActive(on: boolean) {
      active = on;
      canvas.style.cursor = on ? 'crosshair' : '';
      if (!on) { controls.enabled = true; dragging = false; }
    },
    isActive: () => active,
    update,
    reset() { from = to = null; redraw(); },
  };
}
