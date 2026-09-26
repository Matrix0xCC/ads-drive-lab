// ==================== UI：画布交互 ====================
// 路线设置改成**显式模式**（T / toolbar「路线」按钮进入；见 state.ts 的 routePick）：
//   模式内**左键**依次点起点、终点（悬停显示吸附预览），设完自动退出；Esc 取消。
//   右键不再参与拾取（只抑制浏览器菜单）——早期用 contextmenu 触发，右键拖拽平移视角
//   时 Chrome 在按下即触发 → 一动相机就误设起点/终点。
// 模式外左键仍是"点物体看尺寸"tooltip。

import * as THREE from 'three';
import { UiController } from './controller.js';
import type { UiDeps } from './index.js';
import { buildNetworkRoute, findRoute, findClosestPoint } from '../algo/routePlanner.js';
import { createRoute } from '../shared/types.js';
import { px } from '../shared/dimensions.js';

export function wireInteractions(deps: UiDeps, ctl: UiController): void {
  const { canvas, camera } = deps;

  /** 屏幕坐标 → 路网最近点索引（地面平面求交；越界返回 null） */
  function pickRouteIdx(clientX: number, clientY: number): number | null {
    const rect = canvas.getBoundingClientRect();
    const mouse = new THREE.Vector2(
      ((clientX - rect.left) / rect.width) * 2 - 1,
      -((clientY - rect.top) / rect.height) * 2 + 1,
    );
    const raycaster = new THREE.Raycaster();
    raycaster.setFromCamera(mouse, camera);
    const target = new THREE.Vector3();
    raycaster.ray.intersectPlane(new THREE.Plane(new THREE.Vector3(0, 1, 0), 0), target);
    if (!target) return null;
    return findClosestPoint(target.x, target.z, pickPoints());
  }

  /** 拾取空间：**路网地图吸附到节点**（路由的端点就是节点），链式地图吸附到中心线采样点 */
  function pickPoints() {
    const t = deps.engine.track;
    return t.netNodes && t.netNodes.length > 0 ? t.netNodes : t.centerPoints;
  }

  // ── 悬停预览：模式内显示"点下去会吸附到哪个路网点"（幽灵环） ──
  let preview: THREE.Mesh | null = null;
  /** 预览环配色与最终标记一致：起点绿（同绿色起点球）、终点红（同红色终点球），
   *  一眼区分当前在设哪一段。 */
  const PREVIEW_START = 0x44ff44, PREVIEW_GOAL = 0xff4444;
  function showPreview(idx: number | null): void {
    if (idx === null) { if (preview) preview.visible = false; return; }
    const wantColor = ctl.state.routePick === 'picking-goal' ? PREVIEW_GOAL : PREVIEW_START;
    if (!preview) {
      const geom = new THREE.TorusGeometry(8, 1.6, 8, 24);
      preview = new THREE.Mesh(geom, new THREE.MeshBasicMaterial({ color: wantColor, transparent: true, opacity: 0.9 }));
      preview.rotation.x = -Math.PI / 2;
      deps.scene.add(preview);
    }
    const mat = preview.material as THREE.MeshBasicMaterial;
    if (mat.color.getHex() !== wantColor) mat.color.setHex(wantColor);
    const p = pickPoints()[idx];
    preview.position.set(p.x as number, 0.6, p.y as number);
    preview.visible = true;
  }
  canvas.addEventListener('pointermove', (e: PointerEvent) => {
    if (ctl.state.routePick === 'idle') { showPreview(null); return; }
    showPreview(pickRouteIdx(e.clientX, e.clientY));
  });
  canvas.addEventListener('pointerleave', () => showPreview(null));

  // ── 左键点击 → 物体检查 / 路线拾取 ──
  canvas.addEventListener('click', (e: MouseEvent) => {
    // 路线模式：左键一律用于拾取（不弹物体 tooltip）
    if (ctl.state.routePick !== 'idle') {
      const idx = pickRouteIdx(e.clientX, e.clientY);
      if (idx === null) return;
      if (ctl.state.routePick === 'picking-start') {
        ctl.dispatch({ type: 'PICK_POINT', idx });
        return;
      }
      buildRouteFromStart(idx);
      return;
    }
    const rect = canvas.getBoundingClientRect();
    const mouse = new THREE.Vector2(
      ((e.clientX - rect.left) / rect.width) * 2 - 1,
      -((e.clientY - rect.top) / rect.height) * 2 + 1,
    );
    const raycaster = new THREE.Raycaster();
    raycaster.setFromCamera(mouse, camera);

    const hits = raycaster.intersectObjects(deps.scene.children, true);
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

  // ── 右键：仅抑制浏览器菜单（拾取已改为模式内左键） ──
  canvas.addEventListener('contextmenu', (e: MouseEvent) => e.preventDefault());

  /** 起点已选：尝试建路线；失败保持 picking（与旧行为一致） */
  function buildRouteFromStart(idx: number): void {
    const startIdx = ctl.state.pickStartIdx!;
    const tr = deps.engine.track;
    // 路网地图：节点对 → 边 → 链几何装配（与 headless 走**同一个** buildNetworkRoute）
    if (tr.chains && tr.netNodes && tr.netEdges) {
      const r = buildNetworkRoute(deps.engine.roadGraph, tr.netEdges, tr.chains, startIdx, idx);
      if (!r) return;                       // 不可达：保持 picking（与旧行为一致）
      let len = 0;
      for (let i = 1; i < r.waypoints.length; i++) {
        len += Math.hypot(
          (r.waypoints[i].x as number) - (r.waypoints[i - 1].x as number),
          (r.waypoints[i].y as number) - (r.waypoints[i - 1].y as number),
        );
      }
      deps.actions.setRoute(createRoute(r.waypoints, r.chainSeq, startIdx, idx, px(len)), startIdx);
      ctl.dispatch({ type: 'ROUTE_SET' });
      return;
    }
    const pathIndices = findRoute(deps.engine.roadGraph, startIdx, idx);
    if (!pathIndices || pathIndices.length < 2) return;
    const waypoints = pathIndices.map(i => deps.engine.track.centerPoints[i]);
    let totalLen = 0;
    for (let i = 1; i < waypoints.length; i++) {
      totalLen += Math.hypot(
        (waypoints[i].x as number) - (waypoints[i - 1].x as number),
        (waypoints[i].y as number) - (waypoints[i - 1].y as number),
      );
    }
    const r = createRoute(waypoints, pathIndices, startIdx, idx, px(totalLen));
    deps.actions.setRoute(r, startIdx);
    ctl.dispatch({ type: 'ROUTE_SET' });
  }
}
