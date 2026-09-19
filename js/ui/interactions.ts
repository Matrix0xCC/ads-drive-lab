// ==================== UI：画布交互 ====================
// 左键：物体信息 tooltip；右键：路线拾取（两段式：起点 → 终点）。
// 拾取状态归 state.ts 的 routePick；建路线成功后 dispatch ROUTE_SET。

import * as THREE from 'three';
import { UiController } from './controller.js';
import type { UiDeps } from './index.js';
import { findRoute, findClosestPoint } from '../algo/routePlanner.js';
import { createRoute } from '../shared/types.js';
import { px } from '../shared/dimensions.js';

export function wireInteractions(deps: UiDeps, ctl: UiController): void {
  const { canvas, camera } = deps;

  // ── 左键点击 → 物体检查 ──
  canvas.addEventListener('click', (e: MouseEvent) => {
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

  // ── 右键点击 → 路线拾取 ──
  canvas.addEventListener('contextmenu', (e: MouseEvent) => {
    e.preventDefault();
    if (ctl.state.mode !== 'LIVE') return;   // 回放中不给实时世界设路线
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

    const idx = findClosestPoint(target.x, target.z, deps.engine.track.centerPoints);
    const st = ctl.state;

    if (st.routePick === 'idle') {
      ctl.dispatch({ type: 'RIGHT_CLICK', idx });
      return;
    }
    // picking-goal：尝试建路线；失败保持 picking（与旧行为一致）
    const startIdx = st.pickStartIdx!;
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
  });
}
