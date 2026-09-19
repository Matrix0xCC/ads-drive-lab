// ==================== UI：画布交互 ====================
// 左键：物体信息 tooltip；右键：路线拾取（起点 → 终点两段式）。
// 状态逻辑见 state.ts/controller.ts（T5 接线）。

import * as THREE from 'three';
import type { UIContext } from './index.js';
import { createRoute } from '../shared/types.js';
import { px } from '../shared/dimensions.js';
import { findRoute, findClosestPoint } from '../algo/routePlanner.js';

export function wireInteractions(ctx: UIContext): void {
  const { canvas, camera } = ctx;

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
}
