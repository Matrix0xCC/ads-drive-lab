// ==================== 渲染器：静态几何体构建 ====================
//
// 【职责】构建赛道、障碍物、车辆的 Three.js 几何体。
// 这些函数只在首次调用时执行一次（懒创建），后续帧只更新 position/rotation。
//
// 【坐标系】canvas(x,y) → Three.js(x, 0, z)

import * as THREE from 'three';
import { TrackGeometry, Obstacle } from '../../shared/types.js';
import { Px, m, px } from '../../shared/dimensions.js';
import { buildFrenetReference, frenetToCartesian } from '../../algo/plan/frenet.js';

// ═══════════════════════════════════════════════════
// 赛道
// ═══════════════════════════════════════════════════

/**
 * 构建赛道 3D 模型
 * - 路面：ShapeGeometry（外边界挖掉内边界 = 环形路面）
 * - 内外边界线（实线）+ 中心线（虚线）
 */
export function buildTrackGroup(track: TrackGeometry): THREE.Group {
  const group = new THREE.Group();

  // 路面：ShapeGeometry 在 XY 平面创建，旋转 -PI/2 到 XZ 地面时 Z 轴翻转。
  // 因此 Y 取负值，保证旋转后 canvas(x,y) 正确映射到 Three.js(x,0,z)。
  // winding 用 ShapeUtils 检测并修正（外边界 CCW、孔 CW）。
  const toPoints = (pts: { x: Px; y: Px }[]) =>
    pts.map(p => new THREE.Vector2(p.x as number, -(p.y as number)));

  const outerPts = toPoints(track.outerBoundary);
  const innerPts = toPoints(track.innerBoundary);

  const shape = new THREE.Shape(
    THREE.ShapeUtils.isClockWise(outerPts) ? outerPts.reverse() : outerPts
  );
  const hole = new THREE.Path(
    THREE.ShapeUtils.isClockWise(innerPts) ? innerPts : innerPts.reverse()
  );
  shape.holes.push(hole);

  const roadGeom = new THREE.ShapeGeometry(shape);
  const roadMesh = new THREE.Mesh(roadGeom, new THREE.MeshStandardMaterial({ color: 0x3a3a3a }));
  roadMesh.rotation.x = -Math.PI / 2;
  roadMesh.position.y = 0.01;
  roadMesh.userData = { type: 'track', label: '赛道', width: 15, height: 0, length: '环形' };
  group.add(roadMesh);

  // 赛道轮廓线（辅助函数）
  function addLoopLine(points: { x: Px; y: Px }[], color: number, dashMode: boolean) {
    const pts3 = points.map(p => new THREE.Vector3(p.x as number, 0.02, p.y as number));
    const mat = dashMode
      ? new THREE.LineDashedMaterial({ color, dashSize: 8, gapSize: 12 })
      : new THREE.LineBasicMaterial({ color });
    const line = new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts3), mat);
    if (dashMode) line.computeLineDistances();
    group.add(line);
  }
  addLoopLine(track.outerBoundary, 0x888888, false);
  addLoopLine(track.innerBoundary, 0x888888, false);
  addLoopLine(track.centerPoints, 0xaaaaaa, true);

  // 4 车道分隔线（15m ÷ 4 = 3.75m/车道）
  // 分隔线位于 Frenet l = ±3.75m（半宽的 50%）
  const laneRef = buildFrenetReference(track.centerPoints, true);
  const halfLaneWidth = (m(3.75) as number);  // 3.75m → 15 sim-units
  for (const lSign of [-1, 1]) {
    const lv = px(lSign * halfLaneWidth);
    const pts: { x: Px; y: Px }[] = [];
    for (let i = 0; i < track.centerPoints.length; i++) {
      const s = laneRef.s[i];
      const cart = frenetToCartesian(s, lv, laneRef);
      pts.push({ x: px(cart.x as number), y: px(cart.y as number) });
    }
    addLoopLine(pts, 0x666644, true);  // 浅黄虚线
  }

  return group;
}

// ═══════════════════════════════════════════════════
// 障碍物
// ═══════════════════════════════════════════════════

/** 构建障碍物组：红色 BoxGeometry + 边框线 */
export function buildObstacleGroup(obstacles: Obstacle[]): THREE.Group {
  const group = new THREE.Group();
  for (const obs of obstacles) {
    const zH = obs.zHeight || 8;
    const geom = new THREE.BoxGeometry(obs.width as number, zH, obs.height as number);
    const mat = new THREE.MeshStandardMaterial({ color: 0xdd3232, roughness: 0.4 });
    const mesh = new THREE.Mesh(geom, mat);
    mesh.position.set(obs.x as number, zH / 2, obs.y as number);
    mesh.rotation.y = -(obs.angle as number);
    mesh.userData = { type: 'obstacle', label: '障碍物', width: obs.width, height: zH, length: obs.height };

    const edgeGeom = new THREE.EdgesGeometry(geom);
    const edgeLine = new THREE.LineSegments(edgeGeom, new THREE.LineBasicMaterial({ color: 0xff4444 }));
    mesh.add(edgeLine);

    group.add(mesh);
  }
  return group;
}

// ═══════════════════════════════════════════════════
// 车辆
// ═══════════════════════════════════════════════════

/** 构建车辆模型：橙色车身 + 深橙驾驶舱 + 白色车头锥形箭头 */
export function buildCarGroup(): THREE.Group {
  const group = new THREE.Group();

  // 车身：2.0m 宽 × 4.5m 长（4px/m → 8 × 18 sim-units）
  const bodyGeom = new THREE.BoxGeometry(18, 3, 8);
  const bodyMesh = new THREE.Mesh(bodyGeom, new THREE.MeshStandardMaterial({ color: 0xffaa00 }));
  bodyMesh.position.set(0, 1.5, 0);
  bodyMesh.userData = { type: 'car', label: '车辆', width: 2.0, height: 0.75, length: 4.5 };
  group.add(bodyMesh);

  // 驾驶舱
  const cabinGeom = new THREE.BoxGeometry(7, 2.5, 7);
  const cabinMesh = new THREE.Mesh(cabinGeom, new THREE.MeshStandardMaterial({ color: 0xff8800 }));
  cabinMesh.position.set(2, 3.5, 0);
  group.add(cabinMesh);

  // 车头箭头
  const arrowGeom = new THREE.ConeGeometry(1.5, 4, 4);
  const arrowMesh = new THREE.Mesh(arrowGeom, new THREE.MeshStandardMaterial({ color: 0xffffff }));
  arrowMesh.rotation.z = -Math.PI / 2;
  arrowMesh.position.set(11, 1.5, 0);
  group.add(arrowMesh);

  return group;
}
