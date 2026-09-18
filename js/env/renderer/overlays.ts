// ==================== 渲染器：动态覆盖层 ====================
//
// 【职责】管理每帧更新的非静态几何体：
//   - LiDAR 射线（位置 + 颜色）
//   - 规划轨迹线
//   - 路径路线 + 起点/终点标记
//   - GPS 不确定度环
//   - 障碍物位置标记
//
// 【资源管理】每个覆盖层维护自己的懒缓存，resetCache() 统一销毁。

import * as THREE from 'three';
import { LidarScan, Trajectory, Route, GPSReading } from '../../shared/types.js';
import { Px } from '../../shared/dimensions.js';
import { MAX_SENSOR_DIST } from '../../algo/perception.js';

// ═══════════════════════════════════════════════════
// 缓存
// ═══════════════════════════════════════════════════

let _trajectoryLine: THREE.Line | null = null;
let _routeLine: THREE.Line | null = null;
let _startMarker: THREE.Mesh | null = null;
let _goalMarker: THREE.Mesh | null = null;
let _gpsRing: THREE.Mesh | null = null;
let _obsMarkers: THREE.Group | null = null;

/** 销毁所有覆盖层缓存 */
export function resetOverlayCache(scene: THREE.Scene) {
  for (const obj of [_trajectoryLine, _routeLine, _startMarker, _goalMarker, _gpsRing, _obsMarkers]) {
    if (obj) {
      scene.remove(obj);
      if ('geometry' in obj) obj.geometry.dispose();
      if ('material' in obj) (obj as any).material.dispose?.();
    }
  }
  _trajectoryLine = null;
  _routeLine = null;
  _startMarker = null;
  _goalMarker = null;
  _gpsRing = null;
  _obsMarkers = null;
}

// ═══════════════════════════════════════════════════
// LiDAR 射线
// ═══════════════════════════════════════════════════

/** 更新 LiDAR 射线池（复用 61 条 Line，只更新顶点坐标和颜色） */
export function updateLidar(scene: THREE.Scene, lidarScan: LidarScan | null, lidarLines: THREE.Line[], carX: number, carZ: number) {
  if (!lidarScan || !lidarScan.angles) return;

  for (let i = 0; i < lidarLines.length; i++) {
    if (i < lidarScan.angles.length) {
      const angle = lidarScan.angles[i] as number;
      // 渲染用 rawDistances（保留边界命中），规划器用 distances（已过滤边界）
      const raw = lidarScan.rawDistances;
      const dist = (raw && i < raw.length ? raw[i] : lidarScan.distances[i]) as number;
      const endX = carX + Math.cos(angle) * dist;
      const endZ = carZ + Math.sin(angle) * dist;

      const pos = lidarLines[i].geometry.attributes.position.array;
      pos[0] = carX; pos[1] = 2.5; pos[2] = carZ;
      pos[3] = endX; pos[4] = 0.5; pos[5] = endZ;
      lidarLines[i].geometry.attributes.position.needsUpdate = true;

      const t = Math.min(1, dist / 200);
      (lidarLines[i].material as THREE.LineBasicMaterial).color.setRGB(
        1 - t * 0.4, 0.7 + t * 0.3, 0.4 * (1 - t) + 0.6 * t
      );
      lidarLines[i].visible = true;
    } else {
      lidarLines[i].visible = false;
    }
  }
}

// ═══════════════════════════════════════════════════
// 轨迹线
// ═══════════════════════════════════════════════════

/** 更新规划轨迹线（每帧重建） */
export function updateTrajectoryLine(scene: THREE.Scene, trajectory: Trajectory | null) {
  if (_trajectoryLine) {
    scene.remove(_trajectoryLine);
    _trajectoryLine.geometry.dispose();
    (_trajectoryLine.material as THREE.Material).dispose();
    _trajectoryLine = null;
  }

  if (!trajectory || !trajectory.points || trajectory.points.length < 2) return;

  const pts3 = trajectory.points.map(p => new THREE.Vector3(p.x as number, 0.05, p.y as number));
  const color = trajectory.type === 'emergency' ? 0xff0000 : 0x4488ff;
  const mat = new THREE.LineBasicMaterial({ color, linewidth: 2, transparent: true, opacity: 0.7 });
  _trajectoryLine = new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts3), mat);
  scene.add(_trajectoryLine);
}

// ═══════════════════════════════════════════════════
// 路径路线 + 起终点标记
// ═══════════════════════════════════════════════════

/** 更新路径路线（黄色虚线）+ 起点（绿色球）+ 终点（红色球） */
export function updateRouteLine(scene: THREE.Scene, route: Route | null) {
  if (_routeLine) { scene.remove(_routeLine); _routeLine.geometry.dispose(); (_routeLine.material as THREE.Material).dispose(); _routeLine = null; }
  if (_startMarker) { scene.remove(_startMarker); _startMarker.geometry.dispose(); (_startMarker.material as THREE.Material).dispose(); _startMarker = null; }
  if (_goalMarker) { scene.remove(_goalMarker); _goalMarker.geometry.dispose(); (_goalMarker.material as THREE.Material).dispose(); _goalMarker = null; }

  if (!route || route.waypoints.length < 2) return;

  // 黄色虚线路由
  const pts3 = route.waypoints.map(p => new THREE.Vector3(p.x as number, 0.08, p.y as number));
  const geom = new THREE.BufferGeometry().setFromPoints(pts3);
  const mat = new THREE.LineDashedMaterial({ color: 0xffaa00, dashSize: 10, gapSize: 8 });
  _routeLine = new THREE.Line(geom, mat);
  _routeLine.computeLineDistances();
  scene.add(_routeLine);

  // 起点标记（绿色球体）
  const start = route.waypoints[0];
  _startMarker = new THREE.Mesh(
    new THREE.SphereGeometry(5, 8, 8),
    new THREE.MeshStandardMaterial({ color: 0x44ff44, emissive: 0x224422 }),
  );
  _startMarker.position.set(start.x as number, 0.3, start.y as number);
  scene.add(_startMarker);

  // 终点标记（红色球体）
  const goal = route.waypoints[route.waypoints.length - 1];
  _goalMarker = new THREE.Mesh(
    new THREE.SphereGeometry(5, 8, 8),
    new THREE.MeshStandardMaterial({ color: 0xff4444, emissive: 0x442222 }),
  );
  _goalMarker.position.set(goal.x as number, 0.3, goal.y as number);
  scene.add(_goalMarker);
}

// ═══════════════════════════════════════════════════
// GPS 环
// ═══════════════════════════════════════════════════

/** 更新 GPS 不确定度环（蓝色半透明圆环） */
export function updateGPSRing(scene: THREE.Scene, gps: GPSReading | null) {
  if (_gpsRing) { scene.remove(_gpsRing); _gpsRing.geometry.dispose(); (_gpsRing.material as THREE.Material).dispose(); _gpsRing = null; }
  if (!gps) return;

  const ringGeom = new THREE.RingGeometry((gps.uncertainty as number) - 1, gps.uncertainty as number, 32);
  const ringMat = new THREE.MeshBasicMaterial({ color: 0x8888ff, side: THREE.DoubleSide, transparent: true, opacity: 0.3 });
  _gpsRing = new THREE.Mesh(ringGeom, ringMat);
  _gpsRing.rotation.x = -Math.PI / 2;
  _gpsRing.position.set(gps.x as number, 0.05, gps.y as number);
  scene.add(_gpsRing);
}

// ═══════════════════════════════════════════════════
// 障碍物位置标记（AVOID 状态时显示）
// ═══════════════════════════════════════════════════

/** 在 AVOID 状态时，用红色小球标记 LiDAR 检测到的障碍物位置 */
export function updateObstacleMarkers(scene: THREE.Scene, lidarScan: LidarScan | null, carX: number, carZ: number, isAvoid: boolean) {
  if (_obsMarkers) { scene.remove(_obsMarkers); _obsMarkers = null; }
  if (!isAvoid || !lidarScan) return;

  const group = new THREE.Group();
  const maxDist = MAX_SENSOR_DIST as number;
  const markerGeom = new THREE.SphereGeometry(1.5, 4, 4);
  const markerMat = new THREE.MeshBasicMaterial({ color: 0xff2222 });

  const confidences = lidarScan.confidences;

  for (let i = 0; i < lidarScan.angles.length; i++) {
    const dist = lidarScan.distances[i] as number;
    if (dist >= maxDist - 1) continue;  // 边界过滤后或未命中

    // 只标记高置信度障碍物（confidence≧1），低置信（0.5=边缘模糊）不标记
    const conf = confidences && i < confidences.length ? confidences[i] : 1;
    if (conf < 1) continue;

    const angle = lidarScan.angles[i] as number;
    const x = carX + Math.cos(angle) * dist;
    const z = carZ + Math.sin(angle) * dist;

    const marker = new THREE.Mesh(markerGeom, markerMat);
    marker.position.set(x, 1.5, z);
    group.add(marker);
  }

  _obsMarkers = group;
  scene.add(group);
}
