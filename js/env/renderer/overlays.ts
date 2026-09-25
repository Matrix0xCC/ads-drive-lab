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
import type { PlanningViz } from '../../shared/debug.js';
import { Px } from '../../shared/dimensions.js';
import { MAX_SENSOR_DIST } from '../../algo/perception.js';

// ═══════════════════════════════════════════════════
// 缓存
// ═══════════════════════════════════════════════════

let _trajectoryLine: THREE.Line | null = null;
let _corridorGroup: THREE.Group | null = null;
let _routeLine: THREE.Line | null = null;
let _startMarker: THREE.Mesh | null = null;
let _goalMarker: THREE.Mesh | null = null;
let _gpsRing: THREE.Mesh | null = null;
let _obsMarkers: THREE.Group | null = null;

/** 销毁所有覆盖层缓存 */
export function resetOverlayCache(scene: THREE.Scene) {
  for (const obj of [_trajectoryLine, _corridorGroup, _routeLine, _startMarker, _goalMarker, _gpsRing, _obsMarkers]) {
    if (obj) {
      scene.remove(obj);
      if ('geometry' in obj) obj.geometry.dispose();
      if ('material' in obj) (obj as any).material.dispose?.();
    }
  }
  _trajectoryLine = null;
  _corridorGroup = null;
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

// ═══════════════════════════════════════════════════
// 走廊可视化（自由带 / 瓶颈 / 候选路径与发展段）
// ═══════════════════════════════════════════════════

/** 递归释放一个组的几何/材质 */
function disposeGroup(g: THREE.Object3D) {
  g.traverse((o: any) => {
    o.geometry?.dispose?.();
    const m = o.material;
    if (Array.isArray(m)) m.forEach((x: any) => x.dispose?.());
    else m?.dispose?.();
  });
}

/** 每帧重建（与轨迹线同风格）。颜色约定：
 *  自由带 = 半透明绿（车中心可达区，断开处即阻塞）；
 *  瓶颈横断线 = 橙（有缝）/ 红（全封死）；
 *  候选路径 = 按分级上色（畅通蓝 / 慢行橙 / 硬阻塞红），最优亮绿；
 *  每条候选的前 develop 米（横移发展段）用不透明同色叠画——看"横移何时完成、到障碍前是否展开"。 */
export function updateCorridorViz(scene: THREE.Scene, viz: PlanningViz | null) {
  if (_corridorGroup) {
    scene.remove(_corridorGroup);
    disposeGroup(_corridorGroup);
    _corridorGroup = null;
  }
  if (!viz) return;

  const g = new THREE.Group();

  // 自由带（四边形 → 两个三角形）：分两组网格——被雷达扫到的（绿）与未观测的（灰蓝）
  const addBands = (idx: number[], color: number, opacity: number) => {
    const pos: number[] = [];
    for (const i of idx) {
      const q = viz.bands[i];
      if (!q || q.length < 4) continue;
      const [p1, p2, p3, p4] = q;
      for (const t of [[p1, p2, p3], [p1, p3, p4]]) {
        for (const p of t) pos.push(p.x, 0.05, p.y);
      }
    }
    if (pos.length === 0) return;
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
    g.add(new THREE.Mesh(geo, new THREE.MeshBasicMaterial({
      color, transparent: true, opacity, side: THREE.DoubleSide, depthWrite: false })));
  };
  const seen: number[] = [], unseen: number[] = [];
  for (let i = 0; i < viz.bands.length; i++) {
    ((viz.bandObserved[i] ?? true) ? seen : unseen).push(i);
  }
  addBands(seen, 0x22dd77, 0.13);      // 观测到 → 绿（自由）
  addBands(unseen, 0x6b7a8f, 0.30);    // 未观测 → 灰蓝（未知，不代表自由；调亮以便一眼可分）
  // 未观测区描边（外边界线段由 planner 预先算好，渲染层只画）
  if (viz.unobsBoundary.length > 0) {
    const pts: THREE.Vector3[] = [];
    for (const sg of viz.unobsBoundary) {
      pts.push(new THREE.Vector3(sg[0].x, 0.07, sg[0].y));
      pts.push(new THREE.Vector3(sg[1].x, 0.07, sg[1].y));
    }
    g.add(new THREE.LineSegments(
      new THREE.BufferGeometry().setFromPoints(pts),
      new THREE.LineBasicMaterial({ color: 0x9aa7b5, transparent: true, opacity: 0.6 })));
  }

  // 观测极限线（灰蓝）：再往前就没有传感器信息了
  for (const seg of viz.obsLimitSegments) {
    if (seg.length < 2) continue;
    g.add(new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(seg.map(p => new THREE.Vector3(p.x, 0.12, p.y))),
      new THREE.LineBasicMaterial({ color: 0x9aa7b8, transparent: true, opacity: 0.85 })));
  }

  // 瓶颈横断线
  const pinchColor = viz.pinch && viz.pinch.width > 0 ? 0xffaa33 : 0xff3322;
  for (const seg of viz.pinchSegments) {
    if (seg.length < 2) continue;
    g.add(new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(seg.map(p => new THREE.Vector3(p.x, 0.12, p.y))),
      new THREE.LineBasicMaterial({ color: pinchColor, transparent: true, opacity: 0.9 })));
  }

  // 候选路径：①在首个阻塞点截断（舒适阻塞点之后淡出、硬阻塞点之后不再画）
  //          ②只有最优画发展段，非最优更淡（车头不再糊成一团）
  const lineMat = (color: number, opacity: number) =>
    new THREE.LineBasicMaterial({ color, transparent: true, opacity });
  for (const c of viz.candidates) {
    if (c.points.length < 2) continue;
    const isBest = c.targetL === viz.bestTargetL;
    const color = c.blockedHard ? 0xef4444 : (c.blocked ? 0xf59e0b : 0x3b82f6);
    const y = isBest ? 0.16 : 0.10;
    const usableEnd = Math.min(c.blocked ? c.sBlock : Infinity, c.blockedHard ? c.sBlockHard : Infinity);
    const hardEnd = c.blockedHard ? c.sBlockHard : Infinity;
    const solid = c.points.filter(p => p.s <= usableEnd + 1e-6);
    const faded = c.points.filter(p => p.s > usableEnd + 1e-6 && p.s <= hardEnd + 1e-6);
    if (solid.length >= 2) {
      g.add(new THREE.Line(
        new THREE.BufferGeometry().setFromPoints(solid.map(p => new THREE.Vector3(p.x, y, p.y))),
        lineMat(isBest ? 0x22ff88 : color, isBest ? 0.95 : 0.22)));
    }
    // 淡出段：把"用不了"的那截画出来但明显更暗（起点接上上一段，避免断口）
    const fadePts = solid.length > 0 ? [solid[solid.length - 1], ...faded] : faded;
    if (fadePts.length >= 2) {
      g.add(new THREE.Line(
        new THREE.BufferGeometry().setFromPoints(fadePts.map(p => new THREE.Vector3(p.x, y, p.y))),
        lineMat(isBest ? 0x22ff88 : color, 0.12)));
    }
    // 发展段只画最优（与路径线同色、不透明）——非最优画了只在车头叠成一片
    if (isBest) {
      const s0 = c.points[0].s;
      const devPts = solid.filter(p => (p.s - s0) <= c.develop + 1e-6);
      if (devPts.length >= 2) {
        g.add(new THREE.Line(
          new THREE.BufferGeometry().setFromPoints(devPts.map(p => new THREE.Vector3(p.x, y + 0.02, p.y))),
          lineMat(0x22ff88, 1.0)));
      }
    }
  }

  _corridorGroup = g;
  scene.add(g);
}
