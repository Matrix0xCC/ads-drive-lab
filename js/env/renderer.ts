// ==================== 环境：渲染器（编排层） ====================
//
// 拆分后的模块：
//   renderer/scene.ts    — initScene（场景、相机、光照、射线池）
//   renderer/geometry.ts — buildTrack/Obstacle/CarGroup（静态几何体）
//   renderer/overlays.ts — LiDAR 射线 / 轨迹线 / 路线标记 / GPS 环（动态覆盖层）
//   renderer/hud.ts      — updateHUD / findNearestObstacle（DOM HUD）
//
// 本文件职责：
//   - 管理懒缓存（_trackGroup, _obstacleGroup, _carGroup）
//   - updateScene 编排调用各子模块
//   - resetSceneCache 地图切换时清空

import * as THREE from 'three';
import { WorldSnapshot, LidarScan, Trajectory, Route, GPSReading } from '../shared/types.js';
import { initScene, frameCamera } from './renderer/scene.js';
import { buildTrackGroup, buildObstacleGroup, buildCarGroup } from './renderer/geometry.js';
import { updateLidar, updateTrajectoryLine, updateRouteLine, updateGPSRing, updateObstacleMarkers, resetOverlayCache } from './renderer/overlays.js';
import { updateHUD } from './renderer/hud.js';

// ═══════════════════════════════════════════════════
// 懒缓存（静态几何体，首次使用时创建，地图切换时清空）
// ═══════════════════════════════════════════════════

let _trackGroup: THREE.Group | null = null;
let _obstacleGroup: THREE.Group | null = null;
let _carGroup: THREE.Group | null = null;

/** 清空所有缓存（地图切换时调用，触发懒重建） */
export function resetSceneCache(scene: THREE.Scene) {
  for (const obj of [_trackGroup, _obstacleGroup, _carGroup]) {
    if (obj) scene.remove(obj);
  }
  _trackGroup = null;
  _obstacleGroup = null;
  _carGroup = null;
  resetOverlayCache(scene);
}

// ═══════════════════════════════════════════════════
// 每帧更新编排
// ═══════════════════════════════════════════════════

export function updateScene(
  scene: THREE.Scene, snapshot: WorldSnapshot, lidarScan: LidarScan | null,
  plannerState: string, lidarLines: THREE.Line[], trajectory: Trajectory | null,
  route: Route | null = null, gps: GPSReading | null = null,
) {
  const { car, track, obstacles } = snapshot;

  // 懒创建静态几何体
  if (!_trackGroup) { _trackGroup = buildTrackGroup(track); scene.add(_trackGroup); }
  if (!_obstacleGroup) { _obstacleGroup = buildObstacleGroup(obstacles); scene.add(_obstacleGroup); }
  if (!_carGroup) { _carGroup = buildCarGroup(); scene.add(_carGroup); }

  // 更新车辆位姿
  _carGroup.position.set(car.x as number, 0.15, car.y as number);
  _carGroup.rotation.y = -(car.angle as number);

  // 动态覆盖层
  updateLidar(scene, lidarScan, lidarLines, car.x as number, car.y as number);
  updateObstacleMarkers(scene, lidarScan, car.x as number, car.y as number, plannerState === 'AVOID');
  updateRouteLine(scene, route ?? null);
  updateGPSRing(scene, gps ?? null);
  updateTrajectoryLine(scene, trajectory);

  // HUD
  updateHUD(lidarScan, car, plannerState, obstacles, trajectory, route ?? null, gps ?? null);
}

export { initScene, frameCamera };
