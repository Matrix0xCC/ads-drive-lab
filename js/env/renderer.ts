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
import { initScene, frameCamera, frameSmart, followCar, setWorldExtent, FOLLOW_WORLD_THRESHOLD } from './renderer/scene.js';
import { buildTrackGroup, buildObstacleGroup, buildCarGroup, buildFrenetTicksGroup } from './renderer/geometry.js';
import { updateLidar, updateTrajectoryLine, updateRouteLine, updateGPSRing, updateObstacleMarkers, updateCorridorViz, resetOverlayCache } from './renderer/overlays.js';
import type { PlanningViz } from '../shared/debug.js';
import { updateHUD } from './renderer/hud.js';
import { buildFrenetReference } from '../algo/plan/frenet.js';
import { TRACK_HALF_WIDTH } from '../env/track.js';
import { BOUNDS_BUFFER } from '../env/car.js';
import { DOMAIN_OVERSHOOT_RESERVE } from '../algo/plan/paths.js';
import { m } from '../shared/dimensions.js';

// ═══════════════════════════════════════════════════
// 懒缓存（静态几何体，首次使用时创建，地图切换时清空）
// ═══════════════════════════════════════════════════

let _trackGroup: THREE.Group | null = null;
let _obstacleGroup: THREE.Group | null = null;
let _carGroup: THREE.Group | null = null;
// Frenet 刻度组（与走廊可视化独立开关）：参考线口径与规划器一致（有路线用路线开环），
// 因此按"路线/赛道对象身份"重建——换路线或换地图即失效重建。
let _frenetTicks: THREE.Group | null = null;
let _frenetTicksKey: unknown = null;

/** 规划域半宽（与 paths.ts 的 L_MAX 同式；用于刻度里那条与走廊同色的域边线） */
/** 规划域半宽（与 paths.ts 的 L_MAX 同式；随地图半宽变化） */
const domainHalfOf = (halfWidthU: number): number =>
  halfWidthU - ((m(2.0) as number) / 2 + (BOUNDS_BUFFER as number)) - (DOMAIN_OVERSHOOT_RESERVE as number);

/** 清空所有缓存（地图切换时调用，触发懒重建） */
export function resetSceneCache(scene: THREE.Scene) {
  for (const obj of [_trackGroup, _obstacleGroup, _carGroup, _frenetTicks]) {
    if (obj) scene.remove(obj);
  }
  _trackGroup = null;
  _obstacleGroup = null;
  _frenetTicks = null;
  _frenetTicksKey = null;
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
  corridorViz: PlanningViz | null = null, showFrenetTicks = true,
  showLidar = true, showCandidateLabels = false,
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
  updateLidar(scene, showLidar ? lidarScan : null, lidarLines, car.x as number, car.y as number);
  updateObstacleMarkers(scene, lidarScan, car.x as number, car.y as number, plannerState === 'AVOID');
  updateRouteLine(scene, route ?? null);
  updateGPSRing(scene, gps ?? null);
  updateTrajectoryLine(scene, trajectory);
  updateCorridorViz(scene, corridorViz, showCandidateLabels);

  // Frenet 刻度：参考线口径与规划器一致——有路线用路线开环参考线（s 从路线起点算，
  // 这样 debug 面板的 S 值能在地上直接读出来），否则用赛道闭环。
  const ticksKey: unknown = route ?? track;
  if (!showFrenetTicks) {
    if (_frenetTicks) _frenetTicks.visible = false;
  } else {
    if (!_frenetTicks || _frenetTicksKey !== ticksKey) {
      if (_frenetTicks) scene.remove(_frenetTicks);
      const ref = route
        ? buildFrenetReference(route.waypoints, false)
        : buildFrenetReference(track.centerPoints, track.closed ?? true);
      _frenetTicks = buildFrenetTicksGroup(ref, {
        maxS: ref.totalS as number,
        roadHalf: (track.halfWidth ?? TRACK_HALF_WIDTH) as number,
        domainHalf: domainHalfOf((track.halfWidth ?? TRACK_HALF_WIDTH) as number),
      });
      _frenetTicksKey = ticksKey;
      scene.add(_frenetTicks);
    }
    _frenetTicks.visible = true;
  }

  // HUD
  updateHUD(lidarScan, car, plannerState, obstacles, trajectory, route ?? null, gps ?? null);
}

export { initScene, frameCamera, frameSmart, followCar, setWorldExtent, FOLLOW_WORLD_THRESHOLD };
