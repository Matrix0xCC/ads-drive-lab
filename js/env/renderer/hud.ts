// ==================== 渲染器：HUD（HTML DOM 叠层） ====================
//
// 【职责】纯展示层 — 只读取上游模块（感知、规划、控制）的输出数据，
//        写入对应 DOM 元素，不做任何独立计算。
//
// 长度显示为物理单位（m，SIM_UNITS_PER_METER=4 转换）；速度单位由用户偏好
// 决定（默认 km/h，点击切换 m/s——偏好存于 shared/display.ts，由 ui 层写入）。

import { LidarScan, Trajectory, Obstacle, CarState, Route, GPSReading } from '../../shared/types.js';
import { SIM_UNITS_PER_METER } from '../../shared/dimensions.js';
import { display, formatSpeed } from '../../shared/display.js';

/** 仿真单位 → 物理量显示：dist / 4 = 米，保留 d 位小数 */
const toM = (v: number, d = 1) => (v / SIM_UNITS_PER_METER).toFixed(d);

/**
 * 从 LiDAR 扫描中提取前方最近障碍物距离
 *
 * 只取 LiDAR 前方 ±30° 锥形区域内的最小值（代表"最近威胁"）。
 * 不做独立的欧氏距离搜索 — 这已是 perception 模块的职责。
 */
function minForwardDist(lidarScan: LidarScan | null): number | null {
  if (!lidarScan || !lidarScan.distances || lidarScan.distances.length === 0) return null;

  const mid = Math.floor(lidarScan.distances.length / 2);
  // 前方 ±30° = 约 15 条射线（2°/条）
  const halfCone = 15;
  let minDist = Infinity;
  for (let i = mid - halfCone; i <= mid + halfCone; i++) {
    if (i >= 0 && i < lidarScan.distances.length) {
      const d = lidarScan.distances[i] as number;
      if (d < minDist) minDist = d;
    }
  }
  // 只有实际命中障碍物（距离 < MAX_SENSOR_DIST）才算
  return minDist < 250 ? minDist : null;
}

/**
 * 更新 HUD 叠层
 *
 * 所有显示数据来自上游模块的输出，HUD 不做独立计算。
 */
export function updateHUD(
  lidarScan: LidarScan | null,
  car: CarState,
  state: string,
  obstacles: Obstacle[],
  trajectory: Trajectory | null,
  route: Route | null,
  gps: GPSReading | null,
) {
  const elFront = document.getElementById('hud-front');
  const elSpeed = document.getElementById('hud-speed');
  const elState = document.getElementById('hud-state');
  const elRays  = document.getElementById('hud-rays');
  const elObs   = document.getElementById('hud-obstacles');
  const elObsInfo = document.getElementById('hud-obstacle-info');

  // 基础数据
  if (elFront && lidarScan?.distances) {
    const mid = Math.floor(lidarScan.distances.length / 2);
    elFront.textContent = `${toM(lidarScan.distances[mid] as number)} m`;
  }
  if (elSpeed) elSpeed.textContent = formatSpeed(car.speed as number, display.speedUnit);
  if (elRays)  elRays.textContent = lidarScan ? String(lidarScan.angles.length) : '0';
  if (elObs)   elObs.textContent = String(obstacles.length);

  // 转向/油门（回放旧归档可能缺字段/含 NaN→null：显示 '---'，不崩溃）
  const elSteer = document.getElementById('hud-steer');
  const elThrottle = document.getElementById('hud-throttle');
  if (elSteer) {
    elSteer.textContent = Number.isFinite(car.steerAngle as number)
      ? (car.steerAngle as number).toFixed(3) : '---';
  }
  if (elThrottle) {
    const maxV = car.maxSpeed as number;
    elThrottle.textContent = Number.isFinite(maxV) && maxV > 0
      ? ((car.speed as number) / maxV).toFixed(2) : '---';
  }

  // 状态行（state 现在是 PlanState 枚举值）
  const stateLabel: Record<string, string> = {
    CRUISE: '巡航中', FOLLOW: '行驶中', AVOID: '避障中',
    DECEL: '减速中', ARRIVED: '已到达', EMERGENCY: '紧急',
  };
  const label = stateLabel[state] || state;
  const isAlert = state === 'AVOID' || state === 'EMERGENCY';
  if (elState) {
    elState.textContent = `状态: ${label}`;
    elState.className = isAlert ? 'state-avoid' : 'state-normal';
  }

  // 障碍物详情（仅避障时显示）
  if (elObsInfo) {
    if (isAlert) {
      const obsDist = minForwardDist(lidarScan);
      if (obsDist !== null) {
        // 方向来自规划器的决策（trajectory.type），距离来自 LiDAR
        const dir = trajectory?.type === 'avoid_left' ? '左侧' : '右侧';
        elObsInfo.innerHTML = `障碍物: ${dir} ${toM(obsDist, 0)}m（LiDAR 实测）`;
        elObsInfo.className = 'state-avoid';
      } else {
        elObsInfo.textContent = '';
      }
    } else {
      elObsInfo.textContent = '';
    }
  }

  // 路线状态
  const elRoute = document.getElementById('hud-route');
  if (elRoute) {
    if (route && trajectory && trajectory.type !== 'arrived') {
      elRoute.innerHTML = `路线: <span style="color:#ffaa00;">ACTIVE</span> | 剩余 ${toM(route.length as number, 0)}m`;
    } else if (trajectory?.type === 'arrived') {
      elRoute.innerHTML = `路线: <span style="color:#44ff44;">已到达</span>`;
    } else {
      elRoute.textContent = '路线: 点击赛道设置';
    }
  }

  // GPS
  const elGPS = document.getElementById('hud-gps');
  if (elGPS) {
    elGPS.textContent = gps
      ? `GPS: (${toM(gps.x as number, 0)}, ${toM(gps.y as number, 0)}) ±${toM(gps.uncertainty as number, 1)}m`
      : 'GPS: ---';
  }
}
