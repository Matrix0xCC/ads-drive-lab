// ==================== 显示单位偏好（渲染层只读） ====================
//
// 用户可切换的显示单位偏好（如速度 km/h ↔ m/s）。单一来源在 ui/state.ts 的
// UiState.speedUnit，由 ui 层经副作用写入此对象；渲染层（env/renderer）只读取。
// 本模块为叶子模块（无 import），保持依赖方向：renderer → shared。

import { SIM_UNITS_PER_METER } from './dimensions.js';

export type SpeedUnit = 'kmh' | 'ms';

/** 当前显示偏好（默认 km/h；由 ui 层同步） */
export const display: { speedUnit: SpeedUnit } = {
  speedUnit: 'kmh',
};

/**
 * 速度格式化：仿真单位 → 显示字符串
 *   m/s  = simUnits / SIM_UNITS_PER_METER
 *   km/h = m/s × 3.6
 */
export function formatSpeed(simSpeed: number, unit: SpeedUnit): string {
  const mps = simSpeed / SIM_UNITS_PER_METER;
  return unit === 'kmh' ? `${(mps * 3.6).toFixed(1)} km/h` : `${mps.toFixed(1)} m/s`;
}
