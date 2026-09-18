// ==================== 环境：GPS 定位模拟 ====================
//
// 【职责】模拟 GPS 接收器，在车辆真实位置上叠加高斯噪声。
//
// 【噪声模型】
// 使用 Box-Muller 变换生成正态分布噪声。
// 噪声独立作用于 x 和 y 轴，标准差可配置。
// 不确定度 = 2σ（约 95% 置信区间半径）。
//
// 【输出频率】
// 10 Hz（模拟典型 GPS 更新率），由外部定时器控制。

import { CarState, GPSReading } from '../shared/types.js';
import { Px, px } from '../shared/dimensions.js';

export interface LocalizationConfig {
  /** 高斯噪声标准差（px），默认 5px（在 50px 宽的赛道上可见但不致命） */
  noiseStdDev: number;
}

const DEFAULT_CONFIG: LocalizationConfig = { noiseStdDev: 5 };

/**
 * Box-Muller 变换生成标准正态分布随机数
 *
 * 取两个 [0,1) 均匀随机数 u1, u2，
 * 返回 sqrt(-2*ln(u1)) * cos(2π*u2) ~ N(0, 1)
 */
function boxMuller(): number {
  let u1: number, u2: number;
  do { u1 = Math.random(); } while (u1 === 0); // ln(0) = -Infinity
  u2 = Math.random();
  return Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2);
}

/**
 * 模拟一次 GPS 定位
 *
 * @param car    车辆真实状态
 * @param config 噪声配置
 * @returns GPSReading（带噪声的位置 + 不确定度估计）
 */
export function localize(car: CarState, config?: Partial<LocalizationConfig>): GPSReading {
  const cfg = { ...DEFAULT_CONFIG, ...config };
  const sigma = cfg.noiseStdDev;

  const noiseX = boxMuller() * sigma;
  const noiseY = boxMuller() * sigma;

  return {
    x: px((car.x as number) + noiseX),
    y: px((car.y as number) + noiseY),
    uncertainty: px(2 * sigma), // 2σ ≈ 95% 置信区间
    timestamp: performance.now(),
  };
}
