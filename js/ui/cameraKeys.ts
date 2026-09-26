// ==================== UI：WASD 相机平移（纯计算） ====================
//
// W/S 沿"相机→目标"的水平投影前后移动，A/D 左右平移；速度 = 相机距离 × 系数
// （拉远时每帧走得更远 → 屏幕上的视觉速度一致）。只平移相机与目标点，不改朝向/缩放。
// 俯视正下方（水平投影退化）时给一个默认前进方向，避免除零。

import { m } from '../shared/dimensions.js';

/** 速度系数（1/s）：相机距离 × 该值 */
export const WASD_SPEED_FACTOR = 1.2;

export type Vec3 = [number, number, number];

/** 本帧相机平移量（世界坐标增量，y 恒为 0 = 地平面内平移） */
export function panDelta(
  o: { cam: Vec3; target: Vec3 }, f: number, r: number, dt: number,
): Vec3 {
  if (f === 0 && r === 0) return [0, 0, 0];
  const dx = o.target[0] - o.cam[0], dz = o.target[2] - o.cam[2];
  const dist = Math.hypot(dx, o.target[1] - o.cam[1], dz);
  const speed = Math.max(dist, m(5) as number) * WASD_SPEED_FACTOR;
  let fx = dx, fz = dz;
  const fl = Math.hypot(fx, fz);
  if (fl < 1e-6) { fx = 1; fz = 0; } else { fx /= fl; fz /= fl; }
  const rx = -fz, rz = fx;                       // right = forward × up（右手系，up = +y）
  const k = speed * dt;
  return [(fx * f + rx * r) * k, 0, (fz * f + rz * r) * k];
}
