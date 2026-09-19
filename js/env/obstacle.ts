// ==================== 环境：障碍物生成 ====================
//
// 【职责】在赛道上随机生成静态矩形障碍物
//
// 【生成策略】
// 1. 从赛道中心线随机采样位置
// 2. 随机尺寸（宽度 10~25px，高度 6~16px）
// 3. 随机朝向角
//
// 输入：TrackGeometry + 数量
// 输出：Obstacle[]

import { TrackGeometry, Obstacle, obstacleToSegments } from '../shared/types.js';
import { Radians, Px, rad, px } from '../shared/dimensions.js';
import { Rng } from '../shared/rng.js';

export { obstacleToSegments };

/**
 * 在赛道上生成随机障碍物
 *
 * @param track   赛道几何数据
 * @param count   目标数量
 * @param minSize 最小宽度（px）
 * @param maxSize 最大宽度（px）
 * @param rng     随机源（缺省 Math.random；headless 传入可播种 RNG 以复现）
 * @returns Obstacle[]
 */
export function generateObstacles(
  track: TrackGeometry, count: number,
  minSize = 10, maxSize = 25, rng: Rng = Math.random,
): Obstacle[] {
  const obstacles: Obstacle[] = [];
  const n = track.centerPoints.length - 1;

  // Fisher–Yates 洗牌 → 随机不重复采样中心线位置
  const indices = Array.from({ length: n }, (_, i) => i);
  for (let i = indices.length - 1; i > 0; i--) {
    const j = Math.floor(rng() * (i + 1));
    [indices[i], indices[j]] = [indices[j], indices[i]];
  }

  for (const idx of indices) {
    if (obstacles.length >= count) break;
    const pt = track.centerPoints[idx];
    const w = px(minSize + rng() * (maxSize - minSize));
    const h = px(6 + rng() * 10);
    const zH = 5 + rng() * 12;  // 3D 高度（仅渲染使用）
    const angle = rad(rng() * Math.PI * 2);
    obstacles.push({ x: pt.x, y: pt.y, width: w, height: h, zHeight: zH, angle });
  }

  return obstacles;
}
