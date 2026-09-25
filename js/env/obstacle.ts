// ==================== 环境：障碍物生成 ====================
//
// 【职责】在赛道上随机生成静态障碍物
//
// 【生成策略】（2026-09-19 现实化，见 ISSUE_HISTORY 遗留问题 #4）
// 1. 从赛道中心线随机采样位置，横向偏置 ±3m（不再恒定压在中心线上）
// 2. 尺寸按车辆量级：宽 2-3.5m、长 3-5m（默认参数，仿真单位 = px）
// 3. 朝向对齐赛道切线 ± 0.3rad 抖动（模拟沿路停放的车辆/杂物，
//    旧版完全随机朝向会以对角跨占车道）
//
// 输入：TrackGeometry + 数量 + 随机源
// 输出：Obstacle[]

import { TrackGeometry, Obstacle, obstacleToSegments } from '../shared/types.js';
import { Radians, Px, rad, px, m } from '../shared/dimensions.js';
import { Rng } from '../shared/rng.js';

export { obstacleToSegments };

/** 横向偏置最大值（±3m） */
const MAX_LATERAL_OFFSET: Px = m(3);
/** 朝向相对切线抖动最大值（±0.3 rad） */
const MAX_YAW_JITTER = 0.3;

/**
 * 在赛道上生成随机障碍物
 *
 * @param track     赛道几何数据
 * @param count     目标数量
 * @param minSize   最小宽度（px，默认 8 = 2m）
 * @param maxSize   最大宽度（px，默认 14 = 3.5m）
 * @param rng       随机源（缺省 Math.random；headless 传入可播种 RNG 以复现）
 * @param clearance 出生区净空（有路线时传入）：障碍须距 fromIndex 沿弧 ≥ minGap。
 *                  随机障碍曾落在出生点上（oval 种子 7 重叠 3m）——车一出生即
 *                  接触、与速度无关、全程碰撞不可避（实测 3600 帧）。
 * @returns Obstacle[]
 */
export function generateObstacles(
  track: TrackGeometry, count: number,
  minSize = 8, maxSize = 14, rng: Rng = Math.random,
  clearance?: { fromIndex: number; minGap: number },
): Obstacle[] {
  const obstacles: Obstacle[] = [];
  const n = track.centerPoints.length - 1;

  // 累计弧长表（环形），供出生区净空判定（精确沿弧距离，O(1)/次）
  let cum: number[] | null = null;
  if (clearance) {
    cum = [0];
    for (let i = 0; i < n; i++) {
      const a = track.centerPoints[i], b = track.centerPoints[(i + 1) % n];
      cum.push(cum[i] + Math.hypot((b.x as number) - (a.x as number), (b.y as number) - (a.y as number)));
    }
  }
  const tooCloseToStart = (idx: number): boolean => {
    if (!cum || !clearance) return false;
    const d = Math.abs(cum[idx] - cum[clearance.fromIndex]);
    return Math.min(d, cum[n] - d) < clearance.minGap;
  };

  // Fisher–Yates 洗牌 → 随机不重复采样中心线位置
  const indices = Array.from({ length: n }, (_, i) => i);
  for (let i = indices.length - 1; i > 0; i--) {
    const j = Math.floor(rng() * (i + 1));
    [indices[i], indices[j]] = [indices[j], indices[i]];
  }

  for (const idx of indices) {
    if (obstacles.length >= count) break;
    if (tooCloseToStart(idx)) continue;   // 出生区净空：跳过起点附近的采样位
    const pt = track.centerPoints[idx];
    const next = track.centerPoints[(idx + 1) % n];
    const dx = (next.x as number) - (pt.x as number);
    const dy = (next.y as number) - (pt.y as number);
    const len = Math.hypot(dx, dy) || 1;

    const w = px(minSize + rng() * (maxSize - minSize));  // 宽 2-3.5m
    const h = px(12 + rng() * 8);                          // 长 3-5m
    const zH = 5 + rng() * 12;                             // 3D 高度（仅渲染使用）
    const offset = (rng() * 2 - 1) * (MAX_LATERAL_OFFSET as number);
    const nx = -dy / len, ny = dx / len;                   // 赛道法向
    const angle = rad(Math.atan2(dy, dx) + (rng() * 2 - 1) * MAX_YAW_JITTER);

    obstacles.push({
      x: px((pt.x as number) + nx * offset),
      y: px((pt.y as number) + ny * offset),
      width: w, height: h, zHeight: zH, angle,
    });
  }

  return obstacles;
}
