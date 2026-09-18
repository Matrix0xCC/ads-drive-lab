// ==================== LiDAR 碰撞检测 ====================
//
// 【前置条件】
// perception.ts 已在扫描时完成 HD Map 背景减除。
// 传入的 LidarScan 中：
//   distances: 边界命中已替换为 MAX_SENSOR_DIST，障碍物命中保留原始距离
//   confidences: 0=边界 0.5=边缘模糊 1=高置信
// 本模块只需比较距离 + 置信度加权的安全余量。
//
// 【碰撞判定公式】
//   轨迹点 (px, py)，从车位置 (carX, carY) 距离 = ptDist
//   同方向 LiDAR 射线测量距离 = lidarDist
//   碰撞条件：lidarDist + safetyMargin + extraMargin < ptDist
//     即 LiDAR 看到的障碍物在轨迹点后方 → 轨迹点会撞上它
//
// 【锥形搜索】
//   不只是检查单条射线，而是检查最近射线 ±coneHalf 范围内的多条射线。
//   原因：障碍物有一定宽度，轨迹点方向可能与 LiDAR 射线方向不完全重合。
//   近距离用更宽锥形（障碍物在近距离时角度跨度大）。

import { LidarScan } from '../../shared/types.js';
import { Px, m, px } from '../../shared/dimensions.js';
import { MAX_SENSOR_DIST } from '../perception.js';

/** 基础安全余量：15px，补偿轨迹离散采样和 Frenet 投影误差 */
const SAFETY_MARGIN: Px = m(2.5);

/**
 * 检查轨迹点是否与障碍物冲突
 *
 * 【流程】
 *   1. 计算轨迹点相对车辆的方向和距离
 *   2. 在 LiDAR 扫描中找到该方向最近的射线
 *   3. 对最近射线 ±coneHalf 范围内的射线：
 *      a. 如果有命中（dist < MAX-1）→ 检查是否为边界过滤后的真实障碍物
 *      b. 低置信度命中 → 额外 +10px 安全余量（保守处理边缘模糊）
 *      c. 如果 lidarDist + margin < ptDist → 碰撞
 *
 * 【锥形参数】
 *   ptDist < 30  → coneHalf=6 (±12°，近距离障碍物角跨度大)
 *   30 ≤ ptDist < 80 → coneHalf=4 (±8°)
 *   ptDist ≥ 80 → coneHalf=2 (±4°，远距离障碍物角跨度小)
 *
 * @param px, py    轨迹点坐标
 * @param carX, carY 车辆位置
 * @param lidarScan  LiDAR 扫描（已背景减除）
 * @param margin     安全余量（SAFETY_MARGIN + collisionRadius）
 */
export function isLidarCollision(
  px: number, py: number,
  carX: number, carY: number,
  lidarScan: LidarScan,
  margin: number,
): boolean {
  const dx = px - carX, dy = py - carY;
  const ptDist = Math.hypot(dx, dy);
  if (ptDist < 2) return false;  // 车身边缘点，不检测

  const ptAngle = Math.atan2(dy, dx);
  const angles = lidarScan.angles;
  const distances = lidarScan.distances;
  const confidences = lidarScan.confidences;
  const maxDist = MAX_SENSOR_DIST as number;
  const coneHalf = ptDist < 30 ? 6 : ptDist < 80 ? 4 : 2;

  // 线性扫描找最近射线（61 条射线，O(61) 完全足够）
  let bestIdx = 0, bestDiff = Infinity;
  for (let i = 0; i < angles.length; i++) {
    let diff = Math.abs((angles[i] as number) - ptAngle);
    if (diff > Math.PI) diff = 2 * Math.PI - diff;  // 角度归一化
    if (diff < bestDiff) { bestDiff = diff; bestIdx = i; }
  }

  // 检查锥形范围内所有射线
  for (let offset = -coneHalf; offset <= coneHalf; offset++) {
    const idx = bestIdx + offset;
    if (idx < 0 || idx >= distances.length) continue;
    const lidarDist = distances[idx] as number;
    if (lidarDist >= maxDist - 1) continue;  // 无命中或已过滤 → 安全

    // 置信度调整：低置信命中可能是边界边缘，用更保守的余量
    const conf = confidences && idx < confidences.length ? confidences[idx] : 1;
    const extraMargin = conf < 1 ? 10 : 0;

    if (lidarDist + margin + extraMargin < ptDist) return true;
  }

  return false;
}

/**
 * 检查前方是否有需要避障的障碍物（用于状态机切换到 AVOID）
 *
 * 【与 isLidarCollision 的区别】
 * 此函数在候选轨迹评估之前运行，决定状态机是否切换到 AVOID。
 * 需要快速判断"前方是否值得避障"，而不是精确的逐点碰撞检测。
 *
 * 【参数选择】
 *   - 阈值 200px：≈ 巡航速度 180px/s × 2s 轨迹前半段。
 *     更远的障碍物不影响当前轨迹，不需要切换 AVOID。
 *   - 前方 ±10°（mid±5 条射线）：覆盖车道宽度
 *   - 低置信需要 ≥2 条确认：单条边缘模糊射线不足以触发避障
 *     （可能是边界 Frenet 投影的随机误差）
 */
export function detectObstacle(lidarScan: LidarScan): boolean {
  if (lidarScan.distances.length === 0) return false;
  const maxDist = MAX_SENSOR_DIST as number;
  const mid = Math.floor(lidarScan.distances.length / 2);
  const threshold = m(38) as number;  // 38m 前方威胁范围
  let lowConfHits = 0;

  for (let i = mid - 5; i <= mid + 5; i++) {
    if (i >= 0 && i < lidarScan.distances.length) {
      const d = lidarScan.distances[i] as number;
      if (d < maxDist - 1 && d < threshold) {
        const conf = lidarScan.confidences && i < lidarScan.confidences.length ? lidarScan.confidences[i] : 1;
        if (conf >= 1) return true;   // 高置信障碍物 → 立即触发 AVOID
        lowConfHits++;
      }
    }
  }
  return lowConfHits >= 4;  // 低置信需要 ≥4 条射线确认（11 条中 ≥4 = 连续障碍物）
}

export { SAFETY_MARGIN };
