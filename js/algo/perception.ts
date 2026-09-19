// ==================== 算法：感知模块（Perception） ====================
//
// 【职责】接收 WorldSnapshot，执行 LiDAR 扫描 + HD Map 背景减除，输出 LidarScan
//
// 【数据流】
//   WorldSnapshot → 收集线段（赛道边界 + 障碍物）→ 61 条射线求交 →
//   HD Map 背景减除（Frenet |l| 分类）→ LidarScan { distances, rawDistances, confidences }
//
// 【LiDAR 参数】
//   单线 2D 扫描仪（对标 SICK LMS 系列工业激光雷达）
//   - 水平 FOV：±60°（120° 总视场）
//   - 角分辨率：2° → 约 61 条射线
//   - 最大探测距离：125m（覆盖 140 m/s × 2s = 280m 轨迹 + 余量，比例 4px/m）
//   - 更新频率：20 Hz（外部定时器控制）
//
// 【背景减除（HD Map Background Subtraction）】
//   核心设计决策：只用传感器原始数据 + HD Map（赛道边界位置），不使用障碍物列表。
//   对标 Waymo/Apollo 的 map-based point cloud filtering。
//
//   每条 LiDAR 射线的处理管线：
//     1. 射线-线段求交 → 原始距离 rawDist
//     2. 命中点 → 世界坐标 → Frenet 投影 → |l|
//     3. |l| 三级分类：
//        |l| < 4.5m   → confidence=1（高置信障碍物：命中点在赛道中间 60% 区域）
//        4.5m ≤ |l| < 6.0m → confidence=0.5（边缘模糊：障碍物靠近路缘或 Frenet 误差）
//        |l| ≥ 6.0m   → confidence=0（边界：半宽的 80%，1.5m 缓冲吸收 Frenet 投影误差）
//     4. confidence=0 → filteredDist = MAX_SENSOR_DIST（规划层看不到）
//        confidence=0.5/1 → filteredDist = rawDist（规划层保留）
//
//   【为什么不查障碍物列表做交叉验证？】
//   那等价于上帝视角——规划层直接知道障碍物位置。真实系统中感知模块
//   只有传感器数据，无法区分"围墙边的障碍物"和"围墙本身"。
//   2D 单帧 LiDAR 在这个问题上存在物理极限——我们用置信度接受这个模糊性。
//
//   规划层 `collision.ts` 对低置信命中使用更大的安全余量（+10px）。
//
//   【为什么用闭环 Frenet 参考线？】
//   感知层的背景减除使用赛道闭环中心线（track.centerPoints）。
//   这与规划器在 route 模式下使用的开环参考线（route.waypoints）不同。
//   背景减除判断"这是围墙吗"用的是全局赛道几何——围墙在整个环道上存在。
//
// 【输出给规划层的数据】
//   LidarScan {
//     angles:      每条射线的世界绝对角度（弧度）
//     distances:   过滤后距离（规划层使用，边界命中已替换为 MAX）
//     rawDistances: 过滤前距离（渲染层使用，显示真实射线截止位置）
//     confidences:  置信度标记（0=边界 0.5=低置信 1=高置信）
//   }
//
// 【复杂度】
//   每次扫描：61 条射线 × (赛道边界约 400 段 + 障碍物约 20 段) ≈ 25,000 次求交
//   20 Hz 运行 = 每秒 50 万次求交。每次求交只有几十次浮点运算，
//   对现代浏览器 < 1ms。

import { WorldSnapshot, LidarScan, TrackGeometry, createLidarScan, obstacleToSegments, FrenetReference } from '../shared/types.js';
import { Radians, Px, m, rad, px, normalizeAngle } from '../shared/dimensions.js';
import { buildFrenetReference, cartesianToFrenet } from './plan/frenet.js';
import { debug } from '../shared/debug.js';

// ═══════════════════════════════════════════════════
// LiDAR 硬件参数
// ═══════════════════════════════════════════════════

/** 水平视场角：±60°（共 120°），对标 SICK LMS 系列 */
const SCAN_FOV = Math.PI * 2 / 3;

/** 角分辨率：2°（≈ 0.0349 rad），每条射线间距约 12px @ 350px 距离 */
const SCAN_RESOLUTION = 2 * Math.PI / 180;

/** 最大探测距离 125m */
export const MAX_SENSOR_DIST: Px = m(125);

/** 射线总数 ≈ 61 */
export const SCAN_RAY_COUNT = Math.floor(SCAN_FOV / SCAN_RESOLUTION) + 1;

// ═══════════════════════════════════════════════════
// 背景减除阈值
// ═══════════════════════════════════════════════════
//
// 赛道半宽 7.5m（TRACK_HALF_WIDTH）。围墙位置在 |l| ≈ 7.5m。
// 考虑 Frenet 投影误差（最近邻搜索 ±0.5m）和赛道曲率影响，
// LOW 阈值设在半宽 80% 处（6.0m），留出 1.5m 缓冲吸收误差。

/** |l| < 4.5m：明确在赛道内部（半宽的 60%）→ 高置信障碍物 */
const CONFIDENCE_HIGH = (m(4.5) as number);

/** |l| > 6.0m：明确在围墙附近（半宽的 80%，给 Frenet 误差留 1.5m 缓冲）→ 确认是边界 */
const CONFIDENCE_LOW = (m(6.0) as number);

// ═══════════════════════════════════════════════════
// 线段类型
// ═══════════════════════════════════════════════════

interface Segment { x1: number; y1: number; x2: number; y2: number; }

// ═══════════════════════════════════════════════════
// 射线-线段求交（2D 参数方程法 / Cramer's Rule）
// ═══════════════════════════════════════════════════
//
// 【算法】
//   射线：P = O + t·D  (t ≥ 0)
//   线段：P = A + u·(B-A)  (0 ≤ u ≤ 1)
//   联立解 2×2 线性方程组：
//     t = det([A-O, segDir]) / det(rayDir, segDir)
//     u = det([A-O, rayDir]) / det(rayDir, segDir)
//   行列式 = 0 → 平行 → 无交点
//   t > 0 且 0 ≤ u ≤ 1 → 有效交点，返回 t（射线原点到交点的距离）
//
// 【为什么用参数方程而非几何法？】
//   代码紧凑，易于扩展到 3D，计算量相同（都是 2 次叉积 + 1 次除法）。

/**
 * 2D 射线-线段求交
 *
 * @param ox, oy  射线原点（车辆位置）
 * @param dx, dy  射线单位方向向量
 * @param x1-x2   线段端点
 * @returns 交点到射线原点的距离 t，或 null（无有效交点）
 */
function raySegmentIntersect(
  ox: number, oy: number, dx: number, dy: number,
  x1: number, y1: number, x2: number, y2: number,
): number | null {
  const segX = x2 - x1, segY = y2 - y1;            // 线段方向向量
  const denom = dx * segY - dy * segX;              // 行列式 = rayDir × segDir
  if (Math.abs(denom) < 1e-10) return null;        // 平行或共线 → 无交点
  const t = ((x1 - ox) * segY - (y1 - oy) * segX) / denom;  // Cramer's Rule: t
  const u = ((x1 - ox) * dy - (y1 - oy) * dx) / denom;      // Cramer's Rule: u
  if (t > 0 && u >= 0 && u <= 1) return t;          // 交点在射线前方且在线段上
  return null;
}

// ═══════════════════════════════════════════════════
// HD Map 背景参考线（懒缓存）
// ═══════════════════════════════════════════════════

/**
 * 背景减除用的 Frenet 参考线。
 * 赛道不变则只构建一次（`buildFrenetReference` O(n) ≈ 200 点）。
 * 始终使用闭环参考线——围墙在整个环道上存在。
 */
let _bgFrenetRef: FrenetReference | null = null;

function getBackgroundFrenetRef(track: TrackGeometry): FrenetReference {
  if (!_bgFrenetRef) _bgFrenetRef = buildFrenetReference(track.centerPoints, true);
  return _bgFrenetRef;
}

// ═══════════════════════════════════════════════════
// 命中点 Frenet 投影
// ═══════════════════════════════════════════════════

/**
 * 计算 LiDAR 射线命中点在 Frenet 坐标系中的 |l| 值
 *
 * 命中点世界坐标 = carPos + rayDir × dist
 * → cartesianToFrenet → 返回 |l|（横向偏移绝对值）
 *
 * 如果射线未命中（dist ≈ MAX），返回 null。
 * 只用传感器原始数据 + HD Map 参考线，不查障碍物列表。
 */
function hitFrenetAbsL(
  carX: number, carY: number,
  rayAngle: number, dist: number,
  ref: FrenetReference,
): number | null {
  const maxDist = MAX_SENSOR_DIST as number;
  if (dist >= maxDist - 1) return null;  // 未命中 → 无投影

  const hitX = carX + Math.cos(rayAngle) * dist;
  const hitY = carY + Math.sin(rayAngle) * dist;
  const fl = cartesianToFrenet(px(hitX), px(hitY), ref);
  return Math.abs(fl.l as number);
}

// ═══════════════════════════════════════════════════
// 主扫描
// ═══════════════════════════════════════════════════

/**
 * 执行一次完整的 LiDAR 扫描 + 背景减除
 *
 * 【管线（每条射线）】
 *   1. 对全部线段（赛道边界 + 障碍物）做射线求交 → 最近命中距离 minDist
 *   2. rawDistances.push(minDist) — 保留原始值给渲染层
 *   3. 对命中点做 Frenet 投影 → |l|
 *   4. 三级分类 → confidence ∈ {0, 0.5, 1}
 *   5. confidence=0 → distances.push(MAX)（过滤），否则 push(minDist)
 *
 * 【输出】
 *   - distances:   过滤后（规划层），只含障碍物命中
 *   - rawDistances: 过滤前（渲染层），含边界命中
 *   - confidences:  每条射线的置信度
 */
export function sense(snapshot: WorldSnapshot): LidarScan {
  const { car, track, obstacles } = snapshot;
  const angles: Radians[] = [], distances: Px[] = [], rawDistances: Px[] = [], confidences: number[] = [];

  // ── 1. 收集所有待检测线段 ──
  const allSegments: Segment[] = [];
  // 赛道边界（内外各约 200 段 Catmull-Rom 逼近线段）
  for (const boundary of [track.innerBoundary, track.outerBoundary]) {
    for (let i = 0; i < boundary.length - 1; i++) {
      allSegments.push({
        x1: boundary[i].x as number, y1: boundary[i].y as number,
        x2: boundary[i + 1].x as number, y2: boundary[i + 1].y as number,
      });
    }
  }
  // 障碍物矩形 → 4 条线段/个
  for (const obs of obstacles) allSegments.push(...obstacleToSegments(obs));

  // ── 2. HD Map 参考线（懒缓存）──
  const bgRef = getBackgroundFrenetRef(track);

  // ── 3. 扫描主循环 ──
  const carX = car.x as number, carY = car.y as number;
  const carAngle = car.angle as number;
  const maxDist = MAX_SENSOR_DIST as number;

  for (let offset = -SCAN_FOV / 2; offset <= SCAN_FOV / 2; offset += SCAN_RESOLUTION) {
    // 射线世界方向 = 车头朝向 + 偏移角
    const worldAngle = carAngle + offset;
    const dx = Math.cos(worldAngle), dy = Math.sin(worldAngle);

    // 对全部线段求交，取最近距离
    let minDist = maxDist;
    for (const seg of allSegments) {
      const t = raySegmentIntersect(carX, carY, dx, dy, seg.x1, seg.y1, seg.x2, seg.y2);
      if (t !== null && t < minDist) minDist = t;
    }

    // 原始距离（渲染层使用）
    rawDistances.push(px(minDist));

    // ── 4. 背景减除 ──
    let confidence = 1;           // 默认：无命中 → 空旷 → 高置信
    let filteredDist = minDist;

    if (minDist < maxDist) {
      // 有命中 → Frenet 投影判断
      const absL = hitFrenetAbsL(carX, carY, worldAngle, minDist, bgRef);
      if (absL !== null) {
        if (absL > CONFIDENCE_LOW) {
          // |l| > 50：命中点落在已知围墙上 → 背景
          filteredDist = maxDist;
          confidence = 0;
        } else if (absL > CONFIDENCE_HIGH) {
          // 35 < |l| < 50：边缘模糊区 → 低置信（可能是障碍物靠近路缘）
          confidence = 0.5;
        }
        // |l| < 35：明确在赛道内 → 高置信障碍物（confidence=1 不变）
      }
    }

    confidences.push(confidence);
    angles.push(normalizeAngle(rad(worldAngle)));   // 归一化到 [-π,π]，下游无需再处理环绕
    distances.push(px(filteredDist));
  }

  // ── 5. Debug 信息 ──
  const mid = Math.floor(distances.length / 2);
  let minD = maxDist, rawHits = 0;
  for (let i = 0; i < distances.length; i++) {
    if ((distances[i] as number) < minD) minD = distances[i] as number;
    if ((rawDistances[i] as number) < maxDist - 1) rawHits++;
  }
  debug.perception = {
    rayCount: distances.length,
    frontDist: distances[mid] as number,
    minDist: minD,
    minDistAngle: 0,
    rawHits,
    filteredHits: confidences.filter(c => c > 0).length,
  };

  return createLidarScan(angles, distances, rawDistances, confidences);
}
