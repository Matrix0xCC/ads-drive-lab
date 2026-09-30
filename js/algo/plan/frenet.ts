// ==================== Frenet 坐标系（S-L 框架）+ 碰撞检测 ====================
//
// 【什么是 Frenet 坐标系？】
// Frenet 坐标系是自动驾驶中最常用的轨迹规划坐标系。
// 它用两个坐标描述车辆位置：
//   s = 沿参考线（通常为道路中心线）跑了多远（纵向）
//   l = 离参考线偏左/偏右多少（横向）
//
// 【为什么用 Frenet？】
// 在笛卡尔坐标系中，"变道避障"是复杂的 2D 曲线。
// 在 Frenet 中，它只是 L 值从 A 平滑过渡到 B——降维简化了轨迹优化。
// 这是 Apollo、Waymo 等 L4 系统的标准做法。
//
// 【法向量方向约定】
// 法向量指向赛道内侧（track.js 已在生成时保证）。
// l > 0 = 向内侧偏移，l < 0 = 向外侧偏移。

import {
  Radians, Px,
  rad, px,
} from '../../shared/dimensions.js';
import { Point2D, FrenetReference } from '../../shared/types/interfaces.js';

// ═══════════════════════════════════════════════════
// Frenet 参考线构建
// ═══════════════════════════════════════════════════

/**
 * 从赛道中心线构建 Frenet 参考线
 *
 * 【构建流程】
 * 1. 计算每个采样点的累计弧长 s（用于纵向定位）
 * 2. 计算每个采样点的切向量（前进方向）
 * 3. 计算每个采样点的法向量（切向量逆时针旋转 90°，指向赛道内侧）
 *
 * 【复杂度】O(n)，n = 中心线采样点数（约 200）
 *
 * @param centerPoints 赛道中心线采样点
 * @param closedLoop   是否闭环（默认 true，路线用 false）
 * @returns FrenetReference
 */
export function buildFrenetReference(centerPoints: Point2D[], closedLoop: boolean = true): FrenetReference {
  const n = centerPoints.length;
  const s: number[] = new Array(n).fill(0);
  const tangents: { x: number; y: number }[] = new Array(n);
  const normals: { x: number; y: number }[] = new Array(n);

  // 累计弧长：s[i] = s[i-1] + dist(points[i-1], points[i])
  for (let i = 1; i < n; i++) {
    const dx = (centerPoints[i].x as number) - (centerPoints[i - 1].x as number);
    const dy = (centerPoints[i].y as number) - (centerPoints[i - 1].y as number);
    s[i] = s[i - 1] + Math.hypot(dx, dy);
  }

  // 切向量 = 前进方向的单位向量
  // 法向量 = 切向量逆时针旋转 90°
  for (let i = 0; i < n - 1; i++) {
    const dx = (centerPoints[i + 1].x as number) - (centerPoints[i].x as number);
    const dy = (centerPoints[i + 1].y as number) - (centerPoints[i].y as number);
    const len = Math.hypot(dx, dy) || 1;
    tangents[i] = { x: dx / len, y: dy / len };
    normals[i] = { x: -dy / len, y: dx / len };
  }
  // 最后一个点：闭环复用第一个切向量，开环复用前一个
  tangents[n - 1] = closedLoop ? (tangents[0] || tangents[n - 2] || { x: 1, y: 0 }) : (tangents[n - 2] || { x: 1, y: 0 });
  normals[n - 1] = closedLoop ? (normals[0] || normals[n - 2] || { x: 0, y: 1 }) : (normals[n - 2] || { x: 0, y: 1 });

  return { points: centerPoints, s, tangents, normals, totalS: s[n - 1], closedLoop };
}

// ═══════════════════════════════════════════════════
// Cartesian ↔ Frenet 互转
// ═══════════════════════════════════════════════════

/**
 * 笛卡尔坐标 → Frenet 坐标
 *
 * 【算法：最近邻投影】
 * 1. 在参考线上找到离 (x,y) 最近的采样点（最近邻搜索）
 * 2. s = 该点对应的累计弧长
 * 3. l = 向量 (x,y)-ref_point 在法向量方向上的投影（点积）
 *    l > 0 表示在参考线内侧，l < 0 在外侧
 *
 * 【注意】
 * 最近邻搜索对于远离参考线的点会有近似误差。理想做法是
 * 迭代求解垂直投影（Newton-Raphson），但对赛道宽度（±50px）
 * 来说，最近邻的精度足够。
 *
 * @returns { s: 弧长(number), l: 横向偏移(Px), idx: 最近点索引 }
 */
export function cartesianToFrenet(x: Px, y: Px, ref: FrenetReference): { s: number; l: Px; idx: number } {
  const nx = x as number;
  const ny = y as number;
  const n = ref.points.length;

  // 【逐**线段**投影 + 垂距】——这是本模块唯一的正确口径。
  //
  // 【踩过的坑（2026-09-27 幽灵刹车的根因）】旧实现先找**最近的顶点**，再把偏移投到**该顶点的法向量**上。
  // 两个误差来源：① 最近的顶点未必在最近的位置上（Δs 可达点距的一半，G4 图重采样后 ~2.5m）；
  // ② 用顶点法向量代替该处的切向法向量，弯道上进一步偏。
  // 实测：68m 外的命中点横向偏差可达 **~1.2m** ⇒ 相邻车道的车被投影进我这条车道 ⇒ 物理走廊判
  // 我这条车道过不去 ⇒ 提前急刹（用户实测的"幽灵刹车"）。近距离（12m）误差也有 0.2~0.7m。
  //
  // 【口径】与 frenetToCartesian 严格互逆：
  //   t = clamp((p−a)·(b−a)/|b−a|², 0, 1)；q = a + t(b−a)；s = s_i + t·|b−a|
  //   l = (p−q)·n(t)，其中 n(t) = normalize(lerp(n_i, n_{i+1}, t))（与反变换同一插值）
  // ⇒ 往返误差只剩"线段对曲线的线性化"（几毫米级，见 frenet-project.test.ts）。
  let bestD2 = Infinity, bestI = 0, bestT = 0;
  const segs = ref.closedLoop ? n : n - 1;              // 闭环多一段"末点 → 首点"
  for (let i = 0; i < segs; i++) {
    const j = (i + 1) % n;
    const ax = ref.points[i].x as number, ay = ref.points[i].y as number;
    const bx = ref.points[j].x as number, by = ref.points[j].y as number;
    const ex = bx - ax, ey = by - ay;
    const len2 = ex * ex + ey * ey;
    const t = len2 > 0 ? Math.max(0, Math.min(1, ((nx - ax) * ex + (ny - ay) * ey) / len2)) : 0;
    const qx = ax + t * ex, qy = ay + t * ey;
    const d2 = (nx - qx) * (nx - qx) + (ny - qy) * (ny - qy);
    if (d2 < bestD2) { bestD2 = d2; bestI = i; bestT = t; }
  }

  const i = bestI, j = (i + 1) % n;
  const segLen = (ref.s[j] as number) - (ref.s[i] as number) || (ref.s[1] as number) - (ref.s[0] as number);
  let sAt = (ref.s[i] as number) + bestT * segLen;
  if (i === n - 1) sAt = (ref.s[n - 1] as number) + bestT * ((ref.totalS as number) - (ref.s[n - 1] as number));
  if (ref.closedLoop && sAt >= (ref.totalS as number)) sAt -= ref.totalS as number;   // 闭环绕接

  const rx = (ref.points[i].x as number) + bestT * ((ref.points[j].x as number) - (ref.points[i].x as number));
  const ry = (ref.points[i].y as number) + bestT * ((ref.points[j].y as number) - (ref.points[i].y as number));
  const nxi = ref.normals[i].x + bestT * (ref.normals[j].x - ref.normals[i].x);
  const nyi = ref.normals[i].y + bestT * (ref.normals[j].y - ref.normals[i].y);
  const nLen = Math.hypot(nxi, nyi) || 1;
  const lv = ((nx - rx) * nxi + (ny - ry) * nyi) / nLen;      // 与反变换同一法向约定

  // idx 语义保持与原实现一致：给"该点所在参考线段起点"的序号（供调用方做局部搜索/可视化）
  return { s: sAt, l: px(lv), idx: Math.min(i, n - 2) };
}

/**
 * Frenet 坐标 → 笛卡尔坐标
 *
 * 【算法】
 * 1. s 范围处理：闭环取模，开环 clamp（由 ref.closedLoop 决定）
 * 2. 二分查找 s 所在的参考线段
 * 3. 在该线段上线性插值参考点、法向量、切向量
 * 4. 笛卡尔点 = ref_point + l × normal（法向量方向偏移）
 * 5. yaw = 插值后的切向量方向
 */
export function frenetToCartesian(s: number, lv: Px, ref: FrenetReference): { x: Px; y: Px; yaw: Radians } {
  // s 范围：闭环取模，开环 clamp
  let sMod = ref.closedLoop
    ? ((s % ref.totalS) + ref.totalS) % ref.totalS
    : Math.max(0, Math.min(s, ref.totalS));

  // 二分查找 s 所在的线段
  let i = 0;
  const n = ref.s.length;
  while (i < n - 1 && ref.s[i + 1] <= sMod) i++;
  if (i >= n - 1) i = n - 2; // sMod=totalS 时边界保护

  const segLen = ref.s[i + 1] - ref.s[i] || 1;
  const t = (sMod - ref.s[i]) / segLen;

  // 线性插值参考点
  const rx = (ref.points[i].x as number) + t * ((ref.points[i + 1].x as number) - (ref.points[i].x as number));
  const ry = (ref.points[i].y as number) + t * ((ref.points[i + 1].y as number) - (ref.points[i].y as number));
  // 线性插值法向量（需要重新归一化）
  const nxi = ref.normals[i].x + t * (ref.normals[i + 1].x - ref.normals[i].x);
  const nyi = ref.normals[i].y + t * (ref.normals[i + 1].y - ref.normals[i].y);
  const nLen = Math.hypot(nxi, nyi) || 1;
  // 线性插值切向量
  const txi = ref.tangents[i].x + t * (ref.tangents[i + 1].x - ref.tangents[i].x);
  const tyi = ref.tangents[i].y + t * (ref.tangents[i + 1].y - ref.tangents[i].y);
  const l = lv as number;

  return {
    x: px(rx + l * nxi / nLen),
    y: px(ry + l * nyi / nLen),
    yaw: rad(Math.atan2(tyi, txi)),
  };
}

