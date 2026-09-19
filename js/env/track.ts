// ==================== 环境：赛道生成 ====================
//
// 【职责】生成闭合环形赛道的几何数据（中心线 + 内外边界）
//
// 【算法】
// 1. 随机生成 8 个控制点（环状分布）
// 2. Catmull-Rom 样条插值 → 平滑中心线（200 个采样点）
// 3. 计算每个采样点的法向量，偏移 ±TRACK_HALF_WIDTH → 内外边界
//
// 输入：画布宽高
// 输出：TrackGeometry { centerPoints, innerBoundary, outerBoundary }

import { TrackGeometry, Point2D } from '../shared/types.js';
import { Px, m, px } from '../shared/dimensions.js';
import { Rng } from '../shared/rng.js';

/** 赛道半宽 7.5m ≈ 2 车道 */
export const TRACK_HALF_WIDTH: Px = m(7.5);

interface Vec2 { x: number; y: number; }

/**
 * Catmull-Rom 样条插值
 *
 * 给定 4 个控制点 p0-p3 和参数 t ∈ [0,1]，
 * 返回 p1 到 p2 之间 t 位置的平滑插值点。
 * Catmull-Rom 保证 C¹ 连续性（一阶导数连续），适合赛道生成。
 */
function catmullRom(p0: Vec2, p1: Vec2, p2: Vec2, p3: Vec2, t: number): Vec2 {
  const t2 = t * t, t3 = t2 * t;
  return {
    x: 0.5 * ((2 * p1.x) + (-p0.x + p2.x) * t + (2 * p0.x - 5 * p1.x + 4 * p2.x - p3.x) * t2 + (-p0.x + 3 * p1.x - 3 * p2.x + p3.x) * t3),
    y: 0.5 * ((2 * p1.y) + (-p0.y + p2.y) * t + (2 * p0.y - 5 * p1.y + 4 * p2.y - p3.y) * t2 + (-p0.y + 3 * p1.y - 3 * p2.y + p3.y) * t3),
  };
}

/**
 * 生成随机环形赛道
 *
 * @param width  画布宽度
 * @param height 画布高度
 * @returns TrackGeometry
 */
export function generateTrack(width: number, height: number, controlRadii?: number[], rng: Rng = Math.random): TrackGeometry {
  const cx = width / 2, cy = height / 2;

  // 1. 生成 8 个环状控制点（预设半径或随机）
  const numControls = 8;
  const controls: Vec2[] = [];
  for (let i = 0; i < numControls; i++) {
    const angle = (i / numControls) * Math.PI * 2;
    const radius = controlRadii ? controlRadii[i] : (m(60) as number + rng() * m(20) as number);
    controls.push({ x: cx + Math.cos(angle) * radius, y: cy + Math.sin(angle) * radius });
  }

  // 2. Catmull-Rom 插值：8 段 × 每段 25 点 = 200 点
  const samplesPerSegment = 25;
  const centerRaw: Vec2[] = [];
  for (let i = 0; i < numControls; i++) {
    const p0 = controls[(i - 1 + numControls) % numControls];
    const p1 = controls[i];
    const p2 = controls[(i + 1) % numControls];
    const p3 = controls[(i + 2) % numControls];
    for (let j = 0; j < samplesPerSegment; j++) {
      centerRaw.push(catmullRom(p0, p1, p2, p3, j / samplesPerSegment));
    }
  }
  centerRaw.push(centerRaw[0]);  // 闭合：首尾相连

  const centerPoints: Point2D[] = centerRaw.map(p => ({ x: px(p.x), y: px(p.y) }));

  // 3. 生成内外边界：每个中心点偏移 ±TRACK_HALF_WIDTH 沿法向量方向
  const halfW = TRACK_HALF_WIDTH as number;
  const innerBoundary: Point2D[] = [];
  const outerBoundary: Point2D[] = [];
  for (let i = 0; i < centerRaw.length - 1; i++) {
    const p = centerRaw[i], next = centerRaw[i + 1];
    const dx = next.x - p.x, dy = next.y - p.y;
    const len = Math.hypot(dx, dy) || 1;
    // 法向量：切线逆时针旋转 90°，然后确保指向赛道中心
    let nx = -dy / len, ny = dx / len;
    const toCenterX = cx - p.x, toCenterY = cy - p.y;
    if (nx * toCenterX + ny * toCenterY < 0) { nx = -nx; ny = -ny; }
    innerBoundary.push({ x: px(p.x + nx * halfW), y: px(p.y + ny * halfW) });
    outerBoundary.push({ x: px(p.x - nx * halfW), y: px(p.y - ny * halfW) });
  }
  innerBoundary.push(innerBoundary[0]);  // 闭合
  outerBoundary.push(outerBoundary[0]);

  return { centerPoints, innerBoundary, outerBoundary };
}
