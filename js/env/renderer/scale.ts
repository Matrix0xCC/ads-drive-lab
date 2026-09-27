// ==================== 渲染器：比例尺与量距的纯数学 ====================
//
// 【为什么单独成模块】单位与"取哪个方向"最容易错（本会话已在镜像/朝向上栽过几次），
// 抽成不依赖 DOM 的纯函数后可以用测试钉住：1px 对应多少米、整数值怎么取、刻度怎么摆。
//
// 【关键】比例尺用**屏幕水平方向**在**目标点深度**处的尺度：
// 透视相机下"1 屏幕像素对应多少世界单位"随距离变化，且**纵向**会被俯仰（透视压缩）扭曲
// （斜视时沿屏幕竖直方向量地面会越远越密）——所以地图类比例尺一律用横向 ✓ 横向不受俯仰影响。

import * as THREE from 'three';
import { SIM_UNITS_PER_METER } from '../../shared/dimensions.js';

/**
 * 屏幕 1px 在**目标点深度**处对应的地面长度（米）。
 *
 * @param viewH 视口高（像素）
 * @param unitsPerMeter 世界单位/米（默认 SIM_UNITS_PER_METER = 4）
 */
export function metersPerPixel(
  camera: THREE.PerspectiveCamera, target: THREE.Vector3, viewH: number,
  unitsPerMeter: number = SIM_UNITS_PER_METER,
): number {
  if (viewH <= 0 || unitsPerMeter <= 0) return Number.NaN;
  const d = camera.position.distanceTo(target);                       // 世界单位
  const worldPerPx = 2 * d * Math.tan((camera.fov * Math.PI / 180) / 2) / viewH;
  return worldPerPx / unitsPerMeter;
}

/**
 * **在指定地面点处**的横向尺度（米/像素）——比例尺**必须**用自己所在位置的尺度。
 *
 * 为什么不能用画面中心（目标点）的尺度：透视相机下横向尺度随**深度**变化，而斜视时
 * 屏幕底部的地面比中心近得多（实测 33.7° 取景下差 ~1.25 倍）。比例尺画在屏幕底部，
 * 若不按它自己的位置取尺度，画出来的条**实际覆盖距离与标注不符**（用户就是拿地面刻度线
 * 对照发现的：条长看着像 10m，标注却是 5m）。
 *
 * 做法：把相机右向量投到地面作为"屏幕水平方向"，取 `point + 1m·该方向` 投影到屏幕，
 * 量出像素差 → 精确得到该点的 m/px（对离轴点也正确，不像"欧氏距离"那样偏大）。
 */
export function metersPerPixelAtGround(
  camera: THREE.PerspectiveCamera, point: THREE.Vector3, viewW: number, viewH: number,
  unitsPerMeter: number = SIM_UNITS_PER_METER,
): number {
  if (viewW <= 0 || viewH <= 0 || unitsPerMeter <= 0) return Number.NaN;
  const right = new THREE.Vector3().setFromMatrixColumn(camera.matrixWorld, 0);   // 相机右（世界）
  right.y = 0;                                                                   // 投到地面
  if (right.lengthSq() < 1e-12) return Number.NaN;                               // 正俯视退化
  right.normalize();
  const neighbor = point.clone().addScaledVector(right, unitsPerMeter);          // 地面上 1 米
  const a = point.clone().project(camera), b = neighbor.clone().project(camera);
  const dpx = Math.abs(b.x - a.x) * viewW / 2;
  if (!(dpx > 1e-9)) return Number.NaN;
  return 1 / dpx;
}

/** "整数好看"的候选：1 / 2 / 5 × 10ⁿ */
function niceValues(max: number): number[] {
  const out: number[] = [];
  const pow = Math.pow(10, Math.floor(Math.log10(max)));
  for (const p of [pow / 10, pow, pow * 10]) for (const mult of [1, 2, 5]) out.push(p * mult);
  return out.filter(v => v > 0).sort((a, b) => a - b);
}

/** 选一个"整数好看"的距离，使条宽 ≤ maxBarPx（取满足条件的最大值） */
export function niceScaleBar(metersPerPx: number, maxBarPx = 110): { meters: number; widthPx: number; label: string } {
  if (!(metersPerPx > 0) || !Number.isFinite(metersPerPx)) return { meters: 0, widthPx: 0, label: '—' };
  const maxMeters = metersPerPx * maxBarPx;
  let best = niceValues(maxMeters)[0] ?? maxMeters;
  for (const v of niceValues(maxMeters)) if (v <= maxMeters) best = v;
  return { meters: best, widthPx: best / metersPerPx, label: formatMeters(best) };
}

/** 距离文字：<1km 用米（<10m 给一位小数），≥1km 用 km */
export function formatMeters(m: number): string {
  if (!Number.isFinite(m)) return '—';
  if (m >= 1000) return `${(m / 1000).toFixed(2)} km`;
  return `${m < 10 ? m.toFixed(1) : Math.round(m)} m`;
}

/** 尺子的刻度间距：取"整数好看"且使刻度数接近 targetCount 的值 */
export function niceRulerInterval(lengthM: number, targetCount = 8): number {
  const want = lengthM / Math.max(1, targetCount);
  if (!(want > 0) || !Number.isFinite(want)) return 0;
  let best = niceValues(want)[0] ?? want, bestErr = Infinity;
  for (const v of niceValues(want)) {
    const err = Math.abs(Math.log10(v / want));
    if (err < bestErr) { bestErr = err; best = v; }
  }
  return best;
}

/**
 * 尺子刻度：沿 from→to 每 `intervalM` 米打一道横杠（含两端），返回世界坐标下的线段。
 * 这就是"比例尺条"的视觉语言——拖动出来的尺子能像地图上的比例尺一样"数格子"。
 *
 * @param unitsPerMeter 世界单位/米
 * @param tickHalf 半刻度长（世界单位）
 */
export function rulerTicks(
  from: THREE.Vector3, to: THREE.Vector3, intervalM: number,
  unitsPerMeter: number = SIM_UNITS_PER_METER, tickHalf = 6,
): Array<[THREE.Vector3, THREE.Vector3]> {
  const out: Array<[THREE.Vector3, THREE.Vector3]> = [];
  const dx = to.x - from.x, dz = to.z - from.z;
  const len = Math.hypot(dx, dz);
  if (!(len > 0) || !(intervalM > 0)) return out;
  const ux = dx / len, uz = dz / len;
  const px = -uz, pz = ux;                                  // 地面内法向（垂直于尺子）
  const step = intervalM * unitsPerMeter;
  const n = Math.max(1, Math.round(len / step));
  for (let i = 0; i <= n; i++) {
    const t = (i / n) * len;
    const cx = from.x + ux * t, cz = from.z + uz * t;
    out.push([
      new THREE.Vector3(cx + px * tickHalf, from.y, cz + pz * tickHalf),
      new THREE.Vector3(cx - px * tickHalf, from.y, cz - pz * tickHalf),
    ]);
  }
  return out;
}
