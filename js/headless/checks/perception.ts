// ==================== 检查：感知类（幻影 / 漏检 / GPS 误差） ====================
//
// 【幻影命中】过滤后（conf ≥ 0.5）的命中点换算到世界坐标，若容差内
// 没有任何真值障碍物 → 感知层报了不存在的障碍（边界过滤失误）。
//
// 【漏检】真值障碍落在车头 ±FOV 内且距离 < MAX_SENSOR_DIST，
// 但对应角度扇区（障碍角宽 + 2° 容差）没有任何射线在其前方命中
// → 感知丢失目标。
//
// 【GPS】误差 > gpsThresholdSigma × σ 记为超限。

import { FrameRecord, Obstacle } from '../../shared/types.js';
import { Detector } from '../detector.js';
import { MAX_SENSOR_DIST } from '../../algo/perception.js';
import { m } from '../../shared/dimensions.js';

const FOV = Math.PI * 2 / 3;           // ±60°
const CONF_FILTER = 0.5;
const ANGLE_TOLERANCE = 0.035;         // 约 2°（障碍半角外扩容差）

/** 点到矩形（OBB）的距离（内部为 0） */
function pointRectDist(px0: number, py0: number, o: Obstacle): number {
  const c = Math.cos(-(o.angle as number)), s = Math.sin(-(o.angle as number));
  const dx = px0 - (o.x as number), dy = py0 - (o.y as number);
  const lx = dx * c - dy * s, ly = dx * s + dy * c;
  const hw = (o.width as number) / 2, hh = (o.height as number) / 2;
  const ox = Math.max(Math.abs(lx) - hw, 0), oy = Math.max(Math.abs(ly) - hh, 0);
  return Math.hypot(ox, oy);
}

export function runPerceptionChecks(det: Detector, rec: FrameRecord): void {
  if (rec.type === 'perception') {
    const car = det.lastStepRec?.car;      // safety 检查先行更新
    if (!car) return;
    const { angles, distances, confidences } = rec.scan;
    const tolerance = det.cfg.phantomTolerance as number;

    // ④ 幻影命中
    for (let i = 0; i < angles.length; i++) {
      if ((confidences[i] ?? 0) < CONF_FILTER) continue;
      const d = distances[i];
      if (d >= (MAX_SENSOR_DIST as number) - 1) continue;
      const hx = car.x + Math.cos(angles[i]) * d;
      const hy = car.y + Math.sin(angles[i]) * d;
      const near = det.world.obstacles.some(o => pointRectDist(hx, hy, o) <= tolerance);
      if (!near) {
        det.merger.add({
          check: 'phantom_hit', category: 'perception', severity: 'medium', step: rec.step,
          summary: `射线 ${(angles[i] * 180 / Math.PI).toFixed(0)}° 命中 ${(d / 4).toFixed(1)}m 处但无真值障碍`,
          details: { hitX: hx, hitY: hy, dist: d, angle: angles[i] },
        });
      }
    }

    // ⑤ 漏检：逐真值障碍，检查 FOV 内是否被照射
    for (const o of det.world.obstacles) {
      const dx = (o.x as number) - car.x, dy = (o.y as number) - car.y;
      const dist = Math.hypot(dx, dy);
      if (dist >= (MAX_SENSOR_DIST as number)) continue;
      const rel = Math.atan2(dy, dx) - car.angle;
      const relNorm = Math.atan2(Math.sin(rel), Math.cos(rel));
      if (Math.abs(relNorm) > FOV / 2 - 0.05) continue;    // FOV 外
      const halfAngle = Math.atan2((o.width as number) / 2, Math.max(dist - (o.width as number) / 2, 1)) + ANGLE_TOLERANCE;
      let seen = false;
      for (let i = 0; i < angles.length; i++) {
        const ra = angles[i];
        if (Math.abs(ra - relNorm) > halfAngle) continue;
        const expected = dist - (o.height as number) / 2;
        if ((distances[i] as number) <= expected + (m(0.5) as number)) { seen = true; break; }
      }
      if (!seen) {
        det.merger.add({
          check: 'missed_obstacle', category: 'perception', severity: 'medium', step: rec.step,
          summary: `真值障碍 ${(dist / 4).toFixed(1)}m 处未被感知`,
          details: { obstacleX: o.x, obstacleY: o.y, dist },
        });
      }
    }
  }

  // ⑥ GPS 误差超限
  if (rec.type === 'gps') {
    const threshold = (det.cfg.gpsSigma as number) * det.cfg.gpsThresholdSigma;
    if (rec.err > threshold) {
      det.merger.add({
        check: 'gps_error', category: 'perception', severity: 'low', step: rec.step,
        summary: `GPS 误差 ${(rec.err / 4).toFixed(2)}m（> ${det.cfg.gpsThresholdSigma}σ）`,
        details: { err: rec.err },
      });
    }
  }
}
