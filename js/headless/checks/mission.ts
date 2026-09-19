// ==================== 检查：跟踪/任务类（跟踪误差 / 停滞 / 停车异常 / 超时） ====================
//
// 【跟踪误差】规划帧 k 的轨迹在 t(k+1) 时刻按时间线性插值出期望位置，
// 与下一规划帧时车辆实际位置比较；位置偏差超阈值记超限帧，
// 平方和/样本数供报告计算 RMSE。
//
// 【停滞】有路线但持续低速（默认 3s 内平均 < 0.5m/s）。
// 【停车异常】ARRIVED 后 1s 内速度仍超阈值。
// 【超时】运行结束时仍未到达（finish 阶段判定，见 finalizeMissionChecks）。

import { FrameRecord } from '../../shared/types.js';
import { Detector } from '../detector.js';

/** 点到线段的最短距离 */
function pointSegDist(px: number, py: number, x1: number, y1: number, x2: number, y2: number): number {
  const dx = x2 - x1, dy = y2 - y1;
  const len2 = dx * dx + dy * dy;
  if (len2 === 0) return Math.hypot(px - x1, py - y1);
  let t = ((px - x1) * dx + (py - y1) * dy) / len2;
  t = Math.max(0, Math.min(1, t));
  return Math.hypot(px - (x1 + t * dx), py - (y1 + t * dy));
}

export function runMissionChecks(det: Detector, rec: FrameRecord): void {
  if (rec.type === 'step') {
    const speed = rec.car.speed;

    // ⑫a 停滞：有路线且持续低速
    if (det.routeActive) {
      if (speed < (det.cfg.stallSpeed as number)) det.stallRun += 1;
      else det.stallRun = 0;
      if (det.stallRun === det.cfg.stallSteps) {
        det.merger.add({
          check: 'stalled', category: 'mission', severity: 'high', step: rec.step,
          summary: `有路线但停滞 ${(det.cfg.stallSteps / 60).toFixed(0)}s（速度 < 0.5m/s）`,
          details: { speed },
        });
      }
    }

    // ⑫b 到达后未及时停车
    if (det.arrivedAtStep !== null && rec.step - det.arrivedAtStep <= 60
        && speed > (det.cfg.arriveSpeed as number)) {
      det.merger.add({
        check: 'stop_anomaly', category: 'mission', severity: 'medium', step: rec.step,
        summary: `ARRIVED 后速度仍为 ${(speed / 4 * 3.6).toFixed(0)}km/h`,
        details: { speed, sinceArriveSteps: rec.step - det.arrivedAtStep },
      });
    }
  }

  if (rec.type === 'event' && rec.kind === 'arrived') {
    det.arrivedAtStep = rec.step;
  }

  if (rec.type === 'plan') {
    // ⑩ 跟踪误差 = 交叉跟踪距离：车到上一规划帧轨迹折线的最小距离。
    // 旧口径按时间插值比对期望位置，把规划起点 s0 的 Frenet 量化误差
    //（最近采样点，间距 ~2.9m）计成"误差"——实测纵向伪影 ~0.9m 而真实
    // 交叉跟踪仅 ~0.04m（2026-09-19 第五轮，见 ISSUE_HISTORY）。
    if (det.prevTraj && det.prevTraj.points.length >= 2 && det.lastStepRec) {
      const car = det.lastStepRec.car;
      const pts = det.prevTraj.points;
      let minD = Infinity;
      for (let i = 0; i < pts.length - 1; i++) {
        minD = Math.min(minD, pointSegDist(car.x, car.y, pts[i].x, pts[i].y, pts[i + 1].x, pts[i + 1].y));
      }
      if (Number.isFinite(minD)) {
        det.trackingErrSum += minD * minD;
        det.trackingErrN += 1;
        if (minD > (det.cfg.trackingErr as number)) {
          det.merger.add({
            check: 'tracking_error', category: 'mission', severity: 'medium', step: rec.step,
            summary: `交叉跟踪偏差 ${(minD / 4).toFixed(2)}m`,
            details: { minD, carX: car.x, carY: car.y },
          });
        }
      }
    }
    det.prevTraj = rec.trajectory;
  }
}

/** 终局检查（由 Detector.finish 调用）：有路线但从未到达 → 超时 */
export function finalizeMissionChecks(det: Detector): void {
  if (det.routeActive && det.arrivedAtStep === null) {
    const remaining = det.routeLength !== null
      ? det.routeLength - (det.lastPlanRec?.frenet.s ?? 0)
      : null;
    det.merger.add({
      check: 'route_timeout', category: 'mission', severity: 'medium',
      step: det.lastStepRec?.step ?? 0,
      summary: '运行结束仍未到达路线终点'
        + (remaining !== null ? `（剩余 ${(remaining / 4).toFixed(1)}m）` : ''),
      details: { remaining },
    });
  }
}
