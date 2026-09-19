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
    // ⑩ 跟踪误差：上一规划帧轨迹 → 按时间插值到当前时刻
    if (det.prevTraj && det.prevPlanStep !== null && det.lastStepRec) {
      const dt = (rec.step - det.prevPlanStep) / 60;   // 秒
      const pts = det.prevTraj.points;
      let exp: { x: number; y: number } | null = null;
      if (pts.length >= 2 && dt <= pts[pts.length - 1].t) {
        for (let i = 0; i < pts.length - 1; i++) {
          if (dt >= pts[i].t && dt <= pts[i + 1].t) {
            const f = (dt - pts[i].t) / Math.max(pts[i + 1].t - pts[i].t, 1e-9);
            exp = {
              x: pts[i].x + f * (pts[i + 1].x - pts[i].x),
              y: pts[i].y + f * (pts[i + 1].y - pts[i].y),
            };
            break;
          }
        }
      }
      if (exp) {
        const car = det.lastStepRec.car;
        const err = Math.hypot(car.x - exp.x, car.y - exp.y);
        det.trackingErrSum += err * err;
        det.trackingErrN += 1;
        if (err > (det.cfg.trackingErr as number)) {
          det.merger.add({
            check: 'tracking_error', category: 'mission', severity: 'medium', step: rec.step,
            summary: `跟踪偏差 ${(err / 4).toFixed(2)}m（期望 ${(exp.x / 4).toFixed(1)},${(exp.y / 4).toFixed(1)}）`,
            details: { err, expX: exp.x, expY: exp.y, carX: car.x, carY: car.y },
          });
        }
      }
    }
    det.prevTraj = rec.trajectory;
    det.prevPlanStep = rec.step;
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
