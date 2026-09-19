// ==================== 检查：规划行为类（抖动 / 摆动 / 近距险情） ====================
//
// 【状态抖动】滑动窗口内的 state_change 事件计数（规划帧间反复切换）。
// 【横向摆动】相邻规划帧最优 targetL 的跳变幅度（方向反复）。
// 【近距险情】LiDAR 最近过滤命中过近且车速过高；紧急制动事件单列。

import { FrameRecord } from '../../shared/types.js';
import { Detector } from '../detector.js';

export function runPlanningChecks(det: Detector, rec: FrameRecord): void {
  // ⑦ 状态机抖动：滑动窗口计数 state_change 事件
  if (rec.type === 'event' && rec.kind === 'state_change') {
    det.stateChangeSteps.push(rec.step);
    const win = det.cfg.flappingWindowSteps;
    det.stateChangeSteps = det.stateChangeSteps.filter(s => rec.step - s <= win);
    if (det.stateChangeSteps.length >= det.cfg.flappingCount) {
      det.merger.add({
        check: 'state_flapping', category: 'planning', severity: 'medium', step: rec.step,
        summary: `1s 窗口内状态切换 ${det.stateChangeSteps.length} 次`,
        details: { windowSteps: win, changes: det.stateChangeSteps },
      });
    }
  }

  // ⑧ 横向摆动：规划帧间最优 targetL 跳变
  if (rec.type === 'plan') {
    if (det.lastPlanTargetL !== null) {
      const jump = Math.abs(rec.best.targetL - det.lastPlanTargetL);
      if (jump > (det.cfg.lateralJump as number)) {
        det.lateralJumps.push({ step: rec.step, jump });
        const windowSteps = det.cfg.lateralWindowPlans * 4;   // 规划帧 ≈ 每 4 step 一帧
        det.lateralJumps = det.lateralJumps.filter(j => rec.step - j.step <= windowSteps);
        if (det.lateralJumps.length >= det.cfg.lateralCount) {
          det.merger.add({
            check: 'lateral_switching', category: 'planning', severity: 'medium', step: rec.step,
            summary: `${det.cfg.lateralWindowPlans} 个规划帧内横向目标跳变 ${det.lateralJumps.length} 次`,
            details: { jumps: det.lateralJumps.map(j => +(j.jump / 4).toFixed(2)) },
          });
        }
      }
    }
    det.lastPlanTargetL = rec.best.targetL;
  }

  // ⑨ 近距险情：最近过滤命中 < 阈值 且 车速 > 阈值
  if (rec.type === 'perception' && rec.stats && det.lastStepRec) {
    const minD = rec.stats.minDist;
    const speed = det.lastStepRec.car.speed;
    if (minD < (det.cfg.closeCallDist as number) && speed > (det.cfg.closeCallSpeed as number)) {
      det.merger.add({
        check: 'close_call', category: 'planning', severity: 'high', step: rec.step,
        summary: `最近障碍 ${(minD / 4).toFixed(1)}m，时速 ${(speed / 4 * 3.6).toFixed(0)}km/h`,
        details: { minDist: minD, speed },
      });
    }
  }

  // ⑨b 紧急制动事件
  if (rec.type === 'event' && rec.kind === 'emergency_brake') {
    det.merger.add({
      check: 'emergency_brake', category: 'planning', severity: 'medium', step: rec.step,
      summary: `触发紧急制动（前方 ${(Number(rec.payload.frontDist) / 4).toFixed(1)}m）`,
      details: rec.payload,
    });
  }
}
