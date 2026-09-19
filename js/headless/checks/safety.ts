// ==================== 检查：安全类（碰撞 / 越界 / EMERGENCY） ====================
//
// 【碰撞】车身近似为圆（半径 = 对角线一半，与 createCar 一致），
// 与真值障碍物（OBB）相交即命中——用真值而非 LiDAR，检测的是
// "实际撞上了"而不是"感知到风险"。
//
// 【越界】车辆 Frenet |l| 超过赛道半宽（中心线到边界）。

import { FrameRecord } from '../../shared/types.js';
import { Detector } from '../detector.js';
import { cartesianToFrenet } from '../../algo/plan/frenet.js';
import { TRACK_HALF_WIDTH } from '../../env/track.js';
import { m, px } from '../../shared/dimensions.js';

/** 圆（车身）与矩形（障碍物 OBB）相交判定（局部坐标 clamp 法） */
export function circleRectOverlap(
  cx: number, cy: number, r: number,
  obs: { x: number; y: number; width: number; height: number; angle: number },
): boolean {
  const c = Math.cos(-obs.angle), s = Math.sin(-obs.angle);
  const dx = cx - obs.x, dy = cy - obs.y;
  const lx = dx * c - dy * s, ly = dx * s + dy * c;   // 转到障碍物局部系
  const hw = obs.width / 2, hh = obs.height / 2;
  const nx = Math.max(-hw, Math.min(hw, lx));
  const ny = Math.max(-hh, Math.min(hh, ly));
  return Math.hypot(lx - nx, ly - ny) <= r;
}

/** 车身碰撞半径（与 env/car.ts createCar 一致） */
const CAR_COLLISION_R = Math.hypot(m(2.0) as number, m(4.5) as number) / 2;

export function runSafetyChecks(det: Detector, rec: FrameRecord): void {
  if (rec.type === 'step') {
    det.stepCount = rec.step;
    det.lastStepRec = rec;
    const car = rec.car;

    // ① 碰撞：车身圆 vs 真值障碍物（每帧最多记一次）
    for (const obs of det.world.obstacles) {
      if (circleRectOverlap(car.x, car.y, CAR_COLLISION_R, obs)) {
        det.merger.add({
          check: 'collision_true', category: 'safety', severity: 'high', step: rec.step,
          summary: `车身与障碍物相交（speed=${(car.speed / 4).toFixed(1)}m/s，位置 ${(car.x / 4).toFixed(1)},${(car.y / 4).toFixed(1)}m）`,
          details: { carX: car.x, carY: car.y, speed: car.speed, obstacleX: obs.x, obstacleY: obs.y },
        });
        break;
      }
    }

    // ② 越界：|l| 超赛道半宽
    const fl = cartesianToFrenet(px(car.x), px(car.y), det.trackFrenet);
    if (Math.abs(fl.l as number) > (TRACK_HALF_WIDTH as number)) {
      det.merger.add({
        check: 'off_track', category: 'safety', severity: 'high', step: rec.step,
        summary: `越界：|l|=${(Math.abs(fl.l as number) / 4).toFixed(1)}m 超过半宽 ${((TRACK_HALF_WIDTH as number) / 4).toFixed(1)}m`,
        details: { l: fl.l, s: fl.s },
      });
    }
  }

  if (rec.type === 'plan') {
    det.lastPlanRec = rec;
    // ③ EMERGENCY 状态
    if (rec.state === 'EMERGENCY') {
      det.merger.add({
        check: 'emergency_state', category: 'safety', severity: 'medium', step: rec.step,
        summary: '规划器处于 EMERGENCY 状态（所有候选轨迹碰撞）', details: {},
      });
    }
  }
}
