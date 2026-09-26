// ==================== 检查：安全类（碰撞 / 越界 / EMERGENCY） ====================
//
// 【碰撞】车体矩形（OBB）与真值障碍物矩形（OBB）相交即命中——用真值而非
// LiDAR，检测的是"实际撞上了"而不是"感知到风险"。曾用对角线外接圆近似，
// 侧向并行通过时高估车宽（半宽 1m → 半径 2.46m）造成误报
//（见 ISSUE_HISTORY 2026-09-19 批量钻取）。
//
// 【越界】车辆 Frenet |l| 超过赛道半宽（中心线到边界）。

import { FrameRecord } from '../../shared/types.js';
import { Detector } from '../detector.js';
import { cartesianToFrenet } from '../../algo/plan/frenet.js';
import { TRACK_HALF_WIDTH } from '../../env/track.js';
import { m, px } from '../../shared/dimensions.js';

/** 车体尺寸（与 env/car.ts createCar 一致） */
const CAR_L = m(4.5) as number;
const CAR_W = m(2.0) as number;

/**
 * 两个有向矩形（OBB）是否相交（SAT 分离轴定理）
 *
 * 对两个矩形的各轴（角度方向与其法向，共 4 条轴）逐一投影：
 * 任一轴上区间分离 → 不相交；全部重叠 → 相交。
 */
export function obbOverlap(
  ax: number, ay: number, aLen: number, aWid: number, aAngle: number,
  bx: number, by: number, bLen: number, bWid: number, bAngle: number,
): boolean {
  const axes: { x: number; y: number }[] = [];
  for (const ang of [aAngle, bAngle]) {
    const c = Math.cos(ang), s = Math.sin(ang);
    axes.push({ x: c, y: s });      // 物体角度方向（长轴）
    axes.push({ x: -s, y: c });     // 其法向（宽轴）
  }

  const project = (
    cx: number, cy: number, len: number, wid: number, ang: number,
    axis: { x: number; y: number },
  ): { min: number; max: number } => {
    const c = Math.cos(ang), s = Math.sin(ang);
    const ext = Math.abs((len / 2) * (c * axis.x + s * axis.y))
              + Math.abs((wid / 2) * (-s * axis.x + c * axis.y));
    const center = cx * axis.x + cy * axis.y;
    return { min: center - ext, max: center + ext };
  };

  for (const axis of axes) {
    const pa = project(ax, ay, aLen, aWid, aAngle, axis);
    const pb = project(bx, by, bLen, bWid, bAngle, axis);
    if (pa.max < pb.min || pb.max < pa.min) return false;   // 找到分离轴
  }
  return true;
}

/** 点到线段最短距离（越界并集判据用） */
function distToSegment(px0: number, py0: number, x1: number, y1: number, x2: number, y2: number): number {
  const dx = x2 - x1, dy = y2 - y1;
  const l2 = dx * dx + dy * dy;
  if (l2 === 0) return Math.hypot(px0 - x1, py0 - y1);
  let t = ((px0 - x1) * dx + (py0 - y1) * dy) / l2;
  t = Math.max(0, Math.min(1, t));
  return Math.hypot(px0 - (x1 + t * dx), py0 - (y1 + t * dy));
}

export function runSafetyChecks(det: Detector, rec: FrameRecord): void {
  if (rec.type === 'step') {
    det.stepCount = rec.step;
    det.lastStepRec = rec;
    const car = rec.car;

    // ① 碰撞：车体 OBB vs 真值障碍物 OBB（每帧最多记一次）
    for (const obs of det.world.obstacles) {
      if (obbOverlap(
        car.x, car.y, CAR_L, CAR_W, car.angle,
        obs.x as number, obs.y as number,
        obs.width as number, obs.height as number, obs.angle as number,
      )) {
        det.merger.add({
          check: 'collision_true', category: 'safety', severity: 'high', step: rec.step,
          summary: `车身与障碍物相交（speed=${(car.speed / 4).toFixed(1)}m/s，位置 ${(car.x / 4).toFixed(1)},${(car.y / 4).toFixed(1)}m）`,
          details: { carX: car.x, carY: car.y, speed: car.speed, obstacleX: obs.x, obstacleY: obs.y },
        });
        break;
      }
    }

    // ② 越界：|l| 超过**该地图的**半宽。
    //    参考线优先用**路线**（有路线时"道路"就是路线——路网地图由多条链组成，
    //    按某一条链算 |l| 会把正常行驶判成越界，实测 3609 帧误报）；无路线则用赛道中心线。
    //    半宽也必须取地图的（旧代码用全局 TRACK_HALF_WIDTH=7.5m，OSM 图实际 5.625m）。
    const halfW = (det.world.track.halfWidth ?? TRACK_HALF_WIDTH) as number;
    if (det.netChains) {
      // 路网：可行驶域是**链的并集**（路口处比任何单条链都宽）。用 |l| 相对某条链/路线判会误报
      // （实测：车在路口走并集内的位置，相对路线 |l| = 5.6m > 半宽 → 3687 帧误报）。
      let best = Infinity;
      for (const ch of det.netChains) {
        for (let i = 0; i + 1 < ch.length; i++) {
          best = Math.min(best, distToSegment(px(car.x), px(car.y), ch[i].x as number, ch[i].y as number, ch[i + 1].x as number, ch[i + 1].y as number));
          if (best <= halfW) break;
        }
      }
      if (best > halfW) {
        det.merger.add({
          check: 'off_track', category: 'safety', severity: 'high', step: rec.step,
          summary: `越界：到最近路网链 ${(best / 4).toFixed(1)}m 超过半宽 ${(halfW / 4).toFixed(1)}m`,
          details: { dist: best },
        });
      }
    } else {
      const ref = det.routeFrenet ?? det.trackFrenet;
      const fl = cartesianToFrenet(px(car.x), px(car.y), ref);
      if (Math.abs(fl.l as number) > halfW) {
        det.merger.add({
          check: 'off_track', category: 'safety', severity: 'high', step: rec.step,
          summary: `越界：|l|=${(Math.abs(fl.l as number) / 4).toFixed(1)}m 超过半宽 ${(halfW / 4).toFixed(1)}m`,
          details: { l: fl.l, s: fl.s },
        });
      }
    }
  }

  if (rec.type === 'event' && rec.kind === 'bounds_reset') {
    // ②b 复位事件：车体已越出赛道被强拉回中心线。
    // 必须单独处理——物理步进→复位→发布记录在同一 tick 内完成，
    // |l| 超限的瞬间不会出现在 step 记录里（见 ② 的逐帧判定兜底）。
    det.merger.add({
      check: 'off_track', category: 'safety', severity: 'high', step: rec.step,
      summary: '车体越出赛道边界并触发复位', details: rec.payload,
    });
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
