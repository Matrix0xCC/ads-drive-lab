// ==================== 测试：控制层（Pure Pursuit） ====================
//
// 回归背景（2026-09-19 解耦规划批量 A/B）：速度层的停车/紧急剖面末速为 0，
// 但控制层油门对任意目标速度都有 0.15 怠速下限 → "停车"指令被架空，
// 车以 0.15×maxSpeed=6.375m/s 恒速爬行穿过障碍（批量 7/360 帧的放大器）。
// 语义：轨迹终点速度为 0（或≈0）时油门必须为 0，允许真正停车。

import { describe, it } from 'node:test';
import assert from 'node:assert';
import { control } from '../../algo/control.js';
import { createTrajectory, CarState, Trajectory } from '../../shared/types.js';
import { rad, m, mps, mps2, px, pxps, secs, dimless, perpx } from '../../shared/dimensions.js';

function makeCar(speed: number): CarState {
  return {
    x: px(100), y: px(0), angle: rad(0), speed: pxps(speed),
    steerAngle: rad(0), maxSpeed: mps(42.5), acceleration: mps2(8),
    friction: dimless(0.12), width: m(2.0), length: m(4.5),
    collisionRadius: px(Math.hypot(m(2.0) as number, m(4.5) as number) / 2),
  };
}

/** 轨迹：沿 x 轴直线，终点速度 = vEnd（起点速度不影响油门语义） */
function trajEndingAt(vEnd: number): Trajectory {
  const pts: any[] = [];
  for (let i = 0; i <= 10; i++) {
    pts.push({ t: secs(i * 0.1), x: px(100 + i * 10), y: px(0), yaw: rad(0), v: pxps(vEnd), kappa: perpx(0) });
  }
  return createTrajectory(pts, 0, 'test');
}

describe('控制层：油门目标（轨迹终点速度）', () => {
  it('终点速度 0（停车/紧急剖面）→ 油门 0：允许真正停车', () => {
    const cmd = control(trajEndingAt(0), makeCar(25));
    assert.strictEqual(cmd.throttle, 0);
  });

  it('终点速度≈0（数值噪声 0.5u/s）→ 油门 0', () => {
    const cmd = control(trajEndingAt(0.5), makeCar(25));
    assert.strictEqual(cmd.throttle, 0);
  });

  it('终点速度为正 → 保留 0.15 怠速下限（低速目标不熄火）', () => {
    const cmd = control(trajEndingAt(5), makeCar(25));
    assert.ok((cmd.throttle as number) >= 0.15, `实际 ${cmd.throttle}`);
  });

  it('终点速度为巡航值 → 油门 = v/maxSpeed', () => {
    const cmd = control(trajEndingAt(85), makeCar(25));   // 85/170 = 0.5
    assert.ok(Math.abs((cmd.throttle as number) - 0.5) < 1e-9, `实际 ${cmd.throttle}`);
  });
});
