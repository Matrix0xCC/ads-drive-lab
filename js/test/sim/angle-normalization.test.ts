// ==================== 测试：角度源头归一化 ====================
//
// 背景（ISSUE_HISTORY 2026-09-19）：car.angle 与扫描角历史上一路累积不归一化，
// 导致下游（碰撞检测等）必须处理超过 ±π 的原始角。现在在产生处归一化，
// 消除所有下游对"角度可能未归一化"的隐式假设。
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { createCar, stepPhysics } from '../../env/car.js';
import { sense } from '../../algo/perception.js';
import { rad, dimless, px, pxps, pxps2, m } from '../../shared/dimensions.js';

describe('stepPhysics 角度归一化', () => {
  it('car.angle 原始值超过 π 时，步进后保持在 [-π, π] 且等价', () => {
    const car = createCar({ x: px(0), y: px(0) } as any, { x: px(10), y: px(0) } as any);
    car.angle = rad(3.5);                       // 3.5 rad > π
    car.speed = pxps(100);
    stepPhysics(car, { steerAngle: rad(0), throttle: dimless(0.5), timestamp: 0 }, 1 / 60);
    assert.ok(Math.abs(car.angle as number) <= Math.PI, `angle 未归一化: ${car.angle}`);
    assert.ok(Math.abs((car.angle as number) - (3.5 - 2 * Math.PI)) < 1e-9,
      `归一化值不正确: ${car.angle}`);
  });
});

describe('sense 射线角归一化', () => {
  it('car.angle=190° 时所有射线角在 [-π, π]', () => {
    const pts: any[] = [];
    for (let i = 0; i <= 200; i++) pts.push({ x: px(i * 10), y: px(0) });
    pts.push(pts[0]);
    const snapshot = {
      car: {
        x: px(100), y: px(0), angle: rad(190 * Math.PI / 180), speed: pxps(100),
        steerAngle: rad(0), maxSpeed: pxps(170), acceleration: pxps2(32),
        friction: dimless(0.12), width: m(2.0), length: m(4.5),
        collisionRadius: px(Math.hypot(8, 18) / 2),
      },
      track: { centerPoints: pts, innerBoundary: [], outerBoundary: [] },
      obstacles: [],
      timestamp: 0,
    };
    const scan = sense(snapshot as any);
    for (const a of scan.angles) {
      assert.ok(Math.abs(a as number) <= Math.PI + 1e-9, `越界射线角 ${a}`);
    }
  });
});
