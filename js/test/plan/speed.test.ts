// ==================== 测试：速度层（ST 空间） ====================
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { solveSpeedLayer, STObstacle, A_COMFORT } from '../../algo/plan/speed.js';

/** 采样峰值 |加速度| */
function peakAccel(profile: { eval(t: number): number; deriv1(t: number): number; duration: number }): number {
  let peak = 0, prevV = profile.deriv1(0);
  for (let i = 1; i <= 40; i++) {
    const t = (i / 40) * profile.duration;
    const v = profile.deriv1(t);
    peak = Math.max(peak, Math.abs(v - prevV) / (profile.duration / 40));
    prevV = v;
  }
  return peak;
}
const staticObstacle = (sMin: number): STObstacle => ({ sMin, sMax: sMin + 32, tMin: 0, tMax: Infinity });

describe('速度层：巡航', () => {
  it('无阻塞 → cruise，从 v0 收敛到 targetV', () => {
    const r = solveSpeedLayer({ stObstacles: [], v0: 100, targetV: 100 });
    assert.strictEqual(r.mode, 'cruise');
    assert.ok(Math.abs(r.profile.deriv1(0) - 100) < 1);
    assert.ok(Math.abs(r.profile.deriv1(r.profile.duration) - 100) < 2);
    assert.ok(r.profile.eval(r.profile.duration) > 150, '应正常前进');
  });
});

describe('速度层：停车线', () => {
  it('距离充足 → stop：终点停在障碍前（sStop = sMin−STOP_MARGIN），v(T)≈0 且单调', () => {
    const r = solveSpeedLayer({ stObstacles: [staticObstacle(440)], v0: 100, targetV: 100 });
    assert.strictEqual(r.mode, 'stop');
    const sStop = 440 - 16;             // STOP_MARGIN = m(4) = 16
    assert.ok(Math.abs(r.profile.eval(r.profile.duration) - sStop) < 3, `s(T)=${r.profile.eval(r.profile.duration)}`);
    assert.ok(Math.abs(r.profile.deriv1(r.profile.duration)) < 2, 'v(T)≈0');
    // 不后退（前进单调）
    let prev = -1;
    for (let i = 0; i <= 40; i++) {
      const s = r.profile.eval((i / 40) * r.profile.duration);
      assert.ok(s >= prev - 1e-6, '前进单调');
      prev = s;
    }
  });

  it('距离充裕 → 峰值减速度不超过舒适值（10% 容差）', () => {
    // sStop=800：自然时长 16s → 峰值 ≈ 1.875×100/16 ≈ 11.7u/s² < 舒适值 14
    const r = solveSpeedLayer({ stObstacles: [staticObstacle(816)], v0: 100, targetV: 100 });
    assert.strictEqual(r.mode, 'stop');
    assert.ok(peakAccel(r.profile) <= (A_COMFORT as number) * 1.1,
      `峰值减速 ${peakAccel(r.profile).toFixed(1)} > 舒适值 ${A_COMFORT}`);
  });

  it('物理停不住 → emergency（距离 < v0²/2A_MAX）', () => {
    const r = solveSpeedLayer({ stObstacles: [staticObstacle(40)], v0: 240, targetV: 240 });
    assert.strictEqual(r.mode, 'emergency');
    assert.ok(r.profile.deriv1(r.profile.duration) < r.profile.deriv1(0), '应减速');
  });

  it('动态条目（tMin>0）→ cruise（接口预留）', () => {
    const r = solveSpeedLayer({ stObstacles: [{ sMin: 300, sMax: 350, tMin: 2, tMax: 5 }], v0: 100, targetV: 100 });
    assert.strictEqual(r.mode, 'cruise');
  });
});
