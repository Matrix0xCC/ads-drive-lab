// ==================== 测试：跟驰目标选取（C 的入口） ====================
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { selectFollowTarget } from '../../algo/plan/lead.js';
import { m, mps, mps2 } from '../../shared/dimensions.js';

const t = (s: number, l: number, vS: number, confirmed = true) => ({ s, l, vS, confirmed });
const lane = { center: 0, half: m(2.4) };

describe('跟驰目标：选谁', () => {
  it('取"我的车道上、前方、最近"的那个', () => {
    const r = selectFollowTarget(
      [t(m(80), 0, mps(20)), t(m(40), 0, mps(18)), t(m(60), 0, mps(19))],
      0, 0, mps(25), mps2(8), lane.center, lane.half, false, m(1000));
    assert.ok(r && Math.abs(r.gap - (m(40) as number)) < 1, '应取 40m 那个');
    assert.ok(Math.abs(r!.leadSpeed - (mps(18) as number)) < 1);
  });

  it('相邻车道的车不算（归属）', () => {
    const r = selectFollowTarget([t(m(30), m(3.75), mps(18))], 0, 0, mps(25), mps2(8), lane.center, lane.half, false, m(1000));
    assert.strictEqual(r, null);
  });

  it('后方的车不算', () => {
    const r = selectFollowTarget([t(m(-20), 0, mps(18))], 0, 0, mps(25), mps2(8), lane.center, lane.half, false, m(1000));
    assert.strictEqual(r, null);
  });

  it('未确认的轨迹不算（跟踪器还没跟稳）', () => {
    const r = selectFollowTarget([t(m(50), 0, mps(18), false)], 0, 0, mps(25), mps2(8), lane.center, lane.half, false, m(1000));
    assert.strictEqual(r, null);
  });

  it('**安全门**：降不到前车速度时不许跟（间距不足 → 按静态障碍走停车线）', () => {
    // 自车 30m/s、前车 5m/s → 需要 (900−25)/(2·8) ≈ 55m + 10m 余量 = 65m
    const far = selectFollowTarget([t(m(80), 0, mps(5))], 0, 0, mps(30), mps2(8), lane.center, lane.half, false, m(1000));
    assert.ok(far, '80m 足够 → 可以跟');
    const near = selectFollowTarget([t(m(40), 0, mps(5))], 0, 0, mps(30), mps2(8), lane.center, lane.half, false, m(1000));
    assert.strictEqual(near, null, '40m 不够降到 5m/s → 不许跟（必须走静态逻辑）');
  });

  it('基本静止的前车不算跟驰目标（按静态障碍走停车线）', () => {
    const r = selectFollowTarget([t(m(200), 0, mps(0.5))], 0, 0, mps(20), mps2(8), lane.center, lane.half, false, m(1000));
    assert.strictEqual(r, null);
  });

  it('闭环：绕接后在我前方的物体能选到', () => {
    const total = m(400) as number;
    const r = selectFollowTarget([t(m(20), 0, mps(18))], m(390), 0, mps(20), mps2(8), lane.center, lane.half, true, total);
    assert.ok(r, 's=20 在闭环上位于 s=390 的前方 30m');
    assert.ok(Math.abs(r!.gap - m(30)) < 1);
  });
});
