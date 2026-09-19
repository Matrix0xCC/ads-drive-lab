// ==================== 测试：仿真引擎（确定性 + 行为基线） ====================
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { SimEngine, FIXED_DT } from '../../sim/engine.js';
import { MessageBus } from '../../messageBus.js';
import { MAP_PRESETS } from '../../env/mapDefinitions.js';
import { checkBounds } from '../../env/car.js';
import { m, mps, px, rad } from '../../shared/dimensions.js';

function makeEngine(seed = 42) {
  return new SimEngine(new MessageBus('test', false), { map: MAP_PRESETS['oval'], seed });
}

describe('SimEngine 确定性', () => {
  it('同种子两次运行 → 600 步后状态逐字段一致', () => {
    const e1 = makeEngine(42), e2 = makeEngine(42);
    for (let i = 0; i < 600; i++) { e1.step(FIXED_DT); e2.step(FIXED_DT); }
    assert.strictEqual(e1.car.x, e2.car.x);
    assert.strictEqual(e1.car.y, e2.car.y);
    assert.strictEqual(e1.car.angle, e2.car.angle);
    assert.strictEqual(e1.car.speed, e2.car.speed);
    assert.strictEqual(e1.obstacles.length, e2.obstacles.length);
  });
});

describe('SimEngine 不变量（静态场景：无路线，车不动）', () => {
  it('10s 后数值有限、速度归零（与浏览器语义一致：无路线不驱动）', () => {
    const e = makeEngine(42);
    for (let i = 0; i < 600; i++) e.step(FIXED_DT);
    assert.ok(Number.isFinite(e.car.x as number));
    assert.ok(Number.isFinite(e.car.y as number));
    assert.strictEqual(e.car.speed, 0);
    assert.strictEqual(e.stepCount, 600);
  });
});

describe('SimEngine 路线模式', () => {
  it('设置路线后车辆运动，且速度不超过最高速', () => {
    const e = makeEngine(42);
    const wp = e.track.centerPoints.slice(0, 50);
    e.setRoute(
      { waypoints: wp, waypointIndices: wp.map((_, i) => i), startIndex: 0, goalIndex: 49, length: px(0) },
      0,
    );
    assert.strictEqual(e.routeState, 'ACTIVE');
    for (let i = 0; i < 300; i++) e.step(FIXED_DT);
    assert.ok((e.car.speed as number) > 0);
    assert.ok((e.car.speed as number) <= (mps(42.5) as number) + 1e-9);

    // 黄金基线（防重构回归；数值来自实现后首次运行的实测值，
    // 人工核验：5s 前进 ~100m 沿赛道、末速 33.7 m/s、无 NaN）
    assert.strictEqual(e.car.x, 1043.0474534107173);
    assert.strictEqual(e.car.y, 710.5696776418495);
    assert.strictEqual(e.car.angle, 2.924154568650704);
    assert.strictEqual(e.car.speed, 134.8597863124635);
  });
});

describe('checkBounds 返回值', () => {
  it('车在赛道外 → 返回 true 并复位到中心线', () => {
    const e = makeEngine(42);
    e.car.x = px(-10000); e.car.y = px(-10000);
    const reset = checkBounds(e.car, e.track);
    assert.strictEqual(reset, true);
    assert.ok((e.car.x as number) > 0); // 已复位到赛道附近
  });
});
