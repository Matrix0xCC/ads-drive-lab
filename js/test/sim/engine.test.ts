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

    // 黄金基线（防重构回归；数值来自实现后实测值，
    // 人工核验：5s 前进 ~100m 沿赛道、末速 26.7m/s、无 NaN、状态 ACTIVE。
    // 2026-09-19 更新：横向路径夹紧 [l0,target]（消除贴边棘轮复位）后重取）
    // 2026-09-25 更新（阶段 C 收敛迭代）：新增"横向机动速度上限"——车道横向偏移未收敛
    //   时限速（本场景 5s 内收敛到车道中心 l=1.57m；人工核验：进度 107.8m、末速 26.66m/s、
    //   轨迹有限、状态 ACTIVE、迭代轮数恒为 1，位置差 5mm 量级）后重取。
    // 2026-09-25 再次更新（未观测区口径 = 'speed' 见-即-停限速，见 paths.ts UNKNOWN_HANDLING）：
    //   弯道遮挡处可见距离变短 → 限速生效（人工核验：进度 107.8m 不变、末速 26.50m/s、
    //   轨迹有限、状态 ACTIVE、位置差 1m 量级）后重取。
    assert.strictEqual(e.car.x, 1046.8882377338923);
    assert.strictEqual(e.car.y, 702.2694954419853);
    assert.strictEqual(e.car.angle, 2.9329273335407517);
    assert.strictEqual(e.car.speed, 106.015045465087);
  });
});

describe('SimEngine 路线模式：终点之外的障碍', () => {
  it('路线终点之外的障碍不参与规划（不假停车、正常到达）', () => {
    // 实测根因（2026-09-25 走廊接线）：终点外 ~53m 处、横跨终点延长线的障碍，
    // 其命中点经"首尾绕接的幻影闭合段"投影到 s≈totalS、l≈±15m，膨胀后封死整条
    // 走廊 → 迟滞状态机封死 → 在离终点 ~39m 处停车且永不恢复。
    const e = makeEngine(42);
    const wp = e.track.centerPoints.slice(0, 50);
    e.setRoute(
      { waypoints: wp, waypointIndices: wp.map((_, i) => i), startIndex: 0, goalIndex: 49, length: px(0) },
      0,
    );
    const end = wp[wp.length - 1], prev = wp[wp.length - 2];
    const len = Math.hypot((end.x as number) - (prev.x as number), (end.y as number) - (prev.y as number));
    const tx = ((end.x as number) - (prev.x as number)) / len, ty = ((end.y as number) - (prev.y as number)) / len;
    e.obstacles = [{
      x: px((end.x as number) + tx * 61), y: px((end.y as number) + ty * 61),
      width: m(3.0), height: m(3.0), angle: rad(0),
    }];
    for (let i = 0; i < 600; i++) e.step(FIXED_DT);
    assert.strictEqual(e.routeState, 'ARRIVED', '应正常到达终点（终点之外的障碍不得封死路线）');
    const d = Math.hypot((e.car.x as number) - (end.x as number), (e.car.y as number) - (end.y as number));
    // ARRIVED 触发条件是剩余 ≤10px（2.5m），故车停在终点前 0~2.5m 处
    assert.ok(d < px(12), `应停在终点附近，实际差 ${(d / 4).toFixed(1)}m`);
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
