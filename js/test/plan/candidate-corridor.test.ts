// ==================== 测试：候选横向目标限制在可行驶域 ====================
//
// 背景（ISSUE_HISTORY 2026-09-19 第二根因）：横向候选 {laneOffset ± 0/7/14m}
// 在 7.5m 半宽赛道上产生 ±12m/±15.9m 等越界目标；规划器无可行驶域硬约束，
// 把车带向界外 → checkBounds 复位传送 → 常在障碍正前方"复活"引发碰撞。
//
// 可行驶域定义（与 checkBounds 复位阈值一致）：
//   |l| ≤ 半宽 − 车半宽 − 复位缓冲(1.5m) − 跟踪超调余量(0.25m) = 4.75m = 19 units
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { plan } from '../../algo/plan/index.js';
import { debug } from '../../shared/debug.js';
import { CarState, WorldSnapshot, LidarScan } from '../../shared/types.js';
import { rad, m, mps, mps2, px, pxps, dimless } from '../../shared/dimensions.js';
import { MAX_SENSOR_DIST } from '../../algo/perception.js';

const CORRIDOR_MAX = 19;  // 4.75m 的仿真单位值

function makeCar(x: number, y: number, angle: number, speed: number): CarState {
  return {
    x: px(x), y: px(y), angle: rad(angle), speed: pxps(speed),
    steerAngle: rad(0), maxSpeed: mps(42.5), acceleration: mps2(8),
    friction: dimless(0.12), width: m(2.0), length: m(4.5),
    collisionRadius: px(Math.hypot(m(2.0) as number, m(4.5) as number) / 2),
  };
}

function makeSnapshot(car: CarState): WorldSnapshot {
  const pts: any[] = [];
  for (let i = 0; i <= 200; i++) pts.push({ x: px(i * 10), y: px(0) });
  pts.push(pts[0]);
  return { car, track: { centerPoints: pts, innerBoundary: [], outerBoundary: [] }, obstacles: [], timestamp: 0 };
}

function makeClearLidar(): LidarScan {
  const n = 61, angles: any[] = [], distances: any[] = [];
  for (let i = 0; i < n; i++) {
    angles.push(rad(-Math.PI / 3 + (i / (n - 1)) * 2 * Math.PI / 3));
    distances.push(MAX_SENSOR_DIST);
  }
  return { angles, distances, rawDistances: [...distances], confidences: new Array(61).fill(1), timestamp: 0 };
}

function makeBlockedLidar(blockedAngles: number[], blockedDist: number): LidarScan {
  const n = 61, angles: any[] = [], distances: any[] = [];
  for (let i = 0; i < n; i++) {
    const a = -Math.PI / 3 + (i / (n - 1)) * 2 * Math.PI / 3;
    angles.push(rad(a));
    const blocked = blockedAngles.some(ba => Math.abs(a - ba) < 0.05);
    distances.push(blocked ? px(blockedDist) : MAX_SENSOR_DIST);
  }
  return { angles, distances, rawDistances: [...distances], confidences: new Array(61).fill(1), timestamp: 0 };
}

describe('候选横向目标：可行驶域约束', () => {
  it('无路线巡航：所有候选 |targetL| ≤ 4.75m', () => {
    const car = makeCar(100, 0, 0, 120);
    plan(makeSnapshot(car), makeClearLidar());
    const cands = debug.planning!.candidates;
    assert.ok(cands.length > 0);
    for (const c of cands) {
      assert.ok(Math.abs(c.targetL) <= CORRIDOR_MAX + 1e-6, `越界候选 targetL=${c.targetL}`);
    }
  });

  it('50m 处中央障碍（可避让）：仍全部在域内，且存在安全候选', () => {
    const car = makeCar(100, 0, 0, 100);
    plan(makeSnapshot(car), makeBlockedLidar([0], 200));
    const cands = debug.planning!.candidates;
    for (const c of cands) {
      assert.ok(Math.abs(c.targetL) <= CORRIDOR_MAX + 1e-6, `越界候选 targetL=${c.targetL}`);
    }
    assert.ok(cands.some(c => !c.collision), '应存在安全候选（50m 处可在域内完成避让）');
  });

  it('20m 处中央障碍（域内不可避让）：降级为最低速度，不加速冲撞', () => {
    const car = makeCar(100, 0, 0, 120);
    plan(makeSnapshot(car), makeBlockedLidar([0], 80));
    const cands = debug.planning!.candidates;
    for (const c of cands) {
      assert.ok(Math.abs(c.targetL) <= CORRIDOR_MAX + 1e-6, `越界候选 targetL=${c.targetL}`);
    }
    // 无安全候选时，应选择最低速度目标（最小化碰撞能量），而非 cost 最优的 35m/s
    assert.strictEqual(debug.planning!.bestTargetV, mps(17.5) as number,
      `不可避让时应降级到最低速度，实际 ${debug.planning!.bestTargetV / 4}m/s`);
  });
});

describe('速度适应：按最近障碍距离限速（≈2s 横向机动窗口）', () => {
  it('障碍 30m 内 → 所有候选 targetV ≤ 17.5m/s', () => {
    const car = makeCar(100, 0, 0, 150);          // 高速接近
    plan(makeSnapshot(car), makeBlockedLidar([0], 120));  // 30m
    for (const c of debug.planning!.candidates) {
      assert.ok((c.targetV as number) <= (mps(17.5) as number) + 1e-6,
        `30m 内应限速 17.5m/s，出现 ${c.targetV / 4}m/s`);
    }
  });

  it('障碍 50m → 所有候选 targetV ≤ 25m/s', () => {
    const car = makeCar(100, 0, 0, 150);
    plan(makeSnapshot(car), makeBlockedLidar([0], 200));  // 50m
    for (const c of debug.planning!.candidates) {
      assert.ok((c.targetV as number) <= (mps(25) as number) + 1e-6,
        `50m 应限速 25m/s，出现 ${c.targetV / 4}m/s`);
    }
  });

  it('无障碍 → 不限速（存在 35m/s 候选）', () => {
    const car = makeCar(100, 0, 0, 150);
    plan(makeSnapshot(car), makeClearLidar());
    assert.ok(debug.planning!.candidates.some(c => (c.targetV as number) === (mps(35) as number)),
      '无障碍时应保留 35m/s 候选');
  });
});
