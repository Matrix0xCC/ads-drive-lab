// ==================== 测试：规划器 ====================

import { describe, it } from 'node:test';
import assert from 'node:assert';
import { plan, getPlannerState } from '../../algo/plan/index.js';
import { buildFrenetReference } from '../../algo/plan/frenet.js';
import { CarState, WorldSnapshot, LidarScan, Route } from '../../shared/types.js';
import { rad, m, mps, mps2, px, pxps, pxps2, dimless, perpx } from '../../shared/dimensions.js';
import { MAX_SENSOR_DIST } from '../../algo/perception.js';

// ─── 辅助 ───

function makeCar(x: number, y: number, angle: number, speed: number): CarState {
  return {
    x: px(x), y: px(y), angle: rad(angle), speed: pxps(speed),
    steerAngle: rad(0), maxSpeed: mps(42.5), acceleration: mps2(8),
    friction: dimless(0.12), width: m(2.0), length: m(4.5), collisionRadius: px(Math.hypot(m(2.0) as number, m(4.5) as number) / 2),
  };
}

/** 创建一条沿 x 轴的直线参考线（测试车辆在 l≈0 上，障碍物也在 l≈0） */
function makeSnapshot(car: CarState): WorldSnapshot {
  const pts: any[] = [];
  for (let i = 0; i <= 200; i++) pts.push({ x: px(i * 10), y: px(0) });
  pts.push(pts[0]);
  return { car, track: { centerPoints: pts, innerBoundary: [], outerBoundary: [] }, obstacles: [], timestamp: 0 };
}

function makeClearLidar(): LidarScan {
  const n = 61;
  const angles: any[] = [];
  const distances: any[] = [];
  for (let i = 0; i < n; i++) {
    angles.push(rad(-Math.PI / 3 + (i / (n - 1)) * 2 * Math.PI / 3));
    distances.push(MAX_SENSOR_DIST);
  }
  const rawDistances = [...distances];
  return { angles, distances, rawDistances, confidences: new Array(61).fill(1), timestamp: 0 };
}

function makeBlockedLidar(blockedAngles: number[], blockedDist: number): LidarScan {
  const n = 61;
  const angles: any[] = [];
  const distances: any[] = [];
  for (let i = 0; i < n; i++) {
    const a = -Math.PI / 3 + (i / (n - 1)) * 2 * Math.PI / 3;
    angles.push(rad(a));
    const blocked = blockedAngles.some(ba => Math.abs(a - ba) < 0.05);
    distances.push(blocked ? px(blockedDist) : MAX_SENSOR_DIST);
  }
  const rawDistances = [...distances];
  return { angles, distances, rawDistances, confidences: new Array(61).fill(1), timestamp: 0 };
}

// ═══════════════════════════════════════════════════
// 巡航 + 避障
// ═══════════════════════════════════════════════════

describe('planner 巡航', () => {
  it('全清 LiDAR → 正常轨迹', () => {
    const car = makeCar(100, 0, 0, 120);
    const traj = plan(makeSnapshot(car), makeClearLidar());
    assert.ok(traj.points.length > 0);
  });

  it('前方 80px 障碍物 → 避障', () => {
    const car = makeCar(100, 0, 0, 120);
    const traj = plan(makeSnapshot(car), makeBlockedLidar([0], 80));
    assert.ok(traj.type !== 'lane_keep');
    assert.ok(traj.type !== 'emergency');
  });

  it('远处 200px 障碍物 → 不触发紧急', () => {
    const car = makeCar(100, 0, 0, 120);
    const traj = plan(makeSnapshot(car), makeBlockedLidar([0], 200));
    assert.ok(traj.type !== 'emergency');
  });
});

describe('planner 一致性', () => {
  it('相同输入 → 相同类型', () => {
    const car = makeCar(100, 0, 0, 120);
    const lidar = makeClearLidar();
    let firstType = '';
    for (let i = 0; i < 10; i++) {
      const traj = plan(makeSnapshot(car), lidar);
      if (i === 0) firstType = traj.type;
      assert.strictEqual(traj.type, firstType);
      assert.ok(traj.points.length > 0);
    }
  });

  it('30 帧内避障方向切换 ≤2 次', () => {
    const car = makeCar(100, 0, 0, 120);
    const lidar = makeBlockedLidar([0], 60);
    let switches = 0;
    let lastType = '';
    for (let i = 0; i < 30; i++) {
      const traj = plan(makeSnapshot(car), lidar);
      if (lastType && traj.type !== lastType) switches++;
      lastType = traj.type;
    }
    assert.ok(switches <= 2, `实际切换 ${switches} 次`);
  });
});

describe('planner 路线处理', () => {
  it('接近终点 → 到达检测', () => {
    const pts = [];
    for (let i = 0; i <= 20; i++) pts.push({ x: px(i * 5), y: px(0) });
    const ref = buildFrenetReference(pts, false);
    const route: Route = {
      waypoints: ref.points, waypointIndices: [],
      startIndex: 0, goalIndex: 20, length: px(ref.totalS),
    };
    const car = makeCar(90, 0, 0, 80);
    const traj = plan(makeSnapshot(car), makeClearLidar(), route);
    assert.strictEqual(traj.type, 'arrived');
  });

  it('路线中途 → 轨迹正常且未到达', () => {
    const pts = [];
    for (let i = 0; i <= 100; i++) pts.push({ x: px(i * 10), y: px(0) });
    const ref = buildFrenetReference(pts, false);
    const route: Route = {
      waypoints: ref.points, waypointIndices: [],
      startIndex: 0, goalIndex: 100, length: px(ref.totalS),
    };
    const car = makeCar(300, 0, 0, 180);
    const traj = plan(makeSnapshot(car), makeClearLidar(), route);
    assert.notStrictEqual(traj.type, 'arrived');
    assert.ok(traj.points.length > 0);
  });
});
