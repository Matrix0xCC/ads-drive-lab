// ==================== 规划器安全不变量测试 ====================
//
// A. 不变量：速度、时间、碰撞（与障碍物同位面）
// B. 边界：极端障碍物配置
// C. 模糊：60 帧随机场景，验证轨迹点不穿透障碍物

import { describe, it } from 'node:test';
import assert from 'node:assert';
import { plan } from '../../algo/plan/index.js';
import { debug } from '../../shared/debug.js';
import { CarState, WorldSnapshot, LidarScan } from '../../shared/types.js';
import { rad, m, mps, mps2, px, pxps, pxps2, dimless } from '../../shared/dimensions.js';

// ─── 辅助 ───

function makeCar(x: number, y: number, angle: number, speed: number): CarState {
  return { x: px(x), y: px(y), angle: rad(angle), speed: pxps(speed), steerAngle: rad(0), maxSpeed: mps(42.5), acceleration: mps2(8), friction: dimless(0.12), width: m(2.0), length: m(4.5), collisionRadius: px(Math.hypot(m(2.0) as number, m(4.5) as number) / 2) };
}

function makeSnapshot(car: CarState): WorldSnapshot {
  const pts: any[] = [];
  for (let i = 0; i <= 200; i++) pts.push({ x: px(i * 10), y: px(0) });
  pts.push(pts[0]);
  return { car, track: { centerPoints: pts, innerBoundary: [], outerBoundary: [] }, obstacles: [], timestamp: 0 };
}

function lidarWithObstacle(obsAngle: number, obsDist: number, obsWidth: number): LidarScan {
  const n = 61, angles: any[] = [], distances: any[] = [];
  const halfArc = Math.atan2(obsWidth / 2, obsDist) + 0.02;
  for (let i = 0; i < n; i++) {
    const a = -Math.PI / 3 + (i / (n - 1)) * 2 * Math.PI / 3;
    angles.push(rad(a));
    // 障碍物遮挡范围内的射线距离 = 到障碍物中心的距离（近似）
    distances.push(px(Math.abs(a - obsAngle) < halfArc ? obsDist : 600));
  }
  const rawDist = [...distances];
  return { angles, distances, rawDistances: rawDist, confidences: new Array(61).fill(1), timestamp: 0 };
}

function lidarClear(): LidarScan { return lidarWithObstacle(0, 600, 1); }

/** 检查轨迹点 (px, py) 是否在障碍物后方（穿透了障碍物） */
function isBehindObstacle(px: number, py: number, carX: number, carY: number, obsAngle: number, obsDist: number, obsWidth: number): boolean {
  const dx = px - carX, dy = py - carY;
  const d = Math.hypot(dx, dy);
  if (d < 5) return false;
  const a = Math.atan2(dy, dx);
  const halfArc = Math.atan2(obsWidth / 2, obsDist) + 0.02;
  return Math.abs(a - obsAngle) < halfArc && d > obsDist + (m(2.5) as number + 10);
}

// ═══════════════════════════════════════════════════
// A. 不变量
// ═══════════════════════════════════════════════════

describe('不变量', () => {
  it('正前方 15m 障碍（可行驶域内不可避让）→ 降级为最低速度目标且不出域', () => {
    // 25m/s 下 15m 内的横向避让物理上不可行；正确行为是最低速度最小化碰撞能量，
    // 而不是借道界外"绕开"（旧行为，见 ISSUE_HISTORY 2026-09-19 第二根因）
    const car = makeCar(500, 0, 0, 100);  // 100 sim-units = 25 m/s
    const lidar = lidarWithObstacle(0, 60, 10);  // 15m ahead, 2.5m wide
    const traj = plan(makeSnapshot(car), lidar);
    assert.strictEqual(debug.planning!.bestTargetV, mps(17.5) as number, '不可避让时应降级到最低速度');
    for (const pt of traj.points) {
      assert.ok(Math.abs(pt.y as number) <= 30, `轨迹点出域: y=${pt.y}`);  // 直线赛道 l≈y，半宽 30
    }
  });

  it('正前方 50m 障碍（可避让）→ 轨迹不穿障碍物', () => {
    const car = makeCar(500, 0, 0, 100);
    const lidar = lidarWithObstacle(0, 200, 10);  // 50m ahead, 2.5m wide
    const traj = plan(makeSnapshot(car), lidar);
    for (const pt of traj.points) {
      const behind = isBehindObstacle(pt.x as number, pt.y as number, car.x as number, car.y as number, 0, 200, 10);
      assert.ok(!behind, `轨迹点穿过障碍物`);
    }
  });

  it('无障碍 → 轨迹非空且类型合理', () => {
    const traj = plan(makeSnapshot(makeCar(500, 0, 0, 120)), lidarClear());
    assert.ok(traj.points.length > 0, '轨迹非空');
    assert.ok(traj.type !== 'emergency', '无障碍不应急');
  });

  it('轨迹点速度不超 maxSpeed=170', () => {
    const traj = plan(makeSnapshot(makeCar(500, 0, 0, 150)), lidarClear());
    for (const pt of traj.points) {
      assert.ok((pt.v as number) <= 170, `速度 ${(pt.v as number).toFixed(0)} > 170`);
    }
  });

  it('轨迹点时间严格递增', () => {
    const traj = plan(makeSnapshot(makeCar(500, 0, 0, 120)), lidarClear());
    for (let i = 1; i < traj.points.length; i++) {
      assert.ok((traj.points[i].t as number) > (traj.points[i - 1].t as number), '时间乱序');
    }
  });
});

// ═══════════════════════════════════════════════════
// B. 边界
// ═══════════════════════════════════════════════════

describe('边界', () => {
  it('障碍物贴车头 20px → 不应崩溃', () => {
    const car = makeCar(500, 0, 0, 60);
    const traj = plan(makeSnapshot(car), lidarWithObstacle(0, 20, 15));
    assert.ok(traj.type !== undefined, '有输出');
  });

  it('两侧对称 60px 障碍夹击 → 不崩溃', () => {
    const car = makeCar(500, 0, 0, 80);
    const n = 61; const angles: any[] = [], distances: any[] = [];
    for (let i = 0; i < n; i++) {
      const a = -Math.PI / 3 + (i / (n - 1)) * 2 * Math.PI / 3;
      angles.push(rad(a));
      const blocked = Math.abs(a + 0.52) < 0.1 || Math.abs(a - 0.52) < 0.1;
      distances.push(px(blocked ? 60 : 600));
    }
    const traj = plan(makeSnapshot(car), { angles, distances, rawDistances: distances, confidences: new Array(61).fill(1), timestamp: 0 });
    assert.ok(traj.type !== undefined, '有输出');
  });

  it('障碍物在轨迹末端 200px → 不触发紧急', () => {
    const traj = plan(makeSnapshot(makeCar(500, 0, 0, 180)), lidarWithObstacle(0, 200, 10));
    assert.notStrictEqual(traj.type, 'emergency', '200px 不应紧急');
  });
});

// ═══════════════════════════════════════════════════
// C. 模糊测试 — 这是关键：随机场景零穿透
// ═══════════════════════════════════════════════════

describe('模糊', () => {
  // 模糊测试需要真实 LiDAR 模拟（射线-线段求交），暂用固定场景的边界测试覆盖
  it('TODO: 用真实感知模块 sense() 驱动的模糊测试');
});
