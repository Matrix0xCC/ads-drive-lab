// ==================== 测试：前车速度估计 + 跟驰（方案 C） ====================
//
// 这是 C 的核心判据：**同一套公式要能把"静态墙"和"移动前车"分开**——
// 对静态墙，接近速率 = 自车自己的速度 ⟹ 估计出的前车速度 ≈ 0 ⟹ 老老实实停车；
// 对移动前车，扣掉自车贡献后剩下的就是它的速度 ⟹ 应该跟到它的速度，而不是停住等它走。
import { describe, it, beforeEach } from 'node:test';
import assert from 'node:assert';
import { updateLead, resetClosing, SCAN_HZ, FRONT_HALF_ANGLE } from '../../algo/plan/closing.js';
import { solveSpeedLayer } from '../../algo/plan/speed.js';
import { mps, m as M } from '../../shared/dimensions.js';

/** 造一帧"前方 dx 处有一堵墙"的雷达（61 条射线、±60°，只有正前方那条命中） */
function frame(gapM: number, carAngle = 0): { angles: number[]; distances: number[] } {
  const angles: number[] = [], distances: number[] = [];
  for (let k = 0; k < 61; k++) {
    const a = (-60 + k * 2) * (Math.PI / 180);
    angles.push(a);
    // 正前方 ±2° 的射线打到墙上，距离 = gap（别的方向视为量程外）
    distances.push(Math.abs(a) <= 2 * (Math.PI / 180) ? M(gapM) : M(120));
  }
  return { angles, distances };
}

describe('前车速度估计（雷达时序差分）', () => {
  beforeEach(() => resetClosing());

  it('**静态墙**：自车 20m/s 接近一堵不动的墙 → 估计出的前车速度 ≈ 0（会走停车线）', () => {
    const car = { angle: 0, speed: mps(20) as number };
    updateLead(frame(100), car);                       // 第一帧（无速度信息）
    const est = updateLead(frame(100 - 20 / SCAN_HZ), car);   // 一帧后：距离减少 20m/s ÷ 20Hz = 1m
    assert.ok(est.leadSpeed !== null, '第二帧应该给出速度估计');
    assert.ok(Math.abs(est.leadSpeed) < mps(1.5),
      `静态墙的前车速度应 ≈0，实际 ${(est.leadSpeed / 4).toFixed(2)} m/s`);
  });

  it('**移动前车**：前车 15m/s 同向、自车 20m/s → 估计 ≈ 15m/s（会走跟驰）', () => {
    const car = { angle: 0, speed: mps(20) as number };
    updateLead(frame(100), car);
    // 相对接近 5m/s → 一帧后距离减少 5/20 = 0.25m
    const est = updateLead(frame(100 - 5 / SCAN_HZ), car);
    assert.ok(est.leadSpeed !== null);
    assert.ok(Math.abs(est.leadSpeed - mps(15)) < mps(2),
      `前车速度应 ≈15m/s，实际 ${(est.leadSpeed / 4).toFixed(2)} m/s`);
  });

  it('前方无物 → gap=null（不参与跟驰）', () => {
    const est = updateLead(frame(120), { angle: 0, speed: mps(20) as number });
    assert.strictEqual(est.gap, null);
  });

  it('同一帧雷达被调用两次 → 结果一致（15Hz 规划 / 20Hz 雷达，重复帧不能退化成"没信息"）', () => {
    const car = { angle: 0, speed: mps(20) as number };
    updateLead(frame(100), car);
    const f = frame(100 - 5 / SCAN_HZ);
    const a = updateLead(f, car);
    const b = updateLead(f, car);                       // 同一对象
    assert.strictEqual(a.leadSpeed, b.leadSpeed);
    assert.ok(a.leadSpeed !== null, '重复帧必须仍带速度信息');
  });
});

describe('速度层：跟驰 vs 停车线（方案 C 的分派）', () => {
  const blocked = [{ sMin: M(50) as number, sMax: M(60) as number, tMin: 0, tMax: Infinity }];

  it('前车在动且阻塞点就是它 → **不停车**，跟到它的速度附近', () => {
    const r = solveSpeedLayer({
      stObstacles: blocked, v0: mps(25) as number, targetV: mps(30) as number,
      follow: { gap: M(50) as number, leadSpeed: mps(15) as number },
    });
    assert.strictEqual(r.mode, 'cruise', '跟驰时不该进入停车模式');
    // ⚠ 注意：`profile.eval(t)` 是**位移**（s），速度是 `profile.deriv1(t)` —— 我第一版把 eval 当速度读，错了。
    // 50m 间距、最小车距 6m、时距 1.4s → 安全速度 = 15 + 44/1.4 ≈ 46 m/s → 被 targetV=30 压住
    const v2 = r.profile.deriv1(1.9);
    assert.ok(v2 > mps(10), `2 秒时不该停住（实际 ${(v2 / 4).toFixed(1)} m/s）`);
  });

  it('间距小于最小车距 → 安全速度降到前车速度之下（自动拉开距离）', () => {
    // 注意：阻塞点必须与 follow.gap **吻合**才会走跟驰分支（"阻塞来自这辆会走的车"的判据）
    const r = solveSpeedLayer({
      stObstacles: [{ sMin: M(3) as number, sMax: M(13) as number, tMin: 0, tMax: Infinity }],
      v0: mps(25) as number, targetV: mps(30) as number,
      follow: { gap: M(3) as number, leadSpeed: mps(15) as number },
    });
    const v2 = r.profile.deriv1(1.9);
    assert.ok(v2 < mps(15), `间距不足时应低于前车速度，实际 ${(v2 / 4).toFixed(1)} m/s`);
  });

  it('前车基本静止（≤1.5m/s）→ 仍走停车线（**历史行为不变**的保险）', () => {
    // 150m 处：25m/s 有足够距离舒适停住（应急阈值 ≈ 98m），所以应当走 stop 而不是 emergency
    const r = solveSpeedLayer({
      stObstacles: [{ sMin: M(150) as number, sMax: M(160) as number, tMin: 0, tMax: Infinity }],
      v0: mps(25) as number, targetV: mps(30) as number,
      follow: { gap: M(150) as number, leadSpeed: mps(0.5) as number },
    });
    assert.strictEqual(r.mode, 'stop', '慢到静止的前车应当按静态障碍处理（停车线）');
  });

  it('没有 follow（纯静态世界）→ 与改动前完全一致', () => {
    const a = solveSpeedLayer({ stObstacles: blocked, v0: mps(25) as number, targetV: mps(30) as number });
    const b = solveSpeedLayer({ stObstacles: blocked, v0: mps(25) as number, targetV: mps(30) as number, follow: undefined });
    assert.strictEqual(a.mode, b.mode);
    for (const t of [0, 0.5, 1, 1.5, 2]) assert.strictEqual(a.profile.eval(t), b.profile.eval(t));
  });
});
