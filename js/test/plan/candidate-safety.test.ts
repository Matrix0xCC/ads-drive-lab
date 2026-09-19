// ==================== 测试：evaluateTrajectory 安全语义 ====================
//
// 背景（ISSUE_HISTORY 2026-09-19 附带发现）：
//   1. 快速剪枝传 margin = SAFETY_MARGIN + collisionRadius，而代价评估只传
//      SAFETY_MARGIN —— 两处安全余量不一致，统一为参数传入
//   2. collision 标记曾用 `cost >= 1e6` 推断：碰撞罚分会被负的 progress
//      奖励拉回 1e6 以下 → 标记丢失 —— 改为返回显式 collided 布尔量
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { evaluateTrajectory } from '../../algo/plan/candidate.js';
import { buildFrenetReference } from '../../algo/plan/frenet.js';
import { MAX_SENSOR_DIST } from '../../algo/perception.js';
import { rad, px, pxps } from '../../shared/dimensions.js';

// 直线参考线（沿 x 轴），车在原点，正前方 100 单位处有障碍命中
function fixtures() {
  const pts: any[] = [];
  for (let i = 0; i <= 200; i++) pts.push({ x: px(i * 10), y: px(0) });
  pts.push(pts[0]);
  const ref = buildFrenetReference(pts, true);

  const n = 61;
  const angles: any[] = [], distances: any[] = [], rawDistances: any[] = [], confidences: number[] = [];
  for (let i = 0; i < n; i++) {
    const off = (-60 + i * 2) * Math.PI / 180;
    angles.push(rad(off));
    const hit = i === 30;
    distances.push(px(hit ? 100 : (MAX_SENSOR_DIST as number)));
    rawDistances.push(px(hit ? 100 : (MAX_SENSOR_DIST as number)));
    confidences.push(hit ? 1 : 0);
  }
  const scan = { angles, distances, rawDistances, confidences, timestamp: 0 };

  // 轨迹：从车起点沿 +x 前进到 150 单位（越过 100 处的障碍）
  const tp = (t: number, x: number, v: number) =>
    ({ t: t as any, x: px(x), y: px(0), yaw: rad(0), v: pxps(v), kappa: 0 as any });
  const points = [tp(0, 0, 100), tp(1, 150, 100)];
  return { ref, scan, points };
}

describe('evaluateTrajectory: margin 参数', () => {
  it('同一轨迹小 margin 判碰撞、大 margin 不判（证明参数生效）', () => {
    const { ref, scan, points } = fixtures();
    const small = evaluateTrajectory(points, ref, 0, 0, scan as any, 0, 40);   // 100+40 < 150
    const large = evaluateTrajectory(points, ref, 0, 0, scan as any, 0, 60);   // 100+60 > 150
    assert.strictEqual(small.collided, true);
    assert.strictEqual(large.collided, false);
  });
});

describe('evaluateTrajectory: collided 显式标记', () => {
  it('碰撞轨迹 collided=true，且 cost 可低于 1e6（旧 cost 阈值推断会丢失标记）', () => {
    const { ref, scan, points } = fixtures();
    const ev = evaluateTrajectory(points, ref, 0, 0, scan as any, 0, 40);
    assert.strictEqual(ev.collided, true);
    assert.ok(ev.cost < 1e6, `cost=${ev.cost} 应因负 progress 拉低到 1e6 以下`);
  });

  it('无碰撞轨迹 collided=false', () => {
    const { ref, scan, points } = fixtures();
    const ev = evaluateTrajectory(points, ref, 0, 0, scan as any, 0, 60);
    assert.strictEqual(ev.collided, false);
    assert.ok(ev.cost < 1e6);
  });
});
