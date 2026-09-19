// ==================== 测试：检测器 —— 安全类 ====================
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { Detector } from '../../headless/detector.js';
import { FrameRecord } from '../../shared/types.js';
import { m, px, rad } from '../../shared/dimensions.js';

// 直线赛道：x 轴 0..2000，l=0 为中心线（法向量 (0,1)，l>0 向内）
function straightTrack() {
  const pts: any[] = [];
  for (let i = 0; i <= 200; i++) pts.push({ x: px(i * 10), y: px(0) });
  pts.push(pts[0]);
  return { centerPoints: pts, innerBoundary: [], outerBoundary: [] };
}

function stepRec(step: number, x: number, y: number, speed = 250): FrameRecord {
  return { type: 'step', t: step / 60, step,
    car: { x, y, angle: 0, speed, steerAngle: 0 }, control: null };
}

describe('safety: collision_true', () => {
  it('车身圆与障碍物相交 → 出 issue（high）', () => {
    const det = new Detector({}, { track: straightTrack() as any, obstacles: [
      { x: px(120), y: px(0), width: m(2), height: m(4), angle: rad(0) } ] });
    for (let s = 1; s <= 30; s++) det.feed(stepRec(s, px(100 + s * 4), px(0))); // 逐步压上障碍
    const issues = det.finish();
    const col = issues.find(i => i.check === 'collision_true');
    assert.ok(col, '未检测到碰撞');
    assert.strictEqual(col!.severity, 'high');
    assert.ok(col!.firstFrame >= 1);
  });

  it('远离障碍物 → 无碰撞 issue', () => {
    const det = new Detector({}, { track: straightTrack() as any, obstacles: [
      { x: px(120), y: px(500), width: m(2), height: m(4), angle: rad(0) } ] });
    for (let s = 1; s <= 30; s++) det.feed(stepRec(s, px(100 + s * 4), px(0)));
    assert.strictEqual(det.finish().find(i => i.check === 'collision_true'), undefined);
  });
});

describe('safety: off_track', () => {
  it('|l| 超半宽 → issue；中线附近 → 无', () => {
    const t = straightTrack();
    const off = new Detector({}, { track: t as any, obstacles: [] });
    off.feed(stepRec(1, px(100), px(m(9))));   // l > 7.5m
    assert.ok(off.finish().find(i => i.check === 'off_track'));

    const ok = new Detector({}, { track: t as any, obstacles: [] });
    ok.feed(stepRec(1, px(100), px(0)));
    assert.strictEqual(ok.finish().find(i => i.check === 'off_track'), undefined);
  });
});

describe('safety: emergency_state', () => {
  it('EMERGENCY 规划状态区间 → issue（medium）', () => {
    const det = new Detector({}, { track: straightTrack() as any, obstacles: [] });
    const plan = (step: number, state: string): FrameRecord => ({ type: 'plan', t: step / 60, step, state,
      frenet: { s: 0, l: 0, dl_ds: 0, headingDiff: 0 }, candidates: [],
      best: { cost: 0, targetL: 0, targetV: 0 },
      trajectory: { points: [], type: 'emergency', cost: 0 } });
    for (let s = 1; s <= 3; s++) det.feed(plan(s, 'EMERGENCY'));
    for (let s = 4; s <= 10; s++) det.feed(plan(s, 'CRUISE'));
    const issues = det.finish();
    assert.ok(issues.find(i => i.check === 'emergency_state'));
    assert.strictEqual(issues.find(i => i.check === 'emergency_state')!.count, 3);
  });
});
