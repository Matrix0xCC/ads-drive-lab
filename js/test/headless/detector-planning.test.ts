// ==================== 测试：检测器 —— 规划行为类 ====================
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { Detector } from '../../headless/detector.js';
import { FrameRecord } from '../../shared/types.js';
import { m, px } from '../../shared/dimensions.js';

function straightTrack() {
  const pts: any[] = [];
  for (let i = 0; i <= 200; i++) pts.push({ x: px(i * 10), y: px(0) });
  pts.push(pts[0]);
  return { centerPoints: pts, innerBoundary: [], outerBoundary: [] };
}
const event = (step: number, kind: string, payload: any = {}): FrameRecord =>
  ({ type: 'event', t: step / 60, step, kind, payload });

describe('planning: state_flapping', () => {
  it('1s 窗口内状态切换 ≥ 4 次 → issue', () => {
    const det = new Detector({}, { track: straightTrack() as any, obstacles: [] });
    // 每 10 step 切换一次，共 5 次（40 个 step 内）
    for (let i = 0; i < 5; i++) {
      det.feed(event(10 + i * 10, 'state_change',
        { from: i % 2 ? 'AVOID' : 'CRUISE', to: i % 2 ? 'CRUISE' : 'AVOID' }));
    }
    assert.ok(det.finish().find(i => i.check === 'state_flapping'));
  });

  it('偶发一次切换 → 不报', () => {
    const det = new Detector({}, { track: straightTrack() as any, obstacles: [] });
    det.feed(event(10, 'state_change', { from: 'CRUISE', to: 'AVOID' }));
    assert.strictEqual(det.finish().find(i => i.check === 'state_flapping'), undefined);
  });
});

describe('planning: close_call', () => {
  it('最近命中 <10m 且速度 >5m/s → issue；含紧急制动事件单独记录', () => {
    const det = new Detector({}, { track: straightTrack() as any, obstacles: [] });
    det.feed({ type: 'step', t: 0.1, step: 1,
      car: { x: px(0), y: px(0), angle: 0, speed: 250, steerAngle: 0 }, control: null });
    det.feed({ type: 'perception', t: 0.1, step: 1,
      scan: { angles: [], distances: [], rawDistances: [], confidences: [] },
      stats: { rawHits: 1, filteredHits: 1, frontDist: 100, minDist: px(30) } });  // 7.5m
    det.feed(event(2, 'emergency_brake', { frontDist: 30 }));
    const issues = det.finish();
    assert.ok(issues.find(i => i.check === 'close_call'));
    assert.ok(issues.find(i => i.check === 'emergency_brake'));
  });
});

describe('planning: lateral_switching', () => {
  it('15 个规划帧内最优 targetL 大幅摆动 ≥3 次 → issue', () => {
    const det = new Detector({}, { track: straightTrack() as any, obstacles: [] });
    const targets = [0, m(28), 0, m(-28), 0].map(v => v as number);  // ±7m 跳变
    targets.forEach((tL, i) => det.feed({ type: 'plan', t: i / 15, step: i * 4, state: 'AVOID',
      frenet: { s: 0, l: 0, dl_ds: 0, headingDiff: 0 }, candidates: [],
      best: { cost: 1, targetL: tL, targetV: 100 },
      trajectory: { points: [], type: 'avoid_left', cost: 1 } }));
    assert.ok(det.finish().find(i => i.check === 'lateral_switching'));
  });
});
