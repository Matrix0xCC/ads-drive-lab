// ==================== 测试：检测器 —— 跟踪/任务类 ====================
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { Detector } from '../../headless/detector.js';
import { buildSummary } from '../../headless/report.js';
import { FrameRecord } from '../../shared/types.js';
import { m, px } from '../../shared/dimensions.js';

function straightTrack() {
  const pts: any[] = [];
  for (let i = 0; i <= 200; i++) pts.push({ x: px(i * 10), y: px(0) });
  pts.push(pts[0]);
  return { centerPoints: pts, innerBoundary: [], outerBoundary: [] };
}
// 车沿 +x 直线；轨迹预测 60px/s 前进
const traj = (step: number): FrameRecord => ({ type: 'plan', t: step / 60, step, state: 'FOLLOW',
  frenet: { s: 0, l: 0, dl_ds: 0, headingDiff: 0 }, candidates: [],
  best: { cost: 1, targetL: 0, targetV: 60 },
  trajectory: { points: [{ t: 0, x: 0, y: 0, v: 60 }, { t: 2, x: 120, y: 0, v: 60 }], type: 'lane_keep', cost: 1 } });
const stepRec = (step: number, x: number, y: number, speed = 60, steer = 0): FrameRecord =>
  ({ type: 'step', t: step / 60, step, car: { x, y, angle: 0, speed, steerAngle: steer }, control: null });

describe('mission: tracking_error', () => {
  it('实际位置偏离上一规划轨迹 > 1m → issue', () => {
    const det = new Detector({}, { track: straightTrack() as any, obstacles: [] });
    det.feed(stepRec(1, 0, 0));
    det.feed(traj(2));                       // 规划：60px/s 前进
    det.feed(stepRec(6, 30, px(m(3) as number)));  // 横向偏离轨迹线 3m
    det.feed(traj(6));                       // 下一规划帧 → 触发与上一轨迹的比较
    assert.ok(det.finish().find(i => i.check === 'tracking_error'));
  });

  it('跟得上规划 → 不报', () => {
    const det = new Detector({}, { track: straightTrack() as any, obstacles: [] });
    det.feed(stepRec(1, 0, 0));
    det.feed(traj(2));
    det.feed(stepRec(6, 6, 0));              // 在轨迹线上 ✓
    det.feed(traj(6));
    assert.strictEqual(det.finish().find(i => i.check === 'tracking_error'), undefined);
  });

  it('纯纵向偏移（规划起点 Frenet 量化伪影）→ 不误报', () => {
    // 规划起点 s0 量化到最近采样点（间距 ~2.9m），按时间插值的旧口径
    // 会把沿路径的 ~2m 纵向偏移计成"跟踪误差"——物理上车辆仍在路径上
    const det = new Detector({}, { track: straightTrack() as any, obstacles: [] });
    det.feed(stepRec(1, 0, 0));
    det.feed(traj(2));
    det.feed(stepRec(6, 12.8, 0));           // 沿轨迹线纵向超前 2.2m（y 仍为 0）
    det.feed(traj(6));
    assert.strictEqual(det.finish().find(i => i.check === 'tracking_error'), undefined,
      '沿路径的纵向偏移不应判跟踪误差');
  });
});

describe('mission: stop_anomaly / stalled', () => {
  it('有路线且速度 <0.5m/s 持续 3s → stalled issue', () => {
    const det = new Detector({}, { track: straightTrack() as any, obstacles: [] });
    det.routeActive = true;
    for (let s = 1; s <= 190; s++) det.feed(stepRec(s, 0, 0, 0));
    const iss = det.finish();
    assert.ok(iss.find(i => i.check === 'stalled'));
  });

  it('到达后 1s 内速度仍 >1m/s → stop_anomaly', () => {
    const det = new Detector({}, { track: straightTrack() as any, obstacles: [] });
    det.feed({ type: 'event', t: 0.1, step: 6, kind: 'arrived', payload: {} });
    for (let s = 7; s <= 60; s++) det.feed(stepRec(s, 0, 0, px(80)));  // 20m/s
    assert.ok(det.finish().find(i => i.check === 'stop_anomaly'));
  });
});

describe('summary: 到达用时与路线进度', () => {
  /** 指定沿路线弧长 frenet.s 的规划帧 */
  const planAt = (step: number, s: number): FrameRecord => ({
    ...traj(step), frenet: { s, l: 0, dl_ds: 0, headingDiff: 0 },
  } as FrameRecord);

  it('到达事件 → arrivalTimeS = step/60；progress = 最大弧长/路线长度（不倒退）', () => {
    const det = new Detector({}, { track: straightTrack() as any, obstacles: [] });
    det.routeLength = px(600);
    det.feed(planAt(60, 200));
    det.feed(planAt(120, 500));
    det.feed(planAt(180, 300));   // 弧长回落（如被挤回）→ 进度取历史最大
    det.feed({ type: 'event', t: 3, step: 180, kind: 'arrived', payload: {} });
    const s = buildSummary(det, { steps: 240, simTime: 4 });
    assert.strictEqual(s.arrivalTimeS, 3);
    assert.ok(Math.abs((s.progress as number) - 500 / 600) < 1e-9, `progress=${s.progress}`);
  });

  it('未到达 → arrivalTimeS 为 null；无路线 → progress 为 null', () => {
    const det = new Detector({}, { track: straightTrack() as any, obstacles: [] });
    det.feed(planAt(60, 200));
    const s = buildSummary(det, { steps: 120, simTime: 2 });
    assert.strictEqual(s.arrivalTimeS, null);
    assert.strictEqual(s.progress, null);
  });

  it('进度封顶 1（弧长超路线长度时）', () => {
    const det = new Detector({}, { track: straightTrack() as any, obstacles: [] });
    det.routeLength = px(600);
    det.feed(planAt(60, 700));
    const s = buildSummary(det, { steps: 120, simTime: 2 });
    assert.strictEqual(s.progress, 1);
  });
});
