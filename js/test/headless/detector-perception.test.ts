// ==================== 测试：检测器 —— 感知类 ====================
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { Detector } from '../../headless/detector.js';
import { FrameRecord } from '../../shared/types.js';
import { m, px, rad } from '../../shared/dimensions.js';

function straightTrack() {
  const pts: any[] = [];
  for (let i = 0; i <= 200; i++) pts.push({ x: px(i * 10), y: px(0) });
  pts.push(pts[0]);
  return { centerPoints: pts, innerBoundary: [], outerBoundary: [] };
}
const OBST = { x: px(200), y: px(0), width: m(2), height: m(4), angle: rad(0) };

// 61 条射线：angle = -60°..60°（步进 2°），默认全 MAX_SENSOR_DIST
function scanRec(step: number, carX: number, carY: number, hits: Record<number, number>): FrameRecord {
  const n = 61, angles: any[] = [], distances: any[] = [], confs: number[] = [];
  for (let i = 0; i < n; i++) {
    const a = (-60 + i * 2) * Math.PI / 180;
    angles.push(rad(a));
    const d = hits[i] !== undefined ? px(hits[i]) : px(500);
    distances.push(d); confs.push(hits[i] !== undefined ? 1 : 0);
  }
  return { type: 'perception', t: step / 20, step,
    scan: { angles, distances, rawDistances: [...distances], confidences: confs }, stats: null };
}

// 车辆 step 记录（safety 检查先行更新 lastStepRec）
function carAt(step: number, x: number, y: number): FrameRecord {
  return { type: 'step', t: step / 60, step,
    car: { x, y, angle: 0, speed: 100, steerAngle: 0 }, control: null };
}

describe('perception: phantom_hit', () => {
  it('过滤后命中点附近无真值障碍 → issue', () => {
    const det = new Detector({}, { track: straightTrack() as any, obstacles: [] });
    // 车在原点朝 +x；第 30 条射线 = 0°（正前方），报 100px 处有障碍，但真值无障碍
    det.feed(carAt(1, 0, 0));
    det.feed(scanRec(1, 0, 0, { 30: 100 }));
    det.feed(carAt(2, 0, 0));
    det.feed(scanRec(2, 0, 0, { 30: 100 }));
    const iss = det.finish().find(i => i.check === 'phantom_hit');
    assert.ok(iss, '未检出幻影命中');
  });

  it('命中点附近有真值障碍 → 不报', () => {
    const det = new Detector({}, { track: straightTrack() as any, obstacles: [OBST] });
    // 车在 x=100, 障碍物在 x=200 → 0° 射线 100px 命中真值障碍
    det.feed(carAt(1, 100, 0));
    det.feed(scanRec(1, 100, 0, { 30: 100 }));
    assert.strictEqual(det.finish().find(i => i.check === 'phantom_hit'), undefined);
  });
});

describe('perception: missed_obstacle', () => {
  it('真值障碍在 FOV 内却无命中 → issue', () => {
    const det = new Detector({}, { track: straightTrack() as any, obstacles: [OBST] });
    det.feed(carAt(1, 100, 0));
    det.feed(scanRec(1, 100, 0, {}));   // 前方 100px 有障碍，所有射线却报 500
    assert.ok(det.finish().find(i => i.check === 'missed_obstacle'));
  });

  it('障碍在 FOV 外（车后）→ 不报', () => {
    const det = new Detector({}, { track: straightTrack() as any, obstacles: [OBST] });
    det.feed(carAt(1, 500, 0));
    det.feed(scanRec(1, 500, 0, {}));   // 障碍在车后
    assert.strictEqual(det.finish().find(i => i.check === 'missed_obstacle'), undefined);
  });
});

describe('perception: gps_error', () => {
  it('误差 > 3σ → issue', () => {
    const det = new Detector({}, { track: straightTrack() as any, obstacles: [] });
    const gps: FrameRecord = { type: 'gps', t: 0.1, step: 1, x: px(500), y: px(0), err: m(5) };
    det.feed(gps);
    assert.ok(det.finish().find(i => i.check === 'gps_error'));
  });
});
