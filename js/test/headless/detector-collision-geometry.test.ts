// ==================== 测试：碰撞判定的几何准确性（OBB-OBB） ====================
//
// 背景（2026-09-19 批量钻取 hairpin-s3）：collision_true 曾用车身对角线
// 外接圆（半径 2.46m）近似，侧向并行通过时把 1m 车半宽高估为 2.46m →
// 实测侧向间隙 2.08m 的正常通过被误报"碰撞"。改用 OBB-OBB（SAT）做
// 车体矩形与障碍矩形的真实接触判定。
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { Detector } from '../../headless/detector.js';
import { FrameRecord } from '../../shared/types.js';
import { px, rad } from '../../shared/dimensions.js';

function straightTrack() {
  const pts: any[] = [];
  for (let i = 0; i <= 200; i++) pts.push({ x: px(i * 10), y: px(0) });
  pts.push(pts[0]);
  return { centerPoints: pts, innerBoundary: [], outerBoundary: [] };
}

function stepRec(step: number, x: number, y: number, angle = 0): FrameRecord {
  return { type: 'step', t: step / 60, step,
    car: { x, y, angle, speed: 100, steerAngle: 0 }, control: null };
}

describe('collision_true 几何判定（OBB-OBB）', () => {
  it('侧向并行通过（表面距离 2.16m）→ 不判碰撞', () => {
    const det = new Detector({}, { track: straightTrack() as any, obstacles: [
      // 车在原点朝 +x（车体 18×8 单位）；障碍（12×8）平行置于侧方：
      // 中心距 12.64 → 表面距离 8.64（2.16m）、物理间隙 0.64u
      // 旧圆近似：半径 9.85 > 8.64 → 误报；OBB：y 区间 [4] 与 [8.64,16.64] 分离
      { x: px(0), y: px(12.64), width: px(12), height: px(8), angle: rad(0) } ] });
    det.feed(stepRec(1, 0, 0));
    assert.strictEqual(det.finish().find(i => i.check === 'collision_true'), undefined,
      '侧向通过不应判碰撞');
  });

  it('横置车体（angle=90°）与侧方障碍（物理间隙 1.25m）→ 不判碰撞', () => {
    // 车横置：x 向占 [-4,4]；障碍 x∈[9,21] → 分离，间隙 5u=1.25m
    // 旧圆近似：表面距离 9 < 9.85 → 误报
    const det = new Detector({}, { track: straightTrack() as any, obstacles: [
      { x: px(15), y: px(0), width: px(12), height: px(8), angle: rad(0) } ] });
    det.feed(stepRec(1, 0, 0, Math.PI / 2));
    assert.strictEqual(det.finish().find(i => i.check === 'collision_true'), undefined,
      '横置侧方通过不应判碰撞');
  });

  it('正前方实体相交 → 判碰撞（正例对照）', () => {
    const det = new Detector({}, { track: straightTrack() as any, obstacles: [
      { x: px(12), y: px(0), width: px(12), height: px(8), angle: rad(0) } ] });
    det.feed(stepRec(1, 0, 0));
    assert.ok(det.finish().find(i => i.check === 'collision_true'), '相交应判碰撞');
  });

  it('车头斜向贴入障碍角落 → 判碰撞（旋转几何正例）', () => {
    // 车 45° 朝向，障碍中心 (10, 10)：两者角部相交
    const det = new Detector({}, { track: straightTrack() as any, obstacles: [
      { x: px(10), y: px(10), width: px(12), height: px(8), angle: rad(0) } ] });
    det.feed(stepRec(1, 0, 0, Math.PI / 4));
    assert.ok(det.finish().find(i => i.check === 'collision_true'), '角部相交应判碰撞');
  });
});
