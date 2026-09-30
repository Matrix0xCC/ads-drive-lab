// ==================== 测试：本体坐标系近场判据 ====================
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { bodyFrameOffset, nearFieldBlocking, NEAR_FIELD, NEAR_BAND_MARGIN } from '../../algo/plan/bodyFrame.js';
import { m } from '../../shared/dimensions.js';

const car = (angle: number) => ({ x: 0, y: 0, angle });

describe('本体坐标系：bodyFrameOffset', () => {
  it('车头朝 +x：正前方 = ahead>0 lat≈0；正左方 = lat>0', () => {
    const f = bodyFrameOffset(car(0), m(10) as number, 0);
    assert.ok(Math.abs(f.ahead - (m(10) as number)) < 1e-9 && Math.abs(f.lat) < 1e-9);
    const l = bodyFrameOffset(car(0), 0, m(3) as number);
    assert.ok(Math.abs(l.lat - (m(3) as number)) < 1e-9 && Math.abs(l.ahead) < 1e-9);
  });

  it('车头朝 +y（90°）：正前方仍是 ahead>0（跟着航向转）', () => {
    const f = bodyFrameOffset(car(Math.PI / 2), 0, m(10) as number);
    assert.ok(Math.abs(f.ahead - (m(10) as number)) < 1e-9, '前方在 +y 方向');
    const r = bodyFrameOffset(car(Math.PI / 2), m(3) as number, 0);   // +x 方向在朝 +y 时是"右"
    assert.ok(r.lat < 0 && Math.abs(Math.abs(r.lat) - (m(3) as number)) < 1e-9);
  });

  it('八个方向一致（含斜向）', () => {
    for (let k = 0; k < 8; k++) {
      const ang = (k * Math.PI) / 4;
      const f = bodyFrameOffset(car(ang), Math.cos(ang) * (m(5) as number), Math.sin(ang) * (m(5) as number));
      assert.ok(Math.abs(f.ahead - (m(5) as number)) < 1e-6, `朝向 ${k * 45}° 时正前方 ahead 应为 5m`);
      assert.ok(Math.abs(f.lat) < 1e-6);
    }
  });
});

describe('本体坐标系近场判据（只加约束）', () => {
  const halfW = m(1.0) as number;
  it('近处、正前方、横向 0.5m → 算我车头带内（要减速）', () => {
    assert.ok(nearFieldBlocking(car(0), m(10) as number, m(0.5) as number, halfW, NEAR_BAND_MARGIN, NEAR_FIELD));
  });
  it('近处、但横向 3.75m（相邻车道）→ 不算我车头带内 ✓（幽灵刹车的判据）', () => {
    assert.ok(!nearFieldBlocking(car(0), m(10) as number, m(3.75) as number, halfW, NEAR_BAND_MARGIN, NEAR_FIELD));
  });
  it('超出近场距离 → 交给道路坐标系判（本函数不管）', () => {
    assert.ok(!nearFieldBlocking(car(0), m(40) as number, 0, halfW, NEAR_BAND_MARGIN, NEAR_FIELD));
  });
  it('在我后方 → 不算（后向传感器接入后靠这条排除）', () => {
    assert.ok(!nearFieldBlocking(car(0), -m(10) as number, 0, halfW, NEAR_BAND_MARGIN, NEAR_FIELD));
  });
});
