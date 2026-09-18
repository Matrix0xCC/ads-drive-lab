// ==================== 测试：Frenet 坐标系 ====================

import { describe, it } from 'node:test';
import assert from 'node:assert';
import { buildFrenetReference, cartesianToFrenet, frenetToCartesian } from '../../algo/plan/frenet.js';
import { Point2D, FrenetReference } from '../../shared/types.js';
import { px } from '../../shared/dimensions.js';

function makeCircleRef(cx: number, cy: number, r: number, n: number): FrenetReference {
  const pts: Point2D[] = [];
  for (let i = 0; i < n; i++) {
    const a = (i / n) * Math.PI * 2;
    pts.push({ x: px(cx + Math.cos(a) * r), y: px(cy + Math.sin(a) * r) });
  }
  pts.push(pts[0]);
  return buildFrenetReference(pts, true);
}

describe('buildFrenetReference', () => {
  it('闭环 totalS 约等于圆周长', () => {
    const ref = makeCircleRef(500, 500, 200, 100);
    assert.strictEqual(ref.closedLoop, true);
    assert.ok(ref.totalS > 1200 && ref.totalS < 1300);
  });

  it('开环 closedLoop=false', () => {
    const pts: Point2D[] = [];
    for (let i = 0; i <= 50; i++) pts.push({ x: px(i * 10), y: px(0) });
    const ref = buildFrenetReference(pts, false);
    assert.strictEqual(ref.closedLoop, false);
    assert.ok(Math.abs(ref.totalS - 500) < 5);
  });
});

describe('cartesianToFrenet', () => {
  const ref = makeCircleRef(500, 500, 200, 100);

  it('中心线上点 l≈0', () => {
    const result = cartesianToFrenet(px(700), px(500), ref);
    assert.ok(Math.abs(result.l as number) < 2);
    assert.ok(result.idx >= 0 && result.idx < ref.points.length);
  });

  it('内侧点 l>0', () => {
    const inner = cartesianToFrenet(px(660), px(500), ref);
    assert.ok((inner.l as number) > 0, '法向量指向圆心，内侧 l 应为正');
  });

  it('外侧点 l<0', () => {
    const outer = cartesianToFrenet(px(740), px(500), ref);
    assert.ok((outer.l as number) < 0);
  });
});

describe('frenetToCartesian', () => {
  const ref = makeCircleRef(500, 500, 200, 100);

  it('往返转换误差 <5px', () => {
    const points = [{ x: 700, y: 500 }, { x: 500, y: 300 }];
    for (const tp of points) {
      const fl = cartesianToFrenet(px(tp.x), px(tp.y), ref);
      const back = frenetToCartesian(fl.s, fl.l, ref);
      assert.ok(Math.abs(back.x as number - tp.x) < 5);
      assert.ok(Math.abs(back.y as number - tp.y) < 5);
    }
  });

  it('闭环 s=0 和 s=totalS 重合', () => {
    const p0 = frenetToCartesian(0, px(0), ref);
    const p1 = frenetToCartesian(ref.totalS, px(0), ref);
    assert.ok(Math.abs(p0.x as number - p1.x as number) < 3);
  });

  it('开环 s 超出范围应 clamp', () => {
    const pts: Point2D[] = [];
    for (let i = 0; i <= 50; i++) pts.push({ x: px(i * 10), y: px(0) });
    const openRef = buildFrenetReference(pts, false);
    const pEnd = frenetToCartesian(openRef.totalS + 100, px(0), openRef);
    assert.ok(Math.abs(pEnd.x as number - 500) < 3, 's 超范围 clamp 到终点');
    const pStart = frenetToCartesian(-100, px(0), openRef);
    assert.ok(Math.abs(pStart.x as number - 0) < 3, 's 负值 clamp 到起点');
  });
});
