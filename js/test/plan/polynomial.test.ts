// ==================== 测试：五次多项式求解器 ====================

import { describe, it } from 'node:test';
import assert from 'node:assert';
import { solveQuintic, solveSpeedProfile } from '../../algo/plan/polynomial.js';
import { px, pxps, pxps2, dimless, perpx, secs } from '../../shared/dimensions.js';

describe('solveQuintic', () => {
  it('边界条件：l0=0→lf=100，Sf=200，所有导数为0', () => {
    const poly = solveQuintic(
      px(0), dimless(0), perpx(0),
      px(100), dimless(0), perpx(0),
      200,
    );
    assert.ok(Math.abs(poly.eval(0) as number) < 1e-6);
    assert.ok(Math.abs(poly.eval(200) as number - 100) < 1e-6);
    assert.ok(Math.abs(poly.deriv1(0) as number) < 1e-6);
    assert.ok(Math.abs(poly.deriv1(200) as number) < 1e-6);
    assert.ok(Math.abs(poly.deriv2(0) as number) < 1e-6);
    assert.ok(Math.abs(poly.deriv2(200) as number) < 1e-6);
  });

  it('单调递增', () => {
    const poly = solveQuintic(
      px(0), dimless(0), perpx(0),
      px(100), dimless(0), perpx(0),
      200,
    );
    const mid = poly.eval(100) as number;
    assert.ok(mid > 0 && mid < 100);
  });

  it('全零输入 → 全零输出', () => {
    const poly = solveQuintic(
      px(0), dimless(0), perpx(0),
      px(0), dimless(0), perpx(0),
      200,
    );
    for (let s = 0; s <= 200; s += 40) {
      assert.ok(Math.abs(poly.eval(s) as number) < 1e-6);
    }
  });

  it('退化 Sf≈0 不产生 NaN', () => {
    const poly = solveQuintic(
      px(0), dimless(0), perpx(0),
      px(0), dimless(0), perpx(0),
      1e-6,
    );
    assert.ok(!isNaN(poly.eval(0) as number));
  });
});

describe('solveSpeedProfile', () => {
  it('边界条件 s\'(0)=100, s\'(2)=200, s(0)=0', () => {
    const poly = solveSpeedProfile(pxps(100), pxps2(0), pxps(200), secs(2.0));
    assert.ok(Math.abs(poly.deriv1(0) - 100) < 2);
    assert.ok(Math.abs(poly.deriv1(2.0) - 200) < 2);
    assert.ok(Math.abs(poly.eval(0)) < 2);
  });

  it('总行程 ≈ 平均速度 × 时间', () => {
    const poly = solveSpeedProfile(pxps(100), pxps2(0), pxps(200), secs(2.0));
    const dist = poly.eval(2.0);
    assert.ok(Math.abs(dist - 300) < 30);
  });

  it('匀速巡航行程 ≈ speed × time', () => {
    const poly = solveSpeedProfile(pxps(120), pxps2(0), pxps(120), secs(2.0));
    assert.ok(Math.abs(poly.eval(2.0) - 240) < 15);
    assert.ok(Math.abs(poly.deriv1(1.0) - 120) < 10);
  });
});
