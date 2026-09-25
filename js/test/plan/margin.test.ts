// ==================== 测试：误差感知裕度（marginFor，速度分段表） ====================
//
// 形式来自归档标定（docs/margin-calibration.md）：横向偏差实测**随速度下降**
// （低速机动段 ~1.0m，高速稳态 ~0.4m），原假设的 κ·v² 形式被数据否决（k=0）。
// 表：v = 0/5/10/15/25/35 → 1.15/1.0/0.85/0.75/0.65/0.6（线性插值，clamp [0.4, 1.2]；
// 该表为"达到留出集覆盖率 ≥95% 的最小表"，实测 95.7%）。

import { describe, it } from 'node:test';
import assert from 'node:assert';
import { marginFor, MARGIN_MIN, MARGIN_MAX, refCurvatureAt } from '../../algo/plan/margin.js';
import { buildFrenetReference } from '../../algo/plan/frenet.js';
import { px } from '../../shared/dimensions.js';

describe('marginFor（速度分段表）', () => {
  it('表节点：v=0 → 1.15；5 → 1.0；10 → 0.85；15 → 0.75；25 → 0.65；35 → 0.6', () => {
    assert.ok(Math.abs(marginFor(0) - 1.15) < 1e-9);
    assert.ok(Math.abs(marginFor(5) - 1.0) < 1e-9);
    assert.ok(Math.abs(marginFor(10) - 0.85) < 1e-9);
    assert.ok(Math.abs(marginFor(15) - 0.75) < 1e-9);
    assert.ok(Math.abs(marginFor(25) - 0.65) < 1e-9);
    assert.ok(Math.abs(marginFor(35) - 0.6) < 1e-9);
    assert.ok(Math.abs(marginFor(50) - 0.6) < 1e-9, '超出表尾取末值');
  });

  it('分段线性插值：v=7.5 → 0.925；v=12.5 → 0.8', () => {
    assert.ok(Math.abs(marginFor(7.5) - 0.925) < 1e-9, `实际 ${marginFor(7.5)}`);
    assert.ok(Math.abs(marginFor(12.5) - 0.8) < 1e-9, `实际 ${marginFor(12.5)}`);
  });

  it('随速度单调不增（低速更保守）', () => {
    let prev = Infinity;
    for (const v of [0, 2, 5, 7.5, 10, 12.5, 15, 20, 35]) {
      const m = marginFor(v);
      assert.ok(m <= prev + 1e-9, `v=${v} 时应不增：${m} > ${prev}`);
      prev = m;
    }
  });

  it('clamp 到 [0.4, 1.2]', () => {
    assert.ok(marginFor(100) >= MARGIN_MIN - 1e-9);
    assert.ok(marginFor(-5) <= MARGIN_MAX + 1e-9, '负速度按 0 处理');
  });
});

describe('refCurvatureAt（参考线曲率，标定工具口径）', () => {
  const arcRef = buildFrenetReference(
    Array.from({ length: 121 }, (_, i) => {
      const th = (i / 120) * Math.PI / 2;           // 1/4 圆弧，半径 100u
      return { x: px(100 + 100 * Math.sin(th)), y: px(100 - 100 * Math.cos(th)) };
    }), false);

  it('圆弧参考线：κ ≈ 1/R（半径 100u → 0.01 /u）', () => {
    const k = refCurvatureAt(arcRef, 100);
    assert.ok(Math.abs(k - 0.01) < 0.002, `实际 κ=${k}（期望≈0.01/u）`);
  });

  it('直线参考线：κ ≈ 0', () => {
    const line = buildFrenetReference(
      Array.from({ length: 61 }, (_, i) => ({ x: px(i * 10), y: px(0) })), false);
    assert.ok(refCurvatureAt(line, 200) < 1e-6);
  });
});
