// ==================== 测试：可播种随机数 ====================
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { createRng, deriveSeed } from '../../shared/rng.js';

describe('createRng', () => {
  it('同种子 → 同序列（前 5 个值全等）', () => {
    const a = createRng(42), b = createRng(42);
    for (let i = 0; i < 5; i++) assert.strictEqual(a(), b());
  });

  it('不同种子 → 不同序列', () => {
    const a = createRng(42), b = createRng(43);
    assert.notStrictEqual(a(), b());
  });

  it('输出在 [0, 1) 区间', () => {
    const rng = createRng(7);
    for (let i = 0; i < 1000; i++) {
      const v = rng();
      assert.ok(v >= 0 && v < 1, `越界值 ${v}`);
    }
  });
});

describe('deriveSeed', () => {
  it('同参数 → 同结果；不同 label → 不同结果', () => {
    assert.strictEqual(deriveSeed(42, 'gps'), deriveSeed(42, 'gps'));
    assert.notStrictEqual(deriveSeed(42, 'gps'), deriveSeed(42, 'obstacles'));
  });

  it('与派生顺序无关（先派生 gps 再 obstacles，结果不变）', () => {
    const g1 = deriveSeed(42, 'gps');
    deriveSeed(42, 'obstacles');
    assert.strictEqual(deriveSeed(42, 'gps'), g1);
  });
});
