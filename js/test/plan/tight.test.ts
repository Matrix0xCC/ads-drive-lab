// ==================== 测试：紧间隙迟滞状态机（TightState） ====================
//
// 施密特触发 + 3 帧连续防抖：以物理阈值 m 为中心、±0.15m 粘滞带——
// 进入 SEALED 需"更松走廊（m−0.15）也无候选"，退出需"通过更严走廊（m+0.15）"。
// 带内状态保持；切换需目标态连续 3 帧一致。

import { describe, it } from 'node:test';
import assert from 'node:assert';
import { nextTight, tightTarget, TightInput, TightMemory, INITIAL_TIGHT } from '../../algo/plan/tight.js';

const NONE_IN: TightInput = { hasComfortClear: true, hasHardClear: true, hasDeepClear: true, hasDeepBlocked: false };
const CREEP_IN: TightInput = { hasComfortClear: false, hasHardClear: true, hasDeepClear: true, hasDeepBlocked: false };
const BAND_IN: TightInput = { hasComfortClear: false, hasHardClear: true, hasDeepClear: false, hasDeepBlocked: false };
const SEAL_IN: TightInput = { hasComfortClear: false, hasHardClear: false, hasDeepClear: false, hasDeepBlocked: true };

/** 连续喂入 n 帧同一输入，返回末态 */
function feed(mem: TightMemory, input: TightInput, n: number): TightMemory {
  let m = mem;
  for (let i = 0; i < n; i++) m = nextTight(m, input);
  return m;
}

describe('tightTarget（施密特触发）', () => {
  it('基础：舒适畅通 → NONE；仅物理可过 → CREEPING；确实过不去（更松走廊也无候选）→ SEALED', () => {
    assert.strictEqual(tightTarget('NONE', NONE_IN), 'NONE');
    assert.strictEqual(tightTarget('NONE', CREEP_IN), 'CREEPING');
    assert.strictEqual(tightTarget('NONE', SEAL_IN), 'SEALED');
  });

  it('迟滞带（更松有候选、更严无候选）：两侧都保持原状态（不翻转）', () => {
    assert.strictEqual(tightTarget('NONE', BAND_IN), 'CREEPING', '未封死侧：带内进入谨慎档、不封死');
    assert.strictEqual(tightTarget('CREEPING', BAND_IN), 'CREEPING');
    assert.strictEqual(tightTarget('SEALED', BAND_IN), 'SEALED', '已封死侧：带内保持封死、不恢复');
  });

  it('退出 SEALED：通过更严走廊（m+0.15）→ 恢复；仅标称通过（带内）→ 仍封死', () => {
    assert.strictEqual(tightTarget('SEALED', CREEP_IN), 'CREEPING', '更严走廊也过 → 恢复');
    assert.strictEqual(tightTarget('SEALED', NONE_IN), 'NONE');
    assert.strictEqual(tightTarget('SEALED', BAND_IN), 'SEALED', '仅标称通过不够，需更严通过');
  });
});

describe('nextTight（安全优先：升级立即、降级 3 帧确证）', () => {
  it('升级（→SEALED）立即生效（安全动作不等待）', () => {
    const m = feed(INITIAL_TIGHT, SEAL_IN, 1);
    assert.strictEqual(m.state, 'SEALED', '单帧确凿封死即封');
  });

  it('降级需目标态连续 3 帧一致（第 2 帧仍在等待）', () => {
    let m = feed(INITIAL_TIGHT, SEAL_IN, 1);           // 已封
    m = feed(m, CREEP_IN, 2);
    assert.strictEqual(m.state, 'SEALED', '2 帧清晰不放行');
    assert.strictEqual(m.pending, 'CREEPING');
    m = feed(m, CREEP_IN, 1);
    assert.strictEqual(m.state, 'CREEPING', '第 3 帧恢复');
    assert.strictEqual(m.pending, null);
  });

  it('抖动序列（封死与带内交替）→ 一旦封死即保持（降级需连续清晰）', () => {
    let m: TightMemory = INITIAL_TIGHT;
    let sawSeal = false;
    for (let step = 0; step < 40; step++) {
      m = nextTight(m, step % 2 === 0 ? SEAL_IN : BAND_IN);
      if (m.state === 'SEALED') sawSeal = true;
      assert.notStrictEqual(m.state, 'NONE', `step ${step} 不应在抖动中回到 NONE`);
    }
    assert.ok(sawSeal, '抖动含封死帧 → 已进入 SEALED 并保持');
  });

  it('长期运行后的短干扰：升级立即、恢复仍需 3 帧', () => {
    let m = feed(INITIAL_TIGHT, NONE_IN, 50);      // 长期 NONE
    m = feed(m, SEAL_IN, 1);
    assert.strictEqual(m.state, 'SEALED', '短干扰立即封');
    m = feed(m, NONE_IN, 2);
    assert.strictEqual(m.state, 'SEALED', '2 帧清晰仍保持封死');
    m = feed(m, NONE_IN, 1);
    assert.strictEqual(m.state, 'NONE', '3 帧清晰恢复');
  });

  it('确定性：同序列两次结果一致', () => {
    const run = () => {
      let m: TightMemory = INITIAL_TIGHT;
      const out: string[] = [];
      for (let i = 0; i < 12; i++) {
        m = nextTight(m, i < 6 ? SEAL_IN : NONE_IN);
        out.push(m.state);
      }
      return out;
    };
    assert.deepStrictEqual(run(), run());
  });
});
