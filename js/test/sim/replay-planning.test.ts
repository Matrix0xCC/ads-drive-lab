// ==================== 测试：回放帧 → 调试面板数据 ====================
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { replayPlanningDebug } from '../../replay.js';

const planRec = {
  type: 'plan', step: 580, state: 'DECEL',
  frenet: { s: 542.4, l: -13.4, dl_ds: -0.167, headingDiff: -0.162 },
  candidates: [
    { targetL: -17.6, targetV: 32, cost: 283.6, collision: true },
    { targetL: 19.2, targetV: 32, cost: 27.8, collision: true },
  ],
  best: { cost: 27.8, targetL: 19.2, targetV: 32 },
  trajectory: { points: [], type: 'avoid_left', cost: 1 },
};

describe('replayPlanningDebug（回放写回调试面板）', () => {
  it('不开走廊重算 → 用归档候选；字段来自归档记录；剩余距离 = 路线长 − s', () => {
    const d = replayPlanningDebug(planRec, null, 916);
    assert.strictEqual(d.source, 'replay');
    assert.strictEqual(d.state, 'DECEL');
    assert.strictEqual(d.paths.length, 2);
    assert.strictEqual(d.paths[0].blocked, true, '归档 collision → blocked');
    assert.strictEqual(d.bestTargetL, 19.2);
    assert.strictEqual(d.speedTarget, 32);
    assert.strictEqual(d.remainingDist, 916 - 542.4);
    assert.strictEqual(d.viz, null);
    assert.strictEqual(d.rounds, undefined, '实时专有字段不填 → 面板隐藏该行');
  });

  it('开走廊重算 → 候选与画布编号同源（用重算结果）；缺 plan 记录时不崩', () => {
    const viz: any = {
      bands: [], pinchSegments: [], pinch: null, bandObserved: [], obsLimitSegments: [],
      unobsBoundary: [], stepU: 6,
      candidates: [
        { targetL: -17.6, develop: 48, blocked: true, blockedHard: true, sBlock: 1, sBlockHard: 2, cost: 283.6, points: [] },
        { targetL: 7.5, develop: 48, blocked: false, blockedHard: false, sBlock: Infinity, sBlockHard: Infinity, cost: 3.2, points: [] },
      ],
      bestTargetL: 7.5,
    };
    const d = replayPlanningDebug(planRec, viz, 916);
    assert.strictEqual(d.paths.length, 2);
    assert.strictEqual(d.paths[1].targetL, 7.5);
    assert.strictEqual(d.bestTargetL, 7.5);
    assert.strictEqual(d.bestCost, 3.2, '最优代价取重算结果（与画布一致）');
    const empty = replayPlanningDebug(undefined, null, null);
    assert.strictEqual(empty.state, 'NONE');
    assert.deepStrictEqual(empty.paths, []);
    assert.strictEqual(empty.remainingDist, 0);
  });
});
