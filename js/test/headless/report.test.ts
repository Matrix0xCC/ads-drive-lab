// ==================== 测试：报告生成 ====================
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { verdictOf, buildReportMd, buildSummary } from '../../headless/report.js';
import { Detector } from '../../headless/detector.js';

describe('verdictOf', () => {
  it('无 issue → PASS；有 high → ISSUES 且计数正确', () => {
    assert.deepStrictEqual(verdictOf([]).verdict, 'PASS');
    const v = verdictOf([
      { category: 'safety', severity: 'high', check: 'collision_true', firstFrame: 1, lastFrame: 2, count: 2, summary: 'x', details: {}, samples: [1] },
      { category: 'perception', severity: 'low', check: 'gps_error', firstFrame: 3, lastFrame: 3, count: 1, summary: 'y', details: {}, samples: [3] },
    ]);
    assert.strictEqual(v.verdict, 'ISSUES');
    assert.deepStrictEqual(v.counts, { high: 1, medium: 0, low: 1 });
  });
});

describe('buildSummary：收敛迭代指标', () => {
  it('多轮帧占比与 planMs 分位（缺字段的旧记录按 1 轮 / 不计入耗时统计）', () => {
    // rounds 来自逐帧归档（确定量）；planMs 来自运行期统计（墙钟，非确定量，
    // 不进 frames.jsonl——否则破坏"同种子逐字节一致"契约）
    const det = new Detector();
    const plan = (step: number, revised?: boolean) => ({
      type: 'plan', t: step / 60, step, state: 'CRUISE',
      frenet: { s: 0, l: 0, dl_ds: 0, headingDiff: 0 }, candidates: [],
      best: { cost: 0, targetL: 0, targetV: 0 },
      trajectory: { points: [], type: 'lane_keep', cost: 0 },
      ...(revised !== undefined ? { revised } : {}),
    }) as any;
    det.feed(plan(0, false));
    det.feed(plan(1, true));
    det.feed(plan(2, true));
    det.feed(plan(3));                            // 旧归档：无 revised → 按 false
    const s = buildSummary(det, { steps: 4, simTime: 4 / 60, planMs: [1, 2, 3] });
    assert.strictEqual(s.multiRoundPct, 50, '4 帧中 2 帧 >1 轮');
    assert.strictEqual(s.planMsP50, 2);
    assert.strictEqual(s.planMsP95, 3);
    // 无耗时样本（旧调用方）→ null，不崩
    const s2 = buildSummary(det, { steps: 4, simTime: 4 / 60 });
    assert.strictEqual(s2.planMsP50, null);
    assert.strictEqual(s2.planMsP95, null);
  });
});

describe('buildReportMd', () => {
  it('包含判定行、问题列表、指标表与钻取指引', () => {
    const md = buildReportMd({
      meta: { id: 'r1', scenario: 's', map: 'oval', seed: 42, durationS: 60, steps: 3600,
        verdict: 'ISSUES', issueCounts: { high: 1, medium: 0, low: 0 },
        gitCommit: 'abc1234', nodeVersion: 'v18.19.1', createdAt: '', version: 1 } as any,
      issues: [{ category: 'safety', severity: 'high', check: 'collision_true',
        firstFrame: 431, lastFrame: 458, count: 28, summary: '撞上障碍', details: {}, samples: [431] }],
      summary: { steps: 3600, simSeconds: 60, speed: { maxMps: 25, avgMps: 20 },
        arrivedAtStep: null, trackingRmseM: 0.2, minObstacleDistM: 1.2,
        stateCounts: { CRUISE: 100 }, emergencyBrakes: 0, boundsResets: 0 } as any,
      scenario: { name: 's', map: 'oval', seed: 42, duration: 60, version: 1,
        obstacles: { mode: 'random', count: 4 } } as any,
    });
    assert.ok(md.includes('ISSUES'));
    assert.ok(md.includes('collision_true'));
    assert.ok(md.includes('431'));
    assert.ok(md.includes('grep'));
  });
});
