// ==================== 测试：报告生成 ====================
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { verdictOf, buildReportMd } from '../../headless/report.js';

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
