// ==================== 测试：归档写入 ====================
import { describe, it } from 'node:test';
import assert from 'node:assert';
import * as fs from 'node:fs';
import * as os from 'node:os';
import * as path from 'node:path';
import { ArchiveWriter, updateRunsIndex } from '../../headless/archive.js';

function tmpDir(): string {
  return fs.mkdtempSync(path.join(os.tmpdir(), 'ads-archive-'));
}

describe('ArchiveWriter', () => {
  it('写入记录 → 读回逐行 JSON 一致', () => {
    const dir = tmpDir();
    const w = new ArchiveWriter(dir);
    w.appendRecord({ type: 'step', t: 0.0167, step: 1,
      car: { x: 1, y: 2, angle: 0.5, speed: 100, steerAngle: 0 },
      control: { lookaheadDist: 40, alpha: 0.1, steerAngle: 0.1, throttle: 0.5 } });
    w.appendRecord({ type: 'event', t: 0.0167, step: 1, kind: 'arrived', payload: {} });
    const lines = fs.readFileSync(path.join(dir, 'frames.jsonl'), 'utf8').trim().split('\n');
    assert.strictEqual(lines.length, 2);
    assert.strictEqual(JSON.parse(lines[0]).type, 'step');
    assert.strictEqual(JSON.parse(lines[1]).kind, 'arrived');
  });

  it('finalize 写出 issues/summary/report 并把条目并入 index.json', () => {
    const root = tmpDir();
    const dir = path.join(root, 'run1');
    const w = new ArchiveWriter(dir);
    w.finalize({
      issues: [], summary: { steps: 10, simSeconds: 0.17 } as any,
      reportMd: '# 报告\nPASS', meta: { id: 'run1', verdict: 'PASS' } as any,
    });
    assert.ok(fs.existsSync(path.join(dir, 'issues.json')));
    assert.ok(fs.existsSync(path.join(dir, 'summary.json')));
    assert.ok(fs.existsSync(path.join(dir, 'report.md')));
    updateRunsIndex(root, { id: 'run1', scenario: 't', map: 'oval', seed: 1, createdAt: 'x',
      verdict: 'PASS', issues: { high: 0, medium: 0, low: 0 }, durationS: 60 });
    const idx = JSON.parse(fs.readFileSync(path.join(root, 'index.json'), 'utf8'));
    assert.strictEqual(idx.runs.length, 1);
    assert.strictEqual(idx.runs[0].id, 'run1');
  });

  it('index.json 同 id 二次写入 → 更新而非重复', () => {
    const root = tmpDir();
    updateRunsIndex(root, { id: 'a', verdict: 'PASS' } as any);
    updateRunsIndex(root, { id: 'a', verdict: 'ISSUES' } as any);
    const idx = JSON.parse(fs.readFileSync(path.join(root, 'index.json'), 'utf8'));
    assert.strictEqual(idx.runs.length, 1);
    assert.strictEqual(idx.runs[0].verdict, 'ISSUES');
  });
});
