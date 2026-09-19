// ==================== 测试：CLI 端到端确定性 ====================
import { describe, it } from 'node:test';
import assert from 'node:assert';
import * as fs from 'node:fs';
import * as os from 'node:os';
import * as path from 'node:path';
import { execFileSync } from 'node:child_process';

const CLI = path.resolve('dist/js/headless/run.js');

function runCli(outDir: string, extra: string[] = []): void {
  execFileSync('node', [CLI, '--map', 'oval', '--obstacles', '4', '--seed', '7',
    '--duration', '5', '--out', outDir, ...extra], { stdio: 'pipe' });
}

describe('headless CLI E2E', () => {
  it('同场景同种子两次运行 → frames.jsonl 逐字节一致', () => {
    const a = fs.mkdtempSync(path.join(os.tmpdir(), 'ads-e2e-a-'));
    const b = fs.mkdtempSync(path.join(os.tmpdir(), 'ads-e2e-b-'));
    runCli(path.join(a, 'run'));
    runCli(path.join(b, 'run'));
    const fa = fs.readFileSync(path.join(a, 'run', 'frames.jsonl'), 'utf8');
    const fb = fs.readFileSync(path.join(b, 'run', 'frames.jsonl'), 'utf8');
    assert.ok(fa.length > 0);
    assert.strictEqual(fa, fb);
  });

  it('产出 run.json / world.json / frames.jsonl 且 meta 含种子', () => {
    const a = fs.mkdtempSync(path.join(os.tmpdir(), 'ads-e2e-c-'));
    runCli(path.join(a, 'run'));
    const meta = JSON.parse(fs.readFileSync(path.join(a, 'run', 'run.json'), 'utf8'));
    assert.strictEqual(meta.seed, 7);
    assert.ok(fs.existsSync(path.join(a, 'run', 'world.json')));
  });
});
