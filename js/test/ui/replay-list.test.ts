// ==================== 测试：回放运行列表拉取（不得吃 HTTP 缓存） ====================
//
// 回归背景（浏览器实测）：runs/index.json 是可变资源（新增运行、清理归档都会变），
// 若用默认缓存模式拉取，清理后重开面板会拿到缓存里的旧列表（需整页刷新才恢复）。

import { describe, it } from 'node:test';
import assert from 'node:assert';
import { ReplayController } from '../../replay.js';

describe('ReplayController.listRuns', () => {
  it('以 no-store 拉取 runs/index.json（清理由此保证实时）', async () => {
    const calls: { url: string; init: any }[] = [];
    const origFetch = globalThis.fetch;
    (globalThis as any).fetch = async (url: string, init: any) => {
      calls.push({ url, init });
      return { ok: true, json: async () => ({ runs: [{ id: 'r1', scenario: 's', map: 'oval', seed: 1, createdAt: '', verdict: 'PASS', issues: {} }] }) };
    };
    try {
      const runs = await ReplayController.listRuns();
      assert.strictEqual(calls.length, 1);
      assert.strictEqual(calls[0].url, 'runs/index.json');
      assert.strictEqual(calls[0].init?.cache, 'no-store', '可变索引必须绕过 HTTP 缓存');
      assert.strictEqual(runs[0].id, 'r1');
    } finally {
      (globalThis as any).fetch = origFetch;
    }
  });
});
