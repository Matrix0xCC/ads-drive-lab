// ==================== 测试：开发服务器（静态 + 清理归档接口） ====================
import { describe, it, before, after } from 'node:test';
import assert from 'node:assert';
import fs from 'node:fs';
import http from 'node:http';
import os from 'node:os';
import path from 'node:path';
import { createDevServer } from '../../dev-server.js';

/** 原始路径请求（fetch 会在客户端规范化 ../，测越界防护必须绕开） */
function rawGet(port: number, rawPath: string): Promise<number> {
  return new Promise((resolve, reject) => {
    const req = http.request({ host: '127.0.0.1', port, path: rawPath, method: 'GET' },
      res => { res.resume(); resolve(res.statusCode!); });
    req.on('error', reject);
    req.end();
  });
}

describe('开发服务器', () => {
  let tmpRoot: string, tmpRuns: string, srv: any, base: string;

  before(async () => {
    tmpRoot = fs.mkdtempSync(path.join(os.tmpdir(), 'ads-dev-srv-'));
    tmpRuns = path.join(tmpRoot, 'runs');
    // 构造两个"运行归档" + 一个静态文件
    for (const id of ['run-a', 'run-b']) {
      fs.mkdirSync(path.join(tmpRuns, id), { recursive: true });
      fs.writeFileSync(path.join(tmpRuns, id, 'run.json'), '{}');
    }
    fs.writeFileSync(path.join(tmpRuns, 'index.json'), '{"runs":[]}');
    fs.writeFileSync(path.join(tmpRoot, 'hello.txt'), 'hi');
    srv = createDevServer({ root: tmpRoot, runsDir: tmpRuns });
    await new Promise<void>(r => srv.listen(0, '127.0.0.1', () => r()));
    base = `http://127.0.0.1:${srv.address().port}`;
  });

  after(async () => {
    await new Promise(r => srv.close(r));
    fs.rmSync(tmpRoot, { recursive: true, force: true });
  });

  it('GET 静态文件；路径越界（编码的 ../）被拒绝', async () => {
    const res = await fetch(`${base}/hello.txt`);
    assert.strictEqual(res.status, 200);
    assert.strictEqual(await res.text(), 'hi');

    const outside = path.join(os.tmpdir(), `ads-outside-${Date.now()}.txt`);
    fs.writeFileSync(outside, 'secret');
    try {
      const status = await rawGet(srv.address().port, `/%2e%2e/${path.basename(outside)}`);
      assert.strictEqual(status, 403, '越界路径应被拒绝');
    } finally {
      fs.rmSync(outside, { force: true });
    }
  });

  it('不存在/目录路径 → 404', async () => {
    assert.strictEqual((await fetch(`${base}/nope.txt`)).status, 404);
    assert.strictEqual((await fetch(`${base}/runs`)).status, 404);
  });

  it('DELETE /api/runs 清空归档目录（含 index.json）', async () => {
    const res = await fetch(`${base}/api/runs`, { method: 'DELETE' });
    assert.strictEqual(res.status, 200);
    const j: any = await res.json();
    assert.strictEqual(j.ok, true);
    assert.ok(j.deleted >= 3, `应删除 3 项（2 目录 + index.json），实际 ${j.deleted}`);
    assert.deepStrictEqual(fs.readdirSync(tmpRuns), []);
    // 重复清理（目录已空）不报错
    const again = await fetch(`${base}/api/runs`, { method: 'DELETE' });
    assert.strictEqual(again.status, 200);
  });
});
