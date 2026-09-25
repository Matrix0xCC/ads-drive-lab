// ==================== 开发服务器（静态文件 + 运行归档清理接口） ====================
//
// 【为什么需要它】浏览器页面无法删除服务器磁盘文件（页面 JS 沙箱只能发 HTTP
// 请求），而 python -m http.server 是无写接口的纯静态服务——「清理运行记录」
// 按钮因此需要一个浏览器可调用的删除端点。本服务器用 Node 实现，替代原
// python http.server 的用法（从仓库根目录启动、按 URL 路径服务文件）：
//
//   GET  <path>        静态文件（相对仓库根目录；不缓存，改动即生效）
//   DELETE /api/runs   清空 runs/ 归档（无参数、不可越出 runs/ 目录）
//
// 启动：npm run serve（默认 127.0.0.1:8080；PORT 环境变量可改）

import http from 'node:http';
import fs from 'node:fs';
import path from 'node:path';

const MIME: Record<string, string> = {
  '.html': 'text/html; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.mjs': 'text/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.json': 'application/json; charset=utf-8',
  '.jsonl': 'application/x-ndjson; charset=utf-8',
  '.map': 'application/json; charset=utf-8',
  '.png': 'image/png',
  '.jpg': 'image/jpeg',
  '.svg': 'image/svg+xml',
  '.ico': 'image/x-icon',
};

export interface DevServerOptions {
  /** 静态文件根目录（惯例：仓库根，与 python http.server 一致） */
  root: string;
  /** 运行归档目录（清理接口的作用域；测试时可指向临时目录） */
  runsDir: string;
}

export function createDevServer(o: DevServerOptions): http.Server {
  const root = path.resolve(o.root);
  const runsDir = path.resolve(o.runsDir);

  return http.createServer((req, res) => {
    const urlPath = decodeURIComponent((req.url ?? '/').split('?')[0]);

    if (urlPath === '/api/runs') {
      if (req.method === 'DELETE') return clearRuns(runsDir, res);
      return json(res, 405, { ok: false, error: '仅支持 DELETE' });
    }
    if (req.method !== 'GET' && req.method !== 'HEAD') {
      return json(res, 405, { ok: false, error: '仅支持 GET/HEAD/DELETE /api/runs' });
    }
    serveStatic(root, urlPath, res, req.method === 'HEAD');
  });
}

/** 清空 runs/ 目录（含 index.json；重跑 sim 会自动重建） */
function clearRuns(runsDir: string, res: http.ServerResponse): void {
  try {
    let deleted = 0;
    if (fs.existsSync(runsDir)) {
      for (const entry of fs.readdirSync(runsDir)) {
        fs.rmSync(path.join(runsDir, entry), { recursive: true, force: true });
        deleted++;
      }
    }
    json(res, 200, { ok: true, deleted });
  } catch (e) {
    json(res, 500, { ok: false, error: (e as Error).message });
  }
}

function serveStatic(root: string, urlPath: string, res: http.ServerResponse, headOnly: boolean): void {
  const rel = urlPath === '/' ? 'index.html' : urlPath.replace(/^\/+/, '');
  const file = path.resolve(root, rel);
  // 路径越界保护：解码后的路径不得逃出根目录（防 %2e%2e/ 之类）
  if (file !== root && !file.startsWith(root + path.sep)) {
    return json(res, 403, { ok: false, error: 'forbidden' });
  }
  fs.stat(file, (err, st) => {
    if (err || !st.isFile()) return json(res, 404, { ok: false, error: 'not found' });
    res.writeHead(200, {
      'Content-Type': MIME[path.extname(file).toLowerCase()] ?? 'application/octet-stream',
      'Content-Length': st.size,
      'Cache-Control': 'no-store',   // 开发服务器：禁缓存，改代码刷新即生效
    });
    if (headOnly) return res.end();
    fs.createReadStream(file).pipe(res);
  });
}

function json(res: http.ServerResponse, status: number, body: unknown): void {
  const text = JSON.stringify(body);
  res.writeHead(status, { 'Content-Type': 'application/json; charset=utf-8', 'Content-Length': Buffer.byteLength(text) });
  res.end(text);
}

// ── 入口（从仓库根目录启动：npm run serve） ──
if (process.argv[1] && import.meta.url.endsWith(path.basename(process.argv[1]))) {
  const root = process.cwd();
  const port = Number(process.env.PORT ?? 8080);
  createDevServer({ root, runsDir: path.join(root, 'runs') }).listen(port, '127.0.0.1', () => {
    console.log(`开发服务器: http://localhost:${port}  （静态根: ${root}）`);
    console.log('  DELETE /api/runs 可用（「回放运行」面板的清理按钮）');
  });
}
