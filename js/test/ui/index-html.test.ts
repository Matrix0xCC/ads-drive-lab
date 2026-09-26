// ==================== 测试：index.html 的结构约定（无 DOM 依赖，纯文本检查） ====================
//
// 为什么值得测：index.html 是应用里唯一的 HTML/CSS 载体，没有构建期校验——
// 一次把规则嵌进另一条规则（CSS 不支持嵌套）就让整块失效且无人报错（实测踩过：
// toolbar 样式被塞进 #hud-mini 里 → toolbar 与 HUD 一起丢样式）。

import { describe, it } from 'node:test';
import assert from 'node:assert';
import fs from 'node:fs';

const html = fs.readFileSync(`${process.cwd()}/index.html`, 'utf8');
const style = html.slice(html.indexOf('<style>') + 7, html.indexOf('</style>'));

/** 逐字符扫花括号：CSS 不允许规则嵌套 → 深度不得超过 1，且必须回到 0 */
function maxBraceDepth(css: string): { max: number; end: number } {
  let depth = 0, max = 0;
  for (const ch of css) {
    if (ch === '{') { depth += 1; max = Math.max(max, depth); }
    else if (ch === '}') depth -= 1;
  }
  return { max, end: depth };
}

describe('index.html：CSS 结构', () => {
  it('无嵌套规则（花括号深度 ≤1 且收支平衡）', () => {
    const { max, end } = maxBraceDepth(style);
    assert.strictEqual(max, 1, `存在嵌套规则（最大深度 ${max}）——CSS 不支持嵌套，整块会失效`);
    assert.strictEqual(end, 0, '花括号不平衡');
  });

  it('侧栏子树统一 border-box（防 width:100% + padding 溢出成横向滚动条）', () => {
    assert.ok(/#sidebar,\s*#sidebar \*\s*\{\s*box-sizing:\s*border-box/.test(style),
      '应有 #sidebar, #sidebar * { box-sizing: border-box }');
  });

  it('侧栏常展开：.sb-body 不再 display:none', () => {
    assert.ok(/\.sb-body\s*\{[^}]*display:\s*block/.test(style), '.sb-body 应常显');
    assert.ok(!/\.sb-body\s*\{[^}]*display:\s*none/.test(style), '不应残留手风琴的隐藏规则');
  });
});

describe('index.html：画布左缘 toolbar', () => {
  it('四个按钮都在 #canvas-toolbar 内，且旧的左侧栏「仿真」区已移除', () => {
    const bar = html.slice(html.indexOf('id="canvas-toolbar"'), html.indexOf('<!-- 精简遥测'));
    for (const id of ['btn-pause', 'btn-reset-view', 'btn-corridor', 'btn-frenet-ticks', 'btn-lidar', 'btn-route']) {
      assert.ok(bar.includes(`id="${id}"`), `toolbar 内应有 ${id}`);
    }
    assert.ok(!html.includes('menu-sim'), '手风琴的 menu-sim 应已删除');
    assert.ok(!html.includes('section-sim'), '手风琴的 section-sim 应已删除');
  });

  it('toolbar 定位在画布左缘（绝对定位 + 垂直居中）', () => {
    const m = style.match(/#canvas-toolbar\s*\{([^}]*)\}/);
    assert.ok(m, '应有 #canvas-toolbar 规则');
    assert.ok(/position:\s*absolute/.test(m![1]) && /left:/.test(m![1]) && /translateY\(-50%\)/.test(m![1]),
      `toolbar 应绝对定位且垂直居中，实际：${m![1].trim()}`);
  });
});
