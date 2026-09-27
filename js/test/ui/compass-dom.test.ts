// ==================== 测试：罗盘的 DOM 契约（静态扫 index.html） ====================
//
// 【为什么需要】`js/ui/index.ts` 取元素全是 `getElementById('...')?.addEventListener(...)`
// —— **改名/删除都不会报错，只是罗盘悄悄失效**（点上去没反应）。这类"静默失联"只能靠契约测试守。
// 另外两条是 HTML 结构上的硬约束：
//   · 圆盘**不能是 <button>**：里面还有 4 个字母按钮，button 套 button 是非法 HTML，
//     浏览器会把内层按钮"提出去"，圆环布局会散；
//   · 每个字母必须带 data-bearing（JS 靠它决定转到哪个方位），且与 JS 的 N/E/S/W 表一一对应。
import { describe, it } from 'node:test';
import assert from 'node:assert';
import * as fs from 'node:fs';
import * as path from 'node:path';

const html = fs.readFileSync(path.resolve('index.html'), 'utf8');
const uiSrc = fs.readFileSync(path.resolve('js/ui/index.ts'), 'utf8');

describe('罗盘 DOM 契约（index.html ↔ ui/index.ts）', () => {
  it('JS 里引用的每个罗盘元素都存在', () => {
    for (const id of ['compass-dial', 'compass-ring', 'compass-bearing', 'btn-view-pitch']) {
      assert.ok(html.includes(`id="${id}"`), `index.html 缺少 #${id}（JS 里用的是静默的 ?.，会无声失效）`);
    }
    assert.ok(html.includes('class="compass-dir'), 'index.html 缺少 .compass-dir 字母按钮');
  });

  it('圆盘是 div（role=button），不是 <button> —— 否则内层字母按钮会被浏览器提出去', () => {
    assert.match(html, /<div id="compass-dial"[^>]*role="button"/, '圆盘应当是 div[role=button]');
    assert.ok(!/<button[^>]*id="compass-dial"/.test(html), '圆盘不能是 <button>：里面有嵌套按钮');
  });

  it('四个方位字母齐全，且 data-bearing 与 JS 的 N/E/S/W 表一致', () => {
    const bearings = [...html.matchAll(/class="compass-dir[^"]*"\s+data-bearing="(\d+)"/g)].map(m => Number(m[1]));
    assert.deepStrictEqual(bearings.slice().sort((a, b) => a - b), [0, 90, 180, 270], `实际 ${bearings.join(',')}`);
    for (const [letter, deg] of [['N', 0], ['E', 90], ['S', 180], ['W', 270]] as [string, number][]) {
      const re = new RegExp(`class="compass-dir[^"]*"\\s+data-bearing="${deg}"[^>]*>${letter}<`);
      assert.match(html, re, `字母 ${letter} 应对应 data-bearing="${deg}"`);
      assert.ok(uiSrc.includes(`${letter}: ${deg}`), `ui/index.ts 的 BEARINGS 表应有 ${letter}: ${deg}`);
    }
  });

  it('2D/3D 是**一个**切换按钮（用户要求合并；两个按钮还占一行高度）', () => {
    assert.ok(!html.includes('id="btn-view-2d"') && !html.includes('id="btn-view-3d"'),
      '不该再有分开的 2D/3D 两个按钮');
    assert.match(uiSrc, /toggledPitchMode\(viewPitchDeg\(camera, controls\.target\)\)/,
      '点击应当"切到另一边"（当前 2D → 去 3D），而不是固定设成某一个');
  });

  it('底部提示行不能压在罗盘下面（实测文字叠按钮）', () => {
    const compassLeft = Number(/#compass \{[^}]*left:\s*(\d+)px/.exec(html)?.[1] ?? NaN);
    const dialW = Number(/#compass-dial \{[^}]*width:\s*(\d+)px/.exec(html)?.[1] ?? NaN);
    const hintLeft = Number(/#hint-bar \{[^}]*left:\s*(\d+)px/.exec(html)?.[1] ?? NaN);
    assert.ok(Number.isFinite(compassLeft) && Number.isFinite(dialW) && Number.isFinite(hintLeft),
      `解析不到布局数值：compassLeft=${compassLeft} dialW=${dialW} hintLeft=${hintLeft}`);
    assert.ok(hintLeft >= compassLeft + dialW,
      `提示行 left=${hintLeft}px 必须让开罗盘右缘 ${compassLeft + dialW}px，否则两者会叠在一起`);
  });

  it('快捷键 N/2/3 已接线（否则按钮能用但键盘没反应）', () => {
    assert.match(uiSrc, /case 'n': case 'N':/, '缺少 N 键（正北）');
    assert.match(uiSrc, /case '2':/, '缺少 2 键（2D）');
    assert.match(uiSrc, /case '3':/, '缺少 3 键（3D）');
    assert.ok(!/case 'b': case 'B':[\s\S]{0,80}snapViewToBearing/.test(uiSrc), 'B 键应当仍是底图，没被罗盘占用');
  });
});
