// ==================== 测试：世界菜单三层（地点 / 场景 / 路线）的契约 ====================
//
// 【背景】重构前"地图"与"场景"两个下拉职责重叠：场景包含地图（scenario.map），却与地图并排成同级，
// 而"手工设路线"是第三个入口且没在 UI 上出现过 → 用户看不出"路线从哪来"。
// 重构后三层显式：地点（只换地理，**切换即清路线**）、场景（整个世界配方）、路线（来源+摘要+两个动作）。
//
// 这里守三类**静默失效**：JS 取元素是 `?.`（改名不报错）、下拉失败时的回滚、以及两条关键行为规则。
import { describe, it } from 'node:test';
import assert from 'node:assert';
import * as fs from 'node:fs';
import * as path from 'node:path';
import { reduce, INITIAL_UI_STATE } from '../../ui/state.js';

const html = fs.readFileSync(path.resolve('index.html'), 'utf8');
const mainSrc = fs.readFileSync(path.resolve('js/main.ts'), 'utf8');
const toolbarSrc = fs.readFileSync(path.resolve('js/ui/toolbar.ts'), 'utf8');

describe('世界菜单：DOM 契约', () => {
  it('路线行的元素都在（JS 取元素是静默的 ?.，改名会无声失效）', () => {
    for (const id of ['route-row', 'route-source', 'btn-route-pick', 'btn-route-clear', 'world-notice']) {
      assert.ok(html.includes(`id="${id}"`), `index.html 缺少 #${id}`);
    }
  });
  it('两个下拉的标签改成"地点 / 场景"，并用 title 说明各自管什么', () => {
    assert.match(html, /<label class="sb-label"[^>]*>地点<\/label>/, '地图下拉应改称「地点」');
    assert.match(html, /<label class="sb-label"[^>]*>场景<\/label>/);
    assert.match(html, /id="map-dropdown"[^>]*title="[^"]*清除[^"]*路线/, '地点下拉要写明"切换会清除路线"');
    assert.match(html, /id="scenario-dropdown"[^>]*title="[^"]*地点[^"]*交通[^"]*路线/, '场景下拉要写明它包含地点/交通/路线');
  });
});

describe('世界菜单：状态机（routeInfo / notice）', () => {
  it('ROUTE_INFO 更新来源与摘要；内容相同则状态不变（不产生多余副作用）', () => {
    const a = reduce(INITIAL_UI_STATE, { type: 'ROUTE_INFO', source: 'scenario', summary: '场景自带 · 节点 2→93 · 2.93km' });
    assert.deepStrictEqual(a.routeInfo, { source: 'scenario', summary: '场景自带 · 节点 2→93 · 2.93km' });
    const b = reduce(a, { type: 'ROUTE_INFO', source: 'scenario', summary: '场景自带 · 节点 2→93 · 2.93km' });
    assert.strictEqual(b, a, '内容相同必须返回同一对象（否则每帧都会写 DOM）');
    const c = reduce(a, { type: 'ROUTE_INFO', source: 'manual', summary: '手工设点 · 1.90km' });
    assert.strictEqual(c.routeInfo.source, 'manual');
  });
  it('NOTICE 设置与清空', () => {
    const a = reduce(INITIAL_UI_STATE, { type: 'NOTICE', text: '已切换到「西三环」；原路线属于上一张地图，已清除' });
    assert.match(a.notice, /已清除/);
    assert.strictEqual(reduce(a, { type: 'NOTICE', text: '' }).notice, '');
    assert.strictEqual(reduce(a, { type: 'NOTICE', text: a.notice }), a);
  });
  it('初始状态：无路线（车辆静止）', () => {
    assert.deepStrictEqual(INITIAL_UI_STATE.routeInfo, { source: 'none', summary: '无路线（车辆静止）' });
    assert.strictEqual(INITIAL_UI_STATE.notice, '');
  });
});

describe('世界菜单：两条关键行为规则（源码契约）', () => {
  it('换地点必须**丢弃旧路线声明**（节点号/弧长只对原地图有意义）', () => {
    // 静态检查是刻意的：这条规则只能在无 DOM 的环境里这样钉住（同 main-init-order.test.ts 的做法）
    assert.match(mainSrc, /const \{ route: _dropped, \.\.\.rest \} = base/,
      'switchMap 必须把 route 摘掉再套新地图');
    assert.match(mainSrc, /applyScenario\(\{ \.\.\.rest, map \}/, '换地点时不应把 route 带过去');
  });
  it('"随机场景"必须用地图 **key**（下拉值），不能用 mapDef.name（合成地图的 name 是显示名）', () => {
    assert.match(toolbarSrc, /loadDefaultScenario\(deps\.actions\.currentMapKey\(\)\)/,
      '随机场景要传 key；传显示名（Oval / Tech Circuit）会被 parseScenario 拒绝 → 弹错误框（实测踩过）');
    assert.ok(!/loadDefaultScenario\(deps\.engine\.mapDef\.name\)/.test(toolbarSrc), '不能再用显示名当 key');
  });
  it('下拉失败要回滚（否则"下拉已跳、世界没变"）', () => {
    assert.match(toolbarSrc, /mapSelect\.value = deps\.actions\.currentMapKey\(\)/, '地图下拉失败要回滚');
    assert.match(toolbarSrc, /scenarioSelect\.value = ''/, '场景下拉失败要回滚');
  });
});

// ─────────────────────────────────────────────────────────────
// 比例尺（底部常驻）+ 量距（可拖动的尺）
// ─────────────────────────────────────────────────────────────
describe('比例尺与量距：契约', () => {
  it('DOM 元素齐全（JS 取元素是静默的 ?.，改名会无声失效）', () => {
    for (const id of ['scale-bar', 'scale-bar-fill', 'scale-bar-label', 'measure-readout', 'btn-measure']) {
      assert.ok(html.includes(`id="${id}"`), `index.html 缺少 #${id}`);
    }
  });

  it('量距与设路线互斥（都吃左键；一边开、另一边按了没反应会让人困惑）', () => {
    const measuring = reduce(INITIAL_UI_STATE, { type: 'TOGGLE_MEASURE' });
    assert.strictEqual(measuring.measureMode, true);
    // 开量距会退出路线拾取
    const picking = reduce(INITIAL_UI_STATE, { type: 'TOGGLE_ROUTE_SET' });
    assert.strictEqual(picking.routePick, 'picking-start');
    const both = reduce(picking, { type: 'TOGGLE_MEASURE' });
    assert.strictEqual(both.measureMode, true);
    assert.strictEqual(both.routePick, 'idle', '开量距必须退出路线拾取');
    const back = reduce(both, { type: 'TOGGLE_ROUTE_SET' });
    assert.strictEqual(back.measureMode, false, '开始设路线必须退出量距');
  });

  it('量距模式内的左键不吃给物体拾取/路线点选（否则拖一次会顺带选中东西）', () => {
    const src = fs.readFileSync(path.resolve('js/ui/interactions.ts'), 'utf8');
    assert.match(src, /ctl\.state\.measureMode\) return;/, 'click 处理要先判量距模式');
    assert.match(src, /ctl\.state\.measureMode\) \{ showPreview\(null\); return; \}/, '悬停预览也要让位');
  });

  it('快捷键与退出：M 开量距、Esc 优先退出量距', () => {
    const ui = fs.readFileSync(path.resolve('js/ui/index.ts'), 'utf8');
    assert.match(ui, /case 'm': case 'M':/, '缺少 M 键');
    assert.match(ui, /st\.measureMode\) controller\.dispatch\(\{ type: 'TOGGLE_MEASURE' \}\)/,
      'Esc 应先退出量距，再考虑取消路线');
  });

  it('比例尺用**横向**尺度（纵向会被俯仰的透视压缩扭曲）', () => {
    const scale = fs.readFileSync(path.resolve('js/env/renderer/scale.ts'), 'utf8');
    assert.match(scale, /用\*\*屏幕水平方向\*\*在\*\*目标点深度\*\*处的尺度/, '要在文件头写明为什么用横向');
    // 公式：2·d·tan(fov/2)/viewH / unitsPerMeter（与俯仰角无关）
    assert.match(scale, /2 \* d \* Math\.tan\(\(camera\.fov \* Math\.PI \/ 180\) \/ 2\) \/ viewH/);
  });
});
