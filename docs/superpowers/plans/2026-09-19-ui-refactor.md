# UI 面板重构实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** UI 层引入单一状态来源（纯函数状态机）、按职责拆分 `js/ui/` 模块、顶栏分簇布局，并修正暂停/回放/选路/快捷键的语义缺陷。

**Architecture:** `UiState + reduce()` 纯逻辑（可 node 单测）为唯一状态来源；`UiController` 以"状态变化 → 注入副作用"方式驱动 DOM 与引擎；DOM 模块只做渲染与事件转发。`js/ui.ts` 单文件拆为 `js/ui/` 八个聚焦模块。

**Tech Stack:** TypeScript（strict，ESM，`.js` 后缀 import）+ 原生 DOM + Three.js（仅镜头/射线）；测试 node:test；无新依赖、无构建工具变化。

**Spec:** `docs/superpowers/specs/2026-09-19-ui-refactor-design.md`

**计划相对 spec 的两处微调（已记录，实现按本计划）：**
1. `actions.loadScenario` 返回 `string | null`（应用成功返回场景名，供 `SCENARIO_CHANGED` 事件使用）；actions 增加 `switchMap(name)`（承接原 main.onMapChange 的"保留场景配置换地图"逻辑）
2. HUD 顶部「地图/场景」标签留在 main 的 `applyScenario` 内写入（与真实世界状态同处），`ui/hud.ts` 只负责提示条——避免标签与状态双写

## Global Constraints

- TypeScript 严格模式；模块间 import 带 `.js` 后缀；`npx tsc` 编译到 `dist/js/**`
- 不新增 npm 依赖；不引入前端框架
- 纯前端改动：**不动 algo/env 行为**（引擎仅 `RouteState` 类型收缩）；`env/renderer/hud.ts` 不改（已核实 `#hud-route` 写入有 null 守卫，HTML 删除该元素不会崩）
- UI 状态机为纯逻辑：`js/ui/state.ts`、`js/ui/controller.ts` 不得 import DOM/three（仅类型 import 引擎）
- 新增模块中文注释（项目注释规范见 CLAUDE.md）
- 每个任务至少一次 commit（中文 message）；每个任务结束跑 `npm test` 全绿
- 浏览器手工验证统一使用：`python3 -m http.server 8080`（仓库根，已有）→ `http://localhost:8080/` 硬刷新

---

### Task 1: UI 状态机 `js/ui/state.ts`（TDD）

**Files:**
- Create: `js/ui/state.ts`
- Test: `js/test/ui/state.test.ts`

**Interfaces:**
- Consumes: `RouteState`（type-only，来自 `../sim/engine.js`）
- Produces（Task 2/4/5 依赖，名称必须一字不差）：
  - `type UiMode = 'LIVE' | 'REPLAY'`
  - `type RoutePickPhase = 'idle' | 'picking-goal'`
  - `interface UiState { mode: UiMode; livePaused: boolean; routePick: RoutePickPhase; pickStartIdx: number | null; scenarioName: string | null; inspectorOpen: boolean }`
  - `type UiEvent = {type:'TOGGLE_PAUSE'} | {type:'ENTER_REPLAY'} | {type:'EXIT_REPLAY'} | {type:'RIGHT_CLICK';idx:number} | {type:'CANCEL'} | {type:'SCENARIO_CHANGED';name:string} | {type:'ROUTE_SET'} | {type:'TOGGLE_INSPECTOR'}`
  - `INITIAL_UI_STATE: UiState`（LIVE / false / idle / null / null / true）
  - `reduce(state: UiState, e: UiEvent): UiState`
  - `deriveHint(ui: UiState, routeState: RouteState): string`

- [ ] **Step 1: 写失败测试**

创建 `js/test/ui/state.test.ts`：

```typescript
// ==================== 测试：UI 状态机（纯逻辑） ====================
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { reduce, deriveHint, INITIAL_UI_STATE, UiState } from '../../ui/state.js';

const live = (over: Partial<UiState> = {}): UiState => ({ ...INITIAL_UI_STATE, ...over });

describe('reduce: 暂停语义', () => {
  it('LIVE 中 TOGGLE_PAUSE 翻转 livePaused', () => {
    const s1 = reduce(live(), { type: 'TOGGLE_PAUSE' });
    assert.strictEqual(s1.livePaused, true);
    assert.strictEqual(reduce(s1, { type: 'TOGGLE_PAUSE' }).livePaused, false);
  });

  it('REPLAY 中 TOGGLE_PAUSE 为 no-op（状态引用不变）', () => {
    const s = live({ mode: 'REPLAY' });
    assert.strictEqual(reduce(s, { type: 'TOGGLE_PAUSE' }), s);
  });
});

describe('reduce: 回放进出不触碰 livePaused（核心修复）', () => {
  it('暂停中进回放再退出 → 仍为暂停', () => {
    let s = reduce(live(), { type: 'TOGGLE_PAUSE' });        // livePaused=true
    s = reduce(s, { type: 'ENTER_REPLAY' });
    assert.strictEqual(s.mode, 'REPLAY');
    assert.strictEqual(s.livePaused, true);
    s = reduce(s, { type: 'EXIT_REPLAY' });
    assert.strictEqual(s.mode, 'LIVE');
    assert.strictEqual(s.livePaused, true);
  });

  it('重复 ENTER/EXIT 为 no-op', () => {
    const r = live({ mode: 'REPLAY' });
    assert.strictEqual(reduce(r, { type: 'ENTER_REPLAY' }), r);
    const l = live();
    assert.strictEqual(reduce(l, { type: 'EXIT_REPLAY' }), l);
  });
});

describe('reduce: 路线拾取', () => {
  it('idle + RIGHT_CLICK → picking-goal 并记录起点', () => {
    const s = reduce(live(), { type: 'RIGHT_CLICK', idx: 5 });
    assert.strictEqual(s.routePick, 'picking-goal');
    assert.strictEqual(s.pickStartIdx, 5);
  });

  it('picking-goal + RIGHT_CLICK → 状态引用不变（建路线由 controller 处理）', () => {
    const s = reduce(live(), { type: 'RIGHT_CLICK', idx: 5 });
    assert.strictEqual(reduce(s, { type: 'RIGHT_CLICK', idx: 9 }), s);
  });

  it('REPLAY 中 RIGHT_CLICK 被忽略', () => {
    const s = live({ mode: 'REPLAY' });
    assert.strictEqual(reduce(s, { type: 'RIGHT_CLICK', idx: 5 }), s);
  });

  it('CANCEL 取消选点；idle 时 no-op', () => {
    const picking = reduce(live(), { type: 'RIGHT_CLICK', idx: 5 });
    const cancelled = reduce(picking, { type: 'CANCEL' });
    assert.strictEqual(cancelled.routePick, 'idle');
    assert.strictEqual(cancelled.pickStartIdx, null);
    assert.strictEqual(reduce(cancelled, { type: 'CANCEL' }), cancelled);
  });

  it('ROUTE_SET 回到 idle；SCENARIO_CHANGED 重置选路并记录场景名', () => {
    const picking = reduce(live(), { type: 'RIGHT_CLICK', idx: 5 });
    const done = reduce(picking, { type: 'ROUTE_SET' });
    assert.strictEqual(done.routePick, 'idle');
    assert.strictEqual(done.pickStartIdx, null);

    const changed = reduce(picking, { type: 'SCENARIO_CHANGED', name: 'oval-avoid-1' });
    assert.strictEqual(changed.scenarioName, 'oval-avoid-1');
    assert.strictEqual(changed.routePick, 'idle');
    assert.strictEqual(changed.pickStartIdx, null);
  });
});

describe('reduce: inspect 折叠', () => {
  it('TOGGLE_INSPECTOR 翻转', () => {
    const s = reduce(live(), { type: 'TOGGLE_INSPECTOR' });
    assert.strictEqual(s.inspectorOpen, false);
    assert.strictEqual(reduce(s, { type: 'TOGGLE_INSPECTOR' }).inspectorOpen, true);
  });
});

describe('deriveHint', () => {
  it('四种分支文案', () => {
    assert.strictEqual(deriveHint(live(), 'NONE'),
      '右键设置路线起点 · 左键查看物体 · Space 暂停 · R 重置视角');
    assert.strictEqual(deriveHint(reduce(live(), { type: 'RIGHT_CLICK', idx: 1 }), 'NONE'),
      '再次右键设置终点（Esc 取消）');
    assert.strictEqual(deriveHint(reduce(live(), { type: 'TOGGLE_PAUSE' }), 'NONE'),
      '已暂停（Space 继续）');
    assert.strictEqual(deriveHint(live({ mode: 'REPLAY' }), 'NONE'),
      '回放中：Space 播放/暂停 · ←/→ 单帧 · 拖动时间轴 · Esc 退出回放');
  });
});
```

- [ ] **Step 2: 运行确认失败**

Run: `npx tsc && node --test dist/js/test/ui/state.test.js`
Expected: FAIL（`Cannot find module '../../ui/state.js'`）

- [ ] **Step 3: 实现 state.ts**

创建 `js/ui/state.ts`：

```typescript
// ==================== UI 状态机（纯逻辑，可单测） ====================
//
// 单一状态来源：所有 UI 行为都是 reduce(state, event) 的投影。
// 副作用（引擎暂停、面板显隐、回放加载）由 controller 订阅状态变化执行。
// 本模块不得 import DOM/three（仅类型 import 引擎的 RouteState）。

import type { RouteState } from '../sim/engine.js';

export type UiMode = 'LIVE' | 'REPLAY';
export type RoutePickPhase = 'idle' | 'picking-goal';

export interface UiState {
  mode: UiMode;
  /** 实时仿真暂停（回放期间保持不变，退出回放即恢复） */
  livePaused: boolean;
  /** 右键选路流程（从引擎剥离的纯 UI 状态） */
  routePick: RoutePickPhase;
  pickStartIdx: number | null;
  scenarioName: string | null;
  /** inspect 面板展开状态 */
  inspectorOpen: boolean;
}

export type UiEvent =
  | { type: 'TOGGLE_PAUSE' }
  | { type: 'ENTER_REPLAY' }
  | { type: 'EXIT_REPLAY' }
  | { type: 'RIGHT_CLICK'; idx: number }
  | { type: 'CANCEL' }
  | { type: 'SCENARIO_CHANGED'; name: string }
  | { type: 'ROUTE_SET' }
  | { type: 'TOGGLE_INSPECTOR' };

export const INITIAL_UI_STATE: UiState = {
  mode: 'LIVE',
  livePaused: false,
  routePick: 'idle',
  pickStartIdx: null,
  scenarioName: null,
  inspectorOpen: true,
};

export function reduce(state: UiState, e: UiEvent): UiState {
  switch (e.type) {
    case 'TOGGLE_PAUSE':
      // 回放中不作用于实时暂停（Space 的播放/暂停语义由 index.ts 按 mode 路由）
      return state.mode === 'LIVE' ? { ...state, livePaused: !state.livePaused } : state;
    case 'ENTER_REPLAY':
      return state.mode === 'REPLAY' ? state : { ...state, mode: 'REPLAY' };
    case 'EXIT_REPLAY':
      return state.mode === 'LIVE' ? state : { ...state, mode: 'LIVE' };
    case 'RIGHT_CLICK':
      if (state.mode !== 'LIVE') return state;   // 回放中不给实时世界设路线
      if (state.routePick === 'idle') {
        return { ...state, routePick: 'picking-goal', pickStartIdx: e.idx };
      }
      return state;   // 已选起点：建路线（失败保持/成功 ROUTE_SET）由 controller 处理
    case 'CANCEL':
      return state.routePick === 'picking-goal'
        ? { ...state, routePick: 'idle', pickStartIdx: null }
        : state;
    case 'SCENARIO_CHANGED':
      return { ...state, scenarioName: e.name, routePick: 'idle', pickStartIdx: null };
    case 'ROUTE_SET':
      return state.routePick === 'picking-goal'
        ? { ...state, routePick: 'idle', pickStartIdx: null }
        : state;
    case 'TOGGLE_INSPECTOR':
      return { ...state, inspectorOpen: !state.inspectorOpen };
  }
}

/**
 * 上下文提示文案。
 * routeState 参数预留：如需展示路线状态相关文案（导航中/已到达）在此扩展。
 */
export function deriveHint(ui: UiState, routeState: RouteState): string {
  if (ui.mode === 'REPLAY') return '回放中：Space 播放/暂停 · ←/→ 单帧 · 拖动时间轴 · Esc 退出回放';
  if (ui.routePick === 'picking-goal') return '再次右键设置终点（Esc 取消）';
  if (ui.livePaused) return '已暂停（Space 继续）';
  // routeState 暂未参与分支（见 spec 开放问题）
  void routeState;
  return '右键设置路线起点 · 左键查看物体 · Space 暂停 · R 重置视角';
}
```

- [ ] **Step 4: 运行确认通过**

Run: `npx tsc && node --test dist/js/test/ui/state.test.js`
Expected: PASS（8 个用例）

- [ ] **Step 5: 提交**

```bash
git add js/ui/state.ts js/test/ui/state.test.ts
git commit -m "新增 UI 状态机（纯函数 reduce + deriveHint，8 用例）"
```

---

### Task 2: UI 控制器 `js/ui/controller.ts`（TDD）

**Files:**
- Create: `js/ui/controller.ts`
- Test: `js/test/ui/controller.test.ts`

**Interfaces:**
- Consumes: Task 1 的 `UiState/UiEvent/reduce/deriveHint/INITIAL_UI_STATE/UiMode`；`RouteState`（type-only）
- Produces（Task 5 依赖）：
  - `interface UiEffects { onLivePaused(paused: boolean): void; onMode(mode: UiMode): void; onInspector(open: boolean): void; onHint(hint: string): void }`
  - `class UiController`：`constructor(effects: UiEffects, getRouteState?: () => RouteState, initial?: UiState)`；`get state(): UiState`；`dispatch(e: UiEvent): void`（无变化不触发副作用）；`refresh(): void`（按当前状态重发全部副作用，供初始化）

- [ ] **Step 1: 写失败测试**

创建 `js/test/ui/controller.test.ts`：

```typescript
// ==================== 测试：UI 控制器（状态 → 副作用） ====================
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { UiController } from '../../ui/controller.js';
import { INITIAL_UI_STATE } from '../../ui/state.js';

function makeCtl() {
  const calls = { pause: [] as boolean[], mode: [] as string[], insp: [] as boolean[], hint: [] as string[] };
  const ctl = new UiController({
    onLivePaused: (p) => calls.pause.push(p),
    onMode: (m) => calls.mode.push(m),
    onInspector: (o) => calls.insp.push(o),
    onHint: (h) => calls.hint.push(h),
  }, () => 'NONE');
  return { ctl, calls };
}

describe('UiController.dispatch', () => {
  it('LIVE TOGGLE_PAUSE → onLivePaused(true) + 提示变为已暂停', () => {
    const { ctl, calls } = makeCtl();
    ctl.dispatch({ type: 'TOGGLE_PAUSE' });
    assert.deepStrictEqual(calls.pause, [true]);
    assert.strictEqual(calls.hint.at(-1), '已暂停（Space 继续）');
  });

  it('REPLAY 中 TOGGLE_PAUSE → 零副作用（状态未变）', () => {
    const { ctl, calls } = makeCtl();
    ctl.dispatch({ type: 'ENTER_REPLAY' });
    calls.pause.length = calls.mode.length = calls.hint.length = 0;
    ctl.dispatch({ type: 'TOGGLE_PAUSE' });
    assert.deepStrictEqual(calls.pause, []);
    assert.deepStrictEqual(calls.mode, []);
    assert.deepStrictEqual(calls.hint, []);
  });

  it('回放进出：onMode 触发、onLivePaused 从不触发（暂停保持）', () => {
    const { ctl, calls } = makeCtl();
    ctl.dispatch({ type: 'TOGGLE_PAUSE' });          // 先暂停
    calls.pause.length = 0;
    ctl.dispatch({ type: 'ENTER_REPLAY' });
    ctl.dispatch({ type: 'EXIT_REPLAY' });
    assert.deepStrictEqual(calls.mode, ['REPLAY', 'LIVE']);
    assert.deepStrictEqual(calls.pause, []);
    assert.strictEqual(ctl.state.livePaused, true);
  });

  it('RIGHT_CLICK 两步流程：第二步状态不变 → 无副作用；ROUTE_SET 收尾', () => {
    const { ctl, calls } = makeCtl();
    ctl.dispatch({ type: 'RIGHT_CLICK', idx: 3 });
    assert.strictEqual(ctl.state.routePick, 'picking-goal');
    calls.hint.length = 0;
    ctl.dispatch({ type: 'RIGHT_CLICK', idx: 9 });   // 无变化
    assert.deepStrictEqual(calls.hint, []);
    ctl.dispatch({ type: 'ROUTE_SET' });
    assert.strictEqual(ctl.state.routePick, 'idle');
    assert.strictEqual(calls.hint.at(-1), '右键设置路线起点 · 左键查看物体 · Space 暂停 · R 重置视角');
  });

  it('TOGGLE_INSPECTOR → onInspector(false)', () => {
    const { ctl, calls } = makeCtl();
    ctl.dispatch({ type: 'TOGGLE_INSPECTOR' });
    assert.deepStrictEqual(calls.insp, [false]);
  });

  it('refresh 按当前状态重发全部副作用（初始化用）', () => {
    const { ctl, calls } = makeCtl();
    ctl.refresh();
    assert.deepStrictEqual(calls.pause, [false]);
    assert.deepStrictEqual(calls.mode, ['LIVE']);
    assert.deepStrictEqual(calls.insp, [true]);
    assert.strictEqual(calls.hint.length, 1);
  });
});
```

- [ ] **Step 2: 运行确认失败**

Run: `npx tsc && node --test dist/js/test/ui/controller.test.js`
Expected: FAIL（Cannot find module `../../ui/controller.js`）

- [ ] **Step 3: 实现 controller.ts**

创建 `js/ui/controller.ts`：

```typescript
// ==================== UI 控制器（状态 → 副作用） ====================
//
// 持有 UiState；dispatch 走纯函数 reduce，对比前后状态后仅触发变化的副作用。
// 本模块不得 import DOM/three：所有副作用通过 UiEffects 注入，
// 是"状态集中"与"DOM 只是渲染投影"之间的唯一边界。

import { UiState, UiEvent, UiMode, reduce, deriveHint, INITIAL_UI_STATE } from './state.js';
import type { RouteState } from '../sim/engine.js';

export interface UiEffects {
  onLivePaused(paused: boolean): void;
  onMode(mode: UiMode): void;
  onInspector(open: boolean): void;
  onHint(hint: string): void;
}

export class UiController {
  private _state: UiState;

  constructor(
    private effects: UiEffects,
    private getRouteState: () => RouteState = () => 'NONE',
    initial: UiState = INITIAL_UI_STATE,
  ) {
    this._state = initial;
  }

  get state(): UiState { return this._state; }

  dispatch(e: UiEvent): void {
    const prev = this._state;
    const next = reduce(prev, e);
    if (next === prev) return;   // 无变化不触发副作用
    this._state = next;
    if (next.livePaused !== prev.livePaused) this.effects.onLivePaused(next.livePaused);
    if (next.mode !== prev.mode) this.effects.onMode(next.mode);
    if (next.inspectorOpen !== prev.inspectorOpen) this.effects.onInspector(next.inspectorOpen);
    this.effects.onHint(deriveHint(next, this.getRouteState()));
  }

  /** 按当前状态重发全部副作用（setup 初始化时调用一次） */
  refresh(): void {
    this.effects.onLivePaused(this._state.livePaused);
    this.effects.onMode(this._state.mode);
    this.effects.onInspector(this._state.inspectorOpen);
    this.effects.onHint(deriveHint(this._state, this.getRouteState()));
  }
}
```

- [ ] **Step 4: 运行确认通过**

Run: `npx tsc && node --test dist/js/test/ui/controller.test.js`
Expected: PASS（6 个用例）

- [ ] **Step 5: 提交**

```bash
git add js/ui/controller.ts js/test/ui/controller.test.ts
git commit -m "新增 UI 控制器（状态变化 → 注入式副作用，6 用例）"
```

---

### Task 3: index.html 顶栏分簇与提示条（无行为变化）

**Files:**
- Modify: `index.html`

**Interfaces:**
- Produces（Task 4/5 依赖的 DOM id）：`#world-cluster`、`#control-cluster`、`#hint-bar`、`#btn-inspect-toggle`、`#inspect-header`；其余 id 全部保持不变。

- [ ] **Step 1: 重构 HTML（完整替换 `<div id="canvas-container">` 内面板部分与相关 CSS）**

将 `index.html` 的 `<style>` 块中 `#map-select ...` 相关规则替换为以下规则（其余样式保留），并按下面结构改写 body 内的面板：

```css
    /* 顶栏两簇（世界 / 控制） */
    #world-cluster, #control-cluster {
      position: absolute; right: 15px; z-index: 20; pointer-events: all;
      display: flex; gap: 8px; align-items: center;
    }
    #world-cluster { top: 10px; }
    #control-cluster { top: 52px; }
    .cluster-label { color: #888; font-family: monospace; font-size: 11px; margin-right: 2px; }
    #world-cluster select, #control-cluster button {
      background: #333; color: #fff; border: 1px solid #666;
      padding: 6px 10px; font-family: monospace; font-size: 13px;
      border-radius: 4px; cursor: pointer;
    }
    #world-cluster select:hover, #control-cluster button:hover { background: #444; }

    /* 底部上下文提示条 */
    #hint-bar {
      position: absolute; bottom: 10px; left: 15px; z-index: 10; pointer-events: none;
      color: #888; font-family: monospace; font-size: 11px;
    }

    /* inspect 面板标题栏（可折叠） */
    #inspect-header {
      display: flex; justify-content: space-between; align-items: center;
      pointer-events: auto; margin-bottom: 4px;
    }
    #btn-inspect-toggle {
      background: #222; color: #0f0; border: 1px solid #555; cursor: pointer;
      font-family: monospace; font-size: 11px; border-radius: 3px; padding: 0 6px;
    }
```

面板 DOM（替换原 `#map-select` 与 `#hud-instructions`；`#hud` 块保持原样不动）：

```html
    <!-- 世界选择（地图 + 场景） -->
    <div id="world-cluster">
      <span class="cluster-label">世界</span>
      <select id="map-dropdown">
        <option value="oval">Oval</option>
        <option value="hairpin">Hairpin</option>
        <option value="highway">Highway</option>
        <option value="tech_circuit">Tech Circuit</option>
      </select>
      <select id="scenario-dropdown" title="加载场景（与 headless 同源）">
        <option value="">随机场景</option>
      </select>
    </div>

    <!-- 控制（仿真 / 数据 / 视图） -->
    <div id="control-cluster">
      <span class="cluster-label">控制</span>
      <button id="btn-pause" title="暂停/继续 (Space)">&#9208;</button>
      <button id="btn-replay-mode" title="回放已归档运行">&#9194;</button>
      <button id="btn-export" title="导出运行归档 (.run.json)">&#128190;</button>
      <button id="btn-import" title="导入归档并回放">&#128194;</button>
      <button id="btn-reset-view" title="重置视角 (R)">&#8634;</button>
    </div>

    <!-- 底部上下文提示（文案由 UI 状态机驱动，暂为静态占位） -->
    <div id="hint-bar">右键设置路线起点 · 左键查看物体 · Space 暂停 · R 重置视角</div>
```

`#inspect-panel` 内部最前面加标题栏：

```html
    <div id="inspect-panel" style="...原样式不变...">
      <div id="inspect-header">
        <span>Inspect</span>
        <button id="btn-inspect-toggle" title="折叠/展开 (I)">&#9662;</button>
      </div>
    </div>
```

注意：删除 `#map-select` 与 `#hud-instructions` 两个旧容器；`#hud-route`/`#hud-gps` 等 HUD 元素保持不动（`env/renderer/hud.ts` 有 null 守卫，不改 renderer）。

- [ ] **Step 2: 编译与浏览器冒烟（行为应与重构前完全一致）**

Run: `npx tsc && node --test dist/js/test/`（应 109 全绿，纯 HTML 改动不影响）
浏览器硬刷新后验证：
1. 顶右出现「世界」「控制」两簇（地图/场景在上，五个按钮在下）
2. 地图/场景切换、暂停、回放、导出、导入、重置视角全部仍可用
3. 底部提示条显示静态文案；旧的底部说明文字消失
4. inspect 面板出现标题栏与折叠按钮（按钮暂未接线，点击无反应属预期）

> 注：T3 结束时 `#btn-inspect-toggle` 是未接线的死按钮，Task 5 接线；如需测试期间避免误点，可先不点。

- [ ] **Step 3: 提交**

```bash
git add index.html
git commit -m "index.html 顶栏分簇（世界/控制）+ 底部提示条 + inspect 折叠按钮占位"
```

---

### Task 4: `js/ui.ts` 拆分为 `js/ui/` 模块（行为保持）

**Files:**
- Create: `js/ui/index.ts`、`js/ui/toolbar.ts`、`js/ui/interactions.ts`、`js/ui/replayPanel.ts`、`js/ui/inspector.ts`、`js/ui/hud.ts`
- Delete: `js/ui.ts`
- Modify: `js/main.ts`（仅改 import 路径两行）

**Interfaces:**
- Consumes: 现有 `js/ui.ts` 的全部逻辑（**原样迁移，不改行为**）
- Produces（Task 5 在此基础上重写）：
  - `js/ui/index.ts`：`export interface UIContext {...}`（原样搬移）、`export function setupUI(ctx: UIContext): void`、`export { updateInspectPanel } from './inspector.js'`
  - `js/ui/inspector.ts`：`export function updateInspectPanel(dt: number): void`（原 `js/ui.ts` 52-96 行原样）
  - 其余模块导出各自 `wireXxx(ctx)` 接线函数

**迁移映射（`js/ui.ts` 现有行号 → 目标文件）：**

| 现有内容（js/ui.ts 行） | 去向 |
|---|---|
| 1-16 头注释与 imports | 按需分发到各文件（去掉不再用的） |
| 17-50 `UIContext` | `ui/index.ts` |
| 52-96 inspect 面板 | `ui/inspector.ts` |
| 102-103 setupUI 头 | `ui/index.ts` |
| 105-112 地图选择 | `ui/toolbar.ts`（`wireToolbar(ctx)`） |
| 114-139 场景选择 | `ui/toolbar.ts` |
| 141-164 导出/导入按钮 | `ui/toolbar.ts`（导出直接调 `ctx.onExportRun`；导入流程原样，`enterReplay` 经参数传入） |
| 166-175 暂停（函数+按钮+Space） | 暂停按钮 → `ui/toolbar.ts`；Space 键盘 → `ui/index.ts`（暂时保持原逻辑） |
| 177-186 视角重置 | 按钮 → `ui/toolbar.ts`；R 键盘 → `ui/index.ts` |
| 188-214 左键 tooltip | `ui/interactions.ts`（`wireInteractions(ctx)`） |
| 216-252 右键路线 | `ui/interactions.ts` |
| 254-318 回放栏全部 | `ui/replayPanel.ts`（`wireReplayPanel(ctx, replayBar)`；含 enterReplay、标记渲染、箭头键盘） |

- [ ] **Step 1: 建立模块骨架并迁移**

按上表逐段迁入。关键约束：
- `ui/index.ts` 的 `setupUI(ctx)` 保持与现在完全相同的签名与调用顺序（main.ts 只改 import 路径）
- 迁移中**不得**改动任何事件处理逻辑（这是行为保持任务）；`enterReplay` 定义放在 `ui/index.ts`，作为参数传给 `wireToolbar(ctx, enterReplay)` 与 `wireReplayPanel(ctx, enterReplay)`
- 每个被移动的函数顶部补一行模块头注释（中文），其余注释原样保留：

```typescript
// 各新文件头注释模板：
// ==================== UI：<模块名> ====================
// <一句话职责>。事件处理接线，状态逻辑见 state.ts/controller.ts（Task 5 接线）。
```

`js/ui/index.ts` 骨架：

```typescript
// ==================== UI 编排层 ====================
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { Route, GPSReading, TrackGeometry } from '../shared/types.js';
import { MapDefinition } from '../env/mapDefinitions.js';
import { RoadGraph } from '../algo/routePlanner.js';
import { ReplayController } from '../replay.js';
import { wireToolbar } from './toolbar.js';
import { wireInteractions } from './interactions.js';
import { wireReplayPanel } from './replayPanel.js';

export interface UIContext { /* ← 原 js/ui.ts 17-50 行原样搬入 */ }

export { updateInspectPanel } from './inspector.js';

export function setupUI(ctx: UIContext) {
  const replay = ctx.replay;

  /** 进入回放：暂停实时仿真 → 加载数据 → 显示回放栏（Task 5 重写为状态机驱动） */
  async function enterReplay(load: () => Promise<void> | void) {
    debug.paused = true;                 // ← 原逻辑原样（Task 5 删除）
    await load();
    showReplayBar(); renderReplayMarkers();
  }

  const panel = wireReplayPanel(ctx, enterReplay);   // 返回 { showReplayBar, renderReplayMarkers, hideReplayBar }
  wireToolbar(ctx, enterReplay, panel);
  wireInteractions(ctx);
  wireKeyboard(ctx);                                  // Space / R 键盘（原 173-175、184-186 行）
}
```

（`wireKeyboard` 内联在 index.ts 即可；`debug` 的 import 从 `../shared/debug.js`。）

- [ ] **Step 2: 删除旧文件、更新 main.ts import**

```bash
git rm js/ui.ts
```

`js/main.ts` 两处修改：

```diff
-import { setupUI, updateInspectPanel } from './ui.js';
+import { setupUI, updateInspectPanel } from './ui/index.js';
```

- [ ] **Step 3: 编译 + 全量测试 + 浏览器冒烟**

Run: `npx tsc && node --test dist/js/test/`
Expected: 109 全绿（本任务无逻辑变化）
浏览器逐项冒烟（与 Task 3 相同清单）：地图/场景切换、暂停（按钮+Space）、回放（列表/导入/时间轴/标记/箭头/关闭）、导出、右键路线、左键 tooltip、R 视角、inspect 面板刷新。**任何一项与重构前行为不一致 → 修复后再继续。**

- [ ] **Step 4: 提交**

```bash
git add js/ui/ js/main.ts
git commit -m "js/ui.ts 拆分为 js/ui/ 目录模块（行为保持的机械迁移）"
```

---

### Task 5: 状态机接线 + 行为修正 + UiDeps 收窄

**Files:**
- Modify: `js/sim/engine.ts`（RouteState 收缩，1 行）
- Modify: `js/main.ts`（UI 接线重写）
- Modify: `js/ui/index.ts`（重写为状态机编排 + 键盘分发器）
- Modify: `js/ui/toolbar.ts`、`js/ui/interactions.ts`、`js/ui/replayPanel.ts`、`js/ui/inspector.ts`、`js/ui/hud.ts`
- Modify: `CLAUDE.md`（目录与快捷键描述）

**Interfaces:**
- Consumes: Task 1/2 的 `UiState/reduce/UiController`；`ReplayController`；`RouteState`（收缩后）
- Produces（最终形态，供 main 使用）：
  - `interface UiDeps`（见 Step 3 代码，含 `engine` 只读视图与 `actions`）
  - `setupUI(deps: UiDeps): { tick(dt: number): void }`
  - `actions`: `{ loadScenario(json): string | null; loadDefaultScenario(map): void; switchMap(name): void; exportRun(): void; setRoute(route, startIdx): void; setLivePaused(paused): void }`

- [ ] **Step 1: 引擎 RouteState 收缩**

`js/sim/engine.ts:54`：

```diff
-export type RouteState = 'NONE' | 'SETTING_START' | 'SETTING_GOAL' | 'ACTIVE' | 'ARRIVED';
+export type RouteState = 'NONE' | 'ACTIVE' | 'ARRIVED';
```

Run: `npx tsc`→ 预期报错仅出现在 `js/ui.ts` 已被删除的旧引用处（Task 4 已删）。随后全库核对：

```bash
grep -rn "SETTING_START\|SETTING_GOAL\|setRouteState" js/ --include="*.ts" | grep -v dist
```

Expected: 空（无残留）。

- [ ] **Step 2: 重写 `js/ui/hud.ts`（提示条渲染）与 `js/ui/inspector.ts`（折叠支持）**

`js/ui/hud.ts` 最终内容：

```typescript
// ==================== UI：提示条 ====================
// 底部上下文提示（#hint-bar）渲染；文案由 state.ts 的 deriveHint 派生。
// HUD 顶部「地图/场景」标签由 main.ts 的 applyScenario 写入（与真实世界状态同处）。

export function renderHint(text: string): void {
  const el = document.getElementById('hint-bar');
  if (el) el.textContent = text;
}
```

`js/ui/inspector.ts`：保留 `updateInspectPanel(dt)` 原实现，末尾追加：

```typescript
/** 折叠/展开 inspect 面板（控制内容区显隐与按钮箭头） */
export function setInspectorOpen(open: boolean): void {
  const panel = document.getElementById('inspect-panel');
  const btn = document.getElementById('btn-inspect-toggle');
  if (!panel) return;
  const body = panel.querySelector('pre');
  if (body) (body as HTMLElement).style.display = open ? 'block' : 'none';
  if (btn) btn.textContent = open ? '▾' : '▸';
}
```

并为 `updateInspectPanel` 增加首行守卫：

```typescript
  const panelEl = document.getElementById('inspect-panel');
  if (!panelEl) return;
```

（其余原样。）

- [ ] **Step 3: 重写 `js/ui/index.ts`（状态机编排 + 键盘分发器 + UiDeps）**

最终内容：

```typescript
// ==================== UI 编排层 ====================
//
// 组装：UiController（状态机）+ 各面板接线。所有 UI 行为经 dispatch 驱动，
// DOM 是状态的渲染投影；main.ts 只通过 UiDeps.actions 改世界。

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { Route, TrackGeometry } from '../shared/types.js';
import { MapDefinition } from '../env/mapDefinitions.js';
import { RoadGraph } from '../algo/routePlanner.js';
import { ReplayController } from '../replay.js';
import type { RouteState } from '../sim/engine.js';
import { UiController } from './controller.js';
import { wireToolbar } from './toolbar.js';
import { wireInteractions } from './interactions.js';
import { setupReplayPanel } from './replayPanel.js';
import { renderHint } from './hud.js';
import { setInspectorOpen, updateInspectPanel } from './inspector.js';

export interface UiDeps {
  canvas: HTMLCanvasElement;
  camera: THREE.PerspectiveCamera;
  scene: THREE.Scene;
  controls: OrbitControls;
  replay: ReplayController;
  /** 引擎只读视图 */
  engine: {
    readonly mapDef: MapDefinition;
    readonly track: TrackGeometry;
    readonly roadGraph: RoadGraph;
    readonly route: Route | null;
    readonly routeState: RouteState;
  };
  /** UI 改世界的唯一通道 */
  actions: {
    /** 加载场景 JSON；成功返回场景名，失败（已 alert）返回 null */
    loadScenario(json: unknown): string | null;
    /** 当前地图的浏览器默认随机场景 */
    loadDefaultScenario(map: string): void;
    /** 保留当前场景配置，仅换地图 */
    switchMap(name: string): void;
    exportRun(): void;
    setRoute(route: Route, startIdx: number): void;
    setLivePaused(paused: boolean): void;
  };
}

export { updateInspectPanel };

export function setupUI(deps: UiDeps): { tick(dt: number): void } {
  const { canvas, camera, controls } = deps;

  // ── 状态机 + 副作用 ──（replayPanel 在下方定义，effect 仅在运行期触发，无 TDZ 问题）
  const controller = new UiController({
    onLivePaused: (p) => deps.actions.setLivePaused(p),
    onMode: (mode) => (mode === 'REPLAY' ? replayPanel.show() : replayPanel.hide()),
    onInspector: (open) => setInspectorOpen(open),
    onHint: (hint) => renderHint(hint),
  }, () => deps.engine.routeState);

  /** 进入回放：加载数据成功后切模式（回放栏显隐由 onMode 副作用处理） */
  async function enterReplay(load: () => Promise<void> | void) {
    await load();
    controller.dispatch({ type: 'ENTER_REPLAY' });
    replayPanel.renderMarkers();
  }

  /** 退出回放：恢复实时（livePaused 从未被改动，自动恢复原暂停状态） */
  function exitReplay() {
    deps.replay.close();
    controller.dispatch({ type: 'EXIT_REPLAY' });
  }

  /** 场景/地图切换前：回放中先退出回放（避免回放渲染叠加到新世界） */
  function exitReplayIfActive() {
    if (controller.state.mode === 'REPLAY') exitReplay();
  }

  const replayPanel = setupReplayPanel(deps, enterReplay, exitReplay);
  wireToolbar(deps, controller, enterReplay, exitReplayIfActive);
  wireInteractions(deps, controller);

  // inspect 折叠按钮
  document.getElementById('btn-inspect-toggle')?.addEventListener('click', () => {
    controller.dispatch({ type: 'TOGGLE_INSPECTOR' });
  });

  // ── 统一键盘分发器（Esc 三义 / Space 双语义）──
  function resetCamera() {
    camera.position.set(deps.canvas.width / 2, 600, deps.canvas.height / 2 + 400);
    controls.target.set(deps.canvas.width / 2, 0, deps.canvas.height / 2);
    controls.update();
  }
  window.addEventListener('keydown', (e) => {
    if (e.target instanceof HTMLInputElement) return;
    const st = controller.state;
    switch (e.key) {
      case ' ':
        e.preventDefault();
        if (st.mode === 'REPLAY') deps.replay.toggle();
        else controller.dispatch({ type: 'TOGGLE_PAUSE' });
        break;
      case 'Escape':
        if (st.mode === 'REPLAY') exitReplay();
        else controller.dispatch({ type: 'CANCEL' });
        break;
      case 'r': case 'R':
        resetCamera();
        break;
      case 'i': case 'I':
        controller.dispatch({ type: 'TOGGLE_INSPECTOR' });
        break;
      case 'ArrowLeft':
        if (st.mode === 'REPLAY') deps.replay.seekStep(deps.replay.state.currentStep - 1);
        break;
      case 'ArrowRight':
        if (st.mode === 'REPLAY') deps.replay.seekStep(deps.replay.state.currentStep + 1);
        break;
    }
  });

  controller.refresh();   // 初始化：同步暂停按钮/回放栏/inspect/提示条

  return {
    tick(dt: number) {
      if (controller.state.inspectorOpen) updateInspectPanel(dt);
    },
  };
}
```

- [ ] **Step 4: 重写 `js/ui/toolbar.ts`、`js/ui/interactions.ts`、`js/ui/replayPanel.ts`**

`js/ui/toolbar.ts` 最终内容：

```typescript
// ==================== UI：顶栏（世界簇 + 控制簇） ====================
// 世界：地图/场景选择；控制：暂停/回放/导出/导入/视角。
// 只做事件接线 → 状态机 dispatch 或 deps.actions；不持有状态。

import { UiController } from './controller.js';
import type { UiDeps } from './index.js';
import { ReplayController } from '../replay.js';
import { parseBundle } from '../headless/bundle.js';

export function wireToolbar(
  deps: UiDeps, ctl: UiController,
  enterReplay: (load: () => Promise<void> | void) => Promise<void>,
  exitReplayIfActive: () => void,
): void {
  // ── 地图选择（保留场景配置仅换地图；回放中先退出回放）──
  // 下拉初值由 main 的 applyScenario 同步（它在 setupUI 之前执行）
  const mapSelect = document.getElementById('map-dropdown') as HTMLSelectElement | null;
  if (mapSelect) {
    mapSelect.addEventListener('change', () => {
      exitReplayIfActive();
      deps.actions.switchMap(mapSelect.value);
      ctl.dispatch({ type: 'SCENARIO_CHANGED', name: `browser-${mapSelect.value}` });
    });
  }

  // ── 场景加载（scenarios/index.json 列表）──
  const scenarioSelect = document.getElementById('scenario-dropdown') as HTMLSelectElement | null;
  if (scenarioSelect) {
    fetch('scenarios/index.json')
      .then(r => (r.ok ? r.json() : { scenarios: [] }))
      .then((idx: any) => {
        for (const entry of idx.scenarios ?? []) {
          const opt = document.createElement('option');
          opt.value = entry.file;
          opt.textContent = `${entry.name}（${entry.map}）`;
          scenarioSelect.appendChild(opt);
        }
      })
      .catch(() => { /* 索引缺失不阻塞页面 */ });
    scenarioSelect.addEventListener('change', async () => {
      const file = scenarioSelect.value;
      exitReplayIfActive();
      if (!file) { deps.actions.loadDefaultScenario(deps.engine.mapDef.name); ctl.dispatch({ type: 'SCENARIO_CHANGED', name: '随机场景' }); return; }
      try {
        const res = await fetch(`scenarios/${file}`);
        if (!res.ok) throw new Error(`HTTP ${res.status}`);
        const name = deps.actions.loadScenario(await res.json());
        if (name) ctl.dispatch({ type: 'SCENARIO_CHANGED', name });
      } catch (e) {
        alert(`场景加载失败: ${(e as Error).message}`);
      }
    });
  }

  // ── 控制簇 ──
  document.getElementById('btn-pause')?.addEventListener('click', () => {
    // 回放中切回放播放；LIVE 中切实时暂停（与 Space 语义一致）
    if (ctl.state.mode === 'REPLAY') deps.replay.toggle();
    else ctl.dispatch({ type: 'TOGGLE_PAUSE' });
  });

  document.getElementById('btn-replay-mode')?.addEventListener('click', async () => {
    const runs = await ReplayController.listRuns();
    if (runs.length === 0) {
      alert('没有可回放的运行。先执行: npm run sim -- --scenario scenarios/oval-cruise.json');
      return;
    }
    const choice = prompt('选择运行（输入编号）:\n' + runs.map((r, i) => `${i}: ${r.label}`).join('\n'), '0');
    if (choice === null) return;
    const idx = Number(choice);
    if (Number.isNaN(idx) || !runs[idx]) return;
    await enterReplay(() => deps.replay.load(runs[idx].id));
  });

  document.getElementById('btn-export')?.addEventListener('click', () => deps.actions.exportRun());

  document.getElementById('btn-import')?.addEventListener('click', () => {
    const input = document.createElement('input');
    input.type = 'file'; input.accept = '.json';
    input.onchange = (e: any) => {
      const file = e.target?.files?.[0];
      if (!file) return;
      const reader = new FileReader();
      reader.onload = () => {
        try {
          const bundle = parseBundle(JSON.parse(reader.result as string));
          void enterReplay(() => deps.replay.loadBundle(bundle));
        } catch (err) {
          alert(`归档导入失败: ${(err as Error).message}`);
        }
      };
      reader.readAsText(file);
    };
    input.click();
  });

  const resetCamera = () => {
    deps.camera.position.set(deps.canvas.width / 2, 600, deps.canvas.height / 2 + 400);
    deps.controls.target.set(deps.canvas.width / 2, 0, deps.canvas.height / 2);
    deps.controls.update();
  };
  document.getElementById('btn-reset-view')?.addEventListener('click', resetCamera);
}
```

`js/ui/interactions.ts` 最终内容：

```typescript
// ==================== UI：画布交互 ====================
// 左键：物体信息 tooltip；右键：路线拾取（→ 状态机 dispatch，路线构建在此完成）。

import * as THREE from 'three';
import { UiController } from './controller.js';
import type { UiDeps } from './index.js';
import { findRoute, findClosestPoint } from '../algo/routePlanner.js';
import { createRoute } from '../shared/types.js';
import { px } from '../shared/dimensions.js';

export function wireInteractions(deps: UiDeps, ctl: UiController): void {
  const { canvas, camera } = deps;

  // ── 左键：物体信息（原逻辑原样） ──
  canvas.addEventListener('click', (e: MouseEvent) => {
    /* ← 原 js/ui.ts 188-214 行逻辑原样搬入 */
  });

  // ── 右键：路线拾取（两段式：起点 → 终点） ──
  canvas.addEventListener('contextmenu', (e: MouseEvent) => {
    e.preventDefault();
    if (ctl.state.mode !== 'LIVE') return;   // 回放中不给实时世界设路线
    const rect = canvas.getBoundingClientRect();
    const mouse = new THREE.Vector2(
      ((e.clientX - rect.left) / rect.width) * 2 - 1,
      -((e.clientY - rect.top) / rect.height) * 2 + 1,
    );
    const raycaster = new THREE.Raycaster();
    raycaster.setFromCamera(mouse, camera);
    const target = new THREE.Vector3();
    raycaster.ray.intersectPlane(new THREE.Plane(new THREE.Vector3(0, 1, 0), 0), target);
    if (!target) return;

    const idx = findClosestPoint(target.x, target.z, deps.engine.track.centerPoints);
    const st = ctl.state;

    if (st.routePick === 'idle') {
      ctl.dispatch({ type: 'RIGHT_CLICK', idx });
      return;
    }
    // picking-goal：尝试建路线；失败保持 picking（与旧行为一致）
    const startIdx = st.pickStartIdx!;
    const pathIndices = findRoute(deps.engine.roadGraph, startIdx, idx);
    if (!pathIndices || pathIndices.length < 2) return;
    const waypoints = pathIndices.map(i => deps.engine.track.centerPoints[i]);
    let totalLen = 0;
    for (let i = 1; i < waypoints.length; i++) {
      totalLen += Math.hypot(
        (waypoints[i].x as number) - (waypoints[i - 1].x as number),
        (waypoints[i].y as number) - (waypoints[i - 1].y as number),
      );
    }
    const r = createRoute(waypoints, pathIndices, startIdx, idx, px(totalLen));
    deps.actions.setRoute(r, startIdx);
    ctl.dispatch({ type: 'ROUTE_SET' });
  });
}
```

（左键 tooltip 的 27 行原逻辑照搬 Task 4 迁移后的内容，不得改写。）

`js/ui/replayPanel.ts` 最终内容：

```typescript
// ==================== UI：回放栏 ====================
// 时间轴/问题标记/播放/速度/步进/关闭。播放内部状态归 ReplayController；
// 模式（LIVE/REPLAY）归 UiState——本模块只接线。

import type { UiDeps } from './index.js';

export interface ReplayPanelApi {
  show(): void;
  hide(): void;
  renderMarkers(): void;
}

export function setupReplayPanel(
  deps: UiDeps,
  enterReplay: (load: () => Promise<void> | void) => Promise<void>,
  exitReplay: () => void,
): ReplayPanelApi {
  const replay = deps.replay;
  const replayBar = document.getElementById('replay-bar') as HTMLDivElement | null;
  const timeline = document.getElementById('replay-timeline') as HTMLInputElement | null;
  const stepLabel = document.getElementById('replay-step-label');
  const markersBox = document.getElementById('replay-markers');

  function renderMarkers(): void {
    if (!markersBox) return;
    markersBox.innerHTML = '';
    const total = replay.state.totalSteps || 1;
    for (const step of replay.state.markers) {
      const dot = document.createElement('div');
      dot.className = 'replay-marker';
      dot.style.left = `${(step / total) * 100}%`;
      dot.title = `问题帧 ${step}`;
      dot.addEventListener('click', () => replay.seekStep(step));
      markersBox.appendChild(dot);
    }
  }

  document.getElementById('btn-replay-play')?.addEventListener('click', () => replay.toggle());
  document.getElementById('btn-replay-prev')?.addEventListener('click', () => replay.seekStep(replay.state.currentStep - 1));
  document.getElementById('btn-replay-next')?.addEventListener('click', () => replay.seekStep(replay.state.currentStep + 1));
  (document.getElementById('replay-speed') as HTMLSelectElement | null)
    ?.addEventListener('change', (e: any) => replay.setSpeed(Number(e.target.value)));
  document.getElementById('btn-replay-close')?.addEventListener('click', () => exitReplay());
  timeline?.addEventListener('input', () => replay.seekStep(Number(timeline.value)));
  replay.onChange = () => {
    if (timeline) {
      timeline.value = String(Math.floor(replay.state.currentStep));
      timeline.max = String(replay.state.totalSteps);
    }
    if (stepLabel) stepLabel.textContent = `${Math.floor(replay.state.currentStep)} / ${replay.state.totalSteps}`;
  };

  return {
    show() { if (replayBar) replayBar.style.display = 'flex'; },
    hide() { if (replayBar) replayBar.style.display = 'none'; },
    renderMarkers,
  };
}
```

（箭头键在 `ui/index.ts` 键盘分发器中，本模块不再挂 ArrowLeft/Right。）

- [ ] **Step 5: 重写 `js/main.ts` 的 UI 接线**

`js/main.ts` 最终内容：

```typescript
// ==================== 主入口：engine 装配 + 渲染 + UI ====================
import { MessageBus } from './messageBus.js';
import { FrameRecord } from './shared/types.js';
import { debug } from './shared/debug.js';
import { SIM_W, SIM_H, FIXED_DT, SimEngine } from './sim/engine.js';
import { Scenario, parseScenario, buildScenarioWorld, defaultScenario } from './sim/scenario.js';
import { ARCHIVE_TOPICS, recordFrom } from './sim/records.js';
import { assembleBundle } from './headless/bundle.js';
import { initScene, updateScene, resetSceneCache } from './env/renderer.js';
import { MAP_PRESETS, DEFAULT_MAP } from './env/mapDefinitions.js';
import { setupUI } from './ui/index.js';
import { ReplayController } from './replay.js';

const W = SIM_W, H = SIM_H;
const canvas = document.getElementById('canvas') as HTMLCanvasElement;
canvas.width = W; canvas.height = H;

const { renderer, scene, camera, controls, lidarLines } = initScene(canvas, W, H);
const engine = new SimEngine(new MessageBus('autopilot', false), { map: MAP_PRESETS[DEFAULT_MAP] });

// 运行归档收集（与 headless 同源）
const frames: FrameRecord[] = [];
for (const topic of ARCHIVE_TOPICS) {
  engine.bus.subscribe(topic, (p: any) => frames.push(recordFrom(topic, p)));
}

// ==================== 场景（与 headless 完全同源） ====================
let currentScenario: Scenario | null = null;

function applyScenario(json: unknown): string | null {
  let scenario: Scenario;
  let built: ReturnType<typeof buildScenarioWorld>;
  try {
    scenario = parseScenario(json);
    built = buildScenarioWorld(scenario, SIM_W, SIM_H);
  } catch (e) {
    alert(`场景加载失败: ${(e as Error).message}`);
    return null;
  }
  engine.rebuildWorld(built.worldCfg);
  if (built.route) {
    engine.setRoute(built.route, built.route.startIndex, { speed: built.worldCfg.carStart?.speed });
  }
  currentScenario = scenario;
  resetSceneCache(scene);
  frames.length = 0;
  document.getElementById('hud-map')!.textContent = engine.mapDef.name;
  document.getElementById('hud-scenario')!.textContent = `${scenario.name}（种子 ${scenario.seed}）`;
  const mapSelect = document.getElementById('map-dropdown') as HTMLSelectElement | null;
  if (mapSelect) mapSelect.value = scenario.map;
  return scenario.name;
}

function exportRun(): void {
  if (!currentScenario) return;
  const bundle = assembleBundle({
    scenario: currentScenario, track: engine.track, obstacles: engine.obstacles,
    route: engine.route, frames, simTime: engine.simTime,
    id: `${new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19)}_${currentScenario.name}_browser`,
    createdAt: new Date().toISOString(),
    gitCommit: null, nodeVersion: 'browser',
  });
  const blob = new Blob([JSON.stringify(bundle)], { type: 'application/json' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = `${bundle.meta.id}.run.json`;
  a.click();
  URL.revokeObjectURL(a.href);
}

// 初始世界：浏览器默认场景
applyScenario(defaultScenario(DEFAULT_MAP));

const replay = new ReplayController(scene, lidarLines);

const ui = setupUI({
  canvas, camera, scene, controls, replay,
  engine: {
    get mapDef() { return engine.mapDef; },
    get track() { return engine.track; },
    get roadGraph() { return engine.roadGraph; },
    get route() { return engine.route; },
    get routeState() { return engine.routeState; },
  },
  actions: {
    loadScenario(json: unknown) { return applyScenario(json); },
    loadDefaultScenario(map: string) { applyScenario(defaultScenario(map)); },
    switchMap(name: string) {
      const map = MAP_PRESETS[name] ? name : DEFAULT_MAP;
      const base = currentScenario ?? defaultScenario(map);
      applyScenario({ ...base, map });
    },
    exportRun,
    setRoute(r, startIdx) { engine.setRoute(r, startIdx); },
    setLivePaused(paused: boolean) { debug.paused = paused; },
  },
});

engine.bus.subscribe('control:state', ({ snapshot, lidarScan, trajectory, state }: any) => {
  updateScene(scene, snapshot, lidarScan, state, lidarLines, trajectory, engine.route, engine.latestGPS);
});

// ==================== 主循环（固定步长累积器） ====================
let lastTime = performance.now();
let acc = 0;
const MAX_CATCHUP = 5;

function gameLoop(now: number) {
  const realDt = Math.min(0.1, (now - lastTime) / 1000);
  lastTime = now;

  if (replay.state.active) {
    replay.update(realDt);
  } else if (!debug.paused) {
    acc += realDt;
    let n = 0;
    while (acc >= FIXED_DT && n < MAX_CATCHUP) {
      engine.step(FIXED_DT);
      acc -= FIXED_DT;
      n++;
    }
    if (n === MAX_CATCHUP) acc = 0;
  }

  controls.update();
  renderer.render(scene, camera);
  ui.tick(realDt);
  requestAnimationFrame(gameLoop);
}
requestAnimationFrame(gameLoop);
```

（原 `clickStartIdx` 模块变量、`setRouteState/getRouteState/setClickStartIdx/getClickStartIdx` 回调全部随旧 UIContext 一并删除。）

- [ ] **Step 6: 全量测试 + 浏览器验收清单**

Run: `npx tsc && node --test dist/js/test/` → 期望 109 全绿（state/controller 新测试使总数增至 123）：
- Task 1 的 8 例 + Task 2 的 6 例 → 实际运行确认 `# tests 123 / # pass 123`（若数字因用例分组不同略有出入，以 fail=0 为准）

浏览器验收（硬刷新）：

1. **核心回归**：手动 Space 暂停 → 打开回放（⏪）→ 关闭回放 → **仍处于暂停**（旧版会丢失）
2. Space：LIVE 暂停/继续；REPLAY 播放/暂停
3. Esc：回放中退出回放；选点中取消选点（提示条文案同步变化）
4. 回放中切换场景/地图 → 自动退出回放并正常加载
5. 回放中右键不产生路线
6. I 键与标题栏按钮折叠/展开 inspect（按钮箭头 ▾/▸ 同步）
7. 提示条文案四处随状态切换（空闲/选点/暂停/回放）
8. 原有功能无回归：场景加载（含 batch）、导出、导入回放、服务器回放列表、右键设路线（含建路线失败保持选点）、R 视角、地图切换保留场景、左键 tooltip

- [ ] **Step 7: 更新 CLAUDE.md**

1. 目录职责行：`js/ui.ts / js/replay.ts — DOM 交互与回放控制器` → `js/ui/ — UI 层（state.ts 纯状态机 + controller.ts 副作用 + 各面板模块）、js/replay.ts — 回放控制器`
2. 「调试与记录」前的架构说明无需改；在「规划器」等无关，跳过——只加上一条 UI 约定，追加到「核心设计约定」列表第 7 条：

```markdown
7. **UI 状态单一来源** — 所有 UI 行为经 `ui/state.ts` 的纯函数 reduce 驱动，DOM 只是状态的渲染投影；`ui/controller.ts` 是"状态变化 → 副作用"的唯一边界；main 只通过 `UiDeps.actions` 改世界。快捷键：Space 暂停/回放、Esc 取消选点/退出回放、R 视角、I 折叠 Inspect
```

- [ ] **Step 8: 提交**

```bash
git add js/sim/engine.ts js/main.ts js/ui/ CLAUDE.md
git commit -m "UI 状态机接线 + 行为修正：暂停跨回放保持、Esc 三义、回放隔离、提示条驱动、UiDeps 收窄"
```

---

## 计划自审记录

1. **Spec 覆盖**：spec §3（UiState/reducer/controller/RouteState 收缩/UiDeps）→ T1/T2/T5；§4（ui/ 模块结构）→ T4/T5；§5 行为修正 8 条 → T1（暂停语义）、T5（Space/Esc/回放隔离/选点/提示/快捷键/折叠）；§6 DOM 分区 → T3；§7 测试与验收 → T1/T2 单测 + T3/T4/T5 浏览器清单；§8 顺序一致。spec §9 两个开放问题按"暂不扩展/不落盘"执行。
2. **无占位**：T4 是行为保持的机械迁移，迁移映射表给出精确行号来源与落点；其间"原逻辑原样搬入"均指定了确切行号区间，禁止改写（唯一例外已在注①注②说明）
3. **类型一致性**：`UiState/UiEvent/UiEffects/UiController/UiDeps/actions` 在 T1/T2 定义、T5 消费，签名逐字一致；`RouteState` 收缩后 3 态在 main 只读视图与 T1 类型 import 中一致；`loadScenario` 返回 `string | null` 在 T5 的 actions 接口与 main 实现一致
4. **已知取舍**：T3 结束时 `#btn-inspect-toggle` 为未接线按钮（T5 接线，已在 T3 步骤内标注）；T4 迁移期间 `enterReplay` 内 `debug.paused = true` 保持原样（T5 删除——回放模式由 gameLoop 分支与 UiState 接管，不再需要写 paused）
