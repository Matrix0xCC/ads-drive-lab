# UI 面板重构设计（状态集中 + 模块拆分 + 行为修正）

- 日期：2026-09-19
- 状态：设计已与用户逐节确认（S1 状态架构 / S2 行为修正），待用户审阅本文档
- 关联：现状见 `js/ui.ts`（320 行单函数 setupUI）、`index.html`、`js/main.ts`

## 1. 背景与目标

UI 层在一次次的增量功能中积累了三类问题：

**① 功能混装**：`#map-select` 一排挤了 7 个控件跨 5 类职责（世界选择/视角/数据 I-O/暂停/回放入口）；HUD 里遥测与操作提示混排。

**② 状态混乱（核心）**：

- 路线状态三处分裂：`routeState` 在 engine（string 类型，且混入 `SETTING_START`（死状态）/`SETTING_GOAL` 等纯 UI 状态）、`clickStartIdx` 在 main.ts 模块变量、ui.ts 靠 4 个 stringly-typed 回调间接读写
- 暂停语义混乱：`debug.paused`（算法调试标志）被当全局 UI 状态用——回放退出无条件置 false（丢失回放前的暂停）；回放中 Space 仍切换实时引擎暂停
- 回放状态三处隐式同步：`replay.state.active` + `replayBar.style.display` + `debug.paused` 手动保持一致；回放中切场景/地图不设防（渲染错乱风险）

**③ 结构**：`setupUI` 单函数 220 行扛 7 类职责；快捷键散落三处各自复制作用域判断；inspect 面板常显不可收起。

**目标**：状态只有一份（单一来源 + 纯函数转移，可单测）；DOM 是状态的渲染投影；面板按职责分区（布局方案 A：顶栏分簇）；顺带修正全部状态语义缺陷。**纯前端重构，不动 algo/env/headless。**

## 2. 范围

**包含**：UiState 状态机（纯逻辑单测）；`ui.ts` 拆分为 `js/ui/` 目录模块；DOM 分区（顶栏两簇 + 提示条 + 可折叠 Inspect）；暂停/回放/拾取/快捷键的行为修正。

**非目标**：不引入前端框架或构建工具（保持 ESM + tsc）；不重构 `replay.ts` 内部与 `env/renderer/hud.ts`（HUD 遥测仍由 renderer 经 updateScene 更新）；不做移动端/响应式；不改 headless/algo 行为（引擎仅 `RouteState` 类型收缩）。

## 3. 状态架构（S1）

### 3.1 UiState 与纯函数 reducer（`js/ui/state.ts`）

```typescript
export type UiMode = 'LIVE' | 'REPLAY';
export type RoutePickPhase = 'idle' | 'picking-goal';

export interface UiState {
  mode: UiMode;
  livePaused: boolean;          // 实时暂停由 UI 拥有；回放期间保持不变，退出即恢复
  routePick: RoutePickPhase;    // 右键选路流程（从引擎剥离）
  pickStartIdx: number | null;
  scenarioName: string | null;
  inspectorOpen: boolean;       // 默认 true
}

export type UiEvent =
  | { type: 'TOGGLE_PAUSE' }                    // 仅 LIVE 生效（REPLAY 中 no-op，回放控制由 controller 直连）
  | { type: 'ENTER_REPLAY' } | { type: 'EXIT_REPLAY' }
  | { type: 'RIGHT_CLICK'; idx: number }        // 仅 LIVE。idle→picking-goal(记录 idx)；
                                                // picking-goal：reducer 不改状态，由 controller 尝试 findRoute
                                                //   成功 → actions.setRoute 后补发 ROUTE_SET（→idle）
                                                //   失败 → 保持 picking-goal（与现行为一致）
  | { type: 'CANCEL' }                          // 取消选点（REPLAY 的 Esc 由 controller 先处理为退出回放）
  | { type: 'SCENARIO_CHANGED'; name: string }  // 重置 routePick/pickStartIdx
  | { type: 'ROUTE_SET' }                       // picking-goal → idle
  | { type: 'TOGGLE_INSPECTOR' };

export function reduce(state: UiState, e: UiEvent): UiState;       // 纯函数
export function deriveHint(ui: UiState, routeState: RouteState): string;   // 纯函数
```

关键语义（单测覆盖点）：

- `TOGGLE_PAUSE` 在 REPLAY 中为 no-op；`ENTER_REPLAY/EXIT_REPLAY` 不触碰 `livePaused`（**退出回放自动恢复进场前的暂停状态**——修复丢失 bug）
- REPLAY 中 `RIGHT_CLICK` 被忽略（回放不给实时世界设路线）
- `SCENARIO_CHANGED` 重置选路流程

### 3.2 副作用分离（`js/ui/controller.ts`）

`UiController` 持有 UiState；`dispatch(event)` 走 reduce 后对比差异执行副作用：

| 状态变化 | 副作用 |
|---|---|
| livePaused 变化 | `deps.actions.setLivePaused(v)`（main 侧写 debug.paused） |
| mode 变化 | 显示/隐藏回放栏；进入时 `replay.load(...)` 由触发方传入 thunk |
| inspectorOpen 变化 | 面板 display 切换 |
| routePick 变化 | 提示条重渲染（经 deriveHint） |

回放播放态（`replay.state.playing`）不进 UiState（它属于 ReplayController），提示文案二者按需组合。

### 3.3 引擎状态收缩

`RouteState` 由 5 态缩为 `'NONE' | 'ACTIVE' | 'ARRIVED'`（删除 `SETTING_START`/`SETTING_GOAL`）。已核实 headless 不引用这两个 UI 态；ui.ts 的 `getRouteState/setRouteState/setClickStartIdx/getClickStartIdx` 四个回调连同 main.ts 的 `clickStartIdx` 模块变量一并删除。

### 3.4 UIContext 收窄（`UiDeps`）

```typescript
export interface UiDeps {
  canvas: HTMLCanvasElement;
  camera: THREE.PerspectiveCamera; scene: THREE.Scene; controls: OrbitControls;
  replay: ReplayController;
  engine: {                                    // 只读视图
    readonly mapDef: MapDefinition; readonly track: TrackGeometry;
    readonly roadGraph: RoadGraph; readonly route: Route | null;
    readonly routeState: RouteState;
  };
  actions: {                                   // UI 改世界的唯一通道
    loadScenario(json: unknown): boolean;
    loadDefaultScenario(map: string): void;
    exportRun(): void;
    setRoute(route: Route, startIdx: number): void;
    setLivePaused(paused: boolean): void;
  };
}
export function setupUI(deps: UiDeps): { tick(dt: number): void };
```

main.ts 的 gameLoop 改为调用 `ui.tick(realDt)`（内部节流更新 inspect/hud 标签），取代散调 `updateInspectPanel`。

## 4. 模块结构（S1）

```
js/ui/
  index.ts         setupUI 编排：组装 controller + 各面板 + 键盘分发器；暴露 tick(dt)
  state.ts         UiState + reduce() + deriveHint()（纯逻辑）
  controller.ts    UiController：dispatch、副作用订阅、replay 交互
  toolbar.ts       世界簇（地图/场景下拉）+ 控制簇（⏸ ⏪ 💾 📂 ↺）事件接线
  hud.ts           提示条（#hint-bar）渲染；HUD 顶部 地图/场景 标签渲染
  inspector.ts     inspect 面板渲染（原 updateInspectPanel 迁入）+ 折叠显隐
  interactions.ts  画布左键 tooltip + 右键拾取（→ dispatch；路线经 findRoute 构建后走 actions.setRoute）
  replayPanel.ts   回放栏接线（时间轴/问题标记/播放/速度/步进/关闭）
```

`js/ui.ts` 删除；`main.ts` 导入 `./ui/index.js`。

## 5. 行为修正清单（S2）

| # | 问题 | 修正后语义 |
|---|---|---|
| 1 | 回放退出丢失暂停 | `livePaused` 归 UiState，回放期间不变，退出自动恢复 |
| 2 | Space 语义冲突 | LIVE：Space 切实时暂停；REPLAY：Space 切回放播放/暂停；统一键盘分发器按 mode 路由（不再散落 target 判断） |
| 3 | 回放中切场景/地图错乱 | 切换动作**先自动退出回放**（恢复 livePaused）再加载 |
| 4 | 回放中可对实时世界右键 | REPLAY 中 RIGHT_CLICK 被 reducer 忽略 |
| 5 | 选路不可取消、状态寄生 | `routePick: idle→picking-goal`；**Esc 取消**；ROUTE_SET 回 idle；引擎只留 3 态 |
| 6 | 提示散落 | 统一底部 `#hint-bar`，文案由 `deriveHint` 派生：<br>· LIVE idle：「右键设路线起点 · 左键查看物体 · Space 暂停 · R 重置视角」<br>· LIVE picking：「再次右键设终点（Esc 取消）」<br>· LIVE 暂停：「已暂停（Space 继续）」<br>· REPLAY：「Space 播放/暂停 · ←/→ 单帧 · 拖动时间轴 · Esc 退出回放」 |
| 7 | Inspect 常显 | 折叠按钮 + `I` 快捷键，默认展开 |
| 8 | 快捷键散落 | 统一分发器 + 单处 focus 守卫；Esc 三义：REPLAY→退出回放；picking→取消选点；否则 no-op |

## 6. DOM 分区（布局 A）

```
顶右：#world-cluster   [地图 ▾][场景 ▾]        （世界选择）
      #control-cluster [⏸][⏪][💾][📂][↺]      （仿真/数据/视图控制，与左簇间加分隔线）
左上：#hud            仅遥测（地图/场景/前方/速度/状态/…）—— 路线操作提示移入 #hint-bar
底部：#hint-bar       上下文提示（吸收原 #hud-instructions）
右下：#inspect-panel  加折叠按钮（标题栏），受 inspectorOpen 控制
底部居中： #replay-bar 不变
```

现有元素 id（#map-dropdown、#btn-pause 等）保持不变，仅重新分组与新增 #world-cluster/#control-cluster/#hint-bar/#btn-inspect-toggle。

## 7. 测试与验收

**单元测试（进现有套件）** `js/test/ui/state.test.ts`（无 DOM）：

- reducer 全事件转移：TOGGLE_PAUSE（LIVE 翻转 / REPLAY no-op）；ENTER/EXIT_REPLAY 保持 livePaused；RIGHT_CLICK 两步选点与 REPLAY 忽略；CANCEL；SCENARIO_CHANGED 重置；ROUTE_SET；TOGGLE_INSPECTOR
- `deriveHint` 四种模式文案断言

**浏览器手工验收清单**：

1. **核心回归**：手动暂停 → 进回放 → 退出回放 → 仍处于暂停（旧行为会丢）
2. Space：LIVE 暂停/继续；REPLAY 播放/暂停
3. Esc：回放中退出回放；选点中取消选点（提示条文案同步变化）
4. 回放中切换场景/地图 → 自动退出回放并正常加载
5. 回放中右键不产生路线
6. I 键与按钮折叠 inspect
7. 原有功能无回归：场景加载/导出/导入回放/右键路线/R 视角/地图切换
8. 顶栏两簇与提示条布局符合布局 A 示意

## 8. 实施顺序与风险

顺序（细节交后续实施计划）：① `state.ts` TDD → ② `controller.ts` → ③ 拆分 `js/ui/` 各模块（行为保持）→ ④ DOM 分区（index.html）→ ⑤ 行为修正接线（快捷键/暂停/拾取）→ ⑥ 手工验收 + 全量 `npm test`。

| 风险 | 缓解 |
|---|---|
| 拆模块过程中漏接事件/行为回归 | 按面板逐模块迁移，每步浏览器冒烟；state.ts 先行且被单测锁住核心语义 |
| 引擎 RouteState 收缩处的隐藏引用 | 实施时 grep `SETTING_START|SETTING_GOAL|setRouteState` 全量核对（已初步确认 headless 无引用） |
| replay 与 UiState 的职责边界模糊 | 边界约定：回放内部状态（播放/进度）归 ReplayController；模式与暂停归 UiState；提示文案由两者组合派生 |

## 9. 开放问题（实现时定稿）

- `deriveHint` 是否并入 routeState（ACTIVE/ARRIVED 的文案，如「路线导航中」）——先保持 4 分支，出现需要再加
- inspect 折叠状态是否需要 localStorage 记忆——默认不落盘，保持简单
