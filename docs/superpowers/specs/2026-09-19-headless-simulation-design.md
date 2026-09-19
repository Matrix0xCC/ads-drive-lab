# Headless 仿真模式设计（AI 自助测试 + 数据归档 + 回放）

- 日期：2026-09-19
- 状态：设计已与用户逐节确认，待用户审阅本文档
- 关联：架构约定见 `CLAUDE.md`；历史决策见 `ISSUE_HISTORY.md`

## 1. 背景与目标

当前仿真只能在浏览器中运行、靠人肉眼观察。本设计新增 **headless 模式**：在 Node 中无人值守运行仿真，把所有原始数据与模块中间数据归档到文件，自动检测问题并生成报告，供 AI 自助完成「跑场景 → 归档 → 检测 → 读报告 → 钻取原始数据 → 修复 → 同种子重跑对比」的闭环；同时用户能在浏览器中回放任意一次运行，直接看到故障现场。

成功标准（AI 工作流，即验收路径）：

1. 一条命令完成运行 + 归档 + 检测 + 报告：
   `npm run sim -- --scenario scenarios/oval-avoid.json --seed 7`
2. AI 先读 `runs/<id>/report.md`（判定 + 问题列表 + 指标 + 钻取指引），再按指引 `grep` `frames.jsonl` 取原始数据
3. 修复后重跑同场景同种子，两次报告可逐项对比（确定性保证）
4. 用户在浏览器回放面板选择该运行，拖动时间轴、点击问题标记跳到故障帧

## 2. 范围

**包含**：headless runner（场景加载 → 确定性运行 → 归档 → 检测 → 报告）；场景 JSON + CLI；四类自动检测（安全/感知/规划行为/跟踪任务）；浏览器回放（时间轴 + 问题标记）。

**非目标**（明确不做，YAGNI）：

- 内置批量扫描/矩阵运行（shell 循环足够）
- 离线重分析 CLI（仿真确定性，重跑成本秒级，重分析无优势）
- 归档数据压缩/精简档位（先全量，量级可接受）
- 跨平台位级一致性保证（保证同机器同 Node 版本一致）
- 远程/多人运行管理

## 3. 总体架构

```
浏览器（实时）   rAF 累积器(固定 1/60s) → SimEngine.step() → bus 'control:state' → renderer/HUD
headless（确定性）CLI + scenario + seed → SimEngine.step()×N → 帧记录流 → ArchiveWriter + Detector → runs/<id>/
浏览器（回放）   runs/<id>/ → ReplayController → 复用 overlays/HUD
```

### 3.1 SimEngine 提取（`js/sim/engine.ts`）

把 `main.ts` 的世界状态与管线装配搬入 `SimEngine` 类：

- 持有状态：`car/track/obstacles/route/routeState/latestLidarScan/latestTrajectory/latestGPS`
- 持有四个 tick 订阅者及其节拍累加器（感知 20Hz / 规划 15Hz / GPS 10Hz / 控制物理每 tick）
- 内部仍走 `MessageBus`（保持消息队列架构），对外提供：`step(dt = FIXED_DT)`、只读状态访问器、`rebuildWorld(config)`、`setRoute(route, startIdx)`
- `main.ts` 瘦身为：创建 engine + `initScene` + 订阅渲染 + `setupUI` + rAF 驱动

**消息主题契约**（engine 发布，归档器/渲染器各自订阅）：

| 主题 | 频率 | 载荷 | 消费方 |
|------|------|------|--------|
| `tick` | 每步 | `{dt}` | engine 内部订阅者 |
| `perception:result` | 20Hz | `{snapshot, scan, stats}` | 归档器 |
| `planning:result` | 15Hz | `{snapshot, scan, trajectory, state, route}` | 浏览器 Recorder、归档器 |
| `gps:result` | 10Hz | `{reading, truth}` | 归档器 |
| `control:state` | 60Hz(每步) | `{snapshot, lidarScan, trajectory, state}` | 渲染器、归档器(step/control 记录) |
| `sim:event` | 离散 | `{kind, step, t, payload}` | 归档器（kind：状态切换/紧急制动/越界复位/到达） |

### 3.2 固定步长（浏览器行为变更）

浏览器从「可变 rAF dt 直接物理」改为「真实时间累积器 + 固定 1/60s 步进」：

```
acc += realDt; steps=0
while (acc >= FIXED_DT && steps < MAX_CATCHUP(5)) { engine.step(FIXED_DT); acc -= FIXED_DT; steps++ }
if (steps == MAX_CATCHUP) acc = 0   // 防止追帧螺旋
```

理由：物理更稳（游戏引擎常规做法），且保证 headless 与浏览器逐帧数值一致——否则 headless 复现不了浏览器现象，方案失去意义。

### 3.3 目录结构（新增/改动）

```
js/
  sim/
    engine.ts              # 新增：SimEngine（浏览器/headless 共用核心）
    scenario.ts            # 新增：场景 JSON 解析/校验/应用到世界
  shared/
    rng.ts                 # 新增：mulberry32 可播种 PRNG + 子种子派生
    types/interfaces/archive.ts   # 新增：归档记录 schema（record/issue/summary/meta）
  headless/
    run.ts                 # 新增：CLI 入口（参数 → engine → 归档 → 检测 → 报告）
    archive.ts             # 新增：ArchiveWriter（run 目录/JSONL/index 维护）
    detector.ts            # 新增：检测器（帧记录流 + 真值 → issues，纯模块）
    report.ts              # 新增：summary.json / report.md / verdict 生成
  replay.ts                # 新增：浏览器回放控制器（与 ui.ts 平级）
  main.ts                  # 改动：瘦身为渲染/UI 装配 + 固定步长循环
  ui.ts                    # 改动：回放入口与运行列表
  env/{obstacle,localization,track}.ts  # 改动：随机源注入可选 rng 参数
env/renderer/overlays.ts   # 可能小调整：供回放复用
index.html                 # 改动：回放时间轴 DOM/CSS/模式切换
package.json               # 改动：sim 脚本；test 改为 node --test dist/js/test/
.gitignore                 # 改动：加入 runs/
scenarios/*.json           # 新增：场景用例（入库）
```

### 3.4 依赖方向（单向）

`sim/` → env 核心 + algo + shared；`headless/` → sim + shared；渲染侧（main / ui / replay / renderer）依赖所有。**核心永远不 import 渲染**（`env/renderer*`、`ui.ts`、`replay.ts`）。

## 4. 场景与确定性

### 4.1 场景 JSON（`scenarios/*.json`）

```jsonc
{
  "version": 1,
  "name": "oval-avoid-1",
  "map": "oval",                    // MAP_PRESETS 键名
  "seed": 42,                       // 一切随机性的根种子
  "duration": 60,                   // 仿真秒数上限
  "car": { "speed": 25 },           // 可选覆盖；默认赛道起点、朝向切线、25 m/s
  "obstacles": { "mode": "explicit",
    "list": [ { "s": 120, "l": 2.0, "w": 2.0, "len": 2.0, "yaw": 0 } ] },
    // 或 { "mode": "random", "count": 4 } —— 布局由 seed 决定
  "route": { "startS": 0, "goalS": 300 },   // 沿赛道弧长 s 的起终点
  "checks": { }                     // 可选：覆盖默认检测阈值
}
```

规则：

- 障碍物用 **Frenet (s, l)** 声明（非绝对 x/y）：对 AI 直观、对换地图稳健；`yaw` 默认对齐赛道切线，换算到世界坐标由 track 几何完成（s → 采样点索引用中心线累计弧长）
- 路线 `startS/goalS` 转采样点索引后复用现有 Dijkstra（`buildRoadGraph` + `findRoute`）
- 校验失败（缺字段/坏 map 名/越界 s,l/version 不符）→ 明确报错退出，不静默兜底

### 4.2 CLI

```bash
npm run sim -- --scenario scenarios/oval-avoid.json --seed 7          # 场景文件（可覆盖 seed/duration）
npm run sim -- --map oval --obstacles 4 --seed 42 --duration 60       # 快速随机场景（免写文件）
npm run sim -- --scenario xxx.json --out runs/exp1 --fail-on-issue    # 指定输出目录 / 有问题时非零退出
```

参数：`--scenario --map --obstacles --seed --duration --out --fail-on-issue`。默认输出目录 `runs/<YYYYMMDD-HHMMSS>_<name>_s<seed>/`。

### 4.3 随机性与可复现

- `shared/rng.ts`：mulberry32 `createRng(seed)` + `deriveSeed(baseSeed, label)`（字符串哈希派生）
- 注入点：`generateObstacles(track, count, rng?)`、`localize(car, config, rng?)`、`generateTrack(..., rng?)`；浏览器默认 `Math.random` 行为不变
- **每个随机子系统派生独立子种子**：`rng('obstacles')`、`rng('gps')`、`rng('track')`——避免调用顺序影响结果（顺序耦合是确定性最常见的坑）
- `run.json` 记录：seed、场景完整快照（含解析后默认值）、git commit、Node 版本——保证「哪版代码跑的」可追溯

## 5. 归档格式

### 5.1 运行目录

```
runs/<id>/
  run.json        # 元信息 + verdict（见 5.3）
  world.json      # 真值：赛道几何（中心线/内外边界）+ 障碍物精确列表（检测与回放的公共依据）
  frames.jsonl    # 逐帧全量流（5.2）
  issues.json     # 检测结果（6.2）
  summary.json    # 聚合指标（6.3）
  report.md       # 摘要报告（6.3）
runs/index.json   # 全部运行索引（回放列表用）：id/场景/地图/seed/时间/verdict/问题计数/durationS
```

### 5.2 frames.jsonl（每行一条类型化记录）

```jsonc
{"type":"step",       "t":1.20,"step":72,"car":{"x","y","angle","speed","steerAngle"},"control":{lookahead,alpha,steer,throttle}}
{"type":"perception", "t":1.20,"step":72,"scan":{angles,distances,rawDistances,confidences},"stats":{rawHits,filteredHits,frontDist,minDist}}
{"type":"plan",       "t":1.20,"step":72,"state":"AVOID","frenet":{s,l,dl_ds,headingDiff},"candidates":[…15条],"best":{cost,targetL,targetV},"trajectory":{points,type,cost}}
{"type":"gps",        "t":1.20,"step":72,"x":…,"y":…,"err":…}
{"type":"event",      "t":1.20,"step":72,"kind":"state_change","payload":{from,to}}
```

「模块中间数据」来源：绝大多数已存在于 `shared/debug.ts`（感知统计、15 条候选取值、控制内部量）、`LidarScan.rawDistances`、`Recorder` 现有字段，归档器按订阅消息快照成记录，**不为归档单独改造各算法模块**。

体量：约 10MB/分钟（全量含 61 射线原始数据与 15 条候选轨迹点）。JSONL 逐行可读、可 grep、边跑边追加（崩溃保留已有数据）。写盘用同步 append，失败即报错。

## 6. 检测与报告

### 6.1 检测清单（v1）

检测器（`headless/detector.ts`）为纯模块：`feed(record)` 流式接收 + `finish()` 出 issues；无 I/O，可用合成记录做单元测试。阈值集中在配置对象，场景 `checks` 可覆盖。

| 类别 | 检查项 | 默认阈值/口径 |
|------|--------|--------------|
| 安全 | ① 碰撞：车身圆（collisionRadius）与真值障碍物 OBB 相交 | 任意相交即 high |
| 安全 | ② 越界：\|l\| > 赛道半宽（7.5m），含 `checkBounds` 复位事件 | 任意即 high |
| 安全 | ③ EMERGENCY：规划状态区间 | 任意即 medium |
| 感知 | ④ 幻影命中：过滤后命中（conf≥0.5）附近容差内无真值障碍 | 单帧计数；幻影命中/总有效命中 > 1% 报 medium |
| 感知 | ⑤ 漏检：真值障碍在 FOV+量程内但对应角度扇区无命中 | 出现即 medium |
| 感知 | ⑥ GPS 误差：\|GPS−真值\| > 3σ（σ=1.25m） | 超限帧计数，最大误差入 summary |
| 规划行为 | ⑦ 状态机抖动：1s 窗口内状态切换 ≥ 4 次 | 窗口列表 |
| 规划行为 | ⑧ 横向摆动：最优 targetL 单帧跳变 > 10m 出现 ≥ 3 次/15 帧窗口 | 窗口列表 |
| 规划行为 | ⑨ 近距险情：最近 filtered 命中 < 10m 且 speed > 5m/s；含紧急制动事件 | 帧 + 最小距离 |
| 跟踪/任务 | ⑩ 跟踪误差：规划帧 k 的轨迹在 t(k+1) 时刻按时间插值出期望位置，与下一规划帧实际位置比较，位置偏差（含横向分量）> 1.0m 记超限帧 | 报告 RMSE + 超限帧 |
| 跟踪/任务 | ⑪ 路线超时：运行结束未 ARRIVED 且剩余 > 10px | 报剩余距离 |
| 跟踪/任务 | ⑫ 停车异常：ARRIVED 后 1s 内速度 > 1m/s；有路线时未 ARRIVED 而速度 < 0.5m/s 持续 3s（停滞） | 帧区间 |

舒适性指标（jerk/横向加速度/转向速率分位数）与速度统计放 summary，不作 issue。

### 6.2 issue schema

```json
{ "category": "safety|perception|planning|mission",
  "severity": "high|medium|low",
  "check": "collision_true",
  "firstFrame": 431, "lastFrame": 458, "count": 28,
  "summary": "以 12.3m/s 撞上 s=118m 处障碍物",
  "details": { }, "samples": [431, 445, 458] }
```

### 6.3 report.md 与 verdict

`report.md`（AI 第一个读的文件，token 友好）：① 一行判定 `verdict: PASS` 或 `ISSUES: 2 high / 1 medium`（含场景、seed、sim 时长、git commit）② 问题列表（类别/严重度/帧区间/关键数值）③ 关键指标表（到达时间、速度统计、跟踪 RMSE、最小障碍距离、状态统计）④ **钻取指引**：附 `grep '"type":"plan"' frames.jsonl | sed -n 'Xp'` 类示例命令。

`run.json` 含 `verdict` 字段；退出码默认 0，`--fail-on-issue` 时有问题退出非 0（供 shell 批量回归）。

## 7. 浏览器回放

1. **入口**：页面加「回放」模式按钮；运行列表 `fetch('runs/index.json')`（静态服务器从仓库根启动，`runs/` 可达；列表为空时提示先跑一次 `npm run sim`）
2. **加载**：读 `world.json` 重建真值场景（复用 `renderer/geometry`），逐行解析 `frames.jsonl` 入内存
3. **时间轴**（底部 DOM）：播放/暂停、拖动、0.5/1/2/4× 速度、±1 帧步进；**issues 的 firstFrame 在进度条上标红点，点击跳到故障帧**
4. **渲染**（`replay.ts`）：以 step（60Hz）为时钟，各类型取「t ≤ 当前时刻的最近一条记录」，构造与实时管线相同的输入（WorldSnapshot / LidarScan / Trajectory / state）调用现有 overlays 与 HUD——不写第二套渲染
5. **模式互斥**：回放期间停止实时 `engine.step`；退出回放恢复实时模式

## 8. 测试与验证

单元测试（沿用 `node:test`，`js/test/` 下新增子目录；`npm test` 改为 `node --test dist/js/test/` 递归）：

- **rng**：同种子序列一致；不同种子不同；子种子派生稳定（与调用顺序无关）
- **scenario**：解析/默认值/校验失败分支（缺字段、坏 map、非法 s,l、version）
- **detector**：合成帧流，12 项检查各做正向（构造异常）与负向（干净流）断言
- **archive**：写入 → 读回 round-trip；`index.json` 追加正确
- **engine 行为基线**：固定 dt 运行 N 步，车状态黄金数值回归（防 main.ts 重构改变行为；黄金值在实现时人工检查合理后冻结：沿中心线、速度合理、无 NaN）
- **确定性 E2E**：进程内同场景同种子跑两次，两遍记录逐帧完全一致（纳入测试，秒级）

浏览器手工验证清单（重构 main.ts 后必做）：

1. 实时模式：巡航、避障、右键设路线、到达停车、暂停/重置视角、导出导入正常
2. 回放模式：载入一次真实运行，时间轴拖动/播放/速度正常，问题标记可跳转，退出回放实时模式恢复

## 9. 实施顺序与风险

顺序（细节交给 writing-plans）：

1. `rng.ts` + 确定性改造（obstacle/localization/track 注入点）+ 单元测试
2. `SimEngine` 提取 + 浏览器固定步长（含 golden 基线测试 + 浏览器手工验证）
3. `scenario.ts` + `headless/run.ts` + `archive.ts`（含存档 round-trip 测试）
4. `detector.ts` + `report.ts`（含合成帧测试）；加 `scenarios/` 示例用例
5. `replay.ts` + index.html 时间轴 + ui.ts 入口（浏览器手工验证）

| 风险 | 缓解 |
|------|------|
| main.ts 重构导致浏览器行为回归 | golden 基线测试 + 三步浏览器手工清单；engine 提取限定为"搬移+参数化"，不顺手改逻辑 |
| 检测阈值初始值不合理（误报/漏报） | 全部集中在配置对象，场景可覆盖；先保守取值，跑真实场景后校准 |
| 跟踪误差口径在高速/急弯下的解释 | 实现时用合成轨迹（直线/圆弧）先测公式，再上真实运行数据 |
| 归档体量超预期 | JSONL 逐行流可 grep；如必要后续加精简档位（非本版范围） |

## 10. 开放问题（实现时定稿，不阻塞开工）

- 检测阈值默认值需在 2-3 个真实场景校准（已给保守初值，见 6.1）
- `s → 采样点索引` 换算工具的具体归属（`env/track.ts` 或 `sim/scenario.ts`），实现时以最小依赖位置为准
