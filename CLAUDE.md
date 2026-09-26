# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

从零搭建的 2D 算法 + 3D 渲染自动驾驶模拟器：TypeScript + Three.js，浏览器运行。用户是算法初学者（编程经验丰富），通过此项目学习自动驾驶系统架构。解释算法时面向"懂代码但不太懂算法"的读者。

## 常用命令

```bash
npm run typecheck   # 仅类型检查
npm run build       # tsc 编译到 dist/
npm run watch       # 监听模式
npm test            # tsc + node --test（dist 下的 .test.js；脚本里用 shell 通配展开文件列表，
#                     兼容 Node 18~24——Node 24 不再接受目录参数，见路线图 R2）

# OSM 地图生成（构建期；联网抓取，--from-raw 可离线复跑）
npm run mapgen -- --name bj-g4-dujiakan --ref G4 --bbox 39.80,116.16,39.89,116.26 \
  --min-length 1500 --length 2000 --start-at-toll

# headless 仿真（运行 + 归档 + 检测 + 报告）
npm run sim -- --scenario scenarios/oval-avoid-1.json    # 场景文件
npm run sim -- --map oval --obstacles 4 --seed 42        # 快速随机场景
# 参数：--scenario --map --obstacles --seed --duration --out --fail-on-issue

# 单个测试文件（需先编译）
npx tsc && node --test dist/js/test/plan/planner.test.js
# 单个测试用例
node --test --test-name-pattern "到达检测" dist/js/test/plan/planner.test.js
```

必须通过 HTTP 服务器访问，且从仓库根目录启动（index.html 的 import map 指向 `./node_modules/three`）：

```bash
npm run serve                 # 开发服务器（js/dev-server.ts）：静态文件 + DELETE /api/runs
#                             （「回放运行」面板的「清理归档」按钮需要它；PORT 环境变量可换端口）
# python3 -m http.server 8080 也可用（纯静态，但清理按钮不可用）
```

修改 TS 源码后必须重新 `npx tsc`——浏览器加载的是 `dist/js/main.js`，不是 `js/` 源码。

## 架构

### 管线数据流（js/main.ts）

```
requestAnimationFrame 主循环 (~60Hz)
  → bus.publish('tick', {dt})
      ├─ 感知订阅者   20Hz → latestLidarScan
      ├─ 规划订阅者   15Hz → latestTrajectory（并写入 Recorder）
      ├─ GPS 订阅者   10Hz → latestGPS
      └─ 控制/物理订阅者 每帧 → control() + stepPhysics() + checkBounds()
  → bus.publish('control:state') → 渲染订阅者 updateScene()
```

各模块通过 `MessageBus`（进程内同步 pub/sub）以固定频率独立运行，这是刻意模拟 ROS 的松耦合结构。

### 目录职责

- `js/env/` — 纯环境模拟（赛道/障碍物/车辆物理/定位噪声/渲染），不含决策逻辑
- `js/algo/` — 感知（perception）、规划（plan/）、控制（control）、全局路径（routePlanner）
- `js/sim/` — SimEngine 共用核心（浏览器/headless 同一管线装配）+ 场景加载（scenario.ts）
- `js/headless/` — headless CLI（run.ts）、归档（archive.ts）、检测器（detector.ts + checks/）、报告（report.ts）
- `js/shared/` — 接口契约（types/interfaces/，含归档记录 schema）、量纲系统（dimensions.ts）、rng、debug
- `js/ui/` — UI 层（`state.ts` 纯状态机 + `controller.ts` 副作用边界 + sidebar/toolbar/hud/inspector/interactions/replayPanel 面板模块）；`js/replay.ts` — 回放控制器；`js/env/renderer/` — Three.js 渲染
- `js/mapgen/` — **OSM 地图管线**（构建期工具，不进浏览器包）：`osm.ts` 纯几何（way 链合并 / 投影 / 均匀重采样 / **弧长窗口平滑** / 曲率统计 / `projectOntoPath` 地图匹配原语）+ `build-map.ts` CLI（Overpass 多端点 failover、原始数据与侧车 meta 入库、`--from-raw` 离线复跑、`--start-at-toll`）
- `maps/` — 派生地图（入库）：`<name>.json`（中心线 + world + 校验统计 + 收费站桩号）、`raw/<name>.osm.json`（原始 Overpass 响应）、`raw/<name>.meta.json`（取数过程信息；**不进地图文件**，否则"重新生成逐字节相同"不成立）；`js/env/maps/<name>.gen.ts` 为机械转换产物（勿手改）
- `scenarios/` — 场景用例（入库）；新增场景需登记 `scenarios/index.json` 才会出现在浏览器场景下拉；`runs/` — 运行归档（gitignore）
- `js/test/` — node:test 单元测试（`.test.ts`，编译后运行到 `dist/`）
- **Node 版本**：`engines` 要求 ≥18，`.nvmrc` 推荐 24（当前开发机）。注意 `dist/` 不清理 → **删/改测试文件后要手动清 dist 里的旧产物**，否则旧测试仍会被 `npm test` 跑到

### 核心设计约定

1. **env/algo 严格分离** — 算法不直接修改环境状态；跨模块数据只走 `shared/types/interfaces/` 定义的接口+工厂函数（类比 ROS .msg）
2. **2D 算法 + 3D 渲染** — algo/ 全部运算在 2D 地面平面；渲染层做 canvas (x,y) → Three.js (x, 0, z) 映射
3. **传感器驱动** — 规划器只信任 LiDAR 做碰撞检测，禁止直接使用障碍物列表（真实系统中规划器不知道障碍物绝对坐标）
4. **量纲安全** — `dimensions.ts` 品牌类型（Px, Radians, PxPerSec, DimLess...）编译时防量纲错误，编译后完全擦除。工厂函数（`px()`, `m()`, `rad()`...）是唯一 unsafe cast 点，只允许在模块边界使用
5. **单位系统** — 所有物理量配置用 `m()` / `mps()` / `mps2()` 书写（`SIM_UNITS_PER_METER = 4`，改比例尺只需改这一个常量），不要硬编码 px 值
6. **UI 状态单一来源** — 所有 UI 行为经 `ui/state.ts` 的纯函数 reduce 驱动，DOM 只是状态的渲染投影；`ui/controller.ts` 是"状态变化 → 副作用"的唯一边界；main 只通过 `UiDeps.actions` 改世界。布局：左侧菜单栏（分区**常展开**：世界/数据）+ **画布左缘纵向 toolbar**（暂停/视角/**跟随**/走廊/刻度/射线/路线，图标+两字标签，开关类高亮=开）+ 画布右上「信息」面板（详细遥测）+ 右下「调试」面板（Inspect，`infoOpen`/`debugOpen` 可折叠）+ 画布左上速度/状态精简行。快捷键：Space 暂停/回放播放、Esc 取消选点/退出回放、R 视角、F 相机跟随、I 折叠「调试」面板、C 走廊可视化、G Frenet 刻度、L LiDAR 射线、T 设路线、WASD 平移视角
  （画布左缘纵向 toolbar 另有同功能按钮；两者默认：刻度/射线开、走廊关、**相机跟随按地图**——长条大世界（OSM 高速）开、径向小世界关）

### 规划器（js/algo/plan/）——纵向-横向解耦（spec: docs/superpowers/specs/2026-09-19-decoupled-planner-design.md）

- **架构**：`paths.ts`（SL 路径层）决定"走哪条"，`speed.ts`（ST 速度层）决定"什么速度"，`planner.ts` 以收敛迭代协调两层后按 l(s(t)) 合成 Trajectory（控制/回放/归档契约不变）；`states.ts` 状态机：CRUISE / FOLLOW / AVOID / DECEL / ARRIVED / EMERGENCY
- **每帧流程**：Frenet 参考线（有 route → 开环，无 route → 闭环）→ 车辆投影 → 巡航目标 `vCap = clamp(最近障碍/2, 17.5, 35)` → **路径层 × 速度上限收敛迭代（≤3 轮；收敛阈 0.2m / 0.5m·s⁻¹，变化不降反增则提前停）** → `solveSpeedLayer` → 合成
- **路径层**：`corridor.ts` 建**走廊族**（舒适 / 物理 / 更严 / 更松，同一几何只差膨胀量），候选由**物理走廊瓶颈处的自由段**派生（间隙中心 + 边界内缩 + 车道中心，≤7 个）；两段式横向（前 40m 五次多项式横移、其后保持），**自适应缩短发展段**（越界点前完成横移，下限按 `S_min=√(5.77·|Δl|·v²/A_LAT_MAX)` 随速度缩放）+ **横向路径夹紧 [l0, target]**（防贴边棘轮复位）；120m 视程（有路线时截到路线剩余，`sEnd`）、越出走廊 = 阻塞
- **双阈值（走廊版）**：舒适走廊 = 车半宽 + `max(1.5m, 标定 p95)`（只影响排名与慢行档）；物理走廊 = 车半宽 + `marginFor(v)`（标定速度表，决定停车线）；更严/更松 = 物理 ±0.15m（迟滞状态机 `tight.ts` 的进入/退出判据，进入用更松、退出用更严 + 3 帧防抖）。横向排名只看 COMMIT_WINDOW=60m（晚承诺）；全部候选连物理走廊都过不去 → 停车线；存在物理可过候选则限速 8m/s 慢行；**可达性闸门**：绑定点取"种子档几何阻塞点 / 末轮最优阻塞点 / 全部候选最早的舒适阻塞点"三者最早（最后一项兜住"域放宽后大角度绕行候选变舒适畅通、判据消失"），判据用**车辆自身减速能力** `car.acceleration`（不是规划侧保守的 A_BRAKE：能不能做到问能力）——到绑定点前降不到速度前提则停车线、停不住则 emergency
- **收敛迭代（stage C）**：`vAssume` 从 `max(当前速度, 慢行档)` 起，每轮用 `margin(vAssume)`/`S(vAssume)` 求路径，再由路径几何反推速度上限 `u`（封死 0 / 紧间隙慢行档 / 横向机动 `lateralSpeedCap` / `vCap`）；几何评估速度下限 = 慢行档（破"停车 → 裕度最大 → 封死 → 永不起步"自锁）
- **曲率限速（A/B 开关 `CURVATURE_SPEED_LIMIT`，默认关）**：开则按道路曲率限速 `v ≤ √(A_LAT_MAX/κ(s))` 并叠加前视制动可行性（`v² ≤ v_lim² + 2·a_brake·Δs`，取全程最小）。**默认关的原因**：它改变所有地图的速度剖面（径向图半径 80~100m → 限速 17~19m/s），属语义变更需单独 A/B；OSM 高速图（最急 R=141m）关闭时会以 vCap 上限 35m/s 跑完含弯全程。headless 用 `--curvature-limit` 打开
- **道路半宽随地图**：`solvePathLayer` 收 `trackHalfWidth`（规划域 `L_MAX` 随地图半宽收窄）；感知的 |l| 分级阈值改为**半宽比例**（0.6/0.8，径向图 0.6×7.5=4.5m / 0.8×7.5=6.0m 与历史写死值一致 → 行为零变化）
- **地图语义进决策（"地图起作用了吗"的可测判据：几何相同、语义不同 → 行为必须不同）**：
  - **地图限速 → vCap**：`track.speedLimitMps`（OSM `maxspeed`）作为巡航速度**硬上限**，取在 17.5m/s 防呆下限**之后**（低限速图 60km/h=16.7m/s 不会被下限抬高）。实测：G4 图最高速 34.9 → **33.27m/s**（=120km/h）
  - **车道语义 → 车道中心**：`laneCentersOf(halfWidth, laneWidth)` 推出车道数与车道中心，名义车道取最靠近路中心者（径向 4 车道 → +1.875m 与历史硬编码**逐位一致**；OSM 3 车道 → 中间车道 `l=0`）。实测：G4 图车的 |l| p50 由 **1.87m → 0.02m**（旧硬编码值在 3 车道路面上是跨车道线的位置）
  - 两者都在 `debug.planning` 暴露（`mapSpeedLimitMps` / `laneOffsetU` / `laneCentersU`）并在调试面板显示
- **速度层**：ST 条目（tMin=0 静态）→ cruise / stop（五次多项式停车线，T 取最小单调解）/ emergency（A_MAX 兜底）
- `polynomial.ts` 五次多项式（曲率 C² 连续）；`collision.ts` 锥形搜索（供旧耦合路径的遗留工具）
- **指标**：`summary.json` / `report.md` 含到达用时、路线进度、**收敛轮数 >1 的帧占比、单帧规划耗时 p50/p95**（`planMs` 为墙钟量，**不进逐帧归档**——否则破坏"同种子逐字节一致"）
- 路线终点自动减速（DECEL_ZONE=25m）+ 到达检测（剩余 <10px → ARRIVED）

### 感知（js/algo/perception.ts）

LiDAR 单线 2D：±60° FOV、2° 分辨率、61 条射线、最大 125m。核心是 **HD Map 背景减除**：命中点投影到 Frenet 后按 |l| 三级分类（<4.5m 高置信障碍 / <6.0m 边缘模糊 / ≥6.0m 边界），边界命中替换为 MAX_SENSOR_DIST。输出 `distances`（规划用，已过滤）/ `rawDistances`（渲染用，真实命中）/ `confidences`。

### 控制（js/algo/control.ts）

Pure Pursuit：δ = atan2(2·L·sin α, ld)，预瞄距离 = clamp(0.4×speed, 10m, 20m)。油门目标 = 轨迹终点速度；**终点速度 ≈0（停车/紧急剖面）→ 油门 0**（0.15 怠速下限只对正值目标保留——否则"停车"变成 6.375m/s 恒速爬行，批量 A/B 回归根因）。

### 地图来源与开环道路（`env/mapDefinitions.ts` + `env/track.ts`）

- `MapDefinition.kind`：`'radial'`（历史：8 个径向控制点闭环，半宽 7.5m、世界 450×200m）与 `'polyline'`（OSM：点列，可开环，半宽按车道数、世界按包围盒）。地图下拉由 `MAP_PRESETS` **动态填充**（新增地图只需登记 MAP_PRESETS，不必改 index.html）。
- **开环语义**：`closed: false` → `innerBoundary` 为空，`outerBoundary` 是**闭合路面多边形**（左缘→末端帽→右缘→起点帽），并且两端帽沿切线**外延 25m**（否则车出生点正好压在端帽上 → 每帧判越界复位）。`buildRoadGraph`/`sToIndex`/`resetToTrack`/`generateObstacles` 都不再绕接。
- **世界尺寸不再是全局常量**：`engine.worldW/worldH` 取地图声明；地面/网格用 `setWorldExtent` 随地图重建。
- **相机跟随**：toolbar「跟随」开关 / `F` 键（状态在 `ui/state.ts` 的 `followCam`）。开启时 `followCar` 每帧把相机与目标点同步平移（保留用户的旋转/缩放）；**换地图时按世界尺寸给默认值**（`SCENARIO_CHANGED.followCamDefault`：长条图默认开、径向图默认关），用户手动切过后不受影响。`R`/重置视角走 `frameSmart`（小世界整图、大世界取车周围 250m）。

### 路线设置（`ui/interactions.ts` + `js/algo/routePlanner.ts`）

**显式模式**（toolbar「路线」按钮或 `T`）：进入后**左键**依次点赛道设起点、终点（悬停显示吸附预览环），设完自动退出，`Esc` 取消 → 中心线路网 Dijkstra → Route → 规划器切到路线模式。
（早期用右键 `contextmenu` 触发：Chrome 在右键**按下**即触发，而右键拖拽是平移视角 → 一动相机就误设起点/终点；右键现在只抑制浏览器菜单。）
左键点击物体显示尺寸；Space 暂停；R 重置视角；**WASD 平移相机**（速度 ∝ 相机距离，俯视也有效；失焦自动清键防漂移）。

### 调试与记录

- `shared/debug.ts` — 跨模块共享调试对象，各模块写入内部状态，ui.ts 的 inspect 面板读取展示；
  `debug.planning.viz` 是**走廊可视化载荷**（自由带四边形 / 瓶颈横断线 / 候选路径与发展段，引用路径层对象零拷贝），
  由渲染层 `env/renderer/overlays.ts` 的 `updateCorridorViz` 绘制（画布左缘 toolbar 或 C 键）。
  候选线在**暂停时**带 `Ln` 编号（与调试面板候选列表 `#n` 一一对应）；回放模式用归档扫描重算
  （`planner.corridorVizForReplay`，见 `js/replay.ts`）并把候选写回 `debug.planning`（`source:'replay'`）。
  带子按**本帧是否被雷达真正扫到**分色：绿 = 观测到（自由）、灰蓝 = 未观测（遮挡阴影/视锥外，属"未知"），
  并在覆盖极限画横断线——见 `docs/domain.md` P13（HD Map 背景减除的固有盲区）
- **Frenet 刻度**（`env/renderer/geometry.ts` 的 `buildFrenetTicksGroup`，独立开关 + G 键，默认开）：
  沿弧长每 10m 一条横线、每 50m 一条亮线 + `s=50m` 数字标签（canvas Sprite）；
  横向参考线 l=0（赛道中心虚线，已有）/±1.875m（车道中心）/±3.75m（车道分隔，已有）/**±L_MAX（规划域边，与走廊同色）**；
  **每条横向线沿路每 100m 打一个数值标签**（`l=+1.9m` / `l=±6.0m`，颜色与线一致，同文本共享贴图）。
  **参考线口径与规划器一致**（有路线用路线开环参考线、s 从路线起点算）→ debug 面板的 S/L 值能直接在地上读；
  路线或地图变化时按对象身份重建该组。
- 运行归档的导出/导入统一走 headless 格式：浏览器 💾 导出 `.run.json` 归档包（`sim/records.ts` 收集 + `headless/bundle.ts` 组装检测/报告），📂 导入后直接在回放器打开；`.run.json` 包与磁盘 `runs/<id>/` 归档内容同构

### Headless 运行与归档（AI 自助测试流程）

`npm run sim -- --scenario ...` → 产出 `runs/<id>/`：`report.md`（判定+问题+指标+钻取指引）、`frames.jsonl`（逐帧全量流，typed records，可 grep）、`world.json`（真值）、`issues.json`、`summary.json`。工作流：先读 `report.md` → 按指引 `grep '"type":"plan"' frames.jsonl` 钻取 → 改代码 → **同场景同种子复跑**对比（确定性保证逐帧一致）。四类自动检测：安全/感知/规划行为/跟踪任务；车辆无 route 时保持静止（与浏览器一致），行驶类场景必须带 route。浏览器可用回放模式打开任意 runs/<id> 查看现场。

**场景同源**：浏览器与 headless 共用同一管线（`Scenario → parseScenario → buildScenarioWorld → SimEngine`）。浏览器顶栏「场景」下拉可加载 `scenarios/` 任意场景（带路线自动开跑），默认「随机场景」=随机种子+随机 3-5 障碍、无路线（右键设路线）；地图下拉切换保留当前场景配置仅换地图。同场景同种子在浏览器不做任何操作时与 headless 逐帧一致。

## 文档地图（新文档的落位规则）

- `docs/superpowers/specs/` — 设计规格（写码前）；`docs/superpowers/plans/` — 实施计划
- `docs/reports/` — 会话/里程碑复盘（冻结快照，按日期一份：`YYYY-MM-DD-<topic>-session.md`）
- `docs/incidents/` — 缺陷档案（一事一档，规则与模板见该目录 README；架构决策暂用 `[决策]` 前缀并入）
- `docs/domain.md` — 领域知识活文档（问题图鉴 P1～P13 + 术语表 + 参数表），持续增补、认知更新直接改
- `ISSUE_HISTORY.md`（根目录）— 总台账：新记录一行一条 + 链接到上述文档；老记录保留原位不搬迁

## 开发规范

- **先讨论方案，再写代码** — 涉及架构决策或新功能，先讨论设计，用户确认后再实现
- **中文注释** — 所有 `algo/` 模块必须有详细注释（算法原理、公式推导、参数选择依据）；`env/` 简洁即可，物理相关部分需注释；接口字段用 JSDoc 注明量纲
- 模块间 import 带 `.js` 后缀（ESM 风格）；`dist/` 镜像 `js/` 目录结构（rootDir="."）
- 架构决策与历史问题记录见 `ISSUE_HISTORY.md`（含关键设计取舍的原因，如 HD Map 背景减除、单位制迁移、状态机重构）
