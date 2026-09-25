# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

从零搭建的 2D 算法 + 3D 渲染自动驾驶模拟器：TypeScript + Three.js，浏览器运行。用户是算法初学者（编程经验丰富），通过此项目学习自动驾驶系统架构。解释算法时面向"懂代码但不太懂算法"的读者。

## 常用命令

```bash
npm run typecheck   # 仅类型检查
npm run build       # tsc 编译到 dist/
npm run watch       # 监听模式
npm test            # tsc + node --test dist/js/test/

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
- `scenarios/` — 场景用例（入库）；新增场景需登记 `scenarios/index.json` 才会出现在浏览器场景下拉；`runs/` — 运行归档（gitignore）
- `js/test/` — node:test 单元测试（`.test.ts`，编译后运行）

### 核心设计约定

1. **env/algo 严格分离** — 算法不直接修改环境状态；跨模块数据只走 `shared/types/interfaces/` 定义的接口+工厂函数（类比 ROS .msg）
2. **2D 算法 + 3D 渲染** — algo/ 全部运算在 2D 地面平面；渲染层做 canvas (x,y) → Three.js (x, 0, z) 映射
3. **传感器驱动** — 规划器只信任 LiDAR 做碰撞检测，禁止直接使用障碍物列表（真实系统中规划器不知道障碍物绝对坐标）
4. **量纲安全** — `dimensions.ts` 品牌类型（Px, Radians, PxPerSec, DimLess...）编译时防量纲错误，编译后完全擦除。工厂函数（`px()`, `m()`, `rad()`...）是唯一 unsafe cast 点，只允许在模块边界使用
5. **单位系统** — 所有物理量配置用 `m()` / `mps()` / `mps2()` 书写（`SIM_UNITS_PER_METER = 4`，改比例尺只需改这一个常量），不要硬编码 px 值
6. **UI 状态单一来源** — 所有 UI 行为经 `ui/state.ts` 的纯函数 reduce 驱动，DOM 只是状态的渲染投影；`ui/controller.ts` 是"状态变化 → 副作用"的唯一边界；main 只通过 `UiDeps.actions` 改世界。布局：左侧菜单栏（手风琴单开：世界/仿真/数据，`sidebarSection`）+ 画布右上「信息」面板（详细遥测）+ 右下「调试」面板（Inspect，`infoOpen`/`debugOpen` 可折叠）+ 画布左上速度/状态精简行。快捷键：Space 暂停/回放播放、Esc 取消选点/退出回放、R 视角、I 折叠「调试」面板

### 规划器（js/algo/plan/）——纵向-横向解耦（spec: docs/superpowers/specs/2026-09-19-decoupled-planner-design.md）

- **架构**：`paths.ts`（SL 路径层）决定"走哪条"，`speed.ts`（ST 速度层）决定"什么速度"，`planner.ts` 单轮协调后按 l(s(t)) 合成 Trajectory（控制/回放/归档契约不变）；`states.ts` 状态机：CRUISE / FOLLOW / AVOID / DECEL / ARRIVED / EMERGENCY
- **每帧流程**：Frenet 参考线（有 route → 开环，无 route → 闭环）→ 车辆投影 → 巡航目标 `vCap = clamp(最近障碍/2, 17.5, 35)` → `solvePathLayer` → `solveSpeedLayer` → 合成
- **路径层**：候选目标 {车道中心, ±2.5m/±5m 挪移, ±走廊边界 4.4m} clamp 去重；两段式横向（前 40m 五次多项式横移、其后保持），**横向路径夹紧 [l0, target]**（防贴边棘轮复位）；120m 视程、命中点/链段-折线距离做碰撞检查
- **双阈值**：CLEAR = 车半宽+1.5m（舒适，只影响排名）；HARD_CLEAR = 车半宽+0.4m（物理可过性，决定停车线）。横向排名只看 COMMIT_WINDOW=60m（晚承诺，不提前选边）；全部候选连 HARD_CLEAR 都过不去 → 停车线（速度层在障碍前停）；存在物理可过候选则选最优并限速 8m/s 慢行；**可达性闸门**：紧段起点前减速到 8m/s 的距离不够（≈70m@30m/s）视为真无解（emergency）
- **速度层**：ST 条目（tMin=0 静态）→ cruise / stop（五次多项式停车线，T 取最小单调解）/ emergency（A_MAX 兜底）
- `polynomial.ts` 五次多项式（曲率 C² 连续）；`collision.ts` 锥形搜索（供旧耦合路径的遗留工具）
- 路线终点自动减速（DECEL_ZONE=25m）+ 到达检测（剩余 <10px → ARRIVED）

### 感知（js/algo/perception.ts）

LiDAR 单线 2D：±60° FOV、2° 分辨率、61 条射线、最大 125m。核心是 **HD Map 背景减除**：命中点投影到 Frenet 后按 |l| 三级分类（<4.5m 高置信障碍 / <6.0m 边缘模糊 / ≥6.0m 边界），边界命中替换为 MAX_SENSOR_DIST。输出 `distances`（规划用，已过滤）/ `rawDistances`（渲染用，真实命中）/ `confidences`。

### 控制（js/algo/control.ts）

Pure Pursuit：δ = atan2(2·L·sin α, ld)，预瞄距离 = clamp(0.4×speed, 10m, 20m)。油门目标 = 轨迹终点速度；**终点速度 ≈0（停车/紧急剖面）→ 油门 0**（0.15 怠速下限只对正值目标保留——否则"停车"变成 6.375m/s 恒速爬行，批量 A/B 回归根因）。

### 路线设置（js/ui.ts + js/algo/routePlanner.ts）

右键赛道：第一次点击设起点，第二次设终点 → 中心线路网上的 Dijkstra → Route → 规划器切换为路线模式。左键点击物体显示尺寸；Space 暂停；R 重置视角。

### 调试与记录

- `shared/debug.ts` — 跨模块共享调试对象，各模块写入内部状态，ui.ts 的 inspect 面板读取展示
- 运行归档的导出/导入统一走 headless 格式：浏览器 💾 导出 `.run.json` 归档包（`sim/records.ts` 收集 + `headless/bundle.ts` 组装检测/报告），📂 导入后直接在回放器打开；`.run.json` 包与磁盘 `runs/<id>/` 归档内容同构

### Headless 运行与归档（AI 自助测试流程）

`npm run sim -- --scenario ...` → 产出 `runs/<id>/`：`report.md`（判定+问题+指标+钻取指引）、`frames.jsonl`（逐帧全量流，typed records，可 grep）、`world.json`（真值）、`issues.json`、`summary.json`。工作流：先读 `report.md` → 按指引 `grep '"type":"plan"' frames.jsonl` 钻取 → 改代码 → **同场景同种子复跑**对比（确定性保证逐帧一致）。四类自动检测：安全/感知/规划行为/跟踪任务；车辆无 route 时保持静止（与浏览器一致），行驶类场景必须带 route。浏览器可用回放模式打开任意 runs/<id> 查看现场。

**场景同源**：浏览器与 headless 共用同一管线（`Scenario → parseScenario → buildScenarioWorld → SimEngine`）。浏览器顶栏「场景」下拉可加载 `scenarios/` 任意场景（带路线自动开跑），默认「随机场景」=随机种子+随机 3-5 障碍、无路线（右键设路线）；地图下拉切换保留当前场景配置仅换地图。同场景同种子在浏览器不做任何操作时与 headless 逐帧一致。

## 开发规范

- **先讨论方案，再写代码** — 涉及架构决策或新功能，先讨论设计，用户确认后再实现
- **中文注释** — 所有 `algo/` 模块必须有详细注释（算法原理、公式推导、参数选择依据）；`env/` 简洁即可，物理相关部分需注释；接口字段用 JSDoc 注明量纲
- 模块间 import 带 `.js` 后缀（ESM 风格）；`dist/` 镜像 `js/` 目录结构（rootDir="."）
- 架构决策与历史问题记录见 `ISSUE_HISTORY.md`（含关键设计取舍的原因，如 HD Map 背景减除、单位制迁移、状态机重构）
