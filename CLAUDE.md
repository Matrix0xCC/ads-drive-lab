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
python3 -m http.server 8080   # 或 npx serve -l 8080
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
- `js/shared/` — 接口契约（types/interfaces/，含归档记录 schema）、量纲系统（dimensions.ts）、rng、debug、recorder
- `js/ui.ts` / `js/replay.ts` — DOM 交互与回放控制器；`js/env/renderer/` — Three.js 渲染
- `scenarios/` — 场景用例（入库）；`runs/` — 运行归档（gitignore）
- `js/test/` — node:test 单元测试（`.test.ts`，编译后运行）

### 核心设计约定

1. **env/algo 严格分离** — 算法不直接修改环境状态；跨模块数据只走 `shared/types/interfaces/` 定义的接口+工厂函数（类比 ROS .msg）
2. **2D 算法 + 3D 渲染** — algo/ 全部运算在 2D 地面平面；渲染层做 canvas (x,y) → Three.js (x, 0, z) 映射
3. **传感器驱动** — 规划器只信任 LiDAR 做碰撞检测，禁止直接使用障碍物列表（真实系统中规划器不知道障碍物绝对坐标）
4. **量纲安全** — `dimensions.ts` 品牌类型（Px, Radians, PxPerSec, DimLess...）编译时防量纲错误，编译后完全擦除。工厂函数（`px()`, `m()`, `rad()`...）是唯一 unsafe cast 点，只允许在模块边界使用
5. **单位系统** — 所有物理量配置用 `m()` / `mps()` / `mps2()` 书写（`SIM_UNITS_PER_METER = 4`，改比例尺只需改这一个常量），不要硬编码 px 值

### 规划器（js/algo/plan/）

- `planner.ts` 编排入口，`states.ts` 显式状态机：CRUISE / FOLLOW / AVOID / DECEL / ARRIVED / EMERGENCY
- 每帧流程：选 Frenet 参考线（有 route → 开环路线，无 route → 闭环赛道中心线）→ 车辆投影到 Frenet → 生成候选轨迹（横向 ±7/±14m 偏移 **clamp 到可行驶域 |l|≤4.75m 后去重**，典型 3×3=9 条；速度 17.5/25/35 m/s）→ LiDAR 碰撞剪枝 + 代价评估 → 安全候选取最优，**全部碰撞时降级为最低速度**（最小化碰撞能量）
- `polynomial.ts` 五次多项式（保证曲率 C² 连续）；`candidate.ts` 轨迹生成+代价；`collision.ts` 锥形搜索碰撞检测（低置信命中用更大安全余量）
- `W_SWITCH` 切换惩罚防止帧间左右摇摆，改代价权重时勿删
- 路线终点自动减速（DECEL_ZONE=25m）+ 到达检测（剩余 <10px → ARRIVED）

### 感知（js/algo/perception.ts）

LiDAR 单线 2D：±60° FOV、2° 分辨率、61 条射线、最大 125m。核心是 **HD Map 背景减除**：命中点投影到 Frenet 后按 |l| 三级分类（<4.5m 高置信障碍 / <6.0m 边缘模糊 / ≥6.0m 边界），边界命中替换为 MAX_SENSOR_DIST。输出 `distances`（规划用，已过滤）/ `rawDistances`（渲染用，真实命中）/ `confidences`。

### 控制（js/algo/control.ts）

Pure Pursuit：δ = atan2(2·L·sin α, ld)，预瞄距离 = clamp(0.4×speed, 10m, 20m)。

### 路线设置（js/ui.ts + js/algo/routePlanner.ts）

右键赛道：第一次点击设起点，第二次设终点 → 中心线路网上的 Dijkstra → Route → 规划器切换为路线模式。左键点击物体显示尺寸；Space 暂停；R 重置视角。

### 调试与记录

- `shared/debug.ts` — 跨模块共享调试对象，各模块写入内部状态，ui.ts 的 inspect 面板读取展示
- `shared/recorder.ts` — 15Hz 录像、导出/导入 JSON、`analyzeRecording()` 检测 emergency/近距/摇摆帧

### Headless 运行与归档（AI 自助测试流程）

`npm run sim -- --scenario ...` → 产出 `runs/<id>/`：`report.md`（判定+问题+指标+钻取指引）、`frames.jsonl`（逐帧全量流，typed records，可 grep）、`world.json`（真值）、`issues.json`、`summary.json`。工作流：先读 `report.md` → 按指引 `grep '"type":"plan"' frames.jsonl` 钻取 → 改代码 → **同场景同种子复跑**对比（确定性保证逐帧一致）。四类自动检测：安全/感知/规划行为/跟踪任务；车辆无 route 时保持静止（与浏览器一致），行驶类场景必须带 route。浏览器可用回放模式打开任意 runs/<id> 查看现场。

## 开发规范

- **先讨论方案，再写代码** — 涉及架构决策或新功能，先讨论设计，用户确认后再实现
- **中文注释** — 所有 `algo/` 模块必须有详细注释（算法原理、公式推导、参数选择依据）；`env/` 简洁即可，物理相关部分需注释；接口字段用 JSDoc 注明量纲
- 模块间 import 带 `.js` 后缀（ESM 风格）；`dist/` 镜像 `js/` 目录结构（rootDir="."）
- 架构决策与历史问题记录见 `ISSUE_HISTORY.md`（含关键设计取舍的原因，如 HD Map 背景减除、单位制迁移、状态机重构）
