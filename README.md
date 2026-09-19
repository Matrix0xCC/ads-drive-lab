# ADS DriveLab

**2D 算法 + 3D 渲染的自动驾驶模拟器** —— 从零实现感知 / 规划 / 控制全链路，浏览器即开即用，自带 headless 批量测试与归档回放（面向 AI 自助迭代的工作流）。

<!-- 截图位：`python3 -m http.server 8080` 后浏览器打开，截图替换本行 -->

## 特性

- **算法全链路自研**：LiDAR 感知（HD Map 背景减除）→ Frenet **SL/ST 解耦规划**（双阈值通行 / 60m 提交窗口 / 停车线）→ Pure Pursuit 控制 → 运动学自行车模型
- **2D 算法 + 3D 渲染**：决策运算全部在地面平面（Frenet/几何空间），Three.js 做三维俯视渲染；画布随窗口自适应，取景不裁切赛道
- **量纲安全**：品牌类型（`Px` / `Radians` / `PxPerSec` …）编译期防单位混用，`SIM_UNITS_PER_METER` 单点改比例尺
- **场景浏览器 / headless 同源**：同一份 `Scenario → 世界构建` 管线，下拉即载入 `scenarios/` 用例；同种子逐帧一致
- **headless 批量测试**：确定性仿真 + 全量逐帧归档（`frames.jsonl`）+ 安全/感知/规划/任务四类自动检测 + Markdown 报告；改代码后同场景同种子复跑做逐帧 A/B
- **归档与回放**：每次运行可导出 `.run.json` 归档包，浏览器里回放现场、一键跳到问题帧
- **UI 状态单一来源**：纯函数状态机驱动全部 UI 行为，左侧手风琴菜单 + 右侧信息/调试面板

## 快速开始

要求 Node.js 18+。

```bash
npm install
npm run build                 # tsc 编译到 dist/（浏览器加载的是 dist/，改源码后需重新构建）
npm test                      # 单元测试
python3 -m http.server 8080   # 必须走 HTTP，且从仓库根目录启动（import map 指向 ./node_modules）
```

浏览器打开 `http://localhost:8080`。

操作：**右键**赛道设路线起点/终点 · **左键**点击物体查看尺寸 · **Space** 暂停/继续 · **R** 重置视角 · **I** 折叠调试面板 · **Esc** 取消选点 / 退出回放。

## headless 仿真

```bash
npm run sim -- --scenario scenarios/oval-avoid-1.json    # 场景文件
npm run sim -- --map oval --obstacles 4 --seed 42        # 快速随机场景
# 参数：--scenario --map --obstacles --seed --duration --out --fail-on-issue
```

产出 `runs/<id>/`：`report.md`（判定+问题+指标+钻取指引）、`frames.jsonl`（逐帧全量流，可 grep）、`world.json`（真值）、`issues.json`、`summary.json`。

推荐工作流：读 `report.md` → `grep '"type":"plan"' frames.jsonl` 钻取 → 改代码 → **同场景同种子复跑**对比（确定性保证逐帧可比）。

## 架构

```
requestAnimationFrame 主循环
  → MessageBus（进程内同步 pub/sub，模拟 ROS 松耦合）
      ├─ 感知订阅者   20Hz → LiDAR 扫描
      ├─ 规划订阅者   15Hz → Trajectory
      ├─ GPS 订阅者   10Hz → 定位读数
      └─ 控制/物理订阅者 每帧 → Pure Pursuit + 运动学模型
```

| 目录 | 职责 |
|---|---|
| `js/env/` | 纯环境模拟：赛道 / 障碍物 / 车辆物理 / 定位噪声 / Three.js 渲染 |
| `js/algo/` | 感知、规划（`plan/`：SL 路径层 + ST 速度层）、控制、全局路线（Dijkstra） |
| `js/sim/` | SimEngine（浏览器 / headless 共用管线）+ 场景加载 + 归档记录 |
| `js/headless/` | CLI 运行、归档、四类检测器、报告、归档包（`.run.json`） |
| `js/shared/` | 接口契约、量纲系统、确定性 RNG、跨模块调试对象 |
| `js/ui/` | UI 纯函数状态机 + 控制器 + 各面板模块 |
| `scenarios/` | 场景用例（含 `batch/` 批量测试集）；`runs/` 运行归档（gitignore） |

更细的架构约定与开发规范见 [`CLAUDE.md`](CLAUDE.md)；设计取舍、问题根因与负结果记录见 [`ISSUE_HISTORY.md`](ISSUE_HISTORY.md)；规格与实施计划见 [`docs/superpowers/`](docs/superpowers/)。

## 测试与质量

- `npm test`：单元测试全绿（状态机 / 路径层 / 速度层 / 引擎确定性 golden / 归档 / 检测器）
- 批量 A/B 纪律：`scenarios/batch/` 10 场景同种子跑批，单变量实验，负结果回滚并留档
- 规划器解耦升级实测：批量碰撞 6 段 / 195 帧（旧耦合规划器）→ 1 段 / 27 帧（SL/ST 解耦 + 双阈值通行）

## 已知限制

- 障碍物无物理实体（碰撞检测只做记录，不做碰撞响应）——当前目标聚焦"减少碰撞"
- 车辆不建模倒车；无解死口袋处表现为诚实停车
- 单线 2D LiDAR（±60°、61 射线、125m），无动态障碍物
