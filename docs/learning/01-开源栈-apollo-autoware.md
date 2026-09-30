# 01 · 开源栈：Apollo 与 Autoware 是怎么切模块的

> 写给：已经在本仓库手搓出「MessageBus + 定频订阅 + env/algo 分离 + 单线 LiDAR 背景减除 + Frenet 规划器」的你。
> 目的：让你能判断**哪些是你自己的设计缺陷，哪些只是规模还没到**——以及每个模块的「业界最小实现」长什么样。
> 所有链接都已逐条核实可达（2026-09-27，HTTP 200），无法核实的已删除或标注。

## ① 这套东西解决什么问题

你现在的管线是：`tick` 事件 → 感知 20Hz / 规划 15Hz / GPS 10Hz / 控制每帧订阅 → 渲染。它其实是 ROS
的一种极简实现：**节点、话题、定频回调、消息契约**。你缺的不是"更多算法"，而是三件工业界一定会做的事：
① **消息带时间戳、订阅端做时间对齐**（你的 `MessageBus` 现在只有"注册顺序"这一种时序语义）；
② **安全兜底独立成层**（你的 `EMERGENCY` 还长在规划器里）；
③ **模块边界的显式契约**（你已经有 `shared/types/interfaces/`，但没人拦着你 import 具体实现）。
读 Apollo/Autoware 的价值在于：它们把同一件事做了十年，并把"为什么这么切"写进了文档。

## ② 核心概念 + 术语中英对照

| 英文术语 | 通俗说法 | 本仓库的对应物 | 一句话解释 |
|---|---|---|---|
| Node / Topic / Publisher | 节点 / 话题 / 发布者 | `MessageBus` + 各订阅者（`js/messageBus.ts`） | 模块之间只通过"发消息"耦合 |
| Executor / Callback group | 执行器 / 回调组 | 主循环里的定频分发（`js/main.ts`、`js/sim/engine.ts`） | 谁在什么线程/频率上跑回调 |
| message_filters / ApproximateTimeSynchronizer | 消息时间对齐 | **无** | 不同频率/不同延迟的消息按时间戳配对 |
| proto / msg / 消息契约 | 接口定义 | `js/shared/types/interfaces/` | 跨模块数据的唯一合法形态 |
| ReferenceLine / ReferenceLineInfo | 参考线 / 参考线信息 | `js/algo/plan/frenet.ts` 的 `buildFrenetReference` | 平滑过、带曲率与投影能力的**几何对象**（不是点列） |
| Frame | 帧数据（一次规划的全部输入） | `createWorldSnapshot()` 的产物 | 把预测、定位、底盘、地图打包成一次决策的输入 |
| EM Planner | 百度 EM 规划器 | `js/algo/plan/planner.ts` + `paths.ts`/`speed.ts` | 横向-纵向解耦 + 迭代 |
| Decider / Optimizer | 决策器 / 优化器 | `js/algo/plan/states.ts` / `corridor.ts` + `polynomial.ts` | 先定"走哪/停不停"，再求光滑轨迹 |
| DP / QP | 动态规划 / 二次规划 | 走廊自由段派生（近似 DP）+ 五次多项式（近似 QP） | 粗搜索 + 局部光滑 |
| Router / Routing | 全局路由 | `js/algo/routePlanner.ts` + `buildNetworkRoute` | 从起点到终点的车道级路径 |
| Prediction: Predictor / Evaluator | 预测器 / 评估器 | `js/algo/objects.ts`（只有跟踪，没有预测） | Predictor 出轨迹，Evaluator 出特征/打分 |
| Lanelet / lanelet2_matching | 车道图元 / 物体-车道归属 | **无**（你用 `|l|` 与车道中心硬判） | "这个物体在哪条车道"是个**地图查询**问题 |
| occupancy / free space | 占用 / 自由空间 | `UNKNOWN_HANDLING`（`js/algo/plan/paths.ts`） | "没看到"≠"没有"要用空间推理表达 |
| HMI / DreamView | 可视化与调试面板 | `js/ui/*` + `js/shared/debug.ts` + 渲染层 | 状态可视化 + 开关模块 + 调试工具 |
| Monitor / Guardian | 监视 / 守护 | **无** | 监控模块健康，异常时接管并停车 |
| ODD | 运行设计域 | 场景 JSON + `scenarios/index.json` | 声明"我在这类条件下才工作" |

## ③ 必读材料清单（含阅读顺序）

1. **Apollo 2.0 Software Architecture** — [链接](https://apolloauto.readthedocs.io/en/latest/specs/Apollo_2.0_Software_Architecture/) ·
   **为什么读**：一张图看懂"8 个模块怎么切、谁订阅谁"，而且原文明确写了"每个模块是独立 ROS 节点、通过话题通信"——
   这正是你 `MessageBus` 的原型。**读多少**：必读，全文 15 分钟。**难度**：入门。
   （另有一份 Markdown 源文件可离线存档：[raw 版本](https://raw.githubusercontent.com/ApolloAuto/apollo/9488da6c4724ec8831e655bebebbc118e18b5b21/docs/specs/Apollo_2.0_Software_Architecture.md)）
2. **Apollo 3.0 Software Architecture** — [链接](https://apolloauto.readthedocs.io/en/latest/specs/Apollo_3.0_Software_Architecture/) ·
   **为什么读**：多了 `HMI / Monitor / Guardian` 三个模块，其中 **Guardian 是"安全兜底独立成层"的教科书案例**（Monitor 报故障 → Guardian 切断控制信号并停车）；
   还写了**异步传感器融合**为什么必要（原文：Radar 10ms / Camera 33ms / LiDAR 100ms 三种帧率）——这直接对应你还没做的时间同步。
   **读多少**：必读，重点看 Perception 的 Changes 与 Monitor/Guardian 两节。**难度**：入门。
3. **Apollo · Class Architecture and Overview (Planning)** — [链接](https://apolloauto.readthedocs.io/en/latest/specs/Class_Architecture_Planning/) ·
   **为什么读**：这是**你 `planner.ts` 的工业版**。原文给出 `ReferenceLineInfo`（包装平滑参考线）、`Frame`（装载预测轨迹与自车状态）、
   `EM Planner` 的 `Decider / Path Optimizer / Speed Optimizer` 三类任务"交织迭代"。你读到"decider 分 traffic/path/speed"时，
   会发现你 `states.ts` + `corridor.ts` + `speed.ts` 的划分方向是对的，只是粒度更粗。
   **读多少**：必读，全文 20 分钟。**难度**：中等。
4. **Apollo · Reference Line Smoother** — [链接](https://apolloauto.readthedocs.io/en/latest/specs/reference_line_smoother/) ·
   **为什么读**：**和你的 `js/algo/plan/frenet.ts` 正面相关**。原文给的是"QP + 样条插值"：把参考线按 `n` 段拆开、
   每段用五次多项式 `x=f_i(t), y=g_i(t)`，代价是 `Σ∫(f''')² + ∫(g''')²`（即最小化 jerk），
   约束包括**采样点边界**与**接缝处三阶连续**（`f_k'''(s_k)=f_{k+1}'''(s_0)`）。你现在的参考线是"重采样点列 + 逐段投影"，
   读完你会知道工业界为什么必须平滑到三阶连续：**不平滑的参考线会让曲率与横向加速度估计都带噪声**。
   **读多少**：必读，公式要动手推一遍。**难度**：硬核。
5. **Apollo · QP-Spline-Path Optimizer** — [链接](https://apolloauto.readthedocs.io/en/latest/specs/qp_spline_path_optimizer/) ·
   **为什么读**：SL 层的完整 QP 形式：`l=f_i(s)` 五次多项式、代价 = `w1∫(f')² + w2∫(f'')² + w3∫(f''')²`、
   约束 = 起终点 `(l, l', l'')` + 接缝三阶连续 + **逐采样点的上下边界**（`l_lb,j ≤ f(s_j) ≤ l_ub,j`）。
   你的 `corridor.ts` 算出来的自由带，正是这个 `l_lb/l_ub`——你已经有"边界"，缺的是"用 QP 求光滑曲线"这一步。
   **读多少**：选读（建议先读第 6 条的 ST 版，再回来对照这一篇）。**难度**：硬核。
6. **Apollo · QP-Spline-ST-Speed Optimizer** — [链接](https://apolloauto.readthedocs.io/en/latest/specs/qp_spline_st_speed_optimizer/) ·
   **为什么读**：**和你的 `js/algo/plan/speed.ts` 一一对应**。原文：把障碍与自车都投到 S-T 图，
   速度剖面 `s=f_i(t)` 用五次多项式，代价 = 光滑项 + "与巡航剖面之差" + "与上一帧剖面之差"（后者就是**连续性/防抖**），
   约束 = 初始 `(s,s',s'')` + **单调约束（只能前进）** + 接缝光滑 + **速度上下界**。
   对照你的 `STObstacle {sMin,sMax,tMin,tMax}` 与停车线/紧急剖面，你能读出自己缺的是"**用上一帧剖面做正则**"这一项。
   **读多少**：必读，公式抄到笔记上。**难度**：硬核。
7. **Baidu Apollo EM Motion Planner（论文）** — [arXiv:1807.08048](https://arxiv.org/abs/1807.08048) ·
   **为什么读**：为什么必须**迭代**（路径依赖速度上限、速度依赖路径几何）、多车道用"并行算车道级轨迹再比"。
   你的 `planner.ts` 里"路径层 × 速度上限收敛迭代 ≤3 轮"就是这条思路的简化版，论文给了动机与收敛判据。
   **读多少**：选读（§III 方法部分）。**难度**：硬核。
8. **Apollo · How to add a new Predictor** — [链接](https://apolloauto.readthedocs.io/en/latest/howto/how_to_add_a_new_predictor_in_prediction_module/) ·
   **为什么读**：**插件化结构**的样板：继承 `Predictor` → 在 `prediction_conf.proto` 加枚举 → 在 `prediction_conf.pb.txt` 配
   `obstacle_type/obstacle_status → evaluator_type/predictor_type` → 在 manager 里注册。
   它教你一件你后面一定会需要的事：**把"怎么预测"做成可替换的策略，而不是写死在管线里**。
   **读多少**：选读，10 分钟。**难度**：入门。
9. **Apollo · How to add a new Evaluator** — [链接](https://apolloauto.readthedocs.io/en/latest/howto/how_to_add_a_new_evaluator_in_prediction_module/) ·
   **为什么读**：Evaluator 的职责是"从障碍与自车的原始信息里抽特征、喂给模型"。你的 `objects.ts` 里的速度估计、
   以及你打算做的"责任划分"，本质都是 evaluator 级的东西——**抽成独立插件后，A/B 对照就变成配一行配置**。
   **读多少**：查阅（需要时再看）。**难度**：入门。
10. **Autoware · Autoware concepts（架构设计）** — [链接](https://docs.autoware.org/main/design/autoware-concepts/) ·
    **为什么读**：看另一个工业栈怎么描述"感知-规划-控制"的边界与接口（Autoware 是 ROS 2 原生，比 Apollo 更"教科书"）。
    **读多少**：必读，浏览 + 精读规划相关段落。**难度**：入门。
11. **Autoware · Behavior Path Planner** — [链接](https://autowarefoundation.github.io/autoware.universe_planning/main/planning/behavior_path_planner/) ·
    **为什么读**：这是**"把决策交给场景模块"**的样板：一堆独立模块（变道、绕行、靠边停车、到达目标…）各自产出候选路径，
    再由统一的机制合成。你之前在纠结"横移到底该不该发生、该由谁负责"——Autoware 的答案是**给这类行为一个独立模块**，
    而不是塞进走廊几何里去反推。**读多少**：必读，重点看模块列表与职责划分。**难度**：中等。
12. **Autoware · Behavior Velocity Planner** — [链接](https://autowarefoundation.github.io/autoware.universe_planning/main/planning/behavior_velocity_planner/) ·
    **为什么读**：**"速度规划器按场景切分"**——路口、人行横道、停止线、交通灯各是一个模块，各自往路径上打"必须减速/停止"的约束。
    对照你的 `speed.ts`：你现在所有"该减速"的理由都汇进一个 ST 列表，Autoware 是**每个理由一个模块 + 可独立开关/测试**。
    **读多少**：必读（你规划里"责任划分交给场景"的想法在这里有现成答案）。**难度**：中等。
13. **Autoware · ByteTrack（感知跟踪）** — [链接](https://autowarefoundation.github.io/autoware.universe_planning/main/perception/bytetrack/) ·
    **为什么读**：**你的 `js/algo/objects.ts` 的工业对应物**。看看它的输入输出、航迹管理与参数暴露方式；
    你的 α-β 滤波 + 最近邻关联是它的极简版。**读多少**：选读。**难度**：中等。
14. **Autoware · Pure Pursuit Controller** — [链接](https://autowarefoundation.github.io/autoware.universe_planning/main/control/pure_pursuit/) ·
    **为什么读**：你的 `js/algo/control.ts` 用的是同一个算法；看看工业版暴露了哪些参数（预瞄时间/最小预瞄距离/速度相关增益）。
    **读多少**：查阅。**难度**：入门。
15. **Autoware · Autonomous Emergency Braking (AEB)** — [链接](https://autowarefoundation.github.io/autoware.universe_planning/main/control/autonomous_emergency_braking/) ·
    **为什么读**：**你的 `EMERGENCY` 状态应该长成的样子**——注意它在 **control 层**（独立节点），不在规划器内部。
    "紧急制动"是安全层，不是规划的一种模式。**读多少**：选读，10 分钟。**难度**：入门。
16. **Lanelet2（HD 地图框架）** — [README](https://raw.githubusercontent.com/fzi-forschungszentrum-informatik/Lanelet2/master/README.md) ·
    **为什么读**：**专治你踩过的坑**。README 里 `lanelet2_matching` 的一句话职责是"确定一个物体当前位于/可能位于哪条 lanelet"——
    这就是"车道归属"。还有 `lanelet2_projection`（WGS84 ↔ 局部米制，对应你的 `js/shared/geo.ts`）、
    `lanelet2_traffic_rules`（把地图里的交通规则解释成可执行语义）。**读多少**：必读（Package 列表 + 文档索引）。**难度**：中等。
17. **ROS 2 · Approximate Time Synchronizer 教程** — [链接](https://raw.githubusercontent.com/ros2/message_filters/940367ed05972fa7fde2aadb2c87236de109fee3/doc/Tutorials/Approximate-Synchronizer-Python.rst) ·
    **为什么读**：**你完全没做的那一块**。教程给的是最小形态：两个不同频率的话题 + `queue_size` + `max_delay`，
    时间戳在 `max_delay` 内的消息才算"同一时刻"，配对后进入同一个回调。你的多向传感套件（`js/shared/sensors.ts`）
    一旦默认打开，就必须先有这个，否则 20Hz 雷达与 6Hz 角雷达的点会被当成同一帧。
    **读多少**：选读（本仓库《01》练习 A 就是照着它做）。**难度**：中等。
18. **CARLA / CommonRoad** — [CARLA](https://carla.org/) · [CommonRoad](https://commonroad.in.tum.de/) ·
    **为什么读**：CARLA 是闭环仿真器（可以替换你的 `env/`，用真传感器模型跑你的 `algo/`）；
    CommonRoad 是学术规划基准（有标准场景格式与评价指标）。**读多少**：选读，知道它们存在即可。**难度**：入门。

## ④ 关键机制 / 公式 / 数据结构

### 4.1 模块图与"一切都是话题"

Apollo 2.0 的模块清单（原文）：Perception / Prediction / Routing / Planning / Control / CanBus / HD-Map / Localization；
**3.0 增加 HMI / Monitor / Guardian**。三个关键结论：
- **HD-Map 是"库"不是"节点"**：它不发布话题，而是被当作**查询引擎**调用（"这条车道在哪、限速多少"）。
  你的 `js/env/track.ts` + `attrsAtS` 已经天然是这个形态——**这是对的，不要为了"模块化"把它改成话题**。
- **Planning 的输入里没有原始感知**：规划订阅的是 **Prediction 的输出**（它包住了感知障碍）与交通灯话题。
  也就是说"原始障碍 → 规划"这条捷径在工业界是被**切断**的。你现在的规划器直接读 `lidarScan`，这是有意的
  （传感器驱动、信任 LiDAR），但要注意：**你缺的是"被切断的那一侧"——预测**。
- **Guardian 的存在说明一件事**：安全兜底与规划**解耦**，且可以是"切断信号"这种粗暴动作（Monitor 报故障 → 不允许控制信号到 CAN）。

### 4.2 ReferenceLine：为什么它是"对象"而不是"点列"

Smoother 文档给的形式（可直接对照你的 `frenet.ts`）：
- 分段五次多项式：`x = f_i(t) = a_i0 + a_i1·t + … + a_i5·t⁵`，`y` 同理；
- 代价：`cost = Σ_i (∫(f_i''')² + ∫(g_i''')²)`——**最小化 jerk（三阶导）**，不是最小化曲率；
- 约束：**接缝三阶连续**（`f_k(s_k)=f_{k+1}(s_0)` 及一/二/三阶）+ **均匀采样点上的边界约束**。

你现在的参考线是"重采样 + 弧长窗口平滑"（`js/mapgen/osm.ts` 的平滑），投影是逐段垂距（`js/algo/plan/frenet.ts`）。
**你已经解决了"投影要准"（这是你修幽灵刹车时补上的），但还没解决"曲率要平滑"**——在你的 `CURVATURE_SPEED_LIMIT`
（默认关）一旦打开时，曲率噪声会直接变成速度抖动。这就是这条材料与你的直接关系。

### 4.3 EM Planner：Decider + Optimizer，以及为什么必须迭代

`Class_Architecture_Planning` 原文的三类任务：**Deciders**（traffic / path / speed decider）、
**Path Optimizers**（DP/QP）、**Speed Optimizers**（DP/QP），三者"交织"在一个规划周期里；`Frame` 持有全部输入，
`ReferenceLineInfo` 包装参考线。与你仓库的映射：

| 原文 | 你的实现 | 说明 |
|---|---|---|
| traffic decider | `states.ts` + `tight.ts` 迟滞状态机 | 决定"让/停/走" |
| path decider + path optimizer | `corridor.ts`（自由带）+ `paths.ts`（候选与五阶横移） | 你的"候选派生"相当于粗搜索，缺 QP 光滑 |
| speed decider + speed optimizer | `speed.ts`（ST 条目 → cruise/stop/emergency） | 同一层里混了决策与优化 |
| 迭代 | `planner.ts` 的 ≤3 轮收敛迭代 | **方向一致**：路径与速度互相约束 |

### 4.4 Prediction 的插件结构（你该抄的部分）

插件三件套（两份 howto 原文）：`obstacle_conf.proto` 里的枚举 → `prediction_conf.pb.txt` 里的
`obstacle_type/obstacle_status → evaluator_type/predictor_type` 映射 → manager 的 `Register*()`。
`Evaluator` 出特征（`Evaluate(Obstacle*)`），`Predictor` 出轨迹（`Predict(Obstacle*)`），**同一个障碍可以配多个预测器**。

对你的启发：把 `js/algo/objects.ts` 里"车速估计""车道归属""是否切入"这些**判据**做成 evaluator 式的小函数，
每个都有 `js/test/` 下的单测，A/B 时只换配置。你现在的 `closing.ts`（射线差分测速，已废弃）就是一个
"被淘汰的 predictor"——**工业界淘汰一个 predictor 只需删配置，你需要删代码**。

### 4.5 Autoware：包分层 + 场景模块

`autoware_universe` README 的组件表把包分成 Common / Control / Evaluator / Launch / Localization / Map / Perception /
Planning / Sensing / Simulator / System / Vehicle。规划侧两个核心包：
- `behavior_path_planner`：**路径级**的场景模块（变道、绕行、靠边、到达）各自产出候选路径；
- `behavior_velocity_planner`：**速度级**的场景模块（路口、人行横道、停止线、交通灯）各自往路径上打速度/停车约束。

对照你仓库：你把这**两类**东西都塞进了 `paths.ts` + `speed.ts` 的通用机制里（走廊 + ST 条目）。
通用机制的好处是"一套代码管所有场景"，代价是**每个新场景都要在几何里找位置**——你为"旁边车道的车要不要管"
折腾了两轮（见 `docs/domain.md` 与 CLAUDE.md 的幽灵刹车条目），而 Autoware 的答案是"这类行为有独立模块"。

### 4.6 Lanelet2：车道归属是"地图查询"

README 的包列表（挑与你相关的）：`lanelet2_core`（图元与几何计算）、`lanelet2_projection`（WGS84 ↔ 局部米制）、
`lanelet2_routing`（路由图与可达集）、**`lanelet2_matching`（确定物体位于/可能位于哪条 lanelet）**、
`lanelet2_traffic_rules`（解释地图中的交通规则）。另外 README 明确提醒：
"IO 接口能读写 OSM 格式，**不代表它能处理 OSM 地图**"——语义层（lanelet/规则）与几何层（OSM way）是两回事。

对你的启发：你已经有一张带车道数的派生地图（`maps/<name>.json` + `js/env/maps/<name>.gen.ts`），
但**没有"车道"这个一等公民**：车道中心是运行时用 `laneCentersOf(halfWidth, laneWidth)` 现推的标量数组。
真正的 lanelet 式做法是：地图里导出**每条车道的多边形/中心线 + id**，然后"归属"就是查询。

### 4.7 ROS 2：时间语义与多速率

`ApproximateTimeSynchronizer([sub1, sub2], queue_size, max_delay)`：把时间戳落在 `max_delay` 内的多条消息配对。
Apollo 3.0 文档给了动机的具体数字：**Radar 10ms / Camera 33ms / LiDAR 100ms**。
你的 `MessageBus` 是**同步、按注册顺序**派发的（见 CLAUDE.md"步进顺序"纪律：交通流必须注册在感知之前，
否则同一帧里雷达看到的是上一帧的车——你已经用"注册顺序"这一约定解决了单速率下的时序，但**多速率 + 多传感器时它不够用**：
"规划用的这份扫描是 20Hz 里的哪一帧、比当前 tick 旧多少"现在没有答案。

## ⑤ 与本仓库对照表（我们有什么 / 缺什么）

| 业界概念 | 本仓库文件 | 我们还缺什么 |
|---|---|---|
| 消息契约（proto） | [`js/shared/types/interfaces/`](../../js/shared/types/interfaces/) | 缺**时间戳字段**与版本兼容策略（归档 schema 会演进） |
| 时间同步（message_filters） | [`js/messageBus.ts`](../../js/messageBus.ts) | 缺时间戳对齐与"数据新鲜度"校验（多向 rig 打开后必缺） |
| ReferenceLine（平滑） | [`js/algo/plan/frenet.ts`](../../js/algo/plan/frenet.ts) + [`js/mapgen/osm.ts`](../../js/mapgen/osm.ts) | 投影已准 ✓；缺**三阶连续的平滑**（曲率噪声会变速度抖动） |
| Frame（一次规划的全部输入） | [`js/algo/plan/planner.ts`](../../js/algo/plan/planner.ts) | 缺预测轨迹；缺"输入快照不可变"的显式类型 |
| Decider / Optimizer 分层 | [`js/algo/plan/states.ts`](../../js/algo/plan/states.ts) + [`corridor.ts`](../../js/algo/plan/corridor.ts) | 决策与优化混在走廊几何里；缺 QP 级光滑 |
| Prediction（Predictor/Evaluator） | [`js/algo/objects.ts`](../../js/algo/objects.ts) | **完全没有预测**：只有当前帧的 (s,l,v) |
| 插件的配置化注册 | — | 缺：`closing.ts` 这种被淘汰的策略只能靠删代码下线 |
| lanelet2_matching | [`js/env/track.ts`](../../js/env/track.ts) 的 `laneCentersOf` | 车道不是一等公民；"归属"靠 `|l|` 阈值（你已因此踩过幽灵刹车） |
| occupancy / free space | `UNKNOWN_HANDLING`（[`paths.ts`](../../js/algo/plan/paths.ts)） | "未知即不可通行"是二值处理；业界用占用概率 + 空间推理 |
| Monitor / Guardian | — | 缺：模块健康监控 + 独立接管层（你的 `EMERGENCY` 在规划器内） |
| AEB（控制层独立） | [`js/algo/control.ts`](../../js/algo/control.ts) + `EMERGENCY` 状态 | 紧急制动没有独立触发条件与独立单测 |
| HMI / 调试可视化 | [`js/shared/debug.ts`](../../js/shared/debug.ts) + [`js/env/renderer/overlays.ts`](../../js/env/renderer/overlays.ts) | 已相当完整 ✓（走廊载荷是同类工具里少见的） |
| 闭环仿真 | [`js/env/`](../../js/env/) + [`js/sim/engine.ts`](../../js/sim/engine.ts) | 缺真传感器模型（你的 LiDAR 是理想射线）；CARLA 是替代品 |
| 场景库与 ODD | [`scenarios/`](../../scenarios/) + `index.json` | 缺"场景 → 期望行为"的断言（现在只有检测器的通用检查） |

## ⑥ 可落地的最小练习

### 练习 A：给管线加"数据新鲜度"断言（对齐 message_filters 的最小语义）

**做什么**：给 `tick` 事件带上 `simTime`；感知/规划订阅者各自记录"我这份数据产生于哪个 `simTime`"；
在规划入口断言/统计 `now − scanTime <= SLOP`（建议 `SLOP = 3 × 感知周期 ≈ 0.15s`），超限就写进
`debug.planning` 并在调试面板显示一个计数。
**可量化验收**：
1. 新增单测：把感知订阅者的分频人为改成"每 5 个 tick 一次"，跑 10s，断言"超限计数 > 0"；
   恢复正常频率后断言计数为 0（**正反两面都要有**，否则测试可能永远为真）。
2. 现有 **491 个测试全绿**（不能因为这个改动破坏确定性归档：`simTime` 是确定量，允许进归档；墙钟时间不允许）。
**为什么值得做**：这是 `js/shared/sensors.ts` 的多向 rig（`SURROUND_SENSOR_RIG`，144 条射线、各传感器量程不同）
从"可选"变"默认"之前**必须**有的东西。

### 练习 B：把"车道"提升为一等公民（对齐 lanelet2_matching）

**做什么**：在 `js/mapgen/` 的产物里加一段"车道级导出"（每条车道：id、中心线、左右边界、限速、车道数来源），
并在运行时提供纯函数 `laneletOf(s, l) → laneId | null`；把它用在 `js/algo/plan/inlane.ts` 的归属判断上
（替换现在的 `laneCenters + 阈值`）。
**可量化验收**：
1. 新增单测（放在 `js/test/mapgen/`）：**每条车道的多边形必须包含自己中心线上的采样点，且不包含相邻车道中心线上的采样点**——
   这条断言正是 lanelet2_matching 的判据（"物体位于/可能位于哪条 lanelet"）。
2. 三张真地图（`bj-g4-dujiakan` / `bj-g6-junction` / `bj-3rd-ring`）全部通过；且 `-3rd-ring` 的单车道匝道
   （半宽 1.875m，你曾在那里踩过域比投影误差还窄的坑）能被正确识别为 1 条车道。
3. 回归：`js/test/plan/inlane.test.ts` + 黄金基线全绿。
**为什么值得做**：你的幽灵刹车根因之一是"几何反推车道归属"，这个练习把归属变成**地图查询**，
从此"旁边车道的车"不再依赖 `|l|` 的数值运气。

## ⑦ 常见坑与误解

- **把 Apollo 2.0/3.0 文档当成"今天的 Apollo"**：现在 Apollo 已是 7+/9+，模块名（如 planning 的 `tasks` 配置化）
  与目录结构都变了。老文档的价值在**模块划分的理由与消息流**，不在代码细节。要代码就看新版的 `modules/` 目录。
- **以为 ROS 是"额外的框架负担"**：你的 `MessageBus` 已经承担了节点+话题的角色，**但缺时间语义与 QoS**。
  "要不要上 ROS"不是重点，"要不要有消息时间戳"才是。
- **把 ReferenceLine 当成"中心线点列"**：它是一个**平滑且有约束**的几何对象（Apollo 用 QP+样条保证三阶连续）。
  点列 + 最近邻投影是它的退化版本——你的幽灵刹车正是这个退化版本的一个后果。
- **以为 Prediction 是"深度学习猜轨迹"**：工业界的 prediction 是**多模型插件集合**，第一个上线的通常是
  常速度/常转率这类运动学基线（Apollo 的 `PREDICTOR` 枚举里就有 `FREE_MOVE_PREDICTOR`、
  `MOVE_SEQUENCE_PREDICTOR` 等不同假设的模型），模型只是其中一种 evaluator。
- **以为行为决策是一个大 if-else / 状态机**：Autoware 是"**场景模块 + 速度模块**"的插件集合。
  你的 `states.ts` 是集中式状态机，加场景时会更痛——这也是你"责任划分交给场景"直觉的来源。
- **以为"多传感器"就是多开几条射线**：时间对齐、坐标系（`lanelet2_projection` / 你的 `js/shared/geo.ts`）、
  融合与关联（`lanelet2_matching` / ByteTrack）才是主体；射线数只是最后一步。
- **以为 Lanelet2 能直接吃 OSM 地图**：README 明确写了"IO 能读写 OSM 格式 ≠ 能处理 OSM 地图"。
  你的 `js/mapgen/` 已经在做"从 OSM 派生语义"这件事——**这是对的**，但要注意别把"几何来自 OSM"
  误当成"语义也来自 OSM"（你的 `maxspeed` 几乎全是派生值，文档里已记）。
- **以为 Guardian/Monitor 是"大厂才需要"**：它们的本质是"**故障时不依赖主链路**"。
  在单进程模拟器里，等价物是"一个独立的、极简的检查器"——你已经有了雏形：
  `js/headless/detector.ts` + `js/headless/checks/`。

---

**相关篇章**：[总览与学习路线](./README.md) · [02 地图标准-lanelet2-opendrive](./02-地图标准-lanelet2-opendrive.md) · [03 责任划分-rss](./03-责任划分-rss.md) · [04 预测-数据集与基准](./04-预测-数据集与基准.md) · [05 多目标跟踪-MOT](./05-多目标跟踪-MOT.md) · [06 规划与控制-业界参考](./06-规划与控制-业界参考.md)
