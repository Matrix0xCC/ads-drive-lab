# 05 · 多目标跟踪（MOT）：从"每帧一堆点"到"带 id 的轨迹"

> 读者：编程经验丰富、算法初学者。你已经有 [`js/algo/objects.ts`](../../js/algo/objects.ts)
> 这个"物体层"（聚类 → 最近邻关联 → α-β 滤波 → 2 帧确认 / 4 帧删除，速度带物理门与硬夹）。
> 本文讲清楚：**你这个实现处在 MOT 的什么位置、业界每一环怎么做、你缺的是哪几块**。
> 所有链接均已核实（方式见文末附录）。

---

## ① 这套东西解决什么问题

单帧检测只给你"这一帧有一堆点/一堆簇"。规划器需要的却是：
**"这是同一辆车，它已经跟了我 3 秒，现在以 15m/s 沿这条车道走，位置不确定度 ±0.3m"**。

MOT（Multi-Object Tracking）就是把**检测序列**变成**带稳定 id 的轨迹**，
并给出**状态 + 速度 +（业界标配）协方差**。它是三件事的地基：

- **预测**（[04 篇](04-预测-数据集与基准.md)）需要**历史**才谈得上预测；
- **规划**需要**稳定的 id** 才能谈"责任划分"（谁在超我、谁要切入）；
- **协方差**决定"我能不能硬判"——你踩过的"幽灵刹车"，本质就是**没有不确定度，只能按最坏情况硬判**。

---

## ② 核心概念 + 术语中英对照

| 中文 | 英文 | 一句话解释 |
|---|---|---|
| 检测 / 量测 | detection / measurement | 单帧的输出（框/簇/点集） |
| 航迹 | track | 跨帧维护的物体状态（含 id） |
| 状态表示 | state vector | 要估计的量：位置、速度、（朝向、角速度、尺寸） |
| 过程模型 | motion / process model | 状态怎么随**时间**演化：CV / CTRV / CTRA |
| 量测模型 | measurement model | 状态怎么产生观测（常为线性 H，LiDAR 极坐标时为非线性） |
| 卡尔曼滤波 | Kalman Filter (KF) | 线性高斯下的最优递推估计 |
| 扩展/无迹卡尔曼 | EKF / UKF | 非线性系统（EKF 一阶线性化；UKF 用 sigma 点） |
| α-β 滤波 | alpha-beta filter | 只用两个增益的"穷人 KF"（**你现在就是这个**） |
| 交互多模型 | IMM (Interacting Multiple Model) | 同时跑多个运动模型，按似然加权切换（匀速↔转弯） |
| 数据关联 | data association | 决定"这帧哪个检测属于哪条航迹" |
| 门控 | gating | 先剔除明显不可能的配对（半径/椭圆门） |
| 马氏距离 | Mahalanobis distance | 用协方差归一化的距离：$d^2=\nu^\top S^{-1}\nu$ |
| 全局最近邻 | GNN / Hungarian | 匈牙利算法求**全局**最小代价匹配 |
| 联合概率数据关联 | JPDA | 邻近目标的测量按概率**加权**分配（不硬判） |
| 多假设跟踪 | MHT | 保留多套关联假设、延迟决策 |
| 航迹管理 | track management | 出生/确认/丢失/删除、id 保持 |
| M/N 逻辑 | M-of-N confirmation | 最近 N 帧里命中 M 次才确认（你是「连续 2 帧」，删除用「连续丢失 4 帧」） |
| ID 交换 | ID switch (IDSW) | 两条航迹的 id 互换了（最容易掩盖在总指标里） |
| 遮挡 / 丢失 | occlusion / miss | 目标被挡住或漏检（决定丢失阈值） |
| 传感器融合 | sensor fusion | 多传感器在**物体层**合并（不是合并点） |
| 评测 | CLEAR MOT / IDF1 / AMOTA | MOTA/MOTP、IDF1、AMOTA/sAMOTA |

---

## ③ 必读材料清单（含阅读顺序）

**阅读顺序**：`1 SORT → 2 AB3DMOT → 3 3D MOT baseline（与 2 同源，二选一精读）→ 6 MOT16 → 7 IDF1 → 5 综述（当字典）→ 4 DeepSORT → 8/9 查阅`

| # | 标题（链接） | 为什么读 | 读多少 | 难度 |
|---|---|---|---|---|
| 1 | [Simple Online and Realtime Tracking（SORT, ICIP 2016）](https://arxiv.org/abs/1602.00763) | **最小完整 MOT**：KF + 匈牙利 + 航迹管理，260Hz。你的 `objects.ts` 就是"没有 KF 的 SORT" | **必读**，全文（6 页，30 分钟） | ★☆☆ |
| 2 | [AB3DMOT: A Baseline for 3D Multi-Object Tracking and New Evaluation Metrics](https://arxiv.org/abs/2008.08063) ｜ [镜像](http://arxiv.org.las.ac.cn/abs/2008.08063v1) | 3D 版（LiDAR 检测 + 3D KF + 匈牙利），并提出 **sAMOTA / AMOTA / AMOTP** 三个新指标——与你的 LiDAR 场景同源 | **必读**，方法 + 指标（20 分钟） | ★★☆ |
| 3 | [3D Multi-Object Tracking: A Baseline and New Evaluation Metrics（IROS 2020）](https://arxiv.org/abs/1907.03961) | 与 #2 同源的完整论文（#2 是它的扩展摘要），KITTI/nuScenes 上的完整实验与消融 | **必读**（与 #2 二选一精读） | ★★☆ |
| 4 | [Simple Online and Realtime Tracking with a Deep Association Metric（DeepSORT）](https://arxiv.org/abs/1703.07402) | 引入**外观特征**做关联、把 ID switch 降 45%。你没有相机/外观 ⇒ **知道有这条路即可** | **选读**，摘要 + 关联代价那节（15 分钟） | ★★☆ |
| 5 | [A Survey of Manoeuvring Target Tracking Methods（arXiv 1503.07828）](https://arxiv.org/abs/1503.07828) | **机动目标跟踪全景图**：各种运动模型、机动检测、IMM、PDA/JPDA、MHT、平滑。写得很全，当**字典**查 | **选读/查阅**（38 页，只读你需要的节） | ★★★ |
| 6 | [MOT16: A Benchmark for Multi-Object Tracking](https://arxiv.org/abs/1603.00831) | CLEAR MOT（MOTA/MOTP）这套评测协议的来源与坑（"leaderboard 不该被过度解读"） | **选读**，评测章节（20 分钟） | ★☆☆ |
| 7 | [Performance Measures and a Data Set for Multi-Target, Multi-Camera Tracking（IDF1, ECCVW 2016）](https://arxiv.org/abs/1609.01775) | 讲清 **IDF1 / ID switch**：为什么"只看 MOTA 会被 ID 交换掩盖" | **选读**，指标定义（20 分钟） | ★★☆ |
| 8 | [NATO STO 讲义：跟踪滤波器（α-β / KF / IMM / 数据关联）](https://www.sto.nato.int/publications/STO%20Educational%20Notes/RTO-EN-SET-063bis/EN-SET-063bis-06.pdf) | 工程视角的讲义，把 α-β、KF、IMM、GNN/JPDA 放在一张图里讲；**适合当"公式手册"** | **查阅** | ★★☆ |
| 9 | [FilterPy 文档](https://filterpy.readthedocs.io/en/latest/) | 想动手实现 KF/EKF/UKF/IMM 时的参考实现（Python，但公式与流程可直接照搬到 TS） | **查阅/动手** | ★☆☆ |

> **一句话选读策略**：只有 #1 和 #2 是"必须读完"的，其余按需查。
> 你的目标不是读完，而是**把 α-β 换成 KF、把贪心换成带门控的全局匹配、输出协方差**。

---

## ④ 关键机制 / 公式 / 指标

### 4.1 MOT 四件套 + 你现在的差距

| 环节 | 业界 | 你（[`objects.ts`](../../js/algo/objects.ts)） | 差距 |
|---|---|---|---|
| ① 状态表示 | $x=[p_x,p_y,v_x,v_y]$（+朝向/角速度/尺寸）+ 协方差 $P$ | $(s, l, v_S, v_L)$，**无协方差** | 无不确定度 ⇒ 下游只能硬判 |
| ② 运动模型 | CV / CTRV / CTRA（+ IMM） | 一阶外推 $s \mathrel{+}= v_S\Delta t$ | 无转弯模型、无模型切换 |
| ③ 数据关联 | 门控 + 匈牙利/JPDA | 残差贪心最近邻 + 速度放宽门限 | 贪心在邻近目标处会抢测量 |
| ④ 航迹管理 | M/N 确认、生命周期、id 保持 | 连续 2 帧确认 / 丢失 4 帧删除 | 无"合并/分裂"处理 ⇒ **幽灵双胞胎** |

### 4.2 滤波：KF / EKF / UKF，以及"α-β 就是 KF 的特例"

**KF 五步**（线性高斯，业界标准写法）：

$$
\begin{aligned}
\text{预测：}&\quad \hat x^- = F\hat x,\qquad P^- = FPF^\top + Q\\
\text{新息：}&\quad \nu = z - H\hat x^-,\qquad S = HP^-H^\top + R\\
\text{增益：}&\quad K = P^-H^\top S^{-1}\\
\text{更新：}&\quad \hat x = \hat x^- + K\nu,\qquad P = (I-KH)P^-
\end{aligned}
$$

- $Q$：过程噪声（模型不允许的机动）；$R$：量测噪声；$S$：新息协方差。
- **EKF**：$H$ 换成雅可比 $H(x)$（例如 LiDAR 的极坐标量测 $(\rho,\theta)$ → 笛卡尔状态）。
- **UKF**：用 sigma 点传播非线性，避免雅可比；转弯/强非线性下比 EKF 稳。

**你的 α-β 与 KF 的关系**（这条一定要懂，它决定你下一步怎么做）：

α-β 的递推（把它和你的代码逐行对上）：

$$
\hat x^- = \hat x + \hat v\Delta t,\quad
r = z - \hat x^-,\quad
\hat x = \hat x^- + \alpha r,\quad
\hat v = \hat v + \frac{\beta}{\Delta t}r
$$

> 你的 `stepTracks` 正是这个：先 `t.s += t.vS*dt`（预测），再
> `t.s += ALPHA*resS`、`t.vS += BETA*(resS/dt)`（更新）——**写法完全一致**，
> 只是你多了一道"测量速度超 40m/s 就不更新、更新后再硬夹"的物理保护。

- α-β 是 KF 在"**CV 模型 + 线性位置量测 + 稳态增益**"下的特例：
  给定 $\alpha$、$\beta$，等效于某个 $Q/R$ 组合；稳态时常用近似关系
  $\beta \approx \alpha^2/(2-\alpha)$（把 $\alpha=0.55$ 代进去 ⇒ $\beta\approx0.21$；
  你取 0.25，比"KF 稳态等效值"略大一点 ⇒ 速度跟得更紧、也更毛躁）。
- **α-β 的失效场景**（也就是你会遇到的）：
  1. 机动（前车急转/急减速）——增益是常数，跟不上；
  2. 量测噪声变化（远处物体量测差得多）——$R$ 不是常数；
  3. $\Delta t$ 抖动——你用固定周期规避了（[`objects.ts`](../../js/algo/objects.ts) 的
     `OBJECT_DT`，这点是对的 ✓）；
  4. **没有协方差** ⇒ 下游无法判断"这个 $l$ 到底可信多少"。

### 4.3 运动模型：CTRV / CTRA

CTRV 状态 $(x,y,v,\psi,\omega)$，递推见 [04 篇 4.2](04-预测-数据集与基准.md)（预测与跟踪共用同一模型）。
**关键点**：预测用的模型和跟踪用的模型必须一致（否则"跟出来的速度"喂给"预测的公式"会自相矛盾）。

### 4.4 IMM（交互多模型）四步

动机：一辆车"直行"和"转弯"是两个模式，单个 CV 跟不上。
IMM 同时跑 $r$ 个模型（如 CV + CTRV），按概率加权：

1. **混合/交互**：$\mu_{i|j} = \dfrac{p_{ij}\mu_i}{\sum_i p_{ij}\mu_i}$，
   $\hat x_{0j} = \sum_i \mu_{i|j}\hat x_i$，$P_{0j}$ 同理（含均值差的散布项）；
2. **各模型滤波**：以 $(\hat x_{0j}, P_{0j})$ 为初值跑 KF ⇒ $\hat x_j, P_j$；
3. **模型概率更新**：$\mu_j \propto \Lambda_j \sum_i p_{ij}\mu_i$，
   $\Lambda_j = \mathcal N(\nu_j; 0, S_j)$（新息似然）；
4. **融合输出**：$\hat x = \sum_j \mu_j \hat x_j$，
   $P = \sum_j \mu_j\big[P_j + (\hat x_j - \hat x)(\hat x_j - \hat x)^\top\big]$。

> 工程建议：**先别上 IMM**。先把 CV 的 KF（带协方差）+ 门控 + 全局关联做扎实，
> 因为你的主要误差来源不是"模型不够多"，而是"没有不确定度 + 关联是贪心的"。

### 4.5 数据关联：门控 / 代价 / 全局最优 / JPDA

- **门控（gating）**：只在"统计上可能"的配对里做匹配。马氏距离
  $d^2 = \nu^\top S^{-1}\nu \le \gamma$（$\gamma$ 取 $\chi^2$ 分位数，2 维自由度量级 9.2 ≈ 3σ）。
  你的 `ASSOC_ABS + ASSOC_SPEED_GAIN × |v| × dt`（上限 `ASSOC_MAX`）是**"速度自适应的矩形门"**——
  思路对，但它是**各向同性**的：真实不确定度是**椭圆**（远处/纵向更不确定）。
- **代价矩阵**：$c_{ij}$ 可用"马氏距离 + 尺度差 + 速度差 + 外观距离"加权。
  你是纯位置残差 ⇒ 两个近邻目标靠近时会**抢测量**。
- **全局最近邻（GNN）= 匈牙利算法**：最小化 $\sum c_{ij}$（你的贪心只保证"局部最短边先配"，
  不是全局最优）。SORT/AB3DMOT 都是 KF + 匈牙利。
- **JPDA**：对落入多个门的量测按关联概率 $\beta_{ji}$ **加权更新**（不硬判），
  在密集目标/遮挡时明显优于 GNN；代价是组合爆炸 + 实现复杂。
- **MHT**：保留多套假设延迟决策，最强也最贵。

### 4.6 航迹管理（你踩过坑的地方）

- **确认（M/N）**：SORT 的参考实现用 `min_hits=3`（连续 3 帧命中才确认），DeepSORT 用 `n_init=3`；
  你是 2 帧 ✓。确认太松 ⇒ 幽灵轨迹；太紧 ⇒ 远处小目标跟不上。
- **删除**：丢失 $N$ 帧后删除。**业界取值差异极大**：SORT 参考实现 `max_age=1`（一帧没配上就删，
  因为它假设检测连续）、DeepSORT `max_age=30`（为了熬过遮挡）；**你是 4** ✓。
  太大 ⇒ 幽灵轨迹长期存在（FP 上升）；太小 ⇒ 遮挡后 id 断裂（IDSW 上升）。
- **ID 保持**：遮挡结束后要能**接回原 id**（DeepSORT 用外观；无外观时靠"位置/速度预测 + 较大门限 + 延迟删除"）。
- **合并与分裂**（**你的幽灵双胞胎就是这一类**）：一辆车被聚类切成两簇 ⇒ 两条航迹；
  业界在跟踪器里有**"航迹合并"判据**（匈牙利里做），或在**融合层**先合并测量再跟踪。
  你已经实测到它的破坏力：碎片落进"本车道"⇒ 物理走廊判阻塞 ⇒ 提前急刹。

### 4.7 评测指标

$$
\mathrm{MOTA} = 1 - \frac{\sum_t (\mathrm{FN}_t + \mathrm{FP}_t + \mathrm{IDSW}_t)}{\sum_t \mathrm{GT}_t},
\qquad
\mathrm{MOTP} = \frac{\sum_{t,i} d_{t,i}}{\sum_t c_t}
$$

- **MOTA**：漏检 + 误检 + **ID 交换**的合成惩罚（可能为负）。
- **MOTP**：正确匹配的平均定位误差（衡量"框准不准"，不是"跟得稳不稳"）。
- **IDF1**：以"身份"为中心：
  $\mathrm{IDF1} = \dfrac{2\,\mathrm{IDTP}}{2\,\mathrm{IDTP} + \mathrm{IDFP} + \mathrm{IDFN}}$
  （把每条真值轨迹与预测轨迹的"身份重叠"当 precision/recall）——**ID 交换在 IDF1 里无处可藏**。
- **AMOTA / sAMOTA / AMOTP**（AB3DMOT 提出）：在**多个召回阈值**上算 MOTA 类指标再做平均
  （sAMOTA 是归一化/scaled 版本），比单点 MOTA 更能区分 3D MOT 方法。精确定义见
  [AB3DMOT 论文](https://arxiv.org/abs/2008.08063) 与 [AB3DMOT 代码](https://github.com/xinshuoweng/AB3DMOT)（代码链接由论文摘要给出 ✓）。

**为什么"只看 MOTA 会被 ID 交换掩盖"**：两辆车并行时互相换 id，FN/FP 可能都是 0，
MOTA 却掉了真实的"身份正确性"⇒ 必须同时看 IDF1 与 IDSW 计数。

### 4.8 你的参数 ↔ 业界量（对照着调参）

| 你的参数（[`objects.ts`](../../js/algo/objects.ts)） | 业界的对应物 |
|---|---|
| `CLUSTER_JUMP_ABS/REL/GAP_ANG/JUMP_L` | 检测层聚类/分割阈值（不是跟踪量） |
| `ASSOC_ABS + ASSOC_SPEED_GAIN×|v|×dt`（`ASSOC_MAX` 封顶） | **门控**：$\sqrt{\gamma}$ 的近似（各向同性版） |
| `ASSOC_L` | 横向门限（$l$ 方向的不确定度） |
| `ALPHA` / `BETA` | KF 稳态增益（$\alpha,\beta$） |
| `V_MAX` + 硬夹 | 过程噪声/物理约束（业界用 $Q$ + 限幅） |
| `CONFIRM_HITS = 2` | M/N 确认 |
| `MAX_MISSED = 4` | 航迹删除阈值 |

---

## ⑤ 与本仓库对照表

| 业界环节 | 你的文件 | 我们还缺什么 |
|---|---|---|
| 单帧聚类/分割 | [`js/algo/perception.ts`](../../js/algo/perception.ts)（射线命中 + HD 地图背景减除）+ `detectObjects` | 跨传感器**融合去重**（你自己在 CLAUDE.md 里标了"S3 待做"）：重叠覆盖区同一物体会被两个传感器各看一次 ⇒ 幽灵双胞胎的另一半来源 |
| 状态估计 | [`js/algo/objects.ts`](../../js/algo/objects.ts) `stepTracks`（α-β） | **协方差 $P$**（决定下游能不能软判）、CTRV 状态 |
| 数据关联 | 同文件的 `pairs.sort(cost)` 贪心 | 马氏门控 + 匈牙利（全局最优）；JPDA 属进阶 |
| 航迹管理 | 同文件 ③④⑤ 段（hits/missed/confirmed/排序） | 航迹合并判据（或融合层先合并） |
| 投影/坐标系 | [`js/algo/plan/frenet.ts`](../../js/algo/plan/frenet.ts)（已改逐段投影 + 本体坐标系 [`bodyFrame.ts`](../../js/algo/plan/bodyFrame.ts)） | 投影误差曾达 ~1m ⇒ **跟踪/关联用的 $(s,l)$ 会被它污染**；协方差可以把这部分建模成量测噪声 |
| 下游消费 | [`js/algo/plan/lead.ts`](../../js/algo/plan/lead.ts)（跟驰）、[`js/algo/plan/paths.ts`](../../js/algo/plan/paths.ts)（走廊/车道归属） | 用**协方差**做软归属（"它 80% 概率在我车道"）而不是硬 `|l - center| ≤ half` |
| 评测 | [`js/env/traffic.ts`](../../js/env/traffic.ts) 的真值 + `runs/` 归档 + [`js/test/algo/objects.test.ts`](../../js/test/algo/objects.test.ts) | **ID switch 计数**、跟踪指标（MOTA 级）——你现在只有"速度合理且稳定"这类性质断言 |

---

## ⑥ 可落地的最小练习

### 练习 1（2~3 小时）：α-β → KF，并输出协方差

**做什么**
1. 在 [`js/algo/objects.ts`](../../js/algo/objects.ts) 里把 α-β 换成 4 维 KF
   （状态 $[s, l, v_S, v_L]$，$F$ 为 CV 转移矩阵，$H$ 取位置两行），
   `ObjectTrack` 增加 `pS`/`pL`（或完整 $2\times2$ 的 $P$ 位置块）。
2. 保留你现有的两道物理保护（`V_MAX` 门 + 硬夹）——它们是**约束**，不是滤波的一部分，别删。
3. 新建 `js/test/algo/kf.test.ts` 三条：
   - 静止物体：10 帧后 $|v_S| < 0.2$ m/s，且 $P$ 收敛（不爆炸）；
   - 匀速 15m/s 物体：1s 内 $|v_S - 15| < 0.5$ m/s；
   - 连续 3 帧丢失后恢复：状态不跳变（位置残差 < 1m），协方差先变大再收敛。

**验收判据**
- `js/test/algo/objects.test.ts` 现有断言**全绿**（包括"速度合理且逐帧稳定"那两条）；
- 新增 3 条测试通过；
- 跑 G4 交通场景同种子对比：**速度估计的逐帧最大跳变不增大**（基线 6.75 m/s，你量过）。

### 练习 2（2~3 小时）：给关联加马氏门控 + 全局最优，并写"不抢测量"的单测

**做什么**
1. 用练习 1 的 $P$ 计算 $d^2 = \nu^\top S^{-1}\nu$（$S = HPH^\top + R$），
   `d² > 9.2`（2 维 ~3σ）直接剔除 ⇒ 替换掉现在的矩形门限。
2. 把贪心换成**匈牙利**（几十行的经典实现：代价矩阵 → 增广路/或简化的 Kuhn-Munkres）；
   若暂时不想写，可先做"贪心 + 全局代价重排"的折中，但要在注释里写清它为什么不是最优。
3. 新建 `js/test/algo/assoc.test.ts`：
   **构造两个 $s$ 相距 1.5m、$l$ 相同、速度不同的物体**，喂入一个落在两者之间的检测，
   断言：① 匹配给"更可能"的那条航迹；② 交换检测顺序（确定性）结果不变；
   ③ 连续 20 帧后两条航迹 **id 不互换**。

**验收判据**
- 单测通过；全量 `npm test` 全绿；
- 写一个 10 行的统计脚本：在 G4 交通场景里，统计"同一辆真值车（`traffic` 真值逐帧配对）
  在一段连续出现区间内的 **id 变化次数**"，改动后这个总数**必须下降或持平**；
- 碰撞指标保持 0（任何新增碰撞都算失败）。

---

## ⑦ 常见坑与误解

1. **按射线下标做关联** ✗ —— 你的 `closing.ts` 就是死在这上面（前车速度 p50 = 76m/s、
   可信帧仅 1%）。**射线在障碍之间会跳变**，必须先聚成物体、再按物体关联。
2. **没有协方差** ✗ —— 这是你现在最大的结构性缺口。没有 $P$，
   下游（走廊、车道归属、跟驰安全门）只能"非黑即白"地硬判，
   于是"远处一辆邻车被投影偏了 1m"就能把整条车道判成阻塞 ⇒ 幽灵刹车。
3. **贪心关联在邻近目标处抢测量** ✗ —— 单目/单线 LiDAR 的横向分辨差，
   两车 $l$ 相距 1~2m 时非常常见。要么全局最优，要么概率关联（JPDA）。
4. **调 `MAX_MISSED` 当万能旋钮** ✗ —— 调小 ⇒ 遮挡后 id 断裂（IDSW 上升）；
   调大 ⇒ 幽灵轨迹长期存在（FP 上升）。它要和"门控 + 位置预测"一起调。
5. **只看 MOTA** ✗ —— ID 交换被掩盖，而 ID 交换对你的下游是致命的
   （"我以为还是那辆车"⇒ 责任划分/跟驰目标全错）。看 IDF1/IDSW。
6. **在点云/射线上直接融合多传感器** ✗ —— 应在**物体层**融合（你已经把"S3 跨传感器融合去重"
   标成待做 ✓ 方向是对的）：每个传感器各自成物体 → 用概率/几何合并 → 再跟踪。
7. **忘了时间戳与异步** ✗ —— 业界要处理多传感器不同频率与延迟；你用固定步长总线是**简化**，
   但至少要保持"跟踪用的 $dt$ 与实际时间一致"（你已经用 `OBJECT_DT` 而不是墙钟 ✓）。
8. **用真值速度调试跟踪器** ✗ —— 评测可以用真值（练习里的 id/速度统计），
   但**决策回路**只能用跟踪输出（你在仓库里守的"规划器只信 LiDAR"是对的）。
9. **状态量与观测量混用单位/量纲** ✗ —— 你的品牌类型（`Px` / `PxPerSec`）是很好的工程习惯；
   写 KF 时 $Q$（过程噪声）、$R$（量测噪声）的量纲必须与状态/量测一致，
   混了会得到"能跑但结果是错的"滤波。

---

## 附录：链接核实说明

写入时逐条核实（与 [04 篇](04-预测-数据集与基准.md) 同一套纪律）：

- **web_fetch 直接核实**（arXiv API 比对标题/摘要）：SORT（`1602.00763`）、DeepSORT（`1703.07402`）、
  IDF1 与多目标评测数据集（`1609.01775`）、AB3DMOT（`2008.08063`，摘要里的代码链接
  `github.com/xinshuoweng/AB3DMOT` 也一并核对）、3D MOT baseline（`1907.03961`）、
  机动目标跟踪综述（`1503.07828`）、MOT16（`1603.00831`）。
- **web_search 核实**：FilterPy 文档（`filterpy.readthedocs.io`）、NATO STO 讲义
  （`sto.nato.int` 的 `RTO-EN-SET-063bis/EN-SET-063bis-06.pdf`）。
- **没有写进本文**的：CLEAR MOT 原始论文（Bernardin & Stiefelhagen, 2008，付费墙）、
  匈牙利算法原始论文（Kuhn, 1955，付费墙）、JPDA 原始论文（Fortmann 等, 1983，付费墙）、
  Bar-Shalom 的跟踪教材——这些我只写名字，不给 URL；
  需要时按标题检索，**不要相信任何"看起来像"的链接**。
- 你以前记录的 AB3DMOT（`2008.08063` + 镜像 `arxiv.las.ac.cn/abs/2008.08063v1`）与 `1907.03961` 均确认有效 ✓。

---

**相关篇章**：[总览与学习路线](./README.md) · [01 开源栈-apollo-autoware](./01-开源栈-apollo-autoware.md) · [02 地图标准-lanelet2-opendrive](./02-地图标准-lanelet2-opendrive.md) · [03 责任划分-rss](./03-责任划分-rss.md) · [04 预测-数据集与基准](./04-预测-数据集与基准.md) · [06 规划与控制-业界参考](./06-规划与控制-业界参考.md)
