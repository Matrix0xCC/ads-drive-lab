# 03 · 责任划分：RSS（Responsibility-Sensitive Safety）

> 读者画像：编程经验丰富、算法初学者（本仓库作者）。
> 本文所有概念都尽量回连到你自己写过的代码：`js/algo/plan/paths.ts`（走廊）、
> `js/algo/plan/margin.ts`（裕度标定表）、`js/algo/plan/lead.ts`（前车选取）、
> `js/algo/plan/speed.ts`（ST 速度层）、`js/algo/plan/states.ts`（状态机）。

---

## ① 这套东西解决什么问题

你已经有了**几何**层面的安全（物理走廊、裕度、可达性闸门、紧急制动），
但缺一层**语义**：**"这辆车到底该不该由我来让？"**

缺了这一层，你的规划器只能把所有"离我近的东西"一视同仁 ⇒ 你自己实测出来的两个症状：

- **幽灵刹车 / 无法超车**：`js/algo/plan/paths.ts` 的走廊把**相邻车道**那辆车的膨胀算进我这条车道
  ⇒ 我判"过不去" ⇒ 在 60~100m 外就重刹（`CLAUDE.md` 里记着：315 次硬减速里 101 次发生在 60~100m、
  42 次在 >100m，而前方本车道**确实有车**）。
- **该跟不跟**：跟驰的安全门写成"降不到前车速度就不许跟"（`js/algo/plan/lead.ts`）⇒ 一超过门槛，
  跟驰分支被丢弃 ⇒ 退回**停车线**（那是"停在静止障碍前"的规划）⇒ 对"会走的车"用"要停住"的方式规划。

RSS 就是把"谁该让谁"写成**可计算、可验证、可辩论**的规则的那套框架。
它的目标**不是更保守**，而是：**永不做事故的"原因"**，同时保持**不堵路**。

---

## ② 核心概念 + 术语中英对照

| 术语（英） | 中文 | 一句话解释 |
|---|---|---|
| RSS (Responsibility-Sensitive Safety) | 责任敏感安全 | Mobileye/Intel 提出的多智能体安全模型（arXiv:1708.06374） |
| Duty of Care | 注意义务 | 侵权法里的"合理谨慎"；RSS 是它的**一种形式化解释** |
| nominal safety | 名义安全 | 软硬件都正常时，"决策逻辑是否安全"（RSS 管的这一层） |
| functional safety (FuSa) | 功能安全 | 软硬件**失效**导致的风险（ISO 26262 / ASIL 管这一层；论文 §1.1 明确区分二者） |
| SOTIF | 预期功能安全 | ISO 21448：功能正常但**性能局限**（感知/预测不足）导致的风险 —— 与 RSS 互补 |
| safe distance | 安全距离 | 纵向/横向两种；"最坏情况下也不会撞"的最小间距 |
| dangerous situation | 危险情形 | 距离 < 安全距离（或横向同理）的状态 |
| proper response | 恰当响应 | 进入危险情形后**必须**做的动作（纵向：减速到安全；横向：修正/让行） |
| response time ρ | 响应时间 | 从"对方开始动作"到"我完成反应"的时间窗（**发现 + 决策**，不只是感知时延） |
| `a_max,accel` | 最大加速度 | 我（后车）在 ρ 内**最多**会加速多少 |
| `a_min,brake` | 最小制动能力 | 我此后**至少**能提供的减速度（我承诺的最弱制动） |
| `a_max,brake` | 最大制动能力 | 我假设**对方**能有的强制动（用于最坏情况） |
| right of way | 路权 | "是给的，不是抢的"（given, not taken）；RSS 第 3 条规则 |
| cut-in | 切入 | 旁车横向侵入我的车道；RSS 第 2 条规则专门管它 |
| occlusion | 遮挡 | 视线受限区域（RSS 第 4 条：必须更谨慎） |
| Responsibility | 责任 | 谁"导致了"事故（论文 §3.10）；是**可验证的工程定义**，不等于法律免责 |
| Safety Force Field (SFF) | 安全力场 | NVIDIA 提出的另一套"安全过程"框架（与 RSS 同层次、不同表述） |
| lane coordinate system (s/t) | 车道坐标系 | RSS 在自己的车道坐标系里定义安全距离 —— 与你 Frenet 的 (s, l) 同构 |

---

## ③ 必读材料清单（含阅读顺序）

**阅读顺序**：`1 论文摘要页 → 2 Mobileye 科普页（建立直觉）→ 3 论文 §3.1/§3.3（公式）→
4 §3.4 横向 → 5 §3.7 路权 → 6 §3.9 遮挡 → 7 Autoware right_of_way（工程落地）→ 8 OpenDRIVE 优先级`

| # | 标题（链接） | 为什么读 | 读多少 | 难度 |
|---|---|---|---|---|
| 1 | [On a Formal Model of Safe and Scalable Self-driving Cars（arXiv:1708.06374）](https://arxiv.org/abs/1708.06374) | 原始论文的摘要页：RSS 的定位（白盒、可解释、可验证的安全模型）+ 为什么"统计式验证"不可行（论文给了 10⁻⁹/小时的量化论证） | **必读**，摘要页 5 分钟 | ★★☆ |
| 2 | [Mobileye · Responsibility-Sensitive Safety（科普页）](https://www.mobileye.com/technology/responsibility-sensitive-safety/) | **先建立直觉**：5 条规则的中文/英文解释、"Two Second Rule"、`d_min` 的图示、cut-in 与路权为什么必须形式化 | **必读**，全页 15 分钟 | ★☆☆ |
| 3 | [论文全文（HTML 版 v6）](https://arxiv.org/html/1708.06374v6) §3.1 + §3.3 | **纵向安全距离的定义与公式**（Definition 1 + Lemma 2）；"危险情形 / 恰当响应 / 责任"的定义链 | **必读**，§3.1~§3.3（30 分钟） | ★★★ |
| 4 | 同上 §3.4 + §3.5 | **横向安全距离与横向恰当响应**（含横向速度 μ 的定义），以及纵横两种响应如何合成 | **必读**，§3.4~§3.5（25 分钟） | ★★★ |
| 5 | 同上 §3.7 + §3.8 | **路权规则**（多几何 + 红绿灯 + 非结构化道路）与行人场景 | **选读**（做路口/让行前必读）（20 分钟） | ★★★ |
| 6 | 同上 §3.9 + §3.10 + §3.11 | **遮挡谨慎**（"看不见的地方要当作有东西"）、**责任**的定义、以及 "Utopia"（所有人遵守 ⇒ 零事故）的证明思路 | **必读**，§3.9~§3.11（25 分钟） | ★★★ |
| 7 | [Autoware · extra regulatory elements（`right_of_way`）](https://raw.githubusercontent.com/autowarefoundation/autoware_lanelet2_extension/refs/heads/main/autoware_lanelet2_extension/docs/extra_regulatory_elements.md) | **工程落地版的路权**：`right_of_way` + `yield_lane`；原文关键句"know which lanes in their `conflicting lanes` **can be ignored for object detection**" —— 这就是"邻车道车不该让我减速"的生产写法 | **必读**，Right Of Way 一节（10 分钟） | ★★☆ |
| 8 | [OpenDRIVE · 12.4 Connecting roads（含 12.4.1 优先级）](https://publications.pages.asam.net/standards/ASAM_OpenDRIVE/ASAM_OpenDRIVE_Specification/v1.9.0/specification/12_junctions/12_04_connecting_roads.html) | 地图侧怎么带**路权**：`<priority high="…" low="…"/>`（且只在没有信号灯/标志可推时才写） | **选读**，12.4.1（10 分钟） | ★★☆ |
| 9 | [Lanelet2 · Lanelet Primitives（Regulatory Elements 一节）](https://fzi-forschungszentrum-informatik.github.io/Lanelet2/lanelet2_core/LaneletPrimitives/) | 路权在 Lanelet2 里是**规则元素**（可被多个 Lanelet 引用），不是几何 | **查阅** | ★★☆ |

> 关于 **Safety Force Field（NVIDIA）**：可作为"同一层次的另一种表述"了解，
> 但为遵守本目录的链接纪律，这里只给名字不给 URL ——
> 我检索到了 NVIDIA 官方 PDF，但本工具无法解析 PDF 内容，**未能逐字核实**，
> 需要的话请自行搜索 "Safety Force Field NVIDIA" 并自行核对版本。
>
> 关于 **ISO 21448 (SOTIF)**：同理，`iso.org` 对自动访问返回 403，**未核实**，
> 本文只在概念上引用标准编号（ISO 21448）与论文 §1.1 对 FuSa/nominal safety 的区分（这一区分在已核实的论文正文里）。

---

## ④ 关键机制 / 公式 / 判据

### 4.1 五条"常识规则"（RSS 的起点，论文与 Mobileye 页一致的表述）

| # | 英文原文（论文） | 中文 | 你仓库里的对应/缺口 |
|---|---|---|---|
| 1 | Do not hit someone from behind. | 不要追尾 | 有跟驰（`lead.ts` + `speed.ts`），但判据**不是** RSS 形式（见 4.2） |
| 2 | Do not cut-in recklessly. | 不要鲁莽切入 | **缺**：你只有几何膨胀，没有"切入"这个事件 |
| 3 | Right-of-way is given, not taken. | 路权是给的，不是抢的 | **缺**：`netEdges` 只有 `oneway` |
| 4 | Be careful of areas with limited visibility. | 视线受限处要谨慎 | 方向一致 ✓：`UNKNOWN_HANDLING='speed'` + `coverageFromScan`（`paths.ts`） |
| 5 | If you can avoid an accident without causing another one, you must do it. | 能不撞、且不引起另一起，就必须避 | 部分 ✓：`EMERGENCY` + 可达性闸门（`paths.ts`） |

### 4.2 纵向安全距离（论文 Lemma 2，逐字口径）

**定义（Definition 1）**：后车 `c_r` 与前车 `c_f` 同向行驶。若对**前车任何**不超过 `a_max,brake`
的制动，只要后车在响应时间 `ρ` 内**最多**以 `a_max,accel` 加速、此后以**至少** `a_min,brake`
减速直至停车，都不会碰撞 —— 则这个纵向距离是**安全**的。

**最小安全距离（Lemma 2）**：

```
d_min = [ v_r·ρ + ½·a_max,accel·ρ² + (v_r + ρ·a_max,accel)² / (2·a_min,brake) − v_f² / (2·a_max,brake) ]₊
```

其中 `[x]₊ = max{x, 0}`，符号含义：

| 符号 | 含义 | 量纲 |
|---|---|---|
| `v_r` / `v_f` | 后车 / 前车的纵向速度 | m/s |
| `ρ` | 响应时间（发现 + 决策） | s |
| `a_max,accel` | 后车在 ρ 内的最大加速度 | m/s² |
| `a_min,brake` | 后车此后**至少**能提供的减速度（自己承诺的最弱制动） | m/s² |
| `a_max,brake` | 前车**可能**的最大减速度（假设对方的最强制动） | m/s² |

**论文的推导逻辑**（记 `v_ρ,max = v_r + ρ·a_max,accel`）：最坏情况出现在"两车都停住"时 ——
前车位移 `v_f²/(2·a_max,brake)`，后车位移 `v_r·ρ + ½·a_max,accel·ρ² + v_ρ,max²/(2·a_min,brake)`；
要求"停住时两车距离 > 0"，整理即得上式。

**数值直觉**（`ρ=0.5s, a_max,accel=2, a_min,brake=4, a_max,brake=8`，全部 m/s²）：

| `v_r`（我） | `v_f`（前车） | `d_min` | 读法 |
|---|---|---|---|
| 20 | 0 | **65.4 m** | 前方是静止障碍（≈ 你的停车线场景） |
| 20 | 10 | **59.1 m** | 前车在跑，安全距离只减少了 6m 左右 |
| 20 | 20 | **40.4 m** | **同速也要留 40m**（因为它可能急刹） |
| 20 | 35 | **0 m** | 前车比我快得多 ⇒ 无需安全距离（`[x]₊` 起作用） |

对照你现在的做法（`js/algo/plan/margin.ts` 的 `MARGIN_TABLE`）：
你的裕度是**只与我的绝对速度有关**的分段表（v: 0/5/10/15/25/35 ⇒ 1.15/1.00/0.85/0.75/0.65/0.60 m），
它表达的是"横向跟踪误差需要多少余量"，**不是**"纵向安全距离"。
两者不能互相替代：`marginFor(v)` 里**没有前车速度项**，这正是"对会走的车用要停住的方式规划"的来源。

### 4.3 横向安全距离与横向恰当响应（论文 §3.4）

横向形如：**双方的横向运动 + 响应时间**共同决定最小横向距离（论文用横向速度 `μ` 描述，
Definition 5 定义 `μ`-lateral-velocity）。它管的就是**切入（cut-in）**：

- 判定"危险情形"：横向距离 < 横向安全距离 ⇒ **横向恰当响应**（修正横向位置 / 减速 / 让行）。
- 与纵向**合成**（§3.5）：两个方向各自算恰当响应，再取并集（哪个更严用哪个）。

> 诚实说明：我逐字核对了论文的**纵向**公式（Lemma 2）与五条规则；
> **横向公式的确切写法我没有逐字抄录**，请以论文 §3.4 为准（链接已核实可达）。
> 你真正要带走的是**结构**：横向安全距离是"双方横向速度 × 响应时间 + 常数余量"的函数，
> **而不是**"我的走廊膨胀了 0.6m"。

### 4.4 路权与"看不见的地方"（§3.7 / §3.9）

- **路权**：多车几何 + 红绿灯 + 非结构化道路各有规则；核心是"**路权是给的，不是抢的**"，
  以及"即使我有路权，也不能眼看着事故因为别人违规而发生"。
- **遮挡**：视线受限区域必须**更谨慎**（把看不见的地方当作可能有东西）。
  你的对应物是 `UNKNOWN_HANDLING = 'speed'`（`js/algo/plan/paths.ts`）与覆盖记账
  （`docs/domain.md` P13）："未知 ≠ 自由" ✓ 方向正确。
- **责任（§3.10）**：事故责任按"谁违反了哪条规则/谁没做恰当响应"来归因；
  **Utopia（§3.11）**：如果所有 agent 都遵守 RSS，则零事故 —— 这是它"可证明"的含义。

### 4.5 为什么它是"可证明的安全"而不是"更保守的规划"

论文对"过度保守"有明确批评（原文大意）：一种**有效但无用**的解释是"我的动作不应影响别人"，
即变道前必须找到足够大的间隙、即使别人完全不让也能塞进去 —— 这种解释在多数社会里**过度谨慎**，
会让 AV **堵路**，从而限制可扩展性。RSS 的三个目标因此是：**符合人类直觉**、**能产生敏捷的驾驶策略**、
**可高效验证**（因此它用**归纳式**的定义，避免"蝴蝶效应"式的长时程验证）。

对你的直接启示：**"邻车道有车 ⇒ 我减速让开"既不是 RSS 的纵向规则，也不是它的横向规则** ✗ ——
那是几何膨胀的副产物。RSS 会问：它在**我的车道**里吗？不在 ⇒ 我**没有让行义务**；
它**正在切入**吗（横向速度 + 跨线趋势）⇒ 那才进入横向恰当响应。

---

## ⑤ 与本仓库对照表（我们有什么 / 缺什么）

| 业界概念 | 本仓库对应（相对路径） | 我们还缺什么 |
|---|---|---|
| 纵向恰当响应（proper response） | ST 速度层：`cruise` / `stop` / `emergency`：[`../../js/algo/plan/speed.ts`](../../js/algo/plan/speed.ts)；状态机：[`../../js/algo/plan/states.ts`](../../js/algo/plan/states.ts) | 响应是**按几何**选的（过得去/过不去），不是按**责任**选的 |
| 纵向安全距离 `d_min` | `marginFor(v)` 标定表：[`../../js/algo/plan/margin.ts`](../../js/algo/plan/margin.ts) | **无前车速度项**；RSS 是解析式、可证明，你的表是实测拟合 |
| 跟驰（追尾义务） | `selectFollowTarget`：[`../../js/algo/plan/lead.ts`](../../js/algo/plan/lead.ts)；follow 分支 `min(targetV, vLead + (gap−6m)/1.4s)`：[`../../js/algo/plan/speed.ts`](../../js/algo/plan/speed.ts) | 安全门是"降不到前车速度就**不许跟**"⇒ 门槛外退回**停车线**（对会走的车用要停住的方式规划）。建议：把安全门交给**紧急制动层**兜底，跟驰公式本身在间距不足时已经会给出更低速度 |
| "不在我的车道 ⇒ 我无让行义务" | **没有**。走廊膨胀把邻车道车算进来：[`../../js/algo/plan/paths.ts`](../../js/algo/plan/paths.ts) + [`../../js/algo/plan/corridor.ts`](../../js/algo/plan/corridor.ts) + A 方案 `MARGIN_MOVING_EXTRA = 0.6m`：[`../../js/algo/plan/margin.ts`](../../js/algo/plan/margin.ts) | **车道归属门**（物体中心不在我车道 ⇒ 不参与我车道阻塞）—— 这是幽灵刹车/无法超车的正解 |
| 横向恰当响应 / cut-in | 只有"让位"的观感修正（候选排序 + 限幅让位，未完成）：[`../../js/algo/plan/paths.ts`](../../js/algo/plan/paths.ts) | **切入检测**（横向速度 + 跨线趋势 + 责任转移时刻） |
| 路权（right of way） | 地图侧只有 `netEdges.oneway`：[`../../js/env/track.ts`](../../js/env/track.ts) | **让行偏序**（谁 yield 谁）—— Autoware 的 `right_of_way`+yield lanes、OpenDRIVE 的 `priority(high/low)` |
| "conflicting lanes 里哪些可忽略目标检测" | 无 | 生产级的责任过滤：**该忽略的邻车道不参与感知/决策**（Autoware 明确这么写） |
| 遮挡谨慎（§3.9） | `UNKNOWN_HANDLING = 'speed'` + `coverageFromScan`：[`../../js/algo/plan/paths.ts`](../../js/algo/plan/paths.ts)；`docs/domain.md` P13 | 方向一致 ✓；可再补"按可见距离限速"的显式记账（你的 S4） |
| 责任归因（谁撞谁） | 检测器碰撞判据：[`../../js/headless/checks/safety.ts`](../../js/headless/checks/safety.ts) | 归因仍靠"是不是我撞的"几何判据，没有规则化 |
| 可验证性 | `runs/` 归档 + 检测器 + 同种子 A/B（`docs/domain.md`、`CLAUDE.md`） | 把**每公里非预期制动**固化成检测器指标（RSS 的"可验证"精神） |

---

## ⑥ 可落地的最小练习

### 练习 A（推荐，2~3 小时）：把 RSS 纵向安全距离写成纯函数 + 单测（**纯新增，不改行为**）

1. 新建 `js/algo/plan/rss.ts`：

   ```ts
   export interface RssParams {
     rho: number;          // 响应时间 s
     aMaxAccel: number;    // m/s²
     aMinBrake: number;    // m/s²
     aMaxBrake: number;    // m/s²
   }
   /** 纵向最小安全距离（米）—— 论文 Lemma 2 */
   export function rssLongitudinalMinGap(vRear: number, vLead: number, p: RssParams): number {
     const vRho = vRear + p.rho * p.aMaxAccel;
     const d = vRear * p.rho + 0.5 * p.aMaxAccel * p.rho ** 2
             + vRho ** 2 / (2 * p.aMinBrake)
             - vLead ** 2 / (2 * p.aMaxBrake);
     return Math.max(0, d);   // [x]₊
   }
   ```

   注意：入参/返回值都用**米制**，只在模块边界（调用规划器时）用 `m()` 转仿真单位
   （与仓库"量纲安全"约定一致）。

2. 新建 `js/test/plan/rss.test.ts`，断言：
   - `v_r=20, v_f=0, ρ=0.5, a_max,accel=2, a_min,brake=4, a_max,brake=8` ⇒ **65.375 m**（±1e-6）；
   - `v_f=10` ⇒ **59.125 m**；`v_f=20` ⇒ **40.375 m**；`v_f=35` ⇒ **0**（`[x]₊`）；
   - 单调性：`v_r ↑ ⇒ d_min ↑`；`v_f ↑ ⇒ d_min ↓`；`ρ ↑ ⇒ d_min ↑`；
   - 对称性退化：`v_r = v_f = 0` ⇒ `d_min = 0`。

3. 打印对照表（测试里 `console.log` 即可）：把 `v ∈ {0,5,10,15,25,35}` 的
   `marginFor(v)` 与 `rssLongitudinalMinGap(v, 0, 默认参数)` 并列，**写出你对差异的解释**。

**可量化验收**：`npx tsc && node --test dist/js/test/plan/rss.test.js` 全绿；
`npm test` 全量不回归（本练习是纯新增，不应改变任何行为）。

### 练习 B（进阶，3~4 小时）：给走廊加"车道归属门"（**改行为，需要 A/B**）

1. 在 [`../../js/algo/plan/paths.ts`](../../js/algo/plan/paths.ts) 构造移动物体盒时，先算每个物体中心
   落在哪条车道带（用 02 篇练习 A 的 `laneBandsOf`）。**中心不在我这条车道**的物体：
   它的命中点**不参与"我这条车道"的阻塞判定**（仍参与它自己车道的阻塞，也仍参与**紧急制动**层）。
2. 两条单测（新建 `js/test/plan/lane-responsibility.test.ts`）：
   - **邻车道车不阻塞我**：物体中心 `l=3.75m` ⇒ 我这条车道的候选不再被它判 `blockedHard`；
   - **本车道车照样阻塞我（安全不回归）**：物体中心 `l=0` ⇒ 与改动前**逐位一致**。
3. 同种子 A/B（`scenarios/bj-g4-dujiakan-traffic.json`），记录三组数：
   硬减速采样点、EMERGENCY 帧数、`collision_true` 帧数。

**可量化验收**：① 两条单测全绿 + 全量 `npm test` 不回归；② 硬减速采样点**下降**；
③ `collision_true` **不增加**（这条是否决项）；④ 把三组数写进 `docs/domain.md` 或 `CLAUDE.md`。
若 ③ 不满足 ⇒ **回滚**（你在这个仓库已经有过一次这样的教训：不够格的改动不进仓库）。

---

## ⑦ 常见坑与误解

1. **把 RSS 当"更保守的规划器"** ✗：它是**判据/义务**（什么时候算危险、必须做什么），
   不是轨迹生成器；论文明确反对过度保守（会堵路、限制可扩展性）。
2. **忘记 `[x]₊`** ✗：安全距离不能为负；前车比我快得多时 `d_min = 0` 是**正确**结论。
3. **把 `ρ` 当成"感知时延"** ✗：它是"发现 + 决策"的窗口（含感知、融合、规划、执行的总和）。
4. **四个参数的不对称性**：`a_min,brake` 是**我承诺**的最弱制动（保守取小），
   `a_max,brake` 是**我假设对方**能有的制动（保守取大）。两者取错方向会让安全距离变危险。
5. **用绝对速度做跟车判据** ✗（你现在的 `marginFor(v)`）：它会把"前面那辆正在跑的慢车"
   当成"要停的墙" ⇒ 在 60~100m 就重刹（你自己实测：315 次硬减速里 101 次在 60~100m、
   42 次在 >100m）。RSS 的纵向安全距离**必须含前车速度项**。
6. **安全距离 ≠ 目标速度** ✗：它应该当**约束/兜底**（如"距离 < d_min ⇒ 进入恰当响应"），
   直接拿 `d_min` 反推速度目标会把跟驰做成一惊一乍。
7. **把"责任"当"法律免责"** ✗：RSS 是**工程化的可验证解释**，用来设计/验证决策，不是判决书。
8. **忽略感知/预测误差** ✗（RSS 被批评最多的一点）：RSS 假设你能正确估计别人的速度/位置；
   感知误差要靠**冗余/融合**（05 篇）与**SOTIF（ISO 21448）**那一层去补。
9. **坐标系混用** ✗：RSS 定义在**车道坐标系**（s/t）里，与你的 Frenet 同构；
   直接在笛卡尔坐标里算"安全距离"会漏掉弯道与参考线的影响。
10. **只做纵向、不做横向** ✗：切入（cut-in）是横向事件，必须**单独检测**
    （横向速度 + 跨线趋势 + 责任转移时刻），不要指望"走廊膨胀小一点"能表达它 ——
    你在本仓库已经吃过这个亏（A 方案把 `MARGIN_MOVING_EXTRA` 从 0.6 调到 1.0，
    指标在噪声里来回摆，就是因为缺的其实是**语义层**）。

---

## 附录：链接核实说明（重要）

本文所有链接在写入时都经过 **web_fetch 直接核实**（HTTP 200，且返回内容与标题对应）：

- **arXiv:1708.06374**：摘要页 `arxiv.org/abs/1708.06374`（标题、作者、摘要逐字对上）；
  全文 HTML `arxiv.org/html/1708.06374v6`（**已取到正文**：五条常识规则原文、Definition 1、
  Lemma 2 的纵向公式与推导、§3.4/§3.7/§3.9/§3.10/§3.11 的章节结构、§1.1 的 FuSa/nominal safety 对比）。
- **Mobileye RSS 科普页**：`mobileye.com/technology/responsibility-sensitive-safety/`（200；
  注意旧路径 `/responsibility-sensitive-safety/` 返回 **404**，别用）。
- **Autoware `right_of_way`**：`raw.githubusercontent.com/autowarefoundation/autoware_lanelet2_extension/...`
  （200；含"conflicting lanes 里哪些可忽略目标检测"的原句）。
- **ASAM OpenDRIVE 12.4**：`publications.pages.asam.net/.../12_junctions/12_04_connecting_roads.html`
  （200；含 `<priority high low>` 与"连接道路是唯一允许路面重叠的道路"）。
- **Lanelet2 Primitives**：`fzi-forschungszentrum-informatik.github.io/Lanelet2/lanelet2_core/LaneletPrimitives/`
  （200；含 Regulatory Elements 一节）。

**未核实 / 未能核实 ⇒ 没有给链接**：

- `browse.arxiv.org` —— 本机 DNS 解析失败（`getaddrinfo ENOTFOUND`）。请用 `arxiv.org/abs/1708.06374`
  （已核实）。
- **ISO 21448 (SOTIF)**：`iso.org/standard/77490.html` 对自动访问返回 **403**，未能核实 ⇒
  本文只给标准编号与概念，不给 URL。
- **NVIDIA Safety Force Field**：web_search 命中 NVIDIA 官方 PDF，但本工具**无法解析 PDF 内容**
  ⇒ 只给名字，不给 URL。
- 论文 §3.4 的**横向公式**：我核对了该节的**存在与主题**（`μ`-lateral-velocity 的定义），
  但**没有逐字抄录**公式 ⇒ 文中明确标注"以论文 §3.4 为准"。

若你日后发现某条链接失效，请以**标题**重新检索，不要相信任何"看起来像"的 URL。
