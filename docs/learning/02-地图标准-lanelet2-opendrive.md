# 02 · 地图标准：Lanelet2 / OpenDRIVE（车道级地图与"车道归属"）

> 读者画像：编程经验丰富、算法初学者（本仓库作者）。
> 本文所有概念都尽量回连到你自己写过的代码：`js/mapgen/`、`js/env/track.ts`、
> `js/algo/plan/frenet.ts`、`js/algo/plan/inlane.ts`、`js/algo/objects.ts`。

---

## ① 这套东西解决什么问题

你现在的"地图"是**一条道路中心线 + 逐段属性**（几车道、限速、半宽）。

业界的高精地图是**车道级**的：每条车道是一个**有 id、有左右边界、有拓扑邻居**的图元。
一旦地图是车道级的，下面这些问题就从"几何反推"变成"地图查询"：

- 这个物体**在哪条车道**？（不是"它的 |l| 落在哪个带里"）
- 我这条车道的**左邻/右邻**是谁？**能不能变道**过去？
- 路口里**谁接谁**（左转/直行/右转分别连到哪条车道）？**谁优先**？
- 前方 60m 那辆车：它和我**同车道**，还是只是"横向离得近"？

你在本仓库实测过的两个症状都指向这一层：

- **幽灵刹车 / 无法超车**：`js/algo/plan/inlane.ts` 用"命中点的 |l| ≤ 车道半宽 + 0.35m"
  判断归属，`js/algo/plan/paths.ts` 的走廊又把邻车道车的膨胀算进我的车道 ⇒ 邻车道的车
  被我当成了**我这条车道上的障碍**。
- **幽灵双胞胎**：物体层把一辆车裂成两个物体，其中一个碎片落在"我车道内"。

**根因**：地图没有"车道"，只有"离中心线多远"。归属判断只能从几何**反推**，而几何反推
对投影误差、车道宽度误差、弯道都很敏感。

---

## ② 核心概念 + 术语中英对照

| 术语（英） | 中文 | 出处 | 一句话解释 |
|---|---|---|---|
| Lanelet | 车道段 | Lanelet2 | **原子**车道段（沿它交通规则不变化）；由一条左边界 + 一条右边界 LineString 围成 |
| Point / LineString / Polygon / Area | 点 / 折线 / 多边形 / 面 | Lanelet2 | 六原语中的几何基元；Area 表示"面状可行驶区域"（路口、停车场） |
| RegulatoryElement | 规则元素 | Lanelet2 | 交通规则（限速、红绿灯、**路权**、交通标志）；被 Lanelet/Area **引用** |
| left / right | 左邻 / 右邻（可变道） | Lanelet2 routing | 与该车道共享一条边界 ⇒ 允许变道 |
| adjacent left / right | 左/右侧邻（**不可**变道） | Lanelet2 routing | 相邻但不共享边界（例如中间有实线/护栏） |
| succeeding | 后继 | Lanelet2 routing | 沿行驶方向的下一个 Lanelet（不是变道关系） |
| conflicting | 冲突 | Lanelet2 routing | 几何相交但无拓扑连接（路口里最常见）⇒ **责任划分的输入** |
| LaneletMap / RoutingGraph / TrafficRules | 地图容器 / 路由图 / 交通规则 | Lanelet2 | 一个路由图只对**一种参与者**（车/行人/自行车）有效 |
| Route / LaneletPath / LaneletSequence | 路由 / 路径 / 车道序列 | Lanelet2 | Route ⊇ Path（可含变道）⊇ Sequence（不变道的连续段，等价于"一条车道"） |
| Lanelet matching | 车道匹配（车道归属） | Lanelet2 matching | 判"物体在/可能在哪个 lanelet"：确定性（欧氏距离）+ 概率（马氏距离，需协方差） |
| Projector | 投影器 | Lanelet2 | WGS84 ↔ 局部米制（UTM/横轴墨卡托/自有原点） |
| road / laneSection / lane | 道路 / 车道段（横断面）/ 车道 | OpenDRIVE | lane **id 正负**表示左右：0 = 中心车道，正 = 左，负 = 右 |
| junction / connection / laneLink | 路口 / 连接 / 车道连接 | OpenDRIVE | 路口内用"连接道路"表达车道级通路（from 车道 → to 车道） |
| priority (high/low) | 路权优先级 | OpenDRIVE | 仅在**没有**信号灯/标志可推时才写 |
| s–t 坐标 | 参考线坐标系 | OpenDRIVE | s 沿**参考线**弧长，t 横向（左正右负）；与你 Frenet 的 (s, l) 同构 |
| Map proto（base_map 等） | 高精地图 protobuf | Apollo | `Map` 里含 lane/junction/overlap/stop_sign/signal/yield/road/pnc_junction… |
| central_curve / left_boundary / right_boundary | 车道中心线 / 左右边界 | Apollo | 车道几何：中心线是**参考轨迹**，未必是几何正中 |
| LaneSampleAssociation | 车道采样关联 | Apollo | `(s, width)`：中心线采样点到左右边界的距离 ⇒ 车道**逐段宽度** |
| Overlap | 压盖关系 | Apollo | 两个地图元素的关联（车道×车道、车道×路口、车道×人行横道…） |
| PNCJunction / Passage | 规划用路口 / 进口出口 | Apollo | 带 ENTRANCE/EXIT 的语义路口，引用 signal/yield/stop_sign/lane |
| cut-in | 切入 | 业界通用 | 旁车横向侵入我的车道；**必须单独处理**（见 03 RSS 篇） |

---

## ③ 必读材料清单（含阅读顺序）

**阅读顺序**：`1 Lanelet2 首页 → 2 Primitives → 3 Architecture → 4 Matching → 5 Autoware 扩展格式 →
6 OpenDRIVE 车道 → 7 OpenDRIVE 路口 → 8 Apollo 地图模块 → 9 查阅`

| # | 标题（链接） | 为什么读 | 读多少 | 难度 |
|---|---|---|---|---|
| 1 | [Lanelet2 官方文档首页](https://fzi-forschungszentrum-informatik.github.io/Lanelet2/) | 建立模块地图：core / matching / routing / projection / traffic_rules 各自职责；`lanelet2_matching` 的定位就是"判断物体在哪个 lanelet" | **必读**，首页 5 分钟 | ★☆☆ |
| 2 | [Lanelet2 · Lanelet Primitives](https://fzi-forschungszentrum-informatik.github.io/Lanelet2/lanelet2_core/LaneletPrimitives/) | **数据模型本体**：Point/LineString/Polygon/Lanelet/Area/RegulatoryElement；Lanelet 是原子段、左右边界必须同向、centerline 必须在边界内、相邻 Lanelet 必须共享端点 | **必读**，完整读（约 25 分钟） | ★★☆ |
| 3 | [Lanelet2 · Architecture](https://fzi-forschungszentrum-informatik.github.io/Lanelet2/lanelet2_core/Architecture/) | 容器与查询：LaneletMap / TrafficRules / RoutingGraph / Route·Path·Sequence / Projector；**以及一个真实的坑**：centerline 是缓存的，边界改了它不会自动失效 | **必读**，Architecture 全篇（20 分钟） | ★★☆ |
| 4 | [Lanelet2 · Matching](https://fzi-forschungszentrum-informatik.github.io/Lanelet2/lanelet2_matching/) | 你最需要的一页：**确定性匹配**（欧氏距离阈值）与**概率匹配**（对象位姿 × 协方差的马氏距离）+ 用交通规则剔除"逆行"这类非法匹配 | **必读**，全页 + 示例代码（20 分钟） | ★★☆ |
| 5 | [Autoware · lanelet2 format extension](https://raw.githubusercontent.com/autowarefoundation/autoware_lanelet2_extension/refs/heads/main/autoware_lanelet2_extension/docs/lanelet2_format_extension.md) | 生产工程怎么在 Lanelet2 上加字段（`turn_direction` / `speed_limit` / `no_drivable_lane`）；**尤其读 "Centerline" 一节**：显式 centerline 会让 Frenet 横距变成"到 centerline 的距离"⇒ 与你幽灵刹车同源的陷阱 | **必读**，Centerline + Mandatory Tags（20 分钟） | ★★☆ |
| 6 | [Autoware · extra regulatory elements](https://raw.githubusercontent.com/autowarefoundation/autoware_lanelet2_extension/refs/heads/main/autoware_lanelet2_extension/docs/extra_regulatory_elements.md) | `right_of_way` + **yield lanes**：明确"conflicting lanes 里哪些**可以忽略目标检测**"⇒ 业界版的"邻车道车不该让我减速" | **必读**，Right Of Way 一节（10 分钟） | ★★☆ |
| 7 | [ASAM OpenDRIVE 标准页](https://www.asam.net/standards/detail/opendrive/) + [规范在线版（v1.9.0）](https://publications.pages.asam.net/standards/ASAM_OpenDRIVE/ASAM_OpenDRIVE_Specification/v1.9.0/specification/index.html) | 另一套主流标准的地图产物（`.xodr`）；先看标准页的"Application Areas / File Formats / Reference Line"三段，再按需查规范 | **选读**（知道有这套即可，20 分钟） | ★★☆ |
| 8 | [OpenDRIVE · 11.1 Introduction to lanes](https://publications.pages.asam.net/standards/ASAM_OpenDRIVE/ASAM_OpenDRIVE_Specification/v1.9.0/specification/11_lanes/11_01_introduction.html) | **车道编号约定**（这是最容易踩的坑）：中心车道 id=0 且无宽度；正 t 向（左）升序、负 t 向（右）降序、连续无缺口；宽度是 `sOffset/a/b/c/d` 多项式 | **必读**，只读这一节（15 分钟） | ★★☆ |
| 9 | [OpenDRIVE · 12.4 Connecting roads](https://publications.pages.asam.net/standards/ASAM_OpenDRIVE/ASAM_OpenDRIVE_Specification/v1.9.0/specification/12_junctions/12_04_connecting_roads.html) | 路口怎么表达**车道级**通路：`<connection connectingRoad incomingRoad contactPoint>` + `<laneLink from to>`；以及 `priority(high/low)` 只在无信号灯时用 | **选读**，12.4 + 12.4.1（20 分钟） | ★★★ |
| 10 | [Apollo 地图模块文档（中文）](https://apollo.baidu.com/docs/apollo/10.x/md_modules_2map_2README__cn.html) | 一份**中文的、逐字段的**高精地图 schema：`Map`/`Lane`/`Junction`/`Overlap`/`PNCJunction`/`Road`；尤其 `Lane.left_sample/right_sample`（`(s,width)`）与 `left/right_neighbor_forward_lane_id`、`self_reverse_lane_id` | **必读**，Lane 与 PNCJunction 两节（30 分钟） | ★★☆ |
| 11 | [Lanelet2 GitHub 仓库](https://github.com/fzi-forschungszentrum-informatik/Lanelet2) | 想看实现/示例代码时用；README 的 Citation 一节给出了论文（Poggenhans et al., ITSC 2018）出处 | **查阅** | ★★☆ |

> 第 5/6 条给的是 `raw.githubusercontent.com` 链接（我核实到的就是 raw 文件本身，HTTP 200）；
> 想用网页阅读就换成 `github.com/<org>/<repo>/blob/main/`，路径不变。

---

## ④ 关键机制 / 公式 / 数据结构

### 4.1 Lanelet2 的数据模型（六原语）

| 原语 | 语义 | 关键约束（原文要点） |
|---|---|---|
| Point | 3D 点 + 标签 | 全图唯一 id；单独的点没有意义（除虚线端点） |
| LineString | 折线（**点间线性插值**） | 必须有 `type`；不自交；不重复点 |
| Polygon | 闭合折线 | 首尾隐式相连 |
| **Lanelet** | 一个**原子**车道段 | 恰好一条 left + 一条 right LineString，且**同向**；可选 centerline（**必须在左右边界内且不接触边界**）；默认单向；相邻 Lanelet 必须**共享端点**；可变道者必须**共享一条边界**；可引用 RegulatoryElement；**限速必须能直接从 Lanelet 推得**；**可用参与者必须能确定** |
| Area | 面状可行驶区（广场、停车场、应急车道） | 有序 LineString 围成，**顺时针**；有向性弱于 Lanelet |
| RegulatoryElement | 规则（限速/红绿灯/**路权**/标志） | 通用容器：`type` + 角色化引用（如 traffic_light + stop_line） |

标签约定：**全小写 + 下划线**；通用键 `type` / `subtype` / `no_issue`。

### 4.2 Lanelet2 的拓扑与查询 API（routing）

关系（`js/algo/plan` 里你缺的正是这一层）：

| 关系 | 含义 | 对应你的什么 |
|---|---|---|
| `left` / `right` | 共享边界的左右邻 ⇒ **可变道** | 你的"邻车道"概念（现在只按 l 距离猜） |
| `adjacent left` / `adjacent right` | 相邻但**不可**变道 | "旁边那条，但不能过去" |
| `succeeding` | 行驶方向的后继 | 你的 `netEdges`（节点对）近似表达了它 |
| `conflicting` | 几何相交、拓扑不连 | **路口让行/责任划分的输入**（你完全没有） |
| `area` | 车道 ↔ 面区域的可达关系 | 无 |

查询 API（C++，Python 同名）：`graph->left(ll)` / `adjacentLeft(ll)` / `right` / `following(ll)` /
`conflicting(ll)`；查询对象还有 `Route::leftRelation(ll)`、`Route::fullLane(ll)`、`Route::remainingLane(ll)`。
跨参与者（车 vs 行人）用 `RoutingGraphContainer` 求 `conflictingInGraph(s)`。

**Route / LaneletPath / LaneletSequence 的区别**（写规划时很实用）：

- **Route**：能用来到达目的地的**所有** lanelet（含可选变道）。
- **Path**：一条**有序**序列（可含变道）。
- **Sequence**：不含变道的连续段 —— **它就是"一条车道"**。你现在用"road 中心线 + laneCenters"
  表达的东西，在 Lanelet2 里天然是 Sequence。

### 4.3 车道归属（matching）——本仓库最该补的一节

Lanelet2 的 matching 模块明确提供两种口径：

1. **确定性匹配**：找出与物体**欧氏距离 < 阈值**（示例用 4m）的所有 lanelet。
   因为 lanelet 有方向，所以每个 lanelet 会被算两次（正向 + 反向，距离相同）。
2. **概率匹配**：把**物体位姿**（含**协方差**）与 lanelet 做比较，取**马氏距离平方**；
   因为含朝向，正反两次距离**不同**。

再加一步**规则过滤**：`removeNonRuleCompliantMatches(matches, trafficRules)` —— 用交通规则剔除
"逆行车道""人行道"这类不可能/不合法的匹配（这一层本质就是**责任与资格**，见 03 篇）。

对你的直接启示：

- 归属判断应该吃**物体**（中心 + 朝向 + 尺度 + 不确定性），而不是吃**单条射线的命中点**。
- 你现在的 `js/algo/objects.ts` 有 `{s, l, length, width, rays, sensor}` 和 `vS/vL`，
  **没有协方差** ⇒ 你只能做"确定性匹配"的工程版；要升级成概率匹配，先给跟踪器加协方差（见 05 MOT 篇）。
- 你现在的判据 `|hitL − laneCenter| ≤ laneWidth/2 + 1.4px`（`js/algo/plan/inlane.ts`）
  是"点在带内"；业界是"**框 vs 车道多边形**"，并且带**迟滞**（你的 `js/algo/plan/tight.ts` 已有迟滞思想 ✓）。

### 4.4 投影口径（你已经修对了，值得对照确认）

你的 `js/algo/plan/frenet.ts:cartesianToFrenet` 现在是**逐线段投影 + 插值法向**：

```
t   = clamp((p−a)·(b−a)/|b−a|², 0, 1)
q   = a + t·(b−a)
s   = s_i + t·|b−a|
n(t)= normalize(lerp(n_i, n_{i+1}, t))
l   = (p − q) · n(t)
```

要点（也是业界一致的口径）：

- 参考线是**连续**的（OpenDRIVE 用 straight/spiral/arc/poly3 参数化；Lanelet2 的 LineString 是**线性插值**
  折线 —— 所以**点距越密、投影越准**，这是它和 OpenDRIVE 的取舍之一）。
- 投影必须与反变换**严格互逆**（你的 `frenetToCartesian` 用同一套插值法向 ⇒ 往返误差只剩
  "线段对曲线的线性化"，mm 级；守卫测试：`js/test/plan/frenet-project.test.ts` 三条断言）。
- **近场不要用 Frenet**：`js/algo/plan/bodyFrame.ts` 的车辆本体坐标系（x 前 / y 左）定义即精确，
  不依赖地图与投影；它只用来**加约束**（近场窄带内的命中一律算障碍），**永不解除**已判定的阻塞。
- 经纬度 ↔ 局部米：全走 `js/shared/geo.ts`（`latLonToLocalMeters` / `localMetersToLatLon` /
  `scaleFactors`，含 1/cos φ 的纬度因子）；地图里 `geometry.origin` 是**投影原点**，
  `geometry.originWorld` 才是**世界 (0,0) 的经纬度** —— 底图/瓦片/任何换算都用后者。

### 4.5 OpenDRIVE：车道编号与路口（最容易踩的符号约定）

- 车道挂在**道路参考线**（reference line）上；`<laneSection s="0.0">` 按 s 切横断面。
- **lane id**：中心车道 = `0`（**无宽度**，只放最内侧标线）；**正 id 在左（+t 方向）、负 id 在右（−t 方向）**；
  编号从中心向外连续、无缺口。宽度是 s 的多项式
  `<width sOffset="0.0" a="4.0" b="0.0" c="0.0" d="0.0"/>` ⇒ 逐段变宽是**天然**表达
  （你的 `taperedHalfWidthAt` 是"30m 线性过渡"的简化版 ✓）。
- 路口：`<connection connectingRoad="28" incomingRoad="4" contactPoint="start">` +
  `<laneLink from="-3" to="1"/>` ⇒ **车道级**通路；连接道路是 OpenDRIVE 中**唯一允许路面重叠**的道路。
- 路权：`<priority high="1" low="4"/>`，**只在没有信号灯/标志可推时才写**。

### 4.6 Apollo：逐字段可直接抄的 schema

`Map` 顶层：`header`（含 `Projection`，PROJ.4 `+proj=tmerc +lat_0=… +lon_0=…`）、`crosswalk`、
`junction`、`lane`、`stop_sign`、`signal`、`yield`、`overlap`、`clear_area`、`speed_bump`、`road`、
`parking_space`、`pnc_junction`、`rsu`。

`Lane` 里与你最相关的字段：

| 字段 | 含义 | 对应你现在的 |
|---|---|---|
| `central_curve` | 车道中心线（**参考轨迹**，不保证几何正中） | `centerPoints`（但你是**道路**中心线，不是每条车道） |
| `left_boundary` / `right_boundary` | 左右边界（各自是 `Curve` + 边界类型 + `virtual` 标记） | 无（你只有整条路的 `outerBoundary`） |
| `speed_limit` | 车道限速（**m/s**） | `AttrSpan.maxspeedKph`（**km/h**，你另存 `speedLimitMps` 时注意是 px/s！） |
| `length` | 车道长度 | `attrSpans` 的 `s0/s1` |
| `predecessor_id` / `successor_id` | 前后继车道 | `netEdges` + `buildNetworkRoute` |
| `left/right_neighbor_forward_lane_id` | **同向**左右邻 | 无 ⇒ 你的"邻车道"只能按 l 猜 |
| `left/right_neighbor_reverse_lane_id` | **反向**左右邻 | 无 |
| `junction_id` | 属于哪个路口 | 无 |
| `self_reverse_lane_id` | 双向车道被拆成正反两条，互为自相反 | 无（你用 `oneway`） |
| `left_sample` / `right_sample` | `LaneSampleAssociation{s, width}` ⇒ **逐段宽度** | `taperedHalfWidthAt(spans,…)` 同目的 ✓ |

`PNCJunction` → `PassageGroup` → `Passage{signal_id, yield_id, stop_sign_id, lane_id, type: ENTRANCE/EXIT}`
⇒ **"给规划用的路口语义"**，比几何路口更好用；这是你未来做"路口让行"最值得抄的结构。

---

## ⑤ 与本仓库对照表（我们有什么 / 缺什么）

| 业界概念 | 本仓库对应（相对路径） | 我们还缺什么 |
|---|---|---|
| Lanelet（车道级图元） | 无。最接近的是 `TrackGeometry`（`centerPoints` / `halfWidth` / `laneCenters` / `attrSpans`）：[`../../js/shared/types/interfaces/geometry.ts`](../../js/shared/types/interfaces/geometry.ts) | **车道多边形/左右边界线**、**车道 id**、**方向语义**（左/右/对向） |
| Lanelet 左/右边界 + centerline | `centerPoints` 是**道路**中心线；`buildFrenetReference`：[`../../js/algo/plan/frenet.ts`](../../js/algo/plan/frenet.ts) | 逐车道中心线（Lanelet2 的 `Lanelet.centerline()` / Apollo 的 `central_curve`） |
| left / right / adjacent / succeeding / conflicting | 只有节点级可达性：`netNodes` / `netEdges`：[`../../js/env/track.ts`](../../js/env/track.ts) | **车道级**关系（尤其 `left/right` 与 `conflicting`） |
| Matching（确定性/概率 + 规则过滤） | `inLaneGapAhead`（"点在带内"）：[`../../js/algo/plan/inlane.ts`](../../js/algo/plan/inlane.ts)；近场交叉校验：[`../../js/algo/plan/bodyFrame.ts`](../../js/algo/plan/bodyFrame.ts) | **物体框 vs 车道带**、**置信度/迟滞**、**协方差**（`js/algo/objects.ts` 目前无协方差） |
| RegulatoryElement（限速/路权/信号） | 只有限速：`AttrSpan.maxspeedKph` + `attrsAtS` + `speedLimitMps`：[`../../js/env/track.ts`](../../js/env/track.ts) | **路权/红绿灯/停车标志**这类"规则"元素（你的收费站是写死的 `tollgates` 桩号） |
| OpenDRIVE lane id 正负 | `laneCentersOf(halfW, laneWidth)`（**按 l 排序**，故意不引入左右符号）：[`../../js/env/track.ts`](../../js/env/track.ts) | 一旦要表达"左侧车道/对向车道"，就需要**显式 id 约定** |
| OpenDRIVE `<width>` 多项式 | `taperedHalfWidthAt`（30m 过渡段）：[`../../js/env/track.ts`](../../js/env/track.ts) | 多项式精度（你用台阶+线性，够用） |
| junction connecting roads / laneLink | `netNodes/netEdges` + `buildNetworkRoute`（Dijkstra）：[`../../js/algo/routePlanner.ts`](../../js/algo/routePlanner.ts) | **车道级**连接（左转/直行各连哪条车道）、路口内优先级 |
| Apollo PNCJunction / Passage | 无 | 路口让行的语义输入（未来做"无保护左转"必备） |
| Projector / 经纬度↔米 | `latLonToLocalMeters` / `scaleFactors`：[`../../js/shared/geo.ts`](../../js/shared/geo.ts)；OSM 侧投影：[`../../js/mapgen/osm.ts`](../../js/mapgen/osm.ts)（`projectOntoPath`） | 基本对齐 ✓（注意 `origin` 与 `originWorld` 别混） |

---

## ⑥ 可落地的最小练习

### 练习 A（推荐，3~4 小时）：把"车道归属"从"点在带内"升级成"框 vs 车道带"

**目标**：让"邻车道的车"**不再**影响我这条车道的巡航目标 —— 你实测的幽灵刹车就是这个。

1. 在 [`../../js/env/track.ts`](../../js/env/track.ts) 加 `laneBandsOf(halfWidth, laneWidth)`：
   返回每条车道的 `{ id, lMin, lMax, center }`（从最左到最右编号，**只用于归属，不引入左右驾驶语义**）。
2. 在 [`../../js/algo/plan/inlane.ts`](../../js/algo/plan/inlane.ts) 加一个**吃物体**的版本：
   `inLaneGapAheadByObjects(tracks, egoS, egoL, bands)` —— 用物体的 `[l − width/2, l + width/2]`
   与车道带求交，取**本车道内**最近的前方物体。
3. 单测 `js/test/plan/lane-ownership.test.ts`（新建）：
   - 邻车道车（中心 `l = 3.75m`、宽 `2m`）⇒ 它的框 `[2.75, 4.75]` 与我这条车道带 `[-1.875, 1.875]`
     **不相交** ⇒ `inLaneGapAheadByObjects` 返回 `null`；
   - 本车道车（中心 `l = 0`、宽 `2m`）⇒ 返回它的间距（**安全不回归**）；
   - 跨线车（中心 `l = 1.5m`）⇒ 框与两条带都相交 ⇒ 必须被判定为"在我车道内"（保守 ✓）。

**可量化验收**：

- `npx tsc && node --test dist/js/test/plan/lane-ownership.test.js` 全绿；
- `npm run sim -- --scenario scenarios/bj-g4-dujiakan-traffic.json` 与当前基线**同种子**对比：
  用你自己的硬减速判据（0.5s 内掉 >1.5m/s）统计采样点，**不高于**基线（预期下降）；
- 全量 `npm test` 不回归（尤其 `js/test/plan/lead.test.ts`、`js/test/perception/rig.test.ts`）。

### 练习 B（选做，2~3 小时）：给地图产物加"车道多边形"

在 `js/mapgen` 的地图产物里加 `road.lanePolygons`（每条车道的左右边界点列，机械生成：
沿中心线按 `laneCenters` 偏移 ±`laneWidth/2`），并写 `pointInLane(p, polygon)` + 3 条单测
（车道内 / 车道外 / 骑线）。

**验收**：`maps/bj-g4-dujiakan.json` 重新生成后**逐字节稳定**（你的 mapgen 有"重新生成结果一致"的约定）；
`pointInLane` 单测全绿；`npm test` 不回归。

---

## ⑦ 常见坑与误解

1. **把 Lanelet 当"路段"** ✗：它是一条**原子**车道段（沿它规则不变）。一个路口前会有很多个
   （每个变道机会/规则变化点都切一刀）。在 Lanelet2 里"一条车道"是 **LaneletSequence**。
2. **只用 centerline 就想算横向距离** ✗：横向距离取决于**你选的那条参考线**。
   Autoware 文档明确记录了这点：显式 centerline 会让"某车到我车道的横向距离"
   变成"到**centerline** 的距离"，因此他们才额外引入 `overwriteLaneletsCenterlineWithWaypoints`。
   这正是你幽灵刹车的同类陷阱（参考线一偏，归属就错）。
3. **参考线用"最近顶点 + 顶点法向"投影** ✗：你的实测是 68m 处 ~1.2m 横向误差。
   正确口径是**逐线段投影 + 该处插值法向**（已修，守卫测试在
   [`../../js/test/plan/frenet-project.test.ts`](../../js/test/plan/frenet-project.test.ts)）。
4. **近场还用 Frenet** ✗：车头正前方 15m 内的"它挡不挡我"用本体坐标系（`bodyFrame.ts`）定义即精确；
   Frenet 只适合**道路级/远场**判断。
5. **把 OpenDRIVE 的 lane id 正负当成"左右邻"用** ✗：OpenDRIVE 的负号是**右**（−t），
   Apollo/Lanelet2 用**显式字段**表达左右邻。三套约定的符号方向不同，混用必错。
6. **Lanelet2 的 centerline 缓存** ✗：改了左右边界，Lanelet 返回的 centerline **不会自动更新**
   （LaneletMap 的最近点树同样不会自动重建）。
7. **概率匹配需要协方差** ✗：没有协方差只剩确定性匹配。你的 `js/algo/objects.ts` 现在没协方差。
8. **经纬度与字段覆盖率的两个陷阱**：Web Mercator 的"米"不是米（纬度 φ 处 1/cos φ，北京 ≈1.30）——
   别把瓦片像素当米，世界坐标换算一律用 `geometry.originWorld`（不是 `geometry.origin`）；
   另外 OSM 上 `maxspeed` 几乎为空、匝道几乎没有 `lanes` ⇒ 地图里的限速/车道数多为**派生值**
   （你的 `AttrSource = 'osm' | 'derived:class'` 就是在标注这件事，别把它读成真值）。
10. **"有车道级地图"不等于"归属就对了"** ✗：地图只给你**几何/拓扑**；"这辆车算不算我的障碍"
    还要**责任划分**（03 篇）与**不确定性**（05 篇）。

---

## 附录：链接核实说明（重要）

本文所有链接在写入时都经过 **web_fetch 直接核实**（HTTP 200，且返回内容与标题对应）：

- Lanelet2 文档站：首页、`lanelet2_core/LaneletPrimitives/`、`lanelet2_core/Architecture/`、
  `lanelet2_matching/`、`lanelet2_routing/`（均已取到正文，含上文引用的原句）；仓库
  `github.com/fzi-forschungszentrum-informatik/Lanelet2` 的 raw README（200）。
- Autoware：`autoware_lanelet2_extension` 的 `README.md`、`docs/lanelet2_format_extension.md`、
  `docs/extra_regulatory_elements.md`（都是 `raw.githubusercontent.com`，200）。
- ASAM OpenDRIVE：标准页 `asam.net/standards/detail/opendrive/`、规范在线版索引（v1.9.0）、
  `11_lanes/11_01_introduction.html`、`12_junctions/12_04_connecting_roads.html`（均 200）。
- Apollo：`apollo.baidu.com/docs/apollo/10.x/md_modules_2map_2README__cn.html`（200，中文全文）。

**核实不通过 / 未能核实的链接，一律没有写进本文**：

- `lanelet2.readthedocs.io/en/latest/` —— 实际返回 **404**（该 Read the Docs 项目不存在）。
  正确入口是上面的 `fzi-forschungszentrum-informatik.github.io/Lanelet2/`。
- Lanelet2 论文 PDF（`mrt.kit.edu/z/publ/download/2018/Poggenhans2018Lanelet2.pdf`）——
  出现在（已核实的）仓库 README 的 Citation 里，但我没有单独核实该 PDF 可达 ⇒ **不直接给链接**，
  请从仓库 README 的 Citation 一节进入。
- `www.opendrive.org`（旧站）—— 未核实，已用 ASAM 官方页替代。

若你日后发现某条链接失效，请以**标题**重新检索，不要相信任何"看起来像"的 URL。
