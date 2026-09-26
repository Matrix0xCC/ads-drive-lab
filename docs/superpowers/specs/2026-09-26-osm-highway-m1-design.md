# M1 设计：OSM 高速地图管线 + 开环几何（路线 A 第一步）

- 日期：2026-09-26
- 状态：**设计中（本文件随 M1 实施同步更新执行记录）**
- 关联：可行性分析见 `docs/superpowers/specs/2026-09-25-evolution-roadmap.md` §7 与本轮会话；
  路线图 E1/E5 不受影响；本阶段**不改规划器任何决策逻辑**

## 1. 目标与非目标

**目标（M1）**：把真实 OSM 高速主线变成一份**几何净化过、可入库、可复现**的地图文件，并让仿真器能吃"开环等宽"道路。

- M1-a（本文件先做）：OSM 拉取 → 合并单向车行道 → 投影到局部米制 → 重采样+平滑 → 曲率校验 → 产出 `maps/<name>.json`（含来源与 ODbL 署名）。
- M1-b：`MapDefinition` / `generateTrack` 支持"点列 + 开环 + 按地图半宽"，世界改为长条，相机跟随。
- M1-c：场景与批量用例（收费口→出口的**任务语义**留到 M2）。

**非目标**：匝道/立交/分合流（路线 C）；车道级参考线；变宽（收费广场）；GPS 接线（M2）；规划器/感知决策逻辑改动。

## 2. 事实依据（本轮实测，勿重新验证）

| 事实 | 数值 |
|---|---|
| Overpass 可用性 | **必须带 User-Agent**（不带 406）；`overpass-api.de` 会因过载返回 `Dispatcher_Client::timeout`，需多端点 failover + 原始数据缓存 |
| 镜像 | `overpass.private.coffee` 可用；`kumi.systems` 超时；Geofabrik `.pbf` 超时（**不走 pbf 路线**） |
| 数据量 | 北京 5km×5km bbox 内 motorway 93 way；慕尼黑 11×15km 内 152 way / 34.6km / JSON 122KB |
| `out geom` 字段 | 含 `geometry`（lat/lon）**但不含 `nodes` id 数组** → 合并 way 只能按**端点坐标**匹配（OSM 共享节点坐标逐位相同） |
| 原始节点间距 | p50 46.6m、p95 191m、max 388m |
| 原始折线"曲率" | p95 1.9e-2 1/m（R≈53m）、max 4.0e-2 → 粗采样差分假象，**必须重采样+平滑** |
| tag 覆盖 | `oneway`/`lanes`/`maxspeed` 均 ~100%；`lanes` 分布 2:53 / 3:73 / 4:18 / 5:4 / 6:3 |
| 国内数据 | 北京 bbox 2976 way、上海 3752 way，够用 |

## 3. 设计

### 3.1 数据流

```
Overpass(bbbox+ref)  ──fetch(多端点+重试)──►  maps/raw/<name>.osm.json   （入库：原始证据 + 离线可复现）
                                              │
                          parseOverpass ──► chainWays（按端点坐标合并/定向）
                                              │
                          selectCarriageway（oneway + ref + 最长）
                                              │
                          projectToEnu（等距圆柱，原点=首点）
                                              │
                          resampleAndSmooth（Catmull-Rom 过控制点，按步长采样）
                                              │
                          curvatureStats（±窗口三点外接圆） + 速度上限交叉校验(maxspeed)
                                              │
                                              ▼
                                        maps/<name>.json    （入库：派生数据 + 来源声明 + 校验统计）
```

**为什么原始数据也要入库**：① 确定性——同种子逐字节一致的地基是"输入不变"；② 不重复压 Overpass；
③ ODbL 要求可追溯来源。运行时（浏览器/headless）**绝不联网**。

### 3.2 地图文件 schema（v1）

```jsonc
{
  "version": 1,
  "name": "bj-g4",
  "kind": "osm-chain",          // 点列来源：osm-chain（未来 radial / width-profile）
  "closed": false,              // 开环
  "source": {
    "provider": "OpenStreetMap", "license": "ODbL-1.0",
    "attribution": "© OpenStreetMap contributors",
    "via": "https://overpass-api.de/api/interpreter",
    "query": "<原样保存的 Overpass QL>",
    "fetchedAt": "2026-09-26T…Z",
    "ref": "G4", "wayIds": [123, 456]
  },
  "geometry": { "stepM": 3, "origin": [lat, lon], "points": [[x, y], …] },  // 米，局部 ENU
  "road": { "laneCount": 3, "laneWidthM": 3.75, "halfWidthM": 5.625, "maxspeedKph": 120,
            "oneway": true },
  "validation": {
    "lengthM": 1800.0,
    "kappa": { "p50": …, "p95": …, "max": … },     // 1/m（±5m 窗口，抗差分噪声）
    "minRadiusM": …,
    "aLatAtMaxspeed": …,                            // v²κ，与 A_LAT_MAX 对比
    "maxspeedFeasibleMps": …,                       // √(A_LAT_MAX/κ_max)
    "warnings": ["…"]
  }
}
```

**数值定点**：所有坐标四舍五入到 4 位小数（0.1mm），键序固定 → 重新生成得到**逐字节相同**的文件。

### 3.3 关键算法选择与依据

1. **way 合并**：端点坐标按 1e-7°（≈1cm）取整做键。单向车行道（`oneway=yes`）在 OSM 里天然是**链**，
   同一条高速的上下行是两条独立的链 → 取 `ref` 匹配且**最长**的一条 = 一个行驶方向。
   `ref` 允许包含匹配（`G4;G107` 也命中 `G4`）。输出按 way id 排序保证确定性。
2. **投影**：等距圆柱（equirectangular），`x=(lon−lon0)·cos(lat0)·R`，`y=(lat−lat0)·R`，R=6371000。
   1.5km 尺度下相对误差 <1e-3，可忽略；**不用 UTM**（避免引入投影库与带号逻辑）。
   注：`κ` 是坐标的二阶量，尺度误差不影响 κ 的**形状**（均匀缩放不改变曲率半径比例）。
3. **平滑**：把 OSM 节点当**控制多边形**，用 Catmull-Rom（与 `env/track.ts` 同一套）插值后按 `stepM=3m`
   采样。理由：原始点距 46m，直接线性重采样会保留棱角（每个节点变成一个角），而规划器的
   `developMin` / `lateralSpeedCap` / `marginFor` 全读参考线曲率 → 棱角会变成假的"处处不可行"。
4. **曲率口径**：±5m 窗口三点外接圆（与 `margin.ts:refCurvatureAt` 同款算法、更大窗口），
   避免 3m 采样的有限差分噪声。同时输出 `p50/p95/max` 而不是单点，便于判断"是否有真急弯"。

### 3.4 验收判据（M1-a）

- `npm test` 全绿（新增 `js/test/mapgen/osm.test.ts`，**全部离线**，用合成 fixture，不依赖网络）。
- 真实地图：长度 ≥ 1.5km；`κ` 的 p95 ≤ 0.002 1/m（R≥500m，符合高速线形）；`max` 记录并可解释；
  重新运行生成器 → `maps/<name>.json` **逐字节不变**。
- `maps/raw/<name>.osm.json` 与 `maps/<name>.json` 均入库，含 ODbL 署名。

## 4. 本阶段假定（用户未逐项拍板，按推荐值推进；可随时否决）

| 决策点 | 本次取值 | 理由 |
|---|---|---|
| 车道建模 | **单中心线**（沿用现状） | 车道级要重做走廊/规划，非 M1 范围 |
| 世界尺度 | **长条整图**（`SIM_W` 加长到 ~1.5km，相机改跟随） | 改动小；滑动窗口留到路线 C |
| 收费口复杂度 | **单亭停车**（M2 做） | 多亭排队是新的决策问题，量级翻倍 |
| GPS 参与 | **GPS + 地图匹配**（M2 做） | 直接吃噪声读数无法与真值 A/B |
| ODbL | 派生数据 + 原始数据入库 + 来源声明（不上传公共库） | 满足署名/可追溯；个人项目不触发额外义务 |

## 5. 任务拆分

- [x] M1-a.1 `js/mapgen/osm.ts` 纯函数（parse / chain / select / project / resample / 弧长平滑 / 曲率）
- [x] M1-a.2 `js/test/mapgen/osm.test.ts` 离线单测 22 个（含确定性、"平滑掉假棱角"、"真弯不被削平"）
- [x] M1-a.3 `js/mapgen/build-map.ts` CLI（多端点 failover + 原始数据缓存 + `--from-raw` + `--start-at-toll`）
- [x] M1-a.4 生成真实地图（京港澳 G4 杜家坎段 2km）→ `maps/bj-g4-dujiakan.json` + 原始数据 + 侧车 meta；
      `js/test/mapgen/map-file.test.ts` 4 个产物契约测试（sha256 一致性 / 步长 / 线形 / 收费站桩号）
- [ ] M1-b.1 `MapDefinition` 支持 `points/closed/halfWidth`；`generateTrack` 开环 + 端帽
- [ ] M1-b.2 世界长条 + 相机跟随；`TRACK_HALF_WIDTH` 按地图；感知阈值改局部半宽比例
- [ ] M1-b.3 `buildRoadGraph`/`sToIndex`/`buildFrenetReference` 的开环贯通（3 处 `closedLoop` 硬编码）
- [ ] M1-b.4 **曲率限速**（见执行记录发现 4）：`v ≤ √(A_LAT_MAX/κ_ref(s))`
- [ ] M1-b.5 场景 + 批量用例 + 文档
- [ ] M2 收费口停车点原语 + GPS 地图匹配

## 6. 风险

1. **Overpass 不稳定**（本轮已实测两种失败模式）→ 多端点 + 重试 + 原始数据入库 + `--from-raw`。
2. **开环端帽**：`isCarInsideTrack` 依赖"内外边界闭合多边形"，开环要补端帽，否则 `pointInPolygon` 结果不可靠。
3. **半宽参数化遗漏**：`TRACK_HALF_WIDTH` 被感知阈值(4.5/6.0m)、`L_MAX`、场景校验、渲染刻度共用——
   漏一处就会出现"感知把广场边缘当边界删掉"这类**系统性错误**（P6 同类），M1-b 要逐处清点。
4. **曲率限速缺失**：见执行记录发现 4——高速地图**没有它就跑不出合理速度**。

## 7. 执行记录（M1-a，2026-09-26）

### 7.1 产物

| 文件 | 说明 |
|---|---|
| `js/mapgen/osm.ts` | 纯几何：`parseOverpass` / `chainWays` / `selectCarriageway` / `projectToEnu` / `resampleUniform` / `smoothByArcLength` / `resampleSmooth` / `curvatureStats` / `projectOntoPath`（M2 的地图匹配原语）/ `sliceByArcLength` |
| `js/mapgen/build-map.ts` | CLI：多端点 failover、原始数据 + 侧车 meta 入库、`--from-raw` 离线复跑、`--start-at-toll` |
| `maps/bj-g4-dujiakan.json` | 派生地图：**京港澳 G4 杜家坎收费站起 2.0km**，668 点 @3m |
| `maps/raw/bj-g4-dujiakan.osm.json` | 原始 Overpass 响应（62KB，含 27 way / 11.8km 的 G4 链，裁剪到 2km） |
| `maps/raw/bj-g4-dujiakan.meta.json` | 侧车：端点/时间/bbox/查询语句/sha256（**不进地图文件**，见发现 2） |
| `js/test/mapgen/*.test.ts` | 26 个离线测试（22 纯函数 + 4 产物契约）；全库 **283 测试全绿** |

实测数据：链 = 27 way / 11786m（ref=G4）→ 从**杜家坎收费站**（链上 s=1672m）起取 2000m；
3 车道 × 3.75m → 半宽 5.625m；OSM maxspeed 120km/h；
κ p50/p95/max = 3.4e-7 / 3.36e-3 / 7.08e-3（p95 半径 ≈297m，max 半径 141m）。

### 7.2 四个实测发现（都改变了设计）

**发现 1：OSM 单节点位置抖动会造出"假急弯"，弧长窗口平滑是必需的。**
探针段实测 κ max = 3.2e-2（R=31m）——高速上不可能。定位到航向序列
`19.2 19.2 19.8 21.1 23.2 → 27.5 → 33.9 → 30.2 → 23.7 20.6`：真实曲线航向应**单调**变化，
这里是单个节点鼓出的 ±7° 包。**Catmull-Rom 解决不了**（它是插值样条，过该节点，且在尖控制多边形上会过冲），
故管线改为 `线性等距重采样 → 按弧长窗口（30m）三角核平滑 → 再等距重采样`：
同一段 κ max 3.2e-2 → **4.45e-3（R=225m）**，p95 半径回到 588m。
窗口取真实最小半径的 ~10%（单测钉住：R=300m 圆弧平滑后 κ 仍在 ±10% 内）。

**发现 2：地图文件必须只含"由数据本身可推出"的字段，否则"重新生成逐字节相同"不成立。**
第一版把 `via`（命中的 Overpass 端点）、`query`、`rawFile`（含输出目录）写进地图 → 联网抓取与
离线复跑产出不同字节。改为：取数过程信息进 `maps/raw/<name>.meta.json`，地图只留
`rawSha256` + `osmBaseTimestamp` + `wayIds` + 许可署名。**已验证：不同输出目录下离线复跑逐字节一致。**

**发现 3：收费站是"成对节点 + 横向铺开的广场"，不是中心线上的一个点。**
杜家坎收费站 4 个节点两两相距 11~50m（广场两端），投影到中心线后 |l| 达 **18~36m ≫ 半宽 5.6m**。
→ 桩号（s）可用，但横向几何超出当前"恒定半宽"模型：M2 要么把收费口简化为"中心线上的强制停车点"，
要么给地图加**宽度剖面**（`widthProfile`）——这正是 §1 非目标里"变宽"的提前暴露。

**发现 4：规划器没有"基于道路曲率"的限速，高速地图没有它跑不出合理速度。**
现有 `lateralSpeedCap`（`js/algo/plan/paths.ts:49`）管的是**变道机动**的横向加速度，不是**路弯**。
杜家坎段最急 R=141m：按 OSM 的 120km/h 需横向加速度 7.9m/s² ≫ A_LAT_MAX=3.5，可行上限 80km/h。
本项目车辆是运动学自行车（**无侧向附着极限**），车"能"以 120km/h 拐过去——约束只存在于规划器，
因此必须显式加 `v ≤ √(A_LAT_MAX/κ_ref(s))`（M1-b.4），否则高速速度剖面在弯道上不合理，
也失去与真实 maxspeed 对照的意义。

### 7.3 尚未验证

- `--toll-span`（两站夹一段）在杜家坎未触发：该段只有**一个**收费广场（4 节点同属一处），
  用 `--start-at-toll --length` 更贴合；跨站段需要更长的地图（相邻收费站在数十 km 外）。
- ~~世界尺度与开环几何（M1-b）未动~~ → 见 7.4。

---

## 8. M1-b 执行记录（2026-09-26 同日）

### 8.1 已完成

| 项 | 内容 |
|---|---|
| 地图定义 | `MapDefinition` 增 `kind / points / closed / halfWidth / worldW / worldH / laneWidthM / maxspeedKph / tollgates`；`buildTrack()` 统一入口（radial 闭环 / polyline 开环） |
| 开环语义 | `innerBoundary` 空 + `outerBoundary` = **闭合路面多边形**（左缘→端帽→右缘→起点帽，两端沿切线**外延 25m**）；`closed` 贯通 `buildRoadGraph` / `sToIndex` / `resetToTrack` / `generateObstacles` / 感知参考线 / 渲染刻度与车道线 / 检测器 / 归档 `world.json.track` / 回放 |
| 半宽参数化 | 规划器 → `solvePathLayer({ trackHalfWidth })`（`L_MAX` 随地图收窄）；感知 |l| 阈值改 **0.6/0.8×半宽**（径向图 0.6×7.5=4.5 / 0.8×7.5=6.0 与历史写死值一致 → 行为零变化）；渲染车道分隔线按半宽推车道数 |
| 世界尺寸 | `engine.worldW/worldH` 取地图声明；`setWorldExtent` 随地图重建地面/网格；相机 far→20000、maxDistance→15000 |
| 相机跟随 | `followCar`（世界长边 >800px 时每帧把相机与 target 同步平移，保留用户旋转/缩放）；`frameSmart` = 小世界整图 / 大世界取车周围 250m（R 键与 toolbar 按钮共用）→ **径向图取景行为一字不变** |
| 曲率限速 | `curvatureSpeedCap`（`v ≤ √(A_LAT_MAX/κ(s))` + 前视制动可行性 `v² ≤ v_lim²+2·a_brake·Δs`）；A/B 开关 `CURVATURE_SPEED_LIMIT`（**默认关**）+ headless `--curvature-limit` |
| 用例 | `scenarios/bj-g4-dujiakan-cruise.json`（起点=杜家坎收费口，终点 1900m）+ 登记 `scenarios/index.json` |

### 8.2 三个"只有真跑才暴露"的 bug（均已修 + 复验）

1. **端帽压在出生点上** → `checkBounds` 用"到任一边界线段距离 < r"判越界，而端帽横跨路面、正好穿过路线起点 →
   车一出生就判越界，**7200/7200 帧 off_track**、进度 0%。修：端帽沿切线外延 25m。
2. **生成模块把"米"当仿真单位**（`px(v)` 应为 `m(v)`）→ 2km 的高速段被压缩成 500m；
   表现为 `route_timeout` 且"剩余 **500.0m**"（该数字正是线索）。
3. **端帽被感知当成高置信障碍** → 它横跨路面、|l| 很小，落在"高置信障碍"档 → 检测器报 `phantom_hit` 42 帧。
   修：开环地图下"命中点 s ≥ 参考线终点 − 2m ⇒ 已知边界"（与侧墙同等对待）。
   **复验：`bj-g4-dujiakan-cruise` 现在判定 PASS、0 issue、到达 55.2s、0 复位。**

### 8.3 曲率限速 A/B（同种子 10 场景，单变量）

| 场景 | 关（基线） | 开 | 差 |
|---|---|---|---|
| oval-s6 | 13.2s | 17.7s | **+4.5s** |
| oval-s7 | 16.5s | 17.4s | +0.9s |
| oval-s8 | 68% | 68% | 0 |
| oval-s9 | 18.7s | 20.2s | +1.5s |
| oval-s10 | 15.3s | 16.7s | +1.4s |
| hairpin-s6 | 13.7s | 16.5s | +2.8s |
| hairpin-s7 | 13.7s | 15.3s | +1.6s |
| hairpin-s8 | 72% | **76%** | +4pp |
| hairpin-s9 | 13.2s | 14.9s | +1.7s |
| hairpin-s10 | 13.5s | 15.6s | +2.1s |

- **碰撞 0 段/0 帧不变、到达仍 8/10**；用时 **+0.9~4.5s（均值 ≈+1.9s）**；hairpin-s8 进度略升。
- OSM 场景：关 55.2s / **开 58.0s**（最急 R=141m 处确实减速），0 复位。最高速仍是 34.9m/s（在直道上，合理）。
- **结论**：这是**语义变更**（旧行为下 90m 半径环道以 35m/s 行驶 = 横向加速度 13.6m/s²，不物理），
  而不是纯优化——按仓库纪律应**单独提交 + 重取引擎黄金基线**。M1 里保持**默认关**，等用户决定。

### 8.4 待办

- **浏览器目检**（用户）：长条世界取景 + 相机跟随 + 2km 地图上的走廊可视化。
- **M2**：收费口的**强制停车点**（`Route` 增途经停车点 + planner 的"停稳→保持→放行"转移 + 检测项），
  以及 **GPS + 地图匹配**接入规划器（`mapgen/osm.ts` 的 `projectOntoPath` 已是现成的匹配原语）。


---

## 9. 5a + 5b 执行记录（2026-09-26，路网拓扑与真路由）

### 9.1 5a：路网地图（`--network`）

| 产物 | 说明 |
|---|---|
| `js/mapgen/network.ts` | 拓扑：**共享外观顶点处切段**（匝道在 OSM 里接在主线**中间顶点**上，只看 way 端点会得到 0 个路口——实测）→ 节点/边图；`largestComponent` 丢孤岛；`topologyStats` |
| `js/algo/routePlanner.ts` | `buildGraphFromEdges`（显式边表 → `RoadGraph`，支持单向/路口）、`buildNetworkRoute`（节点序列 → 链几何拼接） |
| `maps/bj-g6-junction.json` + raw + meta + gen 模块 | 京藏 G6/S11 立交：**25 节点 / 24 边 / 8 个路口 / 10 死端，5.27km（主线 2.4 + 匝道 2.8）**；路线抽样 12 条边 3.50km、其中 **5 段匝道** |
| `js/test/mapgen/network.test.ts` | 8 个离线测试（含"**路由真的经过匝道**"的验收判据与产物契约） |

**三个实测 bug（都是"只有真跑才暴露"）**

1. **`projectToEnu` 缺省以"传入点列的首点"为原点** → `buildTopology` 对每条 way 各调一次，所有首顶点都落到 (0,0) →
   整个路网塌成一个 **33 度的星形**（33 条边全连到节点 0）。修：**全路网共用一个原点**（单测钉住）。
2. **Overpass 的两种"静默坏数据"**：① 超时返回**部分结果 + `remark`**（HTTP 200、JSON 合法，只检查 `elements`
   会静默接受截断数据——本应 33 条 way 的立交 bbox 只回了 16 条主线、匝道全丢）；② **429 限流返回 HTML 错误页**，
   而原来"多端点 failover"会换到**数据版本不同的镜像**上拿回一份"完整但更旧"的数据。
   修：检测 `remark`、显式看 HTTP 状态、**默认同端点重试（不静默换端点）**、`--endpoint` 显式指定、
   客户端超时 > 服务端 timeout。
3. **抽样起终点没记方向**：成功的是反向路由时仍写 i→j，导致归档里的 routeSample 复算不可达。修：记录真正走通的方向。

### 9.2 5b：运行时多链

- `TrackGeometry` 增 `chains` / `netNodes` / `netEdges`；`buildTrack` 路网分支（`centerPoints` = 最长链仅作回退）。
- **可行驶域 = 到最近链的距离 + 车半径 ≤ 半宽**（`car.ts:isInsideNetwork`）；复位 = 挪到最近链的最近点。
- 渲染：每条链一条 ribbon 路面 + 中心虚线 + 车道分隔线。
- 场景：路网用 `route: { fromNode, toNode }`；障碍沿**路线**布置；`waypointIndices` 记途经链序号。
- 浏览器：路线拾取**吸附到节点**，与 headless 共用 `buildNetworkRoute`。
- 检测器：路网越界判据改用**链并集**（`world.track.chains` 进归档）；非路网地图的越界阈值也从全局
  `TRACK_HALF_WIDTH` 改为**该地图的半宽**（G4 图 5.625m，此前误用 7.5m）。

**第四个实测 bug（本轮最贵的一个）**：`isInsideNetwork` 的**提前退出条件写错**——写成"任一链段进入半宽就下结论"
（`if (best <= halfW) return best + r <= halfW`），而正确判据是"到**最近链**的距离"。24 条链时先遍历到的
邻近链会把结果定成越界：**车精确在链上（距离 0.00m）、21m/s、直线段，却连续复位 3426 次**（占全程一半）。
改成"只有确定在内才提前返回"后 **复位归零**。教训：**"存在一个近的"与"最近的足够近"是两件事**——并集判据里
必须取全局最小值再比较。

### 9.3 验收

| 场景 | 结果 |
|---|---|
| `bj-g6-junction-route`（路网，3.5km，8 路口） | 到达 **110.6s**、进度 **99.9%**、复位 **0**、碰撞 **0**；余 close_call 2 段 / missed_obstacle 39 帧 / emergency_state 6 帧 |
| `bj-g4-dujiakan-cruise`（开环链） | 判定 **PASS**（0 issue）、55.2s、0 复位 |
| 径向批量（种子 6-10，10 场景） | **逐项零回归**：8/10 到达、0 段/0 帧碰撞、0 复位、用时与基线相同 |
| 全量测试 | **305 全绿**（288 + 9 地图语义 + 8 路网） |

### 9.4 尚未做（信息性）

- 本 bbox 的图是**树**（边 24 < 节点 25 ⇒ 无环）：路线唯一，"选择"体现在走哪条匝道，而**不是**替代路径。
  要演示"绕行选择"（边 ≥ 节点）需要取**两条高速之间的立交** bbox，或把 `trunk/primary` 一并抓进来让匝道落地成环。
- 度-2 节点未做收缩（edge contraction）：图里保留中间节点，Dijkstra/渲染都正确，只是折线段更多。
- 未做：车道级路网、变宽（收费广场/加减速车道）、匝道限速差异化（当前全图统一限速/车道数）。
