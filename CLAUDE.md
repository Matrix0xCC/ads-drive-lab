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

# 底图瓦片（构建期；清单入库、瓦片入 gitignore 缓存；缺失时渲染优雅降级为"无底图"）
npm run tiles -- --map bj-g4-dujiakan                 # 默认 z=17、缓存到 maps/tiles/
npm run tiles -- --map bj-3rd-ring --zoom 17 --buffer 120
npm run tiles -- --map bj-g4-dujiakan --dry-run       # 只报瓦片数与体积，不联网

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
- `js/shared/` — 接口契约（types/interfaces/，含归档记录 schema）、量纲系统（dimensions.ts）、rng、debug、**地理换算**（`geo.ts`：经纬度 ↔ 局部米制、Web Mercator 瓦片 ↔ 经纬度 —— 地图管线与渲染层共用同一份公式）
- `js/ui/` — UI 层（`state.ts` 纯状态机 + `controller.ts` 副作用边界 + sidebar/toolbar/hud/inspector/interactions/replayPanel 面板模块）；`js/replay.ts` — 回放控制器；`js/env/renderer/` — Three.js 渲染
- `js/mapgen/` — **OSM 地图管线**（构建期工具，不进浏览器包）：`osm.ts` 纯几何（way 链合并 / 投影 / 均匀重采样 / **弧长窗口平滑** / 曲率统计 / `projectOntoPath` 地图匹配原语）+ `network.ts` **路网拓扑**（共享顶点处切段 → 节点/边图；`largestComponent`）+ `attrs.ts` **逐段道路属性**（车道数/限速/半宽 + 回退规则 + `osm`/`derived:class` 来源标注 + 区间压缩与查表）+ `fetch-tiles.ts` 底图瓦片取数（`npm run tiles`）+ `build-map.ts` CLI（同端点重试抓取、原始数据与侧车 meta 入库、`--from-raw` 离线复跑、`--start-at-toll`、`--network`、`--classes` 选道路类别、`--clip-bbox` 从大样本裁小图）；**抓取按模式分派查询**（路网必须用 `buildNetworkQuery`——曾误用链式查询导致"抓不到匝道"，还被误判成镜像数据版本问题）；`--from-raw` 也会把原始数据落到 `maps/raw/`（否则地图里有 `rawSha256` 却找不到对应原始文件）
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
  **`main.ts` 的模块初始化顺序是硬约束**：`const ui = setupUI(...)` 在**顶层调用 `applyScenario` 之后**才初始化，
  所以"模块初始化阶段就会被调用"的函数（`applyScenario`）**不得引用 `ui`/`replay` 之类后声明的绑定**，
  否则刷新即崩 `Cannot access 'x' before initialization`（TDZ；TypeScript 不报，实测踩过两次：
  `basemapAttribution` 与 `ui`）。避免办法是**不要为派生值新增模块级可变状态**（署名改为从底图句柄读；
  底图可见性由每帧 `updateScene` 同步）。由 `js/test/ui/main-init-order.test.ts` 静态守住
  （只查"初始化阶段被调用的函数"——回调里用 `ui` 是安全的，不能一刀切）。

### 规划器（js/algo/plan/）——纵向-横向解耦（spec: docs/superpowers/specs/2026-09-19-decoupled-planner-design.md）

- **架构**：`paths.ts`（SL 路径层）决定"走哪条"，`speed.ts`（ST 速度层）决定"什么速度"，`planner.ts` 以收敛迭代协调两层后按 l(s(t)) 合成 Trajectory（控制/回放/归档契约不变）；`states.ts` 状态机：CRUISE / FOLLOW / AVOID / DECEL / ARRIVED / EMERGENCY
- **每帧流程**：Frenet 参考线（有 route → 开环，无 route → 闭环）→ 车辆投影 → 巡航目标 `vCap = clamp(最近障碍/2, 17.5, 35)` → **路径层 × 速度上限收敛迭代（≤3 轮；收敛阈 0.2m / 0.5m·s⁻¹，变化不降反增则提前停）** → `solveSpeedLayer` → 合成
- **路径层**：`corridor.ts` 建**走廊族**（舒适 / 物理 / 更严 / 更松，同一几何只差膨胀量），候选由**物理走廊瓶颈处的自由段**派生（间隙中心 + 边界内缩 + 车道中心，≤7 个）；两段式横向（前 40m 五次多项式横移、其后保持），**自适应缩短发展段**（越界点前完成横移，下限按 `S_min=√(5.77·|Δl|·v²/A_LAT_MAX)` 随速度缩放）+ **横向路径夹紧 [l0, target]**（防贴边棘轮复位）；120m 视程（有路线时截到路线剩余，`sEnd`）、越出走廊 = 阻塞
- **双阈值（走廊版）**：舒适走廊 = 车半宽 + `max(1.5m, 标定 p95)`（只影响排名与慢行档）；物理走廊 = 车半宽 + `marginFor(v)`（标定速度表，决定停车线）；更严/更松 = 物理 ±0.15m（迟滞状态机 `tight.ts` 的进入/退出判据，进入用更松、退出用更严 + 3 帧防抖）。横向排名只看 COMMIT_WINDOW=60m（晚承诺）；全部候选连物理走廊都过不去 → 停车线；存在物理可过候选则限速 8m/s 慢行；**可达性闸门**：绑定点取"种子档几何阻塞点 / 末轮最优阻塞点 / 全部候选最早的舒适阻塞点"三者最早（最后一项兜住"域放宽后大角度绕行候选变舒适畅通、判据消失"），判据用**车辆自身减速能力** `car.acceleration`（不是规划侧保守的 A_BRAKE：能不能做到问能力）——到绑定点前降不到速度前提则停车线、停不住则 emergency
- **收敛迭代（stage C）**：`vAssume` 从 `max(当前速度, 慢行档)` 起，每轮用 `margin(vAssume)`/`S(vAssume)` 求路径，再由路径几何反推速度上限 `u`（封死 0 / 紧间隙慢行档 / 横向机动 `lateralSpeedCap` / `vCap`）；几何评估速度下限 = 慢行档（破"停车 → 裕度最大 → 封死 → 永不起步"自锁）
- **曲率限速（A/B 开关 `CURVATURE_SPEED_LIMIT`，默认关）**：开则按道路曲率限速 `v ≤ √(A_LAT_MAX/κ(s))` 并叠加前视制动可行性（`v² ≤ v_lim² + 2·a_brake·Δs`，取全程最小）。**默认关的原因**：它改变所有地图的速度剖面（径向图半径 80~100m → 限速 17~19m/s），属语义变更需单独 A/B；OSM 高速图（最急 R=141m）关闭时会以 vCap 上限 35m/s 跑完含弯全程。headless 用 `--curvature-limit` 打开
- **道路半宽随地图**：`solvePathLayer` 收 `trackHalfWidth`（规划域 `L_MAX` 随地图半宽收窄）；感知的 |l| 分级阈值改为**半宽比例**（0.6/0.8，径向图 0.6×7.5=4.5m / 0.8×7.5=6.0m 与历史写死值一致 → 行为零变化）
- **逐段域与逐段限速（"地图起作用"的第二层）**：`solvePathLayer` 另收 `halfWidthAt(s)` → 走廊**逐站**取域（走廊本来就是逐站求自由补集，接入点天然）；`deriveTargets` 的夹紧用**承诺窗内最小域**（避免进入窄段时目标全被否掉）。
  - **不变量 1：规划域必须包含车辆当前位置**（`domain = clamp(max(名义域, |l0|+5cm), 名义域, 复位崖)`）。否则"候选路径起点就在走廊外 → 全判碰撞 → 永久停车"：实测三环场景卡死在单车道匝道入口（域 ±0.375m 而 l0=0.41m，最近邻投影与点到线段两种口径差几厘米）。
  - **不变量 2：域上限 = 复位崖，绝不更宽**（域比崖宽会让规划器把车贴到崖外 → 复位传送；实测放宽到"物理上限"时 oval-s9 立刻多出 1 次复位）。
  - **复位缓冲按路段宽度缩放**（`car.ts:boundsBufferFor` = min(0.5m, 40%×车中心物理余量)）：固定 0.5m 在 1.875m 半宽的匝道上吃掉近 60% 的中心行程 → 域比投影误差还窄。宽路仍是 0.5m（径向零回归）。
  - **逐段限速 = 制动可行剖面**：`v_allowed(s0) = min over 前方 s' 的 √(v_lim(s')² + 2·a_brake·(s'−s0))`（与曲率限速同一套道理），采样 = 10m 网格 **+ 每段边界**（限速是台阶下降，必须采到下降沿）。**量纲**：属性是 km/h，必须 `mps(kph/3.6)` 转 px/s——写成 m/s 会让 `min(133.3, 33.3)` 把巡航目标砍成 8.3m/s（实测车全程慢行档）。
  - 实测效果：G6 立交路线（匝道 60km/h → 主线 120km/h）到达时间 110.7s → **163.1s**；三环（主线 100 → 匝道 50）**124.1s**、最高速 27.7m/s；G4（两段同为 120）**与基线逐位一致**；径向批量逐项零回归。
- **地图语义进决策（"地图起作用了吗"的可测判据：几何相同、语义不同 → 行为必须不同）**：
  - **地图限速 → vCap**：`track.speedLimitMps`（OSM `maxspeed`）作为巡航速度**硬上限**，取在 17.5m/s 防呆下限**之后**（低限速图 60km/h=16.7m/s 不会被下限抬高）。实测：G4 图最高速 34.9 → **33.27m/s**（=120km/h）
  - **车道语义 → 车道中心**：`laneCentersOf(halfWidth, laneWidth)` 推出车道数与车道中心，名义车道取最靠近路中心者（径向 4 车道 → +1.875m 与历史硬编码**逐位一致**；OSM 3 车道 → 中间车道 `l=0`）。实测：G4 图车的 |l| p50 由 **1.87m → 0.02m**（旧硬编码值在 3 车道路面上是跨车道线的位置）
  - 两者都在 `debug.planning` 暴露（`mapSpeedLimitMps` / `laneOffsetU` / `laneCentersU`）并在调试面板显示
- **速度层**：ST 条目（tMin=0 静态）→ cruise / stop（五次多项式停车线，T 取最小单调解）/ emergency（A_MAX 兜底）
- `polynomial.ts` 五次多项式（曲率 C² 连续）；`collision.ts` 锥形搜索（供旧耦合路径的遗留工具）
- **路线声明**：链式/径向地图用弧长 `route: { startS, goalS }`；**路网地图用节点** `route: { fromNode, toNode }`（见"地图来源与开环/路网道路"）
- **指标**：`summary.json` / `report.md` 含到达用时、路线进度、**收敛轮数 >1 的帧占比、单帧规划耗时 p50/p95**（`planMs` 为墙钟量，**不进逐帧归档**——否则破坏"同种子逐字节一致"）
- 路线终点自动减速（DECEL_ZONE=25m）+ 到达检测（剩余 <10px → ARRIVED）

### 感知（js/algo/perception.ts）

LiDAR 单线 2D：±60° FOV、2° 分辨率、61 条射线、最大 125m。核心是 **HD Map 背景减除**，判据按优先级：
① **命中线段类型**——射线本来就分别对"道路边界线段"与"障碍物矩形"求交，**命中的是哪一类自己就知道**：命中边界 → 一定是背景（confidence 0）。这比事后用 |l| 比值反推可靠得多：|l| 受最近邻投影误差影响，在宽度过渡段的斜边上会把真实路缘判成障碍（实测车停在匝道口）。
② **障碍物命中**才用 |l| 比值细化（<0.6×**该路段**半宽 高置信 / 0.6~0.8× 边缘模糊 0.5 / >0.8× 贴路缘模糊 0.5）。
阈值与边界几何**同源**：都用 `taperedHalfWidthAt(该路段属性)`（见"逐段道路属性"），不能用地图级标量。
输出 `distances`（规划用，已过滤）/ `rawDistances`（渲染用，真实命中）/ `confidences`。

**路网地图的两处特例（"道路即路线"）**：① **射线求交的边界线段**取自 `ribbonPolygon(路线)`——路网的 `outerBoundary` 是空的，照抄会让 60/60 条射线飞到 125m 上限（界面上是一圈穿透路缘的扇形，且护栏不再遮挡）；**只取路线那一条 ribbon**，不能取全部链（路口处别的链的端帽会横在车前方、|l| 很小 → 幻影障碍）。② **背景减除参考线**同样用路线（用"最长链"算 |l| 会让路线路缘变成高置信障碍 → phantom_hit 实测 870 帧）。两者都按对象身份缓存；边界线段另带包围盒做每帧距离粗筛（等价变换，不动结果）。检测器的幻影检查只认 `confidence=1`（0.5 是"可能只是路缘"的模糊带，算进来会把路缘报成幻影）。

### 控制（js/algo/control.ts）

Pure Pursuit：δ = atan2(2·L·sin α, ld)，预瞄距离 = clamp(0.4×speed, 10m, 20m)。油门目标 = 轨迹终点速度；**终点速度 ≈0（停车/紧急剖面）→ 油门 0**（0.15 怠速下限只对正值目标保留——否则"停车"变成 6.375m/s 恒速爬行，批量 A/B 回归根因）。

### 真实地图底图（瓦片；纯表现层）

- **定位**：只给人看，**不参与感知/规划/控制、不进归档**。开关在 toolbar「底图」/ `B` 键（`ui/state.ts` 的 `basemapVisible`，默认关）。
- **产物分工**：`maps/<name>.tiles.json` **入库**（几百字节、可 diff：来源/署名/范围/每张 sha256/`worldOrigin`）；
  `maps/tiles/<provider>/<z>/<x>/<y>.png` **gitignore 缓存**。理由：瓦片是别人的数据 + 外部世界快照
  （许可上不该再分发进仓库、二进制进 git 会永久增重、且本项目所在机器连不上官方站），
  而"清单入库"仍能回答"我们用的是哪一批"。新克隆没有缓存是**常态** → 必须优雅降级。
- **取数**：`npm run tiles -- --map <name>`（可重复执行：已缓存的瓦片按 sha 复用，只补缺的；
  带可识别 UA + 每张间隔 250~500ms 限速）。**默认 z=17**：仿真 4px/m、相机视距 250m，z=17（0.92~1.19 m/px）
  够用；z=18 张数 ×4 而仿真路面自己画了车道线，收益低。
- **地理配准（两个字段别搞混）**：
  - `geometry.origin` = **投影原点**的 `[lat, lon]`（mapgen 内部用的那个）；
  - `geometry.originWorld` = **世界坐标 (0,0) 的 `[lat, lon]`**。地图点列做过"平移到 [margin,...]"的世界归一化
    → 世界 (0,0) ≠ 投影原点（实测差几百米~几公里）。**底图/瓦片/任何"世界坐标 ↔ 经纬度"换算都要用 `originWorld`。**
  - 两个字段都是 `[lat, lon]`、7 位小数（≈1cm）。**曾经**路网地图把"米"写进 `origin` 且恒为 `[0,0]`（同一字段两种含义），
    精度也只有 4 位（≈11m，配准会整体偏）——`js/test/mapgen/georef.test.ts` 现在把这类错误钉死。
- **单位（最容易犯的错）**：`planBasemapQuads` 给的是**米**（地理量纲，便于与地图产物/经纬度对照），
  而场景里路面/障碍/雷达都用**仿真像素**（`SIM_UNITS_PER_METER = 4 px/m`）→ 摆放时必须过
  `quadToSceneRect()`（米 ×4）。**漏了就是"跑道很长、底图只有一小块"**（实测踩过；旧测试只查米制地理框，
  所以完全没发现——`basemap.test.ts` 现在补了"场景量纲"这一环）。
- **分层与绘制顺序（消除"地面从瓦片里穿出来"的黑色乱纹）**：瓦片与地面只差 0.05 仿真像素（1.25cm），
  而相机视距 250m~2km，深度缓冲精度**远不足以分辨** → 深色地面（0x111111）成片穿透，表现为地图上的
  **黑色乱纹**（缩放改变深度分布，所以缩放后尤其明显；实测踩过）。修法是**不依赖深度精度**：
  · 地面 `renderOrder = -2`（最先画，并把 y 放到 -5 留出层间距）；
  · 瓦片 `renderOrder = -1` + `depthTest = false` + `depthWrite = false` + **不透明**（半透明会被排到
    所有不透明物体之后 → 反过来盖住路面；压暗靠 `color` 乘法）；
  · 路面/网格/障碍/雷达等保持 `renderOrder = 0`，正常覆盖瓦片。
- **摆放与朝向**（`env/renderer/basemap.ts`）：每张瓦片取**四个角**各自投影（局部系是等距的 → 瓦片仍是轴对齐矩形，
  用中心+宽高即可精确表示，不必假设瓦片是正方形）；平面 y = `BASEMAP_Y`（-0.05：网格 0、路面 0.01 之下）。
  **朝向要同时满足三条不变量**（少一条就会"看着像对了"，其实是镜像/翻转 —— 实测踩过：地图文字左右反读）：
  ① **法线朝上**（`rotation.x = -π/2`，配 `side: FrontSide`）→ 从上方看到**正面**；
     若用 `+π/2`，法线朝下 → 看到的是**背面** → **整张图左右镜像**（文字反读）；
  ② **图象上方朝北**（世界 +z）：`-π/2` 会把平面 local +Y 转到世界 **-z**，所以必须**同时**设 `texture.flipY = false`
     （图象首行 → local -Y → 世界 +z）；默认 `flipY = true` 会让图象上方落到南边（南北翻转）；
  ③ **图象右方朝东**（世界 +x）：PlaneGeometry 的 u 沿 local +X，`-π/2` 不动 X ✓。
  三者由 `js/test/renderer/basemap.test.ts` 逐条断言（旧版只查"上方朝北"，**镜像的配置恰好也能过** —— 所以必须查法线方向）。
  纹理还要设 `SRGBColorSpace` + mipmap + anisotropy；换地图必须 `dispose()`（否则 VRAM 泄漏）。
  （渲染层其它平面用 -π/2 是因为它们先做过 `Vector2(x, -y)` 镜像，别混。）
- **量纲**：瓦片是 Web Mercator，其"米"不是米（纬度 φ 处比例因子 1/cos φ，北京 39.9° → **1.30**，
  同样的 z 赤道 2.39 m/px、北京 1.83 m/px）。所有换算走 `js/shared/geo.ts`，
  `geo.test.ts` 专门断言这条 30% 的坑。**别把瓦片像素当米。**
- **世界坐标系必须是右手系（北 = −Z）**——这是底图能"读得出来"的前提，也是整轮排查的结论：
  - 仿真平面 (x=东, y=北) 若直接映射成世界 +z，世界就是 (X=东,Y=上,Z=北) 这套**左手系**；
    Three 按右手系渲染 → 地面上**带手性的内容必然镜像**（真实地图文字反读）。仿真自己不含手性
    （路线/车道线镜像后仍是自己），所以三年没暴露。
  - 实现：**场景根一次镜像**（`env/renderer/scene.ts` 的 `scene.scale.z = WORLD_Z_SIGN = -1`）。
    因此**加进 scene 的一切内容照旧用仿真坐标 (x, y)**——路面/障碍/轨迹/路线/雷达/瓦片/刻度/走廊**一处都不用改**；
    **只有相机与 OrbitControls（在场景之外）要写世界坐标**（`simToWorldZ`），**鼠标拾取**要换回仿真 y（`worldZToSim`）。
  - **相机必须放在车的"南侧"**（`frameCamera`/`frameLocal` 里的 `- 400k` / `- 0.6·span`）→ 画面"北在上"。
    四种组合实测（`js/test/renderer/world-convention.test.ts` 逐条钉住）：旧代码（sign=+1+相机北侧）= 北在下（**上下颠倒**）；
    只改符号 = 东在左（**左右镜像**，实测的中间态）；符号 + 相机南侧 = 北在上、东在右 ✓。
  - 副作用（可接受，须知）：**径向（合成）地图的默认取景沿南北翻转 180°**——它们没有地理含义，
    只是统一按"北在上"取景；另有既有行为不变：`followCar` 仍按用户调整过的相对位姿跟随。
- **冲突处理**：瓦片上画的道路与仿真路面**边缘不会重合**（宽度/描边风格不同 + 平滑带来米级位移）→ 默认把底图
  **压暗**（`color` 乘法 ≈0.55）当上下文，仿真路面仍是视觉主体。
- **世界网格让位**：底图**真的显示**时隐藏 `GridHelper`（网格是"黑底"下的空间参照，真实地图自己有路网/地物
  参照，叠在一起就是地图上一层多余的黑线）；用户开了底图但这张地图没有瓦片时**保留**网格。
  判据抽成纯函数 `scene.ts:gridVisibleWhen(showBasemap, hasBasemap)` 并由 `world-convention.test.ts` 钉住。
- **署名**：清单里的 `attribution` 必须**可见**（OSM ODbL / 各提供方要求）——实现为画布左下 HUD 的
  `#hud-basemap-attribution`，只在底图打开且有署名时显示（`main.ts:renderBasemapAttribution`）。
- **查底图必须用"地图产物 key"**（`MAP_PRESETS` 的键 / 场景里的 `scenario.map`），**不是 `MapDefinition.name`**——
  后者对合成地图是显示名（`Oval`、`Tech Circuit`），实测造成 `GET /maps/Oval.tiles.json 404`；
  OSM 地图恰好 key==name，所以只在切回径向图时才暴露。`fetchTileManifest` 会先做 key 形状检查
  （不像 key 就跳过并打日志，不发注定 404 的请求）。
- **径向（合成）地图不查底图**：它们没有经纬度（`geometry.originWorld` 都不存在），瓦片无意义 →
  `main.applyScenario` 里 `kind !== 'radial'` 才调用 `applyBasemap`。

### 地图来源与开环/路网道路（`env/mapDefinitions.ts` + `env/track.ts`）

- `MapDefinition.kind`：`'radial'`（历史：8 个径向控制点闭环，半宽 7.5m、世界 450×200m）、`'polyline'`（OSM：点列，可开环，半宽按车道数、世界按包围盒）、`'network'`（OSM 路网：多链 + 节点/边图）。地图下拉由 `MAP_PRESETS` **动态填充**（新增地图只需登记 MAP_PRESETS，不必改 index.html）。
- **路网地图（多链）语义**：`track.chains`（全部可行驶链）+ `netNodes`/`netEdges`（路由图）。可行驶域 = **到最近链的距离 + 车半径 ≤ 半宽**（`car.ts` 的 `isInsideNetwork`；路口处链的并集天然覆盖，端部自动成墙）——不能用多边形表达。`centerPoints` 取最长链，仅作"无路线时的回退参考线"。渲染为每条链一条 ribbon 路面。**越界判定的提前退出条件只能写"已确定在内"**（写成 `best <= halfW` 会让先遍历到的邻近链把结果定成越界——实测车精确在链上却复位 3400+ 帧）。
- **入库的两张真 OSM 路网**：`bj-g6-junction`（京藏 G6/S11 立交，**树状**：24 边/25 节点，路线唯一）、`bj-3rd-ring`（西三环/六里桥，**有环**：109 边/105 节点/38 路口）。mapgen 的 `routeSample` 会**主动挑一对"能演示选择"的起终点**并量化代价：逐段掐掉最短路上的边再重路由，取最小绕行距离（树状图恒为 `null`）。`bj-3rd-ring` 实测：最短路 2.93km（含 3 段匝道），绕开其中一段匝道要多走 **1159m**。
- **路面/边界逐段变宽 + 过渡段**：链式地图的 `outerBoundary` 用 `ribbonPolygonVar` 按逐点半宽生成（`outerBoundaryHalfWidth` 与边界点一一对应，供 `checkBounds` 取"最近边界点处的局部半宽"算复位缓冲）；路网的感知边界用 `ribbonPolygonVar(路线, 逐点半宽)`。
  **宽度台阶必须变成过渡段**（`WIDTH_TAPER_M = 30m`）：属性区间是台阶函数，直接做成几何会在路面上生成一道**横向的墙**，其命中点投影到台阶之前（那里半宽还是宽路的）→ 被判成障碍 → 车停在收窄处。`taperedHalfWidthAt` 让宽度"提前 30m 开始线性过渡"，并且**路面几何、感知分类阈值、规划域三处共用同一个函数**——三处口径必须一致，否则同一份数据在不同模块里含义不同。
- **逐段道路属性（"这段路几车道/限速多少"）**：地图级标量（`halfWidth`/`maxspeedKph`）之上多一层逐段真值。
  路网地图：`network.chainAttrs[i]`（与 `chains` 对齐）；链式地图：`road.spans[{s0M,s1M,...}]`（中心线是多条 way 合并的，way 边界就是属性可能变化处，G4 实测有 3 车道与 2 车道两段）。
  运行时有 `TrackGeometry.chainAttrs` / `attrSpans`，路线装配时生成 `Route.attrSpans`；消费方按 s 查 `attrsAtS`。
  **量纲**：`halfWidthM`/`laneWidthM` 是**米**（渲染宽度、限速用），弧长是 **px**——gen 模块里写成 `m(...)` 会把米转成 px（实测域变 4 倍）。
  **回退规则必须标注来源**：`lanes` 有 tag 用它，否则 `*_link`→1、`motorway/trunk`→3、其余→2；`maxspeed` 有 tag 用它，否则按等级派生（motorway 120 / trunk 100 / `*_link` = 主线×0.5 夹到 [40,60]）。
  **真实覆盖率（2026-09-27 实测）**：`lanes` —— motorway/G4 100%、trunk 34%、**匝道 ≈0%**；`maxspeed` —— **几乎为 0**（941 条 way 里 1 条）→ 地图里的限速基本是派生值，`validation.attrCoverage` 会给出真实/派生计数（此前文档把默认 120km/h 说成"OSM 真实限速"是错的）。
- **路网路线**：场景用 `route: { fromNode, toNode }`；`buildGraphFromEdges` + `buildNetworkRoute`（Dijkstra 出节点序列 → 逐段取链几何并按方向拼接）→ 规划器/感知照常把它当开环参考线用；`waypointIndices` 记途经链序号（归档可查"走了哪几条匝道"）。浏览器 `T`/toolbar 设路线时**吸附到节点**（`pickPoints()`），与 headless 走同一个 `buildNetworkRoute`。检测器的越界判据对路网用**链并集**（`world.track.chains` 进归档），否则路口处会误报。
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
- **LiDAR 射线池**（`env/renderer/rayPool.ts`，61 条 Line 预创建 + 每帧原地改顶点）：
  **必须 `frustumCulled = false`**——Three.js 的包围球只在第一次用到时算一次，而池子是用**全 0 顶点**创建的
  → 包围球退化成"世界原点的一个点"且永不重算 → 开着剔除时射线随相机平移/缩放**时而出现时而消失**
  （径向图整图取景时原点常在视野里，所以旧版不易察觉；OSM 长条/路网图相机跟着车跑出几公里后必现）。
  单独成模块是为了让这条约束被 `js/test/renderer/ray-pool.test.ts` 钉住（含"包围球不自动重算"的演示用例）
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
