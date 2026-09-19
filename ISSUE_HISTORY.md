# 问题记录 + 架构决策

> 2026-09-18 由 `ISSUE_HISTORY.md`（问题记录 + 架构决策）与 `issue_history.md`（问题历史与解决方案）合并整理。
> 按主题分为三部分：算法实现问题 → 架构决策与重构 → 工程基础设施。
> 条目记录于 2026-07-26 开发阶段，其中的参数值为当时取值，后续迭代可能已调整，最新参数以代码为准。

## 已修复：LiDAR 碰撞检测角度归一化缺陷（2026-09-19 headless 批量测试发现/修复）

**状态**：已修复（TDD，含回归测试）
**影响**：安全关键——避障评估在约 80% 的行驶朝向区间内失明，浏览器与 headless 同样受影响（缺陷随初始提交引入，非新增）

### 修复记录（2026-09-19）

1. `collision.ts` 角度归一化（`diff %= 2π` 后再折叠）——错误射线选择消除
2. 源头归一化：`car.ts stepPhysics` 出口 + `perception.ts sense` 射线角统一到 [-π,π]（中点法均值在归一化前计算，避免跨 π 断裂）
3. 回归测试 `js/test/plan/collision-angle.test.ts`（car.angle=190°/-200° 环绕用例，修复前 2 例失败）
4. 附带修复：`evaluateTrajectory` 增加 margin 参数（与快速剪枝统一为 SAFETY_MARGIN+collisionRadius）且返回显式 `{cost, collided}`（替代 `cost>=1e6` 推断，后者会被负 progress 拉回阈值以下丢失标记）；测试 `js/test/plan/candidate-safety.test.ts`

**A/B 验证（同 10 种子批量重跑）**：最小障碍距离普遍增大（如 0.42→1.47m、0.06→1.94m）；紧急制动 19→4 次；碰撞帧 465→337；但碰撞仍未清零（见下一条新发现）。

### 现象

10 个随机障碍场景（`scenarios/batch/`，oval/hairpin × 5 种子）**全部发生真实碰撞**（车中心到障碍矩形最小距离 0.00m，即车体实际相交）；其中 24.5% 的规划帧所选"最优"轨迹在正确检测下为碰撞轨迹。

### 证据链

1. 归档回放（oval-s5 step 360）：障碍在正前方 8.1m（LiDAR 183° 方向 conf=1 明确命中），但最优候选 L=1.9m@35m/s 的 cost=-105（未被判定碰撞），车辆随后擦撞并越界复位
2. 插桩 `isLidarCollision`：轨迹点方位角 -176.6°，最近射线搜索却选中 idx 59（240.6°，视场边缘）——锥形搜索看向空白区域，必然漏检
3. 条件复现：扫描角数组存**未归一化的绝对角**（`car.angle + offset`，而 `car.angle` 单调累积，实测范围 -175°..273°），`ptAngle` 来自 `atan2` 恒在 [-π,π]；当 `|angles[i] - ptAngle| > 2π` 时，`2π - diff` 为负数，**负值恒为最小值** → 错误射线胜出。约 80.2% 规划帧处于失效窗口（|car.angle| 原始值越大越严重，正负两侧均会发生）

### 根因

`js/algo/plan/collision.ts` 中 `isLidarCollision` 的角度归一化：

```typescript
if (diff > Math.PI) diff = 2 * Math.PI - diff;   // diff ∈ (2π, ∞) 时结果为负
```

未处理 `diff > 2π` 的情况（角度未归一化导致差值可超过一整圈）。

### 修复方向

1. **归一化修正**：`diff %= 2π` 后再折叠到 [0, π]（正负两侧都成立）
2. **源头治理**：`car.angle` 与 `scan.angles` 在产生处归一化到 [-π, π]（`env/car.ts` stepPhysics / `algo/perception.ts` sense 出口），消除所有下游对"角度已归一化"的隐式假设
3. **回归测试**：现有测试全部在 `car.angle = 0` 下运行，未覆盖角度环绕——新增 `car.angle ≈ 190°` 的合成场景，断言碰撞检测在正确方向命中
4. **验证方式**：修复后重跑 `scenarios/batch/` 同种子 10 场景，对比 issues 增减（确定性归档支持逐帧 A/B）

### 附带发现（同批测试）

- ~~`candidate.ts` 的 `evaluateTrajectory` 传 `SAFETY_MARGIN`（2.5m），而 `planner.ts` 快速剪枝传 `SAFETY_MARGIN + collisionRadius`（4.96m）——两处安全余量不一致~~（已修复：统一为参数传入）
- ~~候选 collision 标记用 `cost >= 1e6` 判定：碰撞 penalty 会被负的 progress 奖励拉回 1e6 以下 → 标记丢失，极端情况下含碰撞轨迹可能当选 best（EMERGENCY 判定随之失真）~~（已修复：改为显式 `collided` 布尔量）
- 避险代价函数无速度维度：`W_PROGRESS` 奖励高速，接近障碍时速度反升（实测 17.5→23 m/s 冲向障碍）——建议后续评估"随障碍距离限速"策略

---

## 已修复：横向候选超出可行驶域 → 复位传送引发碰撞（2026-09-19 批量复测发现/修复）

**状态**：已修复（TDD，含回归测试）
**影响**：安全关键——规划器约 20% 的决策把车带向赛道边界之外，触发 checkBounds 复位传送后常在障碍物正前方"复活"，直接引发或加剧碰撞

### 修复记录（2026-09-19 第二轮）

1. **候选 clamp 到可行驶域**（`planner.ts`）：域边界与 `checkBounds` 复位阈值一致（半宽 − 车半宽 − 1.5m 复位缓冲 − 0.25m 跟踪余量 = 4.75m），clamp 后去重（5 个横向候选 → 典型 3 个）
2. **全碰撞降级策略**（`planner.ts`）：域内不可避让（如高速近距障碍）时，不再选"代价最低的碰撞轨迹"（其速度偏好 35m/s），而是**优先最低速度**（最小化碰撞能量），同速比代价
3. 回归测试：`js/test/plan/candidate-corridor.test.ts`（域约束 + 可避让/不可避让语义）；`planner-safety.test.ts` 中"15m 障碍不穿过"旧断言更新——旧行为靠借道界外"绕开"，在域内物理上不可避让，新语义为"降级最低速 + 不出域"

**A/B 验证（同 10 种子第三轮）**：越界决策帧 321→**0**；bounds_reset 7→**2**；复位引发的碰撞链消除。**但碰撞帧 337→405 反升**——clamp 移除了"非法但有效"的宽幅借道，剩余碰撞变为域内紧间隙擦碰（见遗留问题）。

### 现象与证据（修复角度缺陷后的 10 场景复测）

- **19.9%（321/1610）的规划帧最优 targetL 超出赛道半宽（|L|>7.5m）**：横向候选集 `laneOffset(1.875m) ± {0, 7, 14}m` = {1.9, -5.1, **8.9**, **-12.1**, **15.9**}，其中三个在赛道外；代价函数只有软性的中心偏离项（W_CENTER），**没有任何可行驶域硬约束**
- 典型链条（oval-s2 新运行）：车在 l≈4.9m 正常绕行 → 最优候选 L=8.9m 继续把车带向墙外 → `checkBounds` 复位传送（step 332→334 从 l=4.9m 瞬移到 l=0.0m）→ 恰好落在正在绕行的障碍物（l∈[-3.1,3.1]，宽 6.2m）正前方 → 直行穿撞
- 复位发生 5/10 运行；11% 的碰撞 episode 由复位 8 步内直接引发；其余表现为避障被反复打断
- 设计矛盾：HD Map 背景减除让**墙体对规划器不可见**（避免把墙当障碍物是对的），但规划器同时也没有拿到"地图的可行驶域"作约束——真实系统（Apollo 等）中 HD Map 的 drivable area 是规划的硬边界

### 遗留问题（第四轮复测后，按证据排序）

1. ~~紧间隙通过无速度适应~~ **已实施**（v4）：`vCap = clamp(最近障碍距离/2, 17.5, 35) m/s`——70m 外不限、50m→25、35m 内→17.5。显式场景（oval-avoid-1 同条件）碰撞帧 38→19、最小距离 1.06→1.57m；域内几何无解时（0 安全候选）仍有擦碰（如 oval-s5 v4）
2. ~~检测余量过保守~~ **认识修正**：`margin` 在 `lidarDist + margin < ptDist` 中是"容差带"（越大越宽松），调小会更严格——它不是过保守主因，保持现状（本条目保留以防再次误判）
3. **贴边障碍分类盲区（新，下一轮候选）**：现实化后障碍横向偏置可达 ±3m，命中面 |l| 可达 4.75m，与感知边界分类阈值（4.5m/6.0m）重叠 → 部分命中被判 conf=0.5/0（边界）过滤，规划器看不见。证据：hairpin-s3 v4 一次 episode **6/6 候选全标"安全"仍擦碰**（17.5m/s），另一次 9/9"安全"且速度 0（停靠/抵住障碍）
4. **跟踪误差放大器**：全批次 RMSE ~0.8-1.0m，紧间隙擦碰主要由跟踪偏差吃掉标称几何余量——控制层（Pure Pursuit 预瞄参数/纵向跟踪）值得单独评估
5. ~~场景生成器现实性~~ **已实施**（v4）：尺寸宽 2-3.5m × 长 3-5m、沿赛道切线朝向 ±0.3rad、横向偏置 ±3m（`obstacle.ts` + 回归测试）

### 第三轮（v4）A/B 数据（2026-09-19）

- 显式场景同条件对照（oval-avoid-1）：碰撞帧 38→19、最小障碍距离 1.06→1.57m
- 批量（随机布局已变，仅方向性参考）：碰撞 episode 18→15、复位 2→1
- 行为基线：引擎黄金测试更新两次（限速与生成器各一次，数值人工核验）

---

---

## 一、算法实现问题

### 1. 转向量纲错误 → 轨迹向外侧偏移

**状态**：已修复

**现象**：车辆移动时实车轨迹始终偏向弯道外侧，无法沿中心线行驶。

**根因**：`car.ts` 物理模型中 `car.angle += steerAngle * dt`，把前轮转角（弧度）当成横摆角速度（弧度/秒），量纲错误。实际横摆角速度应为 `ω = v/L × tan(δ)`，差值约 8.6 倍，导致严重转向不足。

**解决**：改用运动学自行车模型公式 `car.angle += (speed / WHEELBASE) * tan(steerAngle) * dt`，并用中点法 `(prevAngle + newAngle) / 2` 更新位置消除积分偏差。此后引入品牌类型系统在编译时杜绝此类错误（见 9）。

### 2. dl=0 导致轨迹初始响应过慢

**状态**：已修复

**现象**：车辆贴近外侧跑道时，规划器生成的轨迹初始部分几乎平行于边界，无法有效转回中心线。

**根因**：`plan()` 将 Frenet 状态的 `dl`（横向导数 dl/ds）始终设为 0。五次多项式 l(s) 起点斜率为零，前 60-80px（预瞄距离）横向偏移仅变化约 8px，Pure Pursuit 看到的几乎是直行线。

**解决**：计算真实 `dl/ds = tan(headingDiff)`，其中 `headingDiff = car.angle - tangentAngle`。同时在 `cartesianToFrenet` 返回最近点索引 `idx` 以获取切线方向。

### 3. dl 量纲错误导致轨迹飞出赛道

**状态**：已修复

**现象**：路径规划完全错误，规划轨迹与跑道交叉。

**根因**：`dl = car.speed * sin(headingDiff)` 输出的是横向速度 dl/dt（px/s），但 `solveQuintic` 的 `dl` 参数是 dl/ds（无量纲斜率）。相差约 120 倍。

**解决**：`dl = Math.tan(headingDiff)`（无量纲）。同时修正 `ds` 为 `speed * cos(headingDiff)`（沿参考线的纵向速度分量）。后续引入 TypeScript 品牌类型系统从根本上杜绝此类量纲错误。

### 4. 接近终点时轨迹不再刷新

**状态**：已修复

**现象**：车辆能看到终点时，路径规划的轨迹停止刷新，表现为退化。

**根因**：2 秒轨迹的规划距离（~200px）远超剩余路线长度（如 50px）。`frenetToCartesian` 在开环模式下将超出的 s 全部 clamp 到终点，导致轨迹点堆积在同一位置，Pure Pursuit 无法提取有效方向。

**解决**：动态轨迹时长 `effectiveDuration = remainingDist / max(speed, 30) × 1.2`，限制在 [0.5, 2.0] 秒内，保证轨迹不超出路线终点。

### 5. 避障时路径左右摇摆

**状态**：已修复

**现象**：车辆遇到障碍物时路径规划左右交替切换，最终直接撞上。

**根因**：规划器每帧独立选择代价最小的轨迹。障碍物左右两侧的代价相近时，微小的车辆状态变化导致交替切换方向。

**解决**：新增帧间平滑机制——切换惩罚 `W_SWITCH`（初版 50，当前代码为 100）+ `prevBestTargetL` 记忆上一帧选择的横向目标。当候选横向目标与上一帧偏差超过阈值且不是中心车道时，在其代价上追加 `W_SWITCH`，只有新方向显著更优才切换。

### 6. 初始速度太快，直接穿过障碍物

**状态**：已修复

**现象**：路线激活时车速 100px/s，前方近距离有障碍物时来不及避让。

**根因**：三方面——(a) 初始速度 100 过高，(b) 规划器 15Hz 反应滞后，(c) 碰撞检测在极近距离（<5px）时跳过。

**解决**：
- 初始速度从 100 → 30 px/s
- 紧急制动：LiDAR 正前方 < 30px 且车速 > 20 时，以 200px/s² 强制减速（当前实现在 `main.ts`：前方 < 5m 且速度 > 3m/s 时减 5m/s²，最低 2m/s）
- 近距离碰撞：跳过阈值从 5px → 2px，近距离锥形从 ±2 → ±4 条射线

### 7. LiDAR 探测距离 < 轨迹规划视野

**状态**：已修复

**现象**：LiDAR 最大 250px，但高速候选轨迹（260px/s × 2s）延伸至 520px。超出部分传感器盲区。

**根因**：`MAX_SENSOR_DIST = 250px` 未覆盖最坏情况轨迹长度。且 `isLidarCollision` 将 LiDAR 的"未命中"值 250 当成"障碍物在 250px 处"来判断碰撞（`lidarDist + margin < ptDist` 对 >250px 的点误判为碰撞）。

**解决**：
- `MAX_SENSOR_DIST` → 600px（覆盖 260×2=520 + 余量；单位制迁移后为 125m）
- 碰撞检测增加前置条件：`lidarDist < maxDist - 1`（只有实际命中才判断）
- `detectObstacle` 阈值从 120px → 300px（MAX × 0.5）

真实自动驾驶中，传感器探测距离必须 ≥ 轨迹规划视野 ≥ 最坏情况制动距离，即 **See → Plan → Stop** 安全层级。

---

## 二、架构决策与重构

### 8. 赛道边界过滤：HD Map 背景减除（感知层 vs 规划层的职责边界）

**状态**：已决策并实现

**问题**：LiDAR 射线同时命中赛道边界（围墙）和真实障碍物。如果感知层不做区分，规划层收到的 `LidarScan` 是混合数据——规划层被迫自己猜测"这个命中是边界还是障碍物"。早期现象：车辆一直处于避障状态，因为两侧跑道边界被 LiDAR 检测为障碍物。

**方案演进**：

1. ~~不扫描赛道边界~~：perception 只扫描障碍物。问题：LiDAR 射线视觉上不终止于边界，看起来像无限延伸。
2. ~~规划层事后过滤~~：在 `collision.ts` 中用 Frenet l 值逆向分类命中点（`hitFrenetL()`）。问题：信息丢失在感知→规划传递环节，过滤逻辑需要在多处重复（`isLidarCollision` + `detectObstacle`），且相对 l 偏移引入边界误判。
3. **最终方案：感知层前置过滤**

```
感知层 (perception.ts)：
  1. LiDAR 全部线段求交（边界 + 障碍物）
  2. 对每个命中点 → 世界坐标 → Frenet 投影 → |l|
  3. |l| 分级（初版阈值 45px；单位制改为米后）：
       |l| < 4.5m        → confidence=1   高置信障碍物（赛道中间 60% 区域）
       4.5m ≤ |l| < 6.0m → confidence=0.5 边缘模糊（障碍物靠近路缘或 Frenet 误差）
       |l| ≥ 6.0m        → confidence=0   已知边界 → 替换为 MAX_SENSOR_DIST
  4. 输出 LidarScan（规划层只见障碍物命中；rawDistances 保留全部命中供渲染）

规划层 (collision.ts)：
  直接比较轨迹点距离与 LiDAR 距离（对低置信命中用更大安全余量 +10px）
```

**为什么正确**：

- HD Map 提供了赛道边界位置（半宽 7.5m），这是已知信息；感知层的职责就是用地图过滤已知结构，只报告"意外"
- 规划层应直接消费过滤后的障碍物数据，不参与信号处理
- 为什么按 |l| 值而不是按距离过滤：障碍物在赛道中间 l≈0，边界在 l≈±7.5m。无论障碍物多远，其 Frenet l 值不会飘到边界 l 值上去；纯距离过滤会误杀远处障碍物（其距离可能接近近侧边界距离）
- 感知层不查障碍物列表做交叉验证——那是上帝视角，真实系统中感知模块做不到；用置信度接受 2D 单帧 LiDAR 的固有模糊性
- 业界对标：Waymo/Apollo 的 map-based background subtraction

**涉及文件**：`js/algo/perception.ts`、`js/algo/plan/collision.ts`、`js/algo/plan/candidate.ts`、`js/algo/plan/planner.ts`

### 9. JavaScript → TypeScript + 品牌类型系统

**状态**：已实现

**动机**：量纲错误频发（angle 被当角速度、px/s 被当无量纲斜率，见问题 1、3），纯 JS 无编译时检查。

**方案**：引入 TypeScript branded types（`shared/dimensions.ts`）：

- `Px`、`Radians`、`RadPerSec`、`PxPerSec`、`DimLess` 等品牌类型
- 工厂函数（`px()`、`rad()` 等）是唯一 unsafe cast 点，集中审计
- 量纲感知的数学运算（`yawRate()`、`sin()`、`normalizeAngle()`）保证输入输出类型
- `FrenetState.dl: DimLess` 在编译时阻止赋值 `PxPerSec`

**效果**：项目从 7 个 .js 文件迁移至 31 个 .ts 文件，编译时类型检查、零运行时开销（品牌类型编译后完全擦除）。

### 10. 单位系统：物理量 → 仿真单位

**状态**：已实现

**背景**：代码中所有长度/速度配置都是硬编码像素值（`px(8)`, `pxps(120)`），修改比例需要到处改值。

**方案**：在 `dimensions.ts` 增加物理单位工厂函数，单一比例尺 `SIM_UNITS_PER_METER = 4`：

```typescript
export const SIM_UNITS_PER_METER = 4;   // 1m = 4 仿真单位
export const m   = (v) => px(v * SIM_UNITS_PER_METER);
export const mps = (v) => pxps(v * SIM_UNITS_PER_METER);
export const mps2= (v) => pxps2(v * SIM_UNITS_PER_METER);
```

所有配置用物理量编写：`width: m(2.0)`, `speed: mps(25)`, `TRACK_HALF_WIDTH: m(7.5)`。修改比例尺只需改 `SIM_UNITS_PER_METER` 一个常量。

**涉及文件**：`dimensions.ts` — 新增 `SIM_UNITS_PER_METER`, `m`, `mps`, `mps2`；`car.ts`, `track.ts`, `perception.ts`, `control.ts`, `planner.ts`, `collision.ts`, `main.ts`, `mapDefinitions.ts`, `geometry.ts` — 所有硬编码 px 值替换为物理单位。

**参数速查（4px/m）**：

| 物理量 | 值 | 仿真单位 | 说明 |
|--------|-----|---------|------|
| 车宽 | 2.0m | 8 | |
| 车长 | 4.5m | 18 | |
| 轴距 | 3.0m | 12 | |
| 赛道半宽 | 7.5m | 30 | ≈ 2 车道 |
| 最大速度 | 42.5m/s (153km/h) | 170 | |
| 巡航速度 | 25m/s (90km/h) | 100 | |
| LiDAR 最大 | 125m | 500 | |
| 安全余量 | 2.5m | 10 | |
| 预瞄 min/max | 10/20m | 40/80 | |
| 减速区间 | 25m | 100 | |
| 横向候选 | 0, ±7, ±14m | 0, ±28, ±56 | |

### 11. 规划器 → 显式状态机

**状态**：已完成

**动机**：`plan()` 中状态逻辑散落在 if/else 分支，难以理解和测试。

**方案**（`algo/plan/states.ts`）：

- 定义 6 个显式状态：`CRUISE` / `FOLLOW` / `AVOID` / `DECEL` / `ARRIVED` / `EMERGENCY`
- `nextState()` 函数根据条件输入（路线、障碍物、终点距离）驱动状态转移
- 每个状态独立配置参考线、速度目标、轨迹时长
- HUD 显示状态中文名（"巡航中"/"避障中"/"减速中"/"已到达"/"紧急"）

### 12. 模块拆分

**状态**：已完成

| 拆分对象 | 拆分前 | 拆分后 | 文件数 |
|---------|--------|--------|--------|
| `renderer.ts` | 446 行 | `scene.ts / geometry.ts / overlays.ts / hud.ts` | 4 |
| `types.ts` | 537 行 | `interfaces/geometry|pipeline|planning + frenet.ts + polynomial.ts` | 6→4 |
| `planner.ts` | 286 行 | `collision.ts / candidate.ts / states.ts / planner.ts` | 4 |
| Frenet/多项式 | 放在 `shared/types/` | 移入 `algo/plan/`（属于算法实现） | - |

分层原则：接口定义在 `shared/types/`，算法实现在 `algo/plan/`，渲染在 `env/renderer/`。

### 13. HUD 从独立计算变为纯展示

**状态**：已实现

**动机**：`findNearestObstacle()` 在 DOM 层做独立的欧氏距离搜索，重复了感知模块的工作，且无 FOV/距离约束导致报告 562px 外的障碍物。

**方案**：HUD 只读取上游输出——障碍物距离来自 LiDAR 实测（前方 ±30° 最近），方向来自规划器决策（`trajectory.type`）。

---

## 三、工程基础设施

### 14. 测试框架

**状态**：已实现

| 之前 | 之后 |
|------|------|
| 自定义 `assert.js` | Node.js 内置 `node:assert` |
| 浏览器 `test.html` 运行 | `npm test` → `tsc && node --test` |
| 4 个 .js 测试文件（789 行） | 4 个 .ts 测试文件、30 个用例（当前） |
| 手动断言计数 | TAP 格式输出，`describe/it` |

### 15. Three.js 加载方式

**状态**：已实现

| 之前 | 之后 |
|------|------|
| CDN import map (`unpkg.com`) | npm `three@0.160.0`，本地 `node_modules` |
| 需要外网 | 离线可用 |

### 16. 相机系统

**状态**：已实现

| 之前 | 之后 |
|------|------|
| 正交俯视（OrthographicCamera） | 透视倾斜（PerspectiveCamera 45°） |
| 固定视角 | OrbitControls：右键旋转 / 滚轮缩放 / 中键平移 |
| - | R 键 / 按钮重置视角 |

### 17. 新增功能模块

**状态**：已实现

| 模块 | 职责 |
|------|------|
| 预设地图系统 | 4 张地图（Oval/Hairpin/Highway/Tech Circuit），下拉切换 |
| 全局路径规划 | Dijkstra 最短路径，点击设起终点 |
| 路线导航 | 开环 Frenet 参考线，终点减速 + 到达停止 |
| GPS 定位 | Box-Muller 高斯噪声 σ=5px（1.25m），10Hz，不确定度环可视化 |

### 18. 地面/网格与赛道中心不对齐

**状态**：已修复

**现象**：跑道没有居中在深色网格中央。

**根因**：`PlaneGeometry` 和 `GridHelper` 创建在 Three.js 原点 (0, 0, 0)，但赛道中心在 (w/2, 0, h/2)。正交相机垂直俯视不明显，切换到透视倾斜后偏移暴露。

**解决**：地面和网格 position 移到赛道中心 `(w/2, -0.1, h/2)`，与相机 target 对齐。

### 19. HUD 转向/油门显示 "---"

**状态**：已修复

**现象**：HUD 中转向和油门始终显示 `---`。

**根因**：HTML 有 `#hud-steer` 和 `#hud-throttle` 元素，但 `updateHUD()` 从未写入。

**解决**：从 `CarState` 读取实时 steerAngle 和 speed/maxSpeed 比值。

---

## 架构原则总结

1. **量纲安全**：TypeScript 品牌类型编译时检查，工厂函数是唯一 unsafe cast 点
2. **传感器驱动**：规划器只信任 LiDAR 数据做碰撞检测，不直接使用障碍物列表
3. **HD Map 分工**：地图提供可行驶区域，传感器只检测"意外"障碍物
4. **显式状态机**：规划器状态显式枚举 + 转移函数，可测试可调试
5. **模块单一职责**：接口/算法/渲染三层分离，子模块按职责拆分
6. **See → Plan → Stop**：传感器探测距离 ≥ 轨迹规划视野 ≥ 制动距离
7. **env/algo 严格分离**：环境模拟不包含决策，算法不直接修改环境
