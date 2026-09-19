# 规划层解耦架构升级设计（SL 路径层 + ST 速度层）

- 日期：2026-09-19
- 状态：设计已与用户逐节确认（S1 架构 / S2 算法 / S3 测试），待用户审阅本文档
- 关联：现状见 `js/algo/plan/*`；历史决策见 `ISSUE_HISTORY.md`（第六/七/八轮实验与结论）

## 1. 背景与目标

**动机**：

1. **行为缺口**：剩余 6 次批量碰撞全部是"域内全部横向候选判阻塞 → 降级最低速硬挤"的几何极限场景。当前耦合采样架构**无法表达"横向无解时纵向停车"**——速度候选最低 17.5 m/s，紧急制动是独立于规划的边缘逻辑。解耦后 stop-line 成为速度层的一等公民，这类场景的正确行为（停在障碍前）自然涌现。
2. **架构学习**：SL/ST 解耦是 L4 系统（Apollo EM Planner 等）的标准规划架构；用户目标包含掌握该架构。
3. **耦合病灶的历史证据**：`W_PROGRESS`（速度奖励）与横向目标捆绑采样导致过"接近障碍加速"；第七轮"延长时域"实验因横纵向捆绑决策而净退化——解耦从结构上消除这类耦合伪影。

**验收标准（已确认）**：架构优先——分层清晰、接口明确、单测覆盖；批量指标**不劣化**（当前基线 6 episode / 195 帧）；行为收益（几何极限场景改为停车）如实 A/B 报告。

## 2. 范围

**包含**：SL 路径层（`paths.ts`）、ST 速度层（`speed.ts`）、`planner.ts` 重写为层编排与合成、旧测试迁移、黄金基线更新、批量 A/B 与浏览器验收。

**非目标**：层间迭代协调（静态障碍下 ST 退化为常值带，迭代无增益；接口预留）；QP 路径优化（不引入依赖）；动态障碍的实现（只做通用 ST 接口）；碰撞物理响应（已备忘缓办）；不改 `collision.ts`/`frenet.ts`/`polynomial.ts`/`states.ts` 的算法本体。

## 3. 架构（S1）

```
plan(snapshot, lidarScan, route?)                  ← 对外契约不变（index.ts 导出不变）
 │
 ├─ Frenet 投影 + 速度适应（vCap → 巡航目标 targetV）
 │
 ├─ ① solvePathLayer()            [paths.ts]
 │     SL 空间路径候选 l(s)，视程 PATH_HORIZON=120m
 │     每条记录 { blocked, sBlock }；输出 { best, ranked, allBlocked, sBlock }
 │
 ├─ ② solveSpeedLayer()           [speed.ts]
 │     ST 空间速度剖面 s(t)；输入含 STObstacle[]（通用接口，静态实现）
 │     输出 { profile, mode: 'cruise'|'stop'|'emergency' }
 │
 ├─ ③ 合成 Trajectory：l(s(t)) 采样 → frenetToCartesian → 点序列
 │     **输出契约零改动**（t,x,y,v,yaw,kappa）；控制/回放/归档/检测不动
 │
 └─ ④ 状态机（states.ts 保留）：
       畅通 + route → FOLLOW；畅通无 route → CRUISE；nearEnd → DECEL（沿用缩放）
       阻塞且停车可行 → DECEL（停车线）；停车不可行 → EMERGENCY
```

**模块变化**：`candidate.ts` 拆为 `paths.ts` + `speed.ts`；`planner.ts` 变编排层；其余原样复用（`polynomial.ts` 的 `solveQuintic` 同时服务横向 l(s) 与停车线 s(t)）。

**ST 接口（通用，静态实现）**：

```typescript
/** 障碍在 ST 平面的占据：s 区间 × 时间区间（静态障碍 tMin=0, tMax=∞） */
export interface STObstacle { sMin: Px; sMax: Px; tMin: Seconds; tMax: Seconds; }
```

当前由路径层的碰撞命中推导（静态：占据区间 = 命中点弧长 ± 障碍有效半长，时间全轴）。

## 4. 算法细节（S2）

### 4.1 路径层（SL）

- **候选**：`PATH_LATERAL_TARGETS = [lo, lo−2.5m, lo+2.5m, −L_MAX, +L_MAX]`（lo=1.875m，L_MAX=4.4m 含 0.6m 过冲余量，沿用现约束）clamp 去重（容差 0.5u）→ 典型 4 条
- **生成**：`solveQuintic(l0, dl0, 0, targetL, 0, 0, PATH_HORIZON)`，采样 `PATH_STEPS = 40` 点
- **碰撞检查**：**整条路径**逐点 `isLidarCollision(点, 车, scan, margin)`（不再只剪枝前 4 点）；`margin = SAFETY_MARGIN + car.collisionRadius`（沿用现值 ≈4.96m）；记录首次命中点的弧长 `sBlock`（停车线输入）
- **代价**（非阻塞路径择优）：`W_PATH_CENTER·Σ(l_i−lo)²/N + W_PATH_CURV·Σ|Δyaw|/N + W_SWITCH_PATH·[路径跨档切换]`；权重初值沿用旧量级：`W_PATH_CENTER=0.3, W_PATH_CURV=1.0, W_SWITCH_PATH=100`
- **全阻塞时排序**：按 `sBlock` 降序（选"最晚撞"的路径作为停车基准）

### 4.2 速度层（ST）

- **巡航目标**：`targetV = clamp(minObsDist/2, mps(17.5), mps(35))`（vCap 沿用）；route DECEL 缩放沿用
- **剖面候选**：
  - `cruise`：`solveSpeedProfile(v0, 0, targetV, T=secs(2.0))`（现有函数复用）
  - `stop-line`：`sStop = sBlock − STOP_MARGIN(m(4))`；`T = max(2·sStop/v0, secs(0.8))`；`solveQuintic(0, v0, 0, sStop, 0, 0, T)`；峰值减速度超 `A_COMFORT(mps2(3.5))` 时按 `T ×= sqrt(a_peak/A_COMFORT)` 延长（至多 3 次迭代）
  - `emergency`：`sStop < v0²/(2·A_MAX)`（`A_MAX = mps2(6)`）→ 物理停不住 → 最大减速剖面（接受接触，语义同现 EMERGENCY）
- **选择**：路径阻塞 → stop/emergency；畅通 → cruise；停车为持续状态（停稳后每帧重评估）
- **删除的概念**：多档速度采样网格、`(targetL×targetV)` 组合、fallback 最低速、前 4 点剪枝。算量：4 路径 × 40 点 × 锥形检查 ≈ 1 万次比较/规划帧，15 Hz 无压力

### 4.3 合成

`i=0..N: t_i → s_i=profile(t_i) → l_i=lPath(s_i) → frenetToCartesian(s0+s_i, l_i) → {t,x,y,yaw,v=ds/dt,kappa≈Δyaw/Δs}`；轨迹时长 = 速度层 T（不再固定 2s）；五次 × 五次 → 曲率 C² 连续。

## 5. 测试与验收（S3）

**新单测**：

- `js/test/plan/paths.test.ts`：① 候选域内 ② 全阻塞标记与 sBlock 正确性（近距中央障碍）③ 畅通保持车道 / 侧向可行择侧（改写自现避障用例）④ 无解时按 sBlock 降序选路
- `js/test/plan/speed.test.ts`：① stop-line 端点 BC（s(T)≈sStop、v(T)≈0）与峰值减速度约束 ② 距离不足 → emergency 兜底 ③ cruise 目标一致性
- 合成契约并入 `planner.test.ts`：t 递增、v ≤ maxSpeed、起点贴近车

**旧测试迁移**：

| 旧测试 | 处置 |
|---|---|
| `planner.test.ts` | 场景保留，仅新语义处微调断言 |
| `planner-safety.test.ts` | 不变量保留；"15m 不可避让"用例升级为"stop-line 停在障碍前"（v 渐 0） |
| `candidate-corridor.test.ts` | 域约束 → 迁移 paths 测试；vCap 限速断言保留 |
| `candidate-safety.test.ts` | 碰撞判定语义 → 迁移 paths 测试；旧 API 断言重写/删除 |
| `collision-angle` / `frenet` / `polynomial` | 完全保留 |
| engine 黄金基线 | 一次性更新（人工核验后冻结） |

**验收**：全量测试绿（净增约 10-12 例）→ 批量 10 种子 A/B：门槛 **不劣化**；预期几何极限场景变为"停在障碍前"（碰撞帧显著下降），衍生的 stall/route_timeout 检测项**明账列出**（正确降级行为）→ 浏览器手工验收：观察停车行为 + 巡航/避障/路线/回放无回归 → 更新 CLAUDE.md（规划器一节改写）与 ISSUE_HISTORY（升级决策 + A/B 数据）。

## 6. 实施顺序与风险

**顺序**（细节交实施计划）：① `paths.ts`（TDD）→ ② `speed.ts`（TDD）→ ③ planner 编排重写 + 合成 + 测试迁移 → ④ 黄金基线更新 → ⑤ 批量 A/B + 浏览器验收 → ⑥ 文档。

| 风险 | 缓解 |
|---|---|
| 新架构调参面（路径权重/停车参数）导致指标反噬 | 权重初值沿用旧语义量级；A/B 门槛只要求不劣化；反例单变量回退已制度化 |
| 停车衍生 issue（stall/route_timeout）被误当回归 | 设计接受为正确降级；A/B 报告单列；检测器不改语义 |
| 黄金基线再次重置 | 例行操作（已有成熟流程：人工核验后冻结） |
| 旧测试迁移遗漏耦合语义 | 迁移表逐项处理；耦合专属 API（evaluateTrajectory 旧签名）随实现删除 |

## 7. 开放问题（实现时定稿，不阻塞开工）

- 停车参数终值（STOP_MARGIN/A_COMFORT/A_MAX）先按本文默认值实现，批量 A/B 后如需微调按单变量纪律执行
- 停车后若路径恢复畅通（静态障碍下不会发生）：恢复巡航逻辑在每帧重评估中自然覆盖，无需专门机制
