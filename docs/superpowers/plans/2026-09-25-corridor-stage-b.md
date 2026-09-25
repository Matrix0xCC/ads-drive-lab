# 阶段 B 实施计划：误差感知裕度 + 迟滞状态机（含 A 接线，A+B 合并验收）

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 用归档数据标定随速度/曲率变化的物理裕度 `margin(v, κ)`，并引入 TightState 迟滞状态机消除临界抖动；同时把阶段 A 的走廊表示接线（按执行记录：候选由**物理走廊**派生）——A+B 一体进批量门禁。

**Architecture:** 新增 `js/algo/plan/margin.ts`（标定常数 + `marginFor(v, κ)`）与 `js/algo/plan/tight.ts`（TightState 三态 + 空间带 + 3 帧防抖）；`paths.ts` 接线走廊（候选/越界/贴边惩罚 + 物理走廊膨胀用 `marginFor` + 新增"封死走廊"= 物理+0.15m）；`planner.ts` 的停车触发改由 `tight.ts` 的状态驱动。

**Tech Stack:** TypeScript（ESM）、node:test、既有 `corridor.ts`（阶段 A 地基，17 测试）、headless 归档（`runs/*/frames.jsonl`）。

**Spec:** `docs/superpowers/specs/2026-09-25-corridor-planner-evolution-design.md`（§4 阶段 B；附录"执行记录"= A 的接线方案与已证伪实验）
**前置计划：** `docs/superpowers/plans/2026-09-25-corridor-stage-a.md`（A1/A2 已交付；A3 的接线内容与测试迁移经验在"执行记录"中）

## Global Constraints

- **提交纪律**：不主动 commit；每任务完成汇报，提交按用户指示。
- **合并门禁（A+B）**：批量（种子 6-10）**0 段/0 帧碰撞**、**到达 ≥8/10**、到达用时与路线进度不劣化（基线：0/0、8/10；用时 oval-s6 12.3s、s7 19.4s、s9 17.1s、s10 14.7s、hairpin-s6 11.1s、s7 15.6s、s9 15.8s、s10 12.9s；停车场景 oval-s8 60% / hairpin-s8 76%）。
- **标定物（§4.3）**：留出集覆盖率 ≥95%（实测横向跟踪误差被 `margin(v,κ)` 包住的样本占比）。
- **不变量**：`Trajectory` 输出契约；ST 接口；传感器驱动（走廊只吃 LiDAR 命中）；同种子逐帧确定性；量纲 `m()/mps()`；中文注释。
- **不做**：QP、倒车、感知/控制层改动；不重复已证伪实验（spec §1 负结果：发展段下限 6/3m、限速 3m/s、预瞄 4m）。
- **A/B 键控**：所有新行为必须能通过提交独立回滚（改动集中在 `paths.ts` / `planner.ts` / 三个新模块）。

---

### Task B1: 标定工具 + 裕度模块（margin.ts）

**Files:**
- Create: `js/algo/plan/margin.ts`（标定常数 + `marginFor`）
- Create: `js/headless/calibrate-margin.ts`（归档采样 + 分箱分位 + 最小二乘拟合 + 覆盖率报告）
- Test: `js/test/plan/margin.test.ts`、`js/test/headless/calibrate-margin.test.ts`
- 产出：`docs/margin-calibration.md`（标定报告）

**Interfaces（Produces）:**

```ts
// margin.ts —— 物理裕度（车半宽之外的额外余量，m）：calibrated 2026-09-25
export const MARGIN_BASE = /* 标定产出 */ 0.4;      // m₀（低速基础裕度）
export const MARGIN_K = /* 标定产出 */ 0.0;         // k（κ·v² 系数）
export function marginFor(vMps: number, kappa: number): number;   // clamp [0.4, 1.2] m
```

**标定方法（写进 calibrate-margin.ts 头注释）:**
- 采样：遍历 `runs/*/frames.jsonl`，每对相邻 plan 记录间取 step 记录：`e` = 车到"上一帧轨迹"的横向距离（沿用检测器 tracking_error 口径）；`κ` = 该时刻**路线/参考线在车处的曲率**（由 world.json 路线折线 + 车的 frenet s 三点差分）；`v` = 车速（m/s）。
- 分箱：v ∈ [0,5,10,15,20,25,30,35]，κ ∈ [0,0.010,0.020,0.050,0.100]（1/m）；每箱取 e 的 95 分位（样本 ≥20 才参与拟合）。
- 拟合：对箱值最小二乘拟合 `margin = m₀ + k·κ·v²`（加权=箱样本数）；clamp 到 [0.4, 1.2]。
- 报告：样本量/分箱覆盖表/系数/留出集（按 run 划分，训练 70%）覆盖率；写入 `docs/margin-calibration.md`。

- [ ] **Step 1: 写失败测试**（`margin.test.ts`：`marginFor` 单调性——同 κ 下随 v 不减、clamp 上界 1.2、v=0 时 = MARGIN_BASE）
- [ ] **Step 2: 运行确认失败** → **Step 3: 实现 `marginFor`（形式 + 占位常数）** → **Step 4: 通过 + 全量回归**
- [ ] **Step 5: 实现标定脚本 + 单测**（单测用合成样本验证：分箱分位正确、拟合恢复已知系数 ±容差、覆盖率计算正确；脚本对真实 runs 跑一次输出报告）
- [ ] **Step 6: 用标定结果更新 `margin.ts` 常数** + 写 `docs/margin-calibration.md`（含实际分箱表与覆盖率）
- [ ] **Step 7: 汇报**（含标定系数与覆盖率；若覆盖率 <95%，按 spec §4.4 启动补充采集——脚本化机动场景，需用户确认后再加）

---

### Task B2: TightState 迟滞状态机（tight.ts）

**Files:**
- Create: `js/algo/plan/tight.ts`
- Test: `js/test/plan/tight.test.ts`

**Interfaces（Produces）:**

```ts
export type TightState = 'NONE' | 'CREEPING' | 'SEALED';
export interface TightInput {
  /** 存在候选在（未收缩）物理走廊内 */
  hasHardClear: boolean;
  /** 存在候选在舒适走廊内 */
  hasComfortClear: boolean;
  /** 存在候选在"封死走廊"（物理走廊额外收缩 0.15m）内 */
  hasSealClear: boolean;
  /** 当前帧号（防抖用，调用方传 stepCount） */
  step: number;
}
export function nextTight(prev: TightState, in: TightInput, lastChangeStep: number): { state: TightState; lastChangeStep: number };
```

**规则（spec §4.2）:**
- 目标态：`hasSealClear` 都为 false → SEALED；否则 `hasComfortClear` false 且 `hasHardClear` true → CREEPING；否则 NONE；
- 空间带：SEALED 的进入用"封死走廊"（更严），退出用"物理走廊"（更松）→ 临界带内保持原状态；
- 时间防抖：目标态与当前态不同且 `step − lastChangeStep < 3` → 维持当前态；否则切换并更新 `lastChangeStep`。

- [ ] **Step 1: 写失败测试**（转移表全覆盖：NONE→CREEPING→SEALED→CREEPING→NONE；临界带内（hasHardClear=true, hasSealClear=false）在两个方向上的状态保持；3 帧防抖：连续 2 帧目标态不切换、第 3 帧切换；确定性）
- [ ] **Step 2: 运行确认失败** → **Step 3: 实现（纯函数，无模块级状态）** → **Step 4: 通过 + 全量回归** → **Step 5: 汇报**

---

### Task B3: 接线（A 走廊 + B 裕度/迟滞 → paths/planner）

**Files:**
- Modify: `js/algo/plan/paths.ts`、`js/algo/plan/planner.ts`
- Modify: `js/test/plan/paths.test.ts`、`js/test/plan/planner.test.ts`（迁移 + 新增）
- 参考：A 计划"执行记录"（v2 = 候选由物理走廊派生）

**接线内容（按 A3 已实现并回滚的方案 + B 的增量）:**

1. **paths.ts**：
   - 导入 `buildCorridor/deriveTargets/firstExit`（`corridor.ts`）与 `marginFor`（`margin.ts`）；
   - 物理走廊膨胀 = `halfW + marginFor(v0, κ) * 4`（κ = 参考线在车处的曲率，由调用方传入；planner 计算）；舒适走廊保持常数（2.5m）；
   - 新增**封死走廊**：膨胀 = 物理 + 0.15m（供 TightState 的 SEALED 判据）；
   - 候选由**物理走廊**派生（执行记录 v2 教训）；阻塞判定 = `firstExit`（舒适窗口 / 物理全程）；新增 `allSealed`（全部候选越出封死走廊，全程）进 `PathLayerResult`；
   - 贴边惩罚按 spec §3.4（`W_EDGE = 0.5`、`EDGE_BAND = 0.5m`）；
   - 删除 `NUDGE` 与 `firstBlockArc`（距离判定退役；`segPointDist/segSegDist` 若他处未用则删）。
2. **planner.ts**：
   - `plan()` 入参新增内部计算的 `kappa`（参考线 s 处曲率，三点差分）；
   - 停车触发：`stopEngaged = nextTight(prevState, {...}).state === 'SEALED'`（替换 `allBlockedHard` 的直接触发）；`sBlockHard` 的停车基准取封死走廊的首越界弧长；
   - `debug.planning` 增加 `tight: TightState`（inspector 显示；records.ts 归档加可选字段）。
3. **测试迁移**：按 A 计划"执行记录"中的迁移清单（行为断言替代魔法档位值；对拍测试已在 `corridor.test.ts`）；新增：临界带迟滞集成测试（构造抖动扫描序列，断言 planner 的 stopEngaged 不翻转）。

- [ ] **Step 1: paths.ts 接线 + 单测迁移**（`npm test` 全绿；黄金基线若变化按"重取+人工核验"更新）
- [ ] **Step 2: planner.ts 接线（含 tight 状态）+ 集成测试**（临界带不翻转；`debug.planning.tight` 存在）
- [ ] **Step 3: 全量回归** → **Step 4: 汇报**

---

### Task B4: 批量验收（A+B 合并门禁）+ 文档

- [ ] **Step 1: 批量运行**（种子 6-10，10 场景；命令同 A 计划 A4）
- [ ] **Step 2: 聚合验收**（碰撞段/帧、到达率、到达用时、路线进度，对照 Global Constraints 的基线）
  - **门禁不满足即 STOP 汇报**（不得带病进入阶段 C）；若本轮不过，记录负结果并分析（候选经 A 已有两轮教训：优先查"候选丢失/执行跟不上"两类）
- [ ] **Step 3: 浏览器抽查**：`npm run serve` → hairpin-s7（抖动场景）目检"慢行档不抖"与 hairpin-s8（窄口）行为
- [ ] **Step 4: 标定物更新**：用 B 接线后的新归档重跑标定脚本，确认覆盖率 ≥95% 仍成立（κ/v 分布可能变化）
- [ ] **Step 5: 文档**：`CLAUDE.md` 规划器章节（走廊/裕度/迟滞）；`docs/domain.md` 若有新结论（如迟滞参数依据）追加；spec 附录补 B 的执行记录
- [ ] **Step 6: 汇报**（含 A/B 对比表；提交按用户指示）

---

## Self-Review

- **Spec 覆盖**：§4.1 标定→B1；§4.2 迟滞→B2；§4.3 验收（单测/标定物/批量）→B1/B2/B4；§4.4 风险（覆盖不足的补充采集、裕度上界收紧）→B1 Step 7 与 B4 Step 2；A 接线→B3（与 spec 附录的合并验收一致）。
- **占位符扫描**：`MARGIN_BASE/MARGIN_K` 为"标定产出"（B1 有明确的产出流程，非 TODO）；其余步骤均含具体判据/命令。
- **类型一致性**：`marginFor(vMps, kappa)` 的单位（m/s、1/m）在标定与接线处一致（接线时 `v0` 由 u/s 换算 m/s：`(v0 as number)/4`）；`TightInput` 的三个布尔在 paths 的 `allBlocked/allBlockedHard/allSealed` 处映射明确。
- **风险**：A+B 首次合并仍可能不过门禁（A 的两轮失败模式已知：候选丢失=门太小、执行跟不上=门太大）；B 的迟滞与裕度正是两侧的补偿，但若仍不过，B4 Step 2 的 STOP 纪律要求如实记录并回到分析（不连续盲调）。

---

## 执行记录（2026-09-25，B 首轮执行结果）

| 任务 | 状态 | 说明 |
|---|---|---|
| B1 标定 + margin.ts | ✅ 保留 | 54k 样本；κv² 形式被否决（k=0）；速度分段表部署覆盖率 95.2%（≥95% 达标）；`margin.test.ts` 6 测试 + 标定脚本 4 测试 |
| B2 tight.ts 迟滞状态机 | ✅ 保留 | 施密特带（进入用更松 m−0.15、退出用更严 m+0.15）+ 升级立即、降级 3 帧；8 测试 |
| B3 接线 + 测试迁移 | ⚠️ 行为未过门禁 | 走廊族接线、候选由物理走廊派生、贴边惩罚、`sEnd` 视程截断、迟滞状态驱动停车线、`debug.planning.tight`；213 测试全绿（含 3 个新回归测试）；**引擎黄金基线逐位不变** |
| B4 批量验收 | ❌ 门禁未过 → STOP | 见下表：无任何配置同时满足"0 段/0 帧碰撞"与"到达 ≥8/10" |

### B3 顺带修掉的三个真缺陷（均有回归测试）

1. **开环参考线"幻影闭合段"**（`corridor.projectToRef`）：索引绕接 `[n−1 → 0]` 把首尾连成
   虚线段；路线终点之外的障碍命中点被投影到该虚线段（s≈totalS、l≈±15m），膨胀后封死整条
   走廊 → 车在离终点 ~39m 处假停车且永不恢复（`planner-safety` 115m 全宽墙同源）。
   修：开环不绕接 + 最外段无钳制延拓。测试：`corridor.test.ts` 2 项（修复前必失败，已核验）。
2. **`allBlockedHard` 判据错误**：tier 互斥修复后它按 `tierHard.length` 计算，而 tierHard 只含
   "窗口内也阻塞"的子集 → "窗口外全宽墙"（窗口内畅通、全程硬阻塞）被漏判为可通行 →
   停车线取到 `Infinity` → 速度剖面 **NaN**（41 点全 NaN）。修：直接对候选全查 `blockedHard`。
3. **记忆滞后窗口的 Infinity 停车线**：SEALED 保持但本帧已有硬可过候选（恢复待 3 帧确证）时
   `sBlockHard=Infinity` → NaN。修：按舒适制动距离设虚拟停车点兜底（`planner.ts` 已加断言测试）。

### B4 批量证据（种子 6-10，均为同一代码其余部分）

| 配置 | 碰撞 | 到达 | 用时/进度 |
|---|---|---|---|
| 基线（HEAD，A 接线已回滚） | 0 段/0 帧 | 8/10 | oval-s6 12.3 / s7 19.4 / s9 17.1 / s10 14.7；hairpin-s6 11.1 / s7 15.6 / s9 15.8 / s10 12.9；oval-s8 60% / hairpin-s8 76% |
| ① 物理走廊 = marginFor(v)（spec §4.1 字面） | 0 段/0 帧 | **4/10** | oval-s7/s9、hairpin-s7/s9 中途停车；**自锁**：停车 → v=0 → 裕度 1.15m → 缝隙判死 → 永不再起步 |
| ② ①+速度下限（max(v, TIGHT_SPEED)=0.91m） | **3 段/7547 帧** | 6/10 | 缝隙余量 ~0.9m 与预算 0.91m 同量级 → 判"可过"但执行误差吃满余量 → 接触（oval-s9、hairpin-s9） |
| ③ 物理=几何余量 0.4m + 舒适=max(1.5, p95)（本次） | **2 段/7614 帧** | 7/10 | 同上两场景接触；oval-s7 恢复到达（18.7s ✓）；hairpin-s7 16.7-19.9s（基线 15.6 → 超 +0.5s 门禁） |
| ④ ③+横向换边迟滞（squeeze/hard 档主键减 12m 带宽） | 2 段/7620 帧 | 7/10 | 目标不再逐帧翻边（迟滞生效），但**碰撞不变** → 换边抖动不是碰撞根因 |

### 根因（碰撞，oval-s9 / hairpin-s9）

- 场景形态：障碍簇把可行驶域切成 ~1.95m 的旁侧缝隙（车宽 2.0m → **车身余量 ~0.6~0.95m**）。
- 走廊派生候选会**主动找出最紧的缝隙中心**（旧版固定档位不会去找），并以 8m/s 慢行档通过；
  实测横向跟踪误差在低速段 **0.9~1.15m**（标定 p95，docs/margin-calibration.md）> 缝隙余量 →
  规划"可过"而执行必然蹭上（受试 ②③④ 皆如此，仅发生帧不同）。
- 反向配置（①）则因同一预算把缝隙判死 → 4 场景中途停车 + 自锁（停车抬升裕度 → 更过不去）。
- **结论：本批量场景集里"到达"与"安全通过"在同一裕度下互斥**——缝隙余量正好落在标定误差
  预算的边界上。基线的 0/0 + 8/10 建立在"几何余量 0.4m + 固定档位不主动找最紧缝隙"之上。

### 待决策（用户）

1. **候选准入**：给走廊候选加"可执行余量"门槛（按慢行档的标定预算过滤候选，而不是用走廊
   阈值做封死判据）——被过滤时保留其他候选/停车线，避免"只差一点就自锁"。
2. **候选源稳定性**：走廊间隙中心 + 基线固定档位混合（保留基线已证的横向稳定选择）。
3. **回滚接线**（保留 B1/B2 模块与测试作为地基，同 A 的处理），把横向承诺/收敛留给阶段 C。
4. 其他（按用户指示）。
