# 阶段 A 实施计划：走廊化横向表示（corridor-based lateral representation）

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把横向层从"离散候选 + 逐命中距离阈值"升级为"自由走廊（corridor）+ 走廊派生候选 + 越界即阻塞"——消除候选打不中缝隙中心、被迫贴边两类结构性问题。

**Architecture:** 新增 `js/algo/plan/corridor.ts`（纯几何：命中 → Frenet 投影 → 膨胀 → 每 s 自由区间 + 瓶颈 + 候选派生）；`paths.ts` 接入：候选由走廊派生、阻塞判定 = 路径越出走廊（双走廊 = 双阈值）、代价加贴边惩罚；两段式发展段/晚承诺窗口/合成/输出契约全部不变。

**Tech Stack:** TypeScript（ESM，`.js` 后缀 import）、node:test、既有 `frenet.ts`（`cartesianToFrenet` / `frenetToCartesian`）、量纲工厂 `m()/px()/rad()`。

**Spec:** `docs/superpowers/specs/2026-09-25-corridor-planner-evolution-design.md`（§3 阶段 A 为准；§1 的负结果清单勿重复实验）

## Global Constraints

- **提交纪律**：本仓库**不主动 commit**；每个任务完成后汇报，提交按用户指示。
- **验收门禁（阶段 A，spec §3.6）**：批量（`scenarios/batch/` 种子 6-10）**碰撞 0 段/0 帧不劣化、到达 ≥8/10、到达用时与路线进度不劣化**（基线：0/0、8/10、用时 11.1~19.4s、进度 98-99%；停车场景 oval-s8 60% / hairpin-s8 76%）。
- **不变量**：`Trajectory` 输出契约；`solvePathLayer` 的 `PathLayerResult` 字段语义（`blocked/sBlock/blockedHard/sBlockHard/allBlocked/allBlockedHard/sBlockHard`）；ST 接口；传感器驱动（走廊只吃 `lidarScan` 派生命中，禁止障碍物真值）；同种子逐帧确定性；量纲用 `m()/mps()` 书写。
- **不做**：QP 优化器、倒车、感知/控制层改动、动态障碍。
- 注释中文；`algo/` 模块必须有算法原理/参数依据注释。

---

### Task A1: 走廊构建（corridor.ts：投影 / 膨胀 / 自由区间 / 瓶颈）

**Files:**
- Create: `js/algo/plan/corridor.ts`
- Test: `js/test/plan/corridor.test.ts`

**Interfaces（Produces，后续任务依赖）:**

```ts
export interface FreeSeg { lmin: number; lmax: number }        // 车辆中心可到达的横向区间（u）
export interface CorridorStep { s: number; free: FreeSeg[] }   // 与路径采样同网格（见下）
export interface Corridor {
  steps: CorridorStep[];   // length = steps+1，与 PathCandidate.points 索引一一对应
  pinch: { s: number; width: number; seg: FreeSeg | null } | null;
}
export function buildCorridor(o: {
  ref: FrenetReference; s0: number;
  hits: { x: number; y: number; d: number; low: boolean }[];
  hitSegs: { ax: number; ay: number; bx: number; by: number; low: boolean }[];
  inflate: number; inflateLow: number;   // 膨胀量（常规命中 / 低置信命中）
  lMax: number; horizon: number; steps: number; windowRel: number;
}): Corridor;
```

**算法（写进模块头注释）:**

1. 命中投影：每个命中点 `cartesianToFrenet(hit, ref)` → `(s_h, l_h)`；链段用两端点投影覆盖其 s 范围（`Δs = |s_a − s_b|/2 + 小量`，l 区间 = `[min(l_a,l_b), max(l_a,l_b)]`）。
2. 膨胀：`infl = low ? inflateLow : inflate` → 阻塞区间 `[s_h − Δs, s_h + Δs] × [l_h − infl, l_h + infl]`（点 Δs=0）。
3. 每个网格步 `s_i = s0 + i/steps × horizon`：取覆盖 s_i 的阻塞区间的 l 并集 → 在 `[−lMax, lMax]` 内求补集（保序、只保留宽度 >0 的段）。
4. 瓶颈（pinch）：在 `s_i ≤ s0 + windowRel` 内，取"最大自由段宽度"最小的那一步；若某步无自由段 → `width=0, seg=null`（全封死）；全程无命中 → pinch 取窗口内任一步、`width = 2·lMax`。

- [ ] **Step 1: 写失败测试**（`js/test/plan/corridor.test.ts`）

```ts
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { buildCorridor } from '../../algo/plan/corridor.js';
import { buildFrenetReference } from '../../algo/plan/frenet.js';
import { px, m } from '../../shared/dimensions.js';

const ref = buildFrenetReference(
  Array.from({ length: 61 }, (_, i) => ({ x: px(i * 10), y: px(0) })), true);
const BASE = { ref, s0: 0, inflate: m(2.5), inflateLow: m(4.5), lMax: m(4.4), horizon: m(120), steps: 40, windowRel: m(60) };

describe('corridor: 自由区间', () => {
  it('无命中 → 全宽单段', () => {
    const c = buildCorridor({ ...BASE, hits: [], hitSegs: [] });
    for (const st of c.steps) {
      assert.strictEqual(st.free.length, 1);
      assert.ok(Math.abs(st.free[0].lmin + (m(4.4) as number)) < 1e-6);
    }
  });
  it('中央命中点（200u）→ 中线两侧各一段，边界=命中 l ± 膨胀', () => {
    const c = buildCorridor({ ...BASE, hits: [{ x: px(200), y: px(0), d: px(200), low: false }], hitSegs: [] });
    const st = c.steps.find(s => Math.abs(s.s - 200) < 1)!;
    assert.strictEqual(st.free.length, 2);
    assert.ok(Math.abs(st.free[0].lmax + (m(2.5) as number)) < 1e-6, '左段右边界 = −inflate');
    assert.ok(Math.abs(st.free[1].lmin - (m(2.5) as number)) < 1e-6, '右段左边界 = +inflate');
  });
  it('低置信命中 → 膨胀更大', () => {
    const c = buildCorridor({ ...BASE, hits: [{ x: px(200), y: px(0), d: px(200), low: true }], hitSegs: [] });
    const st = c.steps.find(s => Math.abs(s.s - 200) < 1)!;
    assert.ok(Math.abs(st.free[1].lmin - (m(4.5) as number)) < 1e-6);
  });
  it('偏置命中（贴右）→ 左侧自由段更宽（oval-s2 类几何）', () => {
    const c = buildCorridor({ ...BASE, hits: [{ x: px(200), y: px(0), d: px(200), low: false },
      { x: px(200), y: px(6.4), d: px(206), low: false }], hitSegs: [] });
    // 右偏命中 l≈1.6m（6.4u）：左段 [−17.6, 1.6−10]=[−17.6, −8.4]，宽 9.2u；右段极窄
    const st = c.steps.find(s => Math.abs(s.s - 200) < 1)!;
    const widest = st.free.reduce((a, b) => (b.lmax - b.lmin) > (a.lmax - a.lmin) ? b : a);
    assert.ok(Math.abs(widest.lmax - (6.4 - 10)) < 0.5, `最宽段右边界≈−3.6u，实际 ${widest.lmax}`);
  });
  it('链段横跨中线 → 单侧自由段被切成两段（s 方向）', () => {
    const c = buildCorridor({ ...BASE, hits: [],
      hitSegs: [{ ax: px(200), ay: px(-6), bx: px(200), by: px(6), low: false }] });
    const before = c.steps.find(s => Math.abs(s.s - 160) < 1)!;
    const at = c.steps.find(s => Math.abs(s.s - 200) < 1)!;
    assert.strictEqual(before.free.length, 1, '链段之前全宽');
    assert.strictEqual(at.free.length, 2, '链段处被膨胀区间切开');
  });
});

describe('corridor: 瓶颈', () => {
  it('窗口内最窄处 → pinch.s/width 指向它；无自由段 → seg=null', () => {
    const c = buildCorridor({ ...BASE, hits: [{ x: px(220), y: px(0), d: px(220), low: false }], hitSegs: [] });
    assert.strictEqual(c.pinch!.s, px(220), '唯一瓶颈在命中点处');
    assert.ok(c.pinch!.width > 0);
  });
});
```

- [ ] **Step 2: 运行测试确认失败**：`npx tsc && node --test dist/js/test/plan/corridor.test.js` → 模块不存在（编译错）。预期失败。
- [ ] **Step 3: 实现 `corridor.ts`**（按 Interfaces + 算法注释；投影用 `cartesianToFrenet`；纯函数、无模块级状态）
- [ ] **Step 4: 运行测试至通过**；再跑全量 `npm test` 确认无回归。
- [ ] **Step 5: 汇报**（不提交）。

---

### Task A2: 候选派生（deriveTargets：间隙中心 / 密集采样 / ≤7）

**Files:**
- Modify: `js/algo/plan/corridor.ts`（追加导出）
- Test: `js/test/plan/corridor.test.ts`（追加 describe）

**Interfaces（Produces）:**

```ts
export function deriveTargets(o: {
  corridor: Corridor; laneOffset: number; l0: number; lMax: number;
  spacing: number;      // 0.5m = m(0.5)
  maxTargets: number;   // 7
}): number[];
```

**规则（spec §3.3）:**

1. `pinch.seg === null`（全封死）→ 返回 `[laneOffset]`（单候选，交由停车线处理）。
2. 分段选择：优先"含 laneOffset 的自由段"；否则"离 l0 最近"的段；再追加"其余段中离 l0 最近"的一段（**最多 2 段**）。
3. 每段候选：`{段中点, lmin+0.3m, lmax−0.3m}`（内缩值若越出段或与已有值距离 <0.5u 则去重；段宽 <1m 时自然只剩中点）。
4. 追加"车道中心"（若它落在 pinch 处任一段内）。
5. 优先级裁剪到 `maxTargets`：段中点（间隙中心）> 车道中心 > 内缩边界；同级按 `|target − laneOffset|` 升序。
6. clamp 到 `[−lMax, lMax]`。

- [ ] **Step 1: 写失败测试**

```ts
describe('corridor: 候选派生（间隙中心必被覆盖）', () => {
  const mk = (hits: any[]) => buildCorridor({ ...BASE, hits, hitSegs: [] });
  it('中央点障碍 → 两侧间隙中心都在候选里', () => {
    const t = deriveTargets({ corridor: mk([{ x: px(200), y: px(0), d: px(200), low: false }]),
      laneOffset: px(7.5), l0: px(0), lMax: m(4.4), spacing: m(0.5), maxTargets: 7 });
    assert.ok(t.some(v => Math.abs(v - (-m(4.4) as number) / 1) < 0.01 || v < 0), '含左侧目标');
    assert.ok(t.some(v => v > 0), '含右侧目标');
    assert.ok(t.length <= 7);
  });
  it('障碍贴右 → 最左侧间隙中心 ≈ −3.1m 在候选里（oval-s2 回归）', () => {
    // 右偏命中 l≈1.6m、膨胀 2.5m → 左段 [−4.4, −0.9]m，中心 ≈ −2.65m
    const t = deriveTargets({ corridor: mk([{ x: px(200), y: px(6.4), d: px(206), low: false }]),
      laneOffset: px(7.5), l0: px(0), lMax: m(4.4), spacing: m(0.5), maxTargets: 7 });
    assert.ok(t.some(v => Math.abs(v - (-2.65 * 4)) < 1.2), `实际候选: ${t.map(v => v / 4)}`);
  });
  it('全封死 → 单候选 [laneOffset]', () => {
    const c = mk([{ x: px(2), y: px(0), d: px(2), low: false }, { x: px(2), y: px(4), d: px(4.5), low: false },
      { x: px(2), y: px(-4), d: px(4.5), low: false }]);
    const t = deriveTargets({ corridor: c, laneOffset: px(7.5), l0: px(0), lMax: m(4.4), spacing: m(0.5), maxTargets: 7 });
    assert.deepStrictEqual(t, [px(7.5)]);
  });
});
```

- [ ] **Step 2: 运行确认失败** → **Step 3: 实现** → **Step 4: 通过 + 全量回归** → **Step 5: 汇报**。

---

### Task A3: paths.ts 接入走廊（阻塞=越界 + 贴边惩罚 + 候选迁移）

**Files:**
- Modify: `js/algo/plan/paths.ts`
- Test: `js/test/plan/paths.test.ts`（新增对拍/贴边惩罚；迁移旧断言）

**Interfaces（Consumes A1/A2）:**

```ts
// 走廊与路径点同网格（Task A1 保证：索引 i ↔ s0 + i/PATH_STEPS×PATH_HORIZON）
export function firstExit(corridor: Corridor, points: PathCandidate['points'], windowRel: number): number;
```

- [ ] **Step 1: 走廊接入（在 `solvePathLayer` 内）**

```ts
const halfW = (o.carWidth as number) / 2;
const comfort = buildCorridor({ ref: o.ref, s0: o.s0, hits, hitSegs,
  inflate: halfW + (m(1.5) as number), inflateLow: halfW + (m(1.5) as number) + (m(2.0) as number),
  lMax: L_MAX, horizon: PATH_HORIZON as number, steps: PATH_STEPS, windowRel: COMMIT_WINDOW as number });
const physical = buildCorridor({ ...same, inflate: halfW + (m(0.4) as number), inflateLow: halfW + (m(0.4) as number) + (m(2.0) as number) });
const targets = deriveTargets({ corridor: comfort, laneOffset: lo, l0: o.l0 as number,
  lMax: L_MAX, spacing: m(0.5) as number, maxTargets: 7 });
```

阻塞判定替换（删除 `firstBlockArc` 与 `NUDGE` 常量）：

```ts
const sBlock = firstExit(comfort, points, COMMIT_WINDOW as number);   // 与旧语义：越出舒适走廊
const sBlockHard = firstExit(physical, points, Infinity);             // 越出物理走廊
```

`firstExit` 实现：逐索引 `i`，路径点 `l = points[i].l`；自由 ⇔ `corridor.steps[i].free.some(seg => l >= seg.lmin && l <= seg.lmax)`；越出即返回 `points[i].s`；`points[i].s − o.s0 > windowRel` 则停。走廊网格与路径网格同构（同 `steps`/`horizon`/`s0`，由 Task A1 的接口保证）。

- [ ] **Step 2: 贴边惩罚（cost）**

```ts
const EDGE_BAND = m(0.5) as number;
const W_EDGE = 0.5;   // 与 W_PATH_CENTER 同量级起手；A/B 可调
let edgeSum = 0;
for (let i = 0; i < points.length; i++) {
  const segs = physical.steps[i].free;   // 用物理走廊的边界（贴边即接近物理极限）
  let dEdge = 0;
  for (const sg of segs) if (l >= sg.lmin && l <= sg.lmax) { dEdge = Math.min(l - sg.lmin, sg.lmax - l); break; }
  edgeSum += Math.max(0, EDGE_BAND - dEdge) ** 2;
}
cost += W_EDGE * edgeSum / points.length;
```

- [ ] **Step 3: 对拍测试（新阻塞判定 ⇔ 旧判定的等价性）**

在 `paths.test.ts` 里保留一份**旧算法参考实现**（把改造前的 `firstBlockArc` 逐命中距离判定复制为测试内私有函数 `legacyFirstBlock`），对以下扫描断言两种判定的 `blocked/sBlock` 一致（容差：sBlock 差 ≤ 2m）：

```ts
it('对拍：走廊越界判定与旧逐命中距离判定一致（容差 2m）', () => {
  for (const scan of [blockedScan(100, 0.025), blockedScan(60), blockedScan(300, 0.025), clearScan()]) {
    const r = solve(scan);
    for (const c of r.ranked) {
      const legacy = legacyFirstBlock(c.points, /* 旧 CLEAR */ 10, Infinity);
      if (c.blocked) assert.ok(isFinite(legacy) && Math.abs(c.sBlock - legacy) <= 8, `L=${c.targetL}: 新 ${c.sBlock} vs 旧 ${legacy}`);
      else assert.strictEqual(legacy, Infinity);
    }
  }
});
```

- [ ] **Step 4: 迁移既有断言**（走廊派生后具体档位值改变，只保留**行为断言**）：

| 测试 | 迁移方式 |
|---|---|
| `paths.test` '中央窄障碍（100u）' | 断言语义不变（inLane.blocked、best 为侧向），删除对具体 `targetL≈LO` 的数值依赖 → 改为"best 不越出最优自由段中点 ±1m" |
| `paths.test` '窗口外障碍(300u)' | 同上：`best.targetL === inLane.targetL` 改为"best 在左段内" |
| `planner.test` '双阈值（紧间隙慢行）' | 断言 `speedTarget === mps(8)` 保留；best 目标值改为"落在间隙内" |
| `planner.test` '自适应发展段' | 保持（不依赖档位值） |
| 其余 | 只保留 `blocked/blockedHard/防 NaN/emergency` 类行为断言 |

- [ ] **Step 5: 全量测试** `npm test` 通过（黄金基线若因路径形状变化而变，按"重取+人工核验"流程更新并在测试注释记录）。
- [ ] **Step 6: 汇报**（不提交）。

---

### Task A4: 批量 A/B 验收 + 文档

**Files:**
- Modify: `CLAUDE.md`（规划器章节同步走廊表示）、`docs/domain.md`（若出现新结论）
- 产出：批量对比数据（写入本次汇报/文档）

- [ ] **Step 1: 批量运行**（种子 6-10，10 场景）：

```bash
for s in oval-s6 oval-s7 oval-s8 oval-s9 oval-s10 hairpin-s6 hairpin-s7 hairpin-s8 hairpin-s9 hairpin-s10; do
  node dist/js/headless/run.js --scenario scenarios/batch/$s.json > /dev/null 2>&1
done
```

- [ ] **Step 2: 聚合验收**（碰撞段/帧、到达率、到达用时、路线进度）：

```bash
for d in $(ls -dt runs/*batch* | head -10); do node -e "
const s=require('./$d/summary.json');const j=require('./$d/issues.json');
const c=j.filter(i=>i.check==='collision_true');
console.log('$d', c.reduce((a,x)=>a+x.count,0)+'f/'+c.length+'ep',
  '到达 '+(s.arrivalTimeS!=null?s.arrivalTimeS.toFixed(1)+'s':'—'),
  '进度 '+((s.progress??0)*100).toFixed(0)+'%');
"; done
```

**门禁（不满足即 STOP 并汇报，不得带病进入阶段 B）**：
- 碰撞 **0 段 / 0 帧**；
- 到达 **≥8/10**；
- 用时 ≤ 基线 +0.5s / 场景（基线：oval-s6 12.3、s7 19.4、s9 17.1、s10 14.7；hairpin-s6 11.1、s7 15.6、s9 15.8、s10 12.9）；
- 进度：oval-s8 ≥60%、hairpin-s8 ≥76%。

- [ ] **Step 3: 浏览器抽查**：`npm run serve` → 加载 hairpin-s7（此前的假阻塞场景）目检轨迹是否居中过缝。
- [ ] **Step 4: 文档**：CLAUDE.md 规划器章节（候选来源改为"走廊派生"、阻塞判定改为"越界判定"、新增 `corridor.ts` 职责）；`docs/domain.md` 若有新结论（如"走廊覆盖率"实测）追加。
- [ ] **Step 5: 汇报**（含 A/B 表；提交按用户指示）。

---

## Self-Review

- **Spec 覆盖**：§3.1 走廊构建→A1；§3.2 双走廊→A3 Step1；§3.3 候选派生→A2；§3.4 贴边惩罚→A3 Step2；§3.5 不变项→A3 未触碰项；§3.6 验收（间隙中心存在性→A2 测试；对拍→A3 Step3；批量门禁→A4）；§3.7 风险（投影近似→A3 对拍；性能→A4 观察）。
- **占位符扫描**：无 TBD/TODO；W_EDGE 初值 0.5 已给定（A/B 可调）。
- **类型一致性**：`buildCorridor`/`deriveTargets`/`firstExit` 的签名在 A1/A2/A3 间一致；`Corridor.steps` 与 `PathCandidate.points` 索引对齐的前提在 A1 注释与 A3 对拍中双保险。
- **已知风险**：旧测试断言迁移（A3 Step4）是主要工作量与风险点——原则：只迁断言形式，不迁行为期望；行为变化需在 A4 的 A/B 中可见。

---

## 执行记录（2026-09-25，本计划首轮执行结果）

| 任务 | 状态 | 说明 |
|---|---|---|
| A1 走廊构建 | ✅ 保留 | `corridor.ts`（含 `firstExit`）+ `corridor.test.ts`（17 测试，含对拍） |
| A2 候选派生 | ✅ 保留 | 同上文件；间隙中心覆盖/全封死兜底/确定性均单测 |
| A3 paths 接入 | ⚠️ 已回滚 | 接入按计划完成并通过全部单测（含对拍、黄金基线重取核验），但批量门禁不过 → 接线回滚 |
| A4 批量验收 | ❌ 门禁未过 → STOP | v1（舒适走廊派生）3/10 到达；v2（物理走廊派生）2 段/7661 帧碰撞、5/10 到达 |

**决策（用户确认）**：回滚接线，`corridor.ts` 作为地基保留；**阶段 A 的验收并入 A+B 合并门禁**
（详见 spec `2026-09-25-corridor-planner-evolution-design.md` 附录"执行记录"）。下一轮实现
阶段 B 时，A 的接线与 B 一起进批量验收；对拍测试与断言迁移的经验（A3 Step 4 的迁移清单）
在合并阶段复用。

**回滚确认**：`git checkout` paths.ts / paths.test.ts / 引擎黄金基线后：191 测试全绿；
批量 0/0、到达 8/10、用时/进度与基线逐项一致。
