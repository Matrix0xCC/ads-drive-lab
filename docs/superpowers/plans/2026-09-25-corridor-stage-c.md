# 阶段 C 实施计划：路径-速度收敛迭代（含门禁与能力边界协议）

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把"路径层按当前速度取几何 → 速度层定速度"的单轮顺序传递，改成 **v 与路径几何联合求解的收敛迭代**（EM 风格，≤3 轮），使"停车后起步 / 窄口慢行 / 紧间隙"不再因自身参数化自锁；并把收敛轮数与单帧规划耗时固化进归档与报告。

**Architecture:** `planner.ts` 的 §5–§6 之间引入迭代：每轮用假设速度 `vAssume` 调路径层（裕度 `margin(vAssume)`、发展段 `S(vAssume)`），由路径几何反推速度上限 `u`（封死 0 / 紧间隙慢行档 / 横向机动可行速度 / 巡航目标），`|ΔtargetL|<0.2m 且 |Δu|<0.5m/s` 视为收敛；变化不降反增则提前停（防极限环）。`paths.ts` 新增 `lateralSpeedCap`（横向加速度上限反解速度）与候选的 `develop` 字段，物理走廊裕度恢复为标定表 `marginFor(v)`（spec §4.1/§5）。

**Tech Stack:** TypeScript（ESM）、node:test、既有 `corridor.ts`/`margin.ts`/`tight.ts`、headless 归档（`runs/*/frames.jsonl` + `summary.json`）。

**Spec:** `docs/superpowers/specs/2026-09-25-corridor-planner-evolution-design.md` §5（交替结构、性能与确定性、验收）+ §5.3（能力边界条款）
**前置计划:** `docs/superpowers/plans/2026-09-25-corridor-stage-b.md`（B 的接线已在位；批量门禁未过，证据见其"执行记录"）

## 前置事实（写计划前核验，勿重复实验）

1. **B 的批量证据**（种子 6-10，逐帧确定性）：物理走廊用标定表 → **0 段/0 帧碰撞但 4/10 到达**（窄口判死 + `v=0 → 裕度最大 → 封死 → 永不起步`自锁）；物理走廊用几何余量 0.4m → **7/10 到达但 2 场景接触**（oval-s9 2908 帧、hairpin-s9 4712 帧）。
2. **窄口尺度**：障碍簇只留 **0.6~0.95m 车身余量**（车宽 2.0m），而标定 p95 跟踪误差在低速段 **0.9~1.15m**（`docs/margin-calibration.md`）。
3. **C0 前提（机动/稳态重标定）已被数据否决**（同文件"第二轮分析"）：稳态（ρ≤0.2m）p95 = 0.728m vs 机动 0.793m，按速度分层同样不分离（v 5-10：0.902 vs 0.912）；目标跳变、横向速率预测量也不分离。**结论：预算不能靠重标定收紧；窄口要通过，必须先让轨迹形状稳定（本阶段的迭代）或在候选准入层面不选过窄缝隙。** 因此本阶段**不做 C0**，物理走廊继续用标定速度表。
4. **基线（HEAD）**：0 段/0 帧、8/10 到达；用时 oval-s6 12.3 / s7 19.4 / s9 17.1 / s10 14.7；hairpin-s6 11.1 / s7 15.6 / s9 15.8 / s10 12.9；进度 oval-s8 60% / hairpin-s8 76%。

## Global Constraints

- **提交纪律**：不主动 commit；每任务完成汇报，提交按用户指示。
- **确定性**：同种子逐帧一致；迭代为纯函数链（耗时指标只记录、不参与决策）。
- **量纲**：所有物理量用 `m()/mps()/mps2()` 书写；品牌类型边界用工厂函数。
- **接口契约**：`Trajectory` 输出契约、ST 接口、归档 schema（新增字段必须**可选**，旧归档回放不崩）、传感器驱动（只吃 LiDAR 命中）、中文注释。
- **预算**：单帧规划总耗时 <5ms（15Hz）；超时只记录（`planMs`）不改变行为。
- **不做**：倒车、QP、感知/控制层改动；不重复已证伪实验（机动感知重标定、发展段下限 6/3m、限速 3m/s、预瞄 4m）。
- **A/B 可回滚**：改动集中在 `paths.ts` / `planner.ts` / `debug.ts` / `records.ts` / `detector.ts` / `report.ts`。

## Review Focus

1. **车已停在窄口前（v0=0）且缝隙"慢行可过"** → 不得因"v=0 裕度最大"判死并永不起步（自锁）；迭代的几何评估速度下限必须打破它。
2. **极限环**：两轮之间 `targetL` 或 `u` 交替不收敛 → 必须在 ≤3 轮内停下且输出确定（不得无界循环、不得 NaN）。
3. **速度上限为 0**（物理封死）→ 不得把 0 当作"巡航目标 0m/s"喂给速度层；停车必须由停车线剖面负责。
4. **路线剩余不足一个网格步 / sEnd 很近** → 走廊被截断时迭代不得越界取 `best`（不得抛异常）。
5. **旧归档**（无 `rounds`/`planMs` 字段）→ 检测器/报告/回放不得崩（字段可选 + 缺省）。

---

## 文件结构（本阶段）

- `js/algo/plan/paths.ts`（改）：导出 `lateralSpeedCap`；`PathCandidate` 增 `develop`；物理走廊裕度恢复 `marginFor(v)`。
- `js/algo/plan/planner.ts`（改）：§5–§6 的收敛迭代 + `pathSpeedCap`；`targetV` 由迭代末轮决定；debug 增 `rounds`/`planMs`/`vAssume`。
- `js/shared/debug.ts`（改）：`PlanningDebug` 增三个字段。
- `js/sim/records.ts`（改）：plan 归档记录增 `rounds`/`planMs`。
- `js/shared/types/interfaces/archive.ts`（改）：`PlanRecord` 增**可选** `rounds?`/`planMs?`。
- `js/headless/detector.ts`（改）＋`js/headless/report.ts`（改）：聚合与报告段落。
- `js/ui/inspector.ts`（改）：面板显示轮数/耗时。
- 测试：`js/test/plan/paths.test.ts`、`js/test/plan/planner.test.ts`、`js/test/sim/records.test.ts`、`js/test/headless/report.test.ts`。

---

### Task C1: 收敛迭代核心（planner.ts）+ 横向速度上限（paths.ts）

**Files:**
- Modify: `js/algo/plan/paths.ts`（导出 `lateralSpeedCap`；`PathCandidate.develop`；物理走廊裕度改回 `marginU`）
- Modify: `js/algo/plan/planner.ts`（迭代 + `pathSpeedCap`）
- Modify: `js/shared/debug.ts`（`rounds`/`planMs`/`vAssume`）
- Test: `js/test/plan/paths.test.ts`（裕度自锁的对拍）、`js/test/plan/planner.test.ts`（迭代行为）

**Interfaces:**
- Produces（paths.ts）:
  - `export function lateralSpeedCap(dlU: number, developU: number): number` —— 横向机动可行速度上限（仿真单位/s；`dlU < 1e-6` → `Infinity`）
  - `PathCandidate.develop: number` —— 该候选实际使用的发展段长度（自适应缩短后的值）
- Produces（planner.ts，供归档/测试）: `debug.planning.rounds: number`（本帧路径层求值次数）、`debug.planning.planMs: number`（本帧 `plan()` 总耗时毫秒）、`debug.planning.vAssume: number`（迭代末轮假设速度，仿真单位/s）
- Consumes: `marginFor`（margin.ts）、`TIGHT_SPEED`（paths.ts）、`solvePathLayer` / `solveSpeedLayer`（现状）。

- [ ] **Step 1: 写失败测试（paths.test.ts：裕度自锁对拍）**

```ts
describe('路径层：物理走廊裕度随速度变化（迭代的动机）', () => {
  it('同一窄口：v=0 时裕度最大（1.15m）封死，慢行档（8m/s，0.91m）留下可过缝隙', () => {
    // 障碍横跨 l≈±1.2m（2.4m 宽）位于 25m 处：低速裕度差 0.24m 恰好决定封死/可过
    const n = 61, angles: any[] = [], distances: any[] = [];
    for (let i = 0; i < n; i++) {
      const a = -Math.PI / 3 + (i / (n - 1)) * 2 * Math.PI / 3;
      angles.push(rad(a));
      const hit = Math.abs(a) < Math.atan2(4.8, 100) + 0.02;   // ±1.2m 宽的命中链
      distances.push(hit ? px(100) : MAX_SENSOR_DIST);
    }
    const scan: LidarScan = { angles, distances, rawDistances: [...distances], confidences: new Array(61).fill(1), timestamp: 0 };
    const stop = solve(scan, { speed: 0 });
    const creep = solve(scan, { speed: 32 });                   // 32u/s = 8m/s
    assert.strictEqual(stop.allBlockedHard, true, 'v=0（裕度 1.15m）应判死');
    assert.strictEqual(creep.allBlockedHard, false, '慢行档（裕度 0.91m）应留缝可过');
  });
});
```

- [ ] **Step 2: 运行确认失败**

Run: `npx tsc && node --test --test-name-pattern "裕度随速度变化" dist/js/test/plan/paths.test.js`
Expected: FAIL（当前两档都用几何余量 0.4m → 两边都 `allBlockedHard=false`）

- [ ] **Step 3: paths.ts 改三处**

```ts
// ① PathCandidate 增字段（JSDoc 注明：自适应缩短后的实际发展段，仿真单位）
export interface PathCandidate {
  // ...既有字段...
  /** 实际使用的发展段长度（自适应缩短后的值；供规划器反解横向机动可行速度） */
  develop: number;
}

// ② 导出横向机动速度上限（与 developMin 同一运动学式子的反解）
/** 横向机动可行速度上限：峰值横向加速度 QUINTIC_PEAK_ACCEL·|Δl|·v²/S² ≤ A_LAT_MAX
 *  → v ≤ √(A_LAT_MAX·S²/(QUINTIC_PEAK_ACCEL·|Δl|))（规划器用它把路径几何反馈成速度上限） */
export function lateralSpeedCap(dlU: number, developU: number): number {
  if (!(dlU > 1e-6)) return Infinity;
  return Math.sqrt((A_LAT_MAX as number) * developU * developU / (QUINTIC_PEAK_ACCEL * dlU));
}

// ③ 走廊族：物理走廊裕度改回标定表（迭代的几何评估速度 = max(v0, TIGHT_SPEED)）
const geomU = m(0.4) as number;   // 删除（几何余量不再用于判"能不能过"）
const marginU = marginFor(Math.max((o.v0 as number) / 4, TIGHT_SPEED as number)) * 4;
const comfortU = Math.max(m(1.5) as number, marginU);
const comfort = mkCorridor(comfortU);
const physical = mkCorridor(marginU);
const deep = mkCorridor(marginU + bandU);
const permissive = mkCorridor(Math.max(0, marginU - bandU));
```

并在候选构造里记录实际发展段：

```ts
    let usedDevelop = PATH_DEVELOP as number;
    let { points, evalLat, centerSum, curvSum } = buildPath(usedDevelop);
    const firstAny = firstExit(comfort, points, Infinity);
    if (firstAny - o.s0 < (PATH_DEVELOP as number)) {
      usedDevelop = Math.max(developMin,
        Math.min(PATH_DEVELOP as number, firstAny - o.s0 - (DEVELOP_MARGIN as number)));
      ({ points, evalLat, centerSum, curvSum } = buildPath(usedDevelop));
    }
    // ...
    candidates.push({
      targetL: px(targetL), points, blocked, sBlock, blockedHard, sBlockHard, cost,
      develop: usedDevelop,
      evalL: (s) => evalLat(Math.max(0, Math.min(s, PATH_HORIZON as number))),
    });
```

- [ ] **Step 4: 运行确认通过 + 全量回归**

Run: `npx tsc && node --test dist/js/test/`
Expected: 全绿（若既有 paths/planner 测试的期望值因裕度改回标定表而变化，按"实测值 + 行为语义"迁移断言，并在测试注释里写清新语义；不得改行为去迁就断言）

- [ ] **Step 5: 写失败测试（planner.test.ts：迭代行为）**

```ts
describe('规划器：收敛迭代（path-speed iteration）', () => {
  it('静止车 + 慢行可过的窄口 → 不自锁（tight 不封死、轨迹末速 > 0）', () => {
    const snap = makeSnapshot(makeCar(500, 0, 0, 0));                 // 静止
    const traj = plan(snap, lidarWithObstacle(0, 100, 9.6));          // 25m 处 2.4m 宽
    assert.notStrictEqual(debug.planning!.tight, 'SEALED', '不得因 v=0 裕度最大而封死');
    const last = traj.points[traj.points.length - 1];
    assert.ok((last.v as number) > 0, `末速应 > 0（计划起步），实际 ${last.v}`);
    assert.ok(Number.isFinite(debug.planning!.rounds) && debug.planning!.rounds >= 1);
  });

  it('高速 + 大横移需求 → 速度目标被横向机动上限压低', () => {
    const snap = makeSnapshot(makeCar(500, 0, 0, 140));               // 35 m/s
    const traj = plan(snap, lidarWithObstacle(0, 200, 30));           // 50m 处 7.5m 宽 → 需大横移
    assert.ok((debug.planning!.speedTarget as number) < m(20) as number,
      `速度目标应被横向上限压低，实际 ${(debug.planning!.speedTarget as number) / 4}m/s`);
    for (const pt of traj.points) assert.ok(Number.isFinite(pt.v as number));
  });

  it('确定性：同输入两次规划 → 轮数/目标/速度目标一致；轮数 ≤3', () => {
    const snap = makeSnapshot(makeCar(500, 0, 0, 120));
    const a = plan(snap, lidarWithObstacle(0, 200, 12));
    const ra = { r: debug.planning!.rounds, t: debug.planning!.bestTargetL, u: debug.planning!.speedTarget };
    const b = plan(snap, lidarWithObstacle(0, 200, 12));
    const rb = { r: debug.planning!.rounds, t: debug.planning!.bestTargetL, u: debug.planning!.speedTarget };
    assert.deepStrictEqual(ra, rb);
    assert.ok(ra.r >= 1 && ra.r <= 3, `轮数应在 1..3，实际 ${ra.r}`);
    assert.ok(Number.isFinite(a.points[0].v as number) && Number.isFinite(b.points[0].v as number));
  });

  it('不动点：以迭代末轮假设速度复算路径层 → 最优目标一致（同一解）', () => {
    const snap = makeSnapshot(makeCar(500, 0, 0, 120));
    plan(snap, lidarWithObstacle(0, 200, 12));
    const vAssume = debug.planning!.vAssume as number;
    const again = solvePathLayer({ /* 与 planner 相同的入参 */, v0: vAssume });
    assert.strictEqual(again.best.targetL, debug.planning!.bestTargetL);
  });

  it('性能烟测：30 帧平均单帧规划 <5ms（超时只记录，不改变行为）', () => {
    const snap = makeSnapshot(makeCar(500, 0, 0, 120));
    const t0 = Date.now();
    for (let i = 0; i < 30; i++) plan(snap, lidarWithObstacle(0, 200, 12));
    assert.ok((Date.now() - t0) / 30 < 5, `平均单帧规划耗时 ${(Date.now() - t0) / 30}ms`);
  });
});
```

- [ ] **Step 6: 运行确认失败**

Run: `npx tsc && node --test --test-name-pattern "收敛迭代" dist/js/test/plan/planner.test.js`
Expected: FAIL（`debug.planning.rounds` undefined / 末速 0 / 速度目标未压低）

- [ ] **Step 7: debug.ts 增字段**

```ts
  /** 路径-速度迭代求值次数（1..3；收敛迭代，spec 2026-09-25 §5） */
  rounds: number;
  /** 本帧 plan() 总耗时（ms；只记录，不参与决策） */
  planMs: number;
  /** 迭代末轮的速度假设（仿真单位/s；供不动点测试与 inspect 面板） */
  vAssume: number;
```

- [ ] **Step 8: planner.ts 实现迭代**

参数（§参数区）：

```ts
const MAX_ROUNDS = 3;                                  // 收敛迭代上限（spec §5.1）
const CONV_TARGET_L = m(0.2) as number;                // 横向目标收敛阈（0.2m）
const CONV_SPEED = mps(0.5) as number;                 // 速度上限收敛阈（0.5m/s）
```

辅助（模块级函数，放在 `plan()` 之前）：

```ts
/** 路径几何 → 速度上限（spec §5.1 的"速度上限 u_k"）：物理封死 → 0；紧间隙 → 慢行档；
 *  横向机动 → lateralSpeedCap（发展的横向加速度不超舒适上限）；否则巡航目标 */
function pathSpeedCap(pl: PathLayerResult, l0: number, vCap: number): number {
  if (pl.allBlockedHard) return 0;
  const lateral = lateralSpeedCap(Math.abs((pl.best.targetL as number) - l0), pl.best.develop);
  const tight = pl.best.blocked ? (TIGHT_SPEED as number) : Infinity;
  return Math.max(0, Math.min(vCap, lateral, tight));
}
```

§5–§6（替换原"单轮 solvePathLayer + best.blocked 限速"）：

```ts
  // ─── 5. 路径层 × 速度上限：收敛迭代（spec 2026-09-25 §5.1，EM 风格交替）───
  // 为什么迭代：路径几何（裕度 margin(v)、发展段 S(v)）依赖"通过时的速度"，而"能通过多快"
  // 又由路径几何决定——单轮顺序传递丢反馈：车停下时按 v=0 取几何（标定裕度最大 1.15m）→
  // 窄口判死 → 规划速度 0 → 下一帧依旧 v=0（自锁，批量实证 4 场景中途停车）。
  // 迭代把 v 当变量：初值 = max(当前速度, 慢行档)，几何评估速度下限取慢行档
  // （差异 ≤0.1m 裕度，换取不自锁；紧间隙通过一律限速 ≤TIGHT_SPEED）。
  const t0 = performance.now();
  const vCap = targetV;                                        // 巡航目标（含终点减速缩放）
  const vLo = TIGHT_SPEED as number;
  let vAssume = Math.min(Math.max((car.speed as number) * cosHD, vLo), Math.max(vCap, vLo));
  let pathLayer: PathLayerResult;
  let uCap = Infinity;
  let prevA: number | null = null, prevU: number | null = null, prevDelta = Infinity;
  let rounds = 0;
  for (; rounds < MAX_ROUNDS; rounds++) {
    pathLayer = solvePathLayer({
      ref: frenetRef, l0: fl.l, dl0, s0: fl.s, v0: vAssume,
      carX: car.x as number, carY: car.y as number, lidarScan,
      carWidth: car.width, collisionRadius: car.collisionRadius,
      laneOffset, prevTargetL: _prevBestTargetL,
      sEnd: routeActive ? (frenetRef.totalS as number) : undefined,
    });
    const a = pathLayer.best.targetL as number;
    uCap = pathSpeedCap(pathLayer, fl.l as number, vCap);
    const converged = prevA !== null
      && Math.abs(a - prevA) < CONV_TARGET_L
      && Math.abs(uCap - (prevU as number)) < CONV_SPEED;
    const diverging = prevA !== null && Math.abs(a - prevA) > prevDelta;   // 变化不降反增 → 防极限环
    if (converged || diverging) break;
    prevDelta = prevA === null ? Infinity : Math.abs(a - prevA);
    prevA = a; prevU = uCap;
    vAssume = Math.max(vLo, Math.min(uCap, vCap));               // 下一轮速度假设
  }
  // 规划速度目标：迭代末轮的几何上限（0 = 物理封死 → 交给停车线，不把 0 当巡航目标）
  targetV = Math.min(vCap, uCap > 0 ? uCap : vLo);
```

并在 `debug.planning` 末尾补：

```ts
    rounds, planMs: performance.now() - t0, vAssume,
```

并删除原 §5 之后的 `if (pathLayer.best.blocked) targetV = Math.min(targetV, TIGHT_SPEED ...)` 一行（已并入 `pathSpeedCap`）。

- [ ] **Step 9: 运行确认通过 + 全量回归 + 引擎黄金基线**

Run: `npx tsc && node --test dist/js/test/`
Expected: 全绿。**若 `SimEngine 路线模式` 黄金基线变化**：先人工核验（5s 沿赛道前进量、末速、无 NaN、状态 ACTIVE、`rounds` 分布），再按新实测值更新并在注释里写"2026-09-25 阶段 C 收敛迭代后重取"。
若 `planner-safety` 的"115m 全宽障碍 → stop"或"封死→畅通抖动"失败：检查迭代是否把 `uCap=0` 误当巡航目标（应仍由停车线剖面负责停车）。

- [ ] **Step 10: 汇报**（轮数分布、受影响测试与黄金基线；不提交）

---

### Task C2: 收敛指标进归档 / 汇总 / 报告

**Files:**
- Modify: `js/shared/types/interfaces/archive.ts`（`PlanRecord` 增可选字段）
- Modify: `js/sim/records.ts`（plan 记录映射）
- Modify: `js/headless/detector.ts`（累积）
- Modify: `js/headless/report.ts`（summary 字段 + report.md 行）
- Modify: `js/ui/inspector.ts`（面板显示）
- Test: `js/test/sim/records.test.ts`、`js/test/headless/report.test.ts`

**Interfaces:**
- Produces: `PlanRecord.rounds?: number; PlanRecord.planMs?: number;`、`RunSummary.multiRoundPct: number | null; RunSummary.planMsP50: number | null; RunSummary.planMsP95: number | null;`
- Consumes: Task C1 的 `debug.planning.rounds/planMs`（经 `records.ts` 的 debug 快照读取）。

- [ ] **Step 1: 写失败测试（records.test.ts）**

```ts
  it('planning:result → plan 记录带 rounds/planMs（缺省时 1/0）', () => {
    const r: any = recordFrom('planning:result', {
      t: 0, step: 7, state: 'CRUISE',
      planning: { frenetS: 0, frenetL: 0, dl_ds: 0, headingDiff: 0, paths: [], bestCost: 0,
        bestTargetL: 0, speedTarget: 0, speedMode: 'cruise', tight: 'NONE',
        rounds: 2, planMs: 1.5, remainingDist: 100 },
      trajectory: { points: [], type: 'lane_keep', cost: 0 },
    });
    assert.strictEqual(r.rounds, 2);
    assert.strictEqual(r.planMs, 1.5);
  });

  it('旧归档快照（无 rounds/planMs）→ 缺省 1/0，不抛异常', () => {
    const r: any = recordFrom('planning:result', {
      t: 0, step: 7, state: 'CRUISE',
      planning: { frenetS: 0, frenetL: 0, dl_ds: 0, headingDiff: 0, paths: [], bestCost: 0,
        bestTargetL: 0, speedTarget: 0, speedMode: 'cruise', remainingDist: 100 },
      trajectory: { points: [], type: 'lane_keep', cost: 0 },
    });
    assert.strictEqual(r.rounds, 1);
    assert.strictEqual(r.planMs, 0);
  });
```

- [ ] **Step 2: 运行确认失败**

Run: `npx tsc && node --test --test-name-pattern "rounds/planMs" dist/js/test/sim/records.test.js`
Expected: FAIL（字段 undefined）

- [ ] **Step 3: 实现映射与类型**

`archive.ts`：

```ts
export interface PlanRecord extends BaseRecord {
  // ...既有字段...
  /** 路径-速度收敛迭代求值次数（1..3；旧归档无此字段） */
  rounds?: number;
  /** 单帧规划耗时（ms；旧归档无此字段） */
  planMs?: number;
}
```

`records.ts`（plan 分支的 return 里补两个字段）：

```ts
      return { type: 'plan', t: p.t, step: p.step, state: p.state,
        rounds: pl?.rounds ?? 1, planMs: pl?.planMs ?? 0,
        // ...既有字段...
```

- [ ] **Step 4: 运行确认通过**

Run: `npx tsc && node --test --test-name-pattern "rounds/planMs" dist/js/test/sim/records.test.js`
Expected: PASS

- [ ] **Step 5: 写失败测试（report.test.ts：聚合）**

```ts
describe('summary: 收敛迭代指标', () => {
  it('多轮帧占比与 planMs 分位（含旧记录缺省）', () => {
    const recs = [
      { type: 'plan', t: 0, step: 0, state: 'CRUISE', frenet: { s: 0 }, candidates: [], best: {}, trajectory: { points: [], type: 'x', cost: 0 }, rounds: 1, planMs: 1 },
      { type: 'plan', t: 1, step: 1, state: 'CRUISE', frenet: { s: 0 }, candidates: [], best: {}, trajectory: { points: [], type: 'x', cost: 0 }, rounds: 2, planMs: 3 },
      { type: 'plan', t: 2, step: 2, state: 'CRUISE', frenet: { s: 0 }, candidates: [], best: {}, trajectory: { points: [], type: 'x', cost: 0 }, rounds: 3, planMs: 5 },
      // 旧记录：无 rounds/planMs（按 1/0 计）
      { type: 'plan', t: 3, step: 3, state: 'CRUISE', frenet: { s: 0 }, candidates: [], best: {}, trajectory: { points: [], type: 'x', cost: 0 } },
    ] as any[];
    const det = new Detector(worldStub());          // 复用同文件既有 stub 构造
    for (const r of recs) det.observe(r);           // 与既有测试相同的入口
    const s = buildSummary(det, { steps: 4, simTime: 0.2 });
    assert.strictEqual(s.multiRoundPct, 50);        // 2/4 帧需 >1 轮
    assert.ok(s.planMsP50 !== null && s.planMsP95 !== null && s.planMsP95 >= s.planMsP50);
  });
});
```

（若 `report.test.ts` 现有 stub 入口名不同，按该文件既有写法调整调用方式，断言不变。）

- [ ] **Step 6: 运行确认失败** → **Step 7: 实现 detector + summary + report.md 行**

`detector.ts`：字段 `planCount`/`multiRoundPlans`/`planMsAll: number[]`；`_accumulate` 的 plan 分支：

```ts
      this.planCount += 1;
      if ((rec.rounds ?? 1) > 1) this.multiRoundPlans += 1;
      if (typeof rec.planMs === 'number') this.planMsAll.push(rec.planMs);
```

`report.ts` 的 `buildSummary`：

```ts
    multiRoundPct: det.planCount > 0 ? (100 * det.multiRoundPlans) / det.planCount : null,
    planMsP50: det.planMsAll.length ? quantile(det.planMsAll, 0.5) : null,
    planMsP95: det.planMsAll.length ? quantile(det.planMsAll, 0.95) : null,
```

（`quantile` 为 `report.ts` 内的本地排序取值函数；`RunSummary` 增这三个可空字段。）

report.md 关键指标表补两行：

```ts
  lines.push(`| 收敛轮数 >1 的帧 | ${summary.multiRoundPct != null ? `${summary.multiRoundPct.toFixed(0)}%` : 'n/a'} |`);
  lines.push(`| 单帧规划耗时 p50/p95 | ${summary.planMsP50 != null ? `${summary.planMsP50.toFixed(2)} / ${summary.planMsP95!.toFixed(2)} ms` : 'n/a'} |`);
```

`inspector.ts`：在"紧间隙"行后补 `html += \`  迭代: ${pl.rounds} 轮 ${pl.planMs.toFixed(1)}ms v假设=${(pl.vAssume/4).toFixed(1)}m/s\n\`;`

- [ ] **Step 8: 全量回归 + 一次 headless 运行核对归档**

Run: `npx tsc && node --test dist/js/test/ && npm run sim -- --scenario scenarios/batch/oval-s6.json --out runs/ctmp-check`
Expected: 全绿；`runs/ctmp-check/summary.json` 含三个新字段且 `report.md` 有两行新指标；`grep -c '"rounds"' runs/ctmp-check/frames.jsonl` > 0

- [ ] **Step 9: 汇报**（指标样例；不提交）

---

### Task C3: 批量验收与能力边界协议

**Files:**
- 产出：`docs/superpowers/plans/2026-09-25-corridor-stage-c.md` 的"执行记录"段（本文件内追加）
- 可能修改：`js/algo/plan/*.ts`（仅当门禁失败且原因可定位到具体机制时，按 Task 形式追加，不在本任务里猜测调参）

- [ ] **Step 1: 批量运行（种子 6-10，10 场景）**

```bash
for s in oval-s6 oval-s7 oval-s8 oval-s9 oval-s10 hairpin-s6 hairpin-s7 hairpin-s8 hairpin-s9 hairpin-s10; do
  node dist/js/headless/run.js --scenario scenarios/batch/$s.json > /dev/null 2>&1
done
```

- [ ] **Step 2: 聚合（碰撞段/帧、到达、用时、进度、收敛率、耗时）**

```bash
for d in $(ls -dt runs/*batch* | head -10); do node -e "
const s=require('./$d/summary.json');const j=require('./$d/issues.json');
const c=j.filter(i=>i.check==='collision_true');
console.log(('$d').slice(21).padEnd(22), c.reduce((a,x)=>a+x.count,0)+'f/'+c.length+'ep',
  '到达 '+(s.arrivalTimeS!=null?s.arrivalTimeS.toFixed(1)+'s':'—').padEnd(6),
  '进度 '+((s.progress??0)*100).toFixed(0)+'%',
  '多轮 '+(s.multiRoundPct??0).toFixed(0)+'%',
  'planP95 '+(s.planMsP95??0).toFixed(1)+'ms');
"; done
```

- [ ] **Step 3: 判定（三选一，写进"执行记录"）**
  - **(a) 门禁全过**（0 段/0 帧；≥8/10 到达；用时 ≤ 基线 +0.5s/场景；进度 oval-s8 ≥60%、hairpin-s8 ≥76%）：进入 C4。
  - **(b) 0/0 但到达率不足**：按 spec §5.3 记为**能力边界**——逐场景记录"哪个窄口的车身余量 < 标定预算（0.6~0.95m vs 0.9~1.15m）"的实测证据，并把"候选准入门槛/轨迹形状稳定"列为后续工作。**不得为了到达率把裕度调小**（那是用安全换通过率，且 B 的 ② 号配置已实证会接触）。
  - **(c) 出现接触**：按 superpowers:systematic-debugging 定位（是迭代把不该过的判成可过？还是执行误差吃满余量？）——若是前者，检查该场景的 `margin(vAssume)` 与实际通过速度是否一致（几何评估速度下限的 0.1m 误差是否被放大）。

- [ ] **Step 4: 浏览器抽查**（`npm run serve` → 加载 hairpin-s7 窄口 + oval-s9 簇，目检轨迹是否居中过缝、inspect 面板的迭代轮数/`v假设`）

- [ ] **Step 5: 汇报**（A/B 表 + 判定 + 证据；不提交）

---

### Task C4: 文档与执行记录

**Files:**
- Modify: `CLAUDE.md`（规划器章节：收敛迭代一段 + 新增指标）
- Modify: `docs/domain.md`（"窄口余量 vs 可执行预算"的能力边界条目；若 C3 判定 (a) 则写"迭代后可达"的结论）
- Modify: `docs/superpowers/specs/2026-09-25-corridor-planner-evolution-design.md`（附录：阶段 C 执行记录）
- Modify: 本文件（"执行记录"段）
- Modify: `ISSUE_HISTORY.md`（一行台账）

- [ ] **Step 1: 按 C3 判定写文档**（(a) 交付段 / (b) 能力边界段），每处都带实测数字（碰撞段/帧、到达率、用时、进度、多轮占比、planMs）。
- [ ] **Step 2: 更新 CLAUDE.md 规划器章节**（"每帧流程"改为 收敛迭代 ≤3 轮；指标表补 收敛率/耗时）。
- [ ] **Step 3: 汇报**（文档改动清单；不提交）

---

## Self-Review

- **Spec 覆盖**：§5.1 交替结构 → C1 Step 8（含层间传递量：路径→速度 = 停止线/几何/紧间隙；速度→路径 = vAssume）；§5.2 性能与确定性 → C1 Step 5（性能烟测）、Step 7（`planMs` 只记录）、Step 5 确定性用例、Step 8 防极限环；§5.3 验收 → C3（含能力边界条款 (b)）与 C2（"需 >1 轮的帧占比"+"单帧规划耗时"进归档）；§5.3 hairpin-s8 类窄口 → C3 Step 3 的判定与 domain.md 记录。
- **占位符扫描**：无 TBD/TODO；所有断言与阈值都给数值（0.2m/0.5m/3 轮/5ms/0.6~0.95m/0.9~1.15m）。
- **类型一致性**：`lateralSpeedCap(dlU, developU)` 在 C1 Step 3 定义、Step 8 使用；`PathCandidate.develop` 定义与使用一致；`rounds`/`planMs`/`vAssume` 在 debug→records→archive 三处同名；`multiRoundPct`/`planMsP50`/`planMsP95` 在 detector/summary/report 三处同名。
- **Review Focus 对应测试**：①→C1 Step 5 用例 1；②→C1 Step 5 用例 3（轮数 ≤3）+ Step 8 的 `diverging` 分支；③→C1 Step 9（planner-safety stop 用例必须仍绿）；④→C1 Step 4/9 全量回归（含 sEnd 截断路径）；⑤→C2 Step 1 用例 2（旧记录缺省）。
- **已知风险**：窄口（0.6~0.95m 余量）在标定预算下**本就不可安全通过**（前置事实 3）；因此 C3 判定 (b) 是**预期内的诚实结局**，不是实现失败——此时不得回头调小裕度，而是记录边界并汇报。

---

## 执行记录（2026-09-25，阶段 C 首轮）

| 任务 | 状态 | 说明 |
|---|---|---|
| C1 收敛迭代核心 | ✅ | `planner.ts` 迭代（≤3 轮，阈 0.2m/0.5m·s⁻¹，防极限环）；`paths.ts` 新增 `lateralSpeedCap` + `PathCandidate.develop`；物理走廊裕度恢复标定表 `marginFor(max(v, TIGHT_SPEED))`；可达性闸门改用**种子档几何**（当前速度档）+ 迭代目标速度 |
| C2 指标 | ✅ | `rounds`（确定量）进逐帧归档；`planMs`（墙钟）走 SimEngine 运行期统计 → `summary.json`/`report.md`（**不进逐帧归档**：与"同种子逐字节一致"契约冲突，E2E 立即失败） |
| C3 批量验收 | ⚠️ 判定 (b) 能力边界 | 0 段/0 帧碰撞 ✓、无紧急制动 ✓、**到达 4/10**（基线 8/10）→ 按 spec §5.3 记录为能力边界，不调小裕度（数字为**评审修复后复跑**，见下） |
| C4 文档 | ✅ | 见下（CLAUDE.md / domain.md / spec 附录 / ISSUE_HISTORY） |

### 评审修复（全分支评审发现 1 Critical + 2 Important，均已 TDD 修复）

1. **Critical：物理走廊裕度单位混用 → 标定表失效**（`paths.ts`）：`marginFor(Math.max(v0/4, TIGHT_SPEED))`
   里 `TIGHT_SPEED = mps(8) = 32`（仿真单位）被当成 m/s 传给 `marginFor` → 裕度恒为表尾 **0.615m**
   （8m/s 慢行档应为 0.91m）。修：`(TIGHT_SPEED as number) / 4`；并暴露 `PathLayerResult.marginU`
   + 回归测试钉住（静止车取慢行档、35m/s 取表值）。**上面的批量为修复后复跑**。
2. **Important：可达性闸门不完整**（`planner.ts`）：原用"种子档几何 + 末轮 targetV"，若种子档最优
   畅通、末轮（更低假设速度 ⇒ 更长发展段）才暴露阻塞，则闸门整段跳过。修：取种子档 ∪ 末轮的更早阻塞点。
3. **Important：`rounds` 差 1 / 指标语义**：计数器停在"求值次数−1"。修：显式计入每次求值
   （`rounds ∈ {2,3}`），并新增 **`revised`**（种子假设被路径几何压低 = 确实需要迭代修正）
   ——多轮指标改用 `revised`，报告标签改为"需迭代修正的帧"。
4. Minor：陈旧注释（paths/planner 的"单轮""几何余量 0.4m"）已同步；标定报告的硬编码小结待下次
   `--write` 时修正（本轮手写第二节已在文档里，勿覆盖）。

### 批量对照（种子 6-10，Critical 修复后复跑）

| 场景 | 基线 | 阶段 C（修复后） | 差距 |
|---|---|---|---|
| oval-s6 | 12.3s | **12.5s** | +0.2s ✓ |
| oval-s7 | 19.4s | 停在 141m（进度 52%） | 窄口拒绝 |
| oval-s8 | 60% | 58% | 同为"停车存档"类 |
| oval-s9 | 17.1s | 停在 160m（59%） | 窄口拒绝 |
| oval-s10 | 14.7s | **15.0s** | +0.3s ✓ |
| hairpin-s6 | 11.1s | **11.6s** | +0.5s ✓ |
| hairpin-s7 | 15.6s | 停在 138m（55%） | 窄口 + 死口袋（横向承诺） |
| hairpin-s8 | 76% | 71% | 停车存档类 |
| hairpin-s9 | 15.8s | 停在 155m（59%） | 窄口拒绝 |
| hairpin-s10 | 12.9s | **13.4s** | +0.5s ✓ |

- 碰撞 **0 段/0 帧**（10/10 场景）、**紧急制动 0 次**、6 个非到达场景末态全为 `DECEL` + v=0（**干净停车**，不是接触/兜底）。
- 迭代指标：`需迭代修正的帧 18~28%`（正常巡航/避让，主要是横向偏移未收敛的瞬态）到 **`83~93%`（窄口场景：几何确实要求降速）**；`planMs p50 0.8ms / p95 ≤4.0ms`（<5ms 预算 ✓；首帧 JIT 尖峰 ~30ms 已如实记录）。
- 能力边界证据（hairpin-s7 @138m 实测）：3.1×4.9m 障碍居中（8.8m 车道），两侧车身余量 ≈ 0.6~0.95m；标定 p95 跟踪误差在慢行档 = 0.91m → 物理走廊判"过不去" → 干净停车。**该缝隙按实测误差分布本就不该过**（B 的 ②/③ 号配置正是在同一批缝隙上以 0.4m 几何余量通过并发生接触）。
- 次要成因：**横向承诺/死口袋**（oval-s7、hairpin-s7 末态 l≈±4.0m——车已在角落，下一障碍只留另一侧缝隙，2~3m 内无法横跨）→ 属"晚承诺 + 横向换边"议题，非裕度问题。

### 结论与后续（待用户决策）

阶段 C 把"到达率"换成了"零接触 + 零紧急制动 + 干净停车"：安全门禁全过，到达率与进度不达基线。
按 spec §5.3，这属**能力边界**（本场景集的窄口超出当前执行误差预算），不通过调小裕度解决。可选后续：
1. 候选准入/横向承诺（不死口袋）——降低"停在不该停的地方"的比例；
2. 轨迹形状稳定化（降低 p95 误差本身，从而收紧预算）；
3. 场景集校准（障碍宽度/缝隙宽度与车宽的关系，使批量更贴近"可通行"设计意图）。


### 追加复核（2026-09-25，用户提问 hairpin-s9 为何停车）

逐帧复现（重放归档扫描调 `plan()`）+ 走廊剖面：**归因需分三类**，不是单一"窄口太小"：

| 场景 | 停滞点 allBlockedHard | 窗口最窄自由宽 | 机制 |
|---|---|---|---|
| oval-s7 | true | 0.26m @6m | 合法带极窄（能力边界） |
| oval-s9 | true | 0.02m @15m | 合法带极窄（能力边界） |
| hairpin-s9 | true | 1.35m @30m | **候选锚定单一瓶颈**：30m 瓶颈只留左侧 1.35m、72m 另一障碍只留右侧 4.0m，36~66m 整幅自由；候选全在左侧 → 三条路径均在 69m 撞第二障碍 |
| hairpin-s8 | true | 3.02m @3m | 同上（待逐帧确认） |
| oval-s8 | true | 1.90m @18m | 同上（待逐帧确认） |
| hairpin-s7 | **false** | 1.84m @15m | 路径层判"可过"却停车 → 封死记忆/闸门交互，待钻取 |

**结论**：单 pinch 派生候选 + 单目标五次多项式**表达不了"先左后右"的双瓶颈过渡**（hairpin-s9
的"又长又宽的缝"正是那段过渡区）。修的方向是**候选跨多个瓶颈**（走廊内连续搜索/DP，即路线图 A2），
而不是继续调裕度；能力边界只覆盖第 1 类。
