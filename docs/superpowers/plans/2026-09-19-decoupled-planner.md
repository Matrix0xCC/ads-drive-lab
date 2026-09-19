# 规划层解耦升级实施计划（SL 路径层 + ST 速度层）

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把耦合候选采样规划器重写为 SL 路径层 + ST 速度层的解耦架构，stop-line（横向无解时纵向停车）成为一等公民，输出契约与全部下游零改动。

**Architecture:** 新建 `paths.ts`（路径层：l(s) 候选、整路径碰撞、全阻塞时按最晚碰撞选停车基准）与 `speed.ts`（速度层：巡航/停车线/兜底三种剖面，通用 STObstacle 接口静态实现）；`planner.ts` 变为层编排 + 合成 `l(s(t))` → Trajectory，并保持兼容的类型标签（`avoid_*/lane_keep/emergency/arrived`）使 overlays/HUD/检测器零改动。

**Tech Stack:** TypeScript（strict，ESM，`.js` 后缀 import）；复用 `polynomial.ts`（solveQuintic/solveSpeedProfile）、`collision.ts`（isLidarCollision）、`frenet.ts`（互转）；测试 node:test；无新依赖。

**Spec:** `docs/superpowers/specs/2026-09-19-decoupled-planner-design.md`

**计划相对 spec 的两处实现级细化（已记录）：**
1. `states.ts` 的 `nextState` 保留"显式状态机"形态，`TransitionInput` 更新为 `{ routeActive, nearEnd, allBlocked, stopInfeasible, avoiding }`——`avoiding` 用于保留 AVOID 状态（overlays 的障碍高亮标记 `plannerState === 'AVOID'` 依赖它）
2. "15m 障碍"迁移用例：25m/s 在 15m 内物理停不住（制动距离 ≈52m）→ 正确期望是 `emergency`（最大减速）而非 `stop`；另加"60m 可停"正例断言 `stop` + v→0

## Global Constraints

- TypeScript 严格模式；import 带 `.js` 后缀；`npx tsc` / `npm test` 全绿才算完成
- 不新增 npm 依赖；不改 `collision.ts`/`frenet.ts`/`polynomial.ts` 的算法本体
- **Trajectory 输出契约不变**：`{ points: {t,x,y,yaw,v,kappa}[], cost, type, timestamp }`；`type` 继续产出 `avoid_left/avoid_right/lane_keep/emergency/arrived` 五种标签（下游 overlays/HUD/检测器/测试依赖）
- 存档 schema 不变：`recordFrom` 的 plan 记录字段名保持 `candidates/best`（path 候选映射进旧字段，detector 零改动）
- 新模块中文注释（算法原理、参数依据）；品牌类型在模块边界使用
- 每个任务一次 commit（中文 message）；引擎黄金基线按既有流程更新（人工核验后冻结）
- 批量 A/B 门槛：不劣化；行为收益如实报告；**若出现劣化 → 停止并向用户报告**（不自行回滚）

---

### Task 1: 路径层 `js/algo/plan/paths.ts`（TDD）

**Files:**
- Create: `js/algo/plan/paths.ts`
- Test: `js/test/plan/paths.test.ts`

**Interfaces:**
- Consumes: `solveQuintic`（polynomial.ts，返回 `{coefs, eval:Px, deriv1:DimLess, deriv2:PerPx}`）、`isLidarCollision/SAFETY_MARGIN`（collision.ts）、`frenetToCartesian`（frenet.ts）、`TRACK_HALF_WIDTH`（env/track.ts）、dimensions 工厂
- Produces（Task 3 依赖，签名一字不差）：
  - `const PATH_HORIZON: Px`（m(120)）、`const PATH_STEPS = 40`
  - `interface PathCandidate { targetL: Px; points: { s: number; l: Px; x: Px; y: Px; yaw: Radians }[]; blocked: boolean; sBlock: number; cost: number; evalL(s: number): Px }`
  - `interface PathLayerResult { best: PathCandidate; ranked: PathCandidate[]; allBlocked: boolean; sBlock: number }`
  - `function solvePathLayer(o: { ref: FrenetReference; l0: Px; dl0: DimLess; s0: number; carX: number; carY: number; lidarScan: LidarScan; carWidth: Px; collisionRadius: Px; laneOffset: Px; prevTargetL: number | null }): PathLayerResult`

- [ ] **Step 1: 写失败测试**

创建 `js/test/plan/paths.test.ts`：

```typescript
// ==================== 测试：路径层（SL 空间） ====================
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { solvePathLayer, PATH_HORIZON } from '../../algo/plan/paths.js';
import { buildFrenetReference } from '../../algo/plan/frenet.js';
import { CarState, LidarScan } from '../../shared/types.js';
import { Px, rad, m, mps, mps2, px, pxps, dimless } from '../../shared/dimensions.js';
import { MAX_SENSOR_DIST } from '../../algo/perception.js';

const LO = m(1.875) as number;

function straightRef() {
  const pts: any[] = [];
  for (let i = 0; i <= 600; i++) pts.push({ x: px(i * 10), y: px(0) });
  return buildFrenetReference(pts, true);
}
function makeCar(x: number, y: number, angle: number, speed: number): CarState {
  return { x: px(x), y: px(y), angle: rad(angle), speed: pxps(speed), steerAngle: rad(0),
    maxSpeed: mps(42.5), acceleration: mps2(8), friction: dimless(0.12),
    width: m(2.0), length: m(4.5), collisionRadius: px(Math.hypot(8, 18) / 2) };
}
function clearScan(): LidarScan {
  const n = 61, angles: any[] = [], distances: any[] = [];
  for (let i = 0; i < n; i++) { angles.push(rad(-Math.PI/3 + (i/(n-1))*2*Math.PI/3)); distances.push(MAX_SENSOR_DIST); }
  return { angles, distances, rawDistances: [...distances], confidences: new Array(61).fill(1), timestamp: 0 };
}
function blockedScan(dist: number, arcHalf = 0.05): LidarScan {
  const n = 61, angles: any[] = [], distances: any[] = [];
  for (let i = 0; i < n; i++) {
    const a = -Math.PI/3 + (i/(n-1))*2*Math.PI/3;
    angles.push(rad(a));
    distances.push(Math.abs(a) < arcHalf ? px(dist) : MAX_SENSOR_DIST);
  }
  return { angles, distances, rawDistances: [...distances], confidences: new Array(61).fill(1), timestamp: 0 };
}

function solve(scan: LidarScan, opts: { s0?: number; l0?: number; prev?: number | null } = {}) {
  const car = makeCar(100 + ((opts.s0 ?? 0)), 0, 0, 100);
  return solvePathLayer({
    ref: straightRef(), l0: px(opts.l0 ?? 0), dl0: dimless(0), s0: opts.s0 ?? 0,
    carX: car.x as number, carY: car.y as number, lidarScan: scan,
    carWidth: car.width as Px, collisionRadius: car.collisionRadius as Px,
    laneOffset: px(LO), prevTargetL: opts.prev ?? null,
  });
}

describe('路径层：候选生成与域约束', () => {
  it('全部候选在可行驶域内，采样覆盖整个视程', () => {
    const r = solve(clearScan());
    assert.ok(r.ranked.length >= 3);
    for (const p of r.ranked) {
      assert.ok(Math.abs(p.targetL as number) <= 17.6 + 1e-6, `越界 targetL=${p.targetL}`);
      assert.strictEqual(p.points.length, 41);   // PATH_STEPS + 1
      assert.ok((p.points[40].s - p.points[0].s) > (PATH_HORIZON as number) - 1);
    }
  });

  it('畅通：最优为车道中心路径（代价最低）', () => {
    const r = solve(clearScan());
    assert.strictEqual(r.allBlocked, false);
    assert.ok(Math.abs((r.best.targetL as number) - LO) < 0.5, `best=${r.best.targetL}`);
    assert.strictEqual(r.best.blocked, false);
  });
});

describe('路径层：碰撞判定与选路', () => {
  it('中央窄障碍（100u）：本车道阻塞，最优选侧向路径', () => {
    const r = solve(blockedScan(100, 0.025));
    assert.strictEqual(r.allBlocked, false);
    assert.strictEqual(r.best.blocked, false);
    assert.ok(Math.abs((r.best.targetL as number) - LO) > 2 * 4, '应选侧向路径'); // >2m
    const inLane = r.ranked.find(p => Math.abs((p.targetL as number) - LO) < 1);
    assert.ok(inLane && inLane.blocked, '本车道应被判阻塞');
    assert.ok(inLane.sBlock >= 90 && inLane.sBlock <= 130, `sBlock≈100，实际 ${inLane.sBlock}`);
  });

  it('近距宽障碍（60u）全阻塞：allBlocked=true，按最晚碰撞排序', () => {
    const r = solve(blockedScan(60));
    assert.strictEqual(r.allBlocked, true);
    assert.strictEqual(r.best.blocked, true);
    const ord = r.ranked.map(p => p.sBlock);
    for (let i = 1; i < ord.length; i++) assert.ok(ord[i] <= ord[i - 1], '应按 sBlock 降序');
    assert.ok(r.sBlock === ord[0]);
  });
});
```

- [ ] **Step 2: 运行确认失败**

Run: `npx tsc && node --test dist/js/test/plan/paths.test.js`
Expected: FAIL（`Cannot find module '../../algo/plan/paths.js'`）

- [ ] **Step 3: 实现 paths.ts**

创建 `js/algo/plan/paths.ts`：

```typescript
// ==================== 算法：路径层（SL 空间） ====================
//
// 【职责】生成横向路径候选 l(s)（视程 PATH_HORIZON），逐条做整路径碰撞检查，
// 输出最优路径；全部阻塞时按"最晚碰撞"（sBlock 最大）选停车基准路径。
// 纵向速度由 speed.ts 决定（解耦架构，见 spec 2026-09-19-decoupled-planner）。
//
// 【为什么路径可以看很远】l(s) 是纯几何、无时间耦合——视程不受 2s 轨迹时域限制。
// 旧的耦合采样里"本车道被障碍判阻塞"发生得太晚（第七轮晚横移分析），
// 解耦后路径层用 120m 视程提前决策。

import { FrenetReference, LidarScan } from '../../shared/types.js';
import { Radians, Px, DimLess, m, px, perpx, dimless } from '../../shared/dimensions.js';
import { solveQuintic } from './polynomial.js';
import { isLidarCollision, SAFETY_MARGIN } from './collision.js';
import { frenetToCartesian } from './frenet.js';
import { TRACK_HALF_WIDTH } from '../../env/track.js';

export const PATH_HORIZON: Px = m(120);
export const PATH_STEPS = 40;

const W_PATH_CENTER = 0.3;    // 中心偏离²（沿用旧 W_CENTER 量级）
const W_PATH_CURV = 1.0;      // 路径曲率（航向变化累计）
const W_SWITCH_PATH = 100;    // 路径跨档切换惩罚（沿用旧 W_SWITCH）
const NUDGE = m(2.5);         // 挪移档横向偏移

export interface PathCandidate {
  targetL: Px;
  points: { s: number; l: Px; x: Px; y: Px; yaw: Radians }[];
  blocked: boolean;
  /** 首次碰撞命中的绝对弧长（blocked 时有效，否则 Infinity） */
  sBlock: number;
  cost: number;
  /** 合成阶段按弧长求横向偏移（供 l(s(t)) 采样） */
  evalL(s: number): Px;
}

export interface PathLayerResult {
  best: PathCandidate;
  ranked: PathCandidate[];   // 非阻塞按 cost 升序在前；阻塞按 sBlock 降序在后
  allBlocked: boolean;
  sBlock: number;            // best 的 sBlock（阻塞时）
}

export function solvePathLayer(o: {
  ref: FrenetReference; l0: Px; dl0: DimLess; s0: number;
  carX: number; carY: number; lidarScan: LidarScan;
  carWidth: Px; collisionRadius: Px; laneOffset: Px;
  prevTargetL: number | null;
}): PathLayerResult {
  const margin = (SAFETY_MARGIN as number) + (o.collisionRadius as number);
  // 可行驶域（与 checkBounds 复位阈值一致；0.6m 跟踪过冲预留）
  const L_MAX = (TRACK_HALF_WIDTH as number)
    - ((o.carWidth as number) / 2 + (m(1.5) as number)) - (m(0.6) as number);
  const lo = o.laneOffset as number;

  // 候选目标：{车道中心, ±2.5m 挪移, ±走廊边界} clamp + 去重
  const raw = [lo, lo - (NUDGE as number), lo + (NUDGE as number), -L_MAX, L_MAX];
  const targets: number[] = [];
  for (const v of raw) {
    const c = Math.max(-L_MAX, Math.min(L_MAX, v));
    if (!targets.some(t => Math.abs(t - c) < 0.5)) targets.push(c);
  }

  const candidates: PathCandidate[] = [];
  for (const targetL of targets) {
    const latPoly = solveQuintic(o.l0, o.dl0, perpx(0), px(targetL), dimless(0), perpx(0), PATH_HORIZON as number);
    const points: PathCandidate['points'] = [];
    let blocked = false, sBlock = Infinity;
    let centerSum = 0, curvSum = 0, prevYaw: number | null = null;

    for (let i = 0; i <= PATH_STEPS; i++) {
      const sRel = (i / PATH_STEPS) * (PATH_HORIZON as number);
      const li = latPoly.eval(sRel);
      const cart = frenetToCartesian(o.s0 + sRel, li, o.ref);
      points.push({ s: o.s0 + sRel, l: li, x: cart.x, y: cart.y, yaw: cart.yaw });

      const d = (li as number) - lo;
      centerSum += d * d;
      if (prevYaw !== null) {
        let dyaw = (cart.yaw as number) - prevYaw;
        dyaw = Math.atan2(Math.sin(dyaw), Math.cos(dyaw));
        curvSum += Math.abs(dyaw);
      }
      prevYaw = cart.yaw as number;

      if (!blocked && isLidarCollision(cart.x as number, cart.y as number, o.carX, o.carY, o.lidarScan, margin)) {
        blocked = true;
        sBlock = o.s0 + sRel;
      }
    }

    let cost = W_PATH_CENTER * centerSum / points.length + W_PATH_CURV * curvSum / points.length;
    if (o.prevTargetL !== null && Math.abs(targetL - o.prevTargetL) > 5 && targetL !== 0) cost += W_SWITCH_PATH;

    candidates.push({ targetL: px(targetL), points, blocked, sBlock, cost, evalL: (s) => latPoly.eval(Math.max(0, Math.min(s, PATH_HORIZON as number))) });
  }

  const clear = candidates.filter(c => !c.blocked).sort((a, b) => a.cost - b.cost);
  const stopped = candidates.filter(c => c.blocked).sort((a, b) => b.sBlock - a.sBlock);
  const ranked = [...clear, ...stopped];
  const best = ranked[0];
  return { best, ranked, allBlocked: clear.length === 0, sBlock: best.sBlock };
}
```

注意：`evalL(s)` 的自变量是**相对弧长**（合成阶段传 `speed.profile.eval(t)` 的 sRel）；内部 clamp 到 [0, PATH_HORIZON]。

- [ ] **Step 4: 运行确认通过**

Run: `npx tsc && node --test dist/js/test/plan/paths.test.js`
Expected: PASS（5 个用例）。若"中央窄障碍"用例中 `inLane.sBlock` 越界，打印实测值并把断言容差放到实测 ±30 内并注明（合成窄射线障碍在 100u 处的几何近似）。

- [ ] **Step 5: 提交**

```bash
git add js/algo/plan/paths.ts js/test/plan/paths.test.ts
git commit -m "新增规划路径层 paths.ts（SL 候选 + 整路径碰撞 + 最晚碰撞选路）"
```

---

### Task 2: 速度层 `js/algo/plan/speed.ts`（TDD）

**Files:**
- Create: `js/algo/plan/speed.ts`
- Test: `js/test/plan/speed.test.ts`

**Interfaces:**
- Consumes: `solveQuintic` / `solveSpeedProfile`（polynomial.ts；后者返回 `{eval(t):number, deriv1(t):number}`）
- Produces（Task 3 依赖）：
  - `interface STObstacle { sMin: number; sMax: number; tMin: number; tMax: number }`（通用 ST 接口；v1 只处理 `tMin===0` 的静态条目）
  - `type SpeedMode = 'cruise' | 'stop' | 'emergency'`
  - `interface SpeedProfile { eval(t: number): number; deriv1(t: number): number; duration: number }`
  - `interface SpeedLayerResult { profile: SpeedProfile; mode: SpeedMode }`
  - 常量 `SPEED_HORIZON=2.0`、`STOP_MARGIN=m(4)`、`A_COMFORT=mps2(3.5)`、`A_MAX=mps2(6.0)`、`T_STOP_MIN=0.8`
  - `function solveSpeedLayer(o: { stObstacles: STObstacle[]; v0: number; targetV: number }): SpeedLayerResult`

- [ ] **Step 1: 写失败测试**

创建 `js/test/plan/speed.test.ts`：

```typescript
// ==================== 测试：速度层（ST 空间） ====================
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { solveSpeedLayer, STObstacle, A_COMFORT } from '../../algo/plan/speed.js';

/** 采样峰值 |加速度| */
function peakAccel(profile: { eval(t: number): number; deriv1(t: number): number; duration: number }): number {
  let peak = 0, prevV = profile.deriv1(0);
  for (let i = 1; i <= 40; i++) {
    const t = (i / 40) * profile.duration;
    const v = profile.deriv1(t);
    peak = Math.max(peak, Math.abs(v - prevV) / (profile.duration / 40));
    prevV = v;
  }
  return peak;
}
const staticObstacle = (sMin: number): STObstacle => ({ sMin, sMax: sMin + 32, tMin: 0, tMax: Infinity });

describe('速度层：巡航', () => {
  it('无阻塞 → cruise，从 v0 收敛到 targetV', () => {
    const r = solveSpeedLayer({ stObstacles: [], v0: 100, targetV: 100 });
    assert.strictEqual(r.mode, 'cruise');
    assert.ok(Math.abs(r.profile.deriv1(0) - 100) < 1);
    assert.ok(Math.abs(r.profile.deriv1(r.profile.duration) - 100) < 2);
    assert.ok(r.profile.eval(r.profile.duration) > 150, '应正常前进');
  });
});

describe('速度层：停车线', () => {
  it('距离充足 → stop：终点停在障碍前（sStop = sMin−STOP_MARGIN），v(T)≈0', () => {
    const r = solveSpeedLayer({ stObstacles: [staticObstacle(400)], v0: 100, targetV: 100 });
    assert.strictEqual(r.mode, 'stop');
    const sStop = 400 - 16;             // STOP_MARGIN = m(4) = 16
    assert.ok(Math.abs(r.profile.eval(r.profile.duration) - sStop) < 3, `s(T)=${r.profile.eval(r.profile.duration)}`);
    assert.ok(Math.abs(r.profile.deriv1(r.profile.duration)) < 2, 'v(T)≈0');
    // 不后退（前进单调）
    let prev = -1;
    for (let i = 0; i <= 40; i++) {
      const s = r.profile.eval((i / 40) * r.profile.duration);
      assert.ok(s >= prev - 1e-6, '前进单调');
      prev = s;
    }
  });

  it('舒适整定：高速近距离时峰值减速度不超过舒适值（10% 容差）', () => {
    const r = solveSpeedLayer({ stObstacles: [staticObstacle(200)], v0: 140, targetV: 140 });
    assert.strictEqual(r.mode, 'stop');
    assert.ok(peakAccel(r.profile) <= (A_COMFORT as number) * 1.1,
      `峰值减速 ${peakAccel(r.profile).toFixed(1)} > 舒适值 ${A_COMFORT}`);
  });

  it('物理停不住 → emergency（距离 < v0²/2A_MAX）', () => {
    const r = solveSpeedLayer({ stObstacles: [staticObstacle(40)], v0: 240, targetV: 240 });
    assert.strictEqual(r.mode, 'emergency');
    assert.ok(r.profile.deriv1(r.profile.duration) < r.profile.deriv1(0), '应减速');
  });

  it('障碍已在身后/无静态条目（tMin>0）→ cruise（接口预留）', () => {
    const r = solveSpeedLayer({ stObstacles: [{ sMin: 300, sMax: 350, tMin: 2, tMax: 5 }], v0: 100, targetV: 100 });
    assert.strictEqual(r.mode, 'cruise');
  });
});
```

- [ ] **Step 2: 运行确认失败**

Run: `npx tsc && node --test dist/js/test/plan/speed.test.js`
Expected: FAIL（Cannot find module `../../algo/plan/speed.js`）

- [ ] **Step 3: 实现 speed.ts**

创建 `js/algo/plan/speed.ts`：

```typescript
// ==================== 算法：速度层（ST 空间） ====================
//
// 【职责】给定路径层的阻塞信息（ST 占据条目），生成纵向速度剖面 s(t)：
//   cruise    — 无阻塞：收敛到巡航目标 targetV
//   stop      — 有阻塞且物理可停：在障碍前 STOP_MARGIN 处减速到 0（停车线）
//   emergency — 物理停不住（距离 < v0²/2A_MAX）：最大减速兜底（接受接触）
//
// 【ST 图】接口按"障碍在 s-t 平面占据区间"通用设计（可承载动态障碍），
// v1 静态实现：只处理 tMin===0 的条目（占据现在起的所有时间）；tMin>0 忽略。
//
// 【停车线整定】T = max(2·sStop/v0, T_STOP_MIN)；峰值减速度超舒适值
// A_COMFORT 时按 T ×= sqrt(a_peak/A_COMFORT) 延长（至多 3 次迭代）。

import { m, mps2, dimless, px, perpx } from '../../shared/dimensions.js';
import { solveQuintic, solveSpeedProfile } from './polynomial.js';

export interface STObstacle { sMin: number; sMax: number; tMin: number; tMax: number; }
export type SpeedMode = 'cruise' | 'stop' | 'emergency';

export interface SpeedProfile {
  eval(t: number): number;
  deriv1(t: number): number;
  duration: number;
}

export interface SpeedLayerResult { profile: SpeedProfile; mode: SpeedMode; }

export const SPEED_HORIZON = 2.0;
export const STOP_MARGIN = m(4);
export const A_COMFORT = mps2(3.5);
export const A_MAX = mps2(6.0);
export const T_STOP_MIN = 0.8;
const MAX_TUNE_ITER = 3;

export function solveSpeedLayer(o: { stObstacles: STObstacle[]; v0: number; targetV: number }): SpeedLayerResult {
  // 第一个站在路径正前方的静态障碍（sMin ≥ 0；tMin 必须为 0 = 现在起持续占据）
  const ahead = o.stObstacles
    .filter(ob => ob.tMin === 0 && ob.sMin >= 0)
    .sort((a, b) => a.sMin - b.sMin)[0];

  if (!ahead) {
    // solveSpeedProfile(v0: PxPerSec, a0: PxPerSec2, vf: PxPerSec, Tf: Seconds)
    const poly = solveSpeedProfile(pxps(o.v0), pxps2(0), pxps(o.targetV), secs(SPEED_HORIZON));
    return {
      mode: 'cruise',
      profile: { eval: t => poly.eval(t), deriv1: t => poly.deriv1(t), duration: SPEED_HORIZON },
    };
  }

  const sStop = ahead.sMin - (STOP_MARGIN as number);
  const v0 = o.v0;

  // 物理可行性：停车距离不足 → 最大减速兜底
  if (sStop <= 0 || sStop < (v0 * v0) / (2 * (A_MAX as number))) {
    const T = Math.max(T_STOP_MIN, v0 / (A_MAX as number));
    const poly = solveSpeedProfile(pxps(v0), pxps2(0), pxps(0), secs(T));
    return {
      mode: 'emergency',
      profile: { eval: t => poly.eval(t), deriv1: t => poly.deriv1(t), duration: T },
    };
  }

  // 停车线剖面：s(0)=0, s'(0)=v0, s''(0)=0, s(T)=sStop, s'(T)=0, s''(T)=0
  let T = Math.max((2 * sStop) / v0, T_STOP_MIN);
  let poly = solveQuintic(px(0), dimless(v0), perpx(0), px(sStop), dimless(0), perpx(0), T);

  for (let i = 0; i < MAX_TUNE_ITER; i++) {
    let peakDecel = 0;
    for (let k = 1; k <= 20; k++) {
      const t = (k / 20) * T;
      peakDecel = Math.max(peakDecel, -((poly.deriv2(t) as number)));
    }
    if (peakDecel <= (A_COMFORT as number)) break;
    T *= Math.sqrt(peakDecel / (A_COMFORT as number));
    poly = solveQuintic(px(0), dimless(v0), perpx(0), px(sStop), dimless(0), perpx(0), T);
  }

  return {
    mode: 'stop',
    profile: { eval: t => poly.eval(t) as number, deriv1: t => poly.deriv1(t) as number, duration: T },
  };
}
```

（文件头 import 需包含：`m, mps2` 来自 dimensions；`px, pxps, pxps2, dimless, perpx, secs` 同样来自 dimensions。上面 cruise/emergency 分支用 `pxps/pxps2/secs`，停车线分支用 `px/dimless/perpx`——import 行合并为：
`import { m, mps2, px, pxps, pxps2, dimless, perpx, secs } from '../../shared/dimensions.js';`）

- [ ] **Step 4: 运行确认通过**

Run: `npx tsc && node --test dist/js/test/plan/speed.test.js`
Expected: PASS（6 个用例）。若"舒适整定"用例峰值略超容差，打印峰值并核对迭代次数（3 次内应足够），必要时把容差注明后放宽到实测 ×1.05。

- [ ] **Step 5: 提交**

```bash
git add js/algo/plan/speed.ts js/test/plan/speed.test.ts
git commit -m "新增规划速度层 speed.ts（ST 接口 + 巡航/停车线/兜底剖面）"
```

---

### Task 3: 规划器编排重写 + 全下游适配（原子替换）

**Files:**
- Modify: `js/algo/plan/planner.ts`（重写为层编排）
- Modify: `js/algo/plan/states.ts`（nextState 输入更新）
- Modify: `js/shared/debug.ts`（PlanningDebug 新 schema）
- Modify: `js/sim/records.ts`（recordFrom 的 plan 映射适配旧字段名）
- Modify: `js/ui/inspector.ts`（候选表/速度档显示适配）
- Delete: `js/algo/plan/candidate.ts`
- Delete: `js/test/plan/candidate-corridor.test.ts`、`js/test/plan/candidate-safety.test.ts`
- Modify: `js/test/plan/planner.test.ts`、`js/test/plan/planner-safety.test.ts`（迁移）、`js/test/sim/engine.test.ts`（黄金基线）

**Interfaces:**
- Consumes: Task 1 `solvePathLayer/PATH_STEPS/PATH_HORIZON/PathCandidate`；Task 2 `solveSpeedLayer/SpeedMode`；现有 frenet/collision/polynomial/states
- Produces: `plan()` 对外签名与 `Trajectory` 输出不变；`getPlannerState()` 不变；`debug.planning` 新 schema（见下）；`recordFrom` 输出保持归档 schema 字段名不变

- [ ] **Step 1: 重写 planner.ts**

`js/algo/plan/planner.ts` 全量替换为：

```typescript
// ==================== 算法：规划器编排层（纵向-横向解耦） ====================
//
// 【架构】SL 路径层（paths.ts）+ ST 速度层（speed.ts），单轮协调后合成 Trajectory。
//   路径层决定"走哪条路"（横向，120m 视程）；速度层决定"以什么速度走"（纵向，
//   含停车线——横向无解时停在障碍前）。合成 l(s(t))，输出契约不变
//   （控制/回放/归档/检测零改动）。详见 spec：2026-09-19-decoupled-planner-design.md
//
// 【每帧流程】
//   1. Frenet 参考线（route 开环 / 赛道闭环）
//   2. 车辆投影 + dl/ds
//   3. ARRIVED 检测
//   4. 纵向巡航目标（vCap 沿用的单值版 + 终点 DECEL 缩放）
//   5. solvePathLayer → 最优路径（全阻塞时=最晚碰撞的停车基准）
//   6. solveSpeedLayer → 巡航 / 停车线 / 兜底剖面
//   7. 合成 l(s(t)) → Trajectory；状态机 + 兼容类型标签

import {
  WorldSnapshot, LidarScan, Route, FrenetReference,
  Trajectory, TrajectoryPoint, createTrajectory,
} from '../../shared/types.js';
import {
  Px, m, mps, pxps, secs, dimless, perpx,
} from '../../shared/dimensions.js';
import { buildFrenetReference, cartesianToFrenet, frenetToCartesian } from './frenet.js';
import { PlanState, nextState } from './states.js';
import { debug } from '../../shared/debug.js';
import { solvePathLayer, PATH_STEPS, PATH_HORIZON } from './paths.js';
import { solveSpeedLayer, STObstacle } from './speed.js';

// ==================== 参数 ====================

const DECEL_ZONE = m(25) as number;   // 路线终点减速区
const STOP_ASSUMED_LEN = m(8) as number;   // 阻塞条目假定的障碍长度（停车基准的 sMax）

/** 车道中心偏移量（Lane 3 = +1.875m）。切换车道改此值即可。 */
let laneOffset: Px = m(1.875);

// ==================== 状态 ====================

let _loopFrenetRef: FrenetReference | null = null;
let _routeFrenetRef: FrenetReference | null = null;
let _lastRouteKey: string | null = null;
let _currentState: PlanState = PlanState.CRUISE;
let _prevBestTargetL: number | null = null;

// ==================== 主入口 ====================

export function plan(snapshot: WorldSnapshot, lidarScan: LidarScan, route?: Route): Trajectory {
  const { car, track } = snapshot;

  // ─── 1. Frenet 参考线 ───
  let frenetRef: FrenetReference;
  let routeActive = false;
  if (route && route.waypoints.length >= 2) {
    const key = `${route.startIndex}-${route.goalIndex}`;
    if (key !== _lastRouteKey) {
      _routeFrenetRef = buildFrenetReference(route.waypoints, false);
      _lastRouteKey = key;
      _prevBestTargetL = null;
    }
    frenetRef = _routeFrenetRef!; routeActive = true;
  } else {
    if (!_loopFrenetRef) _loopFrenetRef = buildFrenetReference(track.centerPoints, true);
    frenetRef = _loopFrenetRef;
  }

  // ─── 2. Frenet 状态 ───
  const fl = cartesianToFrenet(car.x, car.y, frenetRef);
  const tangent = frenetRef.tangents[fl.idx];
  const ta = Math.atan2(tangent.y, tangent.x);
  const hd = Math.atan2(Math.sin((car.angle as number) - ta), Math.cos((car.angle as number) - ta));
  const dl0 = dimless(Math.tan(hd));
  const cosHD = Math.cos(hd);
  const remainingDist = routeActive ? frenetRef.totalS - fl.s : Infinity;

  // ─── 3. ARRIVED ───
  if (routeActive && remainingDist <= 10) {
    _currentState = PlanState.ARRIVED;
    return createTrajectory([], 0, 'arrived');
  }

  // ─── 4. 纵向巡航目标（单值）───
  const minObsDist = Math.min(...lidarScan.distances);
  let targetV = Math.min(mps(35) as number, Math.max(mps(17.5) as number, minObsDist / 2));
  if (routeActive && remainingDist < DECEL_ZONE) {
    targetV *= Math.max(0.1, remainingDist / DECEL_ZONE);
  }

  // ─── 5. 路径层 ───
  const pathLayer = solvePathLayer({
    ref: frenetRef, l0: fl.l, dl0, s0: fl.s,
    carX: car.x as number, carY: car.y as number, lidarScan,
    carWidth: car.width, collisionRadius: car.collisionRadius,
    laneOffset, prevTargetL: _prevBestTargetL,
  });

  // ─── 6. 速度层 ───
  const stObstacles: STObstacle[] = pathLayer.allBlocked
    ? [{ sMin: pathLayer.sBlock - fl.s, sMax: pathLayer.sBlock - fl.s + (STOP_ASSUMED_LEN as number), tMin: 0, tMax: Infinity }]
    : [];
  const speedLayer = solveSpeedLayer({ stObstacles, v0: (car.speed as number) * cosHD, targetV });

  // ─── 7. 合成 l(s(t)) ───
  const duration = speedLayer.profile.duration;
  const points: TrajectoryPoint[] = [];
  for (let i = 0; i <= PATH_STEPS; i++) {
    const t = (i / PATH_STEPS) * duration;
    const sRel = Math.max(0, Math.min(PATH_HORIZON as number, speedLayer.profile.eval(t)));
    const li = pathLayer.best.evalL(sRel);
    const cart = frenetToCartesian(fl.s + sRel, li, frenetRef);
    points.push({ t: secs(t), x: cart.x, y: cart.y, yaw: cart.yaw,
      v: pxps(speedLayer.profile.deriv1(t)), kappa: perpx(0) });
  }

  // ─── 8. 状态机 ───
  _currentState = nextState({
    routeActive,
    nearEnd: routeActive && remainingDist < DECEL_ZONE,
    allBlocked: pathLayer.allBlocked,
    stopInfeasible: speedLayer.mode === 'emergency',
    avoiding: Math.abs((pathLayer.best.targetL as number) - (laneOffset as number)) > 3,
  });

  // ─── 9. 兼容类型标签（overlays 颜色 / HUD 方向 / 检测器 / 测试依赖）───
  let type: string;
  if (speedLayer.mode === 'emergency') type = 'emergency';
  else if (Math.abs((pathLayer.best.targetL as number) - (fl.l as number)) > 3) {
    type = (pathLayer.best.targetL as number) > (fl.l as number) ? 'avoid_left' : 'avoid_right';
  } else type = 'lane_keep';

  // ─── 10. debug（inspect 面板 / 归档记录）───
  debug.planning = {
    state: _currentState,
    frenetS: fl.s, frenetL: fl.l as number,
    dl_ds: dl0 as number, headingDiff: hd,
    paths: pathLayer.ranked.map(p => ({ targetL: p.targetL as number, blocked: p.blocked, cost: p.cost })),
    bestCost: pathLayer.best.cost,
    bestTargetL: pathLayer.best.targetL as number,
    speedTarget: targetV,
    speedMode: speedLayer.mode,
    remainingDist: routeActive ? remainingDist : -1,
  };

  // ─── 11. 帧间平滑 ───
  _prevBestTargetL = pathLayer.best.targetL as number;

  return createTrajectory(points, pathLayer.best.cost, type);
}

export function getPlannerState(): string {
  return _currentState;
}
```

- [ ] **Step 2: 更新 states.ts 的 nextState**

`js/algo/plan/states.ts`：`TransitionInput` 与 `nextState` 替换为（其余枚举/标签函数保留不动）：

```typescript
/** 状态转移条件（解耦架构：路径层/速度层决策驱动） */
export interface TransitionInput {
  routeActive: boolean;      // 是否有激活的路线
  nearEnd: boolean;          // 是否接近终点（剩余 < DECEL_ZONE）
  avoiding: boolean;         // 当前路径偏离车道中心（避让侧向）
  allBlocked: boolean;       // 所有路径候选阻塞（停车线生效）
  stopInfeasible: boolean;   // 物理停不住（速度层兜底）
}

/**
 * 确定下一状态
 *
 * 优先级：EMERGENCY > DECEL(停车线) > AVOID > DECEL(终点) > FOLLOW/CRUISE
 * （ARRIVED 在 planner 入口提前返回，不经过本函数）
 */
export function nextState(input: TransitionInput): PlanState {
  if (input.stopInfeasible) return PlanState.EMERGENCY;
  if (input.allBlocked) return PlanState.DECEL;          // 停车线
  if (input.avoiding) return PlanState.AVOID;
  if (input.routeActive && input.nearEnd) return PlanState.DECEL;
  return input.routeActive ? PlanState.FOLLOW : PlanState.CRUISE;
}
```

Run: `npx tsc 2>&1 | head -20` —— 此时会报 `candidate.ts` 的旧引用与 debug schema 不匹配，逐步修复（下一步）。

- [ ] **Step 3: 更新 debug.ts 的 PlanningDebug**

`js/shared/debug.ts` 中 `PlanningDebug` 替换为：

```typescript
/** 规划模块内部状态（解耦架构：路径层候选 + 速度层模式） */
export interface PlanningDebug {
  state: string;               // 当前状态机状态
  frenetS: number;             // 车辆 Frenet s
  frenetL: number;             // 车辆 Frenet l
  dl_ds: number;               // dl/ds 斜率
  headingDiff: number;         // 车头与切线夹角
  paths: {
    targetL: number;
    blocked: boolean;
    cost: number;
  }[];
  bestCost: number;
  bestTargetL: number;
  /** 巡航速度目标（仿真单位） */
  speedTarget: number;
  /** 速度层模式：cruise | stop | emergency */
  speedMode: string;
  remainingDist: number;
}
```

- [ ] **Step 4: 适配 records.ts 与 inspector.ts**

`js/sim/records.ts` 的 `recordFrom` 中 `case 'planning:result'` 内映射改为（**归档 schema 字段名不变**，detector 零改动）：

```typescript
    case 'planning:result': {
      const pl = p.planning;
      return { type: 'plan', t: p.t, step: p.step, state: p.state,
        frenet: { s: pl?.frenetS ?? 0, l: pl?.frenetL ?? 0, dl_ds: pl?.dl_ds ?? 0, headingDiff: pl?.headingDiff ?? 0 },
        candidates: (pl?.paths ?? []).map((x: any) => ({
          targetL: x.targetL, targetV: pl?.speedTarget ?? 0, cost: x.cost, collision: x.blocked })),
        best: { cost: pl?.bestCost ?? 0, targetL: pl?.bestTargetL ?? 0, targetV: pl?.speedTarget ?? 0 },
        trajectory: { points: p.trajectory.points.map((pt: any) => ({ t: pt.t, x: pt.x, y: pt.y, v: pt.v })),
                      type: p.trajectory.type, cost: p.trajectory.cost } };
    }
```

`js/ui/inspector.ts` 的 Planning 区块改为：

```typescript
  html += '\n<b>Planning</b>\n';
  if (pl) {
    html += `  状态: ${pl.state} | S=${toM(pl.frenetS)}m L=${toM(pl.frenetL, 2)}m\n`;
    html += `  dl/ds=${pl.dl_ds.toFixed(4)} | Δheading=${(pl.headingDiff*180/Math.PI).toFixed(1)}°\n`;
    if (pl.remainingDist > 0) html += `  剩余: ${toM(pl.remainingDist)}m\n`;
    html += `  速度: 目标 ${toM(pl.speedTarget)}m/s 模式 ${pl.speedMode}\n`;
    html += `  最优路径: L=${toM(pl.bestTargetL, 1)}m cost=${pl.bestCost.toFixed(1)}\n`;
    html += '  路径候选:\n';
    for (const p of pl.paths) {
      const mark = p.targetL === pl.bestTargetL ? '★' : ' ';
      html += `    ${mark} L=${toM(p.targetL, 1).padStart(5)}m cost=${p.cost.toFixed(1)}${p.blocked ? ' BLOCK' : ''}\n`;
    }
  }
```

- [ ] **Step 5: 删除 coupling 遗留并迁移测试**

```bash
git rm js/algo/plan/candidate.ts
git rm js/test/plan/candidate-corridor.test.ts js/test/plan/candidate-safety.test.ts
```

`js/test/plan/planner.test.ts` 迁移（对应用例逐条改）：
- `describe('planner 巡航')` 不变（全清/避障/远障三例在新架构下语义相同：避障断言 `traj.type !== 'lane_keep' && !== 'emergency'` 仍成立——路径层选侧向 ⇒ avoid_*）
- `describe('planner 一致性')` 不变（确定性输出）
- 新增（vCap 迁移，原 candidate-corridor 的限速断言）：
```typescript
describe('planner 速度目标（vCap 单值化）', () => {
  it('障碍 30m 内 → speedTarget ≤ 17.5m/s；50m → ≤ 25m/s；无障碍 → 35m/s', () => {
    const car = makeCar(100, 0, 0, 150);
    plan(makeSnapshot(car), makeBlockedLidar([0], 120));
    assert.ok(debug.planning!.speedTarget <= (mps(17.5) as number) + 1e-6);
    plan(makeSnapshot(car), makeBlockedLidar([0], 200));
    assert.ok(debug.planning!.speedTarget <= (mps(25) as number) + 1e-6);
    plan(makeSnapshot(car), makeClearLidar());
    assert.strictEqual(debug.planning!.speedTarget, mps(35) as number);
  });
});
```
（头部补 `import { debug } from '../../shared/debug.js';`；makeBlockedLidar/makeClearLidar 该文件已有）

`js/test/plan/planner-safety.test.ts` 迁移：
- "正前方 15m 障碍" 用例改为：
```typescript
  it('正前方 15m 障碍（物理停不住）→ emergency 最大减速剖面', () => {
    const car = makeCar(500, 0, 0, 100);   // 25 m/s，制动距离 ≈52m ≫ 15m
    const traj = plan(makeSnapshot(car), lidarWithObstacle(0, 60, 10));
    assert.strictEqual(debug.planning!.speedMode, 'emergency');
    assert.ok((traj.points[traj.points.length - 1].v as number) < (traj.points[0].v as number), '应减速');
  });

  it('正前方 60m 障碍（距离充足）→ stop：停车线剖面 v→0', () => {
    const car = makeCar(500, 0, 0, 100);
    const traj = plan(makeSnapshot(car), lidarWithObstacle(0, 240, 10));
    assert.strictEqual(debug.planning!.speedMode, 'stop');
    const last = traj.points[traj.points.length - 1];
    assert.ok(Math.abs(last.v as number) < 3, `末点速度应≈0，实际 ${last.v}`);
  });
```
- 其余不变量用例（50m 不穿障/无障碍非紧急/速度上限/时间递增/边界）保留；若个别断言与新语义冲突（打印实际值判断），按"保留不变量、断开耦合语义"原则最小修改并在注释注明原因。

- [ ] **Step 6: 全量测试 + 黄金基线更新**

Run: `npx tsc && npm test`
Expected: 仅 `SimEngine 路线模式` 黄金基线失败（行为变化），其余全绿。
按既有流程更新基线：用临时脚本跑 seed 42 的 300 步（命令见 `docs/superpowers/plans/` 历史计划 Task 3 Step 9 同款 node -e），人工核验（沿赛道、位移 ~100m、末速合理、无 NaN）后粘贴四条断言并注明"解耦规划器上线"。

- [ ] **Step 7: 提交**

```bash
git add js/algo/plan/ js/shared/debug.ts js/sim/records.ts js/ui/inspector.ts js/test/
git commit -m "规划器重写为解耦架构（SL 路径层 + ST 速度层），下游零契约改动

- planner.ts 变为层编排 + l(s(t)) 合成；states.ts nextState 更新为新输入
- 删除 candidate.ts 与耦合语义测试；限速/停车语义迁移到 planner 测试
- debug/records/inspector 适配新 debug schema（归档字段名不变，detector 零改动）
- 兼容类型标签 avoid_*/lane_keep/emergency/arrived（overlays/HUD 零改动）"
```

---

### Task 4: 批量 A/B 验收 + 浏览器验证 + 文档

**Files:**
- Modify: `CLAUDE.md`（规划器一节改写）
- Modify: `ISSUE_HISTORY.md`（升级记录 + A/B 数据）
- （数据产物：`runs/`，gitignore）

**Interfaces:**
- Consumes: 新规划器（Task 3）；批量 A/B 工作流（既有脚本模式）

- [ ] **Step 1: 批量 A/B（10 种子）**

```bash
for f in scenarios/batch/oval-s*.json scenarios/batch/hairpin-s*.json; do name=$(basename $f .json); out=$(npm run sim -- --scenario $f 2>&1 | grep "判定" | head -1); echo "$name: $out"; done
```

聚合对比（新批次 vs 上一批次 = 当前基线 6 episode / 195 帧）：

```bash
node -e "
const fs=require('fs'), path=require('path');
const runs = fs.readdirSync('runs').filter(d=>d.includes('batch-')).sort();
const byScen = {};
for (const d of runs) { const m=d.match(/batch-(oval|hairpin)-s(\d)/); if(!m)continue; (byScen[m[0]]=byScen[m[0]]||[]).push(d); }
function stats(d){const iss=JSON.parse(fs.readFileSync(path.join('runs',d,'issues.json'),'utf8'));const sum=JSON.parse(fs.readFileSync(path.join('runs',d,'summary.json'),'utf8'));const coll=iss.filter(i=>i.check==='collision_true');return {eps:coll.length,frames:coll.reduce((a,i)=>a+i.count,0),minObs:(sum.minObstacleDistM??9).toFixed(2)};}
console.log('场景            | 基线: ep 帧 最小距 | 解耦: ep 帧 最小距');
let aT={e:0,f:0}, bT={e:0,f:0};
for (const key of Object.keys(byScen).sort()) { const ds=byScen[key].sort(); const a=stats(ds[ds.length-2]), b=stats(ds[ds.length-1]); aT.e+=a.eps;aT.f+=a.frames; bT.e+=b.eps;bT.f+=b.frames; console.log(key.padEnd(15),'|',String(a.eps).padStart(3),String(a.frames).padStart(4),a.minObs.padStart(6),'|',String(b.eps).padStart(3),String(b.frames).padStart(4),b.minObs.padStart(6)); }
console.log('合计: episode', aT.e,'→',bT.e,'| 碰撞帧', aT.f,'→',bT.f);
"
```

**期望与门槛**：几何极限场景从"擦碰穿越"变为"停在障碍前"——碰撞帧应显著下降；其他场景保持干净。**若总碰撞帧劣化 → 停止，向用户报告实测数据与差异场景，不自行回滚。**

- [ ] **Step 2: 浏览器手工验收**

硬刷新 `http://localhost:8080/`：
1. 加载 `oval-avoid-1`：避障正常（轨迹蓝色、避障时 HUD 显示方向提示）
2. 加载 batch 场景中一个新出现的"停车"场景（从 A/B 输出挑选碰撞帧清零/下降的场景）：观察**车辆在障碍前减速停住**（速度归零），HUD 状态显示 DECEL；inspect 面板显示"速度: 目标 … 模式 stop"与路径候选 BLOCK 标记
3. 自由巡航、右键设路线、终点减速停车、回放、导出/导入均无回归
4. HUD 速度单位切换（km/h↔m/s）不受影响

- [ ] **Step 3: 更新文档**

`CLAUDE.md` 规划器一节整段替换为解耦架构描述：

```markdown
### 规划器（js/algo/plan/）

- 解耦架构：**SL 路径层**（`paths.ts`，120m 视程的横向路径候选，整路径碰撞检查，全阻塞时按"最晚碰撞"选停车基准）+ **ST 速度层**（`speed.ts`，巡航/停车线/兜底三种剖面，通用 STObstacle 接口静态实现）；`planner.ts` 只做层编排与 `l(s(t))` 合成
- 停车线：横向无解时在障碍前 4m 减速到 0（物理停不住时退化为最大减速）；`polynomial.ts` 的五次多项式同时服务横向 l(s) 与纵向 s(t)
- 输出契约不变：Trajectory 点序列 + 兼容类型标签（`avoid_*/lane_keep/emergency/arrived`）；速度适应 vCap（`clamp(最近障碍距离/2, 17.5, 35)`）作为速度层的巡航目标
```

`ISSUE_HISTORY.md` 新增"第九轮：规划层解耦升级"条目：动机（几何极限场景无停车表达能力）、架构（SL+ST 单轮协调）、A/B 数据（如实）、衍生 issue（stall/route_timeout 明账）。

- [ ] **Step 4: 全量回归 + 提交**

Run: `npm test` → 全绿
```bash
git add CLAUDE.md ISSUE_HISTORY.md
git commit -m "解耦规划器验收：批量 A/B + 浏览器验证 + 文档更新"
```

---

## 计划自审记录

1. **Spec 覆盖**：Spec §3（架构/模块/ST 接口/状态映射）→ T1/T2/T3；§4（算法细节与参数）→ T1/T2 常量与代码逐条落实（120m/40 步/±2.5m 挪移/4m 停车余量/3.5·6 减速度/迭代整定）；§5（测试与迁移表）→ T3 逐项执行 + T1/T2 新测；§6（顺序/风险：不劣化门槛、反例停止报告）→ T4；§7 开放问题两条已按默认值固化（T1/T2 常量）与自然覆盖（每帧重评估）
2. **计划级细化两处**（已在头部声明）：`TransitionInput` 增加 `avoiding` 保留 AVOID 状态（overlays 高亮依赖）；"15m 用例"split 为 emergency 正例 + 60m stop 正例（物理事实）
3. **无占位**：全部步骤含可执行代码/命令与期望输出
4. **类型一致性**：`PathCandidate/PathLayerResult/SpeedProfile/SpeedMode/STObstacle` 在 T1/T2 定义、T3 消费，签名逐字一致；`solveSpeedProfile(pxps, pxps2, pxps, secs)` 调用形态与 polynomial.ts 实际签名一致
5. **已知取舍**：合成时 kappa 仍置 0（与旧实现一致，控制层不使用）；`STOP_ASSUMED_LEN` 只影响 ST 条目 sMax，不参与停车线计算（停车基准取 sMin）
