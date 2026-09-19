# Headless 仿真模式实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 Node 中无人值守运行自动驾驶仿真，全量归档原始/中间数据到文件，自动检测四类问题并生成报告，浏览器可回放任意一次运行。

**Architecture:** 从 `main.ts` 提取 `SimEngine`（世界状态 + 四频率 tick 订阅）作为浏览器/headless 共用核心；浏览器改为固定步长累积器（与 headless 逐帧一致）；headless 由 CLI 驱动，把 engine 发布的消息主题快照成 JSONL 归档；检测器为纯模块（帧记录流 + 真值 → issues）；回放器在浏览器按归档数据驱动现有渲染层。

**Tech Stack:** TypeScript (strict, ES2020/ESNext, ESM) + Node 18 内置模块（node:fs / node:path / node:process / node:test / node:assert）+ 现有 Three.js 渲染层（仅回放使用）。

**Spec:** `docs/superpowers/specs/2026-09-19-headless-simulation-design.md`

## Global Constraints

- TypeScript 严格模式；模块间 import 一律带 `.js` 后缀；编译产物在 `dist/js/**`（rootDir="."，`npx tsc`）
- 不新增任何 npm 依赖（fs/path/process/child_process 均用 Node 内置）
- 测试用 `node:test` + `node:assert`；运行方式 `npx tsc && node --test dist/js/test/...`
- 新增模块必须有中文注释；算法/核心模块注释含原理说明（项目注释规范见 CLAUDE.md）
- 量纲：模块边界使用品牌类型（`Px/Radians/PxPerSec/DimLess...`），`px()/m()/mps()/rad()/dimless()` 工厂函数是唯一 unsafe cast 点
- 随机性必须经 `shared/rng.ts` 注入；确定性要求：**同机器、同 Node 版本、同场景、同种子 → 逐帧数值完全一致**
- **车辆运动语义不变**：仅当 `routeState === 'ACTIVE'` 时物理步进；无路线时 `car.speed = 0`（与当前浏览器一致）。行驶类场景必须设置 route
- 每个任务结束时提交一次 commit（中文 message）

---

### Task 1: 可播种随机数 `shared/rng.ts`

**Files:**
- Create: `js/shared/rng.ts`
- Test: `js/test/sim/rng.test.ts`

**Interfaces:**
- Consumes: 无（纯函数模块）
- Produces: `type Rng = () => number`；`createRng(seed: number): Rng`（mulberry32）；`deriveSeed(baseSeed: number, label: string): number`（FNV-1a，子种子派生）。Task 2/3/4 使用

- [ ] **Step 1: 写失败测试**

创建 `js/test/sim/rng.test.ts`：

```typescript
// ==================== 测试：可播种随机数 ====================
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { createRng, deriveSeed } from '../../shared/rng.js';

describe('createRng', () => {
  it('同种子 → 同序列（前 5 个值全等）', () => {
    const a = createRng(42), b = createRng(42);
    for (let i = 0; i < 5; i++) assert.strictEqual(a(), b());
  });

  it('不同种子 → 不同序列', () => {
    const a = createRng(42), b = createRng(43);
    assert.notStrictEqual(a(), b());
  });

  it('输出在 [0, 1) 区间', () => {
    const rng = createRng(7);
    for (let i = 0; i < 1000; i++) {
      const v = rng();
      assert.ok(v >= 0 && v < 1, `越界值 ${v}`);
    }
  });
});

describe('deriveSeed', () => {
  it('同参数 → 同结果；不同 label → 不同结果', () => {
    assert.strictEqual(deriveSeed(42, 'gps'), deriveSeed(42, 'gps'));
    assert.notStrictEqual(deriveSeed(42, 'gps'), deriveSeed(42, 'obstacles'));
  });

  it('与派生顺序无关（先派生 gps 再 obstacles，结果不变）', () => {
    const g1 = deriveSeed(42, 'gps');
    deriveSeed(42, 'obstacles');
    assert.strictEqual(deriveSeed(42, 'gps'), g1);
  });
});
```

- [ ] **Step 2: 运行测试确认失败**

Run: `npx tsc && node --test dist/js/test/sim/rng.test.js`
Expected: FAIL（编译报错：Cannot find module '../../shared/rng.js'）

- [ ] **Step 3: 实现 rng.ts**

创建 `js/shared/rng.ts`：

```typescript
// ==================== 可播种随机数（PRNG） ====================
//
// 【为什么需要？】
// headless 模式要求"同种子 → 逐帧一致"，Math.random() 无法复现。
// 所有随机子系统（障碍物布局 / GPS 噪声 / 随机赛道）都接收 Rng 注入。
//
// 【mulberry32】32 位种子、周期 2^32、统计质量足够仿真使用，
// 实现仅 6 行且完全确定性（无内部隐式状态）。
//
// 【子种子派生】同一根种子的不同子系统必须用不同 RNG 实例，
// 否则一个子系统的取数次数会偏移另一个子系统的序列（顺序耦合）。
// deriveSeed(base, 'gps') 用 FNV-1a 从根种子派生独立子种子。

export type Rng = () => number;

/** mulberry32 — 返回 [0,1) 均匀分布随机数 */
export function createRng(seed: number): Rng {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6D2B79F5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

/** FNV-1a 派生子种子 — 与调用顺序、次数无关 */
export function deriveSeed(baseSeed: number, label: string): number {
  let h = (2166136261 ^ (baseSeed >>> 0)) >>> 0;
  for (let i = 0; i < label.length; i++) {
    h ^= label.charCodeAt(i);
    h = Math.imul(h, 16777619) >>> 0;
  }
  return h >>> 0;
}
```

- [ ] **Step 4: 运行测试确认通过**

Run: `npx tsc && node --test dist/js/test/sim/rng.test.js`
Expected: PASS（5 个用例）

- [ ] **Step 5: 提交**

```bash
git add js/shared/rng.ts js/test/sim/rng.test.ts
git commit -m "新增可播种随机数模块（mulberry32 + 子种子派生）"
```

---

### Task 2: 环境模块随机源注入（obstacle / localization / track）

**Files:**
- Modify: `js/env/obstacle.ts`（`generateObstacles` 追加 rng 参数，替换 5 处 `Math.random`）
- Modify: `js/env/localization.ts`（`localize` 追加 rng 参数，`boxMuller` 接收 rng）
- Modify: `js/env/track.ts`（`generateTrack` 追加 rng 参数，替换随机半径处）
- Test: `js/test/sim/env-rng.test.ts`

**Interfaces:**
- Consumes: `Rng` from `../shared/rng.js`（Task 1）
- Produces: `generateObstacles(track, count, minSize?, maxSize?, rng?: Rng): Obstacle[]`；`localize(car, config?, rng?: Rng): GPSReading`；`generateTrack(width, height, controlRadii?, rng?: Rng): TrackGeometry`。默认参数 `Math.random`，浏览器行为不变

- [ ] **Step 1: 写失败测试**

创建 `js/test/sim/env-rng.test.ts`：

```typescript
// ==================== 测试：环境模块随机源注入 ====================
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { generateTrack } from '../../env/track.js';
import { generateObstacles } from '../../env/obstacle.js';
import { localize } from '../../env/localization.js';
import { createCar } from '../../env/car.js';
import { createRng } from '../../shared/rng.js';

describe('随机源注入', () => {
  it('generateTrack 随机模式：同种子 → 同赛道', () => {
    const t1 = generateTrack(1800, 800, undefined, createRng(1));
    const t2 = generateTrack(1800, 800, undefined, createRng(1));
    assert.deepStrictEqual(t1.centerPoints, t2.centerPoints);
    const t3 = generateTrack(1800, 800, undefined, createRng(2));
    assert.notDeepStrictEqual(t1.centerPoints, t3.centerPoints);
  });

  it('generateObstacles：同种子 → 同布局', () => {
    const track = generateTrack(1800, 800);
    const o1 = generateObstacles(track, 4, 10, 25, createRng(9));
    const o2 = generateObstacles(track, 4, 10, 25, createRng(9));
    assert.strictEqual(o1.length, 4);
    assert.deepStrictEqual(o1, o2);
  });

  it('localize：同种子 → 同噪声（ignore timestamp）', () => {
    const car = createCar({ x: 100, y: 100 } as any, { x: 110, y: 100 } as any);
    const r1 = localize(car, { noiseStdDev: 1.25 }, createRng(3));
    const r2 = localize(car, { noiseStdDev: 1.25 }, createRng(3));
    assert.strictEqual(r1.x, r2.x);
    assert.strictEqual(r1.y, r2.y);
  });
});
```

- [ ] **Step 2: 运行测试确认失败**

Run: `npx tsc && node --test dist/js/test/sim/env-rng.test.js`
Expected: FAIL（参数数量/类型不匹配或结果不确定）

- [ ] **Step 3: 修改三个环境模块**

`js/env/obstacle.ts` — 导入 rng 类型，函数签名与 5 处调用改为注入式：

```typescript
import { Rng } from '../shared/rng.js';
// ...
export function generateObstacles(
  track: TrackGeometry, count: number,
  minSize = 10, maxSize = 25, rng: Rng = Math.random,
): Obstacle[] {
  // Fisher–Yates 洗牌：Math.random() → rng()
  const j = Math.floor(rng() * (i + 1));
  // ...其余随机取值同理：
  const w = px(minSize + rng() * (maxSize - minSize));
  const h = px(6 + rng() * 10);
  const zH = 5 + rng() * 12;
  const angle = rad(rng() * Math.PI * 2);
```

`js/env/localization.ts` — `boxMuller(rng)` 与 `localize` 第三个参数：

```typescript
import { Rng } from '../shared/rng.js';
// ...
function boxMuller(rng: Rng): number {
  let u1: number, u2: number;
  do { u1 = rng(); } while (u1 === 0);
  u2 = rng();
  return Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2);
}
// ...
export function localize(car: CarState, config?: Partial<LocalizationConfig>, rng: Rng = Math.random): GPSReading {
  const cfg = { ...DEFAULT_CONFIG, ...config };
  const sigma = cfg.noiseStdDev;
  const noiseX = boxMuller(rng) * sigma;
  const noiseY = boxMuller(rng) * sigma;
  // ...其余不变
```

`js/env/track.ts` — `generateTrack` 第四个参数：

```typescript
import { Rng } from '../shared/rng.js';
// ...
export function generateTrack(width: number, height: number, controlRadii?: number[], rng: Rng = Math.random): TrackGeometry {
  // ...
  const radius = controlRadii ? controlRadii[i] : (m(60) as number + rng() * m(20) as number);
```

- [ ] **Step 4: 运行全部测试确认通过**

Run: `npx tsc && node --test dist/js/test/`
Expected: 既有 30 用例 + 新增 3 用例全部 PASS

- [ ] **Step 5: 提交**

```bash
git add js/env/obstacle.ts js/env/localization.ts js/env/track.ts js/test/sim/env-rng.test.ts
git commit -m "环境模块随机源注入（obstacle/localization/track 支持种子复现）"
```

---

### Task 3: SimEngine 提取 + 浏览器固定步长

**Files:**
- Create: `js/sim/engine.ts`
- Modify: `js/env/car.ts`（`checkBounds` 返回 boolean，表示是否发生复位）
- Modify: `js/main.ts`（重写为 engine + 渲染装配 + 固定步长累积器）
- Test: `js/test/sim/engine.test.ts`

**Interfaces:**
- Consumes: Task 1/2 的 rng；现有 `generateTrack/createCar/stepPhysics/checkBounds/generateObstacles/localize/sense/plan/getPlannerState/control/buildRoadGraph/MessageBus/debug`
- Produces: `FIXED_DT`、`SIM_W`、`SIM_H`、`PERCEPTION_HZ/PLANNING_HZ/GPS_HZ` 常量；`WorldConfig`；`RouteState`；`class SimEngine`（成员：`bus/track/roadGraph/car/obstacles/route/routeState/routeStartIdx/latestLidarScan/latestTrajectory/latestGPS/simTime/stepCount`；方法：`step(dt?)/rebuildWorld(cfg)/setRoute(route, startIdx, opts?)`）；消息主题 `tick/perception:result/planning:result/gps:result/control:state/sim:event`。Task 4/5/6/7–10/12 依赖

- [ ] **Step 1: 写失败测试（确定性 + 不变量 + 复位标志）**

创建 `js/test/sim/engine.test.ts`：

```typescript
// ==================== 测试：仿真引擎（确定性 + 行为基线） ====================
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { SimEngine, FIXED_DT } from '../../sim/engine.js';
import { MessageBus } from '../../messageBus.js';
import { MAP_PRESETS } from '../../env/mapDefinitions.js';
import { checkBounds } from '../../env/car.js';
import { m, mps, px, rad } from '../../shared/dimensions.js';

function makeEngine(seed = 42) {
  return new SimEngine(new MessageBus('test', false), { map: MAP_PRESETS['oval'], seed });
}

describe('SimEngine 确定性', () => {
  it('同种子两次运行 → 600 步后状态逐字段一致', () => {
    const e1 = makeEngine(42), e2 = makeEngine(42);
    for (let i = 0; i < 600; i++) { e1.step(FIXED_DT); e2.step(FIXED_DT); }
    assert.strictEqual(e1.car.x, e2.car.x);
    assert.strictEqual(e1.car.y, e2.car.y);
    assert.strictEqual(e1.car.angle, e2.car.angle);
    assert.strictEqual(e1.car.speed, e2.car.speed);
    assert.strictEqual(e1.obstacles.length, e2.obstacles.length);
  });
});

describe('SimEngine 不变量（静态场景：无路线，车不动）', () => {
  it('10s 后数值有限、速度归零（与浏览器语义一致：无路线不驱动）', () => {
    const e = makeEngine(42);
    for (let i = 0; i < 600; i++) e.step(FIXED_DT);
    assert.ok(Number.isFinite(e.car.x as number));
    assert.ok(Number.isFinite(e.car.y as number));
    assert.strictEqual(e.car.speed, 0);
    assert.strictEqual(e.stepCount, 600);
  });
});

describe('SimEngine 路线模式', () => {
  it('设置路线后车辆运动，且速度不超过最高速', () => {
    const e = makeEngine(42);
    const wp = e.track.centerPoints.slice(0, 50);
    e.setRoute(
      { waypoints: wp, waypointIndices: wp.map((_, i) => i), startIndex: 0, goalIndex: 49, length: px(0) },
      0,
    );
    assert.strictEqual(e.routeState, 'ACTIVE');
    for (let i = 0; i < 300; i++) e.step(FIXED_DT);
    assert.ok((e.car.speed as number) > 0);
    assert.ok((e.car.speed as number) <= (mps(42.5) as number) + 1e-9);
  });
});

describe('checkBounds 返回值', () => {
  it('车在赛道外 → 返回 true 并复位到中心线', () => {
    const e = makeEngine(42);
    e.car.x = px(-10000); e.car.y = px(-10000);
    const reset = checkBounds(e.car, e.track);
    assert.strictEqual(reset, true);
    assert.ok((e.car.x as number) > 0); // 已复位到赛道附近
  });
});
```

- [ ] **Step 2: 运行测试确认失败**

Run: `npx tsc && node --test dist/js/test/sim/engine.test.js`
Expected: FAIL（Cannot find module '../../sim/engine.js'；`checkBounds` 返回 void 导致断言失败）

- [ ] **Step 3: 修改 checkBounds 返回复位标志**

`js/env/car.ts` 末尾：

```typescript
export function checkBounds(car: CarState, track: TrackGeometry): boolean {
  const r = (car.width as number) / 2 + (m(1.5) as number);
  if (!isCarInsideTrack(car.x as number, car.y as number, r, track)) {
    resetToTrack(car, track.centerPoints);
    return true;
  }
  return false;
}
```

- [ ] **Step 4: 实现 SimEngine**

创建 `js/sim/engine.ts`（注释保持项目风格，此处省略逐行注释以便阅读计划；实现时按各订阅者职责补中文注释）：

```typescript
// ==================== 仿真核心引擎（浏览器 / headless 共用） ====================
import { MessageBus } from '../messageBus.js';
import {
  WorldSnapshot, LidarScan, Trajectory, Route, GPSReading, TrackGeometry, CarState, Obstacle,
  createWorldSnapshot,
} from '../shared/types.js';
import { Radians, Px, PxPerSec, rad, m, mps, mps2, pxps } from '../shared/dimensions.js';
import { debug } from '../shared/debug.js';
import { generateTrack } from '../env/track.js';
import { generateObstacles } from '../env/obstacle.js';
import { createCar, stepPhysics, checkBounds } from '../env/car.js';
import { MapDefinition } from '../env/mapDefinitions.js';
import { localize, LocalizationConfig } from '../env/localization.js';
import { sense, MAX_SENSOR_DIST, SCAN_RAY_COUNT } from '../algo/perception.js';
import { plan, getPlannerState } from '../algo/plan/index.js';
import { control } from '../algo/control.js';
import { buildRoadGraph, RoadGraph } from '../algo/routePlanner.js';
import { Rng, createRng, deriveSeed } from '../shared/rng.js';

export const SIM_W = 1800, SIM_H = 800;
export const FIXED_DT = 1 / 60;
export const PERCEPTION_HZ = 20, PLANNING_HZ = 15, GPS_HZ = 10;
const DEFAULT_GPS: Partial<LocalizationConfig> = { noiseStdDev: 1.25 };

export interface WorldConfig {
  map: MapDefinition;
  /** 显式障碍物布局（优先于随机） */
  obstacles?: Obstacle[];
  /** 随机模式数量；缺省 3 + rng()*3 */
  obstacleCount?: number;
  /** 随机根种子；缺省 undefined = Math.random（浏览器） */
  seed?: number;
  carStart?: { x: Px; y: Px; angle?: Radians; speed?: PxPerSec };
  gpsConfig?: Partial<LocalizationConfig>;
}

export type RouteState = 'NONE' | 'SETTING_START' | 'SETTING_GOAL' | 'ACTIVE' | 'ARRIVED';

function emptyScan(): LidarScan {
  const d = new Array(SCAN_RAY_COUNT).fill(MAX_SENSOR_DIST) as Px[];
  const a = new Array(SCAN_RAY_COUNT).fill(rad(0)) as Radians[];
  return { angles: a, distances: d, rawDistances: d, confidences: [], timestamp: 0 };
}

export class SimEngine {
  readonly bus: MessageBus;
  mapDef: MapDefinition;
  track!: TrackGeometry;
  roadGraph!: RoadGraph;
  car!: CarState;
  obstacles: Obstacle[] = [];
  route: Route | null = null;
  routeState: RouteState = 'NONE';
  routeStartIdx: number | null = null;
  latestLidarScan: LidarScan;
  latestTrajectory: Trajectory;
  latestGPS: GPSReading | null = null;
  simTime = 0;
  stepCount = 0;

  private _cfg: WorldConfig;
  private _perceptionTimer = 0;
  private _planningTimer = 0;
  private _gpsTimer = 0;
  private _prevPlanState: string | null = null;
  private _braking = false;
  private _gpsRng: Rng = Math.random;

  constructor(bus: MessageBus, cfg: WorldConfig) {
    this.bus = bus;
    this._cfg = cfg;
    this.mapDef = cfg.map;
    this.latestLidarScan = emptyScan();
    this.latestTrajectory = { points: [], cost: 0, type: 'lane_keep', timestamp: 0 };
    this.rebuildWorld(cfg);
    this._wireSubscribers();
  }

  rebuildWorld(cfg: WorldConfig): void {
    this._cfg = cfg;
    this.mapDef = cfg.map;
    this.track = generateTrack(SIM_W, SIM_H, cfg.map.controlRadii);
    this.roadGraph = buildRoadGraph(this.track.centerPoints);

    const seed = cfg.seed;
    const obsRng: Rng = seed === undefined ? Math.random : createRng(deriveSeed(seed, 'obstacles'));
    this._gpsRng = seed === undefined ? Math.random : createRng(deriveSeed(seed, 'gps'));
    const count = cfg.obstacleCount ?? (3 + Math.floor(obsRng() * 3));
    this.obstacles = cfg.obstacles ?? generateObstacles(this.track, count, 10, 25, obsRng);

    this.car = createCar(this.track.centerPoints[0], this.track.centerPoints[1]);
    if (cfg.carStart) {
      this.car.x = cfg.carStart.x;
      this.car.y = cfg.carStart.y;
      if (cfg.carStart.angle !== undefined) this.car.angle = cfg.carStart.angle;
      if (cfg.carStart.speed !== undefined) this.car.speed = cfg.carStart.speed;
    }
    this.route = null;
    this.routeState = 'NONE';
    this.routeStartIdx = null;
    this.latestGPS = null;
    this.simTime = 0;
    this.stepCount = 0;
    this._perceptionTimer = this._planningTimer = this._gpsTimer = 0;
    this._prevPlanState = null;
    this._braking = false;
  }

  /** 与浏览器右键设路线的语义一致：车放到路线起点、朝向第二点 */
  setRoute(route: Route, startIdx: number, opts?: { speed?: PxPerSec }): void {
    this.route = route;
    this.routeStartIdx = startIdx;
    this.routeState = 'ACTIVE';
    this.car.x = route.waypoints[0].x;
    this.car.y = route.waypoints[0].y;
    const np = route.waypoints[Math.min(1, route.waypoints.length - 1)];
    this.car.angle = rad(Math.atan2(
      (np.y as number) - (this.car.y as number),
      (np.x as number) - (this.car.x as number),
    ));
    this.car.speed = opts?.speed ?? mps(7.5);
  }

  step(dt: number = FIXED_DT): void {
    this.simTime += dt;
    this.stepCount += 1;
    this.bus.publish('tick', { dt, t: this.simTime, step: this.stepCount });
  }

  private _emit(kind: string, payload: Record<string, unknown>): void {
    this.bus.publish('sim:event', { kind, step: this.stepCount, t: this.simTime, payload });
  }

  private _wireSubscribers(): void {
    // 感知 20Hz
    this.bus.subscribe('tick', ({ dt }: { dt: number }) => {
      this._perceptionTimer += dt;
      if (this._perceptionTimer >= 1 / PERCEPTION_HZ) {
        const snapshot = createWorldSnapshot(this.car, this.track, this.obstacles);
        this.latestLidarScan = sense(snapshot);
        this.bus.publish('perception:result', {
          snapshot, scan: this.latestLidarScan,
          stats: debug.perception ? { ...debug.perception } : null,
          t: this.simTime, step: this.stepCount,
        });
        this._perceptionTimer = 0;
      }
    });

    // 规划 15Hz
    this.bus.subscribe('tick', ({ dt }: { dt: number }) => {
      this._planningTimer += dt;
      if (this._planningTimer >= 1 / PLANNING_HZ) {
        const snapshot = createWorldSnapshot(this.car, this.track, this.obstacles);
        this.latestTrajectory = plan(snapshot, this.latestLidarScan, this.route ?? undefined);
        const state = getPlannerState();
        if (this._prevPlanState !== null && state !== this._prevPlanState) {
          this._emit('state_change', { from: this._prevPlanState, to: state });
        }
        this._prevPlanState = state;
        if (this.latestTrajectory.type === 'arrived' && this.routeState === 'ACTIVE') {
          this.routeState = 'ARRIVED';
          this._emit('arrived', {});
        }
        this.bus.publish('planning:result', {
          snapshot, scan: this.latestLidarScan, trajectory: this.latestTrajectory,
          state, route: this.route, planning: debug.planning ? { ...debug.planning } : null,
          t: this.simTime, step: this.stepCount,
        });
        this._planningTimer = 0;
      }
    });

    // GPS 10Hz
    this.bus.subscribe('tick', ({ dt }: { dt: number }) => {
      this._gpsTimer += dt;
      if (this._gpsTimer >= 1 / GPS_HZ) {
        const truth = { x: this.car.x, y: this.car.y };
        this.latestGPS = localize(this.car, this._cfg.gpsConfig ?? DEFAULT_GPS, this._gpsRng);
        this.bus.publish('gps:result', {
          reading: this.latestGPS, truth, t: this.simTime, step: this.stepCount,
        });
        this._gpsTimer = 0;
      }
    });

    // 控制 + 物理（每 tick）
    this.bus.subscribe('tick', ({ dt }: { dt: number }) => {
      if (this.routeState === 'ACTIVE') {
        const mid = Math.floor(this.latestLidarScan.distances.length / 2);
        const frontDist = this.latestLidarScan.distances[mid] as number;
        // 紧急制动：LiDAR 前方 < 5m 且速度 > 3m/s 时减 5m/s²，最低 2m/s
        const braking = frontDist < (m(5) as number) && (this.car.speed as number) > (mps(3) as number);
        if (braking !== this._braking) {
          this._braking = braking;
          if (braking) this._emit('emergency_brake', { frontDist });
        }
        if (braking) {
          this.car.speed = pxps(Math.max(
            mps(2) as number,
            (this.car.speed as number) - (mps2(5) as number) * dt,
          ));
        }
        stepPhysics(this.car, control(this.latestTrajectory, this.car), dt);
        if (checkBounds(this.car, this.track)) this._emit('bounds_reset', {});
      } else {
        this.car.speed = pxps(0);
      }
      const snapshot = createWorldSnapshot(this.car, this.track, this.obstacles);
      this.bus.publish('control:state', {
        snapshot, lidarScan: this.latestLidarScan, trajectory: this.latestTrajectory,
        state: getPlannerState(), control: debug.control ? { ...debug.control } : null,
        t: this.simTime, step: this.stepCount,
      });
    });
  }
}
```

注意：`import { localize, LocalizationConfig } from '../env/localization.js'` 只需一次（去掉示例中的别名行）；`LocalizationConfig` 需要从 localization.ts `export`（已存在 ✓）。

- [ ] **Step 5: 运行引擎测试确认通过**

Run: `npx tsc && node --test dist/js/test/sim/engine.test.js`
Expected: PASS（4 个用例）

- [ ] **Step 6: 重写 main.ts（engine + 固定步长 + 渲染装配）**

`js/main.ts` 完整替换为：

```typescript
// ==================== 主入口：engine 装配 + 渲染 ====================
import { MessageBus } from './messageBus.js';
import { Recorder } from './shared/recorder.js';
import { debug } from './shared/debug.js';
import { SIM_W, SIM_H, FIXED_DT, SimEngine } from './sim/engine.js';
import { initScene, updateScene, resetSceneCache } from './env/renderer.js';
import { MAP_PRESETS, DEFAULT_MAP } from './env/mapDefinitions.js';
import { setupUI, updateInspectPanel } from './ui.js';

const W = SIM_W, H = SIM_H;
const canvas = document.getElementById('canvas') as HTMLCanvasElement;
canvas.width = W; canvas.height = H;

const { renderer, scene, camera, controls, lidarLines } = initScene(canvas, W, H);
const engine = new SimEngine(new MessageBus('autopilot', false), { map: MAP_PRESETS[DEFAULT_MAP] });

const recorder = new Recorder();
recorder.start(engine.mapDef.name);
engine.bus.subscribe('planning:result', (p: any) => {
  recorder.recordFrame(p.snapshot.car, p.scan, p.trajectory, p.state, p.route);
});

document.getElementById('hud-map')!.textContent = engine.mapDef.name;

let clickStartIdx: number | null = null;

setupUI({
  W, H, canvas, camera, scene, controls, recorder,
  get mapDef() { return engine.mapDef; },
  get track() { return engine.track; },
  get roadGraph() { return engine.roadGraph; },
  get car() { return engine.car; },
  get route() { return engine.route; },
  get routeState() { return engine.routeState; },
  get latestGPS() { return engine.latestGPS; },
  onMapChange(name: string) {
    const mapDef = MAP_PRESETS[name] || MAP_PRESETS[DEFAULT_MAP];
    engine.rebuildWorld({ map: mapDef });
    resetSceneCache(scene);
    recorder.reset(); recorder.start(mapDef.name);
    document.getElementById('hud-map')!.textContent = mapDef.name;
  },
  onRouteSet(r, startIdx) { engine.setRoute(r, startIdx); },
  setRouteState(s: string) { engine.routeState = s as typeof engine.routeState; },
  setClickStartIdx(idx: number | null) { clickStartIdx = idx; },
  getClickStartIdx: () => clickStartIdx,
  getRouteState: () => engine.routeState,
});

engine.bus.subscribe('control:state', ({ snapshot, lidarScan, trajectory, state }: any) => {
  updateScene(scene, snapshot, lidarScan, state, lidarLines, trajectory, engine.route, engine.latestGPS);
});

let lastTime = performance.now();
let acc = 0;
const MAX_CATCHUP = 5;

function gameLoop(now: number) {
  const realDt = Math.min(0.1, (now - lastTime) / 1000);
  lastTime = now;

  if (!debug.paused) {
    acc += realDt;
    let n = 0;
    while (acc >= FIXED_DT && n < MAX_CATCHUP) {
      engine.step(FIXED_DT);
      acc -= FIXED_DT;
      n++;
    }
    if (n === MAX_CATCHUP) acc = 0; // 防止追帧螺旋
  }

  controls.update();
  renderer.render(scene, camera);
  updateInspectPanel(realDt);
  requestAnimationFrame(gameLoop);
}
requestAnimationFrame(gameLoop);
```

注意：`setClickStartIdx/getClickStartIdx` 驱动 ui.ts 右键路由状态机（`SETTING_START/SETTING_GOAL`），状态保存在 main 侧模块级变量 `clickStartIdx` 中——保持原行为，不迁入 engine。`getRouteState()` 转发 engine.routeState，ui.ts 的 `rs` 判断与 `setRouteState` 协同工作（与改造前一致）。

- [ ] **Step 7: 全部测试 + 类型检查通过**

Run: `npm test`
Expected: 全部 PASS（滚动输出末尾 `# fail 0`）

- [ ] **Step 8: 浏览器手工验证（最大风险点，必做）**

```bash
npm run build && npx serve -l 8080   # 或 python3 -m http.server 8080
```

打开 `http://localhost:8080/`，逐项验证并记录结果：
1. 车辆初始静止；右键赛道设起点、再设终点 → 车辆起步行驶
2. 遇障碍物避障；到终点减速停车（HUD 状态依次 FOLLOW→AVOID→DECEL→ARRIVED）
3. Space 暂停/继续；R 键重置视角；地图下拉切换（Oval→Hairpin）后一切正常
4. 导出/导入按钮可用；inspect 面板数据在刷新
5. 帧率稳定（固定步长无明显抖动）

Expected: 全部正常。任何一项异常 → 停止并修复后再继续（这是行为回归防线）

- [ ] **Step 9: 冻结黄金基线（防后续回归）**

在 `js/test/sim/engine.test.ts` 的路线模式用例末尾追加实际值断言：先临时 `console.log(e.car.x, e.car.y, e.car.angle, e.car.speed)`，运行记录 300 步后的实际值，人工检查合理性（沿赛道前进、速度 ~7.5-10 m/s、无 NaN），然后写入：

```typescript
    // 黄金基线（实现时冻结，防重构回归；数值来自人工核验后的首次运行）
    assert.strictEqual(e.car.x, /* 粘贴实际值 */ 0);
    assert.strictEqual(e.car.y, /* 粘贴实际值 */ 0);
```

注意：必须用**固定种子 42**（makeEngine 默认），确保基线稳定。粘贴完成后重跑 `node --test dist/js/test/sim/engine.test.js` 必须 PASS。

- [ ] **Step 10: 提交**

```bash
git add js/sim/engine.ts js/main.ts js/env/car.ts js/test/sim/engine.test.ts
git commit -m "提取 SimEngine 共享核心，浏览器改为固定步长累积器"
```

---

### Task 4: 场景加载 `sim/scenario.ts`

**Files:**
- Create: `js/sim/scenario.ts`
- Test: `js/test/sim/scenario.test.ts`

**Interfaces:**
- Consumes: `MAP_PRESETS`、`generateTrack`、`generateObstacles`（Task 2 签名）、`buildRoadGraph/findRoute`、`buildFrenetReference/frenetToCartesian/cartesianToFrenet`（algo/plan/frenet.js）、`createRng/deriveSeed`、`dimensions`
- Produces: `interface Scenario`（version/name/map/seed/duration/car?/obstacles/route?/checks?）；`class ScenarioError extends Error`；`parseScenario(json: unknown): Scenario`；`buildScenarioWorld(scenario, width, height): { worldCfg: WorldConfig; route: Route | null; }`。Task 6 使用
- 单位约定：场景文件中 `s/l/w/len/goalS/startS` 一律**米**（`car.speed` 为 m/s），解析时用 `m()/mps()` 转仿真单位

- [ ] **Step 1: 写失败测试**

创建 `js/test/sim/scenario.test.ts`：

```typescript
// ==================== 测试：场景解析与构建 ====================
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { parseScenario, buildScenarioWorld, ScenarioError } from '../../sim/scenario.js';
import { cartesianToFrenet, buildFrenetReference } from '../../algo/plan/frenet.js';
import { m } from '../../shared/dimensions.js';

const VALID = {
  version: 1, name: 't1', map: 'oval', seed: 42, duration: 30,
  obstacles: { mode: 'explicit', list: [{ s: 120, l: 2.0, w: 2.0, len: 4.0 }] },
  route: { startS: 0, goalS: 300 },
};

describe('parseScenario 校验', () => {
  it('合法场景解析成功并填默认值', () => {
    const s = parseScenario(VALID);
    assert.strictEqual(s.name, 't1');
    assert.strictEqual(s.obstacles.mode, 'explicit');
  });

  it('version 不符 / 坏 map 名 / 非法 s → 抛 ScenarioError', () => {
    assert.throws(() => parseScenario({ ...VALID, version: 2 }), ScenarioError);
    assert.throws(() => parseScenario({ ...VALID, map: 'no_such_map' }), ScenarioError);
    assert.throws(() => parseScenario({
      ...VALID,
      obstacles: { mode: 'explicit', list: [{ s: -5, l: 0, w: 2, len: 2 }] },
    }), ScenarioError);
    assert.throws(() => parseScenario({
      ...VALID,
      obstacles: { mode: 'explicit', list: [{ s: 10, l: 99, w: 2, len: 2 }] },
    }), ScenarioError);
  });
});

describe('buildScenarioWorld', () => {
  it('显式障碍物 (s,l) → 世界坐标满足 Frenet 反投影', () => {
    const s = parseScenario(VALID);
    const { worldCfg, track } = buildScenarioWorld(s, 1800, 800);
    const ref = buildFrenetReference(track.centerPoints, true);
    const obs = worldCfg.obstacles![0];
    const fl = cartesianToFrenet(obs.x, obs.y, ref);
    assert.ok(Math.abs((fl.l as number) - (m(2.0) as number)) < (m(1.0) as number),
      `l 偏差过大: ${fl.l} vs ${m(2.0)}`);
    assert.ok(Math.abs(fl.s - (m(120) as number)) < (m(2.0) as number),
      `s 偏差过大: ${fl.s} vs ${m(120)}`);
  });

  it('随机模式同种子 → 同布局；route startS/goalS → 索引正确', () => {
    const s = parseScenario({ ...VALID, obstacles: { mode: 'random', count: 4 } });
    const a = buildScenarioWorld(s, 1800, 800);
    const b = buildScenarioWorld(s, 1800, 800);
    assert.deepStrictEqual(a.worldCfg.obstacles, b.worldCfg.obstacles);
    assert.ok(a.route !== null);
    assert.strictEqual(a.route!.startIndex, 0);
    assert.ok(a.route!.waypointIndices.length > 1);
  });
});
```

注意：上面第一个用例的 `buildFrenetReference` 调用签名不对（这是草稿）——**实现时改写为真实断言**：用 `buildScenarioWorld` 返回的 track（需一并返回，见下）构建 `FrenetReference`，对 `worldCfg.obstacles![0]` 调 `cartesianToFrenet`，断言 `|l - m(2.0)| < 0.4`（sim 单位容差）。

- [ ] **Step 2: 运行测试确认失败**

Run: `npx tsc && node --test dist/js/test/sim/scenario.test.js`
Expected: FAIL（Cannot find module '../../sim/scenario.js'）

- [ ] **Step 3: 实现 scenario.ts**

创建 `js/sim/scenario.ts`，要点：

```typescript
// ==================== 场景加载：JSON → 世界配置 ====================
import { Obstacle, Point2D, Route, TrackGeometry, createRoute } from '../shared/types.js';
import { Px, Radians, rad, m, mps, px } from '../shared/dimensions.js';
import { MAP_PRESETS, MapDefinition } from '../env/mapDefinitions.js';
import { generateTrack, TRACK_HALF_WIDTH } from '../env/track.js';
import { generateObstacles } from '../env/obstacle.js';
import { buildRoadGraph, findRoute } from '../algo/routePlanner.js';
import { buildFrenetReference, frenetToCartesian } from '../algo/plan/frenet.js';
import { createRng, deriveSeed } from '../shared/rng.js';
import { WorldConfig } from './engine.js';

export class ScenarioError extends Error {}

export interface ScenarioObstacle { s: number; l: number; w: number; len: number; yaw?: number; }
export interface Scenario {
  version: 1; name: string; map: string; seed: number; duration: number;
  car?: { speed?: number };
  obstacles: { mode: 'explicit'; list: ScenarioObstacle[] } | { mode: 'random'; count: number };
  route?: { startS: number; goalS: number };
  checks?: Record<string, number>;
}

/** 解析 + 校验；任何非法输入抛 ScenarioError（带字段路径信息） */
export function parseScenario(json: unknown): Scenario { /* 逐字段校验，见下 */ }

/** 累计弧长 → 最近中心线索引（闭环） */
function sToIndex(centerPoints: Point2D[], s: number): number {
  let acc = 0, best = 0, bestErr = Infinity;
  const n = centerPoints.length - 1;
  for (let i = 0; i < n; i++) {
    const err = Math.abs(acc - s);
    if (err < bestErr) { bestErr = err; best = i; }
    acc += Math.hypot(
      (centerPoints[(i + 1) % n].x as number) - (centerPoints[i].x as number),
      (centerPoints[(i + 1) % n].y as number) - (centerPoints[i].y as number),
    );
  }
  return best;
}

export function buildScenarioWorld(scenario: Scenario, width: number, height: number): {
  worldCfg: WorldConfig; track: TrackGeometry; route: Route | null;
} {
  const mapDef: MapDefinition = MAP_PRESETS[scenario.map];
  const track = generateTrack(width, height, mapDef.controlRadii);
  const ref = buildFrenetReference(track.centerPoints, true);

  let obstacles: Obstacle[];
  if (scenario.obstacles.mode === 'explicit') {
    obstacles = scenario.obstacles.list.map(o => {
      const cart = frenetToCartesian(m(o.s) as number, m(o.l), ref);
      return {
        x: cart.x, y: cart.y,
        width: m(o.w), height: m(o.len),
        angle: rad((cart.yaw as number) + (o.yaw ?? 0)),
        zHeight: 8,
      };
    });
  } else {
    const rng = createRng(deriveSeed(scenario.seed, 'obstacles'));
    obstacles = generateObstacles(track, scenario.obstacles.count, 10, 25, rng);
  }

  let route: Route | null = null;
  let carStart: WorldConfig['carStart'] = undefined;
  if (scenario.route) {
    const graph = buildRoadGraph(track.centerPoints);
    const startIdx = sToIndex(track.centerPoints, m(scenario.route.startS) as number);
    const goalIdx = sToIndex(track.centerPoints, m(scenario.route.goalS) as number);
    const pathIndices = findRoute(graph, startIdx, goalIdx);
    if (!pathIndices || pathIndices.length < 2) {
      throw new ScenarioError(`路线不可达: startS=${scenario.route.startS} goalS=${scenario.route.goalS}`);
    }
    const waypoints = pathIndices.map(i => track.centerPoints[i]);
    let totalLen = 0;
    for (let i = 1; i < waypoints.length; i++) {
      totalLen += Math.hypot(
        (waypoints[i].x as number) - (waypoints[i - 1].x as number),
        (waypoints[i].y as number) - (waypoints[i - 1].y as number),
      );
    }
    route = createRoute(waypoints, pathIndices, startIdx, goalIdx, px(totalLen));
    // 车放路线起点，朝向第二点（与 engine.setRoute 一致的语义）
    const np = waypoints[Math.min(1, waypoints.length - 1)];
    carStart = {
      x: waypoints[0].x, y: waypoints[0].y,
      angle: rad(Math.atan2(
        (np.y as number) - (waypoints[0].y as number),
        (np.x as number) - (waypoints[0].x as number),
      )),
      speed: scenario.car?.speed !== undefined ? mps(scenario.car.speed) : mps(25),
    };
  }

  return {
    worldCfg: { map: mapDef, obstacles, seed: scenario.seed, carStart },
    track, route,
  };
}
```

`parseScenario` 校验清单（逐条实现，错误信息含字段名）：
- `version === 1`；`name` 非空字符串；`map` ∈ MAP_PRESETS；`seed` 为整数 ≥ 0；`duration` > 0 且 ≤ 3600
- `obstacles.mode` ∈ {'explicit','random'}；explicit 时 list 非空数组，每项 `s ∈ [0, 3600)`、`|l| < 7.5`（米）、`w/len ∈ (0, 20]`；random 时 count 为 1..50 整数
- `route` 存在时 `startS/goalS ≥ 0` 且换算后 < 赛道总弧长（用 `m(s) < ref.totalS` 判断，ref 在 buildScenarioWorld 内；parse 阶段只做非负与数值校验）

- [ ] **Step 4: 运行测试确认通过**

Run: `npx tsc && node --test dist/js/test/sim/scenario.test.js`
Expected: PASS（3 个用例）。若 Frenet 反投影断言因最近邻投影误差超容差，先把容差放大到实测值 × 1.5 并在测试注释中记录实测偏差

- [ ] **Step 5: 提交**

```bash
git add js/sim/scenario.ts js/test/sim/scenario.test.ts
git commit -m "新增场景加载模块（JSON 校验 + Frenet (s,l) 障碍物 + 弧长路线）"
```

---

### Task 5: 归档类型 + ArchiveWriter

**Files:**
- Create: `js/shared/types/interfaces/archive.ts`
- Modify: `js/shared/types/interfaces.ts`（barrel 追加 export \*）
- Modify: `js/shared/types.ts`（barrel 追加 export type）
- Create: `js/headless/archive.ts`
- Test: `js/test/headless/archive.test.ts`

**Interfaces:**
- Consumes: `Obstacle/Point2D`（类型）、node:fs / node:path
- Produces: 记录类型 `FrameRecord = StepRecord | PerceptionRecord | PlanRecord | GpsRecord | EventRecord`，`Issue`、`RunMeta`、`RunSummary`、`WorldFile`、`RunsIndexEntry`；`class ArchiveWriter { constructor(runDir); writeMeta(meta); writeWorld(world); appendRecord(rec); finalize({issues, summary, reportMd, meta}) }`；`updateRunsIndex(runsRoot, entry)`。Task 6/7–11/12 依赖

- [ ] **Step 1: 写失败测试**

创建 `js/test/headless/archive.test.ts`：

```typescript
// ==================== 测试：归档写入 ====================
import { describe, it } from 'node:test';
import assert from 'node:assert';
import * as fs from 'node:fs';
import * as os from 'node:os';
import * as path from 'node:path';
import { ArchiveWriter, updateRunsIndex } from '../../headless/archive.js';

function tmpDir(): string {
  return fs.mkdtempSync(path.join(os.tmpdir(), 'ads-archive-'));
}

describe('ArchiveWriter', () => {
  it('写入记录 → 读回逐行 JSON 一致', () => {
    const dir = tmpDir();
    const w = new ArchiveWriter(dir);
    w.appendRecord({ type: 'step', t: 0.0167, step: 1,
      car: { x: 1, y: 2, angle: 0.5, speed: 100, steerAngle: 0 },
      control: { lookaheadDist: 40, alpha: 0.1, steerAngle: 0.1, throttle: 0.5 } });
    w.appendRecord({ type: 'event', t: 0.0167, step: 1, kind: 'arrived', payload: {} });
    const lines = fs.readFileSync(path.join(dir, 'frames.jsonl'), 'utf8').trim().split('\n');
    assert.strictEqual(lines.length, 2);
    assert.strictEqual(JSON.parse(lines[0]).type, 'step');
    assert.strictEqual(JSON.parse(lines[1]).kind, 'arrived');
  });

  it('finalize 写出 issues/summary/report 并把条目并入 index.json', () => {
    const root = tmpDir();
    const dir = path.join(root, 'run1');
    const w = new ArchiveWriter(dir);
    w.finalize({
      issues: [], summary: { steps: 10, simSeconds: 0.17 } as any,
      reportMd: '# 报告\nPASS', meta: { id: 'run1', verdict: 'PASS' } as any,
    });
    assert.ok(fs.existsSync(path.join(dir, 'issues.json')));
    assert.ok(fs.existsSync(path.join(dir, 'summary.json')));
    assert.ok(fs.existsSync(path.join(dir, 'report.md')));
    updateRunsIndex(root, { id: 'run1', scenario: 't', map: 'oval', seed: 1, createdAt: 'x',
      verdict: 'PASS', issues: { high: 0, medium: 0, low: 0 }, durationS: 60 });
    const idx = JSON.parse(fs.readFileSync(path.join(root, 'index.json'), 'utf8'));
    assert.strictEqual(idx.runs.length, 1);
    assert.strictEqual(idx.runs[0].id, 'run1');
  });

  it('index.json 同 id 二次写入 → 更新而非重复', () => {
    const root = tmpDir();
    updateRunsIndex(root, { id: 'a', verdict: 'PASS' } as any);
    updateRunsIndex(root, { id: 'a', verdict: 'ISSUES' } as any);
    const idx = JSON.parse(fs.readFileSync(path.join(root, 'index.json'), 'utf8'));
    assert.strictEqual(idx.runs.length, 1);
    assert.strictEqual(idx.runs[0].verdict, 'ISSUES');
  });
});
```

- [ ] **Step 2: 运行测试确认失败**

Run: `npx tsc && node --test dist/js/test/headless/archive.test.js`
Expected: FAIL（Cannot find module '../../headless/archive.js'）

- [ ] **Step 3: 定义记录类型**

创建 `js/shared/types/interfaces/archive.ts`（序列化形态：字段一律普通 `number`，注明仿真单位）：

```typescript
// ==================== 归档接口：headless 记录 / 问题 / 报告 ====================
//
// 这些是序列化形态的数据契约（字段为普通 number，仿真单位）。
// runner 写入、detector 消费、replay 读取三方共用。

export interface RunMeta { version: 1; id: string; scenario: string; map: string; seed: number;
  durationS: number; steps: number; gitCommit: string | null; nodeVersion: string;
  createdAt: string; verdict: 'PASS' | 'ISSUES' | 'ERROR'; issueCounts: { high: number; medium: number; low: number }; }

export interface ObstaclePlain { x: number; y: number; width: number; height: number; angle: number; zHeight?: number; }
export interface WorldFile {
  map: string;
  track: { centerPoints: [number, number][]; innerBoundary: [number, number][]; outerBoundary: [number, number][] };
  obstacles: ObstaclePlain[];
  route: { waypoints: [number, number][]; startIndex: number; goalIndex: number; length: number } | null;
}

export interface BaseRecord { type: string; t: number; step: number; }
export interface StepRecord extends BaseRecord { type: 'step';
  car: { x: number; y: number; angle: number; speed: number; steerAngle: number };
  control: { lookaheadDist: number; alpha: number; steerAngle: number; throttle: number } | null; }
export interface PerceptionRecord extends BaseRecord { type: 'perception';
  scan: { angles: number[]; distances: number[]; rawDistances: number[]; confidences: number[] };
  stats: { rawHits: number; filteredHits: number; frontDist: number; minDist: number } | null; }
export interface PlanRecord extends BaseRecord { type: 'plan'; state: string;
  frenet: { s: number; l: number; dl_ds: number; headingDiff: number };
  candidates: { targetL: number; targetV: number; cost: number; collision: boolean }[];
  best: { cost: number; targetL: number; targetV: number };
  trajectory: { points: { t: number; x: number; y: number; v: number }[]; type: string; cost: number }; }
export interface GpsRecord extends BaseRecord { type: 'gps'; x: number; y: number; err: number; }
export interface EventRecord extends BaseRecord { type: 'event'; kind: string; payload: Record<string, unknown>; }
export type FrameRecord = StepRecord | PerceptionRecord | PlanRecord | GpsRecord | EventRecord;

export interface Issue { category: 'safety' | 'perception' | 'planning' | 'mission';
  severity: 'high' | 'medium' | 'low'; check: string;
  firstFrame: number; lastFrame: number; count: number;
  summary: string; details: Record<string, unknown>; samples: number[]; }

export interface RunSummary { steps: number; simSeconds: number;
  speed: { maxMps: number; avgMps: number }; arrivedAtStep: number | null;
  trackingRmseM: number | null; minObstacleDistM: number | null;
  stateCounts: Record<string, number>; emergencyBrakes: number; boundsResets: number;
  [key: string]: unknown; }

export interface RunsIndexEntry { id: string; scenario: string; map: string; seed: number;
  createdAt: string; verdict: string; issues: { high: number; medium: number; low: number }; durationS: number; }
```

`js/shared/types/interfaces.ts`（barrel）追加：`export * from './archive.js';`（先查看该文件现状，保持既有 re-export 风格）；`js/shared/types.ts` 追加对应 `export type { ... } from './types/interfaces.js';` 条目。

- [ ] **Step 4: 实现 ArchiveWriter**

创建 `js/headless/archive.ts`：

```typescript
// ==================== Headless：运行归档 ====================
import * as fs from 'node:fs';
import * as path from 'node:path';
import { FrameRecord, Issue, RunMeta, RunSummary, WorldFile, RunsIndexEntry } from '../shared/types.js';

export class ArchiveWriter {
  readonly runDir: string;
  constructor(runDir: string) {
    this.runDir = runDir;
    fs.mkdirSync(runDir, { recursive: true });
  }

  writeMeta(meta: RunMeta): void {
    fs.writeFileSync(path.join(this.runDir, 'run.json'), JSON.stringify(meta, null, 2));
  }

  writeWorld(world: WorldFile): void {
    fs.writeFileSync(path.join(this.runDir, 'world.json'), JSON.stringify(world));
  }

  /** 逐行追加（崩溃时已写数据保留）；同步写，失败即抛 */
  appendRecord(rec: FrameRecord): void {
    fs.appendFileSync(path.join(this.runDir, 'frames.jsonl'), JSON.stringify(rec) + '\n');
  }

  finalize(o: { issues: Issue[]; summary: RunSummary; reportMd: string; meta: RunMeta }): void {
    fs.writeFileSync(path.join(this.runDir, 'issues.json'), JSON.stringify(o.issues, null, 2));
    fs.writeFileSync(path.join(this.runDir, 'summary.json'), JSON.stringify(o.summary, null, 2));
    fs.writeFileSync(path.join(this.runDir, 'report.md'), o.reportMd);
    this.writeMeta(o.meta);
  }
}

/** index.json 追加/更新（同 id 覆盖），供回放列表使用 */
export function updateRunsIndex(runsRoot: string, entry: RunsIndexEntry): void {
  const idxPath = path.join(runsRoot, 'index.json');
  let runs: RunsIndexEntry[] = [];
  if (fs.existsSync(idxPath)) {
    try { runs = JSON.parse(fs.readFileSync(idxPath, 'utf8')).runs ?? []; } catch { runs = []; }
  }
  const i = runs.findIndex(r => r.id === entry.id);
  if (i >= 0) runs[i] = entry; else runs.push(entry);
  fs.mkdirSync(runsRoot, { recursive: true });
  fs.writeFileSync(idxPath, JSON.stringify({ runs }, null, 2));
}
```

- [ ] **Step 5: 运行测试确认通过**

Run: `npx tsc && node --test dist/js/test/headless/archive.test.js`
Expected: PASS（3 个用例）

- [ ] **Step 6: 提交**

```bash
git add js/shared/types/interfaces/archive.ts js/shared/types/interfaces.ts js/shared/types.ts js/headless/archive.ts js/test/headless/archive.test.ts
git commit -m "新增归档记录类型与 ArchiveWriter（JSONL 流式写入 + index 维护）"
```

---

### Task 6: CLI 入口 `headless/run.ts`（运行 + 归档 + 确定性 E2E）

**Files:**
- Create: `js/headless/run.ts`
- Modify: `package.json`（新增 `"sim"` 脚本；`"test"` 改为 `tsc && node --test dist/js/test/`）
- Modify: `.gitignore`（追加 `runs/`）
- Test: `js/test/headless/e2e.test.ts`

**Interfaces:**
- Consumes: Task 3 `SimEngine/WorldConfig/FIXED_DT`；Task 4 `parseScenario/buildScenarioWorld`；Task 5 `ArchiveWriter/updateRunsIndex` 与记录类型；`MessageBus`
- Produces: `npm run sim -- [--scenario f.json] [--map oval] [--obstacles 4] [--seed 42] [--duration 60] [--out runs/x] [--fail-on-issue]`；`function recordFrom(topic: string, payload: any): FrameRecord | EventRecord`（主题 → 记录映射，Task 11 复用）；运行产出完整 runs/<id>/ 目录。Task 7–11 在此文件接检测/报告

- [ ] **Step 1: 写失败测试（CLI 确定性 E2E）**

创建 `js/test/headless/e2e.test.ts`：

```typescript
// ==================== 测试：CLI 端到端确定性 ====================
import { describe, it } from 'node:test';
import assert from 'node:assert';
import * as fs from 'node:fs';
import * as os from 'node:os';
import * as path from 'node:path';
import { execFileSync } from 'node:child_process';

const CLI = path.resolve('dist/js/headless/run.js');

function runCli(outDir: string, extra: string[] = []): void {
  execFileSync('node', [CLI, '--map', 'oval', '--obstacles', '4', '--seed', '7',
    '--duration', '5', '--out', outDir, ...extra], { stdio: 'pipe' });
}

describe('headless CLI E2E', () => {
  it('同场景同种子两次运行 → frames.jsonl 逐字节一致', () => {
    const a = fs.mkdtempSync(path.join(os.tmpdir(), 'ads-e2e-a-'));
    const b = fs.mkdtempSync(path.join(os.tmpdir(), 'ads-e2e-b-'));
    runCli(path.join(a, 'run'));
    runCli(path.join(b, 'run'));
    const fa = fs.readFileSync(path.join(a, 'run', 'frames.jsonl'), 'utf8');
    const fb = fs.readFileSync(path.join(b, 'run', 'frames.jsonl'), 'utf8');
    assert.ok(fa.length > 0);
    assert.strictEqual(fa, fb);
  });

  it('产出 run.json / world.json / frames.jsonl 且 meta 含种子', () => {
    const a = fs.mkdtempSync(path.join(os.tmpdir(), 'ads-e2e-c-'));
    runCli(path.join(a, 'run'));
    const meta = JSON.parse(fs.readFileSync(path.join(a, 'run', 'run.json'), 'utf8'));
    assert.strictEqual(meta.seed, 7);
    assert.ok(fs.existsSync(path.join(a, 'run', 'world.json')));
  });
});
```

- [ ] **Step 2: 运行确认失败**

Run: `npx tsc && node --test dist/js/test/headless/e2e.test.js`
Expected: FAIL（run.js 不存在 / 命令非零退出）

- [ ] **Step 3: 实现 run.ts**

创建 `js/headless/run.ts`，结构如下（完整实现）：

```typescript
// ==================== Headless：CLI 入口 ====================
//
// 流程：参数解析 → 场景构建 → engine 装配 → 确定性步进 →
//       订阅主题归档 → 检测 → 报告 → index 更新 → 退出码
import * as fs from 'node:fs';
import * as path from 'node:path';
import { execFileSync } from 'node:child_process';
import * as process from 'node:process';
import { MessageBus } from '../messageBus.js';
import { SimEngine, WorldConfig, FIXED_DT, PERCEPTION_HZ, PLANNING_HZ, GPS_HZ } from '../sim/engine.js';
import { parseScenario, buildScenarioWorld, Scenario } from '../sim/scenario.js';
import { ArchiveWriter, updateRunsIndex } from './archive.js';
import { FrameRecord, RunMeta, WorldFile, Issue, RunSummary } from '../shared/types.js';

interface CliArgs { scenario?: string; map?: string; obstacles?: number; seed: number;
  duration: number; out?: string; failOnIssue: boolean; }

export function parseArgs(argv: string[]): CliArgs {
  const a: CliArgs = { seed: Date.now() % 1000000, duration: 60, failOnIssue: false };
  for (let i = 0; i < argv.length; i++) {
    const k = argv[i];
    if (k === '--scenario') a.scenario = argv[++i];
    else if (k === '--map') a.map = argv[++i];
    else if (k === '--obstacles') a.obstacles = Number(argv[++i]);
    else if (k === '--seed') a.seed = Number(argv[++i]);
    else if (k === '--duration') a.duration = Number(argv[++i]);
    else if (k === '--out') a.out = argv[++i];
    else if (k === '--fail-on-issue') a.failOnIssue = true;
    else throw new Error(`未知参数: ${k}`);
  }
  return a;
}

/** 主题载荷 → 归档记录（纯映射，Task 11 复用） */
export function recordFrom(topic: string, p: any): FrameRecord {
  switch (topic) {
    case 'control:state': return { type: 'step', t: p.t, step: p.step,
      car: { x: p.snapshot.car.x, y: p.snapshot.car.y, angle: p.snapshot.car.angle,
             speed: p.snapshot.car.speed, steerAngle: p.snapshot.car.steerAngle },
      control: p.control };
    case 'perception:result': return { type: 'perception', t: p.t, step: p.step,
      scan: { angles: p.scan.angles, distances: p.scan.distances,
              rawDistances: p.scan.rawDistances, confidences: p.scan.confidences },
      stats: p.stats };
    case 'planning:result': {
      const pl = p.planning;
      return { type: 'plan', t: p.t, step: p.step, state: p.state,
        frenet: { s: pl?.frenetS ?? 0, l: pl?.frenetL ?? 0, dl_ds: pl?.dl_ds ?? 0, headingDiff: pl?.headingDiff ?? 0 },
        candidates: pl?.candidates ?? [],
        best: { cost: pl?.bestCost ?? 0, targetL: pl?.bestTargetL ?? 0, targetV: pl?.bestTargetV ?? 0 },
        trajectory: { points: p.trajectory.points.map((pt: any) => ({ t: pt.t, x: pt.x, y: pt.y, v: pt.v })),
                      type: p.trajectory.type, cost: p.trajectory.cost } };
    }
    case 'gps:result': return { type: 'gps', t: p.t, step: p.step,
      x: p.reading.x, y: p.reading.y,
      err: Math.hypot(p.reading.x - p.truth.x, p.reading.y - p.truth.y) };
    case 'sim:event': return { type: 'event', t: p.t, step: p.step, kind: p.kind, payload: p.payload };
    default: throw new Error(`未知主题 ${topic}`);
  }
}
```

`planning:result` 的 `frenet/candidates/best` 来自 planner 写入的 `debug.planning`（Task 3 的 engine 已在发布载荷中带 `planning` 快照，此处直接消费）。

主流程（同文件）：

```typescript
function toWorldFile(scenario: Scenario, engine: SimEngine, route: any): WorldFile {
  const pts = (arr: any[]): [number, number][] => arr.map(p => [p.x, p.y]);
  return {
    map: scenario.map,
    track: {
      centerPoints: pts(engine.track.centerPoints),
      innerBoundary: pts(engine.track.innerBoundary),
      outerBoundary: pts(engine.track.outerBoundary),
    },
    obstacles: engine.obstacles.map(o => ({
      x: o.x as number, y: o.y as number, width: o.width as number,
      height: o.height as number, angle: o.angle as number, zHeight: o.zHeight,
    })),
    route: route ? {
      waypoints: pts(route.waypoints), startIndex: route.startIndex,
      goalIndex: route.goalIndex, length: route.length as number,
    } : null,
  };
}

function main() {
  const args = parseArgs(process.argv.slice(2));
  // 场景：文件或 CLI 随机
  const scenario: Scenario = args.scenario
    ? parseScenario(JSON.parse(fs.readFileSync(args.scenario, 'utf8')))
    : parseScenario({
        version: 1, name: `cli-${args.map ?? 'oval'}`, map: args.map ?? 'oval',
        seed: args.seed, duration: args.duration,
        obstacles: { mode: 'random', count: args.obstacles ?? 4 },
      });

  const built = buildScenarioWorld(scenario, 1800, 800);
  const engine = new SimEngine(new MessageBus('headless', false), built.worldCfg);
  if (built.route) engine.setRoute(built.route, built.route.startIndex,
    { speed: built.worldCfg.carStart?.speed });

  const id = `${new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19)}_${scenario.name}_s${scenario.seed}`;
  const runsRoot = path.resolve('runs');
  const runDir = args.out ? path.resolve(args.out) : path.join(runsRoot, id);
  const archive = new ArchiveWriter(runDir);
  archive.writeWorld(toWorldFile(scenario, engine, built.route));

  // 订阅归档（同步总线：发布顺序 = 记录顺序）
  for (const topic of ['control:state', 'perception:result', 'planning:result', 'gps:result', 'sim:event']) {
    engine.bus.subscribe(topic, (p: any) => archive.appendRecord(recordFrom(topic, p)));
  }

  // 运行循环：duration 上限；到达后继续 1s（供停车检查）再停
  const maxSteps = Math.floor(scenario.duration * 60);
  let arrivedAtStep: number | null = null;
  engine.bus.subscribe('sim:event', (p: any) => { if (p.kind === 'arrived') arrivedAtStep = p.step; });
  while (engine.stepCount < maxSteps) {
    engine.step(FIXED_DT);
    if (arrivedAtStep !== null && engine.stepCount >= arrivedAtStep + 60) break;
  }

  // TODO(Task 7-11): 检测 + 报告；当前先写占位 verdict，保证 E2E 可用
  const meta: RunMeta = {
    version: 1, id: args.out ? path.basename(runDir) : id,
    scenario: scenario.name, map: scenario.map, seed: scenario.seed,
    durationS: engine.simTime, steps: engine.stepCount,
    gitCommit: (() => { try { return execFileSync('git', ['rev-parse', '--short', 'HEAD']).toString().trim(); } catch { return null; } })(),
    nodeVersion: process.version, createdAt: new Date().toISOString(),
    verdict: 'PASS', issueCounts: { high: 0, medium: 0, low: 0 },
  };
  archive.finalize({ issues: [], summary: { steps: engine.stepCount, simSeconds: engine.simTime } as RunSummary, reportMd: '# 占位（Task 11 生成）', meta });
  if (!args.out) {
    updateRunsIndex(runsRoot, { id: meta.id, scenario: scenario.name, map: scenario.map,
      seed: scenario.seed, createdAt: meta.createdAt, verdict: meta.verdict,
      issues: meta.issueCounts, durationS: meta.durationS });
  }
  console.log(`运行完成: ${runDir}  steps=${engine.stepCount}`);
}

main();
```

注意：`TODO(Task 7-11)` 占位在 Task 11 被替换为真实检测+报告调用——这是**任务边界处的显式衔接点**，本任务结束时 e2e 测试必须通过。

- [ ] **Step 5: 修改 package.json 与 .gitignore**

`package.json` scripts：

```json
"test": "tsc && node --test dist/js/test/",
"sim": "tsc && node dist/js/headless/run.js"
```

`.gitignore` 追加一行 `runs/`。

- [ ] **Step 6: 运行 E2E 与全部测试**

Run: `npm test`
Expected: PASS（含两个 E2E 用例；确认 `frames.jsonl` 逐字节一致）
Run 手工冒烟：`npm run sim -- --map oval --obstacles 4 --seed 7 --duration 5 --out runs/smoke`
Expected: 终端输出 `运行完成: .../runs/smoke  steps=300`；`runs/smoke/frames.jsonl` 行数 ≈ 300(step) + 100(perception) + 75(plan) + 50(gps) + 事件

- [ ] **Step 7: 提交**

```bash
git add js/headless/run.ts js/test/headless/e2e.test.ts package.json .gitignore
git commit -m "新增 headless CLI 入口（场景/随机双模式、JSONL 归档、确定性 E2E）"
```

---

### Task 7: 检测器核心 + 安全类检查

**Files:**
- Create: `js/headless/detector.ts`（核心：配置、issue 累积器、feed 分发）
- Create: `js/headless/checks/safety.ts`
- Test: `js/test/headless/detector-safety.test.ts`

**Interfaces:**
- Consumes: Task 5 记录类型与 `Issue`；`cartesianToFrenet/buildFrenetReference`；`TRACK_HALF_WIDTH`；`m/mps` 工厂
- Produces: `interface DetectorConfig`（各阈值，默认值如下）；`class Detector { constructor(cfg?: Partial<DetectorConfig>, world?: { trackFrenet; obstacles }); feed(rec: FrameRecord); finish(): Issue[]; summary(): RunSummaryPartial }`；检查注册按 `rec.type` 分发。Task 8/9/10 往同一 Detector 里加检查函数

- [ ] **Step 1: 写失败测试（安全三项：正/负例）**

创建 `js/test/headless/detector-safety.test.ts`：

```typescript
// ==================== 测试：检测器 —— 安全类 ====================
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { Detector } from '../../headless/detector.js';
import { FrameRecord } from '../../shared/types.js';
import { m, px, rad } from '../../shared/dimensions.js';

// 直线赛道：x 轴 0..2000，l=0 为中心线（法向量 (0,1)，l>0 向内）
function straightTrack() {
  const pts: any[] = [];
  for (let i = 0; i <= 200; i++) pts.push({ x: px(i * 10), y: px(0) });
  pts.push(pts[0]);
  return { centerPoints: pts, innerBoundary: [], outerBoundary: [] };
}

function stepRec(step: number, x: number, y: number, speed = 250): FrameRecord {
  return { type: 'step', t: step / 60, step,
    car: { x, y, angle: 0, speed, steerAngle: 0 }, control: null };
}

describe('safety: collision_true', () => {
  it('车身圆与障碍物相交 → 出 issue（high）', () => {
    const det = new Detector({}, { track: straightTrack() as any, obstacles: [
      { x: px(120), y: px(0), width: m(2), height: m(4), angle: rad(0) } ] });
    for (let s = 1; s <= 30; s++) det.feed(stepRec(s, px(100 + s * 4), px(0))); // 逐步压上障碍
    const issues = det.finish();
    const col = issues.find(i => i.check === 'collision_true');
    assert.ok(col, '未检测到碰撞');
    assert.strictEqual(col!.severity, 'high');
    assert.ok(col!.firstFrame >= 1);
  });

  it('远离障碍物 → 无碰撞 issue', () => {
    const det = new Detector({}, { track: straightTrack() as any, obstacles: [
      { x: px(120), y: px(500), width: m(2), height: m(4), angle: rad(0) } ] });
    for (let s = 1; s <= 30; s++) det.feed(stepRec(s, px(100 + s * 4), px(0)));
    assert.strictEqual(det.finish().find(i => i.check === 'collision_true'), undefined);
  });
});

describe('safety: off_track', () => {
  it('|l| 超半宽 → issue；中线附近 → 无', () => {
    const t = straightTrack();
    const off = new Detector({}, { track: t as any, obstacles: [] });
    off.feed(stepRec(1, px(100), px(m(9))));   // l > 7.5m
    assert.ok(off.finish().find(i => i.check === 'off_track'));

    const ok = new Detector({}, { track: t as any, obstacles: [] });
    ok.feed(stepRec(1, px(100), px(0)));
    assert.strictEqual(ok.finish().find(i => i.check === 'off_track'), undefined);
  });
});

describe('safety: emergency_state', () => {
  it('EMERGENCY 规划状态区间 → issue（medium）', () => {
    const det = new Detector({}, { track: straightTrack() as any, obstacles: [] });
    const plan = (step: number, state: string): FrameRecord => ({ type: 'plan', t: step / 60, step, state,
      frenet: { s: 0, l: 0, dl_ds: 0, headingDiff: 0 }, candidates: [],
      best: { cost: 0, targetL: 0, targetV: 0 },
      trajectory: { points: [], type: 'emergency', cost: 0 } });
    for (let s = 1; s <= 3; s++) det.feed(plan(s, 'EMERGENCY'));
    for (let s = 4; s <= 10; s++) det.feed(plan(s, 'CRUISE'));
    const issues = det.finish();
    assert.ok(issues.find(i => i.check === 'emergency_state'));
    assert.strictEqual(issues.find(i => i.check === 'emergency_state')!.count, 3);
  });
});
```

- [ ] **Step 2: 运行确认失败**

Run: `npx tsc && node --test dist/js/test/headless/detector-safety.test.js`
Expected: FAIL（Cannot find module '../../headless/detector.js'）

- [ ] **Step 3: 实现 detector 核心**

创建 `js/headless/detector.ts`：

```typescript
// ==================== Headless：问题检测器（核心） ====================
//
// 【形态】纯模块：feed(记录流) 累积 → finish() 产出 Issue[]。
// 线程内流式单遍，无 I/O；可用合成记录做单元测试。
//
// 【issue 合并】同一 check 连续命中（间隔 ≤ mergeGapSteps）合并为一个
// issue 区间（firstFrame..lastFrame + count + samples）。
import { FrameRecord, Issue, Obstacle, TrackGeometry } from '../shared/types.js';
import { Px, PxPerSec, Seconds, DimLess, m, mps, secs, dimless } from '../shared/dimensions.js';
import { buildFrenetReference, cartesianToFrenet } from '../algo/plan/frenet.js';
import { runSafetyChecks } from './checks/safety.js';

export interface DetectorConfig {
  // 安全
  // 感知
  gpsSigma: Px;              // m(1.25)
  gpsThresholdSigma: number; // 3
  phantomTolerance: Px;      // m(1.0)
  phantomRate: number;       // 0.01
  // 规划行为
  closeCallDist: Px;         // m(10)
  closeCallSpeed: PxPerSec;  // mps(5)
  flappingWindowSteps: number;  // 60（1s）
  flappingCount: number;        // 4
  lateralJump: Px;              // m(10)
  lateralWindowPlans: number;   // 15
  lateralCount: number;         // 3
  // 跟踪/任务
  trackingErr: Px;              // m(1.0)
  stallSpeed: PxPerSec;         // mps(0.5)
  stallSteps: number;           // 180
  arriveSpeed: PxPerSec;        // mps(1)
  // 内部
  mergeGapSteps: number;        // 30
}

export const DEFAULT_DETECTOR_CONFIG: DetectorConfig = {
  gpsSigma: m(1.25), gpsThresholdSigma: 3,
  phantomTolerance: m(1.0), phantomRate: 0.01,
  closeCallDist: m(10), closeCallSpeed: mps(5),
  flappingWindowSteps: 60, flappingCount: 4,
  lateralJump: m(10), lateralWindowPlans: 15, lateralCount: 3,
  trackingErr: m(1.0), stallSpeed: mps(0.5), stallSteps: 180, arriveSpeed: mps(1),
  mergeGapSteps: 30,
};

export interface DetectorWorld { track: TrackGeometry; obstacles: Obstacle[]; }

/** 区间合并器：同 check 的离散命中 → 连续区间 issue */
export class IssueMerger {
  private _open = new Map<string, { issue: Issue; lastStep: number }>();
  constructor(private cfg: DetectorConfig) {}

  add(partial: { check: string; category: Issue['category']; severity: Issue['severity'];
    step: number; summary: string; details?: Record<string, unknown> }): void {
    const cur = this._open.get(partial.check);
    if (cur && partial.step - cur.lastStep <= this.cfg.mergeGapSteps) {
      cur.issue.lastFrame = partial.step;
      cur.issue.count += 1;
      if (!cur.issue.samples.includes(partial.step) && cur.issue.samples.length < 3) {
        cur.issue.samples.push(partial.step);
      }
      cur.lastStep = partial.step;
    } else if (cur) {
      this._closed.push(cur.issue);
      this._open.set(partial.check, { issue: this._newIssue(partial), lastStep: partial.step });
    } else {
      this._open.set(partial.check, { issue: this._newIssue(partial), lastStep: partial.step });
    }
  }

  private _closed: Issue[] = [];
  private _newIssue(p: any): Issue {
    return { category: p.category, severity: p.severity, check: p.check,
      firstFrame: p.step, lastFrame: p.step, count: 1,
      summary: p.summary, details: p.details ?? {}, samples: [p.step] };
  }
  finish(): Issue[] { return [...this._closed, ...[...this._open.values()].map(v => v.issue)]; }
}

export class Detector {
  readonly cfg: DetectorConfig;
  readonly world: DetectorWorld;
  readonly merger: IssueMerger;
  readonly trackFrenet: ReturnType<typeof buildFrenetReference>;
  // 供各检查模块读取的累积状态
  lastStepRec: any = null;
  lastPlanRec: any = null;
  stepCount = 0;

  constructor(cfg?: Partial<DetectorConfig>, world?: Partial<DetectorWorld>) {
    this.cfg = { ...DEFAULT_DETECTOR_CONFIG, ...cfg };
    this.world = { track: world?.track ?? { centerPoints: [], innerBoundary: [], outerBoundary: [] }, obstacles: world?.obstacles ?? [] };
    this.merger = new IssueMerger(this.cfg);
    this.trackFrenet = buildFrenetReference(this.world.track.centerPoints, true);
  }

  feed(rec: FrameRecord): void {
    runSafetyChecks(this, rec);
    // Task 8/9/10: runPerceptionChecks(this, rec); runPlanningChecks(...); runMissionChecks(...)
  }

  finish(): Issue[] { return this.merger.finish(); }
}
```

- [ ] **Step 4: 实现安全类检查**

创建 `js/headless/checks/safety.ts`：

```typescript
// ==================== 检查：安全类（碰撞 / 越界 / EMERGENCY） ====================
import { FrameRecord, Issue } from '../../shared/types.js';
import { Detector } from '../detector.js';
import { cartesianToFrenet } from '../../algo/plan/frenet.js';
import { TRACK_HALF_WIDTH } from '../../env/track.js';
import { px } from '../../shared/dimensions.js';

/** 圆（车身）与矩形（障碍物 OBB）相交判定（局部坐标 clamp 法） */
export function circleRectOverlap(cx: number, cy: number, r: number, obs: { x: number; y: number; width: number; height: number; angle: number }): boolean {
  const c = Math.cos(-obs.angle), s = Math.sin(-obs.angle);
  const dx = cx - obs.x, dy = cy - obs.y;
  const lx = dx * c - dy * s, ly = dx * s + dy * c;   // 转到障碍物局部系
  const hw = obs.width / 2, hh = obs.height / 2;
  const nx = Math.max(-hw, Math.min(hw, lx));
  const ny = Math.max(-hh, Math.min(hh, ly));
  return Math.hypot(lx - nx, ly - ny) <= r;
}

export function runSafetyChecks(det: Detector, rec: FrameRecord): void {
  if (rec.type === 'step') {
    det.stepCount = rec.step;
    det.lastStepRec = rec;
    // ① 碰撞：车身圆 vs 真值障碍物
    const car = rec.car;
    for (const obs of det.world.obstacles) {
      void obs;
    }
    // ↑ 改为真实实现：对每个 obstacle 调 circleRectOverlap(car.x, car.y, collisionRadius, obs)
    //   collisionRadius 从车身尺寸算：Math.hypot(m(2), m(4.5))/2（与 createCar 一致）
  }
  if (rec.type === 'plan') {
    det.lastPlanRec = rec;
    // ③ EMERGENCY 区间
    // if (rec.state === 'EMERGENCY') det.merger.add({ check: 'emergency_state', ... });
  }
}
```

实现说明（按此完成，不留注释占位）：
- 车身碰撞半径：`Math.hypot(m(2.0) as number, m(4.5) as number) / 2`（与 createCar 一致）。命中时 `merger.add({ check:'collision_true', category:'safety', severity:'high', step: rec.step, summary: `车身与障碍物相交 (speed=${(rec.car.speed/4).toFixed(1)}m/s)`, details:{ carX, carY } })`
- ② 越界：对 step 记录 `cartesianToFrenet(px(rec.car.x), px(rec.car.y), det.trackFrenet)`，`Math.abs(l) > TRACK_HALF_WIDTH` → `off_track`（severity high，summary 含 |l| 米数）
- ③ EMERGENCY：plan 记录 `state === 'EMERGENCY'` → `emergency_state`（severity medium）

- [ ] **Step 5: 运行测试确认通过**

Run: `npx tsc && node --test dist/js/test/headless/detector-safety.test.js`
Expected: PASS（4 个用例）

- [ ] **Step 6: 提交**

```bash
git add js/headless/detector.ts js/headless/checks/safety.ts js/test/headless/detector-safety.test.ts
git commit -m "新增检测器核心与安全类检查（碰撞/越界/EMERGENCY 区间合并）"
```

---

### Task 8: 感知类检查

**Files:**
- Create: `js/headless/checks/perception.ts`
- Modify: `js/headless/detector.ts`（feed 中调用 `runPerceptionChecks`）
- Test: `js/test/headless/detector-perception.test.ts`

**Interfaces:**
- Consumes: `Detector`（Task 7）；`obstacleToSegments`（`shared/types.js`）；`MAX_SENSOR_DIST`（`algo/perception.js`）
- Produces: `runPerceptionChecks(det, rec)`；检查名 `phantom_hit` / `missed_obstacle` / `gps_error`

- [ ] **Step 1: 写失败测试**

创建 `js/test/headless/detector-perception.test.ts`：构造直线赛道 + 一个真值障碍物（x=200,y=0），合成 perception 记录：

```typescript
// ==================== 测试：检测器 —— 感知类 ====================
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { Detector } from '../../headless/detector.js';
import { FrameRecord } from '../../shared/types.js';
import { m, px, rad } from '../../shared/dimensions.js';

// —— 辅助 ——
function straightTrack() {
  const pts: any[] = [];
  for (let i = 0; i <= 200; i++) pts.push({ x: px(i * 10), y: px(0) });
  pts.push(pts[0]);
  return { centerPoints: pts, innerBoundary: [], outerBoundary: [] };
}
const OBST = { x: px(200), y: px(0), width: m(2), height: m(4), angle: rad(0) };

// 61 条射线：angle = -60°..60°（步进 2°），默认全 MAX
function scanRec(step: number, carX: number, carY: number, hits: Record<number, number>): FrameRecord {
  const n = 61, angles: any[] = [], distances: any[] = [], confs: number[] = [];
  for (let i = 0; i < n; i++) {
    const a = (-60 + i * 2) * Math.PI / 180;
    angles.push(rad(a));
    const d = hits[i] !== undefined ? px(hits[i]) : px(500);
    distances.push(d); confs.push(hits[i] !== undefined ? 1 : 0);
  }
  return { type: 'perception', t: step / 20, step,
    scan: { angles, distances, rawDistances: [...distances], confidences: confs }, stats: null };
}

describe('perception: phantom_hit', () => {
  it('过滤后命中点附近无真值障碍 → issue', () => {
    const det = new Detector({}, { track: straightTrack() as any, obstacles: [] });
    // 车在原点朝 +x；第 30 条射线 = 0°（正前方），报 100px 处有障碍，但真值无障碍
    det.feed(scanRec(1, 0, 0, { 30: 100 }));
    det.feed(scanRec(2, 0, 0, { 30: 100 }));
    const iss = det.finish().find(i => i.check === 'phantom_hit');
    assert.ok(iss, '未检出幻影命中');
  });

  it('命中点附近有真值障碍 → 不报', () => {
    const det = new Detector({}, { track: straightTrack() as any, obstacles: [OBST] });
    // 车在 x=100, 障害物在 x=200 → 0° 射线 100px 命中真值障碍
    det.feed(scanRec(1, 100, 0, { 30: 100 }));
    assert.strictEqual(det.finish().find(i => i.check === 'phantom_hit'), undefined);
  });
});

describe('perception: missed_obstacle', () => {
  it('真值障碍在 FOV 内却无命中 → issue', () => {
    const det = new Detector({}, { track: straightTrack() as any, obstacles: [OBST] });
    det.feed(scanRec(1, 100, 0, {}));   // 前方 100px 有障碍，所有射线却报 500
    assert.ok(det.finish().find(i => i.check === 'missed_obstacle'));
  });

  it('障碍在 FOV 外（车后）→ 不报', () => {
    const det = new Detector({}, { track: straightTrack() as any, obstacles: [OBST] });
    det.feed(scanRec(1, 500, 0, {}));   // 障碍在车后
    assert.strictEqual(det.finish().find(i => i.check === 'missed_obstacle'), undefined);
  });
});

describe('perception: gps_error', () => {
  it('误差 > 3σ → issue', () => {
    const det = new Detector({}, { track: straightTrack() as any, obstacles: [] });
    const gps: FrameRecord = { type: 'gps', t: 0.1, step: 1, x: px(500), y: px(0), err: m(5) };
    det.feed(gps);
    assert.ok(det.finish().find(i => i.check === 'gps_error'));
  });
});
```

- [ ] **Step 2: 运行确认失败**

Run: `npx tsc && node --test dist/js/test/headless/detector-perception.test.js`
Expected: FAIL

- [ ] **Step 3: 实现 checks/perception.ts**

```typescript
// ==================== 检查：感知类（幻影 / 漏检 / GPS 误差） ====================
//
// 【幻影命中】过滤后（conf ≥ 0.5）的命中点换算到世界坐标，若附近
// 容差内没有任何真值障碍物 → 感知层报了不存在的障碍（边界过滤失误）。
//
// 【漏检】真值障碍落在车头 ±FOV 内且距离 < MAX_SENSOR_DIST，
// 但对应角度扇区（障碍角宽 ± 2° 容差）没有任何射线命中 → 感知丢失目标。
import { FrameRecord, Obstacle } from '../../shared/types.js';
import { Detector } from '../detector.js';
import { MAX_SENSOR_DIST } from '../../algo/perception.js';
import { m } from '../../shared/dimensions.js';

const FOV = Math.PI * 2 / 3;           // ±60°
const CONF_FILTER = 0.5;

/** 点到矩形（OBB）的距离（内部 0） */
function pointRectDist(px0: number, py0: number, o: Obstacle): number {
  const c = Math.cos(-(o.angle as number)), s = Math.sin(-(o.angle as number));
  const dx = px0 - (o.x as number), dy = py0 - (o.y as number);
  const lx = dx * c - dy * s, ly = dx * s + dy * c;
  const hw = (o.width as number) / 2, hh = (o.height as number) / 2;
  const ox = Math.max(Math.abs(lx) - hw, 0), oy = Math.max(Math.abs(ly) - hh, 0);
  return Math.hypot(ox, oy);
}

export function runPerceptionChecks(det: Detector, rec: FrameRecord): void {
  if (rec.type === 'perception') {
    const car = det.lastStepRec?.car;      // 由 safety 检查先行更新
    if (!car) return;
    const { angles, distances, confidences } = rec.scan;
    const tolerance = det.cfg.phantomTolerance as number;

    // ④ 幻影命中
    for (let i = 0; i < angles.length; i++) {
      if ((confidences[i] ?? 0) < CONF_FILTER) continue;
      const d = distances[i];
      if (d >= (MAX_SENSOR_DIST as number) - 1) continue;
      const hx = car.x + Math.cos(angles[i]) * d;
      const hy = car.y + Math.sin(angles[i]) * d;
      const near = det.world.obstacles.some(o => pointRectDist(hx, hy, o) <= tolerance);
      if (!near) {
        det.merger.add({ check: 'phantom_hit', category: 'perception', severity: 'medium',
          step: rec.step, summary: `射线 ${(angles[i] * 180 / Math.PI).toFixed(0)}° 命中 ${(d / 4).toFixed(1)}m 处但无真值障碍`,
          details: { hitX: hx, hitY: hy } });
      }
    }

    // ⑤ 漏检：逐真值障碍，检查 FOV 内是否被照射
    for (const o of det.world.obstacles) {
      const dx = (o.x as number) - car.x, dy = (o.y as number) - car.y;
      const dist = Math.hypot(dx, dy);
      if (dist >= (MAX_SENSOR_DIST as number)) continue;
      const rel = Math.atan2(dy, dx) - car.angle;          // 相对车头
      const relNorm = Math.atan2(Math.sin(rel), Math.cos(rel));
      if (Math.abs(relNorm) > FOV / 2 - 0.05) continue;    // FOV 外
      const halfAngle = Math.atan2((o.width as number) / 2, Math.max(dist - (o.width as number) / 2, 1)) + 0.035;
      let seen = false;
      for (let i = 0; i < angles.length; i++) {
        const ra = angles[i];
        if (Math.abs(ra - relNorm) > halfAngle) continue;
        const expected = dist - (o.height as number) / 2;
        if ((distances[i] as number) <= expected + (m(0.5) as number)) { seen = true; break; }
      }
      if (!seen) {
        det.merger.add({ check: 'missed_obstacle', category: 'perception', severity: 'medium',
          step: rec.step, summary: `真值障碍 ${(dist / 4).toFixed(1)}m 处未被感知`, details: { obstacleX: o.x, obstacleY: o.y } });
      }
    }
  }

  // ⑥ GPS 误差超限
  if (rec.type === 'gps') {
    const threshold = (det.cfg.gpsSigma as number) * det.cfg.gpsThresholdSigma;
    if (rec.err > threshold) {
      det.merger.add({ check: 'gps_error', category: 'perception', severity: 'low',
        step: rec.step, summary: `GPS 误差 ${(rec.err / 4).toFixed(2)}m（> ${det.cfg.gpsThresholdSigma}σ）`,
        details: { err: rec.err } });
    }
  }
}
```

同时把 `js/headless/detector.ts` 的 `feed()` 加入 `runPerceptionChecks(this, rec);`。

注意阈值单位混用（px 与 m）已在各比较处转换；`halfAngle` 里的 `0.035` 是 ~2° 容差（实现时可提为配置常量）。

- [ ] **Step 4: 运行测试确认通过**

Run: `npx tsc && node --test dist/js/test/headless/detector-perception.test.js`
Expected: PASS（5 个用例）

- [ ] **Step 5: 提交**

```bash
git add js/headless/checks/perception.ts js/headless/detector.ts js/test/headless/detector-perception.test.ts
git commit -m "新增感知类检查（幻影命中/漏检/GPS 超限）"
```

---

### Task 9: 规划行为类检查

**Files:**
- Create: `js/headless/checks/planning.ts`
- Modify: `js/headless/detector.ts`（feed 中调用）
- Test: `js/test/headless/detector-planning.test.ts`

**Interfaces:**
- Consumes: `Detector`；`EventRecord`（state_change 来自 engine 的 `sim:event`）
- Produces: `runPlanningChecks(det, rec)`；检查名 `state_flapping` / `lateral_switching` / `close_call`

- [ ] **Step 1: 写失败测试**

创建 `js/test/headless/detector-planning.test.ts`（要点）：

```typescript
// ==================== 测试：检测器 —— 规划行为类 ====================
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { Detector } from '../../headless/detector.js';
import { FrameRecord } from '../../shared/types.js';
import { m, px } from '../../shared/dimensions.js';

function straightTrack() {
  const pts: any[] = [];
  for (let i = 0; i <= 200; i++) pts.push({ x: px(i * 10), y: px(0) });
  pts.push(pts[0]);
  return { centerPoints: pts, innerBoundary: [], outerBoundary: [] };
}
const event = (step: number, kind: string, payload: any = {}): FrameRecord =>
  ({ type: 'event', t: step / 60, step, kind, payload });

describe('planning: state_flapping', () => {
  it('1s 窗口内状态切换 ≥ 4 次 → issue', () => {
    const det = new Detector({}, { track: straightTrack() as any, obstacles: [] });
    // 每 10 step 切换一次，共 5 次（40 个 step 内）
    for (let i = 0; i < 5; i++) det.feed(event(10 + i * 10, 'state_change', { from: i % 2 ? 'AVOID' : 'CRUISE', to: i % 2 ? 'CRUISE' : 'AVOID' }));
    assert.ok(det.finish().find(i => i.check === 'state_flapping'));
  });

  it('偶发一次切换 → 不报', () => {
    const det = new Detector({}, { track: straightTrack() as any, obstacles: [] });
    det.feed(event(10, 'state_change', { from: 'CRUISE', to: 'AVOID' }));
    assert.strictEqual(det.finish().find(i => i.check === 'state_flapping'), undefined);
  });
});

describe('planning: close_call', () => {
  it('最近命中 <10m 且速度 >5m/s → issue；含紧急制动事件单独记录', () => {
    const det = new Detector({}, { track: straightTrack() as any, obstacles: [] });
    det.feed({ type: 'step', t: 0.1, step: 1,
      car: { x: px(0), y: px(0), angle: 0, speed: 250, steerAngle: 0 }, control: null });
    det.feed({ type: 'perception', t: 0.1, step: 1,
      scan: { angles: [], distances: [], rawDistances: [], confidences: [] },
      stats: { rawHits: 1, filteredHits: 1, frontDist: 100, minDist: px(30) } });  // 7.5m
    det.feed(event(2, 'emergency_brake', { frontDist: 30 }));
    const issues = det.finish();
    assert.ok(issues.find(i => i.check === 'close_call'));
    assert.ok(issues.find(i => i.check === 'emergency_brake'));
  });
});

describe('planning: lateral_switching', () => {
  it('15 个规划帧内最优 targetL 大幅摆动 ≥3 次 → issue', () => {
    const det = new Detector({}, { track: straightTrack() as any, obstacles: [] });
    const targets = [0, m(28), 0, m(-28), 0].map(v => v as number);  // ±7m 跳变
    targets.forEach((tL, i) => det.feed({ type: 'plan', t: i / 15, step: i * 4, state: 'AVOID',
      frenet: { s: 0, l: 0, dl_ds: 0, headingDiff: 0 }, candidates: [],
      best: { cost: 1, targetL: tL, targetV: 100 },
      trajectory: { points: [], type: 'avoid_left', cost: 1 } }));
    assert.ok(det.finish().find(i => i.check === 'lateral_switching'));
  });
});
```

- [ ] **Step 2: 运行确认失败**

Run: `npx tsc && node --test dist/js/test/headless/detector-planning.test.js`
Expected: FAIL

- [ ] **Step 3: 实现 checks/planning.ts**

实现三个检查（完整逻辑）：

```typescript
// ==================== 检查：规划行为类（抖动 / 摆动 / 近距险情） ====================
import { FrameRecord } from '../../shared/types.js';
import { Detector } from '../detector.js';

export function runPlanningChecks(det: Detector, rec: FrameRecord): void {
  // ⑦ 状态机抖动：滑动窗口计数 state_change 事件
  if (rec.type === 'event' && rec.kind === 'state_change') {
    det.stateChangeSteps.push(rec.step);
    const win = det.cfg.flappingWindowSteps;
    det.stateChangeSteps = det.stateChangeSteps.filter(s => rec.step - s <= win);
    if (det.stateChangeSteps.length >= det.cfg.flappingCount) {
      det.merger.add({ check: 'state_flapping', category: 'planning', severity: 'medium',
        step: rec.step,
        summary: `1s 窗口内状态切换 ${det.stateChangeSteps.length} 次`,
        details: { windowSteps: win } });
    }
  }

  // ⑧ 横向摆动：相邻规划帧最优 targetL 跳变幅度累计
  if (rec.type === 'plan') {
    if (det.lastPlanTargetL !== null) {
      const jump = Math.abs(rec.best.targetL - det.lastPlanTargetL);
      if (jump > (det.cfg.lateralJump as number)) {
        det.lateralJumps.push({ step: rec.step, jump });
        const plans = det.cfg.lateralWindowPlans;
        det.lateralJumps = det.lateralJumps.filter(j => rec.step - j.step <= plans * 4);  // 规划帧≈每 4 step 一帧
        if (det.lateralJumps.length >= det.cfg.lateralCount) {
          det.merger.add({ check: 'lateral_switching', category: 'planning', severity: 'medium',
            step: rec.step,
            summary: `${plans} 个规划帧内横向目标跳变 ${det.lateralJumps.length} 次`,
            details: { jumps: det.lateralJumps.map(j => (j.jump / 4).toFixed(1)) } });
        }
      }
    }
    det.lastPlanTargetL = rec.best.targetL;
  }

  // ⑨ 近距险情：最近过滤命中 < 10m 且车速 > 5m/s
  if (rec.type === 'perception' && rec.stats && det.lastStepRec) {
    const minD = rec.stats.minDist;
    const speed = det.lastStepRec.car.speed;
    if (minD < (det.cfg.closeCallDist as number) && speed > (det.cfg.closeCallSpeed as number)) {
      det.merger.add({ check: 'close_call', category: 'planning', severity: 'high',
        step: rec.step,
        summary: `最近障碍 ${(minD / 4).toFixed(1)}m，时速 ${(speed / 4 * 3.6).toFixed(0)}km/h`,
        details: { minDist: minD, speed } });
    }
  }

  // ⑨b 紧急制动事件
  if (rec.type === 'event' && rec.kind === 'emergency_brake') {
    det.merger.add({ check: 'emergency_brake', category: 'planning', severity: 'medium',
      step: rec.step, summary: `触发紧急制动（前方 ${(Number(rec.payload.frontDist) / 4).toFixed(1)}m）`,
      details: rec.payload });
  }
}
```

配套修改 `js/headless/detector.ts`：`Detector` 类增加累积字段 `stateChangeSteps: number[] = []; lateralJumps: {step:number;jump:number}[] = []; lastPlanTargetL: number | null = null;`，`feed()` 中加入 `runPlanningChecks(this, rec);`。

- [ ] **Step 4: 运行测试确认通过**

Run: `npx tsc && node --test dist/js/test/headless/detector-planning.test.js`
Expected: PASS（4 个用例）

- [ ] **Step 5: 提交**

```bash
git add js/headless/checks/planning.ts js/headless/detector.ts js/test/headless/detector-planning.test.ts
git commit -m "新增规划行为类检查（状态抖动/横向摆动/近距险情）"
```

---

### Task 10: 跟踪/任务类检查

**Files:**
- Create: `js/headless/checks/mission.ts`
- Modify: `js/headless/detector.ts`（feed 中调用 + summary 累积）
- Test: `js/test/headless/detector-mission.test.ts`

**Interfaces:**
- Consumes: `Detector`；plan 记录的 `trajectory.points` 与下一条 plan 记录的 `car`（经 `lastStepRec`）
- Produces: `runMissionChecks(det, rec)`；检查名 `tracking_error` / `route_timeout` / `stop_anomaly` / `stalled`

- [ ] **Step 1: 写失败测试**

创建 `js/test/headless/detector-mission.test.ts`（要点）：

```typescript
// ==================== 测试：检测器 —— 跟踪/任务类 ====================
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { Detector } from '../../headless/detector.js';
import { FrameRecord } from '../../shared/types.js';
import { m, px } from '../../shared/dimensions.js';

function straightTrack() {
  const pts: any[] = [];
  for (let i = 0; i <= 200; i++) pts.push({ x: px(i * 10), y: px(0) });
  pts.push(pts[0]);
  return { centerPoints: pts, innerBoundary: [], outerBoundary: [] };
}
// 车沿 +x 直线；轨迹预测 60px/s 前进
const traj = (step: number): FrameRecord => ({ type: 'plan', t: step / 60, step, state: 'FOLLOW',
  frenet: { s: 0, l: 0, dl_ds: 0, headingDiff: 0 }, candidates: [],
  best: { cost: 1, targetL: 0, targetV: 60 },
  trajectory: { points: [{ t: 0, x: 0, y: 0, v: 60 }, { t: 2, x: 120, y: 0, v: 60 }], type: 'lane_keep', cost: 1 } });
const stepRec = (step: number, x: number, y: number, speed = 60, steer = 0): FrameRecord =>
  ({ type: 'step', t: step / 60, step, car: { x, y, angle: 0, speed, steerAngle: steer }, control: null });

describe('mission: tracking_error', () => {
  it('实际位置偏离上一规划轨迹 > 1m → issue', () => {
    const det = new Detector({}, { track: straightTrack() as any, obstacles: [] });
    det.feed(stepRec(1, 0, 0));
    det.feed(traj(2));                       // 规划：2s 后应到 x=120（60px/s）
    det.feed(stepRec(6, 30, px(m(3) as number)));  // 0.1s 后应在 ~x=6，实际 x=30 且横向偏 3m
    assert.ok(det.finish().find(i => i.check === 'tracking_error'));
  });

  it('跟得上规划 → 不报', () => {
    const det = new Detector({}, { track: straightTrack() as any, obstacles: [] });
    det.feed(stepRec(1, 0, 0));
    det.feed(traj(2));
    det.feed(stepRec(6, 6, 0));              // 0.1s × 60px/s ≈ 6px ✓
    assert.strictEqual(det.finish().find(i => i.check === 'tracking_error'), undefined);
  });
});

describe('mission: stop_anomaly / stalled', () => {
  it('有路线且速度 <0.5m/s 持续 3s → stalled issue', () => {
    const det = new Detector({}, { track: straightTrack() as any, obstacles: [] });
    det.routeActive = true;
    for (let s = 1; s <= 190; s++) det.feed(stepRec(s, 0, 0, 0));
    const iss = det.finish();
    assert.ok(iss.find(i => i.check === 'stalled'));
  });

  it('到达后 1s 内速度仍 >1m/s → stop_anomaly', () => {
    const det = new Detector({}, { track: straightTrack() as any, obstacles: [] });
    det.feed({ type: 'event', t: 0.1, step: 6, kind: 'arrived', payload: {} });
    for (let s = 7; s <= 60; s++) det.feed(stepRec(s, 0, 0, px(80)));  // 20m/s
    assert.ok(det.finish().find(i => i.check === 'stop_anomaly'));
  });
});
```

- [ ] **Step 2: 运行确认失败**

Run: `npx tsc && node --test dist/js/test/headless/detector-mission.test.js`
Expected: FAIL

- [ ] **Step 3: 实现 checks/mission.ts**

```typescript
// ==================== 检查：跟踪/任务类（跟踪误差 / 超时 / 停车异常） ====================
//
// 【跟踪误差】规划帧 k 的轨迹在 t(k+1) 时刻按时间线性插值出期望位置，
// 与下一规划帧时车辆实际位置比较；位置偏差 > 阈值记超限帧（累计 RMSE 进 summary）。
import { FrameRecord } from '../../shared/types.js';
import { Detector } from '../detector.js';

export function runMissionChecks(det: Detector, rec: FrameRecord): void {
  if (rec.type === 'step') {
    const speed = rec.car.speed;
    // ⑫a 停滞：有路线（由 runner 置 det.routeActive）且持续低速
    if (det.routeActive) {
      if (speed < (det.cfg.stallSpeed as number)) det.stallRun += 1;
      else det.stallRun = 0;
      if (det.stallRun === det.cfg.stallSteps) {
        det.merger.add({ check: 'stalled', category: 'mission', severity: 'high',
          step: rec.step, summary: `有路线但停滞 3s（速度 < 0.5m/s）`, details: { speed } });
      }
    }
    // ⑫b 到达后未及时停车
    if (det.arrivedAtStep !== null && rec.step - det.arrivedAtStep <= 60
        && speed > (det.cfg.arriveSpeed as number)) {
      det.merger.add({ check: 'stop_anomaly', category: 'mission', severity: 'medium',
        step: rec.step, summary: `ARRIVED 后速度仍为 ${(speed / 4 * 3.6).toFixed(0)}km/h`, details: { speed } });
    }
  }

  if (rec.type === 'event' && rec.kind === 'arrived') {
    det.arrivedAtStep = rec.step;
  }

  if (rec.type === 'plan') {
    // ⑩ 跟踪误差：上一规划帧轨迹 → 按时间插值到当前时刻
    if (det.prevTraj && det.prevPlanStep !== null && det.lastStepRec) {
      const dt = (rec.step - det.prevPlanStep) / 60;   // 秒
      const pts = det.prevTraj.points;
      let exp: { x: number; y: number } | null = null;
      if (pts.length >= 2 && dt <= pts[pts.length - 1].t) {
        for (let i = 0; i < pts.length - 1; i++) {
          if (dt >= pts[i].t && dt <= pts[i + 1].t) {
            const f = (dt - pts[i].t) / Math.max(pts[i + 1].t - pts[i].t, 1e-9);
            exp = { x: pts[i].x + f * (pts[i + 1].x - pts[i].x),
                    y: pts[i].y + f * (pts[i + 1].y - pts[i].y) };
            break;
          }
        }
      }
      if (exp) {
        const car = det.lastStepRec.car;
        const err = Math.hypot(car.x - exp.x, car.y - exp.y);
        det.trackingErrSum += err * err; det.trackingErrN += 1;
        if (err > (det.cfg.trackingErr as number)) {
          det.merger.add({ check: 'tracking_error', category: 'mission', severity: 'medium',
            step: rec.step, summary: `跟踪偏差 ${(err / 4).toFixed(2)}m`, details: { err } });
        }
      }
    }
    det.prevTraj = rec.trajectory;
    det.prevPlanStep = rec.step;
  }
}
```

配套修改 `js/headless/detector.ts`：字段 `routeActive = false; stallRun = 0; arrivedAtStep: number | null = null; prevTraj: any = null; prevPlanStep: number | null = null; trackingErrSum = 0; trackingErrN = 0;`；`feed()` 加入 `runMissionChecks(this, rec);`。

⑪ 路线超时不在流内检测（需要「运行结束仍 ACTIVE」这一终局信息）：在 `finish()` 里判断 `if (this.routeActive && this.arrivedAtStep === null)` 追加 `route_timeout` issue（severity medium，details 用 `lastPlanRec.frenet.s` 与 route 长度差值——route 总长由 runner 通过 `det.routeLength` 注入）。

- [ ] **Step 4: 运行测试确认通过**

Run: `npx tsc && node --test dist/js/test/headless/detector-mission.test.js`
Expected: PASS（4 个用例）

- [ ] **Step 5: 提交**

```bash
git add js/headless/checks/mission.ts js/headless/detector.ts js/test/headless/detector-mission.test.ts
git commit -m "新增跟踪/任务类检查（跟踪误差/停滞/停车异常/超时）"
```

---

### Task 11: 报告生成 + CLI 集成 + 示例场景

**Files:**
- Create: `js/headless/report.ts`
- Create: `scenarios/oval-cruise.json`、`scenarios/oval-avoid-1.json`、`scenarios/hairpin-route.json`
- Modify: `js/headless/run.ts`（替换 TODO 占位：接入 Detector/报告/verdict/退出码）
- Modify: `CLAUDE.md`（新增 headless 命令与目录说明）
- Test: `js/test/headless/report.test.ts`

**Interfaces:**
- Consumes: Task 7–10 `Detector/finish`；`RunSummary/Issue/RunMeta`；Task 6 `recordFrom`
- Produces: `buildSummary(det, engineInfo): RunSummary`；`buildReportMd(o: { meta, issues, summary, scenario }): string`；`verdictOf(issues): { verdict, counts }`；run.ts 完整闭环（退出码语义）

- [ ] **Step 1: 写失败测试（报告结构 + verdict）**

创建 `js/test/headless/report.test.ts`：

```typescript
// ==================== 测试：报告生成 ====================
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { verdictOf, buildReportMd } from '../../headless/report.js';

describe('verdictOf', () => {
  it('无 issue → PASS；有 high → ISSUES 且计数正确', () => {
    assert.deepStrictEqual(verdictOf([]).verdict, 'PASS');
    const v = verdictOf([
      { category: 'safety', severity: 'high', check: 'collision_true', firstFrame: 1, lastFrame: 2, count: 2, summary: 'x', details: {}, samples: [1] },
      { category: 'perception', severity: 'low', check: 'gps_error', firstFrame: 3, lastFrame: 3, count: 1, summary: 'y', details: {}, samples: [3] },
    ]);
    assert.strictEqual(v.verdict, 'ISSUES');
    assert.deepStrictEqual(v.counts, { high: 1, medium: 0, low: 1 });
  });
});

describe('buildReportMd', () => {
  it('包含判定行、问题列表、指标表与钻取指引', () => {
    const md = buildReportMd({
      meta: { id: 'r1', scenario: 's', map: 'oval', seed: 42, durationS: 60, steps: 3600,
        verdict: 'ISSUES', issueCounts: { high: 1, medium: 0, low: 0 } } as any,
      issues: [{ category: 'safety', severity: 'high', check: 'collision_true',
        firstFrame: 431, lastFrame: 458, count: 28, summary: '撞上障碍', details: {}, samples: [431] }],
      summary: { steps: 3600, simSeconds: 60, speed: { maxMps: 25, avgMps: 20 },
        arrivedAtStep: null, trackingRmseM: 0.2, minObstacleDistM: 1.2,
        stateCounts: { CRUISE: 100 }, emergencyBrakes: 0, boundsResets: 0 } as any,
      scenario: { name: 's', map: 'oval', seed: 42, duration: 60, version: 1,
        obstacles: { mode: 'random', count: 4 } } as any,
    });
    assert.ok(md.includes('ISSUES'));
    assert.ok(md.includes('collision_true'));
    assert.ok(md.includes('431'));
    assert.ok(md.includes('grep'));
  });
});
```

- [ ] **Step 2: 运行确认失败**

Run: `npx tsc && node --test dist/js/test/headless/report.test.js`
Expected: FAIL

- [ ] **Step 3: 实现 report.ts**

```typescript
// ==================== Headless：报告生成 ====================
import { Issue, RunMeta, RunSummary } from '../shared/types.js';
import { Detector } from './detector.js';
import { Scenario } from '../sim/scenario.js';

export function verdictOf(issues: Issue[]): { verdict: 'PASS' | 'ISSUES'; counts: { high: number; medium: number; low: number } } {
  const counts = { high: 0, medium: 0, low: 0 };
  for (const i of issues) counts[i.severity] += 1;
  return { verdict: issues.length > 0 ? 'ISSUES' : 'PASS', counts };
}

/** 从检测器累积量产出聚合指标（速度/RMSE/最小障碍距离/状态计数/事件） */
export function buildSummary(det: Detector, engineInfo: { steps: number; simTime: number }): RunSummary {
  const avg = det.speedN > 0 ? det.speedSum / det.speedN : 0;
  return {
    steps: engineInfo.steps,
    simSeconds: engineInfo.simTime,
    speed: { maxMps: det.speedMax / 4, avgMps: avg / 4 },
    arrivedAtStep: det.arrivedAtStep,
    trackingRmseM: det.trackingErrN > 0 ? Math.sqrt(det.trackingErrSum / det.trackingErrN) / 4 : null,
    minObstacleDistM: Number.isFinite(det.minObstacleDist) ? det.minObstacleDist / 4 : null,
    stateCounts: det.stateCounts,
    emergencyBrakes: det.emergencyBrakeCount,
    boundsResets: det.boundsResetCount,
  };
}

export function buildReportMd(o: { meta: RunMeta; issues: Issue[]; summary: RunSummary; scenario: Scenario }): string {
  const { meta, issues, summary } = o;
  const lines: string[] = [];
  lines.push(`# 运行报告 ${meta.id}`);
  lines.push('');
  lines.push(`**判定: ${meta.verdict}**  |  场景 ${meta.scenario}  |  地图 ${meta.map}  |  种子 ${meta.seed}  |  ` +
    `时长 ${summary.simSeconds.toFixed(1)}s  |  commit ${meta.gitCommit ?? 'n/a'}  |  Node ${meta.nodeVersion}`);
  lines.push('');
  lines.push('## 问题列表');
  if (issues.length === 0) lines.push('（无）');
  for (const i of issues) {
    lines.push(`- **[${i.severity}][${i.category}] ${i.check}** 帧 ${i.firstFrame}–${i.lastFrame}（${i.count} 帧）: ${i.summary}`);
  }
  lines.push('');
  lines.push('## 关键指标');
  lines.push(`| 指标 | 值 |`);
  lines.push(`|---|---|`);
  lines.push(`| 最大/平均速度 | ${summary.speed.maxMps.toFixed(1)} / ${summary.speed.avgMps.toFixed(1)} m/s |`);
  lines.push(`| 到达 step | ${summary.arrivedAtStep ?? '未到达'} |`);
  lines.push(`| 跟踪 RMSE | ${summary.trackingRmseM?.toFixed(3) ?? 'n/a'} m |`);
  lines.push(`| 最近障碍距离 | ${summary.minObstacleDistM?.toFixed(2) ?? 'n/a'} m |`);
  lines.push(`| 状态分布 | ${Object.entries(summary.stateCounts).map(([k, v]) => `${k}:${v}`).join(' ')} |`);
  lines.push('');
  lines.push('## 数据钻取');
  lines.push('```bash');
  lines.push(`grep '"type":"plan"' frames.jsonl | sed -n '<规划帧序号>p'   # 看某帧规划`);
  lines.push(`grep '"type":"event"' frames.jsonl                              # 全部离散事件`);
  lines.push(`sed -n '<step+1>p' frames.jsonl                                  # 按 step 取记录`);
  lines.push('```');
  return lines.join('\n');
}
```

配套修改 `detector.ts`：在 `feed()` 中累积 `speedSum/speedN/speedMax/minObstacleDist/stateCounts/emergencyBrakeCount/boundsResetCount`（step 记录更新速度与最小距离——minDist 需从 perception stats；event 记录更新计数；plan 记录更新 stateCounts）。

- [ ] **Step 4: 集成进 run.ts（替换 TODO 占位）**

在运行循环前创建检测器并接入：

```typescript
  const det = new Detector(scenario.checks as any, { track: engine.track, obstacles: engine.obstacles });
  det.routeActive = built.route !== null;
  det.routeLength = built.route?.length ?? null;
  // 归档订阅循环内追加：det.feed(recordFrom(topic, p))
```

循环结束后替换占位段：

```typescript
  const issues = det.finish();
  const { verdict, counts } = verdictOf(issues);
  const summary = buildSummary(det, { steps: engine.stepCount, simTime: engine.simTime });
  const meta: RunMeta = {
    version: 1, id: args.out ? path.basename(runDir) : id,
    scenario: scenario.name, map: scenario.map, seed: scenario.seed,
    durationS: engine.simTime, steps: engine.stepCount,
    gitCommit: gitCommitOf(), nodeVersion: process.version, createdAt: new Date().toISOString(),
    verdict, issueCounts: counts,
  };
  const reportMd = buildReportMd({ meta, issues, summary, scenario });
  archive.finalize({ issues, summary, reportMd, meta });
  if (!args.out) updateRunsIndex(runsRoot, { id: meta.id, scenario: scenario.name, map: scenario.map,
    seed: scenario.seed, createdAt: meta.createdAt, verdict, issues: counts, durationS: meta.durationS });
  console.log(`运行完成: ${runDir}`);
  console.log(`判定: ${verdict} (high=${counts.high} medium=${counts.medium} low=${counts.low})`);
  process.exitCode = args.failOnIssue && verdict === 'ISSUES' ? 1 : 0;
```

- [ ] **Step 5: 写三个示例场景**

`scenarios/oval-cruise.json`：

```json
{
  "version": 1, "name": "oval-cruise", "map": "oval", "seed": 1, "duration": 90,
  "obstacles": { "mode": "random", "count": 0 },
  "route": { "startS": 0, "goalS": 620 }
}
```

`scenarios/oval-avoid-1.json`：

```json
{
  "version": 1, "name": "oval-avoid-1", "map": "oval", "seed": 42, "duration": 60,
  "obstacles": { "mode": "explicit",
    "list": [ { "s": 120, "l": 2.0, "w": 2.0, "len": 3.0 }, { "s": 135, "l": -2.0, "w": 2.0, "len": 3.0 } ] },
  "route": { "startS": 0, "goalS": 620 }
}
```

`scenarios/hairpin-route.json`：

```json
{
  "version": 1, "name": "hairpin-route", "map": "hairpin", "seed": 7, "duration": 120,
  "obstacles": { "mode": "random", "count": 3 },
  "route": { "startS": 0, "goalS": 540 }
}
```

注意：`count: 0` 需在 parseScenario 校验中允许（0..50）。goalS 值以实际赛道总弧长为参考（实现时先跑一次 `npm run sim` 看 world.json 或日志确认 `< ref.totalS`，必要时调整；buildScenarioWorld 对超长 goalS 的 map 会 clamp——parse 阶段只校验非负，wrap 语义由 sToIndex 的取模保证）。

- [ ] **Step 6: 更新 CLAUDE.md**

在「常用命令」追加：

```bash
npm run sim -- --scenario scenarios/oval-avoid-1.json    # headless 运行+归档+检测
npm run sim -- --map oval --obstacles 4 --seed 42        # 快速随机场景
```

在「目录职责」追加：`js/sim/`（SimEngine 共用核心 + 场景加载）、`js/headless/`（CLI/归档/检测/报告）、`js/replay.ts`、`scenarios/`（场景用例）、`runs/`（运行归档，gitignore）。在「调试与记录」段落补一句 headless 流程（跑 → report.md → frames.jsonl 钻取 → 同种子复跑）。

- [ ] **Step 7: 全量测试 + 冒烟**

Run: `npm test` → Expected: 全部 PASS
Run: `npm run sim -- --scenario scenarios/oval-avoid-1.json`
Expected: 输出判定行；`runs/<id>/report.md` 存在且含「判定」与「数据钻取」；`issues.json` 为合法 JSON
Run: `npm run sim -- --scenario scenarios/oval-avoid-1.json --seed 999`
Expected: seed 覆盖生效（run.json.seed = 999）

- [ ] **Step 8: 提交**

```bash
git add js/headless/report.ts js/headless/run.ts js/headless/detector.ts js/test/headless/report.test.ts scenarios/ CLAUDE.md
git commit -m "接入检测报告闭环（verdict/退出码/report.md）+ 三个示例场景"
```

---

### Task 12: 浏览器回放

**Files:**
- Create: `js/replay.ts`
- Modify: `index.html`（回放栏 DOM + CSS）
- Modify: `js/ui.ts`（回放控件接线）
- Modify: `js/main.ts`（gameLoop 分支：回放模式不步进 engine）

**Interfaces:**
- Consumes: 归档文件（`runs/<id>/{run.json,world.json,issues.json,frames.jsonl}`）；`updateScene/resetSceneCache`（renderer）；`px/rad` 工厂
- Produces: `class ReplayController { constructor(scene, lidarLines); async load(runId): Promise<void>; play/pause/toggle; seekStep(n); setSpeed(x); update(realDt): void; get state(): {active, currentStep, totalSteps, markers: number[]}; }`；`formatRunList(indexJson): {id, label}[]`

- [ ] **Step 1: 实现 replay.ts**

```typescript
// ==================== 浏览器回放控制器 ====================
//
// 读取 runs/<id>/ 归档，以 step（60Hz）为时钟驱动现有渲染层：
// 各记录类型取「t ≤ 当前时刻的最近一条」，构造与实时管线相同的输入，
// 调用 updateScene（复用 overlays/HUD，不写第二套渲染）。
import * as THREE from 'three';
import { updateScene, resetSceneCache } from './env/renderer.js';
import { FrameRecord, RunMeta, WorldFile, Issue } from './shared/types.js';
import { SimEngine } from './sim/engine.js';
import { px, rad } from './shared/dimensions.js';

export interface ReplayState { active: boolean; currentStep: number; totalSteps: number;
  playing: boolean; speed: number; markers: number[]; runId: string | null; }

export class ReplayController {
  readonly state: ReplayState = { active: false, currentStep: 0, totalSteps: 0,
    playing: false, speed: 1, markers: [], runId: null };
  onChange: (() => void) | null = null;

  private _meta: RunMeta | null = null;
  private _world: WorldFile | null = null;
  private _issues: Issue[] = [];
  private _records: FrameRecord[] = [];
  private _stepIndex: Map<number, FrameRecord[]> = new Map();  // step → records
  private _lastRenderedStep = -1;

  constructor(private scene: THREE.Scene, private lidarLines: THREE.Line[]) {}

  static async listRuns(): Promise<{ id: string; label: string }[]> {
    try {
      const res = await fetch('runs/index.json');
      if (!res.ok) return [];
      const idx = await res.json();
      return (idx.runs ?? []).map((r: any) => ({
        id: r.id,
        label: `${r.id} [${r.verdict} h${r.issues?.high ?? 0}/m${r.issues?.medium ?? 0}]`,
      }));
    } catch { return []; }
  }

  async load(runId: string): Promise<void> {
    const base = `runs/${encodeURIComponent(runId)}`;
    this._meta = await (await fetch(`${base}/run.json`)).json();
    this._world = await (await fetch(`${base}/world.json`)).json();
    this._issues = await (await fetch(`${base}/issues.json`)).json();
    const text = await (await fetch(`${base}/frames.jsonl`)).text();
    this._records = text.trim().split('\n').map(l => JSON.parse(l));
    this._stepIndex = new Map();
    let maxStep = 0;
    for (const r of this._records) {
      const arr = this._stepIndex.get(r.step) ?? [];
      arr.push(r); this._stepIndex.set(r.step, arr);
      if (r.step > maxStep) maxStep = r.step;
    }
    this.state.active = true;
    this.state.runId = runId;
    this.state.totalSteps = maxStep;
    this.state.currentStep = 0;
    this.state.markers = this._issues.map(i => i.firstFrame);
    this.state.playing = false;
    resetSceneCache(this.scene);   // 换运行 → 重建静态几何（懒创建）
    this._lastRenderedStep = -1;
    this._render();
    this.onChange?.();
  }

  close(): void { this.state.active = false; this.state.playing = false; this.onChange?.(); }
  play(): void { this.state.playing = true; this.onChange?.(); }
  pause(): void { this.state.playing = false; this.onChange?.(); }
  toggle(): void { this.state.playing ? this.pause() : this.play(); }
  seekStep(n: number): void {
    this.state.currentStep = Math.max(0, Math.min(this.state.totalSteps, Math.round(n)));
    this._render(); this.onChange?.();
  }
  setSpeed(x: number): void { this.state.speed = x; this.onChange?.(); }

  /** 由 main.ts 的 rAF 循环每帧调用 */
  update(realDt: number): void {
    if (!this.state.playing) return;
    this.state.currentStep += realDt * this.state.speed * 60;
    if (this.state.currentStep >= this.state.totalSteps) {
      this.state.currentStep = this.state.totalSteps;
      this.state.playing = false;
    }
    this._render();
    this.onChange?.();
  }

  private _lastOf(type: string, atStep: number): FrameRecord | null {
    let best: FrameRecord | null = null;
    for (let s = atStep; s >= Math.max(0, atStep - 180); s--) {   // 回溯 3s 足够覆盖低频记录
      for (const r of this._stepIndex.get(s) ?? []) {
        if (r.type === type) return r;
      }
    }
    return best;
  }

  private _render(): void {
    if (!this._world || !this._meta) return;
    const step = Math.floor(this.state.currentStep);
    if (step === this._lastRenderedStep) return;
    this._lastRenderedStep = step;

    const world = this._world;
    const toPts = (a: [number, number][]) => a.map(([x, y]) => ({ x: px(x), y: px(y) }));
    const snapshot = {
      car: null as any, obstacles: world.obstacles as any,
      track: { centerPoints: toPts(world.track.centerPoints),
               innerBoundary: toPts(world.track.innerBoundary),
               outerBoundary: toPts(world.track.outerBoundary) },
      timestamp: 0,
    };
    const stepRec: any = this._lastOf('step', step);
    const planRec: any = this._lastOf('plan', step);
    const percRec: any = this._lastOf('perception', step);
    const gpsRec: any = this._lastOf('gps', step);
    if (!stepRec) return;

    snapshot.car = { ...stepRec.car, maxSpeed: 0, acceleration: 0, friction: 0,
      width: 0, length: 0, collisionRadius: 0 };  // 渲染只读 x/y/angle
    const lidarScan = percRec ? {
      angles: percRec.scan.angles.map((a: number) => rad(a)),
      distances: percRec.scan.distances.map((d: number) => px(d)),
      rawDistances: percRec.scan.rawDistances.map((d: number) => px(d)),
      confidences: percRec.scan.confidences, timestamp: 0,
    } : null;
    const trajectory = planRec ? {
      points: planRec.trajectory.points.map((p: any) =>
        ({ t: p.t, x: px(p.x), y: px(p.y), yaw: rad(0), v: px(p.v), kappa: 0 })),
      cost: planRec.trajectory.cost, type: planRec.trajectory.type, timestamp: 0,
    } : null;
    const route = world.route ? {
      waypoints: toPts(world.route.waypoints), waypointIndices: [],
      startIndex: world.route.startIndex, goalIndex: world.route.goalIndex,
      length: px(world.route.length),
    } : null;
    const gps = gpsRec ? { x: px(gpsRec.x), y: px(gpsRec.y), uncertainty: px(5), timestamp: 0 } : null;

    updateScene(this.scene, snapshot as any, lidarScan, planRec?.state ?? 'NONE',
      this.lidarLines, trajectory, route as any, gps as any);
  }
}
```

实现时注意：`updateScene` 内部 `_trackGroup/_obstacleGroup/_carGroup` 是模块级缓存，换运行时 `resetSceneCache` 已处理；`debug` 面板在回放模式不刷新（可接受，v1）。

- [ ] **Step 2: index.html 加回放栏**

在 `#map-select` div 之后追加：

```html
<!-- 回放栏 -->
<div id="replay-bar" style="display:none;position:absolute;bottom:10px;left:50%;transform:translateX(-50%);
  z-index:30;background:rgba(0,0,0,0.85);padding:8px 12px;border-radius:6px;
  display:none;gap:8px;align-items:center;color:#fff;font-family:monospace;font-size:12px;width:900px;">
  <button id="btn-replay-play">⏯</button>
  <input id="replay-timeline" type="range" min="0" max="0" value="0" style="flex:1;">
  <span id="replay-step-label">0 / 0</span>
  <select id="replay-speed">
    <option value="0.5">0.5×</option><option value="1" selected>1×</option>
    <option value="2">2×</option><option value="4">4×</option>
  </select>
  <button id="btn-replay-prev">⟨</button>
  <button id="btn-replay-next">⟩</button>
  <button id="btn-replay-close">✕</button>
</div>
```

注意 `display:none` 与 `display:flex` 的冲突——用 JS 切换 `style.display = 'flex'`；初始 style 里删掉重复的 display（保留一个 none）。`#replay-timeline` 上方叠加问题标记：用 CSS 渐变或简单方案——在时间轴上用 `<datalist>` 不适用；v1 实现：在 JS 里动态生成 marker 小圆点绝对定位在一个 wrapper 内（`position:relative` 的容器包住 range），点击 marker → `seekStep(markerStep)`。

同时 `#map-select` 内加按钮：`<button id="btn-replay-mode" title="回放模式">⏪</button>`。

- [ ] **Step 3: ui.ts 接线**

`setupUI` 内追加：

```typescript
  // ── 回放 ──
  const replayBar = document.getElementById('replay-bar') as HTMLDivElement;
  const timeline = document.getElementById('replay-timeline') as HTMLInputElement;
  const stepLabel = document.getElementById('replay-step-label')!;
  const replay = ctx.replay;

  async function enterReplay(runId: string) {
    debug.paused = true;
    await replay.load(runId);
    replayBar.style.display = 'flex';
    timeline.max = String(replay.state.totalSteps);
  }

  document.getElementById('btn-replay-mode')?.addEventListener('click', async () => {
    const runs = await ReplayController.listRuns();
    if (runs.length === 0) { alert('没有可回放的运行。先执行 npm run sim'); return; }
    const choice = prompt('选择运行（输入编号）:\n' + runs.map((r, i) => `${i}: ${r.label}`).join('\n'), '0');
    if (choice === null) return;
    const idx = Number(choice);
    if (Number.isNaN(idx) || !runs[idx]) return;
    await enterReplay(runs[idx].id);
  });

  document.getElementById('btn-replay-play')?.addEventListener('click', () => replay.toggle());
  document.getElementById('btn-replay-prev')?.addEventListener('click', () => replay.seekStep(replay.state.currentStep - 1));
  document.getElementById('btn-replay-next')?.addEventListener('click', () => replay.seekStep(replay.state.currentStep + 1));
  (document.getElementById('replay-speed') as HTMLSelectElement)?.addEventListener('change', (e: any) => replay.setSpeed(Number(e.target.value)));
  document.getElementById('btn-replay-close')?.addEventListener('click', () => {
    replay.close(); replayBar.style.display = 'none'; debug.paused = false;
  });
  timeline.addEventListener('input', () => replay.seekStep(Number(timeline.value)));
  replay.onChange = () => {
    timeline.value = String(Math.floor(replay.state.currentStep));
    timeline.max = String(replay.state.totalSteps);
    stepLabel.textContent = `${Math.floor(replay.state.currentStep)} / ${replay.state.totalSteps}`;
  };
  window.addEventListener('keydown', (e) => {
    if (!replay.state.active || e.target instanceof HTMLInputElement) return;
    if (e.key === 'ArrowLeft') replay.seekStep(replay.state.currentStep - 1);
    if (e.key === 'ArrowRight') replay.seekStep(replay.state.currentStep + 1);
  });
```

`UIContext` 增加字段 `replay: ReplayController`；`js/ui.ts` 顶部 `import { ReplayController } from './replay.js';`

- [ ] **Step 4: main.ts 接线**

```typescript
import { ReplayController } from './replay.js';
// ...
const replay = new ReplayController(scene, lidarLines);
setupUI({ /* ... 原字段 */, replay });
// gameLoop 内：
  if (replay.state.active) {
    replay.update(realDt);
  } else if (!debug.paused) {
    acc += realDt;
    let n = 0;
    while (acc >= FIXED_DT && n < MAX_CATCHUP) { engine.step(FIXED_DT); acc -= FIXED_DT; n++; }
    if (n === MAX_CATCHUP) acc = 0;
  }
```

- [ ] **Step 5: 全部测试 + 浏览器手工验证**

Run: `npm test` → Expected: 全部 PASS（main.ts 改动不破坏编译）
```bash
npm run sim -- --scenario scenarios/oval-avoid-1.json    # 产生一次真实运行
npm run build && npx serve -l 8080
```
浏览器验证清单：
1. 点地图栏的 ⏪ 按钮 → 弹出运行列表（含刚跑的场景与 verdict）
2. 选择后：场景渲染出来（赛道/障碍物/车在起点），时间轴可拖动，播放/暂停/速度/±1 帧步进正常
3. 问题标记可跳转到故障帧（若该运行有 issue）
4. 关闭回放 → 实时模式恢复（Space 暂停仍可用）

Expected: 全部正常

- [ ] **Step 6: 提交**

```bash
git add js/replay.ts js/ui.ts js/main.ts index.html
git commit -m "新增浏览器回放（时间轴/问题标记/复用现有渲染层）"
```

---

## 计划自审记录（writing-plans 自检）

1. **Spec 覆盖**：Spec §3（engine/固定步长/目录/依赖）→ Task 3；§4（场景/CLI/RNG）→ Task 2/4/6；§5（归档布局/JSONL/体量）→ Task 5/6；§6（12 项检测/issue/report/verdict）→ Task 7–11；§7（回放）→ Task 12；§8（测试）→ 各任务测试步 + Task 6 E2E；§9（顺序/风险）→ 任务编号即顺序，Task 3 含浏览器验证与风险缓解
2. **无占位**：Task 6 的 `TODO(Task 7-11)` 是任务边界的显式衔接（Task 11 Step 4 替换），非计划缺口
3. **类型一致性**：`WorldConfig/RouteState/FrameRecord（含 EventRecord）/DetectorConfig/Issue/RunSummary` 在定义任务与消费者（6/7–11/12）中签名一致；`recordFrom` 输出与 `FrameRecord` 各成员对应；`det.*` 累积字段在 7–10 中逐任务声明且 11 的 `buildSummary` 使用一致
4. **已知取舍**：浏览器维护 `clickStartIdx` 局部状态（ui.ts 协议不变）；回放时 inspect 面板不刷新（v1 可接受，已在 Task 12 注明）
