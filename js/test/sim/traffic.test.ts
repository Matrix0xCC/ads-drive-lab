// ==================== 测试：交通流（方案 A：定速巡航车） ====================
//
// 三条最要紧的性质（本项目的纪律：确定性 + "开/关交通流"必须是干净的正交开关）：
//   ① 同种子两次步进 → 逐位一致（确定性）
//   ② **交通流用独立 RNG 流** → 开关它不改变静态障碍布局（否则 A/B 对照失效）
//   ③ 同车道同速 + 等间距 → 永不追尾（方案 A 的核心取巧，也是它的边界）
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { createTraffic } from '../../env/traffic.js';
import { buildFrenetReference, frenetToCartesian, cartesianToFrenet } from '../../algo/plan/frenet.js';
import { createRng, deriveSeed } from '../../shared/rng.js';
import { px, m, mps, mps2, Px } from '../../shared/dimensions.js';

const L = m(2000);
const ref = buildFrenetReference([{ x: px(0), y: px(0) }, { x: px(0), y: L }], false);

function make(seed: number, density = 8) {
  return createTraffic({
    ref, length: L, closed: false,
    laneCenters: [m(-3.75), px(0), m(3.75)],
    speedLimit: mps(25),
    config: { densityPerKm: density, lanes: 3 },
    rng: createRng(deriveSeed(seed, 'traffic')),
  });
}

describe('交通流：确定性', () => {
  it('同种子两次步进 600 帧 → 车辆状态逐位一致', () => {
    const a = make(7), b = make(7);
    for (let i = 0; i < 600; i++) { a.step(1 / 60); b.step(1 / 60); }
    const va = a.vehicles(), vb = b.vehicles();
    assert.strictEqual(va.length, vb.length);
    for (let i = 0; i < va.length; i++) {
      assert.strictEqual(va[i].s, vb[i].s, `第 ${i} 辆车的 s 不一致`);
      assert.strictEqual(va[i].speed, vb[i].speed);
      assert.strictEqual(va[i].lane, vb[i].lane);
    }
  });
  it('不同种子 → 布局不同（否则种子没起作用）', () => {
    const a = make(7), b = make(8);
    const sa = a.vehicles().map(v => v.s).join(',');
    const sb = b.vehicles().map(v => v.s).join(',');
    assert.notStrictEqual(sa, sb);
  });
});

describe('交通流：物理与车道纪律', () => {
  it('同车道内时刻保持最小间距（定速 + 等间距 → 永不追尾）', () => {
    const t = make(3, 12);
    for (let i = 0; i < 1200; i++) {
      t.step(1 / 60);
      const byLane = new Map<number, number[]>();
      for (const v of t.vehicles()) {
        const arr = byLane.get(v.lane) ?? [];
        arr.push(v.s); byLane.set(v.lane, arr);
      }
      for (const [, ss] of byLane) {
        ss.sort((x, y) => x - y);
        for (let k = 1; k < ss.length; k++) {
          assert.ok(ss[k] - ss[k - 1] > m(4.6), `第 ${i} 帧同车道间距 ${(ss[k] - ss[k - 1]).toFixed(0)}px 小于车长`);
        }
      }
    }
  });
  it('自车出生点前方留空（不许 t=0 就贴着自车）', () => {
    const t = make(5, 20);                       // 高密度也必须有留空
    const near = t.vehicles().filter(v => v.s >= 0 && v.s < m(60));
    assert.strictEqual(near.length, 0, `出生点 60m 内不该有车，实际 ${near.length} 辆`);
  });
  it('开环参考线：越界车辆被回收，起点空了会补车（密度得以维持）', () => {
    const t = make(9, 8);
    const n0 = t.vehicles().length;
    for (let i = 0; i < 60 * 200; i++) t.step(1 / 60);          // 跑 200 秒（足够跑完 2km）
    for (const v of t.vehicles()) {
      assert.ok(v.s >= 0 && v.s <= L + m(30) + 1, `车辆越界：s=${(v.s / 4).toFixed(1)}m`);
    }
    // 20 m/s 的车跑 2km 约 100s → 200s 后应有车辆被回收过；数量仍应维持在同一量级
    assert.ok(t.vehicles().length > n0 * 0.5, `补车失效：${t.vehicles().length} vs 初始 ${n0}`);
  });
  it('闭环参考线：车辆回绕而不丢（径向赛道用）', () => {
    const closedRef = buildFrenetReference([{ x: px(0), y: px(0) }, { x: px(500), y: px(0) }, { x: px(500), y: px(500) }], true);
    const t = createTraffic({
      ref: closedRef, length: m(400), closed: true, laneCenters: [px(0)],
      speedLimit: mps(20), config: { densityPerKm: 10, lanes: 1 }, rng: createRng(1),
    });
    const n0 = t.vehicles().length;
    for (let i = 0; i < 60 * 60; i++) t.step(1 / 60);
    assert.strictEqual(t.vehicles().length, n0, '闭环不应丢车');
    for (const v of t.vehicles()) assert.ok(v.s >= 0 && v.s < m(400), `回绕失效 s=${v.s}`);
  });
  it('障碍矩形带上 isVehicle（感知据此不做 |l| 降置信）', () => {
    const t = make(1, 4);
    const obs = t.obstacles();
    assert.ok(obs.length > 0);
    for (const o of obs) {
      assert.strictEqual(o.isVehicle, true);
      assert.ok(Number.isFinite(o.x as number) && Number.isFinite(o.y as number) && Number.isFinite(o.angle as number));
    }
  });
});

describe('交通流：与静态障碍正交（A/B 对照的前提）', () => {
  it('交通流用 deriveSeed(seed, "traffic")，不占用障碍物的随机流', () => {
    // 同一种子下：两个流必须不同（否则开关车流会连带改变障碍布局）
    const a = createRng(deriveSeed(42, 'obstacles'))();
    const b = createRng(deriveSeed(42, 'traffic'))();
    assert.notStrictEqual(a, b, '两个随机流不应给出同一个数（说明用了不同的 label）');
    const t1 = createRng(deriveSeed(42, 'traffic'))();
    const t2 = createRng(deriveSeed(42, 'traffic'))();
    assert.strictEqual(t1, t2, '同一 label 的流必须可复现');
  });
});

// ─────────────────────────────────────────────────────────────
// 方案 B：IDM 跟驰（每辆车对同车道前车加减速；**自车也算前车**）
// ─────────────────────────────────────────────────────────────
describe('交通流：IDM 跟驰（方案 B）', () => {
  const LANES = [m(-3.75), px(0), m(3.75)];
  // 【参考线必须是密点】`cartesianToFrenet` 是**最近点**搜索（不是最近线段）：拿 2 个点当参考线时，
  // 自车会被投影到端点（实测 s=0、l 完全错）。真实地图/路线的点列本来就是密的，这里照做。
  const dense = buildFrenetReference(
    Array.from({ length: 401 }, (_, i) => ({ x: px(0), y: px(i * 5) })),   // 每 5m 一个点，共 2000m
    false,
  );
  function withCars(cars: { s: number; lane: number; speed: number }[]) {
    return createTraffic({
      ref: dense, length: L, closed: false, laneCenters: LANES, speedLimit: mps(25),
      config: { densityPerKm: 0, lanes: 3 },          // 车道数要和 initialVehicles 里用的下标一致，否则期望速度是 undefined
      rng: createRng(1),
      initialVehicles: cars.map(c => ({
        s: c.s, lane: c.lane, l: LANES[c.lane],
        speed: c.speed, length: m(4.6) as number, width: m(2.0) as number,
      })),
    });
  }
  const lane0 = () => 0;   // 只用一个车道

  it('快车跟慢车：会减速到前车速度，且绝不追尾', () => {
    // 前车 10 m/s 在 300m 处，后车 20 m/s 在 250m 处（净距 50m − 车长）
    const t = withCars([{ s: m(300), lane: lane0(), speed: mps(10) }, { s: m(250), lane: lane0(), speed: mps(20) }]);
    for (let i = 0; i < 60 * 40; i++) {
      t.step(1 / 60);
      const vs = t.vehicles().slice().sort((a, b) => a.s - b.s);
      assert.ok(vs[1].s - vs[0].s > m(4.6) + m(0.5),
        `第 ${i} 帧追尾：净距 ${((vs[1].s - vs[0].s) / 4).toFixed(2)}m`);
    }
    const vs = t.vehicles().slice().sort((a, b) => a.s - b.s);
    assert.ok(Math.abs(vs[1].speed - vs[0].speed) < mps(1.5),
      `后车应跟上前车速度（后 ${(vs[1].speed / 4).toFixed(1)} vs 前 ${(vs[0].speed / 4).toFixed(1)} m/s）`);
    assert.ok(vs[1].speed < mps(20), '后车必须比自由流时慢（说明确实在跟驰）');
  });

  it('**自车当前车**：车流车接近一辆静止的自车时必须刹停，不撞上去（B 的核心行为）', () => {
    const t = withCars([{ s: m(100), lane: lane0(), speed: mps(20) }]);
    const egoS = m(200);
    // 自车必须摆在**同一条车道**：用 `frenetToCartesian(s, 车道中心)` 求笛卡尔位置，而不是把车道
    // 中心的 l 直接当 x ✗（l 的正方向在行进方向**左侧**：靠右行驶时右车道 l 为负、x 为正）。
    // 摆错车道时 NPC 本就不该为它减速——模型的横向判定是对的，是测试摆错了。
    const egoCart = frenetToCartesian(egoS, px(LANES[lane0()] as number), dense);
    for (let i = 0; i < 60 * 30; i++) {
      t.step(1 / 60, { x: px(egoCart.x), y: px(egoCart.y), speed: 0, length: m(4.5) as number });
      const v = t.vehicles()[0];
      assert.ok(v, '车不该被回收（它没越过末端）');
      assert.ok(v.s + m(4.6) / 2 < egoS - m(2.0) / 2, `第 ${i} 帧撞上自车：s=${(v.s / 4).toFixed(1)}m`);
    }
    const v = t.vehicles()[0];
    assert.ok(v.speed < mps(1), `应基本刹停，实际 ${(v.speed / 4).toFixed(2)} m/s`);
    assert.ok(egoS - v.s > m(4.6), '应与自车保持距离');
  });

  it('自由流：单车会加速趋向期望速度，且不显著超过它（IDM 的 (v/v₀)^δ 项）', () => {
    // 放**中间车道**（|l|=0 → speedFactor 取上限 1.0×限速）；外侧车道按设计更慢（0.72×），
    // 这是"慢车靠外"的车道速度规则（单测之前就摆错过车道，这里写清楚）
    const t = withCars([{ s: m(0), lane: 1, speed: 0 }]);
    let maxV = 0;
    for (let i = 0; i < 60 * 60; i++) {
      t.step(1 / 60);
      maxV = Math.max(maxV, t.vehicles()[0]?.speed ?? 0);
    }
    const v0 = mps(25);                                     // 单车道 + speedFactorRange 缺省 → 最内侧 = 1.0×
    assert.ok(maxV > v0 * 0.9, `应当接近期望速度，实际峰值 ${(maxV / 4).toFixed(1)} m/s`);
    assert.ok(maxV <= v0 * 1.05, `不应明显超过期望速度，实际峰值 ${(maxV / 4).toFixed(1)} m/s`);
  });

  it('加速度有界：不出现负速度、也不出现一帧内速度反号的"瞬移"', () => {
    const t = withCars([
      { s: m(100), lane: lane0(), speed: mps(5) },
      { s: m(110), lane: lane0(), speed: mps(25) },          // 故意贴得很近 + 后车更快 → IDM 要给出极大减速
    ]);
    let prev = t.vehicles().map(v => v.speed);
    for (let i = 0; i < 600; i++) {
      t.step(1 / 60);
      const now = t.vehicles().map(v => v.speed);
      for (let k = 0; k < now.length; k++) {
        assert.ok(now[k] >= 0, `出现负速度：${now[k]}`);
        if (k < prev.length) {
          const dv = now[k] - prev[k];
          assert.ok(dv >= mps2(-8) / 60 - 1e-6, `一帧减速超过物理上限：${(dv * 4).toFixed(2)} m/s`);
        }
      }
      prev = now;
    }
  });
});

// ─────────────────────────────────────────────────────────────
// 「自车 vs 车流车」的横向判定依赖一条**隐含约定**：车流的 l 来自 laneCentersOf（地图侧），
// 自车的 l 来自 cartesianToFrenet（算法侧）——两者必须自洽，否则"同车道"永远匹配不上。
// 实测踩过：l 的正方向在行进方向**左侧**（靠右行驶时右车道 l 为负、笛卡尔 x 为正）。
// ─────────────────────────────────────────────────────────────
describe('交通流：车道横向口径自洽（与感知/规划同一套 Frenet）', () => {
  it('laneCentersOf 给的车道中心，经 frenetToCartesian → cartesianToFrenet 往返后 l 不变', () => {
    const ref3 = buildFrenetReference(Array.from({ length: 401 }, (_, i) => ({ x: px(0), y: px(i * 5) })), false);
    const laneCenters = [m(-5.63), m(-1.88), m(1.88), m(5.63)];
    for (const lc of laneCenters) {
      const c = frenetToCartesian(px(800), px(lc), ref3);
      const back = cartesianToFrenet(c.x, c.y, ref3);
      assert.ok(Math.abs((back.l as number) - lc) < 1,
        `车道 l=${(lc / 4).toFixed(2)}m 往返后变成 ${((back.l as number) / 4).toFixed(2)}m（口径不一致）`);
    }
  });
});
