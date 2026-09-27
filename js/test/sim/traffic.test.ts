// ==================== 测试：交通流（方案 A：定速巡航车） ====================
//
// 三条最要紧的性质（本项目的纪律：确定性 + "开/关交通流"必须是干净的正交开关）：
//   ① 同种子两次步进 → 逐位一致（确定性）
//   ② **交通流用独立 RNG 流** → 开关它不改变静态障碍布局（否则 A/B 对照失效）
//   ③ 同车道同速 + 等间距 → 永不追尾（方案 A 的核心取巧，也是它的边界）
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { createTraffic } from '../../env/traffic.js';
import { buildFrenetReference } from '../../algo/plan/frenet.js';
import { createRng, deriveSeed } from '../../shared/rng.js';
import { px, m, mps, Px } from '../../shared/dimensions.js';

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
