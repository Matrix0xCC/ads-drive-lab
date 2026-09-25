// ==================== 测试：场景解析与构建 ====================
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { parseScenario, buildScenarioWorld, defaultScenario, ScenarioError } from '../../sim/scenario.js';
import { createRng } from '../../shared/rng.js';
import { cartesianToFrenet, buildFrenetReference } from '../../algo/plan/frenet.js';
import { m, mps } from '../../shared/dimensions.js';

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

describe('defaultScenario（浏览器默认场景）', () => {
  it('产出可通过 parseScenario 校验，随机障碍数量 3-5', () => {
    for (let i = 0; i < 20; i++) {
      const s = parseScenario(defaultScenario('oval'));
      assert.strictEqual(s.map, 'oval');
      assert.ok(Number.isInteger(s.seed) && s.seed >= 0);
      assert.ok(s.obstacles.mode === 'random' &&
        s.obstacles.count >= 3 && s.obstacles.count <= 5,
        `count=${s.obstacles.mode === 'random' ? s.obstacles.count : 'n/a'}`);
    }
  });

  it('注入 rng 时确定（同 rng 同场景）', () => {
    const a = defaultScenario('oval', createRng(5));
    const b = defaultScenario('oval', createRng(5));
    assert.deepStrictEqual(a, b);
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

  it('有路线时随机障碍与起点的沿弧距离 ≥ 80m（出生区净空）', () => {
    // 回归：随机障碍曾落在出生点上（oval 种子 7 重叠 3m：车一出生即接触、
    // 全程 3600 帧碰撞）。带路线时障碍须距起点沿弧 ≥ 80m。
    const arcDist = (track: any, i: number, j: number) => {
      const n = track.centerPoints.length - 1;
      const cum = [0];
      for (let k = 0; k < n; k++) {
        const a = track.centerPoints[k], b = track.centerPoints[(k + 1) % n];
        cum.push(cum[k] + Math.hypot((b.x as number) - (a.x as number), (b.y as number) - (a.y as number)));
      }
      const d = Math.abs(cum[i] - cum[j]);
      return Math.min(d, cum[n] - d);
    };
    const nearestIdx = (track: any, o: any) => {
      let best = 0, bd = Infinity;
      for (let i = 0; i < track.centerPoints.length - 1; i++) {
        const p = track.centerPoints[i];
        const d = Math.hypot((o.x as number) - (p.x as number), (o.y as number) - (p.y as number));
        if (d < bd) { bd = d; best = i; }
      }
      return best;
    };
    for (const [map, seed] of [['oval', 7], ['oval', 6], ['hairpin', 7], ['hairpin', 6]] as const) {
      const s = parseScenario({ ...VALID, map, seed, obstacles: { mode: 'random', count: 5 } });
      const { worldCfg, track, route } = buildScenarioWorld(s, 1800, 800);
      for (const o of worldCfg.obstacles!) {
        // 容差 3m：障碍有 ±3m 横向偏置，最近点在索引空间可能偏移一段
        const d = arcDist(track, nearestIdx(track, o), route!.startIndex);
        assert.ok(d >= (m(80) as number) - (m(3) as number),
          `${map} 种子${seed}: 障碍距起点沿弧 ${(d / 4).toFixed(1)}m 不足 80m`);
      }
    }
  });

  it('默认起步速度 5 m/s（低起步避免出生点紧邻障碍不可避）；car.speed 可覆盖', () => {
    const s = parseScenario(VALID);
    assert.strictEqual(buildScenarioWorld(s, 1800, 800).worldCfg.carStart!.speed, mps(5) as number);
    const overridden = parseScenario({ ...VALID, car: { speed: 20 } });
    assert.strictEqual(buildScenarioWorld(overridden, 1800, 800).worldCfg.carStart!.speed, mps(20) as number);
  });
});
