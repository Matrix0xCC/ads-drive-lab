// ==================== 测试：逐段道路属性（纯函数 + 回退规则） ====================
//
// 这一层的价值全在"**哪些属性是真实 OSM 数据、哪些是按等级派生的**"以及量纲正确。
// 北京三份样本的实测覆盖率（2026-09-27）：lanes —— motorway/G4 100%、trunk 34%、匝道 ≈0%；
// maxspeed —— 几乎为 0（三份共 941 条 way 里 1 条）。所以回退规则和来源标注必须被钉住。
import { describe, it } from 'node:test';
import assert from 'node:assert';
import {
  chainAttrs, derivedLaneCount, derivedMaxspeedKph, spansFromPointAttrs, attrsAt, attrCoverage,
  DEFAULT_LANE_WIDTH_M, type ChainAttrs,
} from '../../mapgen/attrs.js';

describe('mapgen/attrs：OSM tags → 逐段属性', () => {
  it('lanes 有 tag → 用它并按车道数算半宽（米，不是 px）', () => {
    const a = chainAttrs({ highway: 'motorway', lanes: '3' });
    assert.strictEqual(a.laneCount, 3);
    assert.strictEqual(a.lanesFrom, 'osm');
    assert.strictEqual(a.halfWidthM, 3 * DEFAULT_LANE_WIDTH_M / 2);   // 5.625m
    assert.ok(a.halfWidthM < 12, `halfWidthM 必须是米（px 会大 4 倍）：${a.halfWidthM}`);
  });

  it('maxspeed 有 tag → 用它（含 "120 km/h" 这类写法）', () => {
    assert.strictEqual(chainAttrs({ highway: 'trunk', maxspeed: '100' }).maxspeedKph, 100);
    assert.strictEqual(chainAttrs({ highway: 'trunk', maxspeed: '80 km/h' }).maxspeedKph, 80);
    assert.strictEqual(chainAttrs({ highway: 'trunk', maxspeed: '100' }).maxspeedFrom, 'osm');
  });

  it('lanes 缺失 → 按等级回退：匝道 1 车道、motorway/trunk 3、其余 2', () => {
    assert.strictEqual(chainAttrs({ highway: 'motorway_link' }).laneCount, 1);
    assert.strictEqual(chainAttrs({ highway: 'trunk_link' }).laneCount, 1);
    assert.strictEqual(chainAttrs({ highway: 'trunk' }).laneCount, 3);
    assert.strictEqual(chainAttrs({ highway: 'primary' }).laneCount, 2);
    assert.strictEqual(chainAttrs({ highway: 'motorway_link' }).lanesFrom, 'derived:class');
  });

  it('maxspeed 缺失/无效（none、signals、国别标记）→ 按等级派生，并标注来源', () => {
    assert.strictEqual(derivedMaxspeedKph('motorway'), 120);
    assert.strictEqual(derivedMaxspeedKph('trunk'), 100);
    // 匝道 = 主线 × 0.5，夹到 [40, 60]
    assert.strictEqual(derivedMaxspeedKph('motorway_link'), 60);
    assert.strictEqual(derivedMaxspeedKph('trunk_link'), 50);
    for (const v of ['none', 'signals', 'CN:urban', 'walk']) {
      const a = chainAttrs({ highway: 'trunk', maxspeed: v });
      assert.strictEqual(a.maxspeedFrom, 'derived:class', `${v} 应回退`);
      assert.strictEqual(a.maxspeedKph, 100);
    }
  });

  it('解析容错：lanes "2;3" 取第一个；宽度 tag 合法时才用', () => {
    assert.strictEqual(chainAttrs({ highway: 'trunk', lanes: '2;3' }).laneCount, 2);
    assert.strictEqual(chainAttrs({ highway: 'trunk', lanes: 'x' }).laneCount, derivedLaneCount('trunk'));
    assert.strictEqual(chainAttrs({ highway: 'trunk', width: '10' }).laneWidthM, 10);
    assert.strictEqual(chainAttrs({ highway: 'trunk', width: '999' }).laneWidthM, DEFAULT_LANE_WIDTH_M);
  });
});

describe('mapgen/attrs：区间压缩与查表', () => {
  const A = (laneCount: number, maxspeedKph: number): ChainAttrs => ({
    highway: 'trunk', laneCount, laneWidthM: 3.75, halfWidthM: laneCount * 3.75 / 2,
    maxspeedKph, lanesFrom: 'osm', maxspeedFrom: 'osm',
  });

  it('相邻同属性合并成一段（100 个点 → 2 段）', () => {
    const sM = Array.from({ length: 100 }, (_, i) => i * 10);          // 10m 步长
    const attrs = Array.from({ length: 100 }, (_, i) => i < 50 ? A(3, 100) : A(1, 50));
    const spans = spansFromPointAttrs(sM, attrs);
    assert.strictEqual(spans.length, 2);
    assert.strictEqual(spans[0].s0M, 0);
    assert.strictEqual(spans[0].s1M, 490);                             // 到第 49 点
    assert.strictEqual(spans[1].laneCount, 1);
    assert.strictEqual(spans[1].s1M, 990);
  });

  it('attrsAtS：查表命中正确段，超界取首/末段', () => {
    const spans = spansFromPointAttrs([0, 100, 200, 300], [A(3, 100), A(3, 100), A(1, 50), A(1, 50)]);
    assert.strictEqual(attrsAt(spans, 50)!.laneCount, 3);
    assert.strictEqual(attrsAt(spans, 150)!.laneCount, 1);
    assert.strictEqual(attrsAt(spans, -10)!.laneCount, 3);
    assert.strictEqual(attrsAt(spans, 9999)!.maxspeedKph, 50);
    assert.strictEqual(attrsAt([], 0), null);
  });

  it('attrCoverage 统计真实数据占比', () => {
    const cov = attrCoverage([A(3, 100), { ...A(1, 50), lanesFrom: 'derived:class', maxspeedFrom: 'derived:class' }]);
    assert.deepStrictEqual(
      { total: cov.total, lanesFromOsm: cov.lanesFromOsm, maxspeedFromOsm: cov.maxspeedFromOsm },
      { total: 2, lanesFromOsm: 1, maxspeedFromOsm: 1 },
    );
  });
});
