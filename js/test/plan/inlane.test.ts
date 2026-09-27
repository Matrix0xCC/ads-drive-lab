// ==================== 测试：本车道前方间隙（S1①） ====================
//
// 这条口径直接决定"旁边超车时自车会不会无故减速"（用户实测截图）：
// 旧口径 `min(全部射线)/2` 会把相邻车道的车算进来 → 巡航目标被钉到 17.5m/s。
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { inLaneGapAhead, LANE_BAND_MARGIN } from '../../algo/plan/inlane.js';
import { m, px } from '../../shared/dimensions.js';

const LANES = [m(-3.75), px(0), m(3.75)];      // 三车道：−3.75 / 0 / +3.75
const LW = m(3.75) as number;
const NAN = NaN;

/** 造一条 scan：只有给定的 (s,l) 是真障碍命中 */
function scanOf(hits: [number | null, number | null][]) {
  return {
    hitS: hits.map(h => (h[0] === null ? NAN : (h[0] as number))),
    hitL: hits.map(h => (h[1] === null ? NAN : (h[1] as number))),
  };
}

describe('本车道前方间隙', () => {
  it('**相邻车道**的车不算前方障碍（超车场景：这就是"无故减速"的根因）', () => {
    // 自车在中间车道，右侧 3.75m 处有一辆车、纵向就在旁边
    const scan = scanOf([[m(1), m(3.75)]]);
    assert.strictEqual(inLaneGapAhead(scan, px(0), px(0), LANES, LW), null,
      '相邻车道的车不应产生"前方障碍距离"');
  });

  it('本车道前方的车才算，且取最近的那个（按纵向间隙）', () => {
    const scan = scanOf([[m(60), px(0)], [m(20), px(0)], [m(5), m(3.75)]]);
    const gap = inLaneGapAhead(scan, px(0), px(0), LANES, LW);
    assert.ok(gap !== null && Math.abs(gap - (m(20) as number)) < 1,
      `应取本车道最近的 20m（实际 ${gap === null ? 'null' : (gap / 4).toFixed(1) + 'm'}）`);
  });

  it('**后方**的物体不算（后向传感器接入的前提）', () => {
    const scan = scanOf([[m(-10), px(0)]]);
    assert.strictEqual(inLaneGapAhead(scan, px(0), px(0), LANES, LW), null);
  });

  it('骑线的物体算进来（保守：宁可把贴近车道线的算上）', () => {
    const straddle = (LW as number) / 2 + LANE_BAND_MARGIN - 1;   // 刚好在带内
    const scan = scanOf([[m(30), px(straddle)]]);
    assert.ok(inLaneGapAhead(scan, px(0), px(0), LANES, LW) !== null, '带内应算');
    const outside = scanOf([[m(30), px((LW as number) / 2 + LANE_BAND_MARGIN + 4)]]);
    assert.strictEqual(inLaneGapAhead(outside, px(0), px(0), LANES, LW), null, '带外不算');
  });

  it('没有位置信息（老归档/合成扫描）→ 返回 null，调用方退回旧口径', () => {
    assert.strictEqual(inLaneGapAhead({}, px(0), px(0), LANES, LW), null);
  });

  it('车道信息缺失时退化成一个带（仍只认前方 + 带内）', () => {
    const scan = scanOf([[m(25), m(2)], [m(25), m(50)]]);
    const gap = inLaneGapAhead(scan, px(0), px(0), undefined, LW);
    assert.ok(gap !== null, '无车道中心时按 l=0 的带判断');
  });
});
