// ==================== 测试：逐段属性 → 可行驶域与限速 ====================
//
// 背景（都是实测踩出来的）：
//   ① 单车道匝道半宽 1.875m，若复位缓冲固定 0.5m，车中心只剩 ±0.375m —— 而最近邻投影误差
//      就有几厘米 → "规划域比误差还窄" → 候选路径起点就在走廊外 → 全判碰撞 → 永久停车。
//      修法：缓冲按路段宽度缩放（boundsBufferFor）。
//   ② 规划域**永远不能比复位崖更宽**（否则规划器把车贴到崖外 → 复位传送）。
//   ③ 逐段限速是 km/h（人读）必须转 px/s（规划器量纲）——写成 m/s 会让巡航目标被砍到 1/4。
import { describe, it } from 'node:test';
import assert from 'node:assert';
import * as fs from 'node:fs';
import * as path from 'node:path';
import { boundsBufferFor, BOUNDS_BUFFER } from '../../env/car.js';
import { buildTrack } from '../../env/track.js';
import { MAP_PRESETS } from '../../env/mapDefinitions.js';
import { buildGraphFromEdges, buildNetworkRoute } from '../../algo/routePlanner.js';
import { attrsAtS } from '../../shared/types.js';
import { m } from '../../shared/dimensions.js';

const M = (v: number) => m(v) as number;

describe('env/car：复位缓冲按路段宽度缩放', () => {
  it('宽路保持 0.5m（径向图零回归），窄路按比例缩小', () => {
    const carHalf = M(1.0);
    // 径向：半宽 7.5m → 车中心余量 6.5m → 仍是 0.5m
    assert.strictEqual(boundsBufferFor(M(7.5), carHalf), BOUNDS_BUFFER as number);
    // 3 车道高速：半宽 5.625m → 余量 4.625m → 仍是 0.5m
    assert.strictEqual(boundsBufferFor(M(5.625), carHalf), BOUNDS_BUFFER as number);
    // 单车道匝道：半宽 1.875m → 余量 0.875m → 缩到 0.35m
    const ramp = boundsBufferFor(M(1.875), carHalf);
    assert.ok(ramp < (BOUNDS_BUFFER as number), `窄路缓冲应缩小，实际 ${ramp / 4}m`);
    assert.ok(Math.abs(ramp / 4 - 0.35) < 0.01, `匝道缓冲应≈0.35m，实际 ${ramp / 4}m`);
    // 缩放后车中心可用行程（= 余量 − 缓冲）必须大于投影误差量级（几厘米）
    const room = (M(1.875) - carHalf - ramp) / 4;
    assert.ok(room > 0.5, `匝道上车中心应有 >0.5m 的行程，实际 ${room}m`);
  });
});

describe('地图产物：逐段属性进入运行时', () => {
  it('路网地图：track.chainAttrs 与 chains 等长，半宽是米', () => {
    for (const name of ['bj-g6-junction', 'bj-3rd-ring']) {
      const t = buildTrack(MAP_PRESETS[name], 1800, 800);
      assert.strictEqual(t.chainAttrs!.length, t.chains!.length);
      for (const a of t.chainAttrs!) {
        assert.ok(a.halfWidthM > 1 && a.halfWidthM < 12, `${name} 半宽应是米：${a.halfWidthM}`);
      }
      // 三环真有 1 车道匝道（半宽 1.875m）与 4 车道主路（7.5m）——地图语义真的随路段变
      if (name === 'bj-3rd-ring') {
        const hw = [...new Set(t.chainAttrs!.map(a => a.halfWidthM))].sort((x, y) => x - y);
        assert.ok(hw.length >= 3, `应有多种路宽，实际 ${JSON.stringify(hw)}`);
        assert.ok(hw[0] < 2.5 && hw[hw.length - 1] > 6, `应含窄匝道与宽主路：${JSON.stringify(hw)}`);
      }
    }
  });

  it('链式地图：track.attrSpans 覆盖整条中心线（G4 → 3 车道与 2 车道两段）', () => {
    const t = buildTrack(MAP_PRESETS['bj-g4-dujiakan'], 1800, 800);
    const spans = t.attrSpans!;
    assert.ok(spans.length >= 2, `应分两段，实际 ${spans.length}`);
    assert.strictEqual(spans[0].s0 as number, 0);
    assert.deepStrictEqual([...new Set(spans.map(s => s.laneCount))].sort(), [2, 3]);
    assert.strictEqual(attrsAtS(spans, 0)!.laneCount, 3);
    assert.strictEqual(attrsAtS(spans, 1900 * 4)!.laneCount, 2);
  });

  it('径向地图没有任何逐段属性（走地图级标量 → 零回归路径）', () => {
    const t = buildTrack(MAP_PRESETS['oval'], 1800, 800);
    assert.strictEqual(t.chainAttrs, undefined);
    assert.strictEqual(t.attrSpans, undefined);
  });
});

describe('路网路线：逐段属性沿路线装配（弧长 px，可查表）', () => {
  it('三环路线含 1 车道/50km/h 的匝道段与 3 车道/100km/h 的主路段', () => {
    const t = buildTrack(MAP_PRESETS['bj-3rd-ring'], 1800, 800);
    const graph = buildGraphFromEdges(t.netNodes!.length, t.netEdges!, t.chains!);
    const r = buildNetworkRoute(graph, t.netEdges!, t.chains!, 2, 93, t.chainAttrs)!;
    assert.ok(r, '路线应可达');
    const spans = r.attrSpans;
    assert.ok(spans.length >= 3, `逐段属性应随链切换，实际 ${spans.length} 段`);
    assert.strictEqual(spans[0].s0 as number, 0);
    // 区间连续、覆盖到路线末端
    for (let i = 1; i < spans.length; i++) {
      assert.ok(Math.abs((spans[i].s0 as number) - (spans[i - 1].s1 as number)) < 1e-6, '区间应首尾相接');
    }
    const routeLen = r.waypoints.reduce((acc, p, i) => i === 0 ? 0
      : acc + Math.hypot(p.x - r.waypoints[i - 1].x, p.y - r.waypoints[i - 1].y), 0);
    assert.ok(Math.abs((spans[spans.length - 1].s1 as number) - routeLen) < 1, '应覆盖到路线末端');
    assert.deepStrictEqual([...new Set(spans.map(s => s.laneCount))].sort(), [1, 3, 4]);
    assert.ok(spans.some(s => s.maxspeedKph === 50), '应含匝道限速段');
    assert.ok(spans.some(s => s.maxspeedKph === 100), '应含主路限速段');
    // 查表：匝道段查到 1 车道
    const ramp = spans.find(s => s.laneCount === 1)!;
    assert.strictEqual(attrsAtS(spans, ramp.s0 as number)!.laneCount, 1);
  });

  it('无逐段属性时（径向）不产生区间', () => {
    const t = buildTrack(MAP_PRESETS['oval'], 1800, 800);
    assert.strictEqual(t.attrSpans, undefined);
    assert.strictEqual(attrsAtS(undefined, 0), null);
  });
});
