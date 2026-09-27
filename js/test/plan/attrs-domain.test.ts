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
import { buildTrack, ribbonPolygon, ribbonPolygonVar, taperedHalfWidthAt, halfWidthsAlongPoints } from '../../env/track.js';
import { MAP_PRESETS } from '../../env/mapDefinitions.js';
import { buildGraphFromEdges, buildNetworkRoute } from '../../algo/routePlanner.js';
import { attrsAtS } from '../../shared/types.js';
import { m, px } from '../../shared/dimensions.js';

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

describe('变宽路面与过渡段', () => {
  it('定宽版 == 变宽版（同宽数组）→ 定宽地图逐位不变', () => {
    const pts: any[] = [];
    for (let i = 0; i <= 40; i++) pts.push({ x: px(M(10) + i * M(3)), y: px(M(100) + (i % 2) * M(2)) });
    const w = M(5.625);
    const a = ribbonPolygon(pts, w);
    const b = ribbonPolygonVar(pts, pts.map(() => w)).poly;
    assert.deepStrictEqual(b, a, '变宽版在同宽时必须与定宽版逐位相同');
  });

  it('变宽版真的把窄段画窄了（G4：3 车道 5.625m / 2 车道 3.75m）', () => {
    const t = buildTrack(MAP_PRESETS['bj-g4-dujiakan'], 1800, 800);
    const hw = t.outerBoundaryHalfWidth!;
    assert.ok(hw.length === t.outerBoundary.length, '逐点半宽必须与边界点一一对应');
    // 含过渡段 → 中间值（3.75 ~ 5.625 之间的渐变），但两端值都必须出现
    const vals = [...new Set(hw.map(v => +(v / 4).toFixed(2)))];
    assert.ok(vals.every(v => v >= 3.74 && v <= 5.63), `半宽应落在 [3.75, 5.625]：${vals}`);
    assert.ok(vals.some(v => Math.abs(v - 3.75) < 0.01), '应含 2 车道段 3.75m');
    assert.ok(vals.some(v => Math.abs(v - 5.625) < 0.01), '应含 3 车道段 5.625m');
    // 取中心线末端（2 车道段）验证：到边界的横向距离 ≈ 3.75m（含端帽外延，故只验证量级）
    const last = t.centerPoints[t.centerPoints.length - 1];
    let minD = Infinity;
    for (const p of t.outerBoundary) {
      minD = Math.min(minD, Math.hypot(p.x - last.x, p.y - last.y));
    }
    assert.ok(minD / 4 > 3 && minD / 4 < 9, `末端到边界距离应≈3.75m（含端帽），实际 ${(minD / 4).toFixed(2)}m`);
  });

  it('过渡段：远离台阶时等于台阶值，进入 taper 后线性逼近目标值', () => {
    // [0,1000px] 半宽 22.5px → [1000,2000px] 半宽 7.5px
    const spans = [
      { highway: 'trunk', laneCount: 3, laneWidthM: 3.75, halfWidthM: 5.625, maxspeedKph: 100, s0: px(0), s1: px(1000) },
      { highway: 'trunk_link', laneCount: 1, laneWidthM: 3.75, halfWidthM: 1.875, maxspeedKph: 50, s0: px(1000), s1: px(2000) },
    ] as any;
    const taper = px(M(30)) as number;
    const wAt = taperedHalfWidthAt(spans, px(M(5.625)) as number, taper);
    assert.strictEqual(wAt(0), 22.5, '远离台阶 → 宽段值');
    assert.strictEqual(wAt(1000 - taper - 1), 22.5, '过渡段之前 → 宽段值');
    const mid = wAt(1000 - taper / 2);          // 过渡中点 → 约在两者之间
    assert.ok(mid > 7.5 && mid < 22.5, `过渡中点应在两值之间，实际 ${mid}`);
    assert.ok(wAt(1000 + 1) <= 7.5 + 1e-6, '台阶之后 → 窄段值');
    // 单调不增（越接近窄段越窄）
    let prev = Infinity;
    for (let s = 1000 - taper - 5; s <= 1000 + 5; s += 5) {
      const w = wAt(s);
      assert.ok(w <= prev + 1e-9, `过渡应单调不增：s=${s} w=${w} prev=${prev}`);
      prev = w;
    }
  });

  it('无属性时 halfWidthsAlongPoints 全为 fallback（零回归）', () => {
    const pts = [{ x: px(0), y: px(0) }, { x: px(100), y: px(0) }];
    assert.deepStrictEqual(halfWidthsAlongPoints(pts, undefined, 30), [30, 30]);
  });
});
