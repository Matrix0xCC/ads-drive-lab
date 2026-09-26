// ==================== 测试：OSM 地图生成（纯几何，全离线） ====================
// 不依赖网络：全部用合成 fixture（真实链路见 js/mapgen/build-map.ts 的 CLI）。
import { describe, it } from 'node:test';
import assert from 'node:assert';
import {
  parseOverpass, chainWays, wayPoints, selectCarriageway, projectToEnu,
  resampleSmooth, resampleUniform, smoothByArcLength, curvatureStats, pathLengthM,
  speedCapFromCurvature, lateralAccel,
  projectOntoPath, sliceByArcLength,
  type OsmWay, type Vec2M,
} from '../../mapgen/osm.js';

/** 造一条 way：pts = [[lat, lon], ...] */
function mkWay(id: number, pts: [number, number][], tags: Record<string, string> = {}): OsmWay {
  return { id, tags, geometry: pts.map(([lat, lon]) => ({ lat, lon })) };
}

/** 造一条平直的控制点列（沿 x 轴，米），再转成"经纬度"（用 1e-5 度量级即可） */
function straightControl(n: number, segM: number): Vec2M[] {
  return Array.from({ length: n }, (_, i) => ({ x: i * segM, y: 0 }));
}

describe('mapgen/parseOverpass', () => {
  it('缺少 elements → 抛错（错误页/限流响应必须显性失败）', () => {
    assert.throws(() => parseOverpass({}), /elements/);
    assert.throws(() => parseOverpass(null), /elements/);
  });

  it('跳过非 way / 几何点不足；保留合法 way 并按 id 升序', () => {
    const j = { elements: [
      { type: 'node', id: 1, lat: 0, lon: 0 },
      { type: 'way', id: 30, tags: { highway: 'motorway' }, geometry: [{ lat: 0, lon: 0 }, { lat: 0, lon: 0.01 }] },
      { type: 'way', id: 20, geometry: [{ lat: 0, lon: 0 }] },              // 点不足 → 跳过
      { type: 'way', id: 10, geometry: [{ lat: 0, lon: 0 }, { lat: 0, lon: 0.02 }, null] },
    ] };
    const ways = parseOverpass(j);
    assert.deepStrictEqual(ways.map(w => w.id), [10, 30]);
    assert.strictEqual(ways[0].geometry.length, 2, 'null 点被过滤');
  });

  it('oneway=-1 → 几何就地反转（下游只需处理"链方向=通行方向"）', () => {
    const j = { elements: [{ type: 'way', id: 5, tags: { oneway: '-1' },
      geometry: [{ lat: 0, lon: 0 }, { lat: 0, lon: 0.01 }] }] };
    const w = parseOverpass(j)[0];
    assert.ok(Math.abs(w.geometry[0].lon - 0.01) < 1e-12, '首点应变成原来的尾点');
  });
});

describe('mapgen/chainWays', () => {
  // A: (0,0)→(0,0.01)；B: (0,0.01)→(0,0.02)；C: 独立一条
  const A = mkWay(1, [[0, 0], [0, 0.01]]);
  const B = mkWay(2, [[0, 0.01], [0, 0.02]]);
  const C = mkWay(3, [[1, 1], [1, 1.01]]);

  it('共用端点 → 合成一条链，且顺序正确', () => {
    const chains = chainWays([A, B, C]);
    assert.strictEqual(chains.length, 2);
    const main = chains.find(c => c.length === 2)!;
    assert.deepStrictEqual(main.map(w => w.id), [1, 2]);
    // 链的点列应是连续的一串（接缝只保留一个点）
    assert.strictEqual(wayPoints(main).length, 3);
  });

  it('第二条 way 方向相反 → 自动反转后接上', () => {
    const B2 = mkWay(2, [[0, 0.02], [0, 0.01]]);
    const main = chainWays([A, B2]).find(c => c.length === 2)!;
    const pts = wayPoints(main);
    const lonSeq = pts.map(p => +p.lon.toFixed(6));
    assert.deepStrictEqual(lonSeq, [0, 0.01, 0.02], `应被定向为单调前进，实际 ${lonSeq}`);
  });

  it('输入顺序无关（确定性）：打乱输入 → 结果一致', () => {
    const orig = chainWays([A, B, C]).map(c => c.map(w => w.id));
    const shuffled = chainWays([C, B, A]).map(c => c.map(w => w.id));
    assert.deepStrictEqual(shuffled, orig);
  });
});

describe('mapgen/selectCarriageway', () => {
  const seg = (id: number, lat0: number, ref: string, n = 5) => mkWay(id,
    Array.from({ length: n }, (_, i) => [lat0, i * 0.002] as [number, number]),
    { highway: 'motorway', oneway: 'yes', ref, lanes: '3' });

  it('取"单向 + ref 命中"里最长的一条', () => {
    const short = seg(1, 0, 'G4', 3);
    const long = seg(2, 0.01, 'G4', 8);
    const other = seg(3, 0.02, 'G2', 10);
    const picked = selectCarriageway(chainWays([short, long, other]), { ref: 'G4' })!;
    assert.strictEqual(picked[0].id, 2);
  });

  it('ref 按 `;` 拆分精确匹配（"G4;G107" 命中 "G4"）', () => {
    const w = seg(9, 0, 'G4;G107');
    assert.ok(selectCarriageway(chainWays([w]), { ref: 'G4' }));
    assert.strictEqual(selectCarriageway(chainWays([w]), { ref: 'G5' }), null);
  });

  it('双向道路不参与（不是车行道链）', () => {
    const twoWay = mkWay(1, [[0, 0], [0, 0.01]], { highway: 'motorway', lanes: '2' });
    assert.strictEqual(selectCarriageway(chainWays([twoWay]), {}), null);
  });

  it('minLengthM 过滤：太短 → null', () => {
    const w = seg(1, 0, 'G4', 3);
    assert.ok(selectCarriageway(chainWays([w]), { minLengthM: 10 }));
    assert.strictEqual(selectCarriageway(chainWays([w]), { minLengthM: 100000 }), null);
  });
});

describe('mapgen/投影与重采样', () => {
  it('等距圆柱：0.001° 纬度 ≈ 111.19m；原点缺省取首点', () => {
    const pts = projectToEnu([{ lat: 40, lon: 116 }, { lat: 40.001, lon: 116 }]);
    assert.strictEqual(pts[0].x, 0);
    assert.strictEqual(pts[0].y, 0);
    assert.ok(Math.abs(pts[1].y - 111.19) < 0.5, `实际 ${pts[1].y}`);
  });

  it('直线重采样：等距、保端点、曲率≈0', () => {
    const line = resampleSmooth(straightControl(5, 40), 3);
    assert.ok(Math.abs(line[0].x) < 1e-9 && Math.abs(line[0].y) < 1e-9, '首点保留');
    assert.ok(Math.abs(line[line.length - 1].x - 160) < 1e-6, '末点保留');
    assert.ok(Math.abs(pathLengthM(line) - 160) < 0.5, `长度应≈160m，实际 ${pathLengthM(line)}`);
    const k = curvatureStats(line, 5);
    assert.ok(k.p95 < 1e-6, `直线曲率应≈0，实际 p95=${k.p95}`);
    // 相邻间距≈step
    const gaps = line.slice(1).map((p, i) => Math.hypot(p.x - line[i].x, p.y - line[i].y));
    assert.ok(Math.max(...gaps) < 3.2 && Math.min(...gaps) > 2.8, `步长应≈3m，实际 [${Math.min(...gaps)}, ${Math.max(...gaps)}]`);
  });

  it('折角被平滑：同一控制点列，插值后的最大曲率显著小于线性折线', () => {
    // 控制多边形：一条 30° 折角（模拟 OSM 节点稀疏造成的"棱角"）
    const ctrl: Vec2M[] = [{ x: 0, y: 0 }, { x: 60, y: 0 }];
    const ang = 30 * Math.PI / 180;
    ctrl.push({ x: 60 + 60 * Math.cos(ang), y: 60 * Math.sin(ang) });
    ctrl.push({ x: 60 + 120 * Math.cos(ang), y: 120 * Math.sin(ang) });
    const smooth = resampleSmooth(ctrl, 3);
    // 线性"重采样"（对照）：把控制点直接按 3m 连成折线
    const linear: Vec2M[] = [];
    for (let i = 1; i < ctrl.length; i++) {
      const n = Math.ceil(Math.hypot(ctrl[i].x - ctrl[i - 1].x, ctrl[i].y - ctrl[i - 1].y) / 3);
      for (let j = 0; j < n; j++) {
        const t = j / n;
        linear.push({ x: ctrl[i - 1].x + t * (ctrl[i].x - ctrl[i - 1].x), y: ctrl[i - 1].y + t * (ctrl[i].y - ctrl[i - 1].y) });
      }
    }
    linear.push(ctrl[ctrl.length - 1]);
    const kSmooth = curvatureStats(smooth, 5).max;
    const kLinear = curvatureStats(linear, 5).max;
    assert.ok(kSmooth < kLinear * 0.7, `平滑后最大曲率应明显更小：平滑 ${kSmooth.toExponential(2)} vs 线性 ${kLinear.toExponential(2)}`);
  });

  it('曲率口径：R=300m 的圆弧 → κ ≈ 1/300（±5%）', () => {
    const R = 300;
    const arc: Vec2M[] = Array.from({ length: 40 }, (_, i) => {
      const a = (i / 39) * (Math.PI / 3);   // 60°
      return { x: R * Math.sin(a), y: R * (1 - Math.cos(a)) };
    });
    const k = curvatureStats(resampleSmooth(arc, 3), 5);
    assert.ok(Math.abs(k.p50 - 1 / R) < (1 / R) * 0.05, `p50 实际 ${k.p50}，期望 ${1 / R}`);
  });
});

describe('mapgen/曲率与速度', () => {
  it('速度上限 v ≤ √(A_LAT_MAX/κ)，横向加速度 a = v²κ', () => {
    const k = 1 / 300;
    assert.ok(Math.abs(speedCapFromCurvature(k) - Math.sqrt(3.5 * 300)) < 1e-9);
    assert.ok(Math.abs(lateralAccel(k, 30) - 3.0) < 1e-9);
    assert.strictEqual(speedCapFromCurvature(0), Infinity);
  });
});

describe('mapgen/弧长窗口平滑（治 OSM 单节点抖动）', () => {
  it('直线上的单点侧向鼓包 → 幅值被压下去、最大曲率显著下降', () => {
    const line = resampleUniform([{ x: 0, y: 0 }, { x: 300, y: 0 }], 3);
    const spike = line.map(p => ({ ...p }));
    spike[50] = { x: spike[50].x, y: 3 };            // 单节点横向偏 3m → 假棱角
    const before = curvatureStats(spike, 5).max;
    const after = curvatureStats(smoothByArcLength(spike, 30), 5).max;
    assert.ok(after < before / 3, `平滑后最大曲率应降至 1/3 以下：${after.toExponential(2)} vs ${before.toExponential(2)}`);
    assert.ok(Math.max(...smoothByArcLength(spike, 30).map(p => Math.abs(p.y))) < 1, '鼓包幅值应被压到 1m 以内');
  });

  it('真实圆弧（R=300m）不被削平：κ 保持在 ±10%', () => {
    const R = 300;
    const arc: Vec2M[] = Array.from({ length: 80 }, (_, i) => {
      const a = (i / 79) * (Math.PI / 2);
      return { x: R * Math.sin(a), y: R * (1 - Math.cos(a)) };
    });
    const line = resampleUniform(arc, 3);
    const k = curvatureStats(smoothByArcLength(line, 30), 5);
    assert.ok(Math.abs(k.p50 - 1 / R) < (1 / R) * 0.1, `p50 实际 ${k.p50}，期望 ${1 / R}`);
  });

  it('端点固定（地图起终点由收费站桩号决定，不能被平滑挪动）', () => {
    const line = resampleUniform([{ x: 0, y: 0 }, { x: 100, y: 20 }], 3);
    line[10] = { x: line[10].x, y: line[10].y + 5 };
    const sm = smoothByArcLength(line, 30);
    assert.deepStrictEqual(sm[0], line[0]);
    assert.deepStrictEqual(sm[sm.length - 1], line[line.length - 1]);
  });

  it('resampleUniform：等距 + 保端点 + 不改直线线形', () => {
    const line = resampleUniform([{ x: 0, y: 0 }, { x: 50, y: 0 }, { x: 100, y: 0 }], 2.5);
    assert.ok(Math.abs(line[0].x) < 1e-9 && Math.abs(line[line.length - 1].x - 100) < 1e-9);
    const gaps = line.slice(1).map((p, i) => Math.hypot(p.x - line[i].x, p.y - line[i].y));
    assert.ok(Math.max(...gaps) - Math.min(...gaps) < 1e-6, '间距应完全均匀');
  });
});

describe('mapgen/投影到路径 + 按弧长切片（M2 的 GPS 地图匹配原语）', () => {
  const line: Vec2M[] = [{ x: 0, y: 0 }, { x: 100, y: 0 }];

  it('projectOntoPath：s 为插值弧长（不是最近顶点），l 左正右负', () => {
    const a = projectOntoPath(line, { x: 50, y: 3 });
    assert.ok(Math.abs(a.s - 50) < 1e-9, `s 应为 50，实际 ${a.s}`);
    assert.ok(Math.abs(a.l - 3) < 1e-9, `左侧应为 +3，实际 ${a.l}`);
    const b = projectOntoPath(line, { x: 50, y: -3 });
    assert.ok(Math.abs(b.l + 3) < 1e-9, `右侧应为 −3，实际 ${b.l}`);
  });

  it('projectOntoPath：端点之外 → s 被 clamp 到端点', () => {
    const r = projectOntoPath(line, { x: 150, y: 0 });
    assert.ok(Math.abs(r.s - 100) < 1e-9);
    assert.ok(Math.abs(r.l - 50) < 1e-9);
  });

  it('sliceByArcLength：切出 [20,60]（长度≈40，端点精确）', () => {
    const seg = sliceByArcLength(line, 20, 60);
    assert.ok(Math.abs(seg[0].x - 20) < 1e-9, `首点 ${seg[0].x}`);
    assert.ok(Math.abs(seg[seg.length - 1].x - 60) < 1e-9, `末点 ${seg[seg.length - 1].x}`);
    assert.ok(Math.abs(pathLengthM(seg) - 40) < 1e-9);
  });
});
