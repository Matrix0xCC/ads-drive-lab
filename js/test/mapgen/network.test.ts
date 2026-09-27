// ==================== 测试：OSM 路网拓扑（全离线） ====================
// 合成 fixture 覆盖拓扑规则；产物契约覆盖"入库的立交地图真的能路由、且路由真的经过匝道"。
import { describe, it } from 'node:test';
import assert from 'node:assert';
import * as fs from 'node:fs';
import * as path from 'node:path';
import { createHash } from 'node:crypto';
import { buildTopology, largestComponent, topologyStats, nodeDegrees } from '../../mapgen/network.js';
import { buildGraphFromEdges, buildNetworkRoute } from '../../algo/routePlanner.js';
import { type OsmWay } from '../../mapgen/osm.js';
import { px } from '../../shared/dimensions.js';

/** 造 way：pts = [[lat, lon], ...] */
function mkWay(id: number, pts: [number, number][], tags: Record<string, string> = {}): OsmWay {
  return { id, tags, geometry: pts.map(([lat, lon]) => ({ lat, lon })) };
}

describe('mapgen/buildTopology（路网拓扑）', () => {
  it('匝道接在主线**中间顶点**上 → 主线被切断，形成度 3 的路口', () => {
    // 主线：3 个顶点的直线；匝道：从主线的中间顶点分出去
    const main = mkWay(1, [[0, 0], [0, 0.01], [0, 0.02]], { highway: 'motorway', oneway: 'yes' });
    const ramp = mkWay(2, [[0, 0.01], [0.005, 0.012]], { highway: 'motorway_link', oneway: 'yes' });
    const topo = buildTopology([main, ramp]);
    const st = topologyStats(topo);
    assert.strictEqual(topo.edges.length, 3, '主线被切成 2 段 + 匝道 1 段');
    assert.strictEqual(st.junctions, 1, '中间顶点应是度 3 的路口');
    assert.strictEqual(st.deadEnds, 3, '两个主线端 + 匝道端');
    const deg = nodeDegrees(topo);
    assert.strictEqual(Math.max(...deg), 3);
  });

  it('所有 way 共用同一个投影原点（否则首顶点会全部塌成同一个节点）', () => {
    // 两条相距约 1.1km 的独立 way：若各自用自己的首点为原点，会得到同一个 (0,0) 节点
    const a = mkWay(1, [[0, 0], [0, 0.01]], { highway: 'motorway', oneway: 'yes' });
    const b = mkWay(2, [[0.01, 0.01], [0.01, 0.02]], { highway: 'motorway', oneway: 'yes' });
    const topo = buildTopology([a, b]);
    assert.strictEqual(topo.nodes.length, 4, `两条不相连的 way 应有 4 个节点，实际 ${topo.nodes.length}`);
    assert.strictEqual(topologyStats(topo).junctions, 0);
    // 节点间距离应≈1.1km（说明用的是同一坐标系）
    const d = Math.hypot(topo.nodes[0].x - topo.nodes[2].x, topo.nodes[0].y - topo.nodes[2].y);
    assert.ok(d > 1000 && d < 2000, `节点间距应是"跨两条 way"的量级（≈1.1~1.6km），实际 ${d.toFixed(0)}m`);
  });

  it('largestComponent：丢掉互不相连的孤岛（节点重映射为紧凑序号）', () => {
    const main = mkWay(1, [[0, 0], [0, 0.01], [0, 0.02]], { highway: 'motorway', oneway: 'yes' });
    const ramp = mkWay(2, [[0, 0.01], [0.005, 0.012]], { highway: 'motorway_link', oneway: 'yes' });
    const island = mkWay(3, [[1, 1], [1, 1.01]], { highway: 'motorway', oneway: 'yes' });
    const comp = largestComponent(buildTopology([main, ramp, island]));
    assert.strictEqual(comp.edges.length, 3, '孤岛应被丢掉');
    assert.ok(comp.edges.every(e => e.from < comp.nodes.length && e.to < comp.nodes.length));
  });

  it('确定性：way/顶点顺序打乱 → 拓扑统计一致', () => {
    const a = mkWay(1, [[0, 0], [0, 0.01]], { highway: 'motorway', oneway: 'yes' });
    const b = mkWay(2, [[0, 0.01], [0, 0.02]], { highway: 'motorway', oneway: 'yes' });
    const s1 = topologyStats(buildTopology([a, b]));
    const s2 = topologyStats(buildTopology([b, a]));
    assert.deepStrictEqual(s2, s1);
  });
});

describe('mapgen/入库路网地图契约', () => {
  /** 载入入库路网地图（离线；与 maps/<name>.json 严格对应） */
  function loadNet(name: string) {
    const map = JSON.parse(fs.readFileSync(path.resolve('maps', `${name}.json`), 'utf8'));
    const nodes = map.network.nodes as [number, number][];
    const edges = map.network.edges as { from: number; to: number; chain: number; highway: string; lengthM: number }[];
    const chains = (map.network.chains as [number, number][][])
      .map(ch => ch.map(([x, y]) => ({ x: px(x), y: px(y) })));
    return { map, nodes, edges, chains };
  }

  for (const name of ['bj-g6-junction', 'bj-3rd-ring']) {
    describe(name, () => {
      const { map, nodes, edges, chains } = loadNet(name);

      it('与入库原始数据一致，且索引自洽（链首尾 = 节点坐标）', () => {
        const sha = createHash('sha256')
          .update(fs.readFileSync(path.resolve('maps/raw', `${name}.osm.json`), 'utf8')).digest('hex');
        assert.strictEqual(map.source.rawSha256, sha, '地图与原始 OSM 响应不一致');
        assert.strictEqual(map.kind, 'network');
        // 容差 2mm：拓扑节点键按 1mm 量化（两个相距 <1mm 的顶点会合并成一个节点，
        // 而链端点保留各自的原始坐标）——毫米级差异是设计内的，不是错位。
        const TOL = 2e-3;
        for (const e of edges) {
          const ch = chains[e.chain];
          assert.ok(ch && ch.length >= 2, `边 ${e.chain} 的链缺失`);
          const a = nodes[e.from], b = nodes[e.to];
          assert.ok(Math.hypot(ch[0].x - a[0], ch[0].y - a[1]) < TOL, `链 ${e.chain} 首端与节点不吻合`);
          assert.ok(Math.hypot(ch[ch.length - 1].x - b[0], ch[ch.length - 1].y - b[1]) < TOL, `链 ${e.chain} 末端与节点不吻合`);
        }
      });

      it('是真路网：有路口（度≥3），且 hasCycle 与"边数 ≥ 节点数"一致', () => {
        const deg = new Array(nodes.length).fill(0);
        for (const e of edges) { deg[e.from]++; deg[e.to]++; }
        assert.ok(deg.filter(d => d >= 3).length >= 1, '至少一个路口');
        assert.strictEqual(map.validation.hasCycle, edges.length >= nodes.length);
        assert.ok(map.validation.lengthM > 1000);
      });

      it('抽样路线可复算，且"替代走法"指标与归档一致', () => {
        const graph = buildGraphFromEdges(nodes.length, edges);
        const sample = map.validation.routeSample as {
          fromNode: number; toNode: number; linksUsed: number;
          alternativeDetourM: number | null;
        };
        const r = buildNetworkRoute(graph, edges, chains, sample.fromNode, sample.toNode);
        assert.ok(r, '抽样起终点应可达');
        assert.strictEqual(r!.chainSeq.filter(ci => edges[ci].highway.endsWith('_link')).length, sample.linksUsed);
        // 逐段掐掉重路由 → 最小绕行代价（复算必须等于归档值；树状路网恒为 null）
        const base = r!.waypoints.reduce((acc, p, i) => i === 0 ? 0
          : acc + Math.hypot(p.x - r!.waypoints[i - 1].x, p.y - r!.waypoints[i - 1].y), 0);
        let best = Infinity;
        for (const ei of r!.edgeSeq) {
          const alt = edges.filter((_, i2) => i2 !== ei);
          const r2 = buildNetworkRoute(buildGraphFromEdges(nodes.length, alt), alt, chains, sample.fromNode, sample.toNode);
          if (!r2) continue;
          const len2 = r2.waypoints.reduce((acc, p, i) => i === 0 ? 0
            : acc + Math.hypot(p.x - r2.waypoints[i - 1].x, p.y - r2.waypoints[i - 1].y), 0);
          best = Math.min(best, len2 - base);
        }
        // 归档值定点到 0.1mm；复算与归档允许 1cm 级浮点噪声（两者求和顺序不同）
        if (sample.alternativeDetourM === null) {
          assert.ok(!Number.isFinite(best), '归档说无替代走法，复算也应无');
        } else {
          assert.ok(Number.isFinite(best), '归档说有替代走法，复算也应有');
          assert.ok(Math.abs(best - sample.alternativeDetourM) < 0.01,
            `替代走法应可复算：复算 ${best} vs 归档 ${sample.alternativeDetourM}`);
        }
      });
    });
  }

  it('bj-3rd-ring 是**有环**路网，且抽样路线真有替代走法（"绕行选择"的验收判据）', () => {
    const { map, nodes, edges } = loadNet('bj-3rd-ring');
    assert.strictEqual(map.validation.hasCycle, true, '该图应有环（边 ≥ 节点）');
    const s = map.validation.routeSample as { alternativeDetourM: number | null };
    assert.ok(s.alternativeDetourM !== null && s.alternativeDetourM > 100,
      `抽样路线的替代走法应有可观代价（>100m），实际 ${s.alternativeDetourM}`);
    assert.ok(edges.length > nodes.length);
  });

  it('bj-g6-junction 是树状路网：没有替代走法（对照）', () => {
    const { map } = loadNet('bj-g6-junction');
    const s = map.validation.routeSample as { alternativeDetourM: number | null };
    assert.strictEqual(s.alternativeDetourM, null, '树状路网里任何一段掐掉都不可达 → 无替代');
  });

  it('逐链属性与 chains 对齐、量纲是**米**、半宽自洽（真正防住 px/米搞混）', () => {
    for (const name of ['bj-g6-junction', 'bj-3rd-ring']) {
      const { map, chains } = loadNet(name);
      const attrs = map.network.chainAttrs as {
        highway: string; laneCount: number; laneWidthM: number; halfWidthM: number;
        maxspeedKph: number; lanesFrom: string; maxspeedFrom: string;
      }[];
      assert.strictEqual(attrs.length, chains.length, `${name}: chainAttrs 与 chains 必须等长`);
      for (const a of attrs) {
        assert.ok(a.laneCount >= 1 && a.laneCount <= 8, `${name}: 车道数异常 ${a.laneCount}`);
        // 米制：半宽 = 车道数 × 车道宽 / 2，且必然远小于 12m（px 会大 4 倍）
        assert.ok(Math.abs(a.halfWidthM - a.laneCount * a.laneWidthM / 2) < 1e-3,
          `${name}: 半宽与车道数不自洽 ${a.halfWidthM}`);
        assert.ok(a.halfWidthM > 1 && a.halfWidthM < 12, `${name}: halfWidthM 应是米（实测 ${a.halfWidthM}）`);
        assert.ok(a.maxspeedKph >= 20 && a.maxspeedKph <= 130, `${name}: 限速异常 ${a.maxspeedKph}`);
        assert.ok(['osm', 'derived:class'].includes(a.lanesFrom), `${name}: 来源标注缺失`);
        assert.ok(['osm', 'derived:class'].includes(a.maxspeedFrom), `${name}: 来源标注缺失`);
      }
      // 覆盖率统计必须与实际情况一致（三份北京样本 maxspeed 几乎全无 → 派生为主）
      const cov = map.validation.attrCoverage as { total: number; lanesFromOsm: number; maxspeedFromOsm: number };
      assert.strictEqual(cov.total, attrs.length);
      assert.strictEqual(cov.lanesFromOsm, attrs.filter((a: any) => a.lanesFrom === 'osm').length);
      assert.strictEqual(cov.maxspeedFromOsm, attrs.filter((a: any) => a.maxspeedFrom === 'osm').length);
    }
  });

  it('链式地图的逐段属性区间覆盖整条中心线（G4：3 车道 + 2 车道两段）', () => {
    const map = JSON.parse(fs.readFileSync(path.resolve('maps', 'bj-g4-dujiakan.json'), 'utf8'));
    const spans = map.road.spans as { s0M: number; s1M: number; laneCount: number; halfWidthM: number }[];
    assert.ok(spans.length >= 2, `G4 原始数据里有 2 车道 way → 应至少两段，实际 ${spans.length}`);
    assert.strictEqual(spans[0].s0M, 0, '区间应从 0 开始');
    const lenM = map.validation.lengthM as number;
    assert.ok(Math.abs(spans[spans.length - 1].s1M - lenM) < 5, '区间应覆盖到中心线末端');
    assert.deepStrictEqual([...new Set(spans.map(s => s.laneCount))].sort(), [2, 3]);
    for (const sp of spans) {
      assert.ok(Math.abs(sp.halfWidthM - sp.laneCount * 3.75 / 2) < 1e-3, '半宽与车道数不自洽');
      assert.ok(sp.halfWidthM < 12, 'halfWidthM 应是米');
    }
  });
});
