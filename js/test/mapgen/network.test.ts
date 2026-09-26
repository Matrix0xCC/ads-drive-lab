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

describe('mapgen/入库路网地图契约（bj-g6-junction）', () => {
  const NAME = 'bj-g6-junction';
  const map = JSON.parse(fs.readFileSync(path.resolve('maps', `${NAME}.json`), 'utf8'));
  const nodes = map.network.nodes as [number, number][];
  const edges = map.network.edges as { from: number; to: number; chain: number; highway: string; lengthM: number }[];
  const chains = (map.network.chains as [number, number][][])
    .map(ch => ch.map(([x, y]) => ({ x: px(x), y: px(y) })));

  it('与入库原始数据一致，且含主线 + 匝道', () => {
    const sha = createHash('sha256')
      .update(fs.readFileSync(path.resolve('maps/raw', `${NAME}.osm.json`), 'utf8')).digest('hex');
    assert.strictEqual(map.source.rawSha256, sha, '地图与原始 OSM 响应不一致');
    assert.strictEqual(map.kind, 'network');
    const byH: Record<string, number> = {};
    for (const e of edges) byH[e.highway] = (byH[e.highway] ?? 0) + 1;
    assert.ok((byH['motorway'] ?? 0) > 0 && (byH['motorway_link'] ?? 0) > 0,
      `应同时含主线与匝道，实际 ${JSON.stringify(byH)}`);
  });

  it('是**真路网**：有路口（度≥3）而不是一条链', () => {
    const deg = new Array(nodes.length).fill(0);
    for (const e of edges) { deg[e.from]++; deg[e.to]++; }
    assert.ok(deg.filter(d => d >= 3).length >= 1, '至少一个路口');
    assert.ok(edges.length > 10, `边数应远多于链式地图（实际 ${edges.length}）`);
    assert.ok(map.validation.lengthM > 1000);
    // 【信息性】有环（边数 ≥ 节点数）才意味着"存在替代路径可选"；本 bbox 是树（匝道都悬空到
    // 未抓取的普通道路），故只记录不强制。要造出"可选路径"需取两条高速之间的立交 bbox。
    assert.strictEqual(map.validation.hasCycle, edges.length >= nodes.length);
  });

  it('索引自洽：edges[].chain 指向存在的链，链的首尾与节点坐标一致', () => {
    for (const e of edges) {
      const ch = chains[e.chain];
      assert.ok(ch && ch.length >= 2, `边 ${e.chain} 的链缺失`);
      const a = nodes[e.from], b = nodes[e.to];
      const dHead = Math.hypot(ch[0].x - a[0], ch[0].y - a[1]);
      const dTail = Math.hypot(ch[ch.length - 1].x - b[0], ch[ch.length - 1].y - b[1]);
      assert.ok(dHead < 1e-6 && dTail < 1e-6, `链 ${e.chain} 端点与节点不吻合`);
    }
  });

  it('**路由真的会经过匝道**（验收判据：地图让 Dijkstra 做出选择）', () => {
    const graph = buildGraphFromEdges(nodes.length, edges);
    const sample = map.validation.routeSample as { fromNode: number; toNode: number; linksUsed: number };
    const r = buildNetworkRoute(graph, edges, chains, sample.fromNode, sample.toNode);
    assert.ok(r, '抽样起终点应可达');
    const links = r!.chainSeq.filter(ci => edges[ci].highway === 'motorway_link').length;
    assert.ok(links >= 1, `路线应经过匝道，实际 ${links} 段`);
    assert.strictEqual(links, sample.linksUsed, '记录与复算应一致（确定性）');
    // 复算两次结果一致
    const r2 = buildNetworkRoute(graph, edges, chains, sample.fromNode, sample.toNode);
    assert.deepStrictEqual(r2!.chainSeq, r!.chainSeq);
  });
});
