// ==================== 地图生成：OSM 路网拓扑（多链 + 节点/边图） ====================
//
// 【职责】把 OSM 的一堆 way 变成**可路由的路网**：节点（junction/死端/共享顶点）+ 边（way 片段）。
//
// 【关键事实：匝道接在主线中间顶点上】实测 G6/S11 立交：`motorway` 与 `motorway_link` 的共享
// 坐标**不是** way 端点，而是 way 内部的**形状点**（OSM 里一条高速常是一根长 way 贯穿多个立交）。
// 所以只按"way 端点"建图会得到 0 个路口（实测确实如此：`motorway ∩ link` 端点交集为空）。
// 正确做法：**任意外观顶点若被 ≥2 处共享，就是拓扑节点**，way 在该点被切断成多条边。
//
// 【不做度-2 收缩（edge contraction）】度-2 节点保留在图中：Dijkstra 多走几个节点、
// 渲染多画几段，都不影响正确性；收缩只是"少几条折线"的美观优化，且会引入朝向/合并的边界情况。
// 需要时再加（见 spec 的待办）。

import {
  isOneway, pathLengthM, projectToEnu,
  type LatLon, type OsmWay, type Vec2M,
} from './osm.js';

/** 路网节点（米，局部 ENU） */
export interface NetNode { x: number; y: number }

/** 路网边 = 一条 way 在两个拓扑节点之间的片段 */
export interface NetEdge {
  from: number;
  to: number;
  /** 该片段的折线（米） */
  geom: Vec2M[];
  /** OSM highway 值：motorway | motorway_link | … */
  highway: string;
  ref: string;
  /** 是否单向（false = 双向可通行，建图时两个方向都连） */
  oneway: boolean;
  lengthM: number;
  /** 源 way 的 tags：逐段属性（车道数/限速/宽度）与来源标注都从这里推 */
  tags: Record<string, string>;
}

export interface Topology { nodes: NetNode[]; edges: NetEdge[] }

/** 顶点配对键：0.1mm（与 osm.ts 的端点配对同量级，足够容忍投影浮点差） */
function kp(p: Vec2M): string {
  return `${p.x.toFixed(3)},${p.y.toFixed(3)}`;
}

/**
 * 由 way 列表建拓扑：投影 → 统计顶点共享次数 → 在共享顶点处切断 → 产出节点与边。
 *
 * @param origin 投影原点（缺省 = 第一条 way 的首点）
 */
export function buildTopology(ways: OsmWay[], origin?: LatLon): Topology {
  const usable = ways.filter(w => w.geometry.length >= 2);
  // 【必须用同一个全局原点】`projectToEnu` 缺省以**传入点列的首点**为原点——每条 way 各调一次
  // 会让所有 way 的首顶点都落到 (0,0)，于是整个路网被合并成一个 33 度的星形（实测踩过）。
  const org = origin ?? usable[0]?.geometry[0];
  const proj = usable.map(w => ({ w, pts: projectToEnu(w.geometry, org) }));

  // 顶点出现次数（跨全部 way；同一 way 内重复出现也算 → 自交/绕回也会被切成节点）
  const occ = new Map<string, number>();
  for (const { pts } of proj) {
    for (const p of pts) {
      const k = kp(p);
      occ.set(k, (occ.get(k) ?? 0) + 1);
    }
  }
  const isNodeVertex = (p: Vec2M): boolean => (occ.get(kp(p)) ?? 0) >= 2;

  const nodeId = new Map<string, number>();
  const nodes: NetNode[] = [];
  const idOf = (p: Vec2M): number => {
    const k = kp(p);
    let id = nodeId.get(k);
    if (id === undefined) {
      id = nodes.length;
      nodeId.set(k, id);
      nodes.push({ x: p.x, y: p.y });
    }
    return id;
  };

  const edges: NetEdge[] = [];
  for (const { w, pts } of proj) {
    // 切点 = 两个端点 + 中间的共享顶点
    const marks: number[] = [0];
    for (let i = 1; i < pts.length - 1; i++) if (isNodeVertex(pts[i])) marks.push(i);
    marks.push(pts.length - 1);
    for (let m = 0; m + 1 < marks.length; m++) {
      const seg = pts.slice(marks[m], marks[m + 1] + 1);
      if (seg.length < 2) continue;
      const a = idOf(seg[0]), b = idOf(seg[seg.length - 1]);
      if (a === b) continue;                        // 自环（退化）跳过
      edges.push({
        from: a, to: b, geom: seg,
        highway: w.tags.highway ?? '', ref: w.tags.ref ?? '', tags: w.tags,
        oneway: isOneway(w), lengthM: pathLengthM(seg),
      });
    }
  }
  return { nodes, edges };
}

/**
 * 取**最大连通分量**（按无向连通性，即"物理上连成一片"）。
 *
 * 【为什么需要】一个 bbox 内的 OSM way 常包含若干互不相连的路段（另一条高速、被截断的支线）。
 * 保留它们会让地图出现"孤岛"：Dijkstra 在孤岛之间不可达，且渲染/边界判定要做无用功。
 * 节点 id 会重映射为紧凑序号（按原 id 升序 → 确定性）。
 */
export function largestComponent(topo: Topology): Topology {
  const n = topo.nodes.length;
  const adj: number[][] = Array.from({ length: n }, () => []);
  for (const e of topo.edges) { adj[e.from].push(e.to); adj[e.to].push(e.from); }
  const seen = new Uint8Array(n);
  let best: number[] = [];
  for (let s = 0; s < n; s++) {
    if (seen[s] || adj[s].length === 0) continue;
    const comp: number[] = [];
    const stack = [s];
    seen[s] = 1;
    while (stack.length) {
      const u = stack.pop()!;
      comp.push(u);
      for (const v of adj[u]) if (!seen[v]) { seen[v] = 1; stack.push(v); }
    }
    if (comp.length > best.length) best = comp;
  }
  const keep = best.slice().sort((a, b) => a - b);
  const remap = new Map<number, number>();
  keep.forEach((old, i) => remap.set(old, i));
  return {
    nodes: keep.map(i => ({ ...topo.nodes[i] })),
    edges: topo.edges
      .filter(e => remap.has(e.from) && remap.has(e.to))
      .map(e => ({ ...e, from: remap.get(e.from)!, to: remap.get(e.to)! })),
  };
}

/** 节点度数（含双向边的两次计数；自环不出现，见 buildTopology） */
export function nodeDegrees(topo: Topology): number[] {
  const deg = new Array(topo.nodes.length).fill(0);
  for (const e of topo.edges) {
    deg[e.from] += 1;
    deg[e.to] += 1;
    if (!e.oneway) { deg[e.from] += 1; deg[e.to] += 1; }   // 双向边对度数贡献 2
  }
  return deg;
}

/** 拓扑统计（地图产物验收用）：路口数（度 ≥3）、死端数、按 highway 分类的里程 */
export function topologyStats(topo: Topology): {
  nodes: number; edges: number; junctions: number; deadEnds: number;
  lengthM: number; byHighway: Record<string, { count: number; lengthM: number }>;
} {
  const deg = nodeDegrees(topo);
  const byHighway: Record<string, { count: number; lengthM: number }> = {};
  for (const e of topo.edges) {
    const k = e.highway || '(none)';
    if (!byHighway[k]) byHighway[k] = { count: 0, lengthM: 0 };
    byHighway[k].count += 1;
    byHighway[k].lengthM += e.lengthM;
  }
  return {
    nodes: topo.nodes.length,
    edges: topo.edges.length,
    junctions: deg.filter(d => d >= 3).length,
    deadEnds: deg.filter(d => d === 1).length,
    lengthM: Object.values(byHighway).reduce((s, v) => s + v.lengthM, 0),
    byHighway,
  };
}
