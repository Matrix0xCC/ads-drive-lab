// ==================== 算法：全局路径规划（Dijkstra） ====================
//
// 【职责】在赛道中心线构成的路网上，找到从起点到终点的最短路径。
//
// 【路网表示】
// 赛道中心线有约 200 个采样点，每个点是一个路网节点。
// 相邻节点之间有一条无向边，边权 = 欧氏距离。
// 闭环赛道首尾相连。
//
// 【为什么用 Dijkstra 而不是 A*？】
// 路网只有 200 个节点、约 400 条边，Dijkstra 在 << 1ms 内完成，
// 不需要启发式函数的额外复杂度和潜在的次优路径。

import { Point2D, AttrSpan } from '../shared/types.js';
import { px } from '../shared/dimensions.js';

// ═══════════════════════════════════════════════════
// 路网类型
// ═══════════════════════════════════════════════════

/** 邻接表：adjacency[nodeIdx] = [neighborIdx, edgeCost][] */
export interface RoadGraph {
  adjacency: [number, number][][];
  nodeCount: number;
}

// ═══════════════════════════════════════════════════
// 路网构建
// ═══════════════════════════════════════════════════

/**
 * 从道路中心点构建路网
 *
 * 每个中心点是一个节点，相邻节点双向连接。
 * - 闭环（赛道）：首尾相连（index 0 ↔ index N-1），中心线首尾重复点算一个节点 → 节点数 N-1；
 * - 开环（OSM 高速主线）：**不绕接**，节点数 N，只有 i↔i+1 的边（不能从终点瞬移回起点）。
 *
 * 【复杂度】O(N)；径向图 N≈200，OSM 2km 图 N≈668
 */
export function buildRoadGraph(centerPoints: Point2D[], closed = true): RoadGraph {
  const N = centerPoints.length;
  const nodeCount = closed ? N - 1 : N;
  const adjacency: [number, number][][] = Array.from({ length: nodeCount }, () => []);

  const link = (a: number, b: number) => {
    if (a === b || a < 0 || b < 0 || a >= nodeCount || b >= nodeCount) return;
    const dist = Math.hypot(
      (centerPoints[b].x as number) - (centerPoints[a].x as number),
      (centerPoints[b].y as number) - (centerPoints[a].y as number),
    );
    const cost = Math.round(dist);        // 整数化边权
    adjacency[a].push([b, cost]);
    adjacency[b].push([a, cost]);
  };
  for (let i = 0; i < nodeCount - 1; i++) link(i, i + 1);
  if (closed) link(nodeCount - 1, 0);     // 闭环才接首尾

  return { adjacency, nodeCount };
}

/**
 * 由**显式边表**建路网（OSM 路网地图用）：与 `buildRoadGraph` 的"链式"不同，
 * 这里每条边独立给出 `from/to/lengthM`，支持路口、匝道、单向。
 *
 * @param edges oneway=true 只连 from→to；否则双向各一条边（edge 权 = 边长取整）
 */
/** 折线长度（px）：路线的 s 轴就是它 */
function polylineLen(pts: Point2D[]): number {
  let acc = 0;
  for (let i = 1; i < pts.length; i++) {
    acc += Math.hypot((pts[i].x as number) - (pts[i - 1].x as number), (pts[i].y as number) - (pts[i - 1].y as number));
  }
  return acc;
}

export function buildGraphFromEdges(
  nodeCount: number,
  edges: { from: number; to: number; chain?: number; lengthM?: number; oneway?: boolean }[],
  chains?: Point2D[][],
): RoadGraph {
  /** 边长缺省时从链几何现算（地图文件通常带 lengthM；这里是兜底） */
  const lengthOf = (e: { chain?: number; lengthM?: number }): number => {
    if (typeof e.lengthM === 'number') return e.lengthM;
    const ch = e.chain !== undefined ? chains?.[e.chain] : undefined;
    if (!ch) return 1;
    let acc = 0;
    for (let i = 1; i < ch.length; i++) {
      acc += Math.hypot((ch[i].x as number) - (ch[i - 1].x as number), (ch[i].y as number) - (ch[i - 1].y as number));
    }
    return acc;
  };
  const adjacency: [number, number][][] = Array.from({ length: nodeCount }, () => []);
  for (const e of edges) {
    if (e.from === e.to || e.from < 0 || e.to < 0 || e.from >= nodeCount || e.to >= nodeCount) continue;
    const cost = Math.max(1, Math.round(lengthOf(e)));
    adjacency[e.from].push([e.to, cost]);
    if (!e.oneway) adjacency[e.to].push([e.from, cost]);
  }
  return { adjacency, nodeCount };
}

/**
 * 路网路线装配：Dijkstra 出**节点序列** → 逐段找边 → 取该边对应的链几何（必要时反向）拼接。
 *
 * 【为什么路线要落在链上】规划器/感知用的是"路线折线"（开环 Frenet 参考线）。
 * 路网地图里"链"= 一段可行驶几何，边 = (节点对 → 链) 的映射；把节点序列翻译成折线即可。
 * 平行边（同一节点对有多条链）取第一条（确定性：按 edges 数组顺序）。
 *
 * @returns waypoints + 途经链序号（供检测器/调试看"有没有真的经过匝道"）；不可达返回 null
 */
export function buildNetworkRoute(
  graph: RoadGraph,
  edges: { from: number; to: number; chain: number }[],
  chains: Point2D[][],
  fromNode: number, toNode: number,
  chainAttrs?: { highway: string; laneCount: number; laneWidthM: number; halfWidthM: number; maxspeedKph: number }[],
): { waypoints: Point2D[]; chainSeq: number[]; edgeSeq: number[]; attrSpans: AttrSpan[] } | null {
  const nodePath = findRoute(graph, fromNode, toNode);
  if (!nodePath || nodePath.length < 2) return null;
  const waypoints: Point2D[] = [];
  const chainSeq: number[] = [];
  const edgeSeq: number[] = [];
  /** 逐段属性：按"该链在路线上贡献的弧长区间"累积，相邻同属性合并 */
  const attrSpans: AttrSpan[] = [];
  const pushSpan = (u: number, a?: { highway: string; laneCount: number; laneWidthM: number; halfWidthM: number; maxspeedKph: number }) => {
    if (!a) return;
    const last = attrSpans[attrSpans.length - 1];
    if (last && last.halfWidthM === a.halfWidthM && last.maxspeedKph === a.maxspeedKph
      && last.laneCount === a.laneCount && last.highway === a.highway) { last.s1 = px(u); return; }
    attrSpans.push({
      highway: a.highway, laneCount: a.laneCount, laneWidthM: a.laneWidthM, halfWidthM: a.halfWidthM,
      maxspeedKph: a.maxspeedKph, s0: px(last ? (last.s1 as number) : 0), s1: px(u),
    });
  };
  for (let i = 0; i + 1 < nodePath.length; i++) {
    const a = nodePath[i], b = nodePath[i + 1];
    const ei = edges.findIndex(e => (e.from === a && e.to === b) || (e.from === b && e.to === a));
    const edge = ei >= 0 ? edges[ei] : undefined;
    if (!edge) return null;
    const geom = chains[edge.chain];
    if (!geom || geom.length < 2) return null;
    const pts = edge.from === a ? geom : [...geom].reverse();   // 从 a 走向 b
    for (const p of pts) {
      const last = waypoints[waypoints.length - 1];
      if (last && last.x === p.x && last.y === p.y) continue;   // 接缝去重
      waypoints.push({ x: p.x, y: p.y });
    }
    chainSeq.push(edge.chain);
    edgeSeq.push(ei);
    // 该链贡献到路线末端的弧长（waypoints 长度就是路线 s）
    pushSpan(polylineLen(waypoints), chainAttrs?.[edge.chain]);
  }
  return waypoints.length >= 2 ? { waypoints, chainSeq, edgeSeq, attrSpans } : null;
}

// ═══════════════════════════════════════════════════
// Dijkstra 最短路径
// ═══════════════════════════════════════════════════

/**
 * Dijkstra 最短路径
 *
 * 使用二叉最小堆作为优先队列。
 * 对于 N=200，搜索在 << 1ms 内完成。
 *
 * @param graph      路网
 * @param startIndex 起点节点索引
 * @param goalIndex  终点节点索引
 * @returns 从起点到终点的有序节点索引数组，或 null（不可达）
 */
export function findRoute(graph: RoadGraph, startIndex: number, goalIndex: number): number[] | null {
  const { adjacency, nodeCount } = graph;

  if (startIndex === goalIndex) return [startIndex];

  // dist[node] = 从 start 到 node 的最短距离
  const dist = new Float64Array(nodeCount).fill(Infinity);
  const prev = new Int32Array(nodeCount).fill(-1);
  const visited = new Uint8Array(nodeCount);

  dist[startIndex] = 0;

  // 二叉最小堆：[cost, nodeIdx][]
  const heap: [number, number][] = [[0, startIndex]];

  function heapPush(cost: number, node: number) {
    heap.push([cost, node]);
    let i = heap.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (heap[p][0] <= heap[i][0]) break;
      [heap[p], heap[i]] = [heap[i], heap[p]];
      i = p;
    }
  }

  function heapPop(): [number, number] | null {
    if (heap.length === 0) return null;
    const top = heap[0];
    const last = heap.pop()!;
    if (heap.length > 0) {
      heap[0] = last;
      let i = 0;
      const n = heap.length;
      while (true) {
        let smallest = i;
        const l = 2 * i + 1;
        const r = 2 * i + 2;
        if (l < n && heap[l][0] < heap[smallest][0]) smallest = l;
        if (r < n && heap[r][0] < heap[smallest][0]) smallest = r;
        if (smallest === i) break;
        [heap[i], heap[smallest]] = [heap[smallest], heap[i]];
        i = smallest;
      }
    }
    return top;
  }

  while (heap.length > 0) {
    const [d, u] = heapPop()!;
    if (visited[u]) continue;
    visited[u] = 1;

    if (u === goalIndex) break;

    for (const [v, w] of adjacency[u]) {
      if (visited[v]) continue;
      const nd = d + w;
      if (nd < dist[v]) {
        dist[v] = nd;
        prev[v] = u;
        heapPush(nd, v);
      }
    }
  }

  // 不可达
  if (prev[goalIndex] === -1 && startIndex !== goalIndex) return null;

  // 回溯路径
  const path: number[] = [];
  let curr = goalIndex;
  while (curr !== -1) {
    path.push(curr);
    curr = prev[curr];
  }
  path.reverse();
  return path;
}

// ═══════════════════════════════════════════════════
// 点击 → 路网节点映射
// ═══════════════════════════════════════════════════

/**
 * 找到离给定世界坐标最近的中心点索引
 *
 * 用于将用户点击位置映射到路网节点。
 * 线性扫描 N=200，每一帧点击只调用一次，性能完全不是问题。
 */
export function findClosestPoint(posX: number, posY: number, centerPoints: Point2D[], closed = true): number {
  let minDist = Infinity;
  let bestIdx = 0;
  const N = closed ? centerPoints.length - 1 : centerPoints.length; // 闭环跳过重复点
  for (let i = 0; i < N; i++) {
    const dx = posX - (centerPoints[i].x as number);
    const dy = posY - (centerPoints[i].y as number);
    const d = dx * dx + dy * dy; // 平方距离（避免 sqrt）
    if (d < minDist) { minDist = d; bestIdx = i; }
  }
  return bestIdx;
}
