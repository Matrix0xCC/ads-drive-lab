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

import { Point2D } from '../shared/types.js';

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
