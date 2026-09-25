// ==================== 算法：可行驶走廊（corridor，SL 空间） ====================
//
// 【职责】由 LiDAR 命中（点 + 链段）构建"车辆中心可到达的横向区间"：
//   命中投影到 Frenet (s,l) → 按膨胀量（车半宽 + 裕度）膨胀为阻塞区间 →
//   对每个纵向网格点求 [−lMax, lMax] 内的自由补集。
//
// 【两级走廊（双阈值的新载体，见 spec 2026-09-25-corridor-planner-evolution §3.2）】
//   舒适走廊：膨胀 = 车半宽 + 1.5m → 决定横向排名（晚承诺窗口内）；
//   物理走廊：膨胀 = 车半宽 + 0.4m → 决定停车线（全程）。
//   原"逐命中距离 ≤ CLEAR/HARD_CLEAR"的判定由此统一为"路径是否越出走廊"。
//
// 【圆形膨胀】命中在 (s,l) 平面上按半径 infl 的圆（链段 = 胶囊）膨胀：
//   偏离 s 中心 ds 处，阻塞半宽 = √(infl² − ds²)。这同时解决"点命中落在
//   网格步之间时被整段漏掉"的问题（点障碍的禁止区是圆盘而非一条线）。
//   链段用两端点投影的 (s,l) 包围盒 + 圆膨胀近似（保守；精度由 paths 侧
//   与笛卡尔距离判定的对拍测试兜底）。
//
// 【网格对齐】steps 与路径采样同构（i ↔ s0 + i/steps×horizon），供调用方
// 按索引直接比较路径点（paths.ts 的 firstExit）。

import { FrenetReference } from '../../shared/types.js';
import { m, px } from '../../shared/dimensions.js';
import { cartesianToFrenet, frenetToCartesian } from './frenet.js';

/**
 * 精化投影：cartesianToFrenet 的 s 会吸附到最近参考点（分辨率 = 点间距，约 2.5m），
 * 对走廊精度不够——在最近顶点相邻的两段上做分段最近点投影，得到精确 s 与带符号 l。
 */
function projectToRef(x: number, y: number, ref: FrenetReference): { s: number; l: number } {
  const coarse = cartesianToFrenet(px(x), px(y), ref);
  const n = ref.points.length;
  const idx = coarse.idx;
  // 开环参考线不做首尾绕接：[n−1 → 0] 是"幻影闭合段"（横跨整条参考线的虚线段），
  // 命中点若落在它附近会得到完全错误的 (s,l)（实测：路线末端之外的障碍物被投影到
  // s≈totalS、l≈±15m，膨胀后封死整条走廊 → 假停车）。开环时端点外由"最外段延拓"处理。
  const pairs: [number, number][] = ref.closedLoop
    ? [[(idx - 1 + n) % n, idx], [idx, (idx + 1) % n]]
    : [[Math.max(0, idx - 1), idx], [idx, Math.min(n - 1, idx + 1)]];
  let best: { d: number; s: number; l: number } = { d: Infinity, s: coarse.s, l: coarse.l };
  for (const [a, b] of pairs) {
    if (a === b) continue;
    const ax = ref.points[a].x as number, ay = ref.points[a].y as number;
    const dx = (ref.points[b].x as number) - ax, dy = (ref.points[b].y as number) - ay;
    const len2 = dx * dx + dy * dy || 1;
    const len = Math.sqrt(len2);
    let t = ((x - ax) * dx + (y - ay) * dy) / len2;
    // 最外段允许无钳制延拓（开环参考线两端之外仍是同一 (s,l) 框架，s 越过 totalS）；
    // 内部顶点处钳制，由相邻段接管
    const outer = !ref.closedLoop && (a === 0 || b === n - 1);
    if (!outer) t = Math.max(0, Math.min(1, t));
    const cx = ax + t * dx, cy = ay + t * dy;
    const d = Math.hypot(x - cx, y - cy);
    if (d < best.d) {
      const sgn = (-dy * (x - cx) + dx * (y - cy)) / len;   // 段自身法向（与 ref.normals 同约定）
      best = { d, s: ref.s[a] + t * len, l: sgn >= 0 ? d : -d };
    }
  }
  return { s: best.s, l: best.l };
}

export interface FreeSeg { lmin: number; lmax: number }

export interface CorridorStep {
  s: number;
  /** 该纵向位置自由（车辆中心可到达）的横向区间，升序、互不相交 */
  free: FreeSeg[];
}

export interface Corridor {
  steps: CorridorStep[];
  /** 提交窗口内最窄的"最大自由段"：seg=null/width=0 表示横向全封死；free = 该 s 处全部自由段 */
  pinch: { s: number; width: number; seg: FreeSeg | null; free: FreeSeg[] } | null;
}

/** 由 LiDAR 扫描构建覆盖（用 rawDistances——含边界命中，边界同样遮挡视线；
 *  规划用的 distances 已把边界滤掉，用它会把"墙后"也算作看过）。 */
export function coverageFromScan(
  scan: { angles: number[]; distances: number[]; rawDistances?: number[] },
  carX: number, carY: number,
): Coverage {
  const rays: { angle: number; len: number }[] = [];
  for (let i = 0; i < scan.angles.length; i++) {
    rays.push({ angle: scan.angles[i], len: scan.rawDistances?.[i] ?? scan.distances[i] });
  }
  // 半格：相邻射线夹角按环形差求和（扫描角是绝对方位角，可能跨 ±180°）
  let span = 0;
  for (let i = 1; i < scan.angles.length; i++) {
    let d = scan.angles[i] - scan.angles[i - 1];
    d = Math.atan2(Math.sin(d), Math.cos(d));
    span += Math.abs(d);
  }
  return { rays, carX, carY, halfSpacing: scan.angles.length > 1 ? span / (scan.angles.length - 1) / 2 : 0 };
}

/** 本帧传感器覆盖（B："未知 ≠ 通行"的判据载体）：射线方位 + 长度（用 rawDistances，含边界命中） */
export interface Coverage {
  rays: { angle: number; len: number }[];
  carX: number; carY: number;
  /** 半格（相邻射线夹角的一半）：点落在射线之间的三角空隙也算被扫过 */
  halfSpacing: number;
}

export function buildCorridor(o: {
  ref: FrenetReference; s0: number;
  hits: { x: number; y: number; d: number; low: boolean }[];
  hitSegs: { ax: number; ay: number; bx: number; by: number; low: boolean }[];
  inflate: number; inflateLow: number;
  lMax: number; horizon: number; steps: number; windowRel: number;
  /** 传入后：每步的自由段再与"本帧被雷达扫到的子区间"求交（未观测 = 不可通行） */
  coverage?: Coverage;
}): Corridor {
  // ── 1. 投影：命中 → (s,l) 包围盒 + 膨胀半径 ──
  interface Prim { s1: number; s2: number; l1: number; l2: number; infl: number }
  const prims: Prim[] = [];
  for (const h of o.hits) {
    const fl = projectToRef(h.x, h.y, o.ref);
    prims.push({ s1: fl.s, s2: fl.s, l1: fl.l, l2: fl.l, infl: h.low ? o.inflateLow : o.inflate });
  }
  for (const g of o.hitSegs) {
    const fa = projectToRef(g.ax, g.ay, o.ref);
    const fb = projectToRef(g.bx, g.by, o.ref);
    prims.push({
      s1: Math.min(fa.s, fb.s), s2: Math.max(fa.s, fb.s),
      l1: Math.min(fa.l, fb.l), l2: Math.max(fa.l, fb.l),
      infl: g.low ? o.inflateLow : o.inflate,
    });
  }

  // ── 2. 每个网格步：阻塞并集 → 自由补集 ──
  const steps: CorridorStep[] = [];
  for (let i = 0; i <= o.steps; i++) {
    const s = o.s0 + (i / o.steps) * o.horizon;
    const blocked: FreeSeg[] = [];
    for (const p of prims) {
      const ds = s < p.s1 ? p.s1 - s : (s > p.s2 ? s - p.s2 : 0);
      if (ds > p.infl) continue;                       // 圆膨胀够不到
      const r = Math.sqrt(p.infl * p.infl - ds * ds);  // 圆在该 s 处的半宽
      blocked.push({ lmin: p.l1 - r, lmax: p.l2 + r });
    }
    const free = complement(blocked, o.lMax);
    steps.push({ s, free: o.coverage ? clipToObserved(free, s, o.ref, o.coverage) : free });
  }

  // ── 3. 瓶颈：窗口内"最大自由段宽度"最小的步 ──
  let pinch: Corridor['pinch'] = null;
  for (const st of steps) {
    if (st.s - o.s0 > o.windowRel) break;
    let width = 0, widest: FreeSeg | null = null;
    for (const seg of st.free) {
      const w = seg.lmax - seg.lmin;
      if (w > width) { width = w; widest = seg; }
    }
    if (pinch === null || width < pinch.width) {
      pinch = { s: st.s, width, seg: width > 0 ? { ...widest! } : null, free: st.free.map(f => ({ ...f })) };
    }
  }
  return { steps, pinch };
}

/**
 * 候选目标派生（spec §3.3）：由走廊瓶颈处的自由段生成横向目标。
 *
 *   1. 全封死（pinch.seg=null）→ 单候选 [车道中心]（交由停车线处理）；
 *   2. 取"段中心到车道中心最近"的 **≤2 个自由段**（旧版固定档位打不中缝隙中心）；
 *   3. 每段：段中点（间隙中心）+ 两侧边界内缩 0.3m（窄段去重后自然只剩中点）；
 *   4. 若车道中心落在任一段内，追加为候选；
 *   5. 优先级裁剪到 maxTargets：间隙中心 > 车道中心 > 内缩边界；
 *      同级按 |target − 车道中心| 升序；clamp 到 ±lMax；0.5u 去重；确定性输出。
 */
export function deriveTargets(o: {
  corridor: Corridor; laneOffset: number; l0: number; lMax: number; maxTargets: number;
}): number[] {
  const pinch = o.corridor.pinch;
  if (!pinch || !pinch.seg) return [o.laneOffset];

  const PAD = 1.2;   // 0.3m（仿真单位）
  const cands: { v: number; cls: number }[] = [];
  const push = (v: number, c: number) => {
    const clamped = Math.max(-o.lMax, Math.min(o.lMax, v));
    if (cands.some(x => Math.abs(x.v - clamped) < 0.5)) return;
    cands.push({ v: clamped, cls: c });
  };

  const segMid = (sg: FreeSeg) => (sg.lmin + sg.lmax) / 2;
  const segsSorted = [...pinch.free].sort((a, b) =>
    Math.abs(segMid(a) - o.laneOffset) - Math.abs(segMid(b) - o.laneOffset));
  for (const sg of segsSorted.slice(0, 2)) {
    const mid = segMid(sg);
    push(mid, 0);
    if (sg.lmin + PAD < mid) push(sg.lmin + PAD, 2);
    if (sg.lmax - PAD > mid) push(sg.lmax - PAD, 2);
  }
  if (pinch.free.some(sg => o.laneOffset >= sg.lmin && o.laneOffset <= sg.lmax)) {
    push(o.laneOffset, 1);
  }

  cands.sort((a, b) => a.cls - b.cls || Math.abs(a.v - o.laneOffset) - Math.abs(b.v - o.laneOffset));
  return cands.slice(0, o.maxTargets).map(c => c.v);
}

/**
 * 首个"越出走廊"的弧长（blocked 判定）：逐索引比较路径点与同网格的走廊步
 * （两者由同一 s0/steps/horizon 构造，索引一一对应）。windowRel 限制只看
 * 前段（舒适走廊的晚承诺窗口）；返回 Infinity = 全程未越界。
 */
export function firstExit(
  corridor: Corridor, points: { s: number; l: number }[], windowRel: number,
): number {
  const s0 = corridor.steps[0]?.s ?? 0;
  for (let i = 0; i < points.length && i < corridor.steps.length; i++) {
    if (points[i].s - s0 > windowRel) break;
    const l = points[i].l;
    let inside = false;
    for (const sg of corridor.steps[i].free) {
      if (l >= sg.lmin && l <= sg.lmax) { inside = true; break; }
    }
    if (!inside) return points[i].s;
  }
  return Infinity;
}

/** 把自由段裁到"被观测"的子区间：沿 l 以 0.5m 采样，取连续被观测的 run。
 *  边界处保守收半步（观测→未观测的过渡带归为未观测）。 */
function clipToObserved(segs: FreeSeg[], s: number, ref: FrenetReference, cov: Coverage): FreeSeg[] {
  const SAMPLE = 2;                                  // 0.5m（仿真单位）
  const out: FreeSeg[] = [];
  for (const sg of segs) {
    const n = Math.max(1, Math.ceil((sg.lmax - sg.lmin) / SAMPLE));
    const step = (sg.lmax - sg.lmin) / n;
    let runStart: number | null = null;
    let runStartAtEdge = false;
    for (let i = 0; i <= n; i++) {
      const l = sg.lmin + i * step;
      const p = frenetToCartesian(s, px(l), ref);
      const ok = observedAt(p.x as number, p.y as number, cov.rays, cov.carX, cov.carY, cov.halfSpacing);
      if (ok && runStart === null) { runStart = l; runStartAtEdge = i === 0; }
      if (runStart !== null && (!ok || i === n)) {
        const lo = runStartAtEdge ? runStart : runStart + step / 2;
        const hi = (!ok) ? l - step / 2 : l;         // 观测→未观测：保守收半步
        if (hi > lo) out.push({ lmin: lo, lmax: hi });
        runStart = null;
      }
    }
  }
  return out;
}

/** 阻塞区间的补集（在 [−lMax, lMax] 内）：排序合并后逐段取空档 */
function complement(blocked: FreeSeg[], lMax: number): FreeSeg[] {
  const clipped = blocked
    .map(b => ({ lmin: Math.max(b.lmin, -lMax), lmax: Math.min(b.lmax, lMax) }))
    .filter(b => b.lmax > b.lmin)
    .sort((a, b) => a.lmin - b.lmin);
  const free: FreeSeg[] = [];
  let cur = -lMax;
  for (const b of clipped) {
    if (b.lmin > cur) free.push({ lmin: cur, lmax: b.lmin });
    if (b.lmax > cur) cur = b.lmax;
  }
  if (cur < lMax) free.push({ lmin: cur, lmax: lMax });
  return free;
}

// ═══════════════════════════════════════════════════
// 可视化几何（供渲染层；纯函数、无 THREE 依赖）
// ═══════════════════════════════════════════════════

/** 自由带四边形（笛卡尔）：每对相邻网格步取自由段**交集**，交集为空/过窄则跳过。
 *  取交集是必需的——按序号连相邻步会把带子跨过障碍（步的自由段数目/位置会变）。 */
export function corridorBandPolygons(
  corridor: Corridor, ref: FrenetReference,
): { x: number; y: number }[][] {
  const quads: { x: number; y: number }[][] = [];
  for (let i = 0; i < corridor.steps.length - 1; i++) {
    const a = corridor.steps[i], b = corridor.steps[i + 1];
    for (const sa of a.free) {
      for (const sb of b.free) {
        const lo = Math.max(sa.lmin, sb.lmin), hi = Math.min(sa.lmax, sb.lmax);
        if (hi - lo < 0.1) continue;
        const p1 = frenetToCartesian(a.s, px(lo), ref);
        const p2 = frenetToCartesian(a.s, px(hi), ref);
        const p3 = frenetToCartesian(b.s, px(hi), ref);
        const p4 = frenetToCartesian(b.s, px(lo), ref);
        quads.push([{ x: p1.x, y: p1.y }, { x: p2.x, y: p2.y }, { x: p3.x, y: p3.y }, { x: p4.x, y: p4.y }]);
      }
    }
  }
  return quads;
}

/** 指定 s 处的横断线（笛卡尔）：取最近网格步的自由段；该步全封死 → 一条 ±lMax 满宽标记。
 *  瓶颈线与"观测极限线"共用（渲染层按用途上色）。 */
export function corridorCrossSectionSegments(
  corridor: Corridor, ref: FrenetReference, s: number, lMax: number,
): { x: number; y: number }[][] {
  if (corridor.steps.length === 0) return [];
  let bestStep = corridor.steps[0];
  for (const st of corridor.steps) {
    if (Math.abs(st.s - s) < Math.abs(bestStep.s - s)) bestStep = st;
  }
  const segs: FreeSeg[] = bestStep.free.length > 0 ? bestStep.free : [{ lmin: -lMax, lmax: lMax }];
  return segs.map(sg => {
    const p1 = frenetToCartesian(bestStep.s, px(sg.lmin), ref);
    const p2 = frenetToCartesian(bestStep.s, px(sg.lmax), ref);
    return [{ x: p1.x, y: p1.y }, { x: p2.x, y: p2.y }];
  });
}

/** 瓶颈横断线（笛卡尔）：有缝 → 每个自由段一条；全封死 → 一条 ±lMax 满宽标记。
 *  渲染层按 width>0 / ==0 上色（橙 / 红），封死标记提示"这一段没有任何可走位置"。 */
export function corridorPinchSegments(
  corridor: Corridor, ref: FrenetReference, lMax: number,
): { x: number; y: number }[][] {
  const pinch = corridor.pinch;
  if (!pinch) return [];
  return corridorCrossSectionSegments(corridor, ref, pinch.s, lMax);
}

/**
 * 自由带"是否被本帧雷达真正扫到"（与 corridorBandPolygons 输出一一对应）。
 *
 * 【为什么需要】HD Map 背景减除把道路边界（墙）的命中从规划器输入里剔除了 →
 * 走廊只建模"意外障碍"，看不到"被墙/障碍挡住的阴影区" → 阴影区一律显示为自由。
 * 可视化必须把这段标出来（"未观测" ≠ "自由"），否则图会撒谎。
 *
 * 【判据】以四边形中心点为样本：存在一条射线，其方位与该点方位夹角 ≤ 半格，
 * 且**该射线长度 ≥ 到车的距离**（射线在中途被吸收 = 该点在其阴影里）。判据用
 * **rawDistances**（真实命中：含边界）——规划用的 distances 已经把边界滤掉，
 * 用它会得出"墙后也算观测过"的错误结论。
 */
/** 单点观测谓词：存在方位夹角 ≤ 半格、且长度 ≥ 到车距离的射线 */
function observedAt(
  x: number, y: number, rays: { angle: number; len: number }[],
  carX: number, carY: number, halfSpacing: number,
): boolean {
  const dx = x - carX, dy = y - carY;
  const dist = Math.hypot(dx, dy);
  // 车周豁免：车自身周边（约一个车长）视为已知——车此刻就在那里（或刚从那里驶出），
  // 下一帧就会重新观测；且传感器近场自遮挡会让此处判"未观测"从而把车自己判死。
  if (dist < (m(5) as number)) return true;
  const bear = Math.atan2(dy, dx);
  let bestD = Infinity, bestLen = 0;
  for (const r of rays) {
    let d = bear - r.angle;
    d = Math.atan2(Math.sin(d), Math.cos(d));
    const ad = Math.abs(d);
    if (ad < bestD) { bestD = ad; bestLen = r.len; }
  }
  return bestD <= halfSpacing + 1e-9 && bestLen >= dist - 1;
}

/** 自由带是否被本帧雷达扫到（与 corridorBandPolygons 输出一一对应；取四边形中心为样本） */
export function corridorBandObserved(
  quads: { x: number; y: number }[][],
  rays: { angle: number; len: number }[],
  carX: number, carY: number, halfSpacing: number,
): boolean[] {
  return quads.map(q => {
    if (q.length === 0) return false;
    let cx = 0, cy = 0;
    for (const p of q) { cx += p.x; cy += p.y; }
    return observedAt(cx / q.length, cy / q.length, rays, carX, carY, halfSpacing);
  });
}

/** 观测极限弧长：最后一个"自由段中点被观测"的网格步（全观测或全未观测 → null）。
 *  样本取该步**最宽自由段的中点**（该处才是车可能走的位置；封死步记 false）。 */
export function corridorObservedLimitS(
  corridor: Corridor, ref: FrenetReference,
  rays: { angle: number; len: number }[], carX: number, carY: number, halfSpacing: number,
): number | null {
  let lastObserved: number | null = null;
  let sawUnobserved = false;
  for (const st of corridor.steps) {
    let widest: FreeSeg | null = null;
    for (const sg of st.free) if (!widest || (sg.lmax - sg.lmin) > (widest.lmax - widest.lmin)) widest = sg;
    const ok = widest !== null
      && observedAt(
        (frenetToCartesian(st.s, px((widest.lmin + widest.lmax) / 2), ref).x) as number,
        (frenetToCartesian(st.s, px((widest.lmin + widest.lmax) / 2), ref).y) as number,
        rays, carX, carY, halfSpacing,
      );
    if (ok) lastObserved = st.s; else sawUnobserved = true;
  }
  return sawUnobserved ? lastObserved : null;
}

/** 一组四边形的**外边界**线段（共享边去重后剩下的边；供"未观测区"描边）。
 *  端点按 3 位小数归一化后比较（浮点误差容忍），结果顺序确定（先出现者先出）。 */
export function bandBoundary(quads: { x: number; y: number }[][]): { x: number; y: number }[][] {
  const key = (p: { x: number; y: number }) => `${p.x.toFixed(3)},${p.y.toFixed(3)}`;
  const seen = new Map<string, { a: { x: number; y: number }; b: { x: number; y: number }; n: number }>();
  for (const q of quads) {
    for (let i = 0; i < q.length; i++) {
      const a = q[i], b = q[(i + 1) % q.length];
      const ka = key(a), kb = key(b);
      const k = ka < kb ? `${ka}|${kb}` : `${kb}|${ka}`;
      const hit = seen.get(k);
      if (hit) hit.n += 1;
      else seen.set(k, { a, b, n: 1 });
    }
  }
  const out: { x: number; y: number }[][] = [];
  for (const seg of seen.values()) if (seg.n === 1) out.push([seg.a, seg.b]);
  return out;
}
