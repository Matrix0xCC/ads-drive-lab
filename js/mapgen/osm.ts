// ==================== 地图生成：OSM 高速主线 → 仿真地图（纯几何，无 I/O） ====================
//
// 【职责】把 Overpass 返回的 OSM way 原始几何，处理成仿真器可用的"开环等宽道路中心线"
// （局部米制 ENU、近似均匀步长、曲率受控、可复现）。
//
// 【为什么需要这一层】原始 OSM 节点间距实测 p50 = 46.6m、p95 = 191m、max = 388m；
// 直接线性重采样会把**每个节点变成一个角**——而规划器的 developMin（发展段下限）、
// lateralSpeedCap（横向机动速度上限）、marginFor（误差裕度）全都读参考线曲率，
// 棱角会变成假的"处处不可行"（实测原始折线差分曲率 p95 = 1.9e-2 1/m，R≈53m，
// 而 30m/s 下 R=53m 需 17m/s² 横向加速度，规划上限是 3.5）。
// 所以：**把 OSM 节点当作控制多边形**，Catmull-Rom 插值后再按固定弧长采样。
//
// 【为什么合并 way 只能按端点坐标】Overpass 的 `out geom` 只返回 geometry 的 lat/lon，
// **不含 nodes id 数组**（实测确认）；OSM 中共享节点的坐标逐位相同，故按 1e-7°（≈1cm）
// 取整做键即可精确配对。
//
// 【单位】本模块一律用**米**（普通 number）——它是仿真之前的构建期工具，不参与
// shared/dimensions.ts 的品牌类型体系；写入地图文件后由加载层转成仿真单位。
// 【确定性】所有导出函数都是纯函数；输出集合的排序由 way id 决定，不依赖输入顺序。

/** 经纬度（度）。OSM 原始坐标系 WGS84 */
export interface LatLon { lat: number; lon: number }

/** 我们需要的 OSM way 子集（Overpass `out geom` 的形态） */
export interface OsmWay {
  id: number;
  tags: Record<string, string>;
  /** 至少 2 个点；方向按 way 定义（可能被 oneway=-1 反转，见 parseOverpass） */
  geometry: LatLon[];
}

/** 平面坐标（米，局部 ENU：x 东、y 北） */
export interface Vec2M { x: number; y: number }

export const EARTH_R_M = 6371000;
/**
 * 横向加速度上限（m/s²）——镜像 `js/algo/plan/paths.ts` 的 A_LAT_MAX（3.5）。
 * 那里是模块私有常量，这里不 import（避免构建期工具拖入规划器依赖）；改动需两处同步。
 */
export const A_LAT_MAX_MPS2 = 3.5;

/** 端点配对键：1e-7° ≈ 1cm，足以精确配对共享节点，又能容忍浮点打印误差 */
function keyOf(p: LatLon): string {
  return `${p.lat.toFixed(7)},${p.lon.toFixed(7)}`;
}

function isLatLon(v: unknown): v is LatLon {
  const p = v as LatLon;
  return !!p && typeof p.lat === 'number' && typeof p.lon === 'number'
    && Number.isFinite(p.lat) && Number.isFinite(p.lon);
}

/**
 * 解析 Overpass 响应 → OsmWay[]。
 *
 * 【容错口径】顶层结构不对（无 elements 数组）→ 抛错（调用方拿到的是错误页/空响应，必须显性失败）；
 * 单个元素不合法（非 way / 几何点不足 / 坐标非数）→ 跳过（Overpass 的 `out geom` 有时含
 * 无几何的关系成员或退化 way，不该让整批失败）。
 * 【方向归一】`oneway=-1` 表示几何方向与通行方向相反 → 在这里就地反转，下游只管"链方向 = 通行方向"。
 */
export function parseOverpass(json: unknown): OsmWay[] {
  const o = json as { elements?: unknown };
  if (!o || !Array.isArray(o.elements)) throw new Error('Overpass 响应缺少 elements 数组（可能是错误页或限流响应）');
  const out: OsmWay[] = [];
  for (const el of o.elements as any[]) {
    if (!el || el.type !== 'way') continue;
    if (typeof el.id !== 'number' || !Array.isArray(el.geometry)) continue;
    const geom = (el.geometry as unknown[]).filter(isLatLon).map(p => ({ lat: p.lat, lon: p.lon }));
    if (geom.length < 2) continue;
    const tags: Record<string, string> = {};
    if (el.tags && typeof el.tags === 'object') {
      for (const [k, v] of Object.entries(el.tags as Record<string, unknown>)) {
        if (typeof v === 'string') tags[k] = v;
      }
    }
    if (tags.oneway === '-1') geom.reverse();
    out.push({ id: el.id, tags, geometry: geom });
  }
  // 顺序规范化：按 way id 升序（OSM id 全局唯一 → 完全确定）
  out.sort((a, b) => a.id - b.id);
  return out;
}

/** 该 way 是否为单向（含 oneway=-1，其几何已在 parseOverpass 里反转） */
export function isOneway(w: OsmWay): boolean {
  const v = w.tags.oneway;
  return v === 'yes' || v === '1' || v === '-1' || v === 'true';
}

/** ref 是否命中（`ref` / `int_ref`，按 `;` 拆分后精确比对，如 "G4;G107" 命中 "G4"） */
export function refMatches(w: OsmWay, ref: string): boolean {
  const want = ref.trim().toUpperCase();
  for (const k of ['ref', 'int_ref']) {
    const v = w.tags[k];
    if (!v) continue;
    if (v.split(';').map(s => s.trim().toUpperCase()).includes(want)) return true;
  }
  return false;
}

/**
 * 按端点坐标把 way 合并成**极大链**（每条链内相邻 way 共用一个端点）。
 *
 * 【算法】并查集按"共用端点键"分组 → 组内从"自由端"（只被一个 way 占用）开始走，
 * 边走边定向（若当前 way 的尾端不等于游标，则反转其几何）→ 直到无路可走。
 * 有分支/不连通的脏数据时，剩余 way 另起一条链（**不丢数据**），调用方按长度取舍。
 * 【确定性】起点优先取自由端，多个自由端按键名排序；无自由端（闭环）时取组内最小 way id 的首端；
 * 相邻 way 候选按 id 升序。输出链按首 way id 升序。
 */
export function chainWays(ways: OsmWay[]): OsmWay[][] {
  const n = ways.length;
  if (n === 0) return [];
  const parent = Array.from({ length: n }, (_, i) => i);
  const find = (x: number): number => {
    let r = x;
    while (parent[r] !== r) r = parent[r];
    while (parent[x] !== r) { const nx = parent[x]; parent[x] = r; x = nx; }
    return r;
  };
  const union = (a: number, b: number): void => {
    const ra = find(a), rb = find(b);
    if (ra !== rb) parent[Math.max(ra, rb)] = Math.min(ra, rb);
  };

  const ends: [string, string][] = [];
  const firstSeen = new Map<string, number>();
  for (let i = 0; i < n; i++) {
    const g = ways[i].geometry;
    const k0 = keyOf(g[0]), k1 = keyOf(g[g.length - 1]);
    ends.push([k0, k1]);
    for (const k of [k0, k1]) {
      const s = firstSeen.get(k);
      if (s === undefined) firstSeen.set(k, i);
      else union(s, i);
    }
  }

  const groups = new Map<number, number[]>();
  for (let i = 0; i < n; i++) {
    const r = find(i);
    if (!groups.has(r)) groups.set(r, []);
    groups.get(r)!.push(i);
  }

  const chains: OsmWay[][] = [];
  for (const idxs of [...groups.values()].sort((a, b) => a[0] - b[0])) {
    const inc = new Map<string, number[]>();          // 端点键 → 相邻 way 索引
    for (const i of idxs) {
      for (const k of ends[i]) {
        if (!inc.has(k)) inc.set(k, []);
        inc.get(k)!.push(i);
      }
    }
    const freeEnds = [...inc.entries()].filter(([, v]) => v.length === 1)
      .map(([k]) => k).sort();
    const used = new Set<number>();
    while (used.size < idxs.length) {
      const unused = idxs.filter(i => !used.has(i));
      const free = freeEnds.filter(k => (inc.get(k) ?? []).some(i => !used.has(i)));
      let cur = free.length > 0 ? free[0] : ends[Math.min(...unused)][0];
      const ordered: OsmWay[] = [];
      for (;;) {
        const cands = (inc.get(cur) ?? []).filter(i => !used.has(i)).sort((a, b) => a - b);
        if (cands.length === 0) break;
        const i = cands[0];
        used.add(i);
        const g = ways[i].geometry;
        if (ends[i][0] === cur) { ordered.push(ways[i]); cur = ends[i][1]; }
        else { ordered.push({ ...ways[i], geometry: [...g].reverse() }); cur = ends[i][0]; }
      }
      if (ordered.length > 0) chains.push(ordered);
      else {                                            // 防御：无法推进（自环等）→ 单 way 成链
        const i = Math.min(...unused);
        used.add(i);
        chains.push([ways[i]]);
      }
    }
  }
  chains.sort((a, b) => a[0].id - b[0].id);
  return chains;
}

/** 链 → 点列：相邻 way 的接缝点只保留一个；顺带去掉连续重复点 */
export function wayPoints(ways: OsmWay[]): LatLon[] {
  const pts: LatLon[] = [];
  for (const w of ways) {
    for (const p of w.geometry) {
      const last = pts[pts.length - 1];
      if (last && last.lat === p.lat && last.lon === p.lon) continue;
      pts.push({ lat: p.lat, lon: p.lon });
    }
  }
  return pts;
}

/** 折线长度（米，按投影后的平面坐标算） */
export function pathLengthM(points: Vec2M[]): number {
  let acc = 0;
  for (let i = 1; i < points.length; i++) acc += Math.hypot(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y);
  return acc;
}

/**
 * 选出一条车行道链：单向 + ref 命中 → 取最长（并列时首 way id 小者）。
 * @returns 命中的链；无命中返回 null
 */
export function selectCarriageway(
  chains: OsmWay[][], opts: { ref?: string; minLengthM?: number; origin?: LatLon } = {},
): OsmWay[] | null {
  let best: { chain: OsmWay[]; len: number } | null = null;
  for (const chain of chains) {
    if (!chain.every(isOneway)) continue;
    if (opts.ref && !chain.some(w => refMatches(w, opts.ref!))) continue;
    const pts = projectToEnu(wayPoints(chain), opts.origin);
    const len = pathLengthM(pts);
    if (opts.minLengthM !== undefined && len < opts.minLengthM) continue;
    if (!best || len > best.len || (len === best.len && chain[0].id < best.chain[0].id)) best = { chain, len };
  }
  return best ? best.chain : null;
}

/**
 * 等距圆柱投影（equirectangular）：x = (lon−lon0)·cos(lat0)·R，y = (lat−lat0)·R。
 *
 * 【为什么不用 UTM】1.5km 尺度下相对误差 <1e-3，可忽略；省掉投影库与带号逻辑。
 * 注意 κ 由坐标二阶导得到，**均匀缩放不改变曲率半径的量级**，故投影近似的这点误差
 * 不影响线形判定。
 * @param origin 原点（缺省取首点）
 */
export function projectToEnu(points: LatLon[], origin?: LatLon): Vec2M[] {
  if (points.length === 0) return [];
  const o = origin ?? points[0];
  const kx = Math.cos(o.lat * Math.PI / 180) * EARTH_R_M * Math.PI / 180;
  const ky = EARTH_R_M * Math.PI / 180;
  return points.map(p => ({ x: (p.lon - o.lon) * kx, y: (p.lat - o.lat) * ky }));
}

/** Catmull-Rom 插值（与 env/track.ts 同一套式子）：过 p1→p2，p0/p3 提供切线 */
function catmullRom(p0: Vec2M, p1: Vec2M, p2: Vec2M, p3: Vec2M, t: number): Vec2M {
  const t2 = t * t, t3 = t2 * t;
  return {
    x: 0.5 * ((2 * p1.x) + (-p0.x + p2.x) * t + (2 * p0.x - 5 * p1.x + 4 * p2.x - p3.x) * t2 + (-p0.x + 3 * p1.x - 3 * p2.x + p3.x) * t3),
    y: 0.5 * ((2 * p1.y) + (-p0.y + p2.y) * t + (2 * p0.y - 5 * p1.y + 4 * p2.y - p3.y) * t2 + (-p0.y + 3 * p1.y - 3 * p2.y + p3.y) * t3),
  };
}

/** 折线上按弧长插值取点 */
function atArcLength(points: Vec2M[], s: number): Vec2M {
  if (points.length === 0) return { x: 0, y: 0 };
  if (s <= 0) return { ...points[0] };
  let acc = 0;
  for (let i = 1; i < points.length; i++) {
    const seg = Math.hypot(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y);
    if (acc + seg >= s) {
      const t = seg > 0 ? (s - acc) / seg : 0;
      return { x: points[i - 1].x + t * (points[i].x - points[i - 1].x),
        y: points[i - 1].y + t * (points[i].y - points[i - 1].y) };
    }
    acc += seg;
  }
  return { ...points[points.length - 1] };
}

/**
 * 线性按弧长均匀重采样（不插值外推、不改变线形；保留端点）。
 * 用于把"控制点列"变成间距可控的密折线，作为平滑的输入。
 */
export function resampleUniform(points: Vec2M[], stepM: number): Vec2M[] {
  if (points.length < 2) return points.map(p => ({ ...p }));
  const total = pathLengthM(points);
  const count = Math.max(1, Math.round(total / stepM));
  const out: Vec2M[] = [];
  for (let i = 0; i <= count; i++) out.push(atArcLength(points, (i / count) * total));
  return out.filter((p, i) => i === 0 || Math.hypot(p.x - out[i - 1].x, p.y - out[i - 1].y) > 1e-9);
}

/**
 * 按**弧长窗口**平滑（三角核），端点固定不动。
 *
 * 【为什么必须是弧长窗口，而不是"相邻 N 点平均"】OSM 节点间距实测 p50=46.6m、
 * p95=191m、max=388m——按点数平均在稀疏段几乎不生效、在密集段过度平滑；按米数
 * 取窗才有统一的物理含义。
 *
 * 【为什么需要这一步】实测（京港澳 G4 北京段）真实单节点位置抖动会在航向上造出
 * ±7° 的鼓包（航向序列 23.2→27.5→33.9→30.2→23.7，真实曲线应单调变化），
 * 经 3m 重采样后表现为 κ max = 3.2e-2（R=31m）的假急弯——而规划器按 κ 判可行性，
 * 会把整段判成必须限速到 38km/h。
 *
 * 【窗口选择】窗口应显著小于真实最小曲率半径（R≥300m 时取 30m ≈ R 的 10%），
 * 否则会把真弯一起削平。端点固定：地图起终点由收费站桩号决定，不能被平滑挪动。
 */
export function smoothByArcLength(points: Vec2M[], windowM: number): Vec2M[] {
  const n = points.length;
  if (n < 3 || !(windowM > 0)) return points.map(p => ({ ...p }));
  const s: number[] = new Array(n).fill(0);
  for (let i = 1; i < n; i++) s[i] = s[i - 1] + Math.hypot(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y);
  const half = windowM / 2;
  const out: Vec2M[] = [{ ...points[0] }];
  for (let i = 1; i < n - 1; i++) {
    let wx = 0, wy = 0, wsum = 0;
    for (let j = 0; j < n; j++) {
      const d = Math.abs(s[j] - s[i]);
      if (d > half) continue;
      const w = 1 - d / half;                       // 三角核
      wx += w * points[j].x; wy += w * points[j].y; wsum += w;
    }
    out.push(wsum > 1e-12 ? { x: wx / wsum, y: wy / wsum } : { ...points[i] });
  }
  out.push({ ...points[n - 1] });
  return out;
}

/**
 * 平滑 + 重采样：把**控制点**列表当控制多边形，Catmull-Rom 插值后按弧长均匀采样。
 *
 * 【两步】① 每段按 stepM/4 密集采样（把样条变成足够密的折线）；
 * ② 沿密集折线按弧长每 stepM 取一点 → 输出**近似等距**且曲率连续的线形。
 * 首尾点保证保留（端点不插值外推）。
 *
 * 注意：Catmull-Rom 是**插值**样条（过控制点），在尖锐控制多边形上会有过冲；
 * 地图管线实际走的是 `resampleUniform + smoothByArcLength`（无过冲 + 弧长窗口），
 * 本函数保留给"控制点本身已是光滑采样"的场合。
 */
export function resampleSmooth(control: Vec2M[], stepM: number): Vec2M[] {
  if (control.length < 2) return control.map(p => ({ ...p }));
  const fine: Vec2M[] = [];
  for (let i = 0; i < control.length - 1; i++) {
    const p0 = control[Math.max(0, i - 1)], p1 = control[i];
    const p2 = control[i + 1], p3 = control[Math.min(control.length - 1, i + 2)];
    const segLen = Math.hypot(p2.x - p1.x, p2.y - p1.y);
    const k = Math.max(2, Math.ceil(segLen / (stepM / 4)));
    for (let j = 0; j < k; j++) fine.push(catmullRom(p0, p1, p2, p3, j / k));
  }
  fine.push({ ...control[control.length - 1] });

  const total = pathLengthM(fine);
  const count = Math.max(1, Math.round(total / stepM));
  const out: Vec2M[] = [];
  for (let i = 0; i <= count; i++) out.push(atArcLength(fine, (i / count) * total));
  // 去掉因零长度段产生的重复点
  return out.filter((p, i) => i === 0 || Math.hypot(p.x - out[i - 1].x, p.y - out[i - 1].y) > 1e-9);
}

export interface CurvatureStats { p50: number; p95: number; max: number }

/** 分位数（升序取值，索引 clamp 到末位 → 对样本量不敏感、完全确定） */
export function quantile(sorted: number[], q: number): number {
  if (sorted.length === 0) return 0;
  return sorted[Math.min(sorted.length - 1, Math.floor(q * sorted.length))];
}

/**
 * 曲率统计（1/m）：沿弧长在每个采样点处取 ±windowM 的两点，三点外接圆
 * κ = 2·Area2 / (|ab|·|bc|·|ca|)（与 margin.ts 的 refCurvatureAt 同款式子，窗口更大）。
 *
 * 【为什么用窗口而不是相邻三点差分】3m 步长下的相邻三点差分对坐标噪声极敏感，
 * 会把平滑后的线形again测出假尖峰；±5m 窗口等价于对曲率做低通。
 * 端点 windowM 范围内的样本不参与（取不到对称三点）。
 */
export function curvatureStats(points: Vec2M[], windowM = 5): CurvatureStats {
  const ks: number[] = [];
  const total = pathLengthM(points);
  for (let s = windowM; s <= total - windowM; s += Math.max(1, windowM / 2)) {
    const a = atArcLength(points, s - windowM);
    const b = atArcLength(points, s);
    const c = atArcLength(points, s + windowM);
    const ab = Math.hypot(b.x - a.x, b.y - a.y);
    const bc = Math.hypot(c.x - b.x, c.y - b.y);
    const ca = Math.hypot(a.x - c.x, a.y - c.y);
    const denom = ab * bc * ca;
    if (denom < 1e-9) { ks.push(0); continue; }
    const area2 = Math.abs((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x));
    ks.push((2 * area2) / denom);
  }
  ks.sort((x, y) => x - y);
  return { p50: quantile(ks, 0.5), p95: quantile(ks, 0.95), max: ks.length ? ks[ks.length - 1] : 0 };
}

/**
 * 把点投影到折线上 → 沿弧长 s（米）与带符号横向偏移 l（米，按**行进方向的左侧为正**）。
 *
 * 【为什么现在就要它】① 收费站的桩号要算（地图生成期）；② M2 的 GPS 地图匹配就是它
 * ——把噪声定位读数投影到 HD 地图参考线上得到 (s, l)，正是"GPS + 地图"的接缝。
 * 语义与 `algo/plan/frenet.ts` 的 cartesianToFrenet 一致，但这里是**插值投影**
 * （取最近线段的垂足）而不是最近顶点——最近顶点会把 s 量化到采样网格上（实测偏差
 * p50 0.78m / p95 2.0m，见路线图 E5）。
 */
export function projectOntoPath(path: Vec2M[], p: Vec2M): { s: number; l: number } {
  if (path.length === 0) return { s: 0, l: 0 };
  if (path.length === 1) return { s: 0, l: Math.hypot(p.x - path[0].x, p.y - path[0].y) };
  let best = { d: Infinity, s: 0, l: 0 };
  let acc = 0;
  for (let i = 0; i < path.length - 1; i++) {
    const a = path[i], b = path[i + 1];
    const dx = b.x - a.x, dy = b.y - a.y;
    const len2 = dx * dx + dy * dy;
    const segLen = Math.sqrt(len2);
    if (segLen < 1e-12) continue;
    let t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2;
    t = Math.max(0, Math.min(1, t));
    const cx = a.x + t * dx, cy = a.y + t * dy;
    const d = Math.hypot(p.x - cx, p.y - cy);
    if (d < best.d) {
      const sgn = (-dy * (p.x - cx) + dx * (p.y - cy)) / segLen;   // 左正右负
      best = { d, s: acc + t * segLen, l: sgn >= 0 ? d : -d };
    }
    acc += segLen;
  }
  return { s: best.s, l: best.l };
}

/** 按弧长切出 [s0, s1] 段（保留两端截断点；s 会被 clamp 到 [0, 全长]） */
export function sliceByArcLength(path: Vec2M[], s0: number, s1: number): Vec2M[] {
  const total = pathLengthM(path);
  const a = Math.max(0, Math.min(s0, total));
  const b = Math.max(a, Math.min(s1, total));
  const out: Vec2M[] = [{ ...atArcLength(path, a) }];
  let acc = 0;
  for (let i = 1; i < path.length; i++) {
    const seg = Math.hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
    const s = acc + seg;
    if (s > a && s < b) out.push({ ...path[i] });
    if (s >= b) break;
    acc = s;
  }
  out.push({ ...atArcLength(path, b) });
  return out;
}

/** 该曲率下按横向加速度上限可行的速度（m/s）：v ≤ √(A_LAT_MAX/κ) */
export function speedCapFromCurvature(kappa: number): number {
  return kappa > 1e-9 ? Math.sqrt(A_LAT_MAX_MPS2 / kappa) : Infinity;
}

/** 某速度过该弯所需的横向加速度（m/s²）：a = v²·κ */
export function lateralAccel(kappa: number, vMps: number): number {
  return kappa * vMps * vMps;
}
