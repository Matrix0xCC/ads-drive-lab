// ==================== 环境：赛道生成 ====================
//
// 【职责】生成闭合环形赛道的几何数据（中心线 + 内外边界）
//
// 【算法】
// 1. 随机生成 8 个控制点（环状分布）
// 2. Catmull-Rom 样条插值 → 平滑中心线（200 个采样点）
// 3. 计算每个采样点的法向量，偏移 ±TRACK_HALF_WIDTH → 内外边界
//
// 输入：画布宽高
// 输出：TrackGeometry { centerPoints, innerBoundary, outerBoundary }

import { TrackGeometry, Point2D } from '../shared/types.js';
import { Px, PxPerSec, m, mps, px } from '../shared/dimensions.js';
import { Rng } from '../shared/rng.js';
import { MapDefinition } from './mapDefinitions.js';
import { attrsAtS, type RoadAttrs, type AttrSpan } from '../shared/types.js';
import { SIM_UNITS_PER_METER } from '../shared/dimensions.js';

/** 赛道半宽 7.5m ≈ 4 车道（15m 宽）；**径向历史地图的缺省值**——OSM 地图按车道数自带半宽 */
export const TRACK_HALF_WIDTH: Px = m(7.5);

interface Vec2 { x: number; y: number; }

/**
 * Catmull-Rom 样条插值
 *
 * 给定 4 个控制点 p0-p3 和参数 t ∈ [0,1]，
 * 返回 p1 到 p2 之间 t 位置的平滑插值点。
 * Catmull-Rom 保证 C¹ 连续性（一阶导数连续），适合赛道生成。
 */
function catmullRom(p0: Vec2, p1: Vec2, p2: Vec2, p3: Vec2, t: number): Vec2 {
  const t2 = t * t, t3 = t2 * t;
  return {
    x: 0.5 * ((2 * p1.x) + (-p0.x + p2.x) * t + (2 * p0.x - 5 * p1.x + 4 * p2.x - p3.x) * t2 + (-p0.x + 3 * p1.x - 3 * p2.x + p3.x) * t3),
    y: 0.5 * ((2 * p1.y) + (-p0.y + p2.y) * t + (2 * p0.y - 5 * p1.y + 4 * p2.y - p3.y) * t2 + (-p0.y + 3 * p1.y - 3 * p2.y + p3.y) * t3),
  };
}

/**
 * 生成随机环形赛道
 *
 * @param width  画布宽度
 * @param height 画布高度
 * @returns TrackGeometry
 */
/** 逐段属性：mapgen 的字段 → 运行时（米制字段原样保留，渲染/限速用米） */
export function toRoadAttrs(a: {
  highway: string; laneCount: number; laneWidthM: number; halfWidthM: number; maxspeedKph: number;
}): RoadAttrs {
  return {
    highway: a.highway, laneCount: a.laneCount, laneWidthM: a.laneWidthM,
    halfWidthM: a.halfWidthM, maxspeedKph: a.maxspeedKph,
  };
}

/** 链式地图：把「弧长米」的属性区间转成「弧长 px」的运行时间间 */
export function toAttrSpans(list: {
  s0M: number; s1M: number; highway: string; laneCount: number; laneWidthM: number;
  halfWidthM: number; maxspeedKph: number;
}[]): AttrSpan[] {
  return list.map(sp => ({
    ...toRoadAttrs(sp), s0: px(m(sp.s0M) as number), s1: px(m(sp.s1M) as number),
  }));
}

/** 折线长度（px） */
export function polylineLenPx(pts: { x: number; y: number }[]): number {
  let acc = 0;
  for (let i = 1; i < pts.length; i++) acc += Math.hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
  return acc;
}

export function generateTrack(width: number, height: number, controlRadii?: number[], rng: Rng = Math.random): TrackGeometry {
  const cx = width / 2, cy = height / 2;

  // 1. 生成 8 个环状控制点（预设半径或随机）
  const numControls = 8;
  const controls: Vec2[] = [];
  for (let i = 0; i < numControls; i++) {
    const angle = (i / numControls) * Math.PI * 2;
    const radius = controlRadii ? controlRadii[i] : (m(60) as number + rng() * m(20) as number);
    controls.push({ x: cx + Math.cos(angle) * radius, y: cy + Math.sin(angle) * radius });
  }

  // 2. Catmull-Rom 插值：8 段 × 每段 25 点 = 200 点
  const samplesPerSegment = 25;
  const centerRaw: Vec2[] = [];
  for (let i = 0; i < numControls; i++) {
    const p0 = controls[(i - 1 + numControls) % numControls];
    const p1 = controls[i];
    const p2 = controls[(i + 1) % numControls];
    const p3 = controls[(i + 2) % numControls];
    for (let j = 0; j < samplesPerSegment; j++) {
      centerRaw.push(catmullRom(p0, p1, p2, p3, j / samplesPerSegment));
    }
  }
  centerRaw.push(centerRaw[0]);  // 闭合：首尾相连

  const centerPoints: Point2D[] = centerRaw.map(p => ({ x: px(p.x), y: px(p.y) }));

  // 3. 生成内外边界：每个中心点偏移 ±TRACK_HALF_WIDTH 沿法向量方向
  const halfW = TRACK_HALF_WIDTH as number;
  const innerBoundary: Point2D[] = [];
  const outerBoundary: Point2D[] = [];
  for (let i = 0; i < centerRaw.length - 1; i++) {
    const p = centerRaw[i], next = centerRaw[i + 1];
    const dx = next.x - p.x, dy = next.y - p.y;
    const len = Math.hypot(dx, dy) || 1;
    // 法向量：切线逆时针旋转 90°，然后确保指向赛道中心
    let nx = -dy / len, ny = dx / len;
    const toCenterX = cx - p.x, toCenterY = cy - p.y;
    if (nx * toCenterX + ny * toCenterY < 0) { nx = -nx; ny = -ny; }
    innerBoundary.push({ x: px(p.x + nx * halfW), y: px(p.y + ny * halfW) });
    outerBoundary.push({ x: px(p.x - nx * halfW), y: px(p.y - ny * halfW) });
  }
  innerBoundary.push(innerBoundary[0]);  // 闭合
  outerBoundary.push(outerBoundary[0]);

  return { centerPoints, innerBoundary, outerBoundary };
}

// ═══════════════════════════════════════════════════
// 点列地图（OSM 开环）
// ═══════════════════════════════════════════════════

/**
 * 开环路面多边形：左缘（正序）→ 末端帽 → 右缘（逆序）→ 起点帽，闭合成一个环。
 *
 * 【为什么这样构造】`car.ts` 的越界判定是 `pointInPolygon(outer) && !pointInPolygon(inner)`：
 * 开环道路没有"内场"，把 `innerBoundary` 置空（空多边形恒为 false）后，判定退化为
 * "在外多边形内"，而这个多边形的两侧是路缘、两端是**端帽**——端帽自然成为"不许冲出地图"的端墙。
 * 法向取"切线逆时针 90°"（与 `frenet.ts` 的 normals 同约定），不再用历史径向图的
 * "指向环心"启发式（开环没有环心）。
 */
/**
 * **变宽**路面多边形：每个中心线点带自己的半宽（逐段道路属性 → 2 车道段比 3 车道段窄）。
 *
 * 与定宽版的关系：`ribbonPolygon(center, w)` 等价于 `ribbonPolygonVar(center, 全 w).poly`
 * （有单测钉住这条等价，保证定宽地图逐位不变）。
 * 返回 `halfW` 数组与 `poly` **逐点对齐**（含端帽点）——`checkBounds` 需要"最近边界点处的
 * 局部半宽"来决定该点的复位缓冲（窄路段缓冲要按比例缩小，见 car.ts:boundsBufferFor）。
 */
export function ribbonPolygonVar(center: Point2D[], halfW: number[], endExtend = m(25)): {
  poly: Point2D[]; halfW: number[];
} {
  const n = center.length;
  const left: Point2D[] = [], right: Point2D[] = [];
  const leftW: number[] = [], rightW: number[] = [];
  const ext = endExtend as number;
  for (let i = 0; i < n; i++) {
    const a = center[Math.max(0, i - 1)], b = center[Math.min(n - 1, i + 1)];
    const dx = (b.x as number) - (a.x as number), dy = (b.y as number) - (a.y as number);
    const len = Math.hypot(dx, dy) || 1;
    const nx = -dy / len, ny = dx / len;                 // 左法向
    const w = halfW.length > 0 ? halfW[Math.min(i, halfW.length - 1)] : 0;
    let cx = center[i].x as number, cy = center[i].y as number;
    if (i === 0) { cx -= (dx / len) * ext; cy -= (dy / len) * ext; }
    else if (i === n - 1) { cx += (dx / len) * ext; cy += (dy / len) * ext; }
    left.push({ x: px(cx + nx * w), y: px(cy + ny * w) });
    right.push({ x: px(cx - nx * w), y: px(cy - ny * w) });
    leftW.push(w); rightW.push(w);
  }
  return {
    poly: [...left, ...[...right].reverse(), left[0]],
    // 端帽点在左缘/右缘之外各多一个：poly 的收尾是 left[0]
    halfW: [...leftW, ...[...rightW].reverse(), leftW[0]],
  };
}

/** 车道收窄/变宽的过渡段长度：真实道路上 3 车道→1 车道是渐变（车道收窄），不是一道台阶。 */
export const WIDTH_TAPER_M = m(30);

/**
 * **带过渡的逐段半宽**：把属性区间（台阶函数）变成"提前 `taper` 米开始线性过渡"的函数。
 *
 * 【为什么必须过渡，且必须三处同源】宽度台阶会在路面多边形上生成一道**横向的墙**；
 * 背景减除按命中点投影处的局部半宽判边界时，墙上的点会投影到台阶**之前**的 s（那里半宽
 * 还是宽路的），于是 |l| 远小于局部半宽 → 被判成**高置信障碍物** → 规划器看见"一堵横在
 * 路中间的墙"。实测：三环场景在匝道口（s=2546m）永久停住。
 *
 * 过渡后，界面上那条斜边上的每一点都满足 |l| ≈ 该点的局部半宽 → 正确归类为边界。
 * **几何（ribbon）、分类阈值、规划域都调用本函数**，三者口径必须一致。
 */
export function taperedHalfWidthAt(
  spans: AttrSpan[] | undefined, fallbackPx: number, taperPx = WIDTH_TAPER_M as number,
): (sPx: number) => number {
  if (!spans || spans.length === 0) return () => fallbackPx;
  const stepAt = (s: number): number => {
    const a = attrsAtS(spans, s);
    return a ? a.halfWidthM * SIM_UNITS_PER_METER : fallbackPx;
  };
  const boundaries = spans.slice(1).map(sp => sp.s0 as number);
  return (s: number): number => {
    const here = stepAt(s);
    let w = here;
    for (const b of boundaries) {
      const d = b - s;
      if (d <= 0 || d > taperPx) continue;                 // 只看**前方**过渡段内的边界
      const target = stepAt(b + 1);
      // 线性过渡：离边界越近，越接近边界后的宽度
      w = Math.min(w, here + (target - here) * (1 - d / taperPx));
    }
    return Math.max(0, w);
  };
}

/** 逐点半宽：按累计弧长查属性区间（含过渡段；无属性 → 全部用 fallback） */
export function halfWidthsAlongPoints(
  points: Point2D[], spans: AttrSpan[] | undefined, fallbackPx: number,
  taperPx = WIDTH_TAPER_M as number,
): number[] {
  if (!spans || spans.length === 0) return points.map(() => fallbackPx);
  const wAt = taperedHalfWidthAt(spans, fallbackPx, taperPx);
  const out: number[] = [];
  let s = 0;
  for (let i = 0; i < points.length; i++) {
    if (i > 0) {
      s += Math.hypot((points[i].x as number) - (points[i - 1].x as number),
        (points[i].y as number) - (points[i - 1].y as number));
    }
    out.push(wAt(s));
  }
  return out;
}

export function ribbonPolygon(center: Point2D[], halfW: number, endExtend = m(25)): Point2D[] {
  const n = center.length;
  const left: Point2D[] = [], right: Point2D[] = [];
  const ext = endExtend as number;
  for (let i = 0; i < n; i++) {
    const a = center[Math.max(0, i - 1)], b = center[Math.min(n - 1, i + 1)];
    const dx = (b.x as number) - (a.x as number), dy = (b.y as number) - (a.y as number);
    const len = Math.hypot(dx, dy) || 1;
    const nx = -dy / len, ny = dx / len;                 // 左法向
    // 【为什么两端要沿切线外延】端帽是"不许冲出地图"的墙，而 `checkBounds` 用
    // "到任一边界线段距离 < r" 判定越界——若端帽正好压在路线起点/终点上，车一出生
    // 就在端帽上 → 每帧复位（实测：2000m 图 7200 帧全部 off_track）。
    // 外延一段（25m）给起终点留出余量，车的可行驶区 = 中心线全程 + 两端外延。
    let cx = center[i].x as number, cy = center[i].y as number;
    if (i === 0) { cx -= (dx / len) * ext; cy -= (dy / len) * ext; }
    else if (i === n - 1) { cx += (dx / len) * ext; cy += (dy / len) * ext; }
    left.push({ x: px(cx + nx * halfW), y: px(cy + ny * halfW) });
    right.push({ x: px(cx - nx * halfW), y: px(cy - ny * halfW) });
  }
  return [...left, ...[...right].reverse(), left[0]];
}

/**
 * 车道几何：由**道路半宽与车道宽**推出车道数与各车道中心，并给出名义行驶车道。
 *
 * 【为什么从地图推】车道是地图语义（OSM 的 `lanes` tag）。旧版把"Lane 3 = +1.875m"
 * 硬编码在规划器里——换地图（2/3/4 车道）时名义车道就不对了。
 * 实际车道宽取 `2·halfW/n`（让 n 条车道恰好铺满路面），n = round(2·halfW/laneWidth)。
 * 名义车道取**最靠近路中心**的那条（奇数车道 = 正中间；偶数并列时取正侧——
 * 径向图 4 车道因此得到 +1.875m，与历史硬编码值逐位一致 → 行为零变化）。
 */
export function laneCentersOf(halfW: number, laneWidth: number): { centers: number[]; offset: number } {
  const n = Math.max(2, Math.round(2 * halfW / (laneWidth || 1)));
  const w = 2 * halfW / n;
  const centers: number[] = [];
  for (let k = 0; k < n; k++) centers.push(-halfW + (k + 0.5) * w);
  const offset = centers.reduce((a, b) =>
    (Math.abs(b) < Math.abs(a) || (Math.abs(b) === Math.abs(a) && b > a)) ? b : a, centers[0]);
  return { centers, offset };
}

/**
 * 由地图定义构建赛道几何（统一入口：径向闭环 / 点列开环）。
 *
 * @param defaultW/defaultH 径向地图未声明世界尺寸时的回退（引擎的 SIM_W/SIM_H）
 */
export function buildTrack(map: MapDefinition, defaultW: number, defaultH: number): TrackGeometry {
  if (map.kind === 'polyline') {
    if (!map.points || map.points.length < 2) throw new Error(`地图 ${map.name} 缺 points（polyline 需要 ≥2 点）`);
    if (map.closed) throw new Error(`地图 ${map.name}：闭环点列暂未支持（当前只有开环 OSM 地图）`);
    const halfW = (map.halfWidth ?? TRACK_HALF_WIDTH) as number;
    const laneW = m(map.laneWidthM ?? 3.75) as number;
    const lanes = laneCentersOf(halfW, laneW);
    const spans = map.attrSpans ? toAttrSpans(map.attrSpans) : undefined;
    // 路面与边界按**逐段宽度**：2 车道段比 3 车道段窄（渲染、复位崖、规划域三者同源）
    const rib = spans
      ? ribbonPolygonVar(map.points, halfWidthsAlongPoints(map.points, spans, halfW))
      : null;
    return {
      centerPoints: map.points,
      innerBoundary: [],                                  // 开环无内场
      outerBoundary: rib ? rib.poly : ribbonPolygon(map.points, halfW),
      ...(rib ? { outerBoundaryHalfWidth: rib.halfW } : {}),
      closed: false,
      halfWidth: px(halfW),
      laneWidth: px(laneW),
      laneCenters: lanes.centers.map(px),
      laneOffset: px(lanes.offset),
      ...(map.maxspeedKph !== undefined ? { speedLimitMps: mps(map.maxspeedKph / 3.6) } : {}),
      // 链式地图：中心线由多条 way 合并 → 逐段属性沿弧长成区间（车道数/限速/半宽逐段不同）
      attrSpans: spans,
    };
  }
  if (map.kind === 'network') {
    if (!map.chains || map.chains.length === 0 || !map.nodes || !map.edges) {
      throw new Error(`地图 ${map.name}：network 需要 chains/nodes/edges`);
    }
    const halfW = (map.halfWidth ?? TRACK_HALF_WIDTH) as number;
    const laneW = m(map.laneWidthM ?? 3.75) as number;
    const lanes = laneCentersOf(halfW, laneW);
    // 无路线时的回退参考线：最长链（有路线时规划器/感知都用**路线**折线，见 planner.ts §1）
    let longest = map.chains[0];
    for (const ch of map.chains) if (ch.length > longest.length) longest = ch;
    return {
      centerPoints: longest,
      innerBoundary: [],                       // 路网没有"内场"，边界由"到最近链的距离"判定
      outerBoundary: [],
      closed: false,
      halfWidth: px(halfW),
      laneWidth: px(laneW),
      laneCenters: lanes.centers.map(px),
      laneOffset: px(lanes.offset),
      chains: map.chains,
      netNodes: map.nodes,
      netEdges: map.edges,
      chainAttrs: map.chainAttrs?.map(a => toRoadAttrs(a)),
      // 无路线时的参考线 = 最长链 → 链内属性取该链的一段（区间 [0, 链长]）
      attrSpans: map.chainAttrs
        ? [{ ...toRoadAttrs(map.chainAttrs[map.chains.indexOf(longest)]), s0: px(0), s1: px(polylineLenPx(longest)) }]
        : undefined,
      ...(map.maxspeedKph !== undefined ? { speedLimitMps: mps(map.maxspeedKph / 3.6) } : {}),
    };
  }
  const track = generateTrack(map.worldW ?? defaultW, map.worldH ?? defaultH, map.controlRadii);
  const lanes = laneCentersOf(TRACK_HALF_WIDTH as number, m(3.75) as number);
  return {
    ...track, closed: true, halfWidth: TRACK_HALF_WIDTH,
    laneWidth: m(3.75),
    laneCenters: lanes.centers.map(px),
    laneOffset: px(lanes.offset),
  };
}
