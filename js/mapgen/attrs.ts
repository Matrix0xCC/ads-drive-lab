// ==================== OSM 道路属性（逐路段） ====================
//
// 【为什么单独一个纯模块】"这段路几车道、限速多少、多宽"是**数据问题**，与几何、拓扑无关，
// 但下游（地图产物 / 规划域 / 渲染 / 感知）全靠它。放成纯函数 + 显式回退表 + **来源标注**，
// 才能：① 单测直接钉住回退规则；② 让"哪些属性是真实 OSM 数据、哪些是按等级派生的"永远可查。
//
// 【真实覆盖率（2026-09-27 实测，三份北京样本）】
//   lanes   ：motorway 100%（G6 16/16、六里桥 23/23、G4 79/79）、trunk 34%（140/412）、
//             **匝道 ≈0%**（六里桥 394 条 link 里只有 1 条有 lanes）
//   maxspeed：**几乎没有**（六里桥 829 条 way 里 1 条、G6 0 条、G4 0 条）
// 所以限速基本只能是**按等级派生**——地图里会逐段记录 `maxspeedFrom: "derived:class"`，
// 不允许把派生值当成 OSM 数据（此前的文档曾把默认 120km/h 说成"OSM 真实限速"，是错的）。

/** 车行道宽缺省值（米）：OSM 的 `width` 覆盖率极低，实践中以车道数×3.75 为准 */
export const DEFAULT_LANE_WIDTH_M = 3.75;

export interface ChainAttrs {
  /** OSM highway 值（motorway / trunk / *_link …） */
  highway: string;
  laneCount: number;
  laneWidthM: number;
  /** 半宽 = 车道数 × 车道宽 / 2（米） */
  halfWidthM: number;
  maxspeedKph: number;
  /** `osm` = 来自 tags；`derived:class` = 按道路等级回退 */
  lanesFrom: AttrSource;
  maxspeedFrom: AttrSource;
}

export type AttrSource = 'osm' | 'derived:class';

/** 沿参考线的一段属性（弧长区间，米） */
export interface AttrSpan extends ChainAttrs {
  s0M: number;
  s1M: number;
}

/** 各等级主线的**派生**限速（km/h）：仅当 OSM 无 `maxspeed` 时使用 */
const CLASS_MAXSPEED_KPH: Record<string, number> = {
  motorway: 120, trunk: 100, primary: 60, secondary: 50, tertiary: 40,
  residential: 30, service: 20, unclassified: 30,
};
/** 匝道取主线的这一比例，并夹到 [40, 60]（实际匝道限速常见区间） */
const LINK_SPEED_RATIO = 0.5;
const LINK_SPEED_CLAMP: [number, number] = [40, 60];

/** 派生限速：匝道按"主线 × 0.5"并夹到 40~60；其余按等级表；未知等级给 60 */
export function derivedMaxspeedKph(highway: string): number {
  if (highway.endsWith('_link')) {
    const base = CLASS_MAXSPEED_KPH[highway.slice(0, -'_link'.length)] ?? 100;
    const v = Math.round(base * LINK_SPEED_RATIO);
    return Math.max(LINK_SPEED_CLAMP[0], Math.min(LINK_SPEED_CLAMP[1], v));
  }
  return CLASS_MAXSPEED_KPH[highway] ?? 60;
}

/** 派生车道数：匝道 1；motorway/trunk 3（本仓库既有默认）；其余 2 */
export function derivedLaneCount(highway: string): number {
  if (highway.endsWith('_link')) return 1;
  if (highway === 'motorway' || highway === 'trunk') return 3;
  return 2;
}

/** 解析 OSM `lanes`（可能是 "2"、"2;3"、"1.5"）：取第一个可用整数 */
function parseLanes(v: string | undefined): number | null {
  if (!v) return null;
  const first = v.split(';')[0].trim();
  const n = Number.parseFloat(first);
  return Number.isFinite(n) && n >= 1 && n <= 12 ? Math.round(n) : null;
}

/** 解析 OSM `maxspeed`（可能是 "120"、"120 km/h"、"none"、"signals"、"CN:urban"） */
function parseMaxspeed(v: string | undefined): number | null {
  if (!v) return null;
  const m = /^(\d+(?:\.\d+)?)/.exec(v.trim());
  if (!m) return null;                       // none / signals / walk / 国别标记 → 派生
  const kph = Number.parseFloat(m[1]);
  return Number.isFinite(kph) && kph >= 5 && kph <= 200 ? Math.round(kph) : null;
}

/** 单条 way 的 tags → 逐段属性（含来源标注） */
export function chainAttrs(tags: Record<string, string> | undefined): ChainAttrs {
  const t = tags ?? {};
  const highway = t.highway ?? 'unclassified';
  // 车道宽：优先 OSM `width`（整条路宽，非车道宽时会被高估——保留但不常用）
  const w = t.width ? Number.parseFloat(t.width) : NaN;
  const laneWidthM = Number.isFinite(w) && w > 0 && w < 20 ? w : DEFAULT_LANE_WIDTH_M;

  const lanes = parseLanes(t.lanes);
  const laneCount = lanes ?? derivedLaneCount(highway);

  const ms = parseMaxspeed(t.maxspeed);
  const maxspeedKph = ms ?? derivedMaxspeedKph(highway);

  return {
    highway, laneCount, laneWidthM,
    halfWidthM: round3(laneCount * laneWidthM / 2),
    maxspeedKph,
    lanesFrom: lanes !== null ? 'osm' : 'derived:class',
    maxspeedFrom: ms !== null ? 'osm' : 'derived:class',
  };
}

/** 逐点归属 → 属性区间（相邻同属性合并）。`wayOf[i]` = 第 i 个点所属 way 的属性 */
export function spansFromPointAttrs(sM: number[], attrsOf: ChainAttrs[]): AttrSpan[] {
  const out: AttrSpan[] = [];
  for (let i = 0; i < attrsOf.length; i++) {
    const a = attrsOf[i];
    const prev = out[out.length - 1];
    if (prev && sameAttrs(prev, a)) { prev.s1M = round3(sM[i]); continue; }
    out.push({ ...a, s0M: round3(i === 0 ? 0 : sM[i - 1]), s1M: round3(sM[i]) });
  }
  return out.filter(sp => sp.s1M > sp.s0M);
}

function sameAttrs(a: ChainAttrs, b: ChainAttrs): boolean {
  return a.highway === b.highway && a.laneCount === b.laneCount
    && a.maxspeedKph === b.maxspeedKph && a.laneWidthM === b.laneWidthM;
}

/** 属性来源统计（进地图产物，回答"多少是真实数据"） */
export function attrCoverage(list: ChainAttrs[]): {
  total: number; lanesFromOsm: number; maxspeedFromOsm: number;
  byHighway: Record<string, number>;
} {
  const byHighway: Record<string, number> = {};
  let lanesFromOsm = 0, maxspeedFromOsm = 0;
  for (const a of list) {
    byHighway[a.highway] = (byHighway[a.highway] ?? 0) + 1;
    if (a.lanesFrom === 'osm') lanesFromOsm++;
    if (a.maxspeedFrom === 'osm') maxspeedFromOsm++;
  }
  return { total: list.length, lanesFromOsm, maxspeedFromOsm, byHighway };
}

/** 参考线上某弧长处的属性（线性查表；超出范围取首/末段） */
export function attrsAt(spans: AttrSpan[], sM: number): AttrSpan | null {
  if (spans.length === 0) return null;
  if (sM <= spans[0].s0M) return spans[0];
  for (const sp of spans) if (sM < sp.s1M) return sp;
  return spans[spans.length - 1];
}

function round3(v: number): number { return Math.round(v * 1000) / 1000; }
