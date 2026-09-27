// ==================== 环境：预设地图定义 ====================
//
// 两种地图来源：
//   ① `kind: 'radial'`（历史）——8 个控制点的径向半径数组 + Catmull-Rom 插值成闭环；
//   ② `kind: 'polyline'`（OSM）——直接给点列（世界坐标 px），可开环（`closed: false`）。
//      由 `js/mapgen/build-map.ts` 生成 `js/env/maps/<name>.gen.ts`（勿手改）。
//
// 赛道的半宽与**世界尺寸**都随地图走：径向图沿用历史值（7.5m 半宽 / 1800×800），
// OSM 图按车道数算半宽、按点列包围盒算世界（2km 级 → 必须相机跟随取景）。

import { Point2D } from '../shared/types.js';
import { Px, m } from '../shared/dimensions.js';
import { BJ_G4_DUJIAKAN } from './maps/bj-g4-dujiakan.gen.js';
import { BJ_G6_JUNCTION } from './maps/bj-g6-junction.gen.js';
import { BJ_3RD_RING } from './maps/bj-3rd-ring.gen.js';

/** 地图定义 */
export interface MapDefinition {
  name: string;
  /** 几何来源：径向控制点（历史闭环） | 点列（OSM，可开环） | 路网（多链 + 节点/边图） */
  kind: 'radial' | 'polyline' | 'network';
  /** radial：8 个径向半径（px），按角度 0°,45°,…,315° 排列 */
  controlRadii?: number[];
  /** polyline：中心线点列（世界坐标 px） */
  points?: Point2D[];
  /** polyline：是否闭环 */
  closed?: boolean;
  /** 道路半宽（px）；radial 缺省 m(7.5) */
  halfWidth?: Px;
  /** 世界尺寸（px）；radial 缺省引擎默认 1800×800 */
  worldW?: number;
  worldH?: number;
  /** 车道宽（米，信息/渲染用） */
  laneWidthM?: number;
  /** OSM maxspeed（km/h，信息用） */
  maxspeedKph?: number;
  /** 强制停车点（收费口；M2 消费）：桩号 + 横向偏移（米） */
  tollgates?: { sM: number; lM: number; osmId: number }[];
  /** network：路网节点（世界坐标 px） */
  nodes?: Point2D[];
  /** network：链几何（世界坐标 px）；`edges[].chain` 指向这里 */
  chains?: Point2D[][];
  /** network：边表（节点对 → 链）——路由用；`oneway` 缺省视为双向 */
  edges?: { from: number; to: number; chain: number; lengthM?: number; oneway?: boolean }[];
  /** 逐链道路属性（与 chains 对齐）；缺省 = 全图用地图层标量 */
  chainAttrs?: { highway: string; laneCount: number; laneWidthM: number; halfWidthM: number; maxspeedKph: number; lanesFrom?: string; maxspeedFrom?: string }[];
  /** 链式地图的逐段属性区间（弧长米，沿 points）；缺省 = 全图用地图层标量 */
  attrSpans?: { s0M: number; s1M: number; highway: string; laneCount: number; laneWidthM: number; halfWidthM: number; maxspeedKph: number; lanesFrom?: string; maxspeedFrom?: string }[];
}

/**
 * 预设地图字典（径向半径单位：米，通过 m() 转为仿真单位）
 *
 *   Angle:   0°    45°   90°  135°  180°  225°  270°  315°
 *           (右)  (右上) (上)  (左上) (左)  (左下) (下)  (右下)
 */
export const MAP_PRESETS: Record<string, MapDefinition> = {

  /** 椭圆 — 两条长直道 + 两个圆弧端，适合高速巡航 */
  oval: {
    name: 'Oval',
    kind: 'radial',
    controlRadii: [m(100), m(92), m(80), m(92), m(100), m(92), m(80), m(92)].map(v => v as number),
  },

  /** 发卡弯 — 左侧极紧弯道，考验低速转弯 */
  hairpin: {
    name: 'Hairpin',
    kind: 'radial',
    controlRadii: [m(90), m(90), m(88), m(70), m(57), m(70), m(88), m(90)].map(v => v as number),
  },

  /** 大道 — 近似圆形，均匀半径，高速友好 */
  highway: {
    name: 'Highway',
    kind: 'radial',
    controlRadii: [m(90), m(90), m(90), m(90), m(90), m(90), m(90), m(90)].map(v => v as number),
  },

  /** 技术赛道 — 混合弯道，多变的驾驶挑战 */
  tech_circuit: {
    name: 'Tech Circuit',
    kind: 'radial',
    controlRadii: [m(95), m(70), m(60), m(75), m(95), m(80), m(62), m(82)].map(v => v as number),
  },

  /** 京港澳 G4 杜家坎收费站起 2.0km（真实 OSM，开环，3 车道）——由 mapgen 生成 */
  'bj-g4-dujiakan': BJ_G4_DUJIAKAN,

  /** 京藏 G6/S11 立交路网（真实 OSM，多链 + 节点/边图，8 个路口，**树状**：路线唯一） */
  'bj-g6-junction': BJ_G6_JUNCTION,

  /** 北京西三环/六里桥路网（真实 OSM，**有环**：105 节点 / 109 边 / 38 路口）——能演示"绕行选择" */
  'bj-3rd-ring': BJ_3RD_RING,
};

export const DEFAULT_MAP = 'oval';
