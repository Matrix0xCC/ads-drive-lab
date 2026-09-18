// ==================== 环境：预设地图定义 ====================
//
// 每张地图由 8 个控制点的径向半径数组定义。
// 控制点均匀分布在圆周上（0°, 45°, 90°, ..., 315°），
// 每个半径决定该方向控制点到画布中心的距离。
//
// Catmull-Rom 插值将 8 个控制点平滑为 200 个中心线采样点。
// 赛道半宽由 track.ts 的 TRACK_HALF_WIDTH 控制。

import { Point2D } from '../shared/types.js';
import { m } from '../shared/dimensions.js';

/** 地图定义：8 个控制点半径 + 名称 */
export interface MapDefinition {
  name: string;
  /** 8 个径向半径（米），按角度 0°,45°,90°,...,315° 排列 */
  controlRadii: number[];
  controlPoints?: Point2D[];
}

/**
 * 预设地图字典（半径单位：米，通过 m() 转为仿真单位）
 *
 *   Angle:   0°    45°   90°  135°  180°  225°  270°  315°
 *           (右)  (右上) (上)  (左上) (左)  (左下) (下)  (右下)
 */
export const MAP_PRESETS: Record<string, MapDefinition> = {

  /** 椭圆 — 两条长直道 + 两个圆弧端，适合高速巡航 */
  oval: {
    name: 'Oval',
    controlRadii: [m(100), m(92), m(80), m(92), m(100), m(92), m(80), m(92)].map(v => v as number),
  },

  /** 发卡弯 — 左侧极紧弯道，考验低速转弯 */
  hairpin: {
    name: 'Hairpin',
    controlRadii: [m(90), m(90), m(88), m(70), m(57), m(70), m(88), m(90)].map(v => v as number),
  },

  /** 大道 — 近似圆形，均匀半径，高速友好 */
  highway: {
    name: 'Highway',
    controlRadii: [m(90), m(90), m(90), m(90), m(90), m(90), m(90), m(90)].map(v => v as number),
  },

  /** 技术赛道 — 混合弯道，多变的驾驶挑战 */
  tech_circuit: {
    name: 'Tech Circuit',
    controlRadii: [m(95), m(70), m(60), m(75), m(95), m(80), m(62), m(82)].map(v => v as number),
  },
};

export const DEFAULT_MAP = 'oval';
