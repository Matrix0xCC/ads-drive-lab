// ==================== 接口契约：交通流 ====================
//
// 【定位】交通流是**环境**的一部分（不是算法）：它只负责"让世界里的车动起来"。
// 对算法侧它**不引入新概念**——每辆车每帧被表示成一个普通的 `Obstacle` 矩形，
// 于是感知（LiDAR 打到它就报高置信）、规划（FOLLOW/AVOID/DECEL）、检测器（碰撞判据）
// 全都**原样适用**（见 docs 里的"障碍物只有一个概念"）。
//
// 【确定性】车流必须与仿真同源：固定步长步进 + **独立的 seeded RNG 流**
// （`deriveSeed(seed, 'traffic')`）——独立流是硬要求：否则"开/关交通流"会改变静态障碍的布局，
// A/B 对照就不成立了（js/test/sim/traffic.test.ts 钉住这条）。

/** 交通流配置（场景 → 引擎）。缺省 = 无交通流（与历史行为逐位一致）。 */
export interface TrafficConfig {
  /** 目标密度（辆/km/车道；按参考线长度铺）。缺省 8 */
  densityPerKm?: number;
  /** 期望速度 = 地图限速 × factor；缺省 [0.72, 1.0]（越靠外侧车道越慢） */
  speedFactorRange?: [number, number];
  /** 用几条车道（缺省按逐段属性的 lanes，上限 4；1 = 只跑中心线） */
  lanes?: number;
}

/** 一辆交通车（内部状态；对外只暴露它当帧的 Obstacle 矩形） */
export interface Vehicle {
  /** 沿参考线的弧长（px） */
  s: number;
  /** 车道序号（0 = 最外侧/最慢） */
  lane: number;
  /** 横向偏移（px，车道中心） */
  l: number;
  /** 纵向速度（px/s） */
  speed: number;
  /** 车长（px，纵向） */
  length: number;
  /** 车宽（px，横向） */
  width: number;
}
