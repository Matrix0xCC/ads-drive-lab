// ==================== 环境：交通流（方案 A：定速巡航车） ====================
//
// 【为什么先做这个】它把整条管线打通而**不引入新的算法风险**：
//   车辆 = 会动的 `Obstacle` 矩形 → 感知/规划/检测器/渲染全部复用现有通路。
// 之后再升级到 B（IDM 跟驰）只需替换 `step()` 里的速度更新，外围不动。
//
// 【车道纪律：同车道同速 → 永不追尾】
// 每条车道一个固定速度，车距由"等间距 + 小抖动"给定（抖动 ≤ 间距的 1/4，保证不重排）。
// 于是**同车道内车辆间距恒定**，不需要任何 NPC-NPC 交互 —— 这是方案 A 的取巧之处，
// 也是它便宜的代价：NPC 之间"看不见"彼此（方案 B 才引入跟驰）。
//
// 【越界处理】闭环参考线（径向/环线）到末端回绕；开环参考线（OSM 链/路线）到末端**回收**，
// 并在起点空出足够间距时**重生**（全部由 state 决定 → 确定性不受影响）。

import { Obstacle } from '../shared/types/interfaces/geometry.js';
import { FrenetReference } from '../shared/types/interfaces/planning.js';
import { Vehicle, TrafficConfig } from '../shared/types/interfaces/traffic.js';
import { frenetToCartesian, buildFrenetReference } from '../algo/plan/frenet.js';
import { Rng, createRng, deriveSeed } from '../shared/rng.js';
import { Point2D } from '../shared/types/interfaces/geometry.js';
import { laneCentersOf } from './track.js';
import { px, m, mps, Px, PxPerSec } from '../shared/dimensions.js';

/** 车长/车宽（米 → px；略大于自车，便于观察） */
const VEH_LEN = m(4.6);
const VEH_WID = m(2.0);
/** 开环参考线上，车辆越过末端多远后回收（px） */
const DESPAWN_MARGIN = m(30);
/** 自车出生点前方的留空区（px）：交通车不落在这里（避免 t=0 就不公平地冲突） */
const EGO_CLEAR = m(120);
/** 重生到起点前，起点附近需要的空档（px） */
const RESPAWN_GAP = m(60);

export interface TrafficSystem {
  /** 固定步长推进（引擎在**感知之前**调用，保证雷达看到的是本帧位置） */
  step(dt: number): void;
  /** 当前所有车辆当帧的障碍矩形（感知/规划/检测器用；与静态障碍同一个概念） */
  obstacles(): Obstacle[];
  /** 车辆内部状态（渲染/归档/测试用） */
  vehicles(): Vehicle[];
}

export interface TrafficOptions {
  ref: FrenetReference;
  /** 参考线总长（px） */
  length: Px;
  closed: boolean;
  /** 可用的车道中心横向偏移（px）；空/单元素表示只跑中心线 */
  laneCenters: number[];
  /** 地图限速（px/s），作为期望速度的基准 */
  speedLimit: PxPerSec;
  config?: TrafficConfig;
  rng: Rng;
}

export function createTraffic(o: TrafficOptions): TrafficSystem {
  const cfg = o.config ?? {};
  const density = Math.max(0, cfg.densityPerKm ?? 8);
  const [fLo, fHi] = cfg.speedFactorRange ?? [0.72, 1.0];
  const laneCenters = (o.laneCenters.length > 0 ? o.laneCenters : [px(0)])
    .slice(0, Math.max(1, Math.min(4, cfg.lanes ?? (o.laneCenters.length || 1))));
  const nLanes = laneCenters.length;
  const L = o.length;

  const perLane = Math.max(0, Math.round((density * (L / 4 / 1000))));
  // 每车道一个固定速度：**按 |l|（离中心线的距离）排序**，越靠外侧越慢（"慢车靠外"的直觉）。
  // 不用车道下标排序：`laneCentersOf` 的左右符号约定不必让这里依赖（|l| 是稳的）。
  const maxAbsL = Math.max(...laneCenters.map(v => Math.abs(v)), 1e-9);
  const laneSpeeds = laneCenters.map((lc) => {
    const outer = Math.abs(lc) / maxAbsL;                    // 1 = 最外侧、0 = 中心线
    const t = fHi - (fHi - fLo) * outer;
    return mps(((o.speedLimit as number) / 4) * t) as PxPerSec;
  });

  const vehicles: Vehicle[] = [];
  if (L > 0 && perLane > 0 && (o.speedLimit as number) > 0) {
    const spacing = L / perLane;
    for (let lane = 0; lane < nLanes; lane++) {
      for (let k = 0; k < perLane; k++) {
        // 抖动 ≤ 间距/4：既不是整齐的"列队"，也绝不会重排（重排就可能追尾）
        const jitter = (o.rng() - 0.5) * spacing * 0.5;
        let s = (k + 0.5) * spacing + jitter;
        // 【自车出生区留空】别让交通车贴着自车出生点（那是"不公平的 t=0 冲突"，不是规划器的锅）。
        // 落在 s ∈ (0, EGO_CLEAR] 的车整体前移一个 EGO_CLEAR（确定性：只看初始布局）。
        if (s > 0 && s <= EGO_CLEAR) s += EGO_CLEAR;
        vehicles.push({
          s,
          lane, l: laneCenters[lane],
          speed: laneSpeeds[lane] as number,
          length: VEH_LEN as number, width: VEH_WID as number,
        });
      }
    }
  }

  /** 起点附近是否已经够空（开环重生用） */
  function startClear(): boolean {
    return vehicles.every(v => v.s > RESPAWN_GAP);
  }

  return {
    step(dt: number) {
      for (const v of vehicles) v.s += v.speed * dt;
      if (o.closed) {
        for (const v of vehicles) { if (v.s >= L) v.s -= L; }
      } else {
        for (let i = vehicles.length - 1; i >= 0; i--) {
          if (vehicles[i].s > L + DESPAWN_MARGIN) {
            vehicles.splice(i, 1);                       // 回收（出了参考线范围）
          }
        }
        if (startClear()) {                              // 起点空了就补一辆（密度靠这个维持）
          const lane = Math.floor(o.rng() * nLanes);
          vehicles.push({
            s: 0, lane, l: laneCenters[lane],
            speed: laneSpeeds[lane] as number,
            length: VEH_LEN as number, width: VEH_WID as number,
          });
        }
      }
    },

    obstacles(): Obstacle[] {
      const out: Obstacle[] = [];
      for (const v of vehicles) {
        const c = frenetToCartesian(v.s, px(v.l), o.ref);
        out.push({
          x: c.x, y: c.y,
          width: px(v.width), height: px(v.length),
          angle: c.yaw, zHeight: 8, isVehicle: true,
        } as Obstacle);
      }
      return out;
    },

    vehicles: () => vehicles,
  };
}

/**
 * 由地图/路线组装一个交通流系统（**装配层**，引擎调用）。
 *
 * @param seed 场景种子：**必须**用独立的 RNG 流（`deriveSeed(seed,'traffic')`）——
 *   否则"开/关交通流"会连带改变静态障碍的布局，A/B 对照就失效了（traffic.test.ts 钉住这条）。
 * @returns 没给 `traffic` 配置时返回 null（与历史行为逐位一致）
 */
export function buildTrafficFor(
  track: ChainLike,
  route: { waypoints: Point2D[]; length: number } | null,
  cfg: TrafficConfig | undefined,
  seed: number | undefined,
): TrafficSystem | null {
  if (!cfg) return null;
  const pts = route?.waypoints ?? track.centerPoints;
  if (!pts || pts.length < 2) return null;
  const closed = route ? false : (track.closed ?? true);
  const ref = buildFrenetReference(pts, closed);
  let length = 0;
  for (let i = 1; i < pts.length; i++) {
    length += Math.hypot(
      (pts[i].x as number) - (pts[i - 1].x as number),
      (pts[i].y as number) - (pts[i - 1].y as number),
    );
  }
  if (route) length = route.length as number;
  const { centers } = laneCentersOf((track.halfWidth ?? m(7.5)) as number, (track.laneWidth ?? m(3.75)) as number);
  const rng: Rng = seed === undefined ? Math.random : createRng(deriveSeed(seed, 'traffic'));
  return createTraffic({
    ref, length: px(length), closed,
    laneCenters: centers.map(c => px(c) as number),
    speedLimit: mps(track.speedLimitMps ?? 25) as PxPerSec,
    config: cfg, rng,
  });
}

/** 组装交通流所需的最小地图信息（避免 env/traffic 依赖整个 TrackGeometry） */
export interface ChainLike {
  centerPoints: Point2D[];
  closed?: boolean;
  halfWidth?: number;
  laneWidth?: number;
  speedLimitMps?: number;
}
