// ==================== 环境：交通流（方案 B：IDM 跟驰） ====================
//
// 【演进】A（定速巡航）先把管线打通；B 把"定速"换成 **IDM**（Intelligent Driver Model）——
// 每辆车对**同车道的前车**加减速，而**自车也参与前车搜索**（自车切入时后车会刹车）。
// 车辆对外依然只是一个 `Obstacle` 矩形 → 感知/规划/检测器/渲染一行没改。
//
// 【IDM 是什么】经典的微观跟驰模型，加速度由三项叠加：
//     dv/dt = a·[ 1 − (v/v₀)^δ − (s*/gap)² ]
//     s* = s₀ + max(0, v·T + v·Δv / (2·√(a·b)))          （期望间距）
//   · v₀ = 期望速度（自由流时趋近它）；a = 最大加速度；b = 舒适减速度
//   · gap = 净车距（自己车头到前车尾）；Δv = v − v_前（正数=在接近）
//   · s₀ = 静止时的最小间距；T = 期望车头时距（秒）
//   直觉：`1 − (v/v₀)^δ` 让他想开到 v₀；`(s*/gap)²` 让他在间距不足时刹车，
//   且**间距越小、接近越快，刹车越猛**（这正是"不会追尾"的来源）。
//
// 【确定性】同步更新：先用**旧状态**算完所有车的加速度，再统一应用（与遍历顺序无关）。
//   停车判定、回收、补车全部只看 state → 同种子逐位一致（traject.test.ts 钉住）。
//
// 【越界处理】闭环参考线（径向/环线）到末端回绕；开环参考线（OSM 链/路线）到末端**回收**，
// 起点留出安全间距才**重生**。

import { Obstacle } from '../shared/types/interfaces/geometry.js';
import { FrenetReference } from '../shared/types/interfaces/planning.js';
import { Vehicle, TrafficConfig } from '../shared/types/interfaces/traffic.js';
import { frenetToCartesian, buildFrenetReference, cartesianToFrenet } from '../algo/plan/frenet.js';
import { Rng, createRng, deriveSeed } from '../shared/rng.js';
import { Point2D } from '../shared/types/interfaces/geometry.js';
import { laneCentersOf } from './track.js';
import { px, m, mps, mps2, Px, PxPerSec } from '../shared/dimensions.js';

/** IDM 参数（全部用 m()/mps()/mps2() 书写：改比例尺不用动这里） */
const IDM_A = mps2(1.5) as number;          // 最大加速度
const IDM_B = mps2(2.0) as number;          // 舒适减速度
const IDM_T = 1.2;                          // 期望车头时距（秒）
const IDM_S0 = m(2.0) as number;            // 静止最小间距
const IDM_DELTA = 4;
/** 物理减速度上限：IDM 在"间距 → 0"时会给出巨大的负值，必须夹住（否则一帧内速度反号） */
const IDM_MAX_DECEL = mps2(-8) as number;
/** 判定"在自车前面"的横向容差：自车与本车横向差小于它就算同一车道 */
const EGO_LANE_TOL = m(1.8) as number;

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
  /** 固定步长推进（引擎在**感知之前**调用，保证雷达看到的是本帧位置）。
   *  `ego` 传自车位置/速度：**自车参与 NPC 的前车搜索** → 自车切入时后车会刹车（方案 B 的核心）。 */
  step(dt: number, ego?: { x: number; y: number; speed: number; length?: number }): void;
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
  /** 注入初始车辆（**测试用**：想验"跟驰/让行"这类行为，必须能摆一个确定的车阵）。
   *  给了它就跳过按密度生成；回收/补车逻辑照旧（开环时仍会在起点补车）。 */
  initialVehicles?: Vehicle[];
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
  // 【量纲·踩过的坑】`o.speedLimit` 是**仿真单位（px/s）**——`TrackGeometry.speedLimitMps` 这个名字
  // 有误导性，但它存的就是 px/s（规划器也一直这么用：G4 上是 133.33 px/s = 33.3m/s = 120km/h ✓）。
  // 曾经这里写成 `mps((speedLimit/4)·t)`：对一个已经 px/s 的输入又乘了一次 4 ✗ ⇒ 车流实际以
  // **60~93 m/s（216~335km/h）**飞驰，并从后面撞自车（用户实测反馈 ✓）。现在直接用倍率缩放。
  const laneSpeeds = laneCenters.map((lc) => {
    const outer = Math.abs(lc) / maxAbsL;                    // 1 = 最外侧、0 = 中心线
    const t = fHi - (fHi - fLo) * outer;
    return ((o.speedLimit as number) * t) as PxPerSec;
  });

  const vehicles: Vehicle[] = (o.initialVehicles ?? []).map(v => {
    // 越界车道号会让 laneSpeeds[lane] 变成 undefined → NaN 静默传播（实测踩过）→ 注入时先夹住
    const lane = Math.max(0, Math.min(nLanes - 1, v.lane));
    return { ...v, lane, l: laneCenters[lane] };
  });
  if (!o.initialVehicles && L > 0 && perLane > 0 && (o.speedLimit as number) > 0) {
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

  /**
   * 某条车道的起点是否够空（可以补一辆新车）。
   *
   * 【为什么按车道判、且必须有总量上限】实测：全局每步只补一辆时，入流上限 ≈ 1 辆/(空档/速度)，
   * 跟不上出流（密度×速度×车道数）→ 车流跑着跑着**密度衰减到 1/3**。按车道独立补车后入流≈出流，
   * 密度才守得住；总量上限防止"末端拥堵、车出不去"时无限膨胀。
   */
  function laneStartClear(lane: number, newSpeed: number): boolean {
    const need = RESPAWN_GAP + IDM_S0 + newSpeed * IDM_T;
    for (const v of vehicles) {
      if (v.lane !== lane) continue;
      if (v.s <= need) return false;                    // 本车道起点附近还有车
    }
    return true;
  }

  /** 自车在**本参考线**上的投影（同一条路才谈得上"前车"）；投影用规划器同一套 cartesianToFrenet */
  function projectEgo(ego: { x: number; y: number; speed: number }): { s: number; l: number; speed: number } | null {
    const f = cartesianToFrenet(px(ego.x), px(ego.y), o.ref);
    if (!Number.isFinite(f.s)) return null;
    return { s: f.s as number, l: f.l as number, speed: ego.speed };
  }

  /** 找前车（同车道、s 在自己前面最近的一辆；闭环时"绕回来"的车也算前面） */
  function findLeader(v: Vehicle): { gap: number; speed: number; leadLen: number } | null {
    let best: { gap: number; speed: number; leadLen: number } | null = null;
    for (const other of vehicles) {
      if (other === v || other.lane !== v.lane) continue;
      let ds = other.s - v.s;
      if (o.closed) { if (ds <= 0) ds += L; } else if (ds <= 0) continue;
      if (!best || ds < best.gap) best = { gap: ds, speed: other.speed, leadLen: other.length };
    }
    return best;
  }

  /** IDM 加速度（px/s²）；`gap` 是**净车距** = 中心距 − 两车各半长（前车的车长也要算！） */
  function idmAccel(v: Vehicle, v0: number, lead: { gap: number; speed: number; leadLen: number } | null): number {
    const free = 1 - Math.pow(Math.max(0, v.speed) / Math.max(1e-6, v0), IDM_DELTA);
    if (!lead) return IDM_A * free;
    // 中心距 → 车头到前车尾的净距：**两车车长都要扣**（只扣自己的会停得比 s₀ 更近，实测差 ~2.5m）
    const gap = Math.max(0.1, lead.gap - (VEH_LEN + lead.leadLen) / 2);
    const dv = v.speed - lead.speed;                       // >0 = 在接近
    const sStar = IDM_S0 + Math.max(0, v.speed * IDM_T + v.speed * dv / (2 * Math.sqrt(IDM_A * IDM_B)));
    return IDM_A * (free - (sStar / gap) * (sStar / gap));
  }

  return {
    step(dt: number, ego?: { x: number; y: number; speed: number; length?: number }) {
      const egoLen = (ego?.length ?? (m(4.6) as number)) as number;   // 车长（px）；引擎传 CarState.length
      const egoState = ego ? projectEgo(ego) : null;
      // ① 先用**旧状态**把所有人的加速度算完（同步更新 → 与遍历顺序无关，确定性稳）
      const accels = o.config?.model === 'constant' ? null : vehicles.map((v) => {
        const v0 = (laneSpeeds[v.lane] ?? laneSpeeds[0]) as number;   // 兜底：绝不让 undefined 变成 NaN
        let lead = findLeader(v);
        // 自车也是"前车"之一：同车道且在 v 前面、比现有前车更近 → 换成自车
        if (egoState && Math.abs(egoState.l - v.l) <= EGO_LANE_TOL) {
          let ds = egoState.s - v.s;
          if (o.closed) { if (ds <= 0) ds += L; } else if (ds <= 0) ds = -1;
          if (ds > 0 && (!lead || ds < lead.gap)) lead = { gap: ds, speed: egoState.speed, leadLen: egoLen };
        }
        return idmAccel(v, v0, lead);
      });
      // ② 再统一应用（速度夹到 [0, a] 的物理区间；减速度夹到 IDM_MAX_DECEL）
      for (let i = 0; i < vehicles.length; i++) {
        const v = vehicles[i];
        if (accels) {
          const acc = Math.max(IDM_MAX_DECEL, Math.min(IDM_A, accels[i]));
          v.speed = Math.max(0, v.speed + acc * dt);         // 不倒退
        }
        // model = 'constant'（方案 A 基线）：速度不变，只推进位置
        v.s += v.speed * dt;
      }
      if (o.closed) {
        for (const v of vehicles) { if (v.s >= L) v.s -= L; }
      } else {
        for (let i = vehicles.length - 1; i >= 0; i--) {
          if (vehicles[i].s > L + DESPAWN_MARGIN) {
            vehicles.splice(i, 1);                       // 回收（出了参考线范围）
          }
        }
        // 按车道独立补车（入流≈出流才守得住密度），但不超过目标总量
        const targetTotal = perLane * nLanes;
        if (vehicles.length < targetTotal) {
          for (let lane = 0; lane < nLanes && vehicles.length < targetTotal; lane++) {
            const v0 = laneSpeeds[lane] as number;
            if (!laneStartClear(lane, v0)) continue;
            vehicles.push({
              s: 0, lane, l: laneCenters[lane],
              speed: v0,
              length: VEH_LEN as number, width: VEH_WID as number,
            });
          }
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
    // 已是 px/s（见 createTraffic 里的量纲说明）；缺省 25m/s
  speedLimit: (track.speedLimitMps ?? (mps(25) as number)) as PxPerSec,
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
