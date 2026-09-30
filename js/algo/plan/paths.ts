// ==================== 算法：路径层（SL 空间） ====================
//
// 【职责】生成横向路径候选 l(s)（视程 PATH_HORIZON），逐条做整路径碰撞检查，
// 输出最优路径；全部阻塞时按"最晚碰撞"（sBlock 最大）选停车基准路径。
// 纵向速度由 speed.ts 决定（解耦架构，见 spec 2026-09-19-decoupled-planner）。
//
// 【为什么路径可以看很远】l(s) 是纯几何、无时间耦合——视程不受 2s 轨迹时域限制。
// 旧的耦合采样里"本车道被障碍判阻塞"发生得太晚（第七轮晚横移分析），
// 解耦后路径层用 120m 视程做碰撞检查。
//
// 【横向耐心、纵向保守、双阈值通行】横向排名只看 COMMIT_WINDOW 内的阻塞
// （晚承诺）——旧版直接用 120m 视程排名选路，障碍还在几十米外就提前承诺到
// 侧向路径，随后新障碍堵死该侧 → 死口袋（hairpin-s5 批量回归实测）。
// 通行性由**走廊族**（corridor.ts，spec 2026-09-25 §3/§4）判定，四层同一几何、
// 只差膨胀量（误差感知裕度 marginFor(v) 见 margin.ts）：
//   舒适走廊（车半宽+1.5m）    —— 判"顺不顺"，只影响排名（窗口内）；
//   物理走廊（+ marginFor(v)）—— 判"能不能过"，决定停车线（全程）；
//   更严/更松走廊（±0.15m）    —— 迟滞状态机（tight.ts）的进入/退出判据。
// 全部候选连物理走廊都过不去才停车；否则选可过的路径并限速慢行。
// （批量 A/B：只用舒适阈值判停 → 6/10 场景中途永久停车，其中 oval-s4/
//   hairpin-s5 旧规划器干净通过——存在可行路线，被过保守的判停拒了。）

import { FrenetReference, LidarScan } from '../../shared/types.js';
import { Radians, Px, DimLess, m, mps, mps2, px, perpx, dimless } from '../../shared/dimensions.js';
import { solveQuintic } from './polynomial.js';
import { frenetToCartesian } from './frenet.js';
import { buildCorridor, deriveTargets, firstExit, coverageFromScan } from './corridor.js';
import type { Corridor } from './corridor.js';
import { marginFor, MARGIN_MOVING_EXTRA, MARGIN_MOVING_ENABLED } from './margin.js';
import { MAX_SENSOR_DIST } from '../perception.js';
import { TRACK_HALF_WIDTH } from '../../env/track.js';
import { boundsBufferFor } from '../../env/car.js';
import { BOUNDS_BUFFER } from '../../env/car.js';

export const PATH_HORIZON: Px = m(120);
/** 横移发展距离：在前 40m 内完成横移到目标，其后保持偏移。
 *  若横移按整段 120m 发展，到障碍处偏移还太小（测试实证），不能完成避让。 */
export const PATH_DEVELOP: Px = m(40);
/** 自适应发展段下限：再近也不得短于此（低速横移曲率上限） */
const PATH_DEVELOP_MIN: Px = m(12);
/** 自适应提前量：横移应在首个阻塞点之前完成（留几何余量） */
const DEVELOP_MARGIN: Px = m(6);
/** 横向加速度舒适上限（与速度层 A_COMFORT 一致）：决定给定速度下最短发展段；
 *  也是**道路曲率限速**的口径（planner.ts 的 curvatureSpeedCap） */
export const A_LAT_MAX = mps2(3.5);
/** 归一化五次多项式 10s³−15s⁴+6s⁵ 的峰值 |l''|（发展段内峰值横向加速度系数） */
const QUINTIC_PEAK_ACCEL = 5.7735;
/** 横向机动可行速度上限（收敛迭代用）：峰值横向加速度 QUINTIC_PEAK_ACCEL·|Δl|·v²/S²
 *  ≤ A_LAT_MAX → v ≤ √(A_LAT_MAX·S²/(QUINTIC_PEAK_ACCEL·|Δl|))——与 developMin 同式反解，
 *  把"路径几何需要多大横移、在多长发展段内完成"反馈成速度上限（仿真单位/s）。 */
export function lateralSpeedCap(dlU: number, developU: number): number {
  if (!(dlU > 1e-6)) return Infinity;
  return Math.sqrt((A_LAT_MAX as number) * developU * developU / (QUINTIC_PEAK_ACCEL * dlU));
}

/** 横向选路提交窗口：只有窗口内的阻塞参与候选排名（"横向耐心"——障碍还远时
 *  不提前选边；进入窗口再决策仍有 60m 完成横移）。停车线不看此窗口。 */
export const COMMIT_WINDOW: Px = m(60);
export const PATH_STEPS = 40;

const W_PATH_CENTER = 0.3;    // 中心偏离²（沿用旧 W_CENTER 量级）
const W_PATH_CURV = 1.0;      // 路径曲率（航向变化累计）
const W_SWITCH_PATH = 100;    // 路径跨档切换惩罚（沿用旧 W_SWITCH）
/** 横向换边迟滞带：squeeze/hard 档主键是"阻塞点弧长"（阈值穿越量、按网格量化），
 *  左右两侧差异常在一格（3m）之内 → 逐帧翻边（oval-s9/hairpin-s9 实证：目标在
 *  ±3.8m 间以 ~0.3s 周期来回，车左右摆着蹭上障碍）。换边候选的主键减此带宽 =
 *  "换边需要明显更晚（>12m）才阻塞"，把承诺锁到前后帧。 */
const LATERAL_SWITCH_BAND = m(12);
const W_EDGE = 0.5;           // 贴边惩罚权重（与 W_PATH_CENTER 同量级起手）
const EDGE_BAND = m(0.5);     // 贴边惩罚生效带：距走廊边界 0.5m 以内
/** 规划域相对复位崖的额外内缩（跟踪过冲预留）。0 = 域边即复位崖（当前值） */
export const DOMAIN_OVERSHOOT_RESERVE: Px = m(0);
/** 迟滞粘滞带（tight.ts 施密特触发）：物理阈值上下各 0.15m */
const HYST_BAND: Px = m(0.15);
/** 未观测区（遮挡阴影/视锥外）的口径——A/B 实验开关（离线量指标用；可视化走廊始终不裁，
 *  灰蓝分色要显示原始自由区）：
 *   'free'    —— 旧行为：未观测 = 自由（无处理）
 *   'blocked' —— 未观测 = 不可通行（裁判定走廊）。实测退化：规划视程(120m)总是超出可见距离
 *                (~70m) → 所有候选都在可见边界外"硬阻塞" → 封死 → 永久停车
 *   'speed'   —— 见-即-停限速：不裁几何，只按可见距离限制速度 v ≤ √(2·A_BRAKE·d_vis) */
export type UnknownHandling = 'free' | 'blocked' | 'speed';
const UNKNOWN_HANDLING: UnknownHandling = 'speed';
/** 紧间隙慢行限速（双阈值：物理可过但低于舒适间隙时）。速度层在"舒适阻塞但物理可过"
 *  时不命令超过它；物理走廊的裕度评估也按它取速度下限（见 marginU）。 */
export const TIGHT_SPEED = mps(8);

export interface PathCandidate {
  targetL: Px;
  points: { s: number; l: Px; x: Px; y: Px; yaw: Radians }[];
  /** 窗口（COMMIT_WINDOW）内是否低于舒适间隙——决定横向排名 */
  blocked: boolean;
  /** 窗口内舒适层首次命中的绝对弧长（blocked 时有效，否则 Infinity） */
  sBlock: number;
  /** 全程（PATH_HORIZON）内是否低于物理硬阈值——决定停车线 */
  blockedHard: boolean;
  /** 全程硬阈值下首次命中的绝对弧长（blockedHard 时有效，否则 Infinity） */
  sBlockHard: number;
  cost: number;
  /** 实际使用的发展段长度（自适应缩短后的值，仿真单位；供规划器反解横向机动可行速度） */
  develop: number;
  /** 合成阶段按相对弧长求横向偏移（供 l(s(t)) 采样）；内部 clamp 到 [0, PATH_HORIZON] */
  evalL(s: number): Px;
}

export interface PathLayerResult {
  best: PathCandidate;
  ranked: PathCandidate[];   // 排序分级：舒适畅通（cost 升序）→ 舒适阻塞但物理可过（sBlock 降序）→ 物理阻塞（sBlockHard 降序）
  allBlocked: boolean;       // 窗口内全部候选低于舒适间隙
  sBlock: number;            // best 的 sBlock（舒适阻塞时）
  allBlockedHard: boolean;   // 全部候选连物理硬阈值都过不去（标称）
  sBlockHard: number;        // 物理层最晚首个阻塞弧长（allBlockedHard 时有效，否则 Infinity）
  /** 迟滞输入（tight.ts）：∃候选通过"更严走廊"（物理 + 0.15m）——SEALED 退出条件 */
  deepClear: boolean;
  /** 迟滞输入：∃候选通过"更松走廊"（物理 − 0.15m）——其反为 SEALED 进入条件 */
  permissiveClear: boolean;
  /** 物理走廊实际膨胀量（仿真单位；= 车半宽 + marginFor(几何评估速度)×4）——调试与回归断言用 */
  marginU: number;
  /** 可行驶半宽（车中心 |l| 上限，仿真单位）——可视化/调试用 */
  lMaxU: number;
  /** 物理走廊（自由段 + 瓶颈）——可视化数据源（可视化几何见 corridor.ts 的纯函数） */
  corridor: Corridor;
  /** 本帧未观测区口径（可视化/展示用） */
  unknownHandling: 'free' | 'blocked' | 'speed';
  /** 可视化用的细网格物理走廊（2× 采样 = 1.5m；只影响画出来的带子边界平滑度，
   *  不参与任何判定——判定仍走 3m 网格的 corridor，保证与路径采样索引对齐） */
  vizCorridor: Corridor;
}

export function solvePathLayer(o: {
  ref: FrenetReference; l0: Px; dl0: DimLess; s0: number; v0: number;
  /** 道路半宽（px）；缺省 TRACK_HALF_WIDTH（径向图）。OSM 图按车道数更窄 → 规划域随之收窄 */
  trackHalfWidth?: Px;
  /** 半宽随 s 变化（逐段属性：1 车道匝道 ≠ 4 车道主路）。给了则规划域**逐站**取，
   *  规划域边始终贴着该段的路缘（而非全图一个值）。 */
  halfWidthAt?: (s: number) => number;
  carX: number; carY: number; lidarScan: LidarScan;
  carWidth: Px; collisionRadius: Px; laneOffset: Px;
  /** 【A 方案】物体层给出的**已确认**轨迹（Frenet；px / px·s⁻¹）。有速度的物体用更小的侧向裕度
   *  （政策见 margin.ts 的 MARGIN_MOVING_EXTRA）——"超越/被超越"本来就是贴着过去的。 */
  objects?: { s: number; l: number; width: number; length: number; vS: number; vL: number }[];
  /** 车道中心（px；缺省 → laneOffset）与车道宽（px）：判"物体在不在我这条车道"用 */
  laneCenters?: number[];
  laneWidth?: number;
  prevTargetL: number | null;
  /** 路线总弧长（开环）；传入后走廊视程截到路线剩余——mission 之外的障碍
   *  不参与规划（"路线终点之外"不是车要去的地方）。闭环（巡游）不传。 */
  sEnd?: number;
}): PathLayerResult {
  // 可行驶域 = checkBounds 复位阈值（车半宽 + 复位缓冲）− 过冲预留；用同一个 BOUNDS_BUFFER
  // 派生，避免"规划域放宽了、复位崖没动"（会撞复位传送）。
  // 过冲预留置 0（2026-09-26，用户决策"走廊上限加到 6m、留 0.5m 富裕"）：域边 = 复位崖 →
  // 车中心可达 ±6.0m（= 边界 7.5m − 车半宽 1.0m − 复位缓冲 0.5m），车身离墙 0.5m。代价：
  // 域边不再留 0.6m 跟踪过冲余量，贴边过冲可能触发复位传送——以批量（种子 6-10）"复位次数"
  // 为判据验收：A/B 实测 0 次且到达率 7/10 → 8/10（见 ISSUE_HISTORY 的域加宽记录）。
  // A/B 开关：改回 m(0.6) 即旧行为（域边比复位崖内缩 0.6m）。
  const halfWidthU = (o.trackHalfWidth ?? TRACK_HALF_WIDTH) as number;
  const carHalfU = (o.carWidth as number) / 2 + (BOUNDS_BUFFER as number);
  /**
   * 【规划域必须包含车辆当前横向位置】否则会出现"候选路径的起点就在走廊外 → 全被判碰撞 →
   * 永久停车"的死锁。实测：单车道匝道（半宽 1.875m）上域只有 ±0.375m，而车实测 l0=0.41m
   * （最近邻投影与点到线段两种口径差几厘米）→ 三环场景卡死在匝道入口。
   *
   * 因此域 = clamp(max(名义域, |l0| + 5cm), 名义域, 物理上限)，物理上限 = 半宽 − 车半宽
   * （**不含**复位缓冲 → 不会因为放宽域而把车送到复位崖外）。
   * 宽路上 max 取名义域、上限更大 → 逐位不变（径向图零回归）。
   */
  // 上限 = **复位崖**（checkBounds 同一口径：半宽 − 车半宽 − 缩放后的复位缓冲），
  // 绝不比崖更宽——域比崖宽会让规划器把车贴到崖外（P8 贴边棘轮 / 复位传送）。
  const cliffOf = (halfU: number) => halfU - ((o.carWidth as number) / 2 + boundsBufferFor(halfU, (o.carWidth as number) / 2));
  const domainOf = (halfU: number) => {
    const nominal = cliffOf(halfU) - (DOMAIN_OVERSHOOT_RESERVE as number);
    return Math.min(Math.max(nominal, Math.abs(o.l0 as number) + (m(0.05) as number)), cliffOf(halfU));
  };
  const L_MAX = domainOf(halfWidthU);
  /** 逐站规划域（与 checkBounds 同一口径：半宽 − 车半宽 − 复位缓冲 − 过冲预留） */
  const lMaxAt = o.halfWidthAt
    ? (s: number) => domainOf((o.halfWidthAt as (s: number) => number)(s))
    : undefined;
  const lo = o.laneOffset as number;

  // ── 障碍命中点采集（笛卡尔 → 走廊膨胀）──
  // 路径层的判据是几何问题："路径离感知到的障碍表面点是否足够远"。
  // 命中点/链段按"车半宽 + 裕度"的圆/胶囊膨胀成阻塞区（不绕道极坐标锥形——
  // 锥形窗口在长视程下反复出现边界误判，2026-09-19 解耦升级实测）。
  // 职责分工：路径层判"几何上过得去"（含跟踪误差余量）；速度层管"安全距离"
  // （刹车/停车线）。舒适间隙 = 车半宽 + 1.5m：含跟踪误差（实测 ~0.2m）与表面
  // 采样误差的余量——初版 0.5m 实测过乐观（接受紧通过后易进入死口袋，
  // 批量 A/B 劣化 6/195→7/407，2026-09-19）。
  const maxDist = MAX_SENSOR_DIST as number;
  const DISCONT = m(2.0) as number;                     // 表面连续性阈值（相邻命中深度差）

  // 命中点（按射线顺序排列，供表面折线构建）
  const hits: { x: number; y: number; d: number; low: boolean }[] = [];
  for (let i = 0; i < o.lidarScan.distances.length; i++) {
    const d = o.lidarScan.distances[i] as number;
    if (d >= maxDist - 1) continue;                     // 无命中/已过滤
    const conf = o.lidarScan.confidences?.[i] ?? 1;
    const a = o.lidarScan.angles[i] as number;
    hits.push({ x: o.carX + Math.cos(a) * d, y: o.carY + Math.sin(a) * d, d, low: conf < 1 });
  }
  // 命中表面折线：相邻命中点深度连续（|Δd| ≤ DISCONT）时连成线段。
  // 必须连接——孤立点表示会让路径从"射线间隙"穿针（全宽墙在远距离处
  // 命中行距可达十几单位，实测本车道路径恰好从行间漏过）。
  const hitSegs: { ax: number; ay: number; bx: number; by: number; low: boolean }[] = [];
  for (let i = 0; i < hits.length - 1; i++) {
    if (Math.abs(hits[i].d - hits[i + 1].d) <= DISCONT) {
      hitSegs.push({ ax: hits[i].x, ay: hits[i].y, bx: hits[i + 1].x, by: hits[i + 1].y,
        low: hits[i].low || hits[i + 1].low });
    }
  }

  // ── 走廊族（A 走廊表示 + B 误差裕度/迟滞带，spec 2026-09-25 §3/§4）──
  //   舒适走廊（车半宽 + max(1.5m, 标定 p95)）：横向排名 + "要不要慢行"（晚承诺窗口内）
  //   物理走廊（车半宽 + marginFor(v)）：能不能过 → 停车线；候选由它派生（必须用
  //     物理走廊：舒适走廊在紧间隙处可能整体封死，用它派生会丢掉紧间隙候选）
  //   更严走廊（物理 + 0.15m）  ：迟滞退出条件（确实过得去才恢复）
  //   更松走廊（物理 − 0.15m）  ：迟滞进入条件（确实过不去才封死）
  //
  // 【裕度口径（spec §4.1/§5）】物理走廊用**标定速度表** marginFor(v)（v = 几何评估
  //   速度 = max(v0, 慢行档)，解析见 planner 的收敛迭代）；舒适走廊取 max(1.5m, 标定 p95)
  //   ——实测 p95 ≤1.15m < 1.5m，故舒适档在当前速度域内仍是 1.5m（有实测下界兜底）。
  const halfW = (o.carWidth as number) / 2;
  // 几何评估速度（m/s）：v0 换成 m/s 后与慢行档取大——注意 TIGHT_SPEED 是仿真单位（32），
  // 必须 /4 再进 marginFor（评审实测教训：直接传 32 会让裕度恒为表尾 0.615m，标定表失效）
  const marginU = marginFor(Math.max((o.v0 as number) / 4, (TIGHT_SPEED as number) / 4)) * 4;
  const comfortU = Math.max(m(1.5) as number, marginU);        // 舒适裕度（配不配，实测下界）
  const bandU = HYST_BAND as number;
  // 走廊视程：默认 120m；有路线时截到"路线剩余"（步长不变 → 网格与路径采样
  // 索引仍一一对应，firstExit 按索引比较依然成立；超出走廊的路径点不判阻塞）。
  const stepU = (PATH_HORIZON as number) / PATH_STEPS;
  const corridorHorizon = o.sEnd === undefined ? (PATH_HORIZON as number)
    : Math.max(stepU, Math.floor((o.sEnd - o.s0) / stepU) * stepU);
  const coverage = UNKNOWN_HANDLING === 'blocked' ? coverageFromScan(o.lidarScan, o.carX, o.carY) : undefined;
  // 【A 方案】移动物体的 Frenet 包围盒 + 它们用的膨胀量。判据：物体层说它在动（|v| > 1.5m/s）；
  // 静止/未知一律按静态障碍处理（保守）。
  const MOVING_MIN_V = mps(1.5) as number;
  // 我这条车道的中心/半宽（判"物体在不在我车道"）
  const myLaneCenter = (() => {
    const lc = o.laneCenters;
    if (!lc || lc.length === 0) return o.laneOffset as number;
    let c = lc[0];
    for (const v of lc) if (Math.abs(v - (o.l0 as number)) < Math.abs(c - (o.l0 as number))) c = v;
    return c;
  })();
  const myLaneHalf = ((o.laneWidth ?? m(3.75)) as number) / 2;
  const movingBoxes = (MARGIN_MOVING_ENABLED ? (o.objects ?? []) : [])
    .filter(ob => Math.hypot(ob.vS, ob.vL) > MOVING_MIN_V)
    .map(ob => {
      // 【用户的硬要求】不在我这条车道、且**没有朝我这边动**的物体 ⇒ 它的阻塞带不许进我车道。
      // "在我车道上"按物体中心判（±半车道 + 0.5m 余量）；"朝我这边动"看横向速度分量 vL ✓
      // ⇒ **切入的车保留完整裕度**（安全不受影响 ✓），旁边同向行驶的车不再让我减速 ✓。
      const inMyLane = Math.abs(ob.l - myLaneCenter) <= myLaneHalf + (m(0.5) as number);
      const towardMe = Math.sign(ob.vL) === Math.sign(myLaneCenter - ob.l) && Math.abs(ob.vL) > (mps(0.5) as number);
      return {
        s1: ob.s - ob.length / 2 - (m(0.5) as number),
        s2: ob.s + ob.length / 2 + (m(0.5) as number),
        l1: ob.l - ob.width / 2 - (m(0.5) as number),
        l2: ob.l + ob.width / 2 + (m(0.5) as number),
        centerL: ob.l,
        mayIntrude: inMyLane || towardMe,
      };
    });
  const mkCorridor = (extra: number, steps = Math.round(corridorHorizon / stepU), useCoverage = true) => buildCorridor({
    ref: o.ref, s0: o.s0, hits, hitSegs,
    inflate: halfW + extra, inflateLow: halfW + extra + (m(2.0) as number),
    moving: movingBoxes, movingInflate: halfW + (MARGIN_MOVING_EXTRA as number),
    myLaneCenter, myLaneHalf,
    lMax: L_MAX, lMaxAt, horizon: corridorHorizon, steps,
    windowRel: COMMIT_WINDOW as number,
    coverage: useCoverage ? coverage : undefined,
  });
  const comfort = mkCorridor(comfortU);
  const physical = mkCorridor(marginU);
  const deep = mkCorridor(marginU + bandU);
  const permissive = mkCorridor(Math.max(0, marginU - bandU));
  // 可视化专用：2× 采样（1.5m）的物理走廊——带子边界更贴曲率（纯绘制用，判定不受影响）
  const vizCorridor = mkCorridor(marginU, PATH_STEPS * 2, false);   // 可视化不裁（灰蓝标注未观测区）

  // 候选由走廊瓶颈处派生（间隙中心必被覆盖——旧版固定档位打不中缝隙中心，
  // 障碍贴一侧时被迫贴走廊边缘，oval-s2 类越界复位由此而来）
  // 候选目标夹紧用**承诺窗内的最小域**：若只看 s0 的宽域，进入窄段时目标可能全被走廊否掉
  // （走廊逐站判越界）→ 会退化成不必要的停车。
  let lMaxCommit = L_MAX;
  if (lMaxAt) {
    const stepU2 = m(5) as number;
    for (let d = 0; d <= (COMMIT_WINDOW as number); d += stepU2) {
      lMaxCommit = Math.min(lMaxCommit, lMaxAt(o.s0 + d));
    }
  }
  const targets = deriveTargets({
    corridor: physical, laneOffset: lo, l0: o.l0 as number, lMax: lMaxCommit, maxTargets: 7,
  });

  const candidates: PathCandidate[] = [];
  let sawDeepClear = false;         // ∃候选通过"更严走廊"（迟滞退出条件）
  let sawPermissiveClear = false;   // ∃候选通过"更松走廊"（迟滞进入条件的反）
  for (const targetL of targets) {
    // 横向夹在 [l0, target] 内：为满足初值斜率（车头指向），五次多项式会先向
    // 端点外下沉，逐帧重读 l0 形成棘轮 → 车贴走廊边缘时撞 |l|=5.0 复位崖
    // （hairpin-s5 批量实证，l 从 −4.4 逐帧滑到 −4.93 后复位）。
    const lMin = Math.min(o.l0 as number, targetL), lMax = Math.max(o.l0 as number, targetL);

    // 路径构造（给定发展段长度 developS）：两段式，前 developS 内完成横移、其后保持
    const buildPath = (developS: number) => {
      const latPoly = solveQuintic(o.l0, o.dl0, perpx(0), px(targetL), dimless(0), perpx(0), developS);
      const evalLat = (sRel: number): Px => px(Math.max(lMin, Math.min(lMax,
        latPoly.eval(Math.min(sRel, developS)) as number)));
      const points: PathCandidate['points'] = [];
      let centerSum = 0, curvSum = 0, prevYaw: number | null = null;
      for (let i = 0; i <= PATH_STEPS; i++) {
        const sRel = (i / PATH_STEPS) * (PATH_HORIZON as number);
        const li = evalLat(sRel);
        const cart = frenetToCartesian(o.s0 + sRel, li, o.ref);
        points.push({ s: o.s0 + sRel, l: li, x: cart.x, y: cart.y, yaw: cart.yaw });

        const d = (li as number) - lo;
        centerSum += d * d;
        if (prevYaw !== null) {
          let dyaw = (cart.yaw as number) - prevYaw;
          dyaw = Math.atan2(Math.sin(dyaw), Math.cos(dyaw));
          curvSum += Math.abs(dyaw);
        }
        prevYaw = cart.yaw as number;
      }
      return { points, evalLat, centerSum, curvSum };
    };

    // 自适应发展段（adaptive development）：默认 40m 内若已越出走廊，说明
    // 路径还没横移到位就从障碍横向范围里穿过（判的是实际曲线而非终点目标）——
    // 把发展段缩到越界点之前，让横移提前完成（hairpin-s7 实测停车根因：
    // 障碍 20.8m 处，40m 发展段到那里只走完一半 → 连硬阈值都过不去 → 停死）。
    // 下限按运动学随速度缩放：峰值横向加速度 = 5.77·|Δl|·v²/S² ≤ A_LAT_MAX
    // → S_min = √(5.77·|Δl|·v²/A_LAT_MAX)。高速时不得规划跟不上的急横移
    // （v=35m/s 时 12m 发展段横向加速度 ~39m/s² 是物理幻想）；低速（含停车后
    // 起步）才允许急横移穿过紧间隙。
    const dl = Math.abs((targetL as number) - (o.l0 as number));
    const developMin = Math.max(PATH_DEVELOP_MIN as number, Math.min(
      PATH_DEVELOP as number,
      Math.sqrt(QUINTIC_PEAK_ACCEL * dl * o.v0 * o.v0 / (A_LAT_MAX as number)),
    ));
    let usedDevelop = PATH_DEVELOP as number;
    let { points, evalLat, centerSum, curvSum } = buildPath(usedDevelop);
    const firstAny = firstExit(comfort, points, Infinity);
    if (firstAny - o.s0 < (PATH_DEVELOP as number)) {
      usedDevelop = Math.max(developMin,
        Math.min(PATH_DEVELOP as number, firstAny - o.s0 - (DEVELOP_MARGIN as number)));
      ({ points, evalLat, centerSum, curvSum } = buildPath(usedDevelop));
    }

    // 阻塞判定 = 越出走廊：舒适走廊只看窗口（决定横向排名）；物理走廊看全程
    // （决定停车线）；更严/更松走廊的全程结果供迟滞状态机（tight.ts）消费。
    const sBlock = firstExit(comfort, points, COMMIT_WINDOW as number);
    const sBlockHard = firstExit(physical, points, Infinity);
    const blocked = sBlock < Infinity;
    const blockedHard = sBlockHard < Infinity;
    if (firstExit(deep, points, Infinity) === Infinity) sawDeepClear = true;
    if (firstExit(permissive, points, Infinity) === Infinity) sawPermissiveClear = true;

    // 贴边惩罚（edge penalty）：逼近物理走廊边界时平滑加代价，与"间隙中心
    // 偏好"一致（也抑制贴走廊边缘——贴边距复位崖仅 0.6m，oval-s2 类教训）。
    let edgeSum = 0;
    for (let i = 0; i < points.length; i++) {
      const step = physical.steps[i];
      if (!step) break;                                       // 走廊之外（路线结束后）不惩罚
      const l = points[i].l as number;
      let dEdge = 0;
      for (const sg of step.free) {
        if (l >= sg.lmin && l <= sg.lmax) { dEdge = Math.min(l - sg.lmin, sg.lmax - l); break; }
      }
      const over = (EDGE_BAND as number) - dEdge;
      if (over > 0) edgeSum += over * over;
    }

    let cost = W_PATH_CENTER * centerSum / points.length + W_PATH_CURV * curvSum / points.length
      + W_EDGE * edgeSum / points.length;
    if (o.prevTargetL !== null && Math.abs(targetL - o.prevTargetL) > 5 && targetL !== 0) cost += W_SWITCH_PATH;

    candidates.push({
      targetL: px(targetL), points, blocked, sBlock, blockedHard, sBlockHard, cost,
      develop: usedDevelop,
      evalL: (s) => evalLat(Math.max(0, Math.min(s, PATH_HORIZON as number))),
    });
  }

  // 分级排序：舒适畅通（按代价）→ 舒适阻塞但物理可过（按最晚舒适碰撞）→ 物理阻塞（按最晚硬碰撞）
  // 注意 tier3 必须带 `c.blocked` 条件：blocked 是**窗口内**判定、blockedHard 是**全程**
  // 判定——"窗口内畅通但远处（>60m）被挡"是合法组合（晚承诺哲学：远处障碍不参与选路），
  // 该候选属 tier1；若 tier3 只按 blockedHard 过滤会把它重复计入（ranked 出现重复项）。
  // 换边代价：与 cost 里的 W_SWITCH_PATH 同判据，折算进分级主键（见 LATERAL_SWITCH_BAND）
  const switchU = (t: number) => (o.prevTargetL !== null && Math.abs(t - o.prevTargetL) > 5 && t !== 0)
    ? (LATERAL_SWITCH_BAND as number) : 0;
  const tierClear = candidates.filter(c => !c.blocked).sort((a, b) => a.cost - b.cost);
  const tierSqueeze = candidates.filter(c => c.blocked && !c.blockedHard).sort((a, b) =>
    (b.sBlock - switchU(b.targetL as number)) - (a.sBlock - switchU(a.targetL as number)));
  const tierHard = candidates.filter(c => c.blocked && c.blockedHard).sort((a, b) =>
    (b.sBlockHard - switchU(b.targetL as number)) - (a.sBlockHard - switchU(a.targetL as number)));
  const ranked = [...tierClear, ...tierSqueeze, ...tierHard];
  const best = ranked[0];
  // 停车线基准：全部候选物理阻塞（全程）时触发。判据必须直接对候选全查
  // `blockedHard`——tierHard 里只有"窗口内也阻塞"的子集（tier 互斥修复后），
  // 用 tierHard.length 会在"窗口外全宽墙"（窗口内畅通、全程硬阻塞）下漏判为
  // 可通行 → 停车线取到 Infinity → 速度剖面 NaN（planner-safety 115m 实测）。
  const allBlockedHard = candidates.every(c => c.blockedHard);
  return {
    best, ranked,
    allBlocked: tierClear.length === 0,
    sBlock: best.sBlock,
    allBlockedHard,
    // 迟滞输入（tight.ts）：deepClear = ∃候选通过"更严走廊"（m+0.15）；
    // permissiveClear = ∃候选通过"更松走廊"（m−0.15）
    deepClear: sawDeepClear,
    permissiveClear: sawPermissiveClear,
    marginU, lMaxU: L_MAX, corridor: physical, vizCorridor,
    unknownHandling: UNKNOWN_HANDLING,
    sBlockHard: allBlockedHard ? best.sBlockHard : Infinity,
  };
}
