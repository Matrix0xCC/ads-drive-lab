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
// 停车线由"物理可过"硬阈值（HARD_CLEAR）判定：
//   CLEAR（舒适，车半宽+1.5m）——判"顺不顺"，只影响排名；
//   HARD_CLEAR（车半宽+0.4m）——判"能不能过"，决定停车线。
// 全部候选连硬阈值都过不去才停车；否则选可过的路径并限速慢行。
// （批量 A/B：只用舒适阈值判停 → 6/10 场景中途永久停车，其中 oval-s4/
//   hairpin-s5 旧规划器干净通过——存在可行路线，被过保守的判停拒了。）

import { FrenetReference, LidarScan } from '../../shared/types.js';
import { Radians, Px, DimLess, m, px, perpx, dimless } from '../../shared/dimensions.js';
import { solveQuintic } from './polynomial.js';
import { frenetToCartesian } from './frenet.js';
import { MAX_SENSOR_DIST } from '../perception.js';
import { TRACK_HALF_WIDTH } from '../../env/track.js';

export const PATH_HORIZON: Px = m(120);
/** 横移发展距离：在前 40m 内完成横移到目标，其后保持偏移。
 *  若横移按整段 120m 发展，到障碍处偏移还太小（测试实证），不能完成避让。 */
export const PATH_DEVELOP: Px = m(40);
/** 横向选路提交窗口：只有窗口内的阻塞参与候选排名（"横向耐心"——障碍还远时
 *  不提前选边；进入窗口再决策仍有 60m 完成横移）。停车线不看此窗口。 */
export const COMMIT_WINDOW: Px = m(60);
export const PATH_STEPS = 40;

const W_PATH_CENTER = 0.3;    // 中心偏离²（沿用旧 W_CENTER 量级）
const W_PATH_CURV = 1.0;      // 路径曲率（航向变化累计）
const W_SWITCH_PATH = 100;    // 路径跨档切换惩罚（沿用旧 W_SWITCH）
const NUDGE = m(2.5);         // 挪移档横向偏移

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
  /** 合成阶段按相对弧长求横向偏移（供 l(s(t)) 采样）；内部 clamp 到 [0, PATH_HORIZON] */
  evalL(s: number): Px;
}

export interface PathLayerResult {
  best: PathCandidate;
  ranked: PathCandidate[];   // 排序分级：舒适畅通（cost 升序）→ 舒适阻塞但物理可过（sBlock 降序）→ 物理阻塞（sBlockHard 降序）
  allBlocked: boolean;       // 窗口内全部候选低于舒适间隙
  sBlock: number;            // best 的 sBlock（舒适阻塞时）
  allBlockedHard: boolean;   // 全部候选连物理硬阈值都过不去 → 停车线触发条件
  sBlockHard: number;        // 物理层最晚首个阻塞弧长（allBlockedHard 时有效，否则 Infinity）
}

/** 线段 AB 到点 P 的最短距离 */
function segPointDist(ax: number, ay: number, bx: number, by: number, px0: number, py0: number): number {
  const dx = bx - ax, dy = by - ay;
  const len2 = dx * dx + dy * dy;
  if (len2 === 0) return Math.hypot(px0 - ax, py0 - ay);
  let t = ((px0 - ax) * dx + (py0 - ay) * dy) / len2;
  t = Math.max(0, Math.min(1, t));
  return Math.hypot(px0 - (ax + t * dx), py0 - (ay + t * dy));
}

/** 两线段最短距离（含交叉与端点最近情形） */
function segSegDist(
  ax: number, ay: number, bx: number, by: number,
  cx: number, cy: number, dx2: number, dy2: number,
): number {
  const ux = bx - ax, uy = by - ay;
  const vx = dx2 - cx, vy = dy2 - cy;
  const wx = ax - cx, wy = ay - cy;
  const a = ux * ux + uy * uy;       // |u|²
  const b = ux * vx + uy * vy;       // u·v
  const c = vx * vx + vy * vy;       // |v|²
  const d = ux * wx + uy * wy;       // u·w
  const e = vx * wx + vy * wy;       // v·w
  const denom = a * c - b * b;
  let sN: number, sD = denom, tN: number, tD = denom;
  if (denom < 1e-9) {                // 平行：取端点组合的最小值
    sN = 0; sD = 1; tN = e; tD = c;
  } else {
    sN = b * e - c * d;
    tN = a * e - b * d;
  }
  const s = sN <= 0 ? 0 : (sN >= sD ? 1 : sN / sD);
  const t = tN <= 0 ? 0 : (tN >= tD ? 1 : tN / tD);
  return Math.hypot((ax + s * ux) - (cx + t * vx), (ay + s * uy) - (cy + t * vy));
}

export function solvePathLayer(o: {
  ref: FrenetReference; l0: Px; dl0: DimLess; s0: number;
  carX: number; carY: number; lidarScan: LidarScan;
  carWidth: Px; collisionRadius: Px; laneOffset: Px;
  prevTargetL: number | null;
}): PathLayerResult {
  // 可行驶域（与 checkBounds 复位阈值一致；0.6m 跟踪过冲预留）
  const L_MAX = (TRACK_HALF_WIDTH as number)
    - ((o.carWidth as number) / 2 + (m(1.5) as number)) - (m(0.6) as number);
  const lo = o.laneOffset as number;

  // ── 障碍命中点采集（笛卡尔）──
  // 路径层的判据是几何问题："路径离感知到的障碍表面点是否足够远"。
  // 直接以命中点为圆心做间隙判定——不绕道极坐标锥形（锥形窗口在长视程下
  // 反复出现边界误判，2026-09-19 解耦升级实测）。
  // 职责分工：路径层判"几何上过得去"（含跟踪误差余量）；速度层管"安全距离"
  // （刹车/停车线）。间隙 = 车半宽 + 1.5m：含跟踪误差（实测 ~0.2m）与表面
  // 采样误差的余量——初版 0.5m 实测过乐观（接受紧通过后易进入死口袋，
  // 批量 A/B 劣化 6/195→7/407，2026-09-19）。
  const CLEAR = ((o.carWidth as number) / 2 + (m(1.5) as number));
  const CLEAR_LOW = CLEAR + (m(2.0) as number);         // 低置信命中（边缘模糊）更大间隙
  // 物理可通过硬阈值：车半宽 + 0.4m 车身余量。路径低于此值才算"过不去"。
  const HARD_CLEAR = ((o.carWidth as number) / 2 + (m(0.4) as number));
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

  // 候选目标：{车道中心, ±2.5m/±5m 挪移, ±走廊边界} clamp + 去重。
  // 两级挪移覆盖中间间隙：单级 2.5m 时，障碍贴一侧的宽间隙里只有走廊边缘候选
  // 可清，车被推上边缘（距复位崖 |l|=5.0 仅 0.6m 跟踪余量）→ 越界复位
  // （oval-s2 批量实证：复位后落回障碍正前方导致连锁碰撞）。
  const raw = [lo, lo - (NUDGE as number), lo + (NUDGE as number),
    lo - 2 * (NUDGE as number), lo + 2 * (NUDGE as number), -L_MAX, L_MAX];
  const targets: number[] = [];
  for (const v of raw) {
    const c = Math.max(-L_MAX, Math.min(L_MAX, v));
    if (!targets.some(t => Math.abs(t - c) < 0.5)) targets.push(c);
  }

  const candidates: PathCandidate[] = [];
  for (const targetL of targets) {
    // 两段式路径：前 PATH_DEVELOP 内五次多项式完成横移，其后保持目标偏移。
    // 横向夹在 [l0, target] 内：为满足初值斜率（车头指向），五次多项式会先向
    // 端点外下沉，逐帧重读 l0 形成棘轮 → 车贴走廊边缘时撞 |l|=5.0 复位崖
    // （hairpin-s5 批量实证，l 从 −4.4 逐帧滑到 −4.93 后复位）。
    const lMin = Math.min(o.l0 as number, targetL), lMax = Math.max(o.l0 as number, targetL);
    const latPoly = solveQuintic(o.l0, o.dl0, perpx(0), px(targetL), dimless(0), perpx(0), PATH_DEVELOP as number);
    const evalLat = (sRel: number): Px => px(Math.max(lMin, Math.min(lMax,
      latPoly.eval(Math.min(sRel, PATH_DEVELOP as number)) as number)));
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

    // 阻塞判定：路径折线到"命中点/命中表面折线"的最短距离 ≤ 阈值，返回沿路径
    // 首个命中的绝对弧长（Infinity = 无命中）。用线段而非采样点（路径采样间距
    // 3m > 间隙半径，会混叠漏判，实测）。
    // 两次扫描：舒适层只看窗口（COMMIT_WINDOW，决定横向排名）；硬阈值看全程
    // （决定停车线——只有物理上真的过不去才停）。
    const firstBlockArc = (windowRel: number, hard: boolean): number => {
      for (let i = 0; i < points.length - 1; i++) {
        if (points[i].s - o.s0 > windowRel) break;
        const a = points[i], b = points[i + 1];
        for (const h of hits) {
          const tol = hard ? HARD_CLEAR : (h.low ? CLEAR_LOW : CLEAR);
          if (segPointDist((a.x as number), (a.y as number), (b.x as number), (b.y as number), h.x, h.y) <= tol) {
            return a.s;
          }
        }
        for (const sgm of hitSegs) {
          const tol = hard ? HARD_CLEAR : (sgm.low ? CLEAR_LOW : CLEAR);
          if (segSegDist((a.x as number), (a.y as number), (b.x as number), (b.y as number),
            sgm.ax, sgm.ay, sgm.bx, sgm.by) <= tol) {
            return a.s;
          }
        }
      }
      return Infinity;
    };
    const sBlock = firstBlockArc(COMMIT_WINDOW as number, false);
    const sBlockHard = firstBlockArc(Infinity, true);
    const blocked = sBlock < Infinity;
    const blockedHard = sBlockHard < Infinity;

    let cost = W_PATH_CENTER * centerSum / points.length + W_PATH_CURV * curvSum / points.length;
    if (o.prevTargetL !== null && Math.abs(targetL - o.prevTargetL) > 5 && targetL !== 0) cost += W_SWITCH_PATH;

    candidates.push({
      targetL: px(targetL), points, blocked, sBlock, blockedHard, sBlockHard, cost,
      evalL: (s) => evalLat(Math.max(0, Math.min(s, PATH_HORIZON as number))),
    });
  }

  // 分级排序：舒适畅通（按代价）→ 舒适阻塞但物理可过（按最晚舒适碰撞）→ 物理阻塞（按最晚硬碰撞）
  const tierClear = candidates.filter(c => !c.blocked).sort((a, b) => a.cost - b.cost);
  const tierSqueeze = candidates.filter(c => c.blocked && !c.blockedHard).sort((a, b) => b.sBlock - a.sBlock);
  const tierHard = candidates.filter(c => c.blockedHard).sort((a, b) => b.sBlockHard - a.sBlockHard);
  const ranked = [...tierClear, ...tierSqueeze, ...tierHard];
  const best = ranked[0];
  // 停车线基准：全部候选物理阻塞时，取"最晚硬碰撞"作为可抵达的最远点
  const allBlockedHard = tierHard.length === candidates.length;
  return {
    best, ranked,
    allBlocked: tierClear.length === 0,
    sBlock: best.sBlock,
    allBlockedHard,
    sBlockHard: allBlockedHard ? best.sBlockHard : Infinity,
  };
}
