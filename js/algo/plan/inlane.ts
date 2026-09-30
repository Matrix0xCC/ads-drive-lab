// ==================== 算法：本车道前方间隙（纵向限速的输入） ====================
//
// 【要解决什么】旧的巡航目标是一条一维启发式：
//     vCap = clamp( min(全部射线距离) / 2, 17.5, 35 )
// 它在**径向赛道时代**是合理的（那时候"近的东西"基本都在自己正前方）。但多车道道路上它错了：
// 相邻车道、甚至侧后方的命中同样进 min —— 旁边一辆车超车（侧向 3m）就把巡航目标钉到 17.5m/s
// （用户实测截图：超车时自车无故减速并往另一侧偏）。
//
// 【现在的口径】只看同时满足两条的命中点：
//   ① **在自车所在车道内**：|hitL − 本车道中心| ≤ 车道半宽 + 余量
//   ② **在自车前方**：hitS > 自车 s
// 取其中最小的**纵向间隙**作为"前方障碍距离"。于是：
//   · 相邻车道的车不再影响巡航速度 ✓（它仍然在走廊里，横向避让是路径层的事）
//   · 加后向传感器之后，后方目标天然被 ② 排除 ✓（这是多向传感套件能安全接入的前提）
//
// 【余量为什么加】命中点可能落在车道线附近（宽车、投影误差）→ 用半个车道宽 + 0.35m 作判据，
// 宁可保守一点把"骑线"的物体算进来，也不放过真正挡在前面的东西。
//
// 【找不到任何命中时返回 null】调用方据此走"没有前方障碍"的分支（与旧行为一致：不额外限速）。

import { Px } from '../../shared/dimensions.js';
import { NEAR_FIELD, NEAR_BAND_MARGIN, nearFieldBlocking } from './bodyFrame.js';

/** 车道判据的横向余量（px）：吸收投影误差与"骑线"的物体 */
export const LANE_BAND_MARGIN = 1.4;   // 0.35m × 4

/**
 * 本车道、前方、最近的**纵向**间隙（px）；没有 → null。
 *
 * @param scan       `{ hitS, hitL }`（由感知填充；无命中 = NaN）
 * @param selfS      自车在当前参考线上的弧长（px）
 * @param selfL      自车的横向偏移（px，带符号）
 * @param laneCenters 车道中心（px，带符号）；缺省/空 → 退化成"整条路一个带"
 * @param laneWidth   车道宽（px）
 */
export function inLaneGapAhead(
  scan: { hitS?: number[]; hitL?: number[]; angles?: number[]; distances?: number[] },
  selfS: number,
  selfL: number,
  laneCenters: number[] | undefined,
  laneWidth: number,
  /** 【本体坐标系近场交叉校验】给了 pose 才对**近场**命中额外做一次本体坐标判断。
   *  方向是**只加约束**：近场里落在我车头窄带内的命中，即使 Frenet 判它不在我车道，也算前方障碍 ✓
   *  （地图车道宽/参考线有误差时兜住近处真障碍；永远不会解除已判定的阻塞 ⇒ 不可能削弱安全 ✓）。 */
  carPose?: { x: number; y: number; angle: number; halfWidth: number },
): number | null {
  const hs = scan.hitS, hl = scan.hitL;
  if (!hs || !hl) return null;                       // 感知没给位置（老归档/合成扫描）→ 调用方退回旧口径

  // 本车道中心 = 离自车横向位置最近的那条车道中心
  let center = 0;
  if (laneCenters && laneCenters.length > 0) {
    center = laneCenters[0];
    for (const c of laneCenters) if (Math.abs(c - selfL) < Math.abs(center - selfL)) center = c;
  }
  const half = (laneWidth > 0 ? laneWidth / 2 : 0) + LANE_BAND_MARGIN;

  let best: number | null = null;
  const n = Math.min(hs.length, hl.length);
  for (let i = 0; i < n; i++) {
    const s = hs[i], l = hl[i];
    if (!Number.isFinite(s) || !Number.isFinite(l)) continue;   // 无命中/命中的是路缘
    // ① 在不在我这条车道：道路坐标系（Frenet）为主；近场再用**本体坐标系**交叉校验一次（只加约束）
    let inLane = Math.abs(l - center) <= half;
    if (!inLane && carPose && scan.angles && scan.distances) {
      const d = scan.distances[i] as number, a = scan.angles[i] as number;
      if (Number.isFinite(d) && d > 0) {
        const hx = carPose.x + Math.cos(a) * d, hy = carPose.y + Math.sin(a) * d;
        inLane = nearFieldBlocking(carPose, hx, hy, carPose.halfWidth, NEAR_BAND_MARGIN, NEAR_FIELD);
      }
    }
    if (!inLane) continue;
    const gap = s - selfS;
    if (gap <= 0) continue;                                     // ② 在我后方（后向传感器加上之后靠它排除）
    if (best === null || gap < best) best = gap;
  }
  return best;
}
