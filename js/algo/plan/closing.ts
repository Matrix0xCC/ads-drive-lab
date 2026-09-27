// ==================== 算法：前车速度估计（雷达时序差分） ====================
//
// 【要解决什么】规划器的 ST 条目原先一律是"静态"（`tMin=0`，从现在起永久占据）→ 它把**移动的前车**
// 也当成一堵墙 → 只能"停在前车后面等它走"，于是走走停停（实测全程均速 3.4m/s）。
// 要会跟车，就必须知道**前车多快**。
//
// 【为什么不直接读障碍物的速度字段】本项目有一条硬约定：**规划器只信任 LiDAR**
// （真实系统里规划器拿不到障碍物的绝对速度真值）。所以速度必须**从雷达自己估**。
//
// 【怎么估】射线角度是**机体固定**的（第 i 条射线永远指向车体系里的同一个方向），所以：
//     d/dt( distance_i ) = -( 自车在该射线方向的速度分量 − 障碍在该方向的速度分量 )
//   记 closing_i = (前一次距离 − 本次距离) × 扫描频率（正数 = 在接近），θ_i = 射线相对车头夹角，则
//     v_ego·cosθ_i − v_obs_i = closing_i   ⇒   v_obs_i = v_ego·cosθ_i − closing_i
//   即"扣掉自车自己的贡献，剩下的就是对面在动"。
//
// 【为什么这样对静态障碍是安全的】对一堵静态墙：closing ≈ v_ego·cosθ ⟹ v_obs ≈ 0 ✓。
//   于是"静止障碍"和"移动前车"用**同一个公式**区分，不需要任何真值标记（也不会有特例分支）。
//
// 【确定性】时间基准**不用墙钟**（LidarScan.timestamp 是 performance.now，进不了确定性链路）：
//   按**扫描对象身份**去重（同一帧的雷达对象只算一次）+ 固定扫描周期 ⇒ 同种子逐帧一致。
//
// 【量纲】距离 px、速度 px/s、角度 rad（品牌类型在模块边界转换）。

import { Px, PxPerSec, Radians, rad, px, pxps, mps } from '../../shared/dimensions.js';

/** 只取前向 ±10° 的射线当"前车"候选（跟车只关心正前方那条车道） */
export const FRONT_HALF_ANGLE = rad((10 * Math.PI) / 180) as number;
/** 雷达扫描频率（与引擎的 PERCEPTION_HZ 一致）。**不能用墙钟**：这是确定性链路的一部分 */
export const SCAN_HZ = 20;
/** 距离超过它认为"前方没有东西"（雷达量程的 95%） */
const FRONT_NONE = 120 * 4 * 0.95;
/** 物理可信的前车速度范围（px/s）：超出即认为本帧估计不可用（宁可退回静态假设，也不能凭空放行） */
const LEAD_MIN = mps(-10) as number;
const LEAD_MAX = mps(50) as number;
/** 异常门控：与上一帧平滑值差超过它 → 本帧丢弃（最近射线在障碍之间跳变会造成 100m/s 级的假值） */
const LEAD_JUMP = mps(8) as number;
/** 低通系数（越大越跟手、越小越平滑）。1px 的距离抖动 ÷(1/20s) = 5m/s，不滤波无法用 */
const LEAD_ALPHA = 0.35;

export interface LeadEstimate {
  /** 前车净距（px）；null = 前方没东西 */
  gap: number | null;
  /** 前车速度（px/s）；null = 还没有两帧可比（刚启动/刚换场景） */
  leadSpeed: number | null;
  /** 接近速率（px/s，正 = 在接近）；null = 同上 */
  closing: number | null;
}

const NONE: LeadEstimate = { gap: null, leadSpeed: null, closing: null };

// 上一次扫描的样本（只留前向射线的距离；按对象身份去重）
let _prevScan: unknown = null;
let _prevDist: number[] | null = null;
let _prevIdx: number[] = [];
// 同一帧雷达会被规划调用多次（感知 20Hz、规划 15Hz）→ 缓存本次结果，
// 否则"重复帧"上会退化成"没有速度信息"，跟驰约束时有时无 → 抖动（实测踩过）
let _cacheScan: unknown = null;
/** 上一帧的"几何指纹"与时间戳：**换了雷达/换了世界就不能再做差分**（否则会把上一份数据的距离
 *  当成这一份的，估出一个荒唐的前车速度——实测踩过：单测之间互相污染，让一个必须急刹的场景
 *  被误判成"前车在跑"）。生产里雷达几何恒定、时间戳单调，所以这两条守卫永不误触发。 */
let _prevShape = '';
let _prevTs = -Infinity;
/** 世界键（同一个世界/车辆实例才做时序差分）。换世界 → 历史作废。
 *  【为什么必须有】曾经只有"几何 + 时间戳"两条守卫：不同测试（或换地图）拿着**相同几何**的
 *  两份雷达，时间戳又都是同一个值时，守卫不触发 → 拿上一份的距离当这一份的，估出一个荒唐的
 *  前车速度，把"必须急刹"的场景误判成"前车在跑"（实测踩过两条安全回归测试）。 */
let _prevKey: unknown = null;
/** 平滑后的前车速度（px/s）；null = 还没有可信估计 */
let _emaLead: number | null = null;
let _cacheEst: LeadEstimate = { gap: null, leadSpeed: null, closing: null };

/** 重置（换场景/回放/自车重定位时调用；步号回退也会自动触发） */
export function resetClosing(): void {
  _prevKey = null;
  _emaLead = null;
  _cacheScan = null;
  _prevScan = null;
  _prevDist = null;
  _prevIdx = [];
}

/**
 * 用最新一帧雷达更新"前车"估计。
 *
 * @param scan `{ angles, distances }`（`distances` 是**已过滤**的距离 = 真障碍；地图边界已被背景减除）
 *   `angles` 是**世界系**角度（perception 输出如此），所以要用 `car.angle` 换成车体系。
 * @param car  自车 `{ angle, speed }`（px/s）
 */
export function updateLead(
  scan: { angles: number[]; distances: number[]; timestamp?: number },
  car: { angle: number; speed: number },
  /** 世界键（传 `snapshot.track` 即可）：变了就丢掉历史 */
  key?: unknown,
): LeadEstimate {
  if (_cacheScan === scan) return _cacheEst;              // 同一帧 → 直接给上次结果（见上面的注释）
  // 几何指纹（长度 + 首/中/末角）或时间戳不连续 → 丢掉历史，本帧只记样本
  const n0 = scan.angles.length;
  const shape = `${n0}|${scan.angles[0]}|${scan.angles[n0 >> 1]}|${scan.angles[n0 - 1]}`;
  const ts = scan.timestamp;
  if (key !== undefined && key !== _prevKey) { _prevDist = null; _prevIdx = []; }
  if (key !== undefined) _prevKey = key;
  if (shape !== _prevShape || (ts !== undefined && ts < _prevTs)) {
    _prevDist = null; _prevIdx = [];
  }
  _prevShape = shape;
  if (ts !== undefined) _prevTs = ts;
  const n = scan.distances.length;
  // ① 前向射线（车体系）里最近的障碍
  const front: number[] = [];
  let gap = Infinity;
  let minI = -1;
  for (let i = 0; i < n && i < scan.angles.length; i++) {
    let a = scan.angles[i] - (car.angle as number);
    while (a > Math.PI) a -= 2 * Math.PI;                 // 归一化到 [-π,π]（perception 输出已归一化，
    while (a < -Math.PI) a += 2 * Math.PI;                // 但减去 car.angle 后要再归一化一次）
    if (Math.abs(a) > FRONT_HALF_ANGLE) continue;
    front.push(i);
    const d = scan.distances[i];
    if (Number.isFinite(d) && d < gap) { gap = d; minI = i; }
  }
  if (minI < 0 || gap >= FRONT_NONE) {                    // 正前方没东西 → 没有前车
    _prevScan = scan; _prevDist = null; _prevIdx = [];
    _emaLead = null;
    _cacheScan = scan;
    _cacheEst = NONE;
    return NONE;
  }

  // ② 时序差分：同一根射线的距离变化率（取"和最近障碍同一簇"的射线求均值，抗最近射线跳变）
  let leadSpeed: number | null = null;
  let closing: number | null = null;
  if (_prevDist && _prevScan !== scan) {
    // 同一簇 = 距离与"最近障碍"相差 8% 或 1m 以内。
    // 【别用宽松的约数】曾经写成 gap*1.25+4：前方 100m 的墙旁边那些打到 120m 量程外的射线
    // （480px < 504px）被算进同一簇，它们的距离不变 → 把接近速率摊薄，静态墙被估成 14.4m/s ✗。
    const clusterLim = gap + Math.max(4, gap * 0.08);
    const rates: number[] = [];
    let cnt = 0, sumCos = 0;
    for (const i of front) {
      const p = _prevIdx.indexOf(i);
      if (p < 0) continue;
      const dNow = scan.distances[i];
      if (!Number.isFinite(dNow) || dNow > clusterLim) continue;
      const dPrev = _prevDist[p];
      if (!Number.isFinite(dPrev)) continue;
      let a = scan.angles[i] - (car.angle as number);
      while (a > Math.PI) a -= 2 * Math.PI;
      while (a < -Math.PI) a += 2 * Math.PI;
      rates.push((dPrev - dNow) * SCAN_HZ);                // px/s：正 = 在接近
      sumCos += Math.cos(a);
      cnt++;
    }
    if (cnt >= 2) {
      // 用**中位数**而不是均值：最近射线在不同障碍间跳变时均值会被单个离群样本带飞（实测 p50=76m/s）
      rates.sort((a, b) => a - b);
      closing = rates.length % 2 ? rates[(rates.length - 1) >> 1]
                                 : (rates[rates.length / 2 - 1] + rates[rates.length / 2]) / 2;
      // 前车速度 = 自车在该方向的分量 − 接近速率（见文件头的推导）
      const raw = ((car.speed as number) * (sumCos / cnt)) - closing;
      if (raw < LEAD_MIN || raw > LEAD_MAX) {
        leadSpeed = null;                                   // 超物理范围 → 本帧不可用
      } else if (_emaLead !== null && Math.abs(raw - _emaLead) > LEAD_JUMP) {
        leadSpeed = null;                                   // 跳变过大 → 丢弃本帧（门控）
      } else {
        _emaLead = _emaLead === null ? raw : LEAD_ALPHA * raw + (1 - LEAD_ALPHA) * _emaLead;
        leadSpeed = Math.max(0, _emaLead);
      }
    }
  }

  // ③ 记录本次样本（只留前向射线；拷贝一份，别持有扫描内部数组）
  _prevScan = scan;
  _prevIdx = front.slice();
  _prevDist = front.map(i => scan.distances[i]);
  _cacheScan = scan;
  _cacheEst = {
    gap: px(gap) as number,
    leadSpeed: leadSpeed === null ? null : pxps(leadSpeed) as number,
    closing: closing === null ? null : pxps(closing) as number,
  };
  return _cacheEst;
}

/**
 * 是否把前车估计喂给速度层（跟驰）。
 *
 * 【默认关——有实测证据】机制（估计器 + 速度层跟驰分支 + 规划器接线）都已实现并有单测，
 * 但**逐帧差分的估计质量还不足以放行**：
 *   · 不滤波：单帧 1px 的距离抖动 ÷ (1/20s) = 5m/s，实测前车速度 p50 = **76m/s**、p90 = 163m/s（垃圾）；
 *   · 加中位数 + 跳变门控 + 低通后：可信帧只剩 **1%**，且通过的都是 ~46m/s 的饱和值。
 * 根因是**最近射线在不同障碍之间跳变**——逐帧差分对"同一根射线打同一个物体"这个前提不成立。
 * 于是打开它对交通场景**没有改善**（到达 83.5→83.8s、碰撞 12 帧不变）。
 *
 * 【正确的下一步】逐障碍跟踪：把一帧雷达**按距离/角度聚类成物体** → 帧间做**位置匹配** →
 * 对同一个物体跟踪其距离变化率（而不是拿射线下标硬匹配）。那时再默认打开本开关。
 */
export const FOLLOW_ENABLED = false;
