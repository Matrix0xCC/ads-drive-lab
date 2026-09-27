// ==================== 算法：物体层（聚类 → 跟踪 → 带速度的物体列表） ====================
//
// 【为什么需要这一层】规划器原来直接对**射线距离**做决策，于是两件事都做不好：
//   ① 测速：拿"同一根射线的距离差分"估前车速度 —— 最近射线会在障碍之间跳变，
//      实测前车速度 p50 = 76m/s（垃圾）；加滤波后可信帧只剩 1%。
//   ② 归属：分不清"我这条车道上的东西"和"旁边车道的车"，于是旁边超车就减速/横移。
// 根因是**射线不是物体**：射线下标不是物体身份。所以先把一帧雷达变成**物体**，再跟踪成**轨迹**。
//
// 【管线】
//   一帧扫描 → ① 按射线相邻性聚类（同一传感器内，距离/横向连续）→ 物体（含 Frenet 位置与尺度）
//            → ② 帧间关联（按预测位置门控匹配，**不是**按下标）→ ③ α-β 滤波更新速度/位置
//            → ④ 出生/消亡管理（连续命中才确认，连续丢失才删除）
//
// 【量纲】s/l 是 px（Frenet），速度 px/s，时间 s。
//
// 【确定性】纯函数 + 固定扫描周期；跟踪器状态在模块内，但**按世界键重置**（换地图/重载场景必须清空，
// 否则会拿上一个世界的轨迹去关联 —— closing.ts 上已经用血换过这条教训）。不使用墙钟。

import { LidarScan } from '../shared/types.js';
import { px, m, mps } from '../shared/dimensions.js';

/** 一个物体（单帧检测结果） */
export interface DetectedObject {
  /** Frenet 弧长（px）——用**最近那条射线**的 s（近端面，适合算"还有多远"） */
  s: number;
  /** Frenet 横向位置（px）——簇内均值 */
  l: number;
  /** 沿路方向的尺度（px；单射线时取车辆默认长） */
  length: number;
  /** 横向尺度（px；单射线时取车辆默认宽） */
  width: number;
  /** 这个物体被几条射线看到（置信度的一个代理） */
  rays: number;
  /** 主要由哪个传感器看到（Scan.sensorOf） */
  sensor: number;
}

/** 一条轨迹（跨帧跟踪结果） */
export interface ObjectTrack extends DetectedObject {
  /** 稳定 id（同一物体跨帧保持一致） */
  id: number;
  /** 沿路速度（px/s，正 = 沿参考线向前） */
  vS: number;
  /** 横向速度（px/s，正 = 向左） */
  vL: number;
  /** 连续命中帧数 */
  hits: number;
  /** 连续丢失帧数 */
  missed: number;
  /** 是否已确认（连续命中 ≥ CONFIRM_HITS） */
  confirmed: boolean;
}

/** 跟踪参数（单测与调参都看这里） */
export const OBJECT = {
  /** 同一簇内允许的角间隔（rad）：超过就断开 */
  CLUSTER_GAP_ANG: (8 * Math.PI) / 180,
  /** 同一簇内允许的距离跳变：max(绝对, 相对×距离) */
  CLUSTER_JUMP_ABS: m(0.8) as number,
  CLUSTER_JUMP_REL: 0.3,
  /** 同一簇内允许的横向跳变 */
  CLUSTER_JUMP_L: m(2.0) as number,
  /** 关联门限：位置残差 = min(绝对 + 速度×dt×系数, 上限) */
  ASSOC_ABS: m(3.5) as number,
  ASSOC_SPEED_GAIN: 2.5,
  ASSOC_MAX: m(12) as number,
  /** 关联门限（横向） */
  ASSOC_L: m(2.5) as number,
  /** α-β 滤波：位置/速度增益 */
  ALPHA: 0.55,
  BETA: 0.25,
  /** 速度更新的物理门限（超过就只更位置、不更速度：抗跳变）。
   *  高速公路场景下"物体相对地面"的速度上限取 40m/s（144km/h）——比它更离谱的测量一定是
   *  关联错配/簇合并，不能进入状态。**并且更新后再硬夹一次**（见 stepTracks 末尾）：
   *  宁可报一个被夹住的值，也不能把一个幻想值交给规划器。 */
  V_MAX: mps(40) as number,
  /** 出生/消亡 */
  CONFIRM_HITS: 2,
  MAX_MISSED: 4,
  /** 单射线物体（远处一辆车）的默认尺度 */
  DEFAULT_LEN: m(4.6) as number,
  DEFAULT_WID: m(2.0) as number,
};

/**
 * 一帧扫描 → 物体列表（纯函数，无状态）。
 * 只在**同一传感器内**聚类（不同传感器的坐标系/量程不同，跨传感器合并由融合层做 —— 见下一步）。
 */
export function detectObjects(scan: LidarScan): DetectedObject[] {
  const hs = scan.hitS, hl = scan.hitL, so = scan.sensorOf;
  if (!hs || !hl) return [];                       // 老归档/合成扫描没有 Frenet 位置 → 不产生物体
  const n = Math.min(hs.length, hl.length);
  const out: DetectedObject[] = [];
  let cur: { s: number[]; l: number[]; d: number[]; sensor: number; } | null = null;

  const flush = () => {
    if (!cur) return;
    const sNear = cur.s[0];                        // 第一条射线的 s 最小（射线按角度扫，最近的先记）
    let lSum = 0, sMin = Infinity, sMax = -Infinity, lMin = Infinity, lMax = -Infinity;
    for (let k = 0; k < cur.s.length; k++) {
      lSum += cur.l[k];
      if (cur.s[k] < sMin) sMin = cur.s[k];
      if (cur.s[k] > sMax) sMax = cur.s[k];
      if (cur.l[k] < lMin) lMin = cur.l[k];
      if (cur.l[k] > lMax) lMax = cur.l[k];
    }
    out.push({
      s: sMin,                                     // 近端面：算"还有多远"用它
      l: lSum / cur.s.length,
      length: Math.max(OBJECT.DEFAULT_LEN, sMax - sMin),
      width: Math.max(OBJECT.DEFAULT_WID, lMax - lMin),
      rays: cur.s.length,
      sensor: cur.sensor,
    });
    cur = null;
  };

  for (let i = 0; i < n; i++) {
    const s = hs[i], l = hl[i];
    if (!Number.isFinite(s) || !Number.isFinite(l)) { flush(); continue; }   // 无命中/路缘 → 断开
    const d = scan.distances[i] as number;
    const sen = so ? so[i] : 0;
    if (cur && cur.sensor === sen) {
      const dl = Math.abs(l - cur.l[cur.l.length - 1]);
      const dd = Math.abs(d - cur.d[cur.d.length - 1]);
      const jump = Math.max(OBJECT.CLUSTER_JUMP_ABS, OBJECT.CLUSTER_JUMP_REL * d);
      if (dl <= OBJECT.CLUSTER_JUMP_L && dd <= jump) {
        cur.s.push(s); cur.l.push(l); cur.d.push(d);
        continue;
      }
    }
    flush();
    cur = { s: [s], l: [l], d: [d], sensor: sen };
  }
  flush();
  return out;
}

// ─────────────────────────────────────────────────────────────
// 跟踪器（模块内状态；按世界键重置）
// ─────────────────────────────────────────────────────────────
let _tracks: ObjectTrack[] = [];
let _nextId = 1;
let _prevKey: unknown = null;
let _prevScan: unknown = null;
let _cacheKey: unknown = null;
let _cache: ObjectTrack[] = [];

/** 重置（换世界/回放/重载场景时调用） */
export function resetObjects(): void {
  _tracks = []; _nextId = 1; _prevKey = null; _prevScan = null; _cacheKey = null; _cache = [];
}

/** 读取当前轨迹（只读，供调试/规划用） */
export function getTracks(): ObjectTrack[] { return _tracks; }

/**
 * 帧间跟踪：一步。
 *
 * @param tracks 上一帧的轨迹（就地更新并返回；调用方通常用 trackedObjects 的组合入口）
 * @param dets   本帧检测物体
 * @param dt     帧间隔（s；由**固定扫描周期**给出，不用墙钟）
 */
export function stepTracks(tracks: ObjectTrack[], dets: DetectedObject[], dt: number): ObjectTrack[] {
  // ① 预测
  for (const t of tracks) { t.s += t.vS * dt; t.l += t.vL * dt; }

  // ② 关联：按位置残差贪心（残差小者优先），门限随速度放宽（确定性：按 id 顺序遍历）
  const free = dets.map((_, i) => i);
  const pairs: { t: number; d: number; cost: number }[] = [];
  for (let ti = 0; ti < tracks.length; ti++) {
    for (const di of free) {
      const det = dets[di], t = tracks[ti];
      const ds = det.s - t.s;
      const dl = det.l - t.l;
      const gate = Math.min(OBJECT.ASSOC_MAX, OBJECT.ASSOC_ABS + OBJECT.ASSOC_SPEED_GAIN * Math.hypot(t.vS, t.vL) * dt);
      if (Math.abs(ds) > gate || Math.abs(dl) > OBJECT.ASSOC_L) continue;
      pairs.push({ t: ti, d: di, cost: Math.hypot(ds, dl) });
    }
  }
  pairs.sort((a, b) => a.cost - b.cost || a.t - b.t || a.d - b.d);
  const usedT = new Set<number>(), usedD = new Set<number>();
  const matched: { ti: number; di: number }[] = [];
  for (const p of pairs) {
    if (usedT.has(p.t) || usedD.has(p.d)) continue;
    usedT.add(p.t); usedD.add(p.d); matched.push({ ti: p.t, di: p.d });
  }

  // ③ 更新（α-β）：位置用残差校正，速度用"测量速度"，超过物理门限就只更位置
  for (const { ti, di } of matched) {
    const t = tracks[ti], det = dets[di];
    const resS = det.s - t.s, resL = det.l - t.l;
    const vMeasS = resS / Math.max(1e-6, dt), vMeasL = resL / Math.max(1e-6, dt);
    t.s += OBJECT.ALPHA * resS;
    t.l += OBJECT.ALPHA * resL;
    if (Math.hypot(vMeasS, vMeasL) <= (OBJECT.V_MAX as number)) {
      t.vS += OBJECT.BETA * vMeasS;
      t.vL += OBJECT.BETA * vMeasL;
    }
    // 硬夹：α-β 的中间量仍可能越界（连续几帧同向偏置）→ 保证**对外报出的速度永远物理合理**
    const vMag = Math.hypot(t.vS, t.vL);
    if (vMag > (OBJECT.V_MAX as number)) {
      const k = (OBJECT.V_MAX as number) / vMag;
      t.vS *= k; t.vL *= k;
    }
    // 尺度用本次观测（平滑一点：与旧的取平均）
    t.length = 0.5 * (t.length + det.length);
    t.width = 0.5 * (t.width + det.width);
    t.rays = det.rays; t.sensor = det.sensor;
    t.hits += 1; t.missed = 0;
    t.confirmed = t.hits >= OBJECT.CONFIRM_HITS;
  }

  // ④ 未匹配：丢失计数；匹配上的但没被任何轨迹用到的检测 → 新轨迹
  for (let ti = 0; ti < tracks.length; ti++) if (!usedT.has(ti)) tracks[ti].missed += 1;
  for (const di of free) {
    if (usedD.has(di)) continue;
    const det = dets[di];
    tracks.push({
      ...det, id: _nextId++, vS: 0, vL: 0, hits: 1, missed: 0, confirmed: false,
    });
  }

  // ⑤ 消亡 + 稳定排序（按 s 升序，确定性）
  _tracks = tracks
    .filter(t => t.missed <= OBJECT.MAX_MISSED)
    .sort((a, b) => (a.s - b.s) || (a.id - b.id));
  return _tracks;
}

/**
 * 组合入口：喂一帧扫描，返回当前轨迹（供规划器/调试用）。
 * @param key 世界键（传 `snapshot.track`）：变了就清空历史
 */
export function trackedObjects(scan: LidarScan, dt: number, key?: unknown): ObjectTrack[] {
  if (key !== undefined && key !== _prevKey) { resetObjects(); _prevKey = key; }
  if (_prevScan === scan) return _cache;              // 同一帧扫描被调用多次（感知 20Hz / 规划 15Hz）→ 复用
  const dets = detectObjects(scan);
  _prevScan = scan;
  _cache = stepTracks(_tracks, dets, dt);
  _cacheKey = key;
  return _cache;
}

/** 帧间隔：由**固定扫描周期**给出（与引擎 PERCEPTION_HZ 一致），不用墙钟 */
export const OBJECT_DT = 1 / 20;

/** 便于测试/调试：把 px 换算成米 */
export function toMeters(v: number): number { return v / 4; }
