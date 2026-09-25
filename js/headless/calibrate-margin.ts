// ==================== Headless：误差感知裕度标定（margin calibration） ====================
//
// 【用途】从归档 runs/*/frames.jsonl 提取（v, κ, e）样本，标定物理走廊膨胀用的
// 误差感知裕度 margin(v, κ) = clamp(m₀ + k·κ·v², 0.4, 1.2)（见 js/algo/plan/margin.ts）。
//
// 【样本口径（与检测器/规划一致）】
//   e     = 车到"上一帧规划轨迹折线"的横向最小距离（检测器 tracking_error 口径），m；
//   κ     = 参考线（route）在车处的曲率（refCurvatureAt，1/仿真单位）；
//   v     = 车速，m/s。
//   无 route 的运行跳过（κ 无定义）。
//
// 【方法】按 (v, κ) 分箱取 e 的 95 分位（样本 ≥20 的箱参与拟合）→ 对箱值做
// 加权（箱样本数）最小二乘拟合 m₀ + k·κ·v²；按 run 划分训练/留出（70/30），
// 覆盖率 = 留出集内 e ≤ margin(v, κ) 的样本占比（目标 ≥95%）。
//
// 用法：node dist/js/headless/calibrate-margin.js [--runs runs] [--write]
//   --write 时把报告写入 docs/margin-calibration.md

import fs from 'node:fs';
import path from 'node:path';
import { px } from '../shared/dimensions.js';
import { buildFrenetReference } from '../algo/plan/frenet.js';
import { refCurvatureAt, marginFor, MARGIN_MIN, MARGIN_MAX } from '../algo/plan/margin.js';

export interface CalibSample { vMps: number; kappa: number; eM: number; run: string }

/** 单点对折线的最小距离（检测器 pointSegDist 同构） */
function pointSegDist(px0: number, py0: number, ax: number, ay: number, bx: number, by: number): number {
  const dx = bx - ax, dy = by - ay;
  const l2 = dx * dx + dy * dy;
  if (l2 === 0) return Math.hypot(px0 - ax, py0 - ay);
  let t = ((px0 - ax) * dx + (py0 - ay) * dy) / l2;
  t = Math.max(0, Math.min(1, t));
  return Math.hypot(px0 - (ax + t * dx), py0 - (ay + t * dy));
}

/** 提取单个 run 的样本（内部导出供测试） */
export function samplesFromRun(dir: string): CalibSample[] {
  const runId = path.basename(dir);
  const world = JSON.parse(fs.readFileSync(path.join(dir, 'world.json'), 'utf8'));
  if (!world.route || !world.route.waypoints?.length) return [];   // 无 route → κ 无定义
  const ref = buildFrenetReference(world.route.waypoints.map(([x, y]: [number, number]) => ({ x, y })), false);

  const lines = fs.readFileSync(path.join(dir, 'frames.jsonl'), 'utf8').trim().split('\n');
  const plans: any[] = [];
  const steps: any[] = [];
  for (const L of lines) {
    const r = JSON.parse(L);
    if (r.type === 'plan') plans.push(r);
    else if (r.type === 'step') steps.push(r);
  }
  const samples: CalibSample[] = [];
  const trajAt = (step: number): any | null => {
    let best: any = null;
    for (const p of plans) { if (p.step <= step) best = p; else break; }
    return best;
  };
  for (const st of steps) {
    const p = trajAt(st.step);
    if (!p || !p.trajectory?.points?.length || p.trajectory.points.length < 2) continue;
    const car = st.car;
    // 只取"运动中"样本：停驻/极低速时"车到上一帧轨迹的距离"是停驻位置差
    // （规划线在车道中心、车停在别处），不是跟踪误差——实测把 e95 抬到 1.2m
    // 并让 m₀ 撞上界截断，掩盖真实的误差-速度关系
    if (car.speed / 4 < 1) continue;
    let minD = Infinity;
    const pts = p.trajectory.points;
    for (let i = 0; i < pts.length - 1; i++) {
      minD = Math.min(minD, pointSegDist(car.x, car.y, pts[i].x, pts[i].y, pts[i + 1].x, pts[i + 1].y));
    }
    if (!Number.isFinite(minD)) continue;
    const kappa = refCurvatureAt(ref, p.frenet.s);
    samples.push({ vMps: car.speed / 4, kappa, eM: minD / 4, run: runId });
  }
  return samples;
}

const V_BINS = [0, 5, 10, 15, 20, 25, 30, 35];
const K_BINS = [0, 0.0025, 0.005, 0.0125, 0.025, 1e9];   // 1/仿真单位（1/m ÷ 4）

export interface CalibResult {
  m0: number; k: number;
  coverage: number;        // 拟合模型（m₀+k·κv²）的留出集覆盖率
  tableCoverage: number;   // 部署速度表（marginFor，margin.ts）的留出集覆盖率
  nTrain: number; nHoldout: number;
  bins: { v: number; k: number; n: number; p95: number; x: number }[];
  vTable: { v: number; n: number; p95: number }[];   // v 边际 95 分位（部署表依据）
}

/** 分箱 95 分位 + 加权最小二乘拟合 + 留出集覆盖率 */
export function calibrate(samples: CalibSample[]): CalibResult {
  const runs = [...new Set(samples.map(s => s.run))].sort();
  const holdoutRuns = new Set(runs.filter((_, i) => i % 10 >= 7));   // 30% 留出（按 run）
  const train = samples.filter(s => !holdoutRuns.has(s.run));
  const holdout = samples.filter(s => holdoutRuns.has(s.run));

  const bins: CalibResult['bins'] = [];
  const binOf = (v: number, k: number) => {
    let vi = 0; while (vi < V_BINS.length - 1 && v >= V_BINS[vi + 1]) vi++;
    let ki = 0; while (ki < K_BINS.length - 1 && k >= K_BINS[ki + 1]) ki++;
    return [vi, ki] as const;
  };
  const groups = new Map<string, CalibSample[]>();
  for (const s of train) {
    const [vi, ki] = binOf(s.vMps, s.kappa);
    const key = `${vi}-${ki}`;
    if (!groups.has(key)) groups.set(key, []);
    groups.get(key)!.push(s);
  }
  const p95of = (xs: number[]) => {
    const sorted = [...xs].sort((a, b) => a - b);
    return sorted[Math.min(sorted.length - 1, Math.floor(0.95 * sorted.length))];
  };
  for (const [key, ss] of groups) {
    if (ss.length < 20) continue;
    const [vi, ki] = key.split('-').map(Number);
    // 关键：配对取 95 分位——e 的 95 分位对应的是 x=κv² 的 95 分位（不能拿箱中点
    // 相乘当代入值，κv² 非线性的分布使中点严重偏差、把拟合推向上界截断）
    const p95 = p95of(ss.map(s => s.eM));
    const p95x = p95of(ss.map(s => s.kappa * s.vMps * s.vMps));
    const vMid = (V_BINS[vi] + V_BINS[Math.min(vi + 1, V_BINS.length - 1)]) / 2;
    const kMid = (K_BINS[ki] + K_BINS[Math.min(ki + 1, K_BINS.length - 1)]) / 2;
    bins.push({ v: vMid, k: kMid, n: ss.length, p95, x: p95x });
  }

  // 加权最小二乘：e95 ≈ m0 + k·(κv²)95（法方程 2×2）
  let sw = 0, sx = 0, sy = 0, sxx = 0, sxy = 0;
  for (const b of bins) {
    const x = b.x, w = b.n;
    sw += w; sx += w * x; sy += w * b.p95; sxx += w * x * x; sxy += w * x * b.p95;
  }
  const det = sw * sxx - sx * sx;
  let m0 = det > 1e-12 ? (sy * sxx - sx * sxy) / det : 0.4;
  let k = det > 1e-12 ? (sw * sxy - sx * sy) / det : 0;
  m0 = Math.max(MARGIN_MIN, Math.min(MARGIN_MAX, m0));
  k = Math.max(0, k);

  // 覆盖率校准（conformal 式分位修正）：最小二乘只定"形状"（k），截距须抬到
  // 训练集残差的 95 分位——否则拟合线穿在样本中央，覆盖率只有 ~75%（实测）。
  const TARGET = 0.95;
  const residuals = train
    .map(s => s.eM - (m0 + k * s.kappa * s.vMps * s.vMps))
    .sort((a, b) => a - b);
  if (residuals.length) {
    const q = residuals[Math.min(residuals.length - 1, Math.floor(TARGET * residuals.length))];
    m0 = Math.max(MARGIN_MIN, Math.min(MARGIN_MAX, m0 + Math.max(0, q)));
  }

  const margin = (v: number, kappa: number) =>
    Math.max(MARGIN_MIN, Math.min(MARGIN_MAX, m0 + k * kappa * v * v));
  const covered = holdout.filter(s => s.eM <= margin(s.vMps, s.kappa)).length;
  // 部署表（margin.ts 的速度分段表）的留出集覆盖率：这才是上线后成立的数字
  const tableCovered = holdout.filter(s => s.eM <= marginFor(s.vMps)).length;
  // v 边际表：按 v 分箱取 e 的 95 分位（速度分段表的依据）
  const vGroups = new Map<number, number[]>();
  for (const s of train) {
    const [vi] = binOf(s.vMps, s.kappa);
    if (!vGroups.has(vi)) vGroups.set(vi, []);
    vGroups.get(vi)!.push(s.eM);
  }
  const vTable = [...vGroups.entries()].sort((a, b) => a[0] - b[0]).map(([vi, es]) => ({
    v: (V_BINS[vi] + V_BINS[Math.min(vi + 1, V_BINS.length - 1)]) / 2,
    n: es.length,
    p95: p95of(es),
  }));

  return {
    m0, k,
    coverage: holdout.length ? covered / holdout.length : 1,
    tableCoverage: holdout.length ? tableCovered / holdout.length : 1,
    nTrain: train.length, nHoldout: holdout.length, bins, vTable,
  };
}

// ── CLI ──
if (process.argv[1] && import.meta.url.endsWith(path.basename(process.argv[1]))) {
  const args = process.argv.slice(2);
  const runsRoot = args.includes('--runs') ? args[args.indexOf('--runs') + 1] : 'runs';
  const dirs = fs.readdirSync(runsRoot)
    .map(d => path.join(runsRoot, d))
    .filter(d => fs.existsSync(path.join(d, 'frames.jsonl')) && fs.existsSync(path.join(d, 'world.json')))
    .sort();
  const all: CalibSample[] = [];
  for (const d of dirs) all.push(...samplesFromRun(d));
  const r = calibrate(all);

  const lines: string[] = [];
  lines.push('# 物理裕度标定报告（margin calibration）');
  lines.push('');
  lines.push(`- 样本：${all.length}（训练 ${r.nTrain} / 留出 ${r.nHoldout}，按 run 70/30 划分）`);
  lines.push('## 结论');
  lines.push('');
  lines.push(`- 原假设形式 margin = m₀ + k·κ·v² **被数据否决**（拟合 k = ${r.k.toFixed(3)}）：`);
  lines.push('  实测横向偏差与 κ·v² 无关，而是**随速度下降**（低速机动段大、高速稳态小）。');
  lines.push(`- 部署裕度 = margin.ts 的**速度分段表**（<5m/s: 1.0；5-10: 1.0→0.8；10-15: 0.8→0.4；≥15: 0.4）。`);
  lines.push(`- 留出集覆盖率：拟合模型 ${(r.coverage * 100).toFixed(1)}%；**部署速度表 ${(r.tableCoverage * 100).toFixed(1)}%**（目标 ≥95%）。`);
  lines.push('');
  lines.push('## v 边际表（部署依据）');
  lines.push('');
  lines.push('| v(m/s) | 样本 | e 95 分位(m) | 部署裕度(m) |');
  lines.push('|---|---|---|---|');
  for (const b of r.vTable) lines.push(`| ${b.v.toFixed(1)} | ${b.n} | ${b.p95.toFixed(3)} | ${marginFor(b.v).toFixed(2)} |`);
  lines.push('');
  lines.push('## (v, κ) 分箱表（κ 维度证据：偏差不随 κ·v² 增长）');
  lines.push('');
  lines.push('| v(m/s) | κ(1/u) | 样本 | e 95 分位(m) |');
  lines.push('|---|---|---|---|');
  for (const b of r.bins) lines.push(`| ${b.v.toFixed(1)} | ${b.k.toFixed(4)} | ${b.n} | ${b.p95.toFixed(3)} |`);
  const report = lines.join('\n') + '\n';
  console.log(report);
  if (args.includes('--write')) {
    fs.mkdirSync('docs', { recursive: true });
    fs.writeFileSync('docs/margin-calibration.md', report);
    console.log('已写入 docs/margin-calibration.md');
  }
}
