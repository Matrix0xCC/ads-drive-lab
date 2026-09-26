// ==================== 运行归档包（RunBundle）：组装与解析 ====================
//
// 【定位】归档的"单文件容器"形态：与磁盘 runs/<id>/ 目录内容同构
// （meta/world/frames/issues/summary/reportMd）。两处使用：
//   1. headless CLI：运行结束后组装 → 拆写到 runs/<id>/（唯一报告生成点）
//   2. 浏览器导出：运行中收集帧 → 导出时组装 → 下载 .run.json
// 检测与报告在组装时由 headless 同一套模块（Detector/report）产出，
// 浏览器不再维护第二套导出/分析格式。
//
// 【无 node 依赖】浏览器可直接 import。

import { FrameRecord, Issue, RunMeta, RunSummary, WorldFile, TrackGeometry, Obstacle, Route } from '../shared/types.js';
import { Scenario } from '../sim/scenario.js';
import { buildWorldFile } from '../sim/records.js';
import { Detector, DetectorConfig } from './detector.js';
import { verdictOf, buildSummary, buildReportMd } from './report.js';

export interface RunBundle {
  version: 1;
  meta: RunMeta;
  world: WorldFile;
  frames: FrameRecord[];
  issues: Issue[];
  summary: RunSummary;
  reportMd: string;
}

export interface AssembleInput {
  scenario: Scenario;
  track: TrackGeometry;
  obstacles: Obstacle[];
  route: Route | null;
  frames: FrameRecord[];
  simTime: number;
  /** 单帧规划耗时样本（ms；可选——不参与确定性契约，仅进 summary/报告） */
  planMs?: number[];
  id: string;
  createdAt: string;
  gitCommit: string | null;
  nodeVersion: string;
}

/** 组装归档包：真值序列化 → 检测器单遍 → 汇总 → 报告 → meta */
export function assembleBundle(o: AssembleInput): RunBundle {
  const world = buildWorldFile(o.track, o.obstacles, o.scenario, o.route);

  const det = new Detector(o.scenario.checks as Partial<DetectorConfig> | undefined, {
    // 路线一并交给检测器：路网地图的越界判定以路线为基准（见 detector.ts 的 routeFrenet）
    ...(o.route ? { route: { waypoints: o.route.waypoints.map(p => [p.x as number, p.y as number] as [number, number]) } } : {}),
    track: o.track, obstacles: o.obstacles,
  });
  det.routeActive = o.route !== null;
  det.routeLength = o.route ? (o.route.length as number) : null;
  for (const f of o.frames) det.feed(f);

  const issues = det.finish();
  const { verdict, counts } = verdictOf(issues);
  const steps = o.frames.reduce((m, f) => Math.max(m, f.step), 0);
  const summary = buildSummary(det, { steps, simTime: o.simTime, planMs: o.planMs });
  const meta: RunMeta = {
    version: 1, id: o.id, scenario: o.scenario.name, map: o.scenario.map, seed: o.scenario.seed,
    durationS: o.simTime, steps, gitCommit: o.gitCommit, nodeVersion: o.nodeVersion,
    createdAt: o.createdAt, verdict, issueCounts: counts,
  };
  const reportMd = buildReportMd({ meta, issues, summary, scenario: o.scenario });

  return { version: 1, meta, world, frames: o.frames, issues, summary, reportMd };
}

/** 解析导入的归档包（浏览器导入回放路径）；格式非法时抛错 */
export function parseBundle(json: unknown): RunBundle {
  const bad = (why: string): never => { throw new Error(`归档格式无效: ${why}`); };
  if (typeof json !== 'object' || json === null) bad('不是对象');
  const b = json as Record<string, any>;
  if (b.version !== 1) bad(`version 必须为 1（实际 ${b.version}）`);
  if (typeof b.meta !== 'object' || b.meta === null) bad('缺少 meta');
  if (typeof b.world !== 'object' || b.world === null) bad('缺少 world');
  if (!Array.isArray(b.frames)) bad('缺少 frames 数组');
  if (!Array.isArray(b.issues)) bad('缺少 issues 数组');
  return b as unknown as RunBundle;
}
