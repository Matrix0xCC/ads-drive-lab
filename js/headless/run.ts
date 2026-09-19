// ==================== Headless：CLI 入口 ====================
//
// 【流程】参数解析 → 场景构建 → engine 装配 → 确定性步进 →
//        订阅主题归档 → 检测 → 报告 → index 更新 → 退出码
//
// 【用法】
//   npm run sim -- --scenario scenarios/oval-avoid-1.json [--seed 7] [--out runs/x]
//   npm run sim -- --map oval --obstacles 4 --seed 42 --duration 60
//
// 【确定性】虚拟时间步进（固定 FIXED_DT），随机性全部来自场景种子；
// 同机器同 Node 版本下同场景同种子 → frames.jsonl 逐字节一致。

import * as fs from 'node:fs';
import * as path from 'node:path';
import { execFileSync } from 'node:child_process';
import process from 'node:process';
import { MessageBus } from '../messageBus.js';
import { SimEngine, FIXED_DT } from '../sim/engine.js';
import { parseScenario, buildScenarioWorld, Scenario } from '../sim/scenario.js';
import { ArchiveWriter, updateRunsIndex } from './archive.js';
import { Detector, DetectorConfig } from './detector.js';
import { verdictOf, buildSummary, buildReportMd } from './report.js';
import { FrameRecord, RunMeta, WorldFile } from '../shared/types.js';

interface CliArgs {
  scenario?: string;
  map?: string;
  obstacles?: number;
  /** undefined = 未显式指定（文件场景保留自身 seed，随机场景用时间戳） */
  seed?: number;
  /** undefined = 未显式指定（场景文件保留自身 duration） */
  duration?: number;
  out?: string;
  failOnIssue: boolean;
}

export function parseArgs(argv: string[]): CliArgs {
  const a: CliArgs = { failOnIssue: false };
  for (let i = 0; i < argv.length; i++) {
    const k = argv[i];
    if (k === '--scenario') a.scenario = argv[++i];
    else if (k === '--map') a.map = argv[++i];
    else if (k === '--obstacles') a.obstacles = Number(argv[++i]);
    else if (k === '--seed') a.seed = Number(argv[++i]);
    else if (k === '--duration') a.duration = Number(argv[++i]);
    else if (k === '--out') a.out = argv[++i];
    else if (k === '--fail-on-issue') a.failOnIssue = true;
    else throw new Error(`未知参数: ${k}`);
  }
  return a;
}

/** 主题载荷 → 归档记录（纯映射；planning 的中间量来自 debug 快照） */
export function recordFrom(topic: string, p: any): FrameRecord {
  switch (topic) {
    case 'control:state':
      return { type: 'step', t: p.t, step: p.step,
        car: { x: p.snapshot.car.x, y: p.snapshot.car.y, angle: p.snapshot.car.angle,
               speed: p.snapshot.car.speed, steerAngle: p.snapshot.car.steerAngle },
        control: p.control };
    case 'perception:result':
      return { type: 'perception', t: p.t, step: p.step,
        scan: { angles: p.scan.angles, distances: p.scan.distances,
                rawDistances: p.scan.rawDistances, confidences: p.scan.confidences },
        stats: p.stats };
    case 'planning:result': {
      const pl = p.planning;
      return { type: 'plan', t: p.t, step: p.step, state: p.state,
        frenet: { s: pl?.frenetS ?? 0, l: pl?.frenetL ?? 0, dl_ds: pl?.dl_ds ?? 0, headingDiff: pl?.headingDiff ?? 0 },
        candidates: pl?.candidates ?? [],
        best: { cost: pl?.bestCost ?? 0, targetL: pl?.bestTargetL ?? 0, targetV: pl?.bestTargetV ?? 0 },
        trajectory: { points: p.trajectory.points.map((pt: any) => ({ t: pt.t, x: pt.x, y: pt.y, v: pt.v })),
                      type: p.trajectory.type, cost: p.trajectory.cost } };
    }
    case 'gps:result':
      return { type: 'gps', t: p.t, step: p.step,
        x: p.reading.x, y: p.reading.y,
        err: Math.hypot(p.reading.x - p.truth.x, p.reading.y - p.truth.y) };
    case 'sim:event':
      return { type: 'event', t: p.t, step: p.step, kind: p.kind, payload: p.payload };
    default:
      throw new Error(`未知主题 ${topic}`);
  }
}

const ARCHIVE_TOPICS = ['control:state', 'perception:result', 'planning:result', 'gps:result', 'sim:event'];

function gitCommitOf(): string | null {
  try { return execFileSync('git', ['rev-parse', '--short', 'HEAD'], { stdio: 'pipe' }).toString().trim(); }
  catch { return null; }
}

function toWorldFile(scenario: Scenario, engine: SimEngine, route: any): WorldFile {
  const pts = (arr: any[]): [number, number][] => arr.map(p => [p.x, p.y]);
  return {
    map: scenario.map,
    track: {
      centerPoints: pts(engine.track.centerPoints),
      innerBoundary: pts(engine.track.innerBoundary),
      outerBoundary: pts(engine.track.outerBoundary),
    },
    obstacles: engine.obstacles.map(o => ({
      x: o.x as number, y: o.y as number, width: o.width as number,
      height: o.height as number, angle: o.angle as number, zHeight: o.zHeight,
    })),
    route: route ? {
      waypoints: pts(route.waypoints), startIndex: route.startIndex,
      goalIndex: route.goalIndex, length: route.length as number,
    } : null,
  };
}

function main(): void {
  const args = parseArgs(process.argv.slice(2));

  // 场景：文件模式或 CLI 随机模式；显式 CLI 参数覆盖场景字段
  let scenario: Scenario;
  if (args.scenario) {
    scenario = parseScenario(JSON.parse(fs.readFileSync(args.scenario, 'utf8')));
    if (args.seed !== undefined) scenario.seed = args.seed;
    if (args.duration !== undefined) scenario.duration = args.duration;
  } else {
    scenario = parseScenario({
      version: 1, name: `cli-${args.map ?? 'oval'}`, map: args.map ?? 'oval',
      seed: args.seed ?? (Date.now() % 1000000), duration: args.duration ?? 60,
      obstacles: { mode: 'random', count: args.obstacles ?? 4 },
    });
  }

  const built = buildScenarioWorld(scenario, 1800, 800);
  const engine = new SimEngine(new MessageBus('headless', false), built.worldCfg);
  if (built.route) {
    engine.setRoute(built.route, built.route.startIndex, { speed: built.worldCfg.carStart?.speed });
  }

  const id = `${new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19)}_${scenario.name}_s${scenario.seed}`;
  const runsRoot = path.resolve('runs');
  const runDir = args.out ? path.resolve(args.out) : path.join(runsRoot, id);
  const archive = new ArchiveWriter(runDir);
  archive.writeWorld(toWorldFile(scenario, engine, built.route));

  // 检测器（阈值可由场景 checks 覆盖；路线信息供超时/停滞检查）
  const det = new Detector(scenario.checks as Partial<DetectorConfig> | undefined, {
    track: engine.track, obstacles: engine.obstacles,
  });
  det.routeActive = built.route !== null;
  det.routeLength = built.route ? (built.route.length as number) : null;

  // 订阅归档 + 检测（同步总线：发布顺序 = 记录顺序）
  for (const topic of ARCHIVE_TOPICS) {
    engine.bus.subscribe(topic, (p: any) => {
      const r = recordFrom(topic, p);
      archive.appendRecord(r);
      det.feed(r);
    });
  }

  // 运行循环：duration 上限；到达后继续 1s（供停车检查）再停
  const maxSteps = Math.floor(scenario.duration * 60);
  let arrivedAtStep: number | null = null;
  engine.bus.subscribe('sim:event', (p: any) => { if (p.kind === 'arrived') arrivedAtStep = p.step; });
  while (engine.stepCount < maxSteps) {
    engine.step(FIXED_DT);
    if (arrivedAtStep !== null && engine.stepCount >= arrivedAtStep + 60) break;
  }

  // 检测 → 报告 → 归档收尾
  const issues = det.finish();
  const { verdict, counts } = verdictOf(issues);
  const summary = buildSummary(det, { steps: engine.stepCount, simTime: engine.simTime });
  const meta: RunMeta = {
    version: 1, id: args.out ? path.basename(runDir) : id,
    scenario: scenario.name, map: scenario.map, seed: scenario.seed,
    durationS: engine.simTime, steps: engine.stepCount,
    gitCommit: gitCommitOf(), nodeVersion: process.version, createdAt: new Date().toISOString(),
    verdict, issueCounts: counts,
  };
  const reportMd = buildReportMd({ meta, issues, summary, scenario });
  archive.finalize({ issues, summary, reportMd, meta });
  if (!args.out) {
    updateRunsIndex(runsRoot, { id: meta.id, scenario: scenario.name, map: scenario.map,
      seed: scenario.seed, createdAt: meta.createdAt, verdict,
      issues: counts, durationS: meta.durationS });
  }
  console.log(`运行完成: ${runDir}  steps=${engine.stepCount}`);
  console.log(`判定: ${verdict} (high=${counts.high} medium=${counts.medium} low=${counts.low})`);
  process.exitCode = args.failOnIssue && verdict === 'ISSUES' ? 1 : 0;
}

main();
