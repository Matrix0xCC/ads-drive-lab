// ==================== Headless：CLI 入口 ====================
//
// 【流程】参数解析 → 场景构建 → engine 装配 → 确定性步进 →
//        订阅主题归档（流式写盘）+ 收集帧 → assembleBundle（检测+报告）→
//        index 更新 → 退出码
//
// 【用法】
//   npm run sim -- --scenario scenarios/oval-avoid-1.json [--seed 7] [--out runs/x]
//   npm run sim -- --map oval --obstacles 4 --seed 42 --duration 60
//
// 【确定性】虚拟时间步进（固定 FIXED_DT），随机性全部来自场景种子；
// 同机器同 Node 版本下同场景同种子 → frames.jsonl 逐字节一致。
//
// 【与浏览器同源】记录映射（recordFrom）、真值序列化（buildWorldFile）、
// 检测与报告组装（assembleBundle）均来自共享模块——浏览器导出运行归档时
// 走的是完全相同的一套。

import * as fs from 'node:fs';
import * as path from 'node:path';
import { execFileSync } from 'node:child_process';
import process from 'node:process';
import { MessageBus } from '../messageBus.js';
import { SimEngine, FIXED_DT } from '../sim/engine.js';
import { parseScenario, buildScenarioWorld, Scenario } from '../sim/scenario.js';
import { ARCHIVE_TOPICS, recordFrom, buildWorldFile } from '../sim/records.js';
import { ArchiveWriter, updateRunsIndex } from './archive.js';
import { assembleBundle } from './bundle.js';
import { FrameRecord } from '../shared/types.js';

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

function gitCommitOf(): string | null {
  try { return execFileSync('git', ['rev-parse', '--short', 'HEAD'], { stdio: 'pipe' }).toString().trim(); }
  catch { return null; }
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
  archive.writeWorld(buildWorldFile(engine.track, engine.obstacles, scenario, built.route));

  // 订阅归档：流式写盘（崩溃保留已有数据）+ 收集帧（运行后组装检测/报告）
  const frames: FrameRecord[] = [];
  for (const topic of ARCHIVE_TOPICS) {
    engine.bus.subscribe(topic, (p: any) => {
      const r = recordFrom(topic, p);
      archive.appendRecord(r);
      frames.push(r);
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

  // 检测 + 报告 + 归档收尾（与浏览器导出共用 assembleBundle）
  const bundle = assembleBundle({
    scenario, track: engine.track, obstacles: engine.obstacles, route: built.route,
    frames, simTime: engine.simTime, planMs: engine.planMsSamples,
    id: args.out ? path.basename(runDir) : id,
    createdAt: new Date().toISOString(),
    gitCommit: gitCommitOf(), nodeVersion: process.version,
  });
  archive.finalize({
    issues: bundle.issues, summary: bundle.summary, reportMd: bundle.reportMd, meta: bundle.meta,
  });
  if (!args.out) {
    updateRunsIndex(runsRoot, { id: bundle.meta.id, scenario: scenario.name, map: scenario.map,
      seed: scenario.seed, createdAt: bundle.meta.createdAt, verdict: bundle.meta.verdict,
      issues: bundle.meta.issueCounts, durationS: bundle.meta.durationS });
  }
  console.log(`运行完成: ${runDir}  steps=${engine.stepCount}`);
  console.log(`判定: ${bundle.meta.verdict} (high=${bundle.meta.issueCounts.high} medium=${bundle.meta.issueCounts.medium} low=${bundle.meta.issueCounts.low})`);
  process.exitCode = args.failOnIssue && bundle.meta.verdict === 'ISSUES' ? 1 : 0;
}

main();
