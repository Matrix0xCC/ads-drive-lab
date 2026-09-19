// ==================== Headless：运行归档 ====================
//
// 【目录结构】runs/<id>/{run.json, world.json, frames.jsonl,
//   issues.json, summary.json, report.md}
//
// 【写入策略】frames.jsonl 逐行同步追加——崩溃时已写数据保留；
// 其余文件在 finalize 时一次性写入。索引 index.json 供回放列表使用。

import * as fs from 'node:fs';
import * as path from 'node:path';
import { FrameRecord, Issue, RunMeta, RunSummary, WorldFile, RunsIndexEntry } from '../shared/types.js';

export class ArchiveWriter {
  readonly runDir: string;

  constructor(runDir: string) {
    this.runDir = runDir;
    fs.mkdirSync(runDir, { recursive: true });
  }

  writeMeta(meta: RunMeta): void {
    fs.writeFileSync(path.join(this.runDir, 'run.json'), JSON.stringify(meta, null, 2));
  }

  writeWorld(world: WorldFile): void {
    fs.writeFileSync(path.join(this.runDir, 'world.json'), JSON.stringify(world));
  }

  /** 逐行追加；同步写，失败即抛 */
  appendRecord(rec: FrameRecord): void {
    fs.appendFileSync(path.join(this.runDir, 'frames.jsonl'), JSON.stringify(rec) + '\n');
  }

  finalize(o: { issues: Issue[]; summary: RunSummary; reportMd: string; meta: RunMeta }): void {
    fs.writeFileSync(path.join(this.runDir, 'issues.json'), JSON.stringify(o.issues, null, 2));
    fs.writeFileSync(path.join(this.runDir, 'summary.json'), JSON.stringify(o.summary, null, 2));
    fs.writeFileSync(path.join(this.runDir, 'report.md'), o.reportMd);
    this.writeMeta(o.meta);
  }
}

/** runs/index.json 追加/更新（同 id 覆盖），供回放列表使用 */
export function updateRunsIndex(runsRoot: string, entry: RunsIndexEntry): void {
  const idxPath = path.join(runsRoot, 'index.json');
  let runs: RunsIndexEntry[] = [];
  if (fs.existsSync(idxPath)) {
    try { runs = JSON.parse(fs.readFileSync(idxPath, 'utf8')).runs ?? []; } catch { runs = []; }
  }
  const i = runs.findIndex(r => r.id === entry.id);
  if (i >= 0) runs[i] = entry; else runs.push(entry);
  fs.mkdirSync(runsRoot, { recursive: true });
  fs.writeFileSync(idxPath, JSON.stringify({ runs }, null, 2));
}
