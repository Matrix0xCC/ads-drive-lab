// ==================== Headless：报告生成 ====================
//
// 【三层产物】
//   issues.json  — 结构化问题（机器消费）
//   summary.json — 聚合指标（机器消费）
//   report.md    — 摘要报告（AI/人第一个读的文件，token 友好，含钻取指引）

import { Issue, RunMeta, RunSummary } from '../shared/types.js';
import { Detector } from './detector.js';
import { Scenario } from '../sim/scenario.js';
import { FIXED_DT } from '../sim/engine.js';

export function verdictOf(issues: Issue[]): {
  verdict: 'PASS' | 'ISSUES';
  counts: { high: number; medium: number; low: number };
} {
  const counts = { high: 0, medium: 0, low: 0 };
  for (const i of issues) counts[i.severity] += 1;
  return { verdict: issues.length > 0 ? 'ISSUES' : 'PASS', counts };
}

/** 从检测器累积量产出聚合指标（速度/RMSE/最小障碍距离/状态计数/事件） */
export function buildSummary(det: Detector, engineInfo: { steps: number; simTime: number }): RunSummary {
  const avg = det.speedN > 0 ? det.speedSum / det.speedN : 0;
  return {
    steps: engineInfo.steps,
    simSeconds: engineInfo.simTime,
    speed: { maxMps: det.speedMax / 4, avgMps: avg / 4 },
    arrivedAtStep: det.arrivedAtStep,
    // 到达用时：评估"到达质量"的效率维度——仅看到达率时，更慢但更安全的改动
    // （如紧间隙限速下调）的代价不可见（批次 A/B 的教训）
    arrivalTimeS: det.arrivedAtStep !== null ? det.arrivedAtStep * FIXED_DT : null,
    // 路线进度：未到达的运行也给出 0..1 的完成度（停在 30% 与 90% 才可区分）
    progress: det.routeLength !== null && det.routeLength > 0
      ? Math.min(1, det.maxFrenetS / det.routeLength) : null,
    trackingRmseM: det.trackingErrN > 0 ? Math.sqrt(det.trackingErrSum / det.trackingErrN) / 4 : null,
    minObstacleDistM: Number.isFinite(det.minObstacleDist) ? det.minObstacleDist / 4 : null,
    stateCounts: det.stateCounts,
    emergencyBrakes: det.emergencyBrakeCount,
    boundsResets: det.boundsResetCount,
  };
}

export function buildReportMd(o: {
  meta: RunMeta; issues: Issue[]; summary: RunSummary; scenario: Scenario;
}): string {
  const { meta, issues, summary } = o;
  const lines: string[] = [];
  lines.push(`# 运行报告 ${meta.id}`);
  lines.push('');
  lines.push(`**判定: ${meta.verdict}**  |  场景 ${meta.scenario}  |  地图 ${meta.map}  |  种子 ${meta.seed}  |  ` +
    `时长 ${summary.simSeconds.toFixed(1)}s  |  commit ${meta.gitCommit ?? 'n/a'}  |  Node ${meta.nodeVersion}`);
  lines.push('');
  lines.push('## 问题列表');
  if (issues.length === 0) lines.push('（无）');
  for (const i of issues) {
    lines.push(`- **[${i.severity}][${i.category}] ${i.check}** 帧 ${i.firstFrame}–${i.lastFrame}（${i.count} 帧）: ${i.summary}`);
  }
  lines.push('');
  lines.push('## 关键指标');
  lines.push('| 指标 | 值 |');
  lines.push('|---|---|');
  lines.push(`| 最大/平均速度 | ${summary.speed.maxMps.toFixed(1)} / ${summary.speed.avgMps.toFixed(1)} m/s |`);
  lines.push(`| 到达用时 | ${summary.arrivalTimeS != null ? `${summary.arrivalTimeS.toFixed(1)} s（step ${summary.arrivedAtStep}）` : '未到达'} |`);
  lines.push(`| 路线进度 | ${summary.progress != null ? `${(summary.progress * 100).toFixed(0)}%` : 'n/a'} |`);
  lines.push(`| 交叉跟踪 RMSE | ${summary.trackingRmseM?.toFixed(3) ?? 'n/a'} m |`);
  lines.push(`| 最近障碍距离 | ${summary.minObstacleDistM?.toFixed(2) ?? 'n/a'} m |`);
  lines.push(`| 状态分布 | ${Object.entries(summary.stateCounts).map(([k, v]) => `${k}:${v}`).join(' ')} |`);
  lines.push(`| 紧急制动 / 越界复位 | ${summary.emergencyBrakes} / ${summary.boundsResets} |`);
  lines.push('');
  lines.push('## 数据钻取');
  lines.push('```bash');
  lines.push(`grep '"type":"plan"' frames.jsonl | sed -n '<规划帧序号>p'   # 看某帧规划`);
  lines.push(`grep '"type":"event"' frames.jsonl                              # 全部离散事件`);
  lines.push(`grep '"step":431' frames.jsonl                                  # 按 step 取记录`);
  lines.push('```');
  return lines.join('\n');
}
