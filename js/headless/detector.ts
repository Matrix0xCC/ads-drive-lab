// ==================== Headless：问题检测器（核心） ====================
//
// 【形态】纯模块：feed(记录流) 累积 → finish() 产出 Issue[]。
// 流式单遍、无 I/O；可用合成记录做单元测试。
//
// 【issue 合并】同一 check 的离散命中（间隔 ≤ mergeGapSteps）合并为
// 一个 issue 区间（firstFrame..lastFrame + count + samples）。
//
// 【检查分发】按记录类型调用各检查模块：
//   checks/safety.ts     碰撞 / 越界 / EMERGENCY
//   checks/perception.ts 幻影 / 漏检 / GPS
//   checks/planning.ts   状态抖动 / 横向摆动 / 近距险情
//   checks/mission.ts    跟踪误差 / 停滞 / 停车异常 / 超时

import { FrameRecord, Issue, Obstacle, TrackGeometry, FrenetReference } from '../shared/types.js';
import { Px, PxPerSec, m, mps } from '../shared/dimensions.js';
import { buildFrenetReference } from '../algo/plan/frenet.js';
import { runSafetyChecks } from './checks/safety.js';
import { runPerceptionChecks } from './checks/perception.js';
import { runPlanningChecks } from './checks/planning.js';
import { runMissionChecks, finalizeMissionChecks } from './checks/mission.js';

export interface DetectorConfig {
  // 安全（阈值由检查模块各自使用）
  // 感知
  gpsSigma: Px;              // GPS 标称 σ
  gpsThresholdSigma: number; // 超限倍数
  phantomTolerance: Px;      // 幻影命中容差
  phantomRate: number;       // 幻影率告警阈值
  // 规划行为
  closeCallDist: Px;
  closeCallSpeed: PxPerSec;
  flappingWindowSteps: number;
  flappingCount: number;
  lateralJump: Px;
  lateralWindowPlans: number;
  lateralCount: number;
  // 跟踪/任务
  trackingErr: Px;
  stallSpeed: PxPerSec;
  stallSteps: number;
  arriveSpeed: PxPerSec;
  // 内部
  mergeGapSteps: number;
}

export const DEFAULT_DETECTOR_CONFIG: DetectorConfig = {
  gpsSigma: m(1.25), gpsThresholdSigma: 3,
  phantomTolerance: m(1.0), phantomRate: 0.01,
  closeCallDist: m(5), closeCallSpeed: mps(5),
  flappingWindowSteps: 60, flappingCount: 4,
  lateralJump: m(10), lateralWindowPlans: 15, lateralCount: 3,
  trackingErr: m(2.0), stallSpeed: mps(0.5), stallSteps: 180, arriveSpeed: mps(1),
  mergeGapSteps: 30,
};

export interface DetectorWorld { track: TrackGeometry; obstacles: Obstacle[]; }

/** 区间合并器：同 check 的离散命中 → 连续区间 issue */
export class IssueMerger {
  private _open = new Map<string, { issue: Issue; lastStep: number }>();
  private _closed: Issue[] = [];

  constructor(private cfg: DetectorConfig) {}

  add(p: {
    check: string; category: Issue['category']; severity: Issue['severity'];
    step: number; summary: string; details?: Record<string, unknown>;
  }): void {
    const cur = this._open.get(p.check);
    if (cur && p.step - cur.lastStep <= this.cfg.mergeGapSteps) {
      cur.issue.lastFrame = p.step;
      cur.issue.count += 1;
      if (cur.issue.samples.length < 3 && !cur.issue.samples.includes(p.step)) {
        cur.issue.samples.push(p.step);
      }
      cur.lastStep = p.step;
    } else {
      if (cur) this._closed.push(cur.issue);
      this._open.set(p.check, { issue: this._newIssue(p), lastStep: p.step });
    }
  }

  private _newIssue(p: {
    check: string; category: Issue['category']; severity: Issue['severity'];
    step: number; summary: string; details?: Record<string, unknown>;
  }): Issue {
    return {
      category: p.category, severity: p.severity, check: p.check,
      firstFrame: p.step, lastFrame: p.step, count: 1,
      summary: p.summary, details: p.details ?? {}, samples: [p.step],
    };
  }

  finish(): Issue[] {
    return [...this._closed, ...[...this._open.values()].map(v => v.issue)];
  }
}

export class Detector {
  readonly cfg: DetectorConfig;
  readonly world: DetectorWorld;
  readonly merger: IssueMerger;
  readonly trackFrenet: FrenetReference;

  // —— 跨帧累积状态（各检查模块共享） ——
  lastStepRec: any = null;
  lastPlanRec: any = null;
  stepCount = 0;
  // 规划行为
  stateChangeSteps: number[] = [];
  lateralJumps: { step: number; jump: number }[] = [];
  lastPlanTargetL: number | null = null;
  // 任务
  routeActive = false;
  routeLength: number | null = null;
  stallRun = 0;
  arrivedAtStep: number | null = null;
  prevTraj: { points: { t: number; x: number; y: number; v: number }[] } | null = null;
  trackingErrSum = 0;
  trackingErrN = 0;
  // summary 聚合（Task 11 的 buildSummary 使用）
  speedSum = 0;
  speedN = 0;
  speedMax = 0;
  minObstacleDist = Infinity;
  stateCounts: Record<string, number> = {};
  /** 历史最大沿路线弧长（进度指标用，防弧长回落时进度倒退） */
  maxFrenetS = 0;
  emergencyBrakeCount = 0;
  boundsResetCount = 0;
  /** 规划帧数 / 其中需 >1 轮（收敛迭代指标；耗时见 report.buildSummary 的 engineInfo） */
  planCount = 0;
  multiRoundPlans = 0;

  constructor(cfg?: Partial<DetectorConfig>, world?: Partial<DetectorWorld>) {
    this.cfg = { ...DEFAULT_DETECTOR_CONFIG, ...cfg };
    this.world = {
      track: world?.track ?? { centerPoints: [], innerBoundary: [], outerBoundary: [] },
      obstacles: world?.obstacles ?? [],
    };
    this.merger = new IssueMerger(this.cfg);
    this.trackFrenet = buildFrenetReference(this.world.track.centerPoints, true);
  }

  feed(rec: FrameRecord): void {
    runSafetyChecks(this, rec);
    runPerceptionChecks(this, rec);
    runPlanningChecks(this, rec);
    runMissionChecks(this, rec);
    this._accumulate(rec);
  }

  /** summary 聚合（报告用）：速度/最小障碍距离/状态计数/事件计数 */
  private _accumulate(rec: FrameRecord): void {
    if (rec.type === 'step') {
      const s = rec.car.speed;
      this.speedSum += s;
      this.speedN += 1;
      if (s > this.speedMax) this.speedMax = s;
    } else if (rec.type === 'perception') {
      const md = rec.stats?.minDist;
      if (typeof md === 'number' && md < this.minObstacleDist) this.minObstacleDist = md;
    } else if (rec.type === 'plan') {
      this.stateCounts[rec.state] = (this.stateCounts[rec.state] ?? 0) + 1;
      this.planCount += 1;
      if (rec.revised === true) this.multiRoundPlans += 1;
      const fs = (rec.frenet as { s?: number } | undefined)?.s;
      if (typeof fs === 'number' && fs > this.maxFrenetS) this.maxFrenetS = fs;
    } else if (rec.type === 'event') {
      if (rec.kind === 'emergency_brake') this.emergencyBrakeCount += 1;
      else if (rec.kind === 'bounds_reset') this.boundsResetCount += 1;
    }
  }

  finish(): Issue[] {
    finalizeMissionChecks(this);
    return this.merger.finish();
  }
}
