// ==================== 归档接口：headless 记录 / 问题 / 报告 ====================
//
// 这些是序列化形态的数据契约（字段为普通 number，仿真单位）。
// runner 写入、detector 消费、replay 读取三方共用。
// 与 pipeline/planning 的运行时品牌类型不同，归档记录不带量纲标签。

export interface RunMeta {
  version: 1;
  id: string;
  scenario: string;
  map: string;
  seed: number;
  durationS: number;
  steps: number;
  gitCommit: string | null;
  nodeVersion: string;
  createdAt: string;
  verdict: 'PASS' | 'ISSUES' | 'ERROR';
  issueCounts: { high: number; medium: number; low: number };
}

export interface ObstaclePlain { x: number; y: number; width: number; height: number; angle: number; zHeight?: number; }

export interface WorldFile {
  map: string;
  track: {
    centerPoints: [number, number][];
    innerBoundary: [number, number][];
    outerBoundary: [number, number][];
  };
  obstacles: ObstaclePlain[];
  route: { waypoints: [number, number][]; startIndex: number; goalIndex: number; length: number } | null;
}

export interface BaseRecord { type: string; t: number; step: number; }

export interface StepRecord extends BaseRecord {
  type: 'step';
  car: { x: number; y: number; angle: number; speed: number; steerAngle: number };
  control: { lookaheadDist: number; alpha: number; steerAngle: number; throttle: number } | null;
}

export interface PerceptionRecord extends BaseRecord {
  type: 'perception';
  scan: { angles: number[]; distances: number[]; rawDistances: number[]; confidences: number[] };
  stats: { rawHits: number; filteredHits: number; frontDist: number; minDist: number } | null;
}

export interface PlanRecord extends BaseRecord {
  type: 'plan';
  state: string;
  frenet: { s: number; l: number; dl_ds: number; headingDiff: number };
  candidates: { targetL: number; targetV: number; cost: number; collision: boolean }[];
  best: { cost: number; targetL: number; targetV: number };
  trajectory: { points: { t: number; x: number; y: number; v: number }[]; type: string; cost: number };
}

export interface GpsRecord extends BaseRecord {
  type: 'gps';
  x: number;
  y: number;
  err: number;
}

export interface EventRecord extends BaseRecord {
  type: 'event';
  kind: string;
  payload: Record<string, unknown>;
}

export type FrameRecord = StepRecord | PerceptionRecord | PlanRecord | GpsRecord | EventRecord;

export interface Issue {
  category: 'safety' | 'perception' | 'planning' | 'mission';
  severity: 'high' | 'medium' | 'low';
  check: string;
  firstFrame: number;
  lastFrame: number;
  count: number;
  summary: string;
  details: Record<string, unknown>;
  samples: number[];
}

export interface RunSummary {
  steps: number;
  simSeconds: number;
  speed: { maxMps: number; avgMps: number };
  arrivedAtStep: number | null;
  trackingRmseM: number | null;
  minObstacleDistM: number | null;
  stateCounts: Record<string, number>;
  emergencyBrakes: number;
  boundsResets: number;
  [key: string]: unknown;
}

export interface RunsIndexEntry {
  id: string;
  scenario: string;
  map: string;
  seed: number;
  createdAt: string;
  verdict: string;
  issues: { high: number; medium: number; low: number };
  durationS: number;
}
