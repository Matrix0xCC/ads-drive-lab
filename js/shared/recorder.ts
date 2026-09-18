// ==================== 数据记录与回放 ====================
//
// 【用途】记录自动驾驶流水线的完整状态，导出/导入 JSON 用于离线回放和问题定位。
//
// 【记录频率】每次 plan() 调用记录一帧（15 Hz），包含：
//   - 车辆状态（x, y, angle, speed）
//   - LiDAR 扫描（angles, distances）
//   - 规划输出（轨迹点、代价、类型、状态）
//   - 激活的路线
//
// 【使用方式】
//   recorder.start()          → 开始记录
//   recorder.recordFrame(...)  → 每帧调用
//   recorder.exportJSON()      → 导出 JSON 字符串
//   recorder.importJSON(json)  → 加载历史记录
//   recorder.getFrame(i)       → 获取第 i 帧数据
//   recorder.frames            → 所有帧数据

import { CarState, LidarScan, Trajectory, Route } from './types.js';

// ═══════════════════════════════════════════════════
// 数据类型
// ═══════════════════════════════════════════════════

/** 单帧记录 */
export interface RecordedFrame {
  frame: number;
  timestamp: number;
  car: {
    x: number; y: number; angle: number; speed: number;
    steerAngle: number;
  };
  lidar: {
    angles: number[];       // 只存最近障碍物的方向+距离（非 MAX 值）
    distances: number[];
  };
  trajectory: {
    points: { t: number; x: number; y: number; v: number }[];
    type: string;
    cost: number;
    state: string;
  };
  route: {
    startIndex: number; goalIndex: number; length: number;
  } | null;
}

/** 完整记录 */
export interface Recording {
  version: 1;
  mapName: string;
  recordedAt: string;
  frames: RecordedFrame[];
}

// ═══════════════════════════════════════════════════
// 记录器
// ═══════════════════════════════════════════════════

export class Recorder {
  private _recording = false;
  private _frames: RecordedFrame[] = [];
  private _mapName = 'unknown';
  private _frameCount = 0;

  /** 开始记录 */
  start(mapName: string) {
    this._recording = true;
    this._frames = [];
    this._mapName = mapName;
    this._frameCount = 0;
  }

  /** 停止记录 */
  stop() {
    this._recording = false;
  }

  get isRecording() { return this._recording; }
  get frames() { return this._frames; }

  /**
   * 记录一帧完整数据
   */
  recordFrame(
    car: CarState,
    lidarScan: LidarScan,
    trajectory: Trajectory,
    plannerState: string,
    route: Route | null,
  ) {
    if (!this._recording) return;

    // LiDAR 数据压缩：只保存距离 < MAX 的命中（稀疏存储）
    const lidarAngles: number[] = [];
    const lidarDistances: number[] = [];
    const maxDist = 599; // MAX_SENSOR_DIST - 1
    for (let i = 0; i < lidarScan.angles.length; i++) {
      const d = lidarScan.distances[i] as number;
      if (d < maxDist) {
        lidarAngles.push(lidarScan.angles[i] as number);
        lidarDistances.push(d);
      }
    }

    this._frames.push({
      frame: this._frameCount++,
      timestamp: performance.now(),
      car: {
        x: car.x as number, y: car.y as number,
        angle: car.angle as number, speed: car.speed as number,
        steerAngle: car.steerAngle as number,
      },
      lidar: { angles: lidarAngles, distances: lidarDistances },
      trajectory: {
        points: trajectory.points.map(p => ({
          t: p.t as number, x: p.x as number, y: p.y as number, v: p.v as number,
        })),
        type: trajectory.type,
        cost: trajectory.cost,
        state: plannerState,
      },
      route: route ? {
        startIndex: route.startIndex,
        goalIndex: route.goalIndex,
        length: route.length as number,
      } : null,
    });
  }

  /**
   * 导出为 JSON 字符串
   *
   * 稀疏 LiDAR + 只存关键字段，典型 30 秒记录 ≈ 450 帧 ≈ 200KB
   */
  exportJSON(): string {
    const recording: Recording = {
      version: 1,
      mapName: this._mapName,
      recordedAt: new Date().toISOString(),
      frames: this._frames,
    };
    return JSON.stringify(recording, null, 2);
  }

  /**
   * 从 JSON 字符串导入历史记录
   */
  importJSON(json: string): Recording {
    const recording = JSON.parse(json) as Recording;
    this._frames = recording.frames;
    this._mapName = recording.mapName;
    this._frameCount = this._frames.length;
    return recording;
  }

  /** 重置 */
  reset() {
    this._frames = [];
    this._frameCount = 0;
    this._recording = false;
  }
}

// ═══════════════════════════════════════════════════
// 回放分析工具
// ═══════════════════════════════════════════════════

/**
 * 分析记录中的异常帧
 *
 * 检查项：
 *   - emergency 轨迹（规划器认为无安全路径）
 *   - 轨迹点紧邻障碍物（最近 LiDAR 命中 < 20px）
 *   - 状态切换（AVOID ↔ CRUISE 反复切换 = 摇摆）
 */
export function analyzeRecording(frames: RecordedFrame[]): {
  emergencies: number[];
  closeCalls: number[];
  oscillations: { frame: number; from: string; to: string }[];
} {
  const emergencies: number[] = [];
  const closeCalls: number[] = [];
  const oscillations: { frame: number; from: string; to: string }[] = [];

  for (let i = 0; i < frames.length; i++) {
    const f = frames[i];

    if (f.trajectory.type === 'emergency') {
      emergencies.push(f.frame);
    }

    const minLidarDist = Math.min(...f.lidar.distances, Infinity);
    if (minLidarDist < 20) {
      closeCalls.push(f.frame);
    }

    if (i > 0) {
      const prev = frames[i - 1].trajectory.state;
      const curr = f.trajectory.state;
      if (prev !== curr && (prev === 'AVOID' || curr === 'AVOID')) {
        oscillations.push({ frame: f.frame, from: prev, to: curr });
      }
    }
  }

  return { emergencies, closeCalls, oscillations };
}
