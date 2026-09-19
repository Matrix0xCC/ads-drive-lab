// ==================== 共享归档记录层（浏览器 / headless 同源） ====================
//
// 归档格式的唯一生产点：engine 消息主题载荷 → FrameRecord 映射、
// WorldFile 真值序列化。浏览器（导出运行归档）与 headless CLI 共用；
// 本模块保持无 node 依赖，可被浏览器直接 import。

import {
  FrameRecord, WorldFile, TrackGeometry, Obstacle, Route,
} from '../shared/types.js';
import { Scenario } from './scenario.js';

/** 归档订阅的主题清单（录制/导出/检测三处共用，保证顺序一致） */
export const ARCHIVE_TOPICS = [
  'control:state', 'perception:result', 'planning:result', 'gps:result', 'sim:event',
];

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

/** 真值序列化：赛道几何 + 障碍物 + 路线 → WorldFile（检测与回放的公共依据） */
export function buildWorldFile(
  track: TrackGeometry, obstacles: Obstacle[], scenario: Scenario, route: Route | null,
): WorldFile {
  const pts = (arr: any[]): [number, number][] => arr.map(p => [p.x, p.y]);
  return {
    map: scenario.map,
    track: {
      centerPoints: pts(track.centerPoints),
      innerBoundary: pts(track.innerBoundary),
      outerBoundary: pts(track.outerBoundary),
    },
    obstacles: obstacles.map(o => ({
      x: o.x as number, y: o.y as number, width: o.width as number,
      height: o.height as number, angle: o.angle as number, zHeight: o.zHeight,
    })),
    route: route ? {
      waypoints: pts(route.waypoints), startIndex: route.startIndex,
      goalIndex: route.goalIndex, length: route.length as number,
    } : null,
  };
}
