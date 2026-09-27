// ==================== 共享类型（barrel） ====================
//
// 拆分后的模块：
//   shared/types/interfaces/geometry.ts  — Point2D, TrackGeometry, Obstacle, CarState, obstacleToSegments
//   shared/types/interfaces/pipeline.ts  — WorldSnapshot, LidarScan, ControlCommand + factories
//   shared/types/interfaces/planning.ts  — Frenet, Trajectory, Route, GPSReading + factories
//
// Frenet 坐标系和五次多项式已移至 algo/plan/（属于算法实现，非共享类型）。

export type {
  Point2D, TrackGeometry, Obstacle, CarState, RoadAttrs, AttrSpan,
  WorldSnapshot, LidarScan, PlanningOutput, ControlCommand,
  FrenetState, FrenetReference,
  TrajectoryPoint, Trajectory,
  Route, GPSReading,
  RunMeta, ObstaclePlain, WorldFile, BaseRecord, StepRecord, PerceptionRecord,
  PlanRecord, GpsRecord, EventRecord, FrameRecord, Issue, RunSummary, RunsIndexEntry,
} from './types/interfaces.js';

export {
  VEHICLE_MAX_STEER, obstacleToSegments, attrsAtS,
  createWorldSnapshot, createLidarScan, createControlCommand,
  createTrajectory, createRoute, createGPSReading,
} from './types/interfaces.js';
