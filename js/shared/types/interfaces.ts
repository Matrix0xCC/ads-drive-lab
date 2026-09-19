// ==================== 数据接口（barrel） ====================

export type {
  Point2D, TrackGeometry, Obstacle, CarState,
} from './interfaces/geometry.js';
export { VEHICLE_MAX_STEER, obstacleToSegments } from './interfaces/geometry.js';

export type {
  WorldSnapshot, LidarScan, PlanningOutput, ControlCommand,
} from './interfaces/pipeline.js';
export { createWorldSnapshot, createLidarScan, createControlCommand } from './interfaces/pipeline.js';

export type {
  FrenetState, FrenetReference, TrajectoryPoint, Trajectory, Route, GPSReading,
} from './interfaces/planning.js';
export { createTrajectory, createRoute, createGPSReading } from './interfaces/planning.js';

export type {
  RunMeta, ObstaclePlain, WorldFile, BaseRecord, StepRecord, PerceptionRecord,
  PlanRecord, GpsRecord, EventRecord, FrameRecord, Issue, RunSummary, RunsIndexEntry,
} from './interfaces/archive.js';
