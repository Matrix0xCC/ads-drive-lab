// ==================== 算法：控制模块（Pure Pursuit 轨迹跟踪） ====================
//
// 【职责】接收 Trajectory，用 Pure Pursuit 算法将轨迹点转为转向角+速度指令
//
// 【Pure Pursuit 算法】
// 这是自动驾驶/机器人领域最经典的轨迹跟踪算法之一。
// 它模拟了人类驾驶员的"预瞄"行为：盯着前方某个距离的点，
// 计算到达那个点需要的方向盘转角。
//
// 【几何原理】
//
//             目标点 (target)
//                ●
//               ╱|
//              ╱ │
//         ld  ╱  │
//            ╱   │
//           ╱    │ 
//          ╱ α   │
//    车 ↗───────┘
//       ← L →     (L = wheelbase, 轴距)
//
//   α = 目标点方向 - 车头朝向（需要转多少角度才能对准目标点）
//   ld = 预瞄距离（车到目标点的直线距离）
//
//   曲率公式：κ = 2 × sin(α) / ld
//   转向角公式：δ = arctan(κ × L) = arctan(2 × L × sin(α) / ld)
//
// 【推导】
// 车辆沿圆弧到达预瞄点，弦长 = ld，圆心角 = 2α，半径 = R。
// 由几何关系：sin(α) = ld / (2R) → κ = 1/R = 2×sin(α)/ld
// 自行车模型：κ = tan(δ)/L → δ = atan(κ×L) = atan(2×L×sin(α)/ld)
//
// 【直观理解】
// - 目标点在正前方（α≈0）→ κ≈0 → 直走
// - 目标点偏右（α>0）→ κ>0 → 右转
// - 目标点偏左（α<0）→ κ<0 → 左转
// - 预瞄距离越短 → 转向越激进（追近点）
// - 预瞄距离越长 → 转向越平缓（望远点）
//
// 【为什么叫"Pure" Pursuit？】
// 因为它是"纯追踪"——不去计算最优轨迹，不建立车辆动力学模型，
// 只是简单地追踪前方一个参考点。简单、鲁棒、计算量极小。
// 缺点是高速时可能"切弯"（过于激进地切入弯道内侧）。
//
// 【速度自适应预瞄距离】
// 高速时看更远（预瞄距离大），低速时看更近（预瞄距离小）。
// 这模拟了人类驾驶行为：高速巡航时目光放远，低速泊车时注意近处。
//
// 输入：Trajectory + CarState
// 输出：ControlCommand { steerAngle(Radians), throttle(DimLess) }

import { CarState, Trajectory, TrajectoryPoint, ControlCommand, createControlCommand } from '../shared/types.js';
import { Radians, Px, PxPerSec, DimLess, rad, m, px, pxps, dimless, radps, sin, atan2, normalizeAngle } from '../shared/dimensions.js';
import { debug } from '../shared/debug.js';

// ==================== 控制参数 ====================

/** 最小预瞄距离 10m */
const MIN_LOOKAHEAD: Px = m(10);

/** 最大预瞄距离 20m */
const MAX_LOOKAHEAD: Px = m(20);

/** 预瞄增益 — lookahead = gain × speed，限制在 [MIN, MAX] */
const LOOKAHEAD_GAIN: DimLess = dimless(0.4);

/** 轴距 3m — 与 env/car.ts 保持一致 */
const WHEELBASE: Px = m(3.0);

// ==================== Pure Pursuit 核心 ====================

/**
 * 在轨迹上寻找预瞄点
 *
 * 【查找策略】
 * 从轨迹起点开始（index 0），沿轨迹顺序搜索，
 * 找到第一个距离 ≥ lookahead 的点。这确保预瞄点始终在
 * 车辆前方（因为轨迹起点的距离 ≈ 0）。
 *
 * 如果所有轨迹点距离都 < lookahead（轨迹太短），返回最后一点。
 *
 * @param carX, carY 车辆当前位置
 * @param points     轨迹点序列（时间升序）
 * @param lookahead  预瞄距离
 * @returns 预瞄目标点
 */
function findLookaheadPoint(carX: Px, carY: Px, points: TrajectoryPoint[], lookahead: Px): TrajectoryPoint {
  let target = points[points.length - 1];  // 默认：轨迹终点（兜底）
  const lh = lookahead as number;
  const cx = carX as number;
  const cy = carY as number;

  for (let i = 0; i < points.length; i++) {
    const d = Math.hypot((points[i].x as number) - cx, (points[i].y as number) - cy);
    if (d >= lh) {
      target = points[i];
      break;
    }
  }

  return target;
}

/**
 * Pure Pursuit 控制器
 *
 * 【执行流程】
 * 1. 空轨迹保护 → 直行 + 怠速油门
 * 2. 计算速度自适应预瞄距离
 * 3. 在轨迹上找预瞄点
 * 4. 计算 α（alpha）= 预瞄方向与车头朝向的夹角
 * 5. 用几何公式计算转向角：δ = atan2(2×L×sin(α), ld)
 * 6. 油门 = 轨迹起点的目标速度 / 最大速度（限制在 [0.15, 1]）
 *
 * 【为什么不直接用轨迹点 yaw？】
 * Pure Pursuit 的输入只需要 (x, y) 坐标。几何关系决定：
 * 车在 P_car，目标在 P_target，弧线唯一确定 → 转向角直接算出。
 * 轨迹点的 yaw 只用于显示和代价评估，不参与控制。
 *
 * @param trajectory 规划模块输出的轨迹
 * @param car        当前车辆状态
 * @returns ControlCommand { steerAngle, throttle }
 */
export function control(trajectory: Trajectory, car: CarState): ControlCommand {
  const points = trajectory.points;
  if (!points || points.length === 0) {
    // 无轨迹：直行 + 怠速（防止完全停车）
    return createControlCommand(rad(0), dimless(0.15));
  }

  // 1. 速度自适应预瞄距离
  const lookaheadVal = Math.min(
    MAX_LOOKAHEAD as number,
    Math.max(MIN_LOOKAHEAD as number, (LOOKAHEAD_GAIN as number) * (car.speed as number))
  );
  const lookahead = px(lookaheadVal);

  // 2. 找到预瞄点
  const target = findLookaheadPoint(car.x, car.y, points, lookahead);

  // 3. 计算 α（车辆朝向与预瞄方向之间的夹角）
  const dx = (target.x as number) - (car.x as number);
  const dy = (target.y as number) - (car.y as number);
  const targetAngle = atan2(dy, dx);                // 预瞄方向的世界角度
  // α = targetAngle - car.angle，归一化到 [-π, π]
  let alpha = normalizeAngle(rad((targetAngle as number) - (car.angle as number)));

  // 4. 曲率 + 转向角计算
  const ld = Math.hypot(dx, dy) || 1;               // 实际预瞄距离
  // δ = atan2(2 × L × sin(α), ld)
  // TypeScript 保证：sin(α)=DimLess, WHEELBASE=Px, ld=number → atan2 → Radians ✓
  const steerAngle = atan2(2 * (WHEELBASE as number) * (sin(alpha) as number), ld);

  // 5. 油门：使用轨迹终点的目标速度（points[0].v 是当前速度，不是目标）
  // 终点速度≈0（速度层停车/紧急剖面）→ 油门必须为 0：0.15 怠速下限会把
  // "停车"变成 6.375m/s 恒速爬行，架空停车语义（批量 A/B 回归根因，2026-09-19）。
  const targetSpeed = points[points.length - 1].v as number;
  const throttle = targetSpeed < 1
    ? 0
    : Math.max(0.15, Math.min(1, targetSpeed / (car.maxSpeed as number)));

  debug.control = {
    lookaheadDist: lookaheadVal,
    lookaheadX: target.x as number, lookaheadY: target.y as number,
    alpha: alpha as number,
    steerAngle: steerAngle as number,
    throttle,
  };

  return createControlCommand(steerAngle, dimless(throttle));
}
