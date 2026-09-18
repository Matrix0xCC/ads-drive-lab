// ==================== 规划器状态机 ====================
//
// 【状态转换图】
//
//                   障碍物清除
//              ┌──────────────┐
//              ▼              │
//   CRUISE ──→ AVOID ────────┘
//     │          │
//     │ 设置路线  │ 设置路线
//     ▼          ▼
//   FOLLOW ──→ AVOID ────────┐
//     │          │            │ 障碍物清除
//     │ 近终点    │ 近终点     │
//     ▼          ▼            │
//   DECEL ───→ ARRIVED        │
//     │                       │
//     └── 取消路线 → CRUISE ◄─┘
//
//   任意状态 ──全碰撞──→ EMERGENCY（一次性，下一帧重评估）

/** 规划器状态枚举 */
export enum PlanState {
  /** 闭环巡航 — 无路线，跟随赛道中心线 */
  CRUISE = 'CRUISE',
  /** 路线跟随 — 有路线，正常行驶 */
  FOLLOW = 'FOLLOW',
  /** 避障 — 前方检测到障碍物，偏离中心线避开 */
  AVOID = 'AVOID',
  /** 终点减速 — 路线剩余距离 < 150px */
  DECEL = 'DECEL',
  /** 已到达 — 车辆已到达路线终点 */
  ARRIVED = 'ARRIVED',
  /** 紧急 — 所有候选轨迹都碰撞，直行减速 */
  EMERGENCY = 'EMERGENCY',
}

/** 状态转移条件 */
export interface TransitionInput {
  routeActive: boolean;         // 是否有激活的路线
  nearEnd: boolean;             // 是否接近终点（剩余 < DECEL_ZONE）
  atEnd: boolean;               // 是否已到达终点（剩余 < 10px）
  hasObstacle: boolean;         // 前方是否检测到障碍物
  allBlocked: boolean;          // 是否所有候选轨迹都碰撞
  wasEmergency: boolean;        // 上一帧是否是 EMERGENCY
}

/**
 * 确定下一状态
 *
 * 优先级：ARRIVED > EMERGENCY > AVOID > DECEL > FOLLOW/CRUISE
 * EMERGENCY 只持续一帧（下一帧重新评估，除非又全碰撞）
 */
export function nextState(current: PlanState, input: TransitionInput): PlanState {
  // 到达终点 → 直接 ARRIVED，不可逆
  if (input.atEnd) return PlanState.ARRIVED;

  // 上一帧紧急 → 这一帧重新评估（不卡死）
  if (input.wasEmergency && input.allBlocked) return PlanState.EMERGENCY;

  // 所有轨迹碰撞 → 紧急
  if (input.allBlocked) return PlanState.EMERGENCY;

  // 前方有障碍物 → 避障（优先级高于 DECEL）
  if (input.hasObstacle) return PlanState.AVOID;

  // 路线终点减速
  if (input.routeActive && input.nearEnd) return PlanState.DECEL;

  // 路线模式
  if (input.routeActive) return PlanState.FOLLOW;

  // 无路线 → 巡航
  return PlanState.CRUISE;
}

/** 状态对应的轨迹类型标签 */
export function stateTrajectoryType(state: PlanState, targetL?: number, currentL?: number): string {
  switch (state) {
    case PlanState.CRUISE: return 'lane_keep';
    case PlanState.FOLLOW: return 'lane_keep';
    case PlanState.DECEL:  return 'lane_keep';
    case PlanState.ARRIVED: return 'arrived';
    case PlanState.EMERGENCY: return 'emergency';
    case PlanState.AVOID:
      if (targetL !== undefined && currentL !== undefined) {
        return targetL > currentL ? 'avoid_left' : 'avoid_right';
      }
      return 'avoid_left';
  }
}
