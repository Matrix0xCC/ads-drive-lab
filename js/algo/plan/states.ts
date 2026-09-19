// ==================== 规划器状态机 ====================
//
// 【解耦架构下的状态语义】（见 spec 2026-09-19-decoupled-planner）
//   畅通 + 路线     → FOLLOW（轨迹目标速度由速度层巡航剖面承载）
//   畅通 + 无路线   → CRUISE
//   路径偏离车道中心 → AVOID（避障侧向；overlays 的障碍高亮依赖此状态）
//   全程无可行路径  → DECEL（停车线：所有候选在整段视程内都被堵，速度层
//                     在障碍前减速停车；横向窗口内阻塞不算——还可能有出路）
//   物理停不住      → EMERGENCY（速度层最大减速兜底）
//   ARRIVED 由 planner 入口提前返回，不经过本函数
//
// 【状态转换】
//                   路径偏离车道
//              ┌──────────────────┐
//              ▼                  │ 回到车道
//   CRUISE/FOLLOW ──── AVOID ────┘
//     │                 │
//     │ 近终点           │ 全部阻塞（停车线）
//     ▼                 ▼
//   DECEL ◄─────────────┘
//     │                 │ 停不住
//     ▼                 ▼
//   ARRIVED*        EMERGENCY（每帧重评估）
//   （* planner 入口处理）

/** 规划器状态枚举 */
export enum PlanState {
  /** 闭环巡航 — 无路线，跟随赛道中心线 */
  CRUISE = 'CRUISE',
  /** 路线跟随 — 有路线，正常行驶 */
  FOLLOW = 'FOLLOW',
  /** 避障 — 当前路径偏离车道中心 */
  AVOID = 'AVOID',
  /** 减速 — 路线终点减速区 或 停车线（全部路径阻塞） */
  DECEL = 'DECEL',
  /** 已到达 — 车辆已到达路线终点 */
  ARRIVED = 'ARRIVED',
  /** 紧急 — 物理停不住，最大减速兜底 */
  EMERGENCY = 'EMERGENCY',
}

/** 状态转移条件（解耦架构：路径层/速度层决策驱动） */
export interface TransitionInput {
  routeActive: boolean;      // 是否有激活的路线
  nearEnd: boolean;          // 是否接近终点（剩余 < DECEL_ZONE）
  avoiding: boolean;         // 当前路径偏离车道中心（避让侧向）
  allBlockedFar: boolean;    // 所有候选在全程视程内阻塞（停车线生效）
  stopInfeasible: boolean;   // 物理停不住（速度层兜底）
}

/**
 * 确定下一状态
 *
 * 优先级：EMERGENCY > DECEL(停车线) > AVOID > DECEL(终点) > FOLLOW/CRUISE
 */
export function nextState(input: TransitionInput): PlanState {
  if (input.stopInfeasible) return PlanState.EMERGENCY;
  if (input.allBlockedFar) return PlanState.DECEL;       // 停车线（真无解）
  if (input.avoiding) return PlanState.AVOID;
  if (input.routeActive && input.nearEnd) return PlanState.DECEL;
  return input.routeActive ? PlanState.FOLLOW : PlanState.CRUISE;
}
