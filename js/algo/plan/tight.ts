// ==================== 算法：紧间隙迟滞状态机（TightState） ====================
//
// 【职责】把"慢行通过 / 封死停车"的判定做成有记忆的施密特触发（Schmitt trigger），
// 消除临界抖动（实测：路径最小间距在物理阈值两侧 ±0.15m 内 3 帧翻转，导致
// "慢行"与"封死→紧急制动"互换，见 docs/domain.md 相关条目）。
//
// 【空间带（施密特触发）】以物理走廊（膨胀 = 车半宽 + margin(v)）为标称阈值 m，
// 上下各留 0.15m 的粘滞带：
//   - 进入 SEALED：连"更松"的走廊（m − 0.15）都没有候选 → 车确实过不去；
//   - 退出 SEALED：存在候选通过"更严"的走廊（m + 0.15）→ 车确实过得去；
//   - 带内（m ± 0.15）：保持原状态——这就是消除抖动的迟滞带。
//   （注意方向：进入用更松、退出用更严，两侧都不在标称线上翻转。）
//
// 【时间防抖（安全优先）】升级（→SEALED）**立即生效**——封死是安全动作（停车线/
// 紧急制动），宁可早封；降级（SEALED → 恢复）需目标态**连续一致 3 帧**（≈50ms）
// 确证后才解除（防恢复抖动；注意是"连续计数"而非"距上次切换的时间间隔"）。
//
// 【状态语义】
//   NONE     —— 存在舒适畅通候选（正常行驶）
//   CREEPING —— 只有物理可过候选（紧间隙慢行档，限速 8m/s）
//   SEALED   —— 连更松的走廊都无候选（停车线/紧急制动触发）

export type TightState = 'NONE' | 'CREEPING' | 'SEALED';

export interface TightInput {
  /** 存在候选在舒适走廊内 */
  hasComfortClear: boolean;
  /** 存在候选在物理走廊（标称 m）内 */
  hasHardClear: boolean;
  /** 存在候选在"更严"走廊（m + 0.15m）内 —— SEALED 的退出条件 */
  hasDeepClear: boolean;
  /** 连"更松"走廊（m − 0.15m）都无候选 —— SEALED 的进入条件 */
  hasDeepBlocked: boolean;
}

/** 迟滞记忆（规划器持有一份；纯数据，便于确定性复现与单测） */
export interface TightMemory {
  state: TightState;
  /** 正在等待确认的目标态（连续计数中） */
  pending: TightState | null;
  pendingN: number;
}

export const INITIAL_TIGHT: TightMemory = { state: 'NONE', pending: null, pendingN: 0 };

/** 目标态（施密特触发：进入/退出用阈值两侧的更松/更严判据，带内保持当前态） */
export function tightTarget(prev: TightState, cur: TightInput): TightState {
  if (prev === 'SEALED') {
    // 退出判据（更严，m+0.15）：确实过得去才恢复
    if (!cur.hasDeepClear) return 'SEALED';
    return cur.hasComfortClear ? 'NONE' : 'CREEPING';
  }
  // 进入判据（更松，m−0.15）：确实过不去才封死
  if (cur.hasDeepBlocked) return 'SEALED';
  // 未封死：标称线上无候选但在带内 → 谨慎档（CREEPING）
  return cur.hasComfortClear && cur.hasHardClear ? 'NONE' : 'CREEPING';
}

/** 推进迟滞状态机：升级立即生效；降级需目标态连续 3 帧一致 */
export function nextTight(prev: TightMemory, cur: TightInput): TightMemory {
  const target = tightTarget(prev.state, cur);
  if (target === prev.state) return { state: prev.state, pending: null, pendingN: 0 };
  if (target === 'SEALED') return { state: 'SEALED', pending: null, pendingN: 0 };   // 安全动作：立即封
  const pendingN = prev.pending === target ? prev.pendingN + 1 : 1;
  if (pendingN >= 3) return { state: target, pending: null, pendingN: 0 };
  return { state: prev.state, pending: target, pendingN };
}
