// ==================== UI 状态机（纯逻辑，可单测） ====================
//
// 单一状态来源：所有 UI 行为都是 reduce(state, event) 的投影。
// 副作用（引擎暂停、面板显隐、回放加载）由 controller 订阅状态变化执行。
// 本模块不得 import DOM/three（仅类型 import 引擎的 RouteState）。

import type { RouteState } from '../sim/engine.js';
import type { SpeedUnit } from '../shared/display.js';

export type UiMode = 'LIVE' | 'REPLAY';
export type RoutePickPhase = 'idle' | 'picking-goal';

export interface UiState {
  mode: UiMode;
  /** 实时仿真暂停（回放期间保持不变，退出回放即恢复） */
  livePaused: boolean;
  /** 右键选路流程（从引擎剥离的纯 UI 状态） */
  routePick: RoutePickPhase;
  pickStartIdx: number | null;
  scenarioName: string | null;
  /** inspect 面板展开状态 */
  inspectorOpen: boolean;
  /** 速度显示单位（点击 HUD 速度切换；默认 km/h） */
  speedUnit: SpeedUnit;
}

export type UiEvent =
  | { type: 'TOGGLE_PAUSE' }
  | { type: 'ENTER_REPLAY' }
  | { type: 'EXIT_REPLAY' }
  | { type: 'RIGHT_CLICK'; idx: number }
  | { type: 'CANCEL' }
  | { type: 'SCENARIO_CHANGED'; name: string }
  | { type: 'ROUTE_SET' }
  | { type: 'TOGGLE_INSPECTOR' }
  | { type: 'TOGGLE_SPEED_UNIT' };

export const INITIAL_UI_STATE: UiState = {
  mode: 'LIVE',
  livePaused: false,
  routePick: 'idle',
  pickStartIdx: null,
  scenarioName: null,
  inspectorOpen: true,
  speedUnit: 'kmh',
};

export function reduce(state: UiState, e: UiEvent): UiState {
  switch (e.type) {
    case 'TOGGLE_PAUSE':
      // 回放中不作用于实时暂停（Space 的播放/暂停语义由 index.ts 按 mode 路由）
      return state.mode === 'LIVE' ? { ...state, livePaused: !state.livePaused } : state;
    case 'ENTER_REPLAY':
      return state.mode === 'REPLAY' ? state : { ...state, mode: 'REPLAY' };
    case 'EXIT_REPLAY':
      return state.mode === 'LIVE' ? state : { ...state, mode: 'LIVE' };
    case 'RIGHT_CLICK':
      if (state.mode !== 'LIVE') return state;   // 回放中不给实时世界设路线
      if (state.routePick === 'idle') {
        return { ...state, routePick: 'picking-goal', pickStartIdx: e.idx };
      }
      return state;   // 已选起点：建路线（失败保持/成功 ROUTE_SET）由 controller 处理
    case 'CANCEL':
      return state.routePick === 'picking-goal'
        ? { ...state, routePick: 'idle', pickStartIdx: null }
        : state;
    case 'SCENARIO_CHANGED':
      return { ...state, scenarioName: e.name, routePick: 'idle', pickStartIdx: null };
    case 'ROUTE_SET':
      return state.routePick === 'picking-goal'
        ? { ...state, routePick: 'idle', pickStartIdx: null }
        : state;
    case 'TOGGLE_INSPECTOR':
      return { ...state, inspectorOpen: !state.inspectorOpen };
    case 'TOGGLE_SPEED_UNIT':
      return { ...state, speedUnit: state.speedUnit === 'kmh' ? 'ms' : 'kmh' };
  }
}

/**
 * 上下文提示文案。
 * routeState 参数预留：如需展示路线状态相关文案（导航中/已到达）在此扩展。
 */
export function deriveHint(ui: UiState, routeState: RouteState): string {
  if (ui.mode === 'REPLAY') return '回放中：Space 播放/暂停 · ←/→ 单帧 · 拖动时间轴 · Esc 退出回放';
  if (ui.routePick === 'picking-goal') return '再次右键设置终点（Esc 取消）';
  if (ui.livePaused) return '已暂停（Space 继续）';
  // routeState 暂未参与分支（见 spec 开放问题）
  void routeState;
  return '右键设置路线起点 · 左键查看物体 · Space 暂停 · R 重置视角';
}
