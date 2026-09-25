// ==================== UI 状态机（纯逻辑，可单测） ====================
//
// 单一状态来源：所有 UI 行为都是 reduce(state, event) 的投影。
// 副作用（引擎暂停、面板显隐、回放加载）由 controller 订阅状态变化执行。
// 本模块不得 import DOM/three（仅类型 import 引擎的 RouteState）。

import type { RouteState } from '../sim/engine.js';
import type { SpeedUnit } from '../shared/display.js';

export type UiMode = 'LIVE' | 'REPLAY';
export type RoutePickPhase = 'idle' | 'picking-goal';
/** 左侧菜单栏分区（手风琴单开） */
export type SidebarSection = 'world' | 'sim' | 'data';

export interface UiState {
  mode: UiMode;
  /** 实时仿真暂停（回放期间保持不变，退出回放即恢复） */
  livePaused: boolean;
  /** 右键选路流程（从引擎剥离的纯 UI 状态） */
  routePick: RoutePickPhase;
  pickStartIdx: number | null;
  scenarioName: string | null;
  /** 左侧菜单当前展开的分区（null = 全收起） */
  sidebarSection: SidebarSection | null;
  /** 右侧信息面板（详细遥测）展开状态 */
  infoOpen: boolean;
  /** 右侧调试面板（Inspect）展开状态 */
  debugOpen: boolean;
  /** 回放运行选择框（分页列表）开关 */
  runPickerOpen: boolean;
  /** 速度显示单位（点击 HUD 速度切换；默认 km/h） */
  speedUnit: SpeedUnit;
  /** 走廊可视化（自由带/瓶颈/候选路径；默认关。回放模式不跑规划器 → 无数据不显示） */
  corridorVisible: boolean;
  /** Frenet 刻度（s 横线/l 参考线/数字标签；默认开。与走廊各自独立开关） */
  frenetTicksVisible: boolean;
}

export type UiEvent =
  | { type: 'TOGGLE_PAUSE' }
  | { type: 'ENTER_REPLAY' }
  | { type: 'EXIT_REPLAY' }
  | { type: 'RIGHT_CLICK'; idx: number }
  | { type: 'CANCEL' }
  | { type: 'SCENARIO_CHANGED'; name: string }
  | { type: 'ROUTE_SET' }
  | { type: 'TOGGLE_SECTION'; section: SidebarSection }
  | { type: 'TOGGLE_INFO' }
  | { type: 'TOGGLE_DEBUG' }
  | { type: 'OPEN_RUN_PICKER' }
  | { type: 'CLOSE_RUN_PICKER' }
  | { type: 'TOGGLE_SPEED_UNIT' }
  | { type: 'TOGGLE_CORRIDOR' }
  | { type: 'TOGGLE_FRENET_TICKS' };

export const INITIAL_UI_STATE: UiState = {
  mode: 'LIVE',
  livePaused: false,
  routePick: 'idle',
  pickStartIdx: null,
  scenarioName: null,
  sidebarSection: null,
  infoOpen: true,
  debugOpen: true,
  runPickerOpen: false,
  speedUnit: 'kmh',
  /** 走廊可视化（默认关；回放模式无数据，不显示） */
  corridorVisible: false,
  /** Frenet 刻度（默认开） */
  frenetTicksVisible: true,
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
    case 'TOGGLE_SECTION':
      // 手风琴单开：点已展开的分区 → 收起；点其它分区 → 切换（前一个自动收起）
      return { ...state, sidebarSection: state.sidebarSection === e.section ? null : e.section };
    case 'TOGGLE_INFO':
      return { ...state, infoOpen: !state.infoOpen };
    case 'TOGGLE_DEBUG':
      return { ...state, debugOpen: !state.debugOpen };
    case 'OPEN_RUN_PICKER':
      return state.runPickerOpen ? state : { ...state, runPickerOpen: true };
    case 'CLOSE_RUN_PICKER':
      return state.runPickerOpen ? { ...state, runPickerOpen: false } : state;
    case 'TOGGLE_SPEED_UNIT':
      return { ...state, speedUnit: state.speedUnit === 'kmh' ? 'ms' : 'kmh' };
    case 'TOGGLE_CORRIDOR':
      return { ...state, corridorVisible: !state.corridorVisible };
    case 'TOGGLE_FRENET_TICKS':
      return { ...state, frenetTicksVisible: !state.frenetTicksVisible };
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
