// ==================== UI 控制器（状态 → 副作用） ====================
//
// 持有 UiState；dispatch 走纯函数 reduce，对比前后状态后仅触发变化的副作用。
// 本模块不得 import DOM/three：所有副作用通过 UiEffects 注入，
// 是"状态集中"与"DOM 只是渲染投影"之间的唯一边界。

import { UiState, UiEvent, UiMode, reduce, deriveHint, INITIAL_UI_STATE } from './state.js';
import type { RouteState } from '../sim/engine.js';
import type { SpeedUnit } from '../shared/display.js';

export interface UiEffects {
  onLivePaused(paused: boolean): void;
  onMode(mode: UiMode): void;
  onInspector(open: boolean): void;
  onHint(hint: string): void;
  onSpeedUnit(unit: SpeedUnit): void;
}

export class UiController {
  private _state: UiState;

  constructor(
    private effects: UiEffects,
    private getRouteState: () => RouteState = () => 'NONE',
    initial: UiState = INITIAL_UI_STATE,
  ) {
    this._state = initial;
  }

  get state(): UiState { return this._state; }

  dispatch(e: UiEvent): void {
    const prev = this._state;
    const next = reduce(prev, e);
    if (next === prev) return;   // 无变化不触发副作用
    this._state = next;
    if (next.livePaused !== prev.livePaused) this.effects.onLivePaused(next.livePaused);
    if (next.mode !== prev.mode) this.effects.onMode(next.mode);
    if (next.inspectorOpen !== prev.inspectorOpen) this.effects.onInspector(next.inspectorOpen);
    if (next.speedUnit !== prev.speedUnit) this.effects.onSpeedUnit(next.speedUnit);
    this.effects.onHint(deriveHint(next, this.getRouteState()));
  }

  /** 按当前状态重发全部副作用（setup 初始化时调用一次） */
  refresh(): void {
    this.effects.onLivePaused(this._state.livePaused);
    this.effects.onMode(this._state.mode);
    this.effects.onInspector(this._state.inspectorOpen);
    this.effects.onSpeedUnit(this._state.speedUnit);
    this.effects.onHint(deriveHint(this._state, this.getRouteState()));
  }
}
