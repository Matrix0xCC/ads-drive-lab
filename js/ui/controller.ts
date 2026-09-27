// ==================== UI 控制器（状态 → 副作用） ====================
//
// 持有 UiState；dispatch 走纯函数 reduce，对比前后状态后仅触发变化的副作用。
// 本模块不得 import DOM/three：所有副作用通过 UiEffects 注入，
// 是"状态集中"与"DOM 只是渲染投影"之间的唯一边界。

import { UiState, UiEvent, UiMode, RoutePickPhase, RouteSource, reduce, deriveHint, INITIAL_UI_STATE } from './state.js';
import type { RouteState } from '../sim/engine.js';
import type { SpeedUnit } from '../shared/display.js';

export interface UiEffects {
  onLivePaused(paused: boolean): void;
  onMode(mode: UiMode): void;
  onInfoPanel(open: boolean): void;
  onDebugPanel(open: boolean): void;
  onRunPicker(open: boolean): void;
  onHint(hint: string): void;
  onSpeedUnit(unit: SpeedUnit): void;
  onCorridorVisible(visible: boolean): void;
  onFrenetTicksVisible(visible: boolean): void;
  onLidarVisible(visible: boolean): void;
  onBasemapVisible(visible: boolean): void;
  onFollowCam(on: boolean): void;
  onRoutePickPhase(phase: RoutePickPhase): void;
  onRouteInfo(info: { source: RouteSource; summary: string }): void;
  onNotice(text: string): void;
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
    if (next.infoOpen !== prev.infoOpen) this.effects.onInfoPanel(next.infoOpen);
    if (next.debugOpen !== prev.debugOpen) this.effects.onDebugPanel(next.debugOpen);
    if (next.runPickerOpen !== prev.runPickerOpen) this.effects.onRunPicker(next.runPickerOpen);
    if (next.speedUnit !== prev.speedUnit) this.effects.onSpeedUnit(next.speedUnit);
    if (next.corridorVisible !== prev.corridorVisible) this.effects.onCorridorVisible(next.corridorVisible);
    if (next.frenetTicksVisible !== prev.frenetTicksVisible) this.effects.onFrenetTicksVisible(next.frenetTicksVisible);
    if (next.lidarVisible !== prev.lidarVisible) this.effects.onLidarVisible(next.lidarVisible);
    if (next.basemapVisible !== prev.basemapVisible) this.effects.onBasemapVisible(next.basemapVisible);
    if (next.followCam !== prev.followCam) this.effects.onFollowCam(next.followCam);
    if (next.routePick !== prev.routePick) this.effects.onRoutePickPhase(next.routePick);
    if (next.routeInfo !== prev.routeInfo) this.effects.onRouteInfo(next.routeInfo);
    if (next.notice !== prev.notice) this.effects.onNotice(next.notice);
    this.effects.onHint(deriveHint(next, this.getRouteState()));
  }

  /** 按当前状态重发全部副作用（setup 初始化时调用一次） */
  refresh(): void {
    this.effects.onLivePaused(this._state.livePaused);
    this.effects.onMode(this._state.mode);
    this.effects.onInfoPanel(this._state.infoOpen);
    this.effects.onDebugPanel(this._state.debugOpen);
    this.effects.onRunPicker(this._state.runPickerOpen);
    this.effects.onSpeedUnit(this._state.speedUnit);
    this.effects.onCorridorVisible(this._state.corridorVisible);
    this.effects.onFrenetTicksVisible(this._state.frenetTicksVisible);
    this.effects.onLidarVisible(this._state.lidarVisible);
    this.effects.onBasemapVisible(this._state.basemapVisible);
    this.effects.onFollowCam(this._state.followCam);
    this.effects.onRoutePickPhase(this._state.routePick);
    this.effects.onRouteInfo(this._state.routeInfo);
    this.effects.onNotice(this._state.notice);
    this.effects.onHint(deriveHint(this._state, this.getRouteState()));
  }
}
