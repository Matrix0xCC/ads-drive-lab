// ==================== 测试：UI 控制器（状态 → 副作用） ====================
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { UiController } from '../../ui/controller.js';

function makeCtl() {
  const calls = {
    pause: [] as boolean[], mode: [] as string[],
    info: [] as boolean[], debug: [] as boolean[], picker: [] as boolean[],
    hint: [] as string[], unit: [] as string[],
  };
  const ctl = new UiController({
    onLivePaused: (p) => calls.pause.push(p),
    onMode: (m) => calls.mode.push(m),
    onInfoPanel: (o) => calls.info.push(o),
    onDebugPanel: (o) => calls.debug.push(o),
    onRunPicker: (o) => calls.picker.push(o),
    onHint: (h) => calls.hint.push(h),
    onSpeedUnit: (u) => calls.unit.push(u),
    onCorridorVisible: () => {},
    onFrenetTicksVisible: () => {},
    onLidarVisible: () => {},
    onRoutePickPhase: () => {},
  }, () => 'NONE');
  return { ctl, calls };
}

describe('UiController.dispatch', () => {
  it('LIVE TOGGLE_PAUSE → onLivePaused(true) + 提示变为已暂停', () => {
    const { ctl, calls } = makeCtl();
    ctl.dispatch({ type: 'TOGGLE_PAUSE' });
    assert.deepStrictEqual(calls.pause, [true]);
    assert.strictEqual(calls.hint[calls.hint.length - 1], '已暂停（Space 继续）');
  });

  it('REPLAY 中 TOGGLE_PAUSE → 零副作用（状态未变）', () => {
    const { ctl, calls } = makeCtl();
    ctl.dispatch({ type: 'ENTER_REPLAY' });
    calls.pause.length = calls.mode.length = calls.hint.length = 0;
    ctl.dispatch({ type: 'TOGGLE_PAUSE' });
    assert.deepStrictEqual(calls.pause, []);
    assert.deepStrictEqual(calls.mode, []);
    assert.deepStrictEqual(calls.hint, []);
  });

  it('回放进出：onMode 触发、onLivePaused 从不触发（暂停保持）', () => {
    const { ctl, calls } = makeCtl();
    ctl.dispatch({ type: 'TOGGLE_PAUSE' });          // 先暂停
    calls.pause.length = 0;
    ctl.dispatch({ type: 'ENTER_REPLAY' });
    ctl.dispatch({ type: 'EXIT_REPLAY' });
    assert.deepStrictEqual(calls.mode, ['REPLAY', 'LIVE']);
    assert.deepStrictEqual(calls.pause, []);
    assert.strictEqual(ctl.state.livePaused, true);
  });

  it('路线模式两步流程：进模式 → 选起点 → 第二步状态不变（建路线由 controller 处理）', () => {
    const { ctl, calls } = makeCtl();
    ctl.dispatch({ type: 'TOGGLE_ROUTE_SET' });
    ctl.dispatch({ type: 'PICK_POINT', idx: 3 });
    assert.strictEqual(ctl.state.routePick, 'picking-goal');
    calls.hint.length = 0;
    ctl.dispatch({ type: 'PICK_POINT', idx: 9 });   // 起点已选：交给 controller 建路线（此处无 roadGraph → 无变化）
    assert.deepStrictEqual(calls.hint, []);
    ctl.dispatch({ type: 'ROUTE_SET' });
    assert.strictEqual(ctl.state.routePick, 'idle');
    assert.strictEqual(calls.hint[calls.hint.length - 1], 'T 设路线 · WASD 平移视角 · 左键查看物体 · Space 暂停 · R 重置视角');
  });

  it('TOGGLE_INFO / TOGGLE_DEBUG → 各自面板副作用', () => {
    const { ctl, calls } = makeCtl();
    ctl.dispatch({ type: 'TOGGLE_INFO' });
    ctl.dispatch({ type: 'TOGGLE_DEBUG' });
    assert.deepStrictEqual(calls.info, [false]);
    assert.deepStrictEqual(calls.debug, [false]);
  });

  it('OPEN/CLOSE_RUN_PICKER → onRunPicker 开关', () => {
    const { ctl, calls } = makeCtl();
    ctl.dispatch({ type: 'OPEN_RUN_PICKER' });
    ctl.dispatch({ type: 'CLOSE_RUN_PICKER' });
    assert.deepStrictEqual(calls.picker, [true, false]);
  });

  it('TOGGLE_SPEED_UNIT → onSpeedUnit 按 kmh/ms 交替', () => {
    const { ctl, calls } = makeCtl();
    ctl.dispatch({ type: 'TOGGLE_SPEED_UNIT' });
    ctl.dispatch({ type: 'TOGGLE_SPEED_UNIT' });
    assert.deepStrictEqual(calls.unit, ['ms', 'kmh']);
  });

  it('refresh 按当前状态重发全部副作用（初始化用）', () => {
    const { ctl, calls } = makeCtl();
    ctl.refresh();
    assert.deepStrictEqual(calls.pause, [false]);
    assert.deepStrictEqual(calls.mode, ['LIVE']);
    assert.deepStrictEqual(calls.info, [true]);
    assert.deepStrictEqual(calls.debug, [true]);
    assert.deepStrictEqual(calls.picker, [false]);
    assert.deepStrictEqual(calls.unit, ['kmh']);
    assert.strictEqual(calls.hint.length, 1);
  });
});
