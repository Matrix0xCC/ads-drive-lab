// ==================== 测试：UI 状态机（纯逻辑） ====================
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { reduce, deriveHint, INITIAL_UI_STATE, UiState } from '../../ui/state.js';

const live = (over: Partial<UiState> = {}): UiState => ({ ...INITIAL_UI_STATE, ...over });

describe('reduce: 暂停语义', () => {
  it('LIVE 中 TOGGLE_PAUSE 翻转 livePaused', () => {
    const s1 = reduce(live(), { type: 'TOGGLE_PAUSE' });
    assert.strictEqual(s1.livePaused, true);
    assert.strictEqual(reduce(s1, { type: 'TOGGLE_PAUSE' }).livePaused, false);
  });

  it('REPLAY 中 TOGGLE_PAUSE 为 no-op（状态引用不变）', () => {
    const s = live({ mode: 'REPLAY' });
    assert.strictEqual(reduce(s, { type: 'TOGGLE_PAUSE' }), s);
  });
});

describe('reduce: 回放进出不触碰 livePaused（核心修复）', () => {
  it('暂停中进回放再退出 → 仍为暂停', () => {
    let s = reduce(live(), { type: 'TOGGLE_PAUSE' });        // livePaused=true
    s = reduce(s, { type: 'ENTER_REPLAY' });
    assert.strictEqual(s.mode, 'REPLAY');
    assert.strictEqual(s.livePaused, true);
    s = reduce(s, { type: 'EXIT_REPLAY' });
    assert.strictEqual(s.mode, 'LIVE');
    assert.strictEqual(s.livePaused, true);
  });

  it('重复 ENTER/EXIT 为 no-op', () => {
    const r = live({ mode: 'REPLAY' });
    assert.strictEqual(reduce(r, { type: 'ENTER_REPLAY' }), r);
    const l = live();
    assert.strictEqual(reduce(l, { type: 'EXIT_REPLAY' }), l);
  });
});

describe('reduce: 路线拾取', () => {
  it('idle + RIGHT_CLICK → picking-goal 并记录起点', () => {
    const s = reduce(live(), { type: 'RIGHT_CLICK', idx: 5 });
    assert.strictEqual(s.routePick, 'picking-goal');
    assert.strictEqual(s.pickStartIdx, 5);
  });

  it('picking-goal + RIGHT_CLICK → 状态引用不变（建路线由 controller 处理）', () => {
    const s = reduce(live(), { type: 'RIGHT_CLICK', idx: 5 });
    assert.strictEqual(reduce(s, { type: 'RIGHT_CLICK', idx: 9 }), s);
  });

  it('REPLAY 中 RIGHT_CLICK 被忽略', () => {
    const s = live({ mode: 'REPLAY' });
    assert.strictEqual(reduce(s, { type: 'RIGHT_CLICK', idx: 5 }), s);
  });

  it('CANCEL 取消选点；idle 时 no-op', () => {
    const picking = reduce(live(), { type: 'RIGHT_CLICK', idx: 5 });
    const cancelled = reduce(picking, { type: 'CANCEL' });
    assert.strictEqual(cancelled.routePick, 'idle');
    assert.strictEqual(cancelled.pickStartIdx, null);
    assert.strictEqual(reduce(cancelled, { type: 'CANCEL' }), cancelled);
  });

  it('ROUTE_SET 回到 idle；SCENARIO_CHANGED 重置选路并记录场景名', () => {
    const picking = reduce(live(), { type: 'RIGHT_CLICK', idx: 5 });
    const done = reduce(picking, { type: 'ROUTE_SET' });
    assert.strictEqual(done.routePick, 'idle');
    assert.strictEqual(done.pickStartIdx, null);

    const changed = reduce(picking, { type: 'SCENARIO_CHANGED', name: 'oval-avoid-1' });
    assert.strictEqual(changed.scenarioName, 'oval-avoid-1');
    assert.strictEqual(changed.routePick, 'idle');
    assert.strictEqual(changed.pickStartIdx, null);
  });
});

describe('reduce: 左侧菜单手风琴（世界/仿真/数据）', () => {
  it('默认全收起（sidebarSection=null）', () => {
    assert.strictEqual(live().sidebarSection, null);
  });

  it('点开分区 → 展开；单开：切换分区时前一个自动收起', () => {
    const s1 = reduce(live(), { type: 'TOGGLE_SECTION', section: 'world' });
    assert.strictEqual(s1.sidebarSection, 'world');
    const s2 = reduce(s1, { type: 'TOGGLE_SECTION', section: 'data' });
    assert.strictEqual(s2.sidebarSection, 'data');
  });

  it('点已展开的分区 → 收起（null）', () => {
    const open = reduce(live(), { type: 'TOGGLE_SECTION', section: 'world' });
    const closed = reduce(open, { type: 'TOGGLE_SECTION', section: 'world' });
    assert.strictEqual(closed.sidebarSection, null);
  });
});

describe('reduce: 右侧面板（信息/调试）', () => {
  it('默认都展开', () => {
    assert.strictEqual(live().infoOpen, true);
    assert.strictEqual(live().debugOpen, true);
  });

  it('TOGGLE_INFO / TOGGLE_DEBUG 各自独立翻转', () => {
    const s1 = reduce(live(), { type: 'TOGGLE_INFO' });
    assert.strictEqual(s1.infoOpen, false);
    assert.strictEqual(s1.debugOpen, true, '互不影响');
    const s2 = reduce(s1, { type: 'TOGGLE_DEBUG' });
    assert.strictEqual(s2.debugOpen, false);
    assert.strictEqual(reduce(s2, { type: 'TOGGLE_INFO' }).infoOpen, true);
  });
});

describe('reduce: 回放运行选择框', () => {
  it('默认关闭；OPEN/CLOSE 切换', () => {
    assert.strictEqual(live().runPickerOpen, false);
    const open = reduce(live(), { type: 'OPEN_RUN_PICKER' });
    assert.strictEqual(open.runPickerOpen, true);
    assert.strictEqual(reduce(open, { type: 'CLOSE_RUN_PICKER' }).runPickerOpen, false);
  });

  it('重复 OPEN/CLOSE 为 no-op（状态引用不变）', () => {
    const open = reduce(live(), { type: 'OPEN_RUN_PICKER' });
    assert.strictEqual(reduce(open, { type: 'OPEN_RUN_PICKER' }), open);
    const closed = reduce(open, { type: 'CLOSE_RUN_PICKER' });
    assert.strictEqual(reduce(closed, { type: 'CLOSE_RUN_PICKER' }), closed);
  });
});

describe('reduce: 速度显示单位', () => {
  it('默认 km/h；TOGGLE_SPEED_UNIT 在 kmh/ms 间翻转', () => {
    assert.strictEqual(live().speedUnit, 'kmh');
    const ms = reduce(live(), { type: 'TOGGLE_SPEED_UNIT' });
    assert.strictEqual(ms.speedUnit, 'ms');
    assert.strictEqual(reduce(ms, { type: 'TOGGLE_SPEED_UNIT' }).speedUnit, 'kmh');
  });
});

describe('deriveHint', () => {
  it('四种分支文案', () => {
    assert.strictEqual(deriveHint(live(), 'NONE'),
      '右键设置路线起点 · 左键查看物体 · Space 暂停 · R 重置视角');
    assert.strictEqual(deriveHint(reduce(live(), { type: 'RIGHT_CLICK', idx: 1 }), 'NONE'),
      '再次右键设置终点（Esc 取消）');
    assert.strictEqual(deriveHint(reduce(live(), { type: 'TOGGLE_PAUSE' }), 'NONE'),
      '已暂停（Space 继续）');
    assert.strictEqual(deriveHint(live({ mode: 'REPLAY' }), 'NONE'),
      '回放中：Space 播放/暂停 · ←/→ 单帧 · 拖动时间轴 · Esc 退出回放');
  });
});
