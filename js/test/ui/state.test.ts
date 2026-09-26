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
  it('T 进入模式 + PICK_POINT → picking-goal 并记录起点', () => {
    const s = reduce(reduce(live(), { type: 'TOGGLE_ROUTE_SET' }), { type: 'PICK_POINT', idx: 5 });
    assert.strictEqual(s.routePick, 'picking-goal');
    assert.strictEqual(s.pickStartIdx, 5);
  });

  it('picking-goal + PICK_POINT → 状态引用不变（建路线由 controller 处理）', () => {
    const s = reduce(reduce(live(), { type: 'TOGGLE_ROUTE_SET' }), { type: 'PICK_POINT', idx: 5 });
    assert.strictEqual(reduce(s, { type: 'PICK_POINT', idx: 9 }), s);
  });

  it('未进入模式时 PICK_POINT 无效（不再有"随手右键就设起点"）', () => {
    const l = live();
    assert.strictEqual(reduce(l, { type: 'PICK_POINT', idx: 5 }), l);
  });

  it('TOGGLE_ROUTE_SET 再按一次 → 退出模式', () => {
    const on = reduce(live(), { type: 'TOGGLE_ROUTE_SET' });
    assert.strictEqual(on.routePick, 'picking-start');
    assert.strictEqual(reduce(on, { type: 'TOGGLE_ROUTE_SET' }).routePick, 'idle');
  });

  it('REPLAY 中进入模式/PICK_POINT 均被忽略', () => {
    const s = live({ mode: 'REPLAY' });
    assert.strictEqual(reduce(s, { type: 'TOGGLE_ROUTE_SET' }), s);
    assert.strictEqual(reduce(s, { type: 'PICK_POINT', idx: 5 }), s);
  });

  it('CANCEL 取消选点；idle 时 no-op', () => {
    const picking = reduce(reduce(live(), { type: 'TOGGLE_ROUTE_SET' }), { type: 'PICK_POINT', idx: 5 });
    const cancelled = reduce(picking, { type: 'CANCEL' });
    assert.strictEqual(cancelled.routePick, 'idle');
    assert.strictEqual(cancelled.pickStartIdx, null);
    assert.strictEqual(reduce(cancelled, { type: 'CANCEL' }), cancelled);
  });

  it('ROUTE_SET 回到 idle；SCENARIO_CHANGED 重置选路并记录场景名', () => {
    const picking = reduce(reduce(live(), { type: 'TOGGLE_ROUTE_SET' }), { type: 'PICK_POINT', idx: 5 });
    const done = reduce(picking, { type: 'ROUTE_SET' });
    assert.strictEqual(done.routePick, 'idle');
    assert.strictEqual(done.pickStartIdx, null);

    const changed = reduce(picking, { type: 'SCENARIO_CHANGED', name: 'oval-avoid-1' });
    assert.strictEqual(changed.scenarioName, 'oval-avoid-1');
    assert.strictEqual(changed.routePick, 'idle');
    assert.strictEqual(changed.pickStartIdx, null);
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

describe('reduce: 相机跟随开关', () => {
  it('默认关；TOGGLE_FOLLOW_CAM 翻转；重复两次回到原值', () => {
    assert.strictEqual(live().followCam, false);
    const on = reduce(live(), { type: 'TOGGLE_FOLLOW_CAM' });
    assert.strictEqual(on.followCam, true);
    assert.strictEqual(reduce(on, { type: 'TOGGLE_FOLLOW_CAM' }).followCam, false);
  });

  it('SCENARIO_CHANGED 带 followCamDefault → 按新世界尺寸设默认（长条图开/小世界关）', () => {
    const big = reduce(live(), { type: 'SCENARIO_CHANGED', name: 'osm', followCamDefault: true });
    assert.strictEqual(big.followCam, true);
    const small = reduce(big, { type: 'SCENARIO_CHANGED', name: 'oval', followCamDefault: false });
    assert.strictEqual(small.followCam, false);
  });

  it('SCENARIO_CHANGED 不带 followCamDefault → 保留用户当前选择', () => {
    const on = reduce(live(), { type: 'TOGGLE_FOLLOW_CAM' });
    const after = reduce(on, { type: 'SCENARIO_CHANGED', name: 'x' });
    assert.strictEqual(after.followCam, true, '未给默认值不应改掉用户的开关');
    assert.strictEqual(after.scenarioName, 'x');
    assert.strictEqual(after.routePick, 'idle');
  });
});

describe('deriveHint', () => {
  it('各分支文案（含路线模式两段）', () => {
    assert.strictEqual(deriveHint(live(), 'NONE'),
      'T 设路线 · F 跟随视角 · WASD 平移 · 左键查看物体 · Space 暂停 · R 重置视角');
    assert.strictEqual(deriveHint(reduce(live(), { type: 'TOGGLE_ROUTE_SET' }), 'NONE'),
      '设起点：左键点击赛道（Esc 取消）');
    assert.strictEqual(deriveHint(reduce(reduce(live(), { type: 'TOGGLE_ROUTE_SET' }), { type: 'PICK_POINT', idx: 1 }), 'NONE'),
      '设终点：左键点击赛道（Esc 取消）');
    assert.strictEqual(deriveHint(reduce(live(), { type: 'TOGGLE_PAUSE' }), 'NONE'),
      '已暂停（Space 继续）');
    assert.strictEqual(deriveHint(live({ mode: 'REPLAY' }), 'NONE'),
      '回放中：Space 播放/暂停 · ←/→ 单帧 · 拖动时间轴 · Esc 退出回放');
  });
});

describe('reduce: 走廊可视化开关', () => {
  it('默认关；TOGGLE_CORRIDOR 翻转；重复两次回到原值', () => {
    assert.strictEqual(INITIAL_UI_STATE.corridorVisible, false);
    const on = reduce(live(), { type: 'TOGGLE_CORRIDOR' });
    assert.strictEqual(on.corridorVisible, true);
    assert.strictEqual(reduce(on, { type: 'TOGGLE_CORRIDOR' }).corridorVisible, false);
  });
});

describe('reduce: Frenet 刻度开关', () => {
  it('默认开；TOGGLE_FRENET_TICKS 翻转（与走廊开关互不影响）', () => {
    assert.strictEqual(INITIAL_UI_STATE.frenetTicksVisible, true);
    const off = reduce(live(), { type: 'TOGGLE_FRENET_TICKS' });
    assert.strictEqual(off.frenetTicksVisible, false);
    assert.strictEqual(off.corridorVisible, false, '不应联动走廊开关');
    assert.strictEqual(reduce(off, { type: 'TOGGLE_CORRIDOR' }).frenetTicksVisible, false, '走廊开关不影响刻度');
    assert.strictEqual(reduce(off, { type: 'TOGGLE_FRENET_TICKS' }).frenetTicksVisible, true);
  });
});

describe('reduce: LiDAR 射线开关', () => {
  it('默认开；TOGGLE_LIDAR 翻转，且不影响走廊/刻度开关', () => {
    assert.strictEqual(INITIAL_UI_STATE.lidarVisible, true);
    const off = reduce(live(), { type: 'TOGGLE_LIDAR' });
    assert.strictEqual(off.lidarVisible, false);
    assert.strictEqual(off.corridorVisible, false);
    assert.strictEqual(off.frenetTicksVisible, true);
    assert.strictEqual(reduce(off, { type: 'TOGGLE_LIDAR' }).lidarVisible, true);
  });
});
