// ==================== 测试：共享归档记录层（浏览器 / headless 同源） ====================
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { recordFrom, ARCHIVE_TOPICS, buildWorldFile } from '../../sim/records.js';
import { px, rad } from '../../shared/dimensions.js';

describe('recordFrom（主题 → 归档记录）', () => {
  it('control:state → step 记录（car + control 透传）', () => {
    const r: any = recordFrom('control:state', {
      t: 1.5, step: 90,
      snapshot: { car: { x: px(10), y: px(20), angle: rad(0.5), speed: px(100), steerAngle: rad(0.1) } },
      control: { lookaheadDist: 40, alpha: 0.1, steerAngle: 0.1, throttle: 0.5 },
    });
    assert.strictEqual(r.type, 'step');
    assert.strictEqual(r.step, 90);
    assert.strictEqual(r.car.x, 10);
    assert.strictEqual(r.car.speed, 100);
    assert.strictEqual(r.control.throttle, 0.5);
  });

  it('planning:result → plan 记录（路径候选映射进归档字段）', () => {
    const r: any = recordFrom('planning:result', {
      t: 1, step: 60, state: 'AVOID',
      planning: { frenetS: 100, frenetL: 14, dl_ds: 0.1, headingDiff: 0.05,
        paths: [{ targetL: 7.5, blocked: false, cost: 1 }],
        bestCost: 1, bestTargetL: 7.5, speedTarget: 100, speedMode: 'cruise' },
      trajectory: { points: [{ t: 0, x: 1, y: 2, v: 100 }], type: 'avoid_left', cost: 1 },
    });
    assert.strictEqual(r.type, 'plan');
    assert.strictEqual(r.state, 'AVOID');
    assert.strictEqual(r.frenet.l, 14);
    assert.strictEqual(r.candidates.length, 1);
    assert.strictEqual(r.candidates[0].blocked === undefined, true, '归档字段名为 collision');
    assert.strictEqual(r.candidates[0].collision, false);
    assert.strictEqual(r.best.targetL, 7.5);
    assert.strictEqual(r.best.targetV, 100);
    assert.strictEqual(r.trajectory.type, 'avoid_left');
  });

  it('planning:result 无 debug 快照时字段回退为 0/空', () => {
    const r: any = recordFrom('planning:result', {
      t: 1, step: 60, state: 'CRUISE', planning: null,
      trajectory: { points: [], type: 'lane_keep', cost: 0 },
    });
    assert.strictEqual(r.frenet.s, 0);
    assert.deepStrictEqual(r.candidates, []);
  });

  it('gps:result → err = 读数与真值距离', () => {
    const r: any = recordFrom('gps:result', {
      t: 1, step: 60,
      reading: { x: px(13), y: px(4) }, truth: { x: px(10), y: px(0) },
    });
    assert.strictEqual(r.type, 'gps');
    assert.strictEqual(r.err, 5);
  });

  it('sim:event → event 透传', () => {
    const r: any = recordFrom('sim:event', { t: 1, step: 60, kind: 'arrived', payload: { a: 1 } });
    assert.strictEqual(r.type, 'event');
    assert.strictEqual(r.kind, 'arrived');
    assert.deepStrictEqual(r.payload, { a: 1 });
  });

  it('未知主题 → 抛错', () => {
    assert.throws(() => recordFrom('nope', {}), /未知主题/);
  });

  it('ARCHIVE_TOPICS 覆盖五个发布主题', () => {
    assert.deepStrictEqual(ARCHIVE_TOPICS,
      ['control:state', 'perception:result', 'planning:result', 'gps:result', 'sim:event']);
  });
});

describe('buildWorldFile（真值序列化）', () => {
  it('赛道/障碍/无路线 → WorldFile', () => {
    const track: any = { centerPoints: [{ x: px(1), y: px(2) }], innerBoundary: [], outerBoundary: [] };
    const obstacles: any = [{ x: px(3), y: px(4), width: px(8), height: px(12), angle: rad(0), zHeight: 5 }];
    const scenario: any = { version: 1, name: 't', map: 'oval', seed: 1, duration: 60,
      obstacles: { mode: 'random', count: 0 } };
    const wf = buildWorldFile(track, obstacles, scenario, null);
    assert.strictEqual(wf.map, 'oval');
    assert.deepStrictEqual(wf.track.centerPoints, [[1, 2]]);
    assert.strictEqual(wf.obstacles[0].width, 8);
    assert.strictEqual(wf.route, null);
  });

  it('有路线 → waypoints/索引序列化', () => {
    const track: any = { centerPoints: [{ x: px(1), y: px(2) }], innerBoundary: [], outerBoundary: [] };
    const route: any = { waypoints: [{ x: px(1), y: px(2) }, { x: px(3), y: px(4) }],
      waypointIndices: [0, 1], startIndex: 0, goalIndex: 1, length: px(100) };
    const scenario: any = { version: 1, name: 't', map: 'oval', seed: 1, duration: 60,
      obstacles: { mode: 'random', count: 0 } };
    const wf = buildWorldFile(track, [], scenario, route);
    assert.deepStrictEqual(wf.route!.waypoints, [[1, 2], [3, 4]]);
    assert.strictEqual(wf.route!.goalIndex, 1);
    assert.strictEqual(wf.route!.length, 100);
  });
});
