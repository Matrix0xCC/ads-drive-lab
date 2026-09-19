// ==================== 测试：运行归档包（RunBundle）组装与解析 ====================
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { assembleBundle, parseBundle } from '../../headless/bundle.js';
import { FrameRecord } from '../../shared/types.js';
import { px, rad } from '../../shared/dimensions.js';

function straightTrack() {
  const pts: any[] = [];
  for (let i = 0; i <= 200; i++) pts.push({ x: px(i * 10), y: px(0) });
  pts.push(pts[0]);
  return { centerPoints: pts, innerBoundary: [], outerBoundary: [] };
}

const SCENARIO: any = { version: 1, name: 't1', map: 'oval', seed: 7, duration: 60,
  obstacles: { mode: 'random', count: 0 } };

function stepRec(step: number, x: number, y: number): FrameRecord {
  return { type: 'step', t: step / 60, step,
    car: { x, y, angle: 0, speed: 100, steerAngle: 0 }, control: null };
}

describe('assembleBundle（浏览器导出与 CLI 共用的组装）', () => {
  it('含碰撞帧 → verdict=ISSUES、issues/report 齐全、世界真值序列化', () => {
    const obstacles: any[] = [{ x: px(100), y: px(0), width: px(8), height: px(12), angle: rad(0) }];
    const bundle = assembleBundle({
      scenario: SCENARIO, track: straightTrack() as any, obstacles,
      route: null, frames: [stepRec(1, 0, 0), stepRec(2, 100, 0)],   // 第二帧与障碍相交
      simTime: 0.05, id: 'run-x', createdAt: '2026-09-19T00:00:00Z',
      gitCommit: null, nodeVersion: 'test',
    });
    assert.strictEqual(bundle.version, 1);
    assert.strictEqual(bundle.meta.verdict, 'ISSUES');
    assert.strictEqual(bundle.meta.issueCounts.high, 1);
    assert.ok(bundle.issues.some(i => i.check === 'collision_true'));
    assert.ok(bundle.reportMd.includes('判定'));
    assert.strictEqual(bundle.meta.steps, 2);
    assert.strictEqual(bundle.world.map, 'oval');
    assert.strictEqual(bundle.world.obstacles.length, 1);
    assert.strictEqual(bundle.world.track.centerPoints.length, 202);   // 201 点 + 闭合点
  });

  it('干净帧（远离障碍）→ verdict=PASS', () => {
    const obstacles: any[] = [{ x: px(100), y: px(500), width: px(8), height: px(12), angle: rad(0) }];
    const bundle = assembleBundle({
      scenario: SCENARIO, track: straightTrack() as any, obstacles,
      route: null, frames: [stepRec(1, 0, 0), stepRec(2, 10, 0)],
      simTime: 0.05, id: 'run-y', createdAt: 'x', gitCommit: null, nodeVersion: 'test',
    });
    assert.strictEqual(bundle.meta.verdict, 'PASS');
    assert.strictEqual(bundle.issues.length, 0);
  });

  it('JSON 往返（导出→导入回放路径）保持问题数据一致', () => {
    const obstacles: any[] = [{ x: px(100), y: px(0), width: px(8), height: px(12), angle: rad(0), zHeight: 5 }];
    const bundle = assembleBundle({
      scenario: SCENARIO, track: straightTrack() as any, obstacles,
      route: null, frames: [stepRec(2, 100, 0)],
      simTime: 0.05, id: 'run-z', createdAt: 'x', gitCommit: null, nodeVersion: 'test',
    });
    const round = parseBundle(JSON.parse(JSON.stringify(bundle)));
    assert.deepStrictEqual(round.issues, bundle.issues);
    assert.deepStrictEqual(round.world, bundle.world);
    assert.strictEqual(round.frames.length, bundle.frames.length);
  });
});

describe('parseBundle（导入校验）', () => {
  it('拒绝非对象 / 版本不符 / 缺 frames', () => {
    assert.throws(() => parseBundle(null), /归档/);
    assert.throws(() => parseBundle({ version: 2 }), /归档/);
    assert.throws(() => parseBundle({ version: 1, meta: {}, world: {}, frames: 'x' }), /归档/);
  });
});
