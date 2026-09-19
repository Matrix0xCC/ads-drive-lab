// ==================== 测试：路径层（SL 空间） ====================
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { solvePathLayer, PATH_HORIZON } from '../../algo/plan/paths.js';
import { buildFrenetReference } from '../../algo/plan/frenet.js';
import { CarState, LidarScan } from '../../shared/types.js';
import { Px, rad, m, mps, mps2, px, pxps, dimless } from '../../shared/dimensions.js';
import { MAX_SENSOR_DIST } from '../../algo/perception.js';

const LO = m(1.875) as number;

function straightRef() {
  const pts: any[] = [];
  for (let i = 0; i <= 600; i++) pts.push({ x: px(i * 10), y: px(0) });
  return buildFrenetReference(pts, true);
}
function makeCar(x: number, y: number, angle: number, speed: number): CarState {
  return { x: px(x), y: px(y), angle: rad(angle), speed: pxps(speed), steerAngle: rad(0),
    maxSpeed: mps(42.5), acceleration: mps2(8), friction: dimless(0.12),
    width: m(2.0), length: m(4.5), collisionRadius: px(Math.hypot(8, 18) / 2) };
}
function clearScan(): LidarScan {
  const n = 61, angles: any[] = [], distances: any[] = [];
  for (let i = 0; i < n; i++) { angles.push(rad(-Math.PI/3 + (i/(n-1))*2*Math.PI/3)); distances.push(MAX_SENSOR_DIST); }
  return { angles, distances, rawDistances: [...distances], confidences: new Array(61).fill(1), timestamp: 0 };
}
function blockedScan(dist: number, arcHalf = 0.05): LidarScan {
  const n = 61, angles: any[] = [], distances: any[] = [];
  for (let i = 0; i < n; i++) {
    const a = -Math.PI/3 + (i/(n-1))*2*Math.PI/3;
    angles.push(rad(a));
    distances.push(Math.abs(a) < arcHalf ? px(dist) : MAX_SENSOR_DIST);
  }
  return { angles, distances, rawDistances: [...distances], confidences: new Array(61).fill(1), timestamp: 0 };
}

function solve(scan: LidarScan, opts: { s0?: number; l0?: number; prev?: number | null } = {}) {
  // 直线参考线从 x=0 起算、间距 10u：车与 s0 必须一致（车在参考线该弧长处）
  const car = makeCar(opts.s0 ?? 0, 0, 0, 100);
  return solvePathLayer({
    ref: straightRef(), l0: px(opts.l0 ?? 0), dl0: dimless(0), s0: opts.s0 ?? 0,
    carX: car.x as number, carY: car.y as number, lidarScan: scan,
    carWidth: car.width as Px, collisionRadius: car.collisionRadius as Px,
    laneOffset: px(LO), prevTargetL: opts.prev ?? null,
  });
}

describe('路径层：候选生成与域约束', () => {
  it('全部候选在可行驶域内，采样覆盖整个视程', () => {
    const r = solve(clearScan());
    assert.ok(r.ranked.length >= 3);
    for (const p of r.ranked) {
      assert.ok(Math.abs(p.targetL as number) <= 17.6 + 1e-6, `越界 targetL=${p.targetL}`);
      assert.strictEqual(p.points.length, 41);   // PATH_STEPS + 1
      assert.ok((p.points[40].s - p.points[0].s) > (PATH_HORIZON as number) - 1);
    }
  });

  it('畅通：最优为车道中心路径（代价最低）', () => {
    const r = solve(clearScan());
    assert.strictEqual(r.allBlocked, false);
    assert.ok(Math.abs((r.best.targetL as number) - LO) < 0.5, `best=${r.best.targetL}`);
    assert.strictEqual(r.best.blocked, false);
  });
});

describe('路径层：碰撞判定与选路', () => {
  it('中央窄障碍（100u）：本车道阻塞，最优选侧向路径', () => {
    const r = solve(blockedScan(100, 0.025));
    assert.strictEqual(r.allBlocked, false);
    assert.strictEqual(r.best.blocked, false);
    assert.ok(Math.abs((r.best.targetL as number) - LO) > 2 * 4, '应选侧向路径'); // >2m
    const inLane = r.ranked.find(p => Math.abs((p.targetL as number) - LO) < 1);
    assert.ok(inLane && inLane.blocked, '本车道应被判阻塞');
    // 判阻塞发生在"路径进入命中点 CLEAR(2.5m) 范围内"处：sBlock ≈ 60-130
    assert.ok(inLane.sBlock >= 60 && inLane.sBlock <= 130, `sBlock 应≈60-130，实际 ${inLane.sBlock}`);
  });

  it('近距宽障碍（60u）全阻塞：allBlocked=true，按最晚碰撞排序', () => {
    const r = solve(blockedScan(60));
    assert.strictEqual(r.allBlocked, true);
    assert.strictEqual(r.best.blocked, true);
    const ord = r.ranked.map(p => p.sBlock);
    for (let i = 1; i < ord.length; i++) assert.ok(ord[i] <= ord[i - 1], '应按 sBlock 降序');
    assert.ok(r.sBlock === ord[0]);
    assert.strictEqual(r.allBlockedFar, true, '近距全阻塞必然全程全阻塞');
    assert.strictEqual(r.sBlockFar, r.best.sBlockFar, '全程停车基准=最晚碰撞候选');
  });
});

describe('路径层：提交窗口（横向耐心）', () => {
  it('窗口外障碍（300u=75m）→ 本车道不判近距阻塞；全程记录远距阻塞；停车线不触发', () => {
    const r = solve(blockedScan(300, 0.025));
    const inLane = r.ranked.find(p => Math.abs((p.targetL as number) - LO) < 1)!;
    assert.ok(inLane && !inLane.blocked, '窗口外不应判近距阻塞（不提前选边）');
    assert.ok(inLane.sBlockFar < Infinity, '但全程应记录远距阻塞（供停车线）');
    assert.strictEqual(r.allBlocked, false);
    assert.strictEqual(r.best.targetL, inLane.targetL, '近距全清 → 最优应为本车道');
    assert.strictEqual(r.allBlockedFar, false, '侧向候选全程畅通 → 停车线不触发');
    assert.strictEqual(r.sBlockFar, Infinity);
  });

  it('窗口内障碍（200u=50m）→ 本车道判阻塞，最优选侧', () => {
    const r = solve(blockedScan(200, 0.025));
    const inLane = r.ranked.find(p => Math.abs((p.targetL as number) - LO) < 1)!;
    assert.ok(inLane && inLane.blocked, '窗口内应判阻塞');
    assert.ok(Math.abs((r.best.targetL as number) - LO) > 8, '应选侧向路径');   // >2m
  });
});
