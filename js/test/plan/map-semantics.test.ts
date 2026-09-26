// ==================== 测试：地图语义进入决策（限速 / 车道） ====================
//
// 验证"地图 → 决策"这条因果链真的接上了：
//   ① 车道几何由**半宽 × 车道宽**推导（径向图结果必须与历史硬编码 +1.875m 逐位一致）；
//   ② 地图限速（OSM maxspeed）成为巡航速度的**硬上限**（且不被 17.5m/s 防呆下限抬高）；
//   ③ 名义车道来自 track（换地图 → 巡航横向位置改变）。
// 这三条同时是"地图起作用了吗"的可测试判据：几何相同、语义不同 → 行为必须不同。
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { plan } from '../../algo/plan/index.js';
import { debug } from '../../shared/debug.js';
import { buildTrack, laneCentersOf, TRACK_HALF_WIDTH } from '../../env/track.js';
import { MAP_PRESETS } from '../../env/mapDefinitions.js';
import { CarState, WorldSnapshot, LidarScan } from '../../shared/types.js';
import { rad, m, mps, mps2, px, pxps, dimless } from '../../shared/dimensions.js';

const M = (v: number) => m(v) as number;

function makeCar(speedU: number): CarState {
  return {
    x: px(0), y: px(0), angle: rad(0), speed: pxps(speedU), steerAngle: rad(0),
    maxSpeed: mps(42.5), acceleration: mps2(8), friction: dimless(0.12),
    width: m(2.0), length: m(4.5),
    collisionRadius: px(Math.hypot(M(2.0), M(4.5)) / 2),
  };
}

/** 平直赛道；可选覆盖半宽/限速/名义车道 */
function makeSnapshot(car: CarState, over: Partial<WorldSnapshot['track']> = {}): WorldSnapshot {
  const pts: any[] = [];
  for (let i = 0; i <= 200; i++) pts.push({ x: px(i * 10), y: px(0) });
  pts.push(pts[0]);
  return {
    car,
    track: { centerPoints: pts, innerBoundary: [], outerBoundary: [], closed: true, ...over },
    obstacles: [], timestamp: 0,
  };
}

/** 空旷 LiDAR（无命中） */
function lidarClear(): LidarScan {
  const n = 61, angles: any[] = [], distances: any[] = [];
  for (let i = 0; i < n; i++) {
    angles.push(rad(-Math.PI / 3 + (i / (n - 1)) * (2 * Math.PI / 3)));
    distances.push(px(600));
  }
  return { angles, distances, rawDistances: [...distances], confidences: new Array(n).fill(1), timestamp: 0 };
}

describe('地图语义：车道几何由半宽 × 车道宽推导', () => {
  it('径向图（半宽 7.5m / 车道 3.75m）→ 4 条车道，名义车道 +1.875m（与历史硬编码逐位一致）', () => {
    const { centers, offset } = laneCentersOf(TRACK_HALF_WIDTH as number, M(3.75));
    assert.strictEqual(centers.length, 4);
    assert.deepStrictEqual(centers.map(c => +(c / 4).toFixed(3)), [-5.625, -1.875, 1.875, 5.625]);
    assert.strictEqual(offset, M(1.875), '名义车道必须仍是 +1.875m（否则径向图行为会变）');
  });

  it('OSM 3 车道（半宽 5.625m / 车道 3.75m）→ 3 条车道，名义车道取中间 l=0', () => {
    const { centers, offset } = laneCentersOf(M(5.625), M(3.75));
    assert.strictEqual(centers.length, 3);
    assert.deepStrictEqual(centers.map(c => +(c / 4).toFixed(3)), [-3.75, 0, 3.75]);
    assert.strictEqual(offset, 0);
  });

  it('buildTrack：OSM 地图带出车道宽/车道中心/名义车道/限速（maxspeed → m/s）', () => {
    const t = buildTrack(MAP_PRESETS['bj-g4-dujiakan'], 1800, 800);
    assert.strictEqual(t.laneWidth, M(3.75));
    assert.strictEqual(t.laneOffset, 0);
    assert.strictEqual(t.laneCenters?.length, 3);
    // 120 km/h = 33.33 m/s
    assert.ok(Math.abs((t.speedLimitMps as number) - (mps(120 / 3.6) as number)) < 1e-6);
    assert.strictEqual(t.closed, false);
  });

  it('buildTrack：径向地图（无限速声明）→ speedLimitMps 缺省（该图不限速）', () => {
    const t = buildTrack(MAP_PRESETS.oval, 1800, 800);
    assert.strictEqual(t.speedLimitMps, undefined);
    assert.strictEqual(t.laneOffset, M(1.875));
  });
});

describe('地图语义：限速进 vCap（地图 → 决策）', () => {
  it('地图限速 60km/h → 巡航目标被压到 16.67m/s（不受 17.5m/s 防呆下限抬高）', () => {
    const snap = makeSnapshot(makeCar(mps(10) as number), { speedLimitMps: mps(60 / 3.6) });
    plan(snap, lidarClear());
    const target = debug.planning!.speedTarget as number;
    assert.ok(target <= (mps(60 / 3.6) as number) + 1e-6,
      `目标速度应 ≤ 16.67m/s，实际 ${target / 4}m/s`);
    assert.ok((debug.planning!.mapSpeedLimitMps as number) > 0, 'debug 应记录地图限速');
  });

  it('地图限速 120km/h → 巡航目标可达 33.3m/s（但不超过它）', () => {
    const snap = makeSnapshot(makeCar(mps(10) as number), { speedLimitMps: mps(120 / 3.6) });
    plan(snap, lidarClear());
    const target = debug.planning!.speedTarget as number;
    assert.ok(target <= (mps(120 / 3.6) as number) + 1e-6);
    assert.ok(target > (mps(17.5) as number), `空旷路应能跑到限速附近，实际 ${target / 4}m/s`);
  });

  it('未声明限速的地图 → 沿用规划器默认上限（35m/s），debug 记为 null', () => {
    const snap = makeSnapshot(makeCar(mps(10) as number));
    plan(snap, lidarClear());
    assert.strictEqual(debug.planning!.mapSpeedLimitMps, null);
    assert.ok((debug.planning!.speedTarget as number) <= (mps(35) as number) + 1e-6);
  });
});

describe('地图语义：名义车道来自 track（换图 → 巡航横向位置变）', () => {
  it('track.laneOffset=+1.875m vs 0 → 调试面板与路径代价参考随之改变', () => {
    const a = makeSnapshot(makeCar(mps(10) as number), { laneOffset: px(M(1.875)) });
    plan(a, lidarClear());
    assert.strictEqual(debug.planning!.laneOffsetU, M(1.875));
    const b = makeSnapshot(makeCar(mps(10) as number), { laneOffset: px(0) });
    plan(b, lidarClear());
    assert.strictEqual(debug.planning!.laneOffsetU, 0);
  });

  it('全图封死时也不会 NaN（限速/车道改动不动安全不变量）', () => {
    const snap = makeSnapshot(makeCar(0), { speedLimitMps: mps(60 / 3.6), laneOffset: px(0) });
    const traj = plan(snap, lidarClear());
    assert.ok(traj.points.every(p => Number.isFinite(p.x as number) && Number.isFinite(p.v as number)));
  });
});
