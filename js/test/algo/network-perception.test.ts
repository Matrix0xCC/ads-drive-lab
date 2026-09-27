// ==================== 测试：路网地图的雷达边界（路缘必须挡住射线） ====================
//
// 背景：路网地图的 `outerBoundary` 是空的（可行驶域是"到最近链的距离"，不是多边形），
// 而 `sense()` 的射线求交线段原来就取自内外边界 → 路网上**没有任何可撞的边界**：
// 实测 60/60 条射线飞到 125m 上限（界面上是一圈穿透路缘的扇形，且护栏不再遮挡）。
// 修法：路网按"**我要走的那条路**"（路线）的 ribbon 取边界，并保证这些路缘命中被
// HD Map 背景减除**过滤掉**（否则路口处别的链的端帽会变成幻影障碍）。
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { sense, MAX_SENSOR_DIST } from '../../algo/perception.js';
import { createWorldSnapshot, createRoute, CarState, TrackGeometry } from '../../shared/types.js';
import { rad, m, mps2, px, pxps, dimless } from '../../shared/dimensions.js';

const M = (v: number) => m(v) as number;

/** 一条 200m 直路的开环路网地图（3 车道 → 半宽 5.625m） */
function straightNetwork(): { track: TrackGeometry; route: ReturnType<typeof createRoute> } {
  const pts: any[] = [];
  for (let i = 0; i <= 100; i++) pts.push({ x: px(M(10) + i * M(2)), y: px(M(100)) });
  const route = createRoute(pts, [0], 0, 1, px(M(200)));
  const track: TrackGeometry = {
    centerPoints: pts,
    innerBoundary: [],
    outerBoundary: [],                      // ← 路网：故意为空（可行驶域靠距离判据）
    closed: false,
    halfWidth: px(M(5.625)),
    laneWidth: px(M(3.75)),
    chains: [pts],
    netNodes: [{ x: pts[0].x, y: pts[0].y }],
    netEdges: [{ from: 0, to: 1, chain: 0 }],
  };
  return { track, route };
}

function makeCar(x: number, y: number, angle: number): CarState {
  return {
    x: px(x), y: px(y), angle: rad(angle), speed: pxps(0), steerAngle: rad(0),
    maxSpeed: mps2(0) as any, acceleration: mps2(8), friction: dimless(0.12),
    width: m(2.0), length: m(4.5), collisionRadius: px(Math.hypot(M(2.0), M(4.5)) / 2),
  };
}

describe('路网地图：雷达被路缘挡住（不是飞到 125m）', () => {
  const { track, route } = straightNetwork();
  // 车在路中心、朝 +x；±90° 的射线应撞到路缘（≈半宽 5.625m）
  const snap = createWorldSnapshot(makeCar(M(110), M(100), 0), track, [], route);
  const scan = sense(snap);

  it('侧向射线的原始距离 ≈ 半宽（而不是 125m 上限）', () => {
    const n = scan.angles.length;
    const mid = (n - 1) / 2;
    const left = scan.rawDistances[Math.round(mid + mid / 2)] / 4;    // ≈ +60°
    const right = scan.rawDistances[Math.round(mid - mid / 2)] / 4;
    const halfW = M(5.625) / 4;
    // 60° 方向：路缘在 l=halfW 处 → 距离 = halfW / sin(60°) ≈ 6.5m
    assert.ok(left > 4 && left < 12, `+60° 射线应撞路缘，实际 ${left.toFixed(1)}m`);
    assert.ok(right > 4 && right < 12, `−60° 射线应撞路缘，实际 ${right.toFixed(1)}m`);
  });

  it('路缘命中被背景减除过滤 → 规划用 distances 里没有幻影障碍', () => {
    const maxD = (MAX_SENSOR_DIST as number) / 4;
    const nonMax = scan.distances.filter(d => d / 4 < maxD - 1).length;
    assert.strictEqual(nonMax, 0, `空旷路面不应有障碍物命中，实际 ${nonMax} 条`);
    // 渲染用的原始距离里应有"路缘命中"（否则就是又回到"没有边界"的老 bug）
    const hits = scan.rawDistances.filter(d => d / 4 < maxD - 1).length;
    assert.ok(hits > 10, `原始距离里应有大量路缘命中，实际 ${hits} 条`);
  });
});

// ─────────────────────────────────────────────────────────────
// 逐段宽度：1 车道匝道的路缘必须在 1.875m 处挡住射线
// ─────────────────────────────────────────────────────────────
describe('路网地图：路缘随路段宽度变化（匝道 1.875m vs 主路 5.625m）', () => {
  /** 一条直路：主路 5.625m 半宽（前 100m）+ 1 车道匝道 1.875m（后 100m） */
  function twoWidthNetwork() {
    const pts: any[] = [];
    for (let i = 0; i <= 100; i++) pts.push({ x: px(M(10) + i * M(2)), y: px(M(100)) });
    const spans = [
      { highway: 'trunk', laneCount: 3, laneWidthM: 3.75, halfWidthM: 5.625, maxspeedKph: 100, s0: px(0), s1: px(M(100)) },
      { highway: 'trunk_link', laneCount: 1, laneWidthM: 3.75, halfWidthM: 1.875, maxspeedKph: 50, s0: px(M(100)), s1: px(M(200)) },
    ] as any;
    const route = createRoute(pts, [0], 0, 1, px(M(200)), spans);
    const track: TrackGeometry = {
      centerPoints: pts, innerBoundary: [], outerBoundary: [], closed: false,
      halfWidth: px(M(5.625)), laneWidth: px(M(3.75)),
      chains: [pts],
      chainAttrs: [
        { highway: 'trunk', laneCount: 3, laneWidthM: 3.75, halfWidthM: 5.625, maxspeedKph: 100 },
        { highway: 'trunk_link', laneCount: 1, laneWidthM: 3.75, halfWidthM: 1.875, maxspeedKph: 50 },
      ],
      netNodes: [{ x: pts[0].x, y: pts[0].y }],
      netEdges: [{ from: 0, to: 1, chain: 0 }],
    };
    return { track, route, pts };
  }

  /** 接近 ±60°（FOV 边缘内侧）的侧向射线斜距：撞到路缘时 ≈ 半宽 / sin(56°) */
  function sideRayDist(carXm: number, track: TrackGeometry, route: any): number {
    const snap = createWorldSnapshot(makeCar(M(carXm), M(100), 0), track, [], route);
    const scan = sense(snap);
    const n = scan.angles.length;
    const dl = scan.rawDistances[n - 3] / 4;   // ≈ +56°
    const dr = scan.rawDistances[2] / 4;       // ≈ −56°
    return Math.min(dl, dr);
  }

  it('主路段：侧向射线撞在 5.625m 的路缘上（≈6.5m 斜距）', () => {
    const { track, route } = twoWidthNetwork();
    const d = sideRayDist(60, track, route);            // 主路段（0~100m）
    // 半宽 5.625m / sin(56°) ≈ 6.8m（若边界错成 1.875m 会得到 ~2.2m）
    assert.ok(d > 5.8 && d < 8.5, `主路侧向应≈6.8m，实际 ${d.toFixed(2)}m`);
  });

  it('匝道段：侧向射线撞在 1.875m 的路缘上（≈2.2m 斜距，而不是 6.5m）', () => {
    const { track, route } = twoWidthNetwork();
    const d = sideRayDist(170, track, route);           // 匝道段（>100m + 过渡段之后）
    // 半宽 1.875m / sin(56°) ≈ 2.3m（若仍用地图级 5.625m 会得到 ~6.8m）
    assert.ok(d > 1.6 && d < 3.0, `匝道侧向应≈2.3m，实际 ${d.toFixed(2)}m`);
    // 若仍按地图级 5.625m 建边界，这里会得到 ~6.5m —— 正是本用例要防住的回归
    assert.ok(d < 4, '匝道路缘不能被地图级半宽替代');
  });

  it('命中 HD Map 路缘 → 一定是背景（不因投影误差落进模糊带而变成障碍）', () => {
    const { track, route } = twoWidthNetwork();
    const snap = createWorldSnapshot(makeCar(M(170), M(100), 0), track, [], route);
    const scan = sense(snap);
    const maxD = (MAX_SENSOR_DIST as number) / 4;
    const asObstacle = scan.distances.filter(d => d / 4 < maxD - 1).length;
    assert.strictEqual(asObstacle, 0, '路缘命中不该出现在规划用的 distances 里');
    assert.ok(scan.rawDistances.some(d => d / 4 < 4), '渲染用的原始距离里应能看到匝道路缘');
  });
});
