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
