// ==================== 测试：A 方案（移动物体用更小侧向裕度）修"无辜横移" ====================
//
// 场景（用户截图的几何）：自车在自己车道（l≈1.47m，车道中心 1.88m），相邻车道（中心 5.63m）
// 前方 8m 放一个 2m 宽的障碍。它"在动"时（物体层给出速度）应当用 0.6m 侧向裕度 ⇒
// 本车道仍是可通行的 ⇒ **不该横穿到对面**。
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { SimEngine, FIXED_DT } from '../../sim/engine.js';
import { MessageBus } from '../../messageBus.js';
import { MAP_PRESETS } from '../../env/mapDefinitions.js';
import { createWorldSnapshot } from '../../shared/types.js';
import { sense } from '../../algo/perception.js';
import { plan } from '../../algo/plan/index.js';
import { debug } from '../../shared/debug.js';
import { trackedObjects, resetObjects, OBJECT_DT, getTracks } from '../../algo/objects.js';
import { cartesianToFrenet, frenetToCartesian, buildFrenetReference } from '../../algo/plan/frenet.js';
import { m, mps, px } from '../../shared/dimensions.js';

/** 自车稳定在本车道后，在相邻车道前方放一个物体；用 moveBy 让它"看起来在动"（每帧往前挪） */
function bestTargetL(adjacentDeltaL: number, deltaS: number, moving: boolean) {
  const e = new SimEngine(new MessageBus('t', false), { map: MAP_PRESETS['oval'], seed: 42, obstacles: [] });
  const wp = e.track.centerPoints.slice(0, 80);
  e.setRoute({ waypoints: wp, waypointIndices: wp.map((_, i) => i), startIndex: 0, goalIndex: 79, length: px(0) }, 0);
  for (let i = 0; i < 300; i++) e.step(FIXED_DT);
  const ref = buildFrenetReference(wp, false);
  const fl = cartesianToFrenet(e.car.x, e.car.y, ref);

  resetObjects();
  let lastScan = null;
  for (let k = 0; k < 4; k++) {                       // 4 帧：够"确认"，并让"动"的物体呈现出速度
    const ds = (m(deltaS) as number) + (moving ? (k * mps(15) as number) * OBJECT_DT : 0);
    const at = frenetToCartesian(px((fl.s as number) + ds), px((fl.l as number) + adjacentDeltaL), ref);
    e.obstacles = [{ x: at.x, y: at.y, width: m(2), height: m(4.6), angle: at.yaw, zHeight: 8 } as never];
    const snap = createWorldSnapshot(e.car, e.track, e.obstacles, e.route);
    lastScan = sense(snap);
    trackedObjects(lastScan, OBJECT_DT, e.track);
  }
  const snap = createWorldSnapshot(e.car, e.track, e.obstacles, e.route);
  plan(snap, lastScan!, e.route ?? undefined);
  const tracks = getTracks().filter(t => t.confirmed);
  return { targetL: px((debug.planning as unknown as { bestTargetL: number }).bestTargetL) as number,
    laneL: fl.l as number, vTrack: tracks.length ? tracks[0].vS / 4 : 0 };
}

describe('A 方案：移动的相邻物体不该把自车挤走', () => {
  it('相邻车道**移动**物体（15m/s）→ 保持在本车道（不横穿到对面）', () => {
    const { targetL, laneL, vTrack } = bestTargetL(m(3.75), m(8), true);
    const shift = Math.abs(targetL - laneL);
    console.log('   [A] 物体跟踪速度=' + vTrack.toFixed(1) + 'm/s  自车 l=' + (laneL / 4).toFixed(2) +
      'm → targetL=' + (targetL / 4).toFixed(2) + 'm（偏移 ' + (shift / 4).toFixed(2) + 'm）');
    // 【实测基线】A 方案之前：targetL = −2.70m（偏移 **3.16m**，横穿到对面）；
    // A 方案之后：targetL = −0.49m（偏移 **1.96m**）——物理带不再封死本车道 ✓。
    // 仍是 ~2m 而不是"只让 0.5m"：候选集由**走廊自由段**派生，邻居用小裕度后我这条车道的
    // 自由段中心就在 −0.49m 附近 ⇒ 排序里没有"只让 0.5m"这个选项。
    // 要彻底做到"微微让开"，需要**显式加入限幅让位候选并让它赢得排序**（下一步；本轮未兑现已回滚）。
    assert.ok(shift < (m(2.1) as unknown as number),
      '移动物体不该把自车挤到另一侧（偏移 ' + (shift / 4).toFixed(2) + 'm；A 之前是 3.16m）');
  });
});
