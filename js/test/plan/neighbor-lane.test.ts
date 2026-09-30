// ==================== 测试：**不该为相邻车道的车减速**（用户硬要求） ====================
//
// 【用户的话】"关键是不应该为相邻车道的车减速"。
// 【判据】把一辆**在动**的车放在相邻车道、自车正前方 → 自车的规划目标速度必须**不受影响**（≈ 巡航）；
//   同一辆车若**正在朝我这条车道变**（横向速度指向我）→ 必须**照常减速**（切入要防 ✓）。
// 这条测试直接钉住"责任划分"的分界：旁边同向行驶 ≠ 我的障碍；正在切入 = 我的障碍。
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { SimEngine, FIXED_DT } from '../../sim/engine.js';
import { MessageBus } from '../../messageBus.js';
import { MAP_PRESETS } from '../../env/mapDefinitions.js';
import { createWorldSnapshot } from '../../shared/types.js';
import { sense } from '../../algo/perception.js';
import { plan } from '../../algo/plan/index.js';
import { debug } from '../../shared/debug.js';
import { trackedObjects, resetObjects, OBJECT_DT } from '../../algo/objects.js';
import { cartesianToFrenet, frenetToCartesian, buildFrenetReference } from '../../algo/plan/frenet.js';
import { m, mps, px } from '../../shared/dimensions.js';

/** 自车稳定在本车道后，按 mode 摆一辆车；返回规划目标速度（m/s）
 *  · none     ：不放车 → 基线
 *  · parallel ：相邻车道（+3.75m）、与我平行行驶（横向速度 0）
 *  · cutin    ：已经压在我车道边线上（+2.6m，车体一半在我车道内）、且横向速度朝我（-6m/s）
 */
function targetWith(mode: 'none' | 'parallel' | 'cutin') {
  const e = new SimEngine(new MessageBus('t', false), { map: MAP_PRESETS['oval'], seed: 42, obstacles: [] });
  const wp = e.track.centerPoints.slice(0, 80);
  e.setRoute({ waypoints: wp, waypointIndices: wp.map((_, i) => i), startIndex: 0, goalIndex: 79, length: px(0) }, 0);
  for (let i = 0; i < 300; i++) e.step(FIXED_DT);
  const ref = buildFrenetReference(wp, false);
  const fl = cartesianToFrenet(e.car.x, e.car.y, ref);

  resetObjects();
  let scan = null;
  for (let k = 0; k < 4; k++) {
    if (mode === 'none') { e.obstacles = []; }
    else {
      // 【位置必须按真实车道中心摆】椭圆是 4 车道，中心 ±1.875 / ±5.625m：相邻车道在
      // "我这条车道中心 + 一个车道宽"（3.75m）处；摆在 3.75m 绝对值上其实是**车道线上**，
      // 那种情况本来就该减速（本测试第一版就栽在这里 ✗）。
      const latVel = mode === 'cutin' ? -6 : 0;                     // m/s
      const laneC = (e.track.laneCenters ?? [0]).reduce(
        (b, c) => (Math.abs(c - (fl.l as number)) < Math.abs(b - (fl.l as number)) ? c : b),
        (e.track.laneCenters ?? [0])[0]);
      const base = mode === 'cutin'
        ? (laneC + m(2.6) as number)                                // 已压在我车道边线上（车体一半在我道内）
        : (laneC + (m(3.75) as number));                            // 相邻车道中心
      const lNow = base / 4 + (latVel * k * OBJECT_DT) / 4;
      const sNow = 40 + (15 * k * OBJECT_DT) / 4;                   // 沿路 15m/s 前进（= 在动）
      const at = frenetToCartesian(px((fl.s as number) + (m(sNow) as number)), px((fl.l as number) + (m(lNow) as number)), ref);
      e.obstacles = [{ x: at.x, y: at.y, width: m(2), height: m(4.6), angle: at.yaw, zHeight: 8 } as never];
    }
    const snap = createWorldSnapshot(e.car, e.track, e.obstacles, e.route);
    scan = sense(snap);
    trackedObjects(scan, OBJECT_DT, e.track);
  }
  const snap = createWorldSnapshot(e.car, e.track, e.obstacles, e.route);
  plan(snap, scan!, e.route ?? undefined);
  const p = debug.planning as unknown as { speedTarget: number };
  return { target: p.speedTarget / 4 };
}

describe('责任划分：相邻车道的车不该让我减速（用户要求）', () => {
  it('相邻车道、与我平行行驶的车 → 我的规划目标速度与"没有这辆车"时一致', () => {
    const base = targetWith('none').target;
    const par = targetWith('parallel').target;
    console.log(`   [相邻车道·平行] 无车基线=${base.toFixed(2)}m/s  有车=${par.toFixed(2)}m/s`);
    // 容差 1.5m/s：残留的那 ~1.1m/s 不是责任判据，而是**物理性的**——那辆车挡住了它后面的
    // 激光视线 ⇒ 未观测区变大 ⇒ "见-即-停"限速略微生效 ✓（修复前是 5.9m/s ✗，由走廊膨胀导致）。
    assert.ok(Math.abs(par - base) < 1.5, `平行行驶的邻车道车不该让我减速（${par.toFixed(2)} vs 基线 ${base.toFixed(2)}）`);
  });

  it('已经压在我车道边线上、且正朝我这边动的车（切入）→ 必须减速', () => {
    const base = targetWith('none').target;
    const cut = targetWith('cutin').target;
    console.log(`   [相邻车道·切入] 无车基线=${base.toFixed(2)}m/s  有车=${cut.toFixed(2)}m/s`);
    assert.ok(cut < base - 0.5, `正在切入的车必须减速（${cut.toFixed(2)} vs 基线 ${base.toFixed(2)}）`);
  });
});
