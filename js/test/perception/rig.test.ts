// ==================== 测试：多向传感器套件（S2） ====================
//
// 这一层要证明三件事：
//   ① 套件**真的覆盖 360°**（不是把参数写在那儿就算）；
//   ② 正后方的目标：前向套件**看不见**（±60° 之外）、多向套件**看得见**；
//   ③ 加传感器**不改变"该看到的东西"之外的行为**——尤其：只看得到后方的物体
//      不得改变巡航速度（S1① 的"本车道 + 前方"过滤就是为这条存在的）。
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { SimEngine, FIXED_DT } from '../../sim/engine.js';
import { MessageBus } from '../../messageBus.js';
import { MAP_PRESETS } from '../../env/mapDefinitions.js';
import { createWorldSnapshot } from '../../shared/types.js';
import { sense } from '../../algo/perception.js';
import { plan } from '../../algo/plan/index.js';
import { debug } from '../../shared/debug.js';
import { FRONT_ONLY_RIG, SURROUND_SENSOR_RIG, rigRayCount, sensorRayCount } from '../../shared/sensors.js';
import { m, rad, normalizeAngle } from '../../shared/dimensions.js';

/** 造世界：自车在 oval 起点的车道上，**正后方 15m** 放一个障碍 */
function worldWithRearObstacle() {
  const e = new SimEngine(new MessageBus('t', false), { map: MAP_PRESETS['oval'], seed: 42, obstacles: [] });
  for (let i = 0; i < 120; i++) e.step(FIXED_DT);        // 跑 2s 进入稳定
  const back = m(15) as number;
  e.obstacles = [{
    x: (e.car.x as number) - Math.cos(e.car.angle as number) * back,
    y: (e.car.y as number) - Math.sin(e.car.angle as number) * back,
    width: m(2), height: m(4.6), angle: e.car.angle, zHeight: 8,
  } as never];
  return e;
}

describe('S2：多向传感器套件', () => {
  it('覆盖 360°：各传感器的视场并集无缝（含重叠区）', () => {
    // 把每个传感器的朝向区间铺成 [起, 止]，排序后检查首尾相接
    const spans = SURROUND_SENSOR_RIG.map(s => {
      const mid = s.pose.yaw as number;
      return [mid - s.fov / 2, mid + s.fov / 2] as [number, number];
    }).map(([a, b]) => [a - Math.PI, b - Math.PI] as [number, number]);   // 以"后向"为 −π 对齐
    spans.sort((p, q) => p[0] - q[0]);
    let reach = spans[0][1];
    for (const [a, b] of spans.slice(1)) {
      assert.ok(a <= reach + 1e-6, `视场之间有缝：${(a * 180 / Math.PI).toFixed(0)}° > ${(reach * 180 / Math.PI).toFixed(0)}°`);
      reach = Math.max(reach, b);
    }
    assert.ok(reach - spans[0][0] >= Math.PI * 2 - 1e-6, '并集应覆盖整圈 360°');
  });

  it('正后方目标：前向套件看不见（射线只在 ±60°）、多向套件看得见', () => {
    const e = worldWithRearObstacle();
    const snap = createWorldSnapshot(e.car, e.track, e.obstacles, e.route);
    const front = sense(snap, FRONT_ONLY_RIG);
    for (let i = 0; i < front.angles.length; i++) {
      const rel = Math.abs(normalizeAngle(rad(front.angles[i] - (e.car.angle as number))));
      assert.ok(rel <= Math.PI / 3 + 1e-6, '前向套件的射线都在 ±60° 内 ⇒ 正后方天然看不见');
    }
    const sur = sense(snap, SURROUND_SENSOR_RIG);
    let rearHits = 0;
    for (let i = 0; i < sur.angles.length; i++) {
      const rel = Math.abs(normalizeAngle(rad(sur.angles[i] - ((e.car.angle as number) + Math.PI))));
      const d = sur.rawDistances[i] as number;
      if (rel <= Math.PI / 3 && d > m(10) && d < m(20)) rearHits++;      // 15m 处的那个障碍（扣掉投影误差给个范围）
    }
    assert.ok(rearHits > 0, `多向套件应在正后方看到 15m 处的障碍（实际命中 ${rearHits} 条射线）`);
  });

  it('射线数：多向 = 144（61+31+26+26），前向 = 61', () => {
    assert.strictEqual(rigRayCount(FRONT_ONLY_RIG), 61);
    assert.strictEqual(rigRayCount(SURROUND_SENSOR_RIG), 144);
    for (const s of SURROUND_SENSOR_RIG) assert.ok(sensorRayCount(s) > 0);
  });

  it('**加传感器不改变"该看到的东西之外"的行为**：只看得到后方的物体不得压低巡航目标', () => {
    const e = worldWithRearObstacle();
    const snap = createWorldSnapshot(e.car, e.track, e.obstacles, e.route);
    // 看的是**路径层之前**的巡航目标 cruiseV（= 地图限速 ∩ "本车道 + 前方"的最近命中，S1①）：
    // 后向目标在"前方"过滤里天然出局 ⇒ 两套 rig 必须给出同一个巡航目标。
    // 注：**最终** speedTarget 在多向 rig 下会变（后向命中进入走廊 → 路径几何变了）——
    // 这正是多向套件先不设为默认、要等 S3（物体归属）的原因（见 shared/sensors.ts 的说明）。
    plan(snap, sense(snap, FRONT_ONLY_RIG), e.route ?? undefined);
    const vFront = (debug.planning as unknown as { cruiseV: number }).cruiseV;
    plan(snap, sense(snap, SURROUND_SENSOR_RIG), e.route ?? undefined);
    const vSurround = (debug.planning as unknown as { cruiseV: number }).cruiseV;
    assert.strictEqual(vSurround, vFront,
      '后向目标不该影响**巡航目标**（前向 ' + (vFront / 4).toFixed(2) + ' vs 多向 ' + (vSurround / 4).toFixed(2) + ' m/s）');
  });
});
