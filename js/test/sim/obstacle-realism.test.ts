// ==================== 测试：障碍生成器现实性 ====================
//
// 背景（ISSUE_HISTORY 2026-09-19 遗留问题 #4）：旧生成器产出 2.5-6.25m 宽的
// 大方块、随机朝向、恒定压在中心线上——15m 路宽内固有间隙仅 0.6-2m。
// 现实化：尺寸按车辆量级（宽 2-3.5m、长 3-5m）、沿切线朝向 ± 抖动、
// 横向偏置 ±3m。
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { generateTrack } from '../../env/track.js';
import { generateObstacles } from '../../env/obstacle.js';
import { buildFrenetReference, cartesianToFrenet } from '../../algo/plan/frenet.js';
import { MAP_PRESETS } from '../../env/mapDefinitions.js';
import { createRng } from '../../shared/rng.js';
import { m } from '../../shared/dimensions.js';

const track = generateTrack(1800, 800, MAP_PRESETS['oval'].controlRadii);
const ref = buildFrenetReference(track.centerPoints, true);

describe('障碍生成器现实性', () => {
  it('尺寸为车辆量级：宽 8-14、长 12-20（仿真单位 = 2-3.5m × 3-5m）', () => {
    const obs = generateObstacles(track, 8, 8, 14, createRng(11));
    assert.strictEqual(obs.length, 8);
    for (const o of obs) {
      assert.ok((o.width as number) >= 8 - 1e-6 && (o.width as number) <= 14 + 1e-6,
        `宽越界 ${o.width}`);
      assert.ok((o.height as number) >= 12 - 1e-6 && (o.height as number) <= 20 + 1e-6,
        `长越界 ${o.height}`);
    }
  });

  it('横向偏置 ≤ 3m，朝向对齐赛道切线 ± 0.3rad（含投影容差）', () => {
    const obs = generateObstacles(track, 8, 8, 14, createRng(12));
    for (const o of obs) {
      const fl = cartesianToFrenet(o.x, o.y, ref);
      assert.ok(Math.abs(fl.l as number) <= (m(3) as number) + (m(0.3) as number),
        `横向偏置过大 l=${fl.l}`);
      const t = ref.tangents[fl.idx];
      const ta = Math.atan2(t.y, t.x);
      let dAng = Math.abs((o.angle as number) - ta) % (2 * Math.PI);
      if (dAng > Math.PI) dAng = 2 * Math.PI - dAng;
      assert.ok(dAng <= 0.3 + 0.15 + 1e-6, `朝向偏差过大 ${dAng}`);
    }
  });
});
