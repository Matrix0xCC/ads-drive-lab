// ==================== 测试：环境模块随机源注入 ====================
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { generateTrack } from '../../env/track.js';
import { generateObstacles } from '../../env/obstacle.js';
import { localize } from '../../env/localization.js';
import { createCar } from '../../env/car.js';
import { createRng } from '../../shared/rng.js';

describe('随机源注入', () => {
  it('generateTrack 随机模式：同种子 → 同赛道', () => {
    const t1 = generateTrack(1800, 800, undefined, createRng(1));
    const t2 = generateTrack(1800, 800, undefined, createRng(1));
    assert.deepStrictEqual(t1.centerPoints, t2.centerPoints);
    const t3 = generateTrack(1800, 800, undefined, createRng(2));
    assert.notDeepStrictEqual(t1.centerPoints, t3.centerPoints);
  });

  it('generateObstacles：同种子 → 同布局', () => {
    const track = generateTrack(1800, 800);
    const o1 = generateObstacles(track, 4, 10, 25, createRng(9));
    const o2 = generateObstacles(track, 4, 10, 25, createRng(9));
    assert.strictEqual(o1.length, 4);
    assert.deepStrictEqual(o1, o2);
  });

  it('localize：同种子 → 同噪声（ignore timestamp）', () => {
    const car = createCar({ x: 100, y: 100 } as any, { x: 110, y: 100 } as any);
    const r1 = localize(car, { noiseStdDev: 1.25 }, createRng(3));
    const r2 = localize(car, { noiseStdDev: 1.25 }, createRng(3));
    assert.strictEqual(r1.x, r2.x);
    assert.strictEqual(r1.y, r2.y);
  });
});
