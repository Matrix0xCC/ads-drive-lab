// ==================== 测试：WASD 相机平移（纯计算） ====================
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { panDelta, WASD_SPEED_FACTOR } from '../../ui/cameraKeys.js';

const near = (a: number, b: number, eps = 1e-9) => Math.abs(a - b) < eps;
const len = (v: number[]) => Math.hypot(v[0], v[1], v[2]);

describe('panDelta（WASD 相机平移）', () => {
  it('无按键 → 零位移', () => {
    assert.deepStrictEqual(panDelta({ cam: [0, 50, 0], target: [0, 0, 0] }, 0, 0, 1 / 60), [0, 0, 0]);
  });

  it('W → 沿"相机→目标"水平方向前进；S 反向；y 恒为 0（贴地平面平移）', () => {
    const o = { cam: [0, 50, 100] as [number, number, number], target: [0, 0, 0] as [number, number, number] };
    const w = panDelta(o, 1, 0, 0.1);
    assert.ok(w[2] < 0 && near(w[1], 0), `W 应朝 -z（向目标）移动，实际 ${w}`);
    const s = panDelta(o, -1, 0, 0.1);
    assert.ok(near(s[0], 0) && near(s[1], 0) && near(s[2], -w[2]), `S 应沿反方向，实际 ${s}`);
  });

  it('D 向右、A 向左（与前进方向正交）', () => {
    const o = { cam: [0, 50, 0] as [number, number, number], target: [0, 0, -100] as [number, number, number] };
    const d = panDelta(o, 0, 1, 0.1);
    assert.ok(d[0] > 0, `正对 -z 时 D 应朝 +x，实际 ${d}`);
    const f = panDelta(o, 1, 0, 0.1);
    assert.ok(near(f[0] * d[0] + f[2] * d[2], 0), '右移与前进正交');
  });

  it('速度随相机距离成比例（拉远走得更快，视觉一致）；俯视正下方不退化', () => {
    const near2 = panDelta({ cam: [0, 20, 0], target: [0, 0, 0] }, 1, 0, 1);
    const far2 = panDelta({ cam: [0, 80, 0], target: [0, 0, 0] }, 1, 0, 1);
    assert.ok(near(len(far2) / len(near2), 80 / 20, 1e-6), `速度应与距离成比例：${len(far2)} vs ${len(near2)}`);
    assert.ok(near(len(near2), 20 * WASD_SPEED_FACTOR, 1e-9));
    const top = panDelta({ cam: [0, 50, 0], target: [0, 0, 0] }, 1, 0, 0.1);   // 正俯视
    assert.ok(Number.isFinite(len(top)) && len(top) > 0, '俯视正下方也应有确定方向');
  });
});
