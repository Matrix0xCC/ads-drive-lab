// ==================== 测试：裕度标定（合成样本） ====================
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { calibrate, CalibSample } from '../../headless/calibrate-margin.js';
import { createRng } from '../../shared/rng.js';

/** 合成样本：e = (m0 + k·κ·v²)·(1 ± noise)（项目 RNG，确定性） */
function synth(m0: number, k: number, noise: number): CalibSample[] {
  const rng = createRng(12345);
  const samples: CalibSample[] = [];
  for (let run = 0; run < 10; run++) {
    for (let i = 0; i < 400; i++) {
      const v = rng() * 30;
      const kappa = rng() * 0.01;
      const e = m0 + k * kappa * v * v;
      samples.push({ vMps: v, kappa, eM: e * (1 + (rng() * 2 - 1) * noise), run: `run-${run}` });
    }
  }
  return samples;
}

describe('calibrate（分箱分位 + 加权拟合）', () => {
  it('恢复已知系数（±30% 噪声下，m₀/k 容差 30%）', () => {
    const r = calibrate(synth(0.45, 0.05, 0.3));
    assert.ok(Math.abs(r.m0 - 0.45) < 0.15, `m₀=${r.m0}`);
    assert.ok(Math.abs(r.k - 0.05) / 0.05 < 0.3, `k=${r.k}`);
  });

  it('无曲率项（k=0）→ 拟合 k 接近 0、m₀ 接近常数误差', () => {
    const r = calibrate(synth(0.5, 0, 0.1));
    assert.ok(Math.abs(r.m0 - 0.5) < 0.1, `m₀=${r.m0}`);
    assert.ok(r.k < 0.01, `k=${r.k} 应接近 0`);
  });

  it('覆盖率：模型覆盖留出集 95 分位样本 → 覆盖率 ≥90%', () => {
    const r = calibrate(synth(0.45, 0.05, 0.05));
    assert.ok(r.coverage >= 0.9, `覆盖率=${r.coverage}`);
    assert.ok(r.bins.length > 0, '应产出分箱');
    assert.ok(r.bins.every(b => b.n >= 20));
  });

  it('确定性：同输入两次结果一致', () => {
    const s = synth(0.45, 0.05, 0.2);
    assert.deepStrictEqual(calibrate(s), calibrate(s));
  });
});
