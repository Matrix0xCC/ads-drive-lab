// ==================== 测试：比例尺与量距的数学 ====================
//
// 单位与方向都极易写错（本会话的教训）。这里钉住：1px 多少米、整数取值、刻度摆法、文字格式。
// 例：fov=45°、视口高 800px、目标离相机 1000 世界单位（=250m，4 单位/米）
//     → 每像素 2·1000·tan22.5°/800 = 1.0355 世界单位 = **0.2589 m**
import { describe, it } from 'node:test';
import assert from 'node:assert';
import * as THREE from 'three';
import {
  metersPerPixel, niceScaleBar, formatMeters, niceRulerInterval, rulerTicks,
} from '../../env/renderer/scale.js';

function cam(fov = 45): THREE.PerspectiveCamera { return new THREE.PerspectiveCamera(fov, 1.5, 10, 20000); }

describe('比例尺：每像素米数', () => {
  it('已知取景 → 0.2589 m/px（横向尺度，与俯仰无关）', () => {
    const c = cam();
    c.position.set(0, 800, 600);                            // 离原点 1000 世界单位
    const target = new THREE.Vector3(0, 0, 0);
    const mpp = metersPerPixel(c, target, 800);
    assert.ok(Math.abs(mpp - 0.2589) < 0.001, `实际 ${mpp.toFixed(4)} m/px`);
  });
  it('距离加倍 → 每像素米数加倍；视口变高 → 每像素米数变小（视野更细）', () => {
    const c = cam();
    c.position.set(0, 0, 1000);
    const t = new THREE.Vector3(0, 0, 0);
    const a = metersPerPixel(c, t, 800);
    c.position.set(0, 0, 2000);
    assert.ok(Math.abs(metersPerPixel(c, t, 800) - 2 * a) < 1e-9, '距离加倍 → 尺度加倍');
    c.position.set(0, 0, 1000);
    assert.ok(Math.abs(metersPerPixel(c, t, 1600) - a / 2) < 1e-9, '视口高加倍 → 尺度减半');
  });
  it('非法输入给 NaN（而不是 Infinity 传到界面上）', () => {
    assert.ok(Number.isNaN(metersPerPixel(cam(), new THREE.Vector3(), 0)));
  });
});

describe('比例尺：整数取值与文字', () => {
  it('0.2589 m/px、限宽 110px → 取 20 m（条宽 ~77px）', () => {
    const { meters, widthPx, label } = niceScaleBar(0.2589, 110);
    assert.strictEqual(meters, 20);
    assert.strictEqual(label, '20 m');
    assert.ok(widthPx > 60 && widthPx <= 110, `条宽 ${widthPx.toFixed(0)}px 应在 (60,110]`);
  });
  it('任何尺度下条宽都不超过上限，且取值只可能是 1/2/5×10ⁿ', () => {
    for (const mpp of [0.01, 0.05, 0.2, 0.5, 1, 3, 12, 50, 400]) {
      const { meters, widthPx } = niceScaleBar(mpp, 110);
      assert.ok(widthPx <= 110.0001, `mpp=${mpp} 条宽 ${widthPx.toFixed(1)}px 超限`);
      const mant = meters / Math.pow(10, Math.floor(Math.log10(meters)));
      assert.ok([1, 2, 5].some(v => Math.abs(mant - v) < 1e-9), `mpp=${mpp} 取值 ${meters} 不是 1/2/5×10ⁿ`);
    }
  });
  it('文字：<10m 给一位小数，10~999m 取整，≥1km 用 km', () => {
    assert.strictEqual(formatMeters(5.24), '5.2 m');
    assert.strictEqual(formatMeters(20.4), '20 m');
    assert.strictEqual(formatMeters(999), '999 m');
    assert.strictEqual(formatMeters(1000), '1.00 km');
    assert.strictEqual(formatMeters(2930), '2.93 km');
    assert.strictEqual(formatMeters(NaN), '—');
  });
});

describe('量距尺：刻度', () => {
  it('刻度间距取整数好看且刻度数接近目标', () => {
    assert.strictEqual(niceRulerInterval(100, 8), 10);     // 100m → 10m 一格
    assert.strictEqual(niceRulerInterval(1000, 8), 100);
    assert.strictEqual(niceRulerInterval(3, 6), 0.5);
  });
  it('刻度沿尺子均布（含两端），且垂直于尺子方向', () => {
    const from = new THREE.Vector3(0, 0.06, 0), to = new THREE.Vector3(400, 0.06, 0);  // 400 单位 = 100m
    const ticks = rulerTicks(from, to, 20);                // 20m = 80 单位 → 5 段 → 6 道
    assert.strictEqual(ticks.length, 6);
    for (const [a, b] of ticks) {
      assert.ok(Math.abs(a.x - b.x) < 1e-9, '尺子沿 +x → 刻度应沿 z 方向（垂直于尺子）');
      assert.ok(Math.abs(Math.abs(a.z - b.z) - 12) < 1e-9, '刻度总长 = 2×tickHalf');
    }
    assert.ok(Math.abs(ticks[0][0].x - 0) < 1e-9 && Math.abs(ticks[5][0].x - 400) < 1e-9, '两端要有刻度');
  });
  it('退化输入（零长/零间距）返回空，不产生 NaN 顶点', () => {
    const p = new THREE.Vector3(0, 0.06, 0);
    assert.deepStrictEqual(rulerTicks(p, p.clone(), 10), []);
    assert.deepStrictEqual(rulerTicks(p, new THREE.Vector3(100, 0.06, 0), 0), []);
  });
});
