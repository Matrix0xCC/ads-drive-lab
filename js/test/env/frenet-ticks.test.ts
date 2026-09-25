// ==================== 测试：Frenet 刻度（s 横线 / l 参考线 / 标签） ====================
import { describe, it } from 'node:test';
import assert from 'node:assert';
import * as THREE from 'three';
import { frenetSTicks, frenetLLines, buildFrenetTicksGroup } from '../../env/renderer/geometry.js';
import { buildFrenetReference } from '../../algo/plan/frenet.js';
import { px, m } from '../../shared/dimensions.js';

const straightRef = buildFrenetReference(
  Array.from({ length: 61 }, (_, i) => ({ x: px(i * 40), y: px(0) })), false);   // 直线，间距 10m

describe('frenetSTicks（沿弧长的横线刻度）', () => {
  it('每 10m 一条；每 50m 为 major 并带标签；端点跨度为 ±路面半宽', () => {
    const ticks = frenetSTicks(straightRef, m(200) as number, m(7.5) as number);
    assert.strictEqual(ticks.length, 21, '0..200m 每 10m 一条 = 21 条');
    const majors = ticks.filter(t => t.major);
    assert.strictEqual(majors.length, 5, '每 50m 一条 major（0/50/100/150/200）');
    for (const t of ticks) {
      assert.strictEqual(t.major, Math.abs(t.s % (m(50) as number)) < 1e-6);
      // 直线参考：横线应垂直于路（沿 y 方向），端点 y = ±7.5m
      assert.ok(Math.abs(t.a.x - t.s) < 1e-6 && Math.abs(t.b.x - t.s) < 1e-6, '横线应在同一 s 上');
      assert.ok(Math.abs(t.a.y + (m(7.5) as number)) < 1e-6 && Math.abs(t.b.y - (m(7.5) as number)) < 1e-6,
        '横线应横跨 ±路面半宽');
    }
    for (const t of majors) {
      assert.ok(t.label, 'major 应带标签位置');
      assert.strictEqual(t.label!.text, `s=${(t.s / 4).toFixed(0)}m`);
    }
  });
});

describe('frenetLLines（横向参考线）', () => {
  it('每条线全程保持给定 l；点数随参考线采样', () => {
    const lines = frenetLLines(straightRef, [m(1.875) as number, -(m(5.4) as number)], 0, m(200) as number);
    assert.strictEqual(lines.length, 2);
    for (const ln of lines) {
      assert.ok(ln.points.length > 2);
      for (const p of ln.points) {
        assert.ok(Math.abs(p.y - ln.l) < 1e-6, `直线参考下 l 应为常数（期望 ${ln.l}，实际 ${p.y}）`);
      }
      assert.ok(ln.points[0].x >= 0 && ln.points[ln.points.length - 1].x <= ((m(200) as number) + 1e-6));
    }
  });
});

describe('buildFrenetTicksGroup（渲染组）', () => {
  it('生成线 + 标签 Sprite，且可整体显隐', () => {
    const g = buildFrenetTicksGroup(straightRef, { maxS: m(100) as number, roadHalf: m(7.5) as number, domainHalf: m(5.4) as number });
    const lines = g.children.filter(c => (c as any).isLine);
    const sprites = g.children.filter(c => (c as any).isSprite);
    assert.ok(lines.length >= 10, `应有横线 + l 参考线，实际 ${lines.length}`);
    if (typeof document !== 'undefined') {
      assert.ok(sprites.length >= 2, `50m 处应有数字标签，实际 ${sprites.length}`);
    }   // headless 无 DOM：标签走 document，跳过（浏览器里目检）
    g.visible = false;
    assert.strictEqual(g.visible, false, '整组可显隐（开关用）');
    for (const c of g.children) {
      const pos = (c as any).geometry?.getAttribute?.('position');
      if (!pos) continue;
      for (let i = 0; i < pos.count; i++) assert.ok(pos.getY(i) >= 0.02, '刻度顶点应贴地（y≥0.02）');
    }
    assert.ok(new THREE.Box3().setFromObject(g).isEmpty() === false);
  });
});
