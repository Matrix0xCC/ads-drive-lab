// ==================== 测试：坐标网格与刻度尺几何（纯函数 + 组装冒烟） ====================
import { describe, it } from 'node:test';
import assert from 'node:assert';
import * as THREE from 'three';
import { gridSegments, rulerTicks, buildTrackGrid } from '../../env/renderer/geometry.js';

// 世界 450×200m（1800×800 仿真单位）；细网格 10m、主网格 50m
const BOUNDS = { minX: 0, minY: 0, maxX: 1800, maxY: 800 };
const MINOR = 40, MAJOR = 200, TICK = 16;

describe('gridSegments（坐标网格，仿真单位）', () => {
  it('网格线覆盖全域、不含边界；主/次分类按 majorU 判定', () => {
    const segs = gridSegments(BOUNDS, MINOR, MAJOR);
    const verts = segs.filter(s => Math.abs(s.ax - s.bx) < 1e-6);
    const hors = segs.filter(s => Math.abs(s.ay - s.by) < 1e-6);
    assert.strictEqual(verts.length, 1800 / 40 - 1, '竖线数 = 450/10 − 1');
    assert.strictEqual(hors.length, 800 / 40 - 1, '横线数 = 200/10 − 1');
    for (const s of segs) {
      for (const [x, y] of [[s.ax, s.ay], [s.bx, s.by]]) {
        assert.ok(x >= BOUNDS.minX && x <= BOUNDS.maxX && y >= BOUNDS.minY && y <= BOUNDS.maxY,
          `线段出界：(${x},${y})`);
      }
      const coord = Math.abs(s.ax - s.bx) < 1e-6 ? s.ax : s.ay;
      assert.strictEqual(s.major, Math.abs(coord % MAJOR) < 1e-6, `主网格判定错误：coord=${coord}`);
    }
    assert.strictEqual(segs.filter(s => s.major).length, (1800 / 200 - 1) + (800 / 200 - 1), '主网格线数');
  });

  it('按边界偏移生成（非 0 起点也正确）', () => {
    const segs = gridSegments({ minX: 100, minY: 50, maxX: 500, maxY: 250 }, MINOR, MAJOR);
    for (const s of segs) {
      assert.ok(s.ax >= 100 - 1e-6 && s.bx <= 500 + 1e-6 && s.ay >= 50 - 1e-6 && s.by <= 250 + 1e-6);
    }
  });
});

describe('rulerTicks（边缘刻度尺）', () => {
  it('左边缘（x=minX）与下边缘（y=minY）每 majorU 一条，含端点', () => {
    const ticks = rulerTicks(BOUNDS, MAJOR, TICK);
    const left = ticks.filter(t => t.axis === 'y');
    const bottom = ticks.filter(t => t.axis === 'x');
    assert.strictEqual(left.length, 800 / 200 + 1, 'y 轴刻度含端点');
    assert.strictEqual(bottom.length, 1800 / 200 + 1, 'x 轴刻度含端点');
    assert.ok(left.every(t => t.ax === BOUNDS.minX && t.bx === BOUNDS.minX + TICK), '左刻度朝内、贴边');
    assert.ok(bottom.every(t => t.ay === BOUNDS.minY && t.by === BOUNDS.minY + TICK));
    assert.strictEqual(left[0].value, 0);
    assert.strictEqual(left[left.length - 1].value, 800);
  });
});

describe('buildTrackGrid（组装冒烟：真实 THREE，标签工厂可注入）', () => {
  const track: any = {
    centerPoints: [{ x: 100, y: 100 }, { x: 500, y: 400 }],
    innerBoundary: [], outerBoundary: [],
  };
  it('组内：网格线 + 刻度线 + 标签（标签工厂注入，无需 DOM）', () => {
    const labels: string[] = [];
    const labelFactory = (text: string) => {
      labels.push(text);
      return new THREE.Mesh(new THREE.BufferGeometry(), new THREE.MeshBasicMaterial());
    };
    const g = buildTrackGrid(track, labelFactory);
    const lines = g.children.filter(c => (c as any).isLineSegments) as THREE.LineSegments[];
    const meshes = g.children.filter(c => (c as any).isMesh);
    assert.strictEqual(lines.length, 3, '细网格 + 主网格 + 刻度线各一组');
    assert.ok(lines[0].geometry.getAttribute('position').count > 0);
    assert.ok(meshes.length > 0, '应生成标签');
    assert.ok(labels.every(t => /^-?\d+$/.test(t)), `标签应为纯数字：${labels.join(',')}`);
    // 刻度落在 majorU（200u = 50m）的整数倍上 → 米数标签是整十数（此夹具范围内为 50/100）
    for (const expected of ['50', '100']) {
      assert.ok(labels.includes(expected), `应含米数刻度 ${expected}：实际 ${[...new Set(labels)].join(',')}`);
    }
  });
});
