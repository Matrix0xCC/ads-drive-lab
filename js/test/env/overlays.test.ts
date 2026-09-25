// ==================== 测试：走廊可视化覆盖层（真实 THREE，无 WebGL） ====================
//
// 只验证"数据 → 场景对象"的契约（颜色/对象数/顶点数/清理），不涉及 WebGL 渲染。
// three 的核心几何/材质在 Node 下可直接构造（已在 node:test 环境验证）。

import { describe, it } from 'node:test';
import assert from 'node:assert';
import * as THREE from 'three';
import { updateCorridorViz, resetOverlayCache } from '../../env/renderer/overlays.js';
import type { PlanningViz } from '../../shared/debug.js';

function makeViz(over: Partial<PlanningViz> = {}): PlanningViz {
  return {
    stepU: 6,
    bands: [[{ x: 0, y: -4 }, { x: 0, y: 4 }, { x: 3, y: 4 }, { x: 3, y: -4 }]],
    bandObserved: [true],
    obsLimitSegments: [],
    unobsBoundary: [],
    pinchSegments: [[{ x: 1, y: -4 }, { x: 1, y: 0.5 }]],
    pinch: { s: 1, width: 1.2, sealed: false },
    candidates: [
      { targetL: 7.5, develop: 48, blocked: false, blockedHard: false, sBlock: Infinity, sBlockHard: Infinity,
        points: [{ s: 0, x: 0, y: 0 }, { s: 1, x: 1, y: 0.2 }, { s: 2, x: 2, y: 0.5 }] },
      { targetL: -8, develop: 48, blocked: true, blockedHard: true, sBlock: 1, sBlockHard: 2,
        points: [{ s: 0, x: 0, y: 0 }, { s: 1, x: 1, y: -0.3 }, { s: 2, x: 2, y: -0.6 }] },
    ],
    bestTargetL: 7.5,
    ...over,
  };
}
const hex = (o: any) => (o.material as THREE.MeshBasicMaterial).color.getHexString();
const lines = (g: THREE.Object3D) => g.children.filter(c => (c as any).isLine) as THREE.Line[];
const meshes = (g: THREE.Object3D) => g.children.filter(c => (c as any).isMesh);

describe('走廊可视化：数据 → 场景对象', () => {
  it('自由带 → 半透明绿色网格（四边形拆两个三角形）；瓶颈 → 横断线（有缝橙/封死红）', () => {
    const scene = new THREE.Scene();
    updateCorridorViz(scene, makeViz());
    const g = scene.children[0] as THREE.Group;
    const mesh = meshes(g)[0] as THREE.Mesh;
    assert.ok(mesh, '应有自由带网格');
    assert.strictEqual(hex(mesh), '22dd77');
    assert.strictEqual((mesh.material as THREE.MeshBasicMaterial).transparent, true);
    assert.strictEqual((mesh.geometry.getAttribute('position') as THREE.BufferAttribute).count, 6, '一个四边形 = 2 三角形 = 6 顶点');
    // 瓶颈横断线（有缝 = 橙）
    assert.ok(lines(g).some(l => hex(l) === 'ffaa33'), '有缝时瓶颈线应为橙');
    updateCorridorViz(scene, makeViz({ pinch: { s: 1, width: 0, sealed: true }, pinchSegments: [[{ x: 1, y: -4.4 }, { x: 1, y: 4.4 }]] }));
    const g2 = scene.children[0] as THREE.Group;
    assert.ok(lines(g2).some(l => hex(l) === 'ff3322'), '封死时瓶颈线应为红');
  });

  it('候选路径：按分级上色（畅通蓝/慢行橙/硬阻塞红），最优亮绿；只有最优画发展段', () => {
    const scene = new THREE.Scene();
    updateCorridorViz(scene, makeViz());
    const g = scene.children[0] as THREE.Group;
    const ls = lines(g);
    // 最优（畅通）→ 亮绿；其发展段（前 develop 米；本夹具 3 点全部 ≤ develop）与路径同色
    const greens = ls.filter(l => hex(l) === '22ff88');
    assert.strictEqual(greens.length, 2, '最优候选：路径 + 发展段各一条，同为亮绿');
    assert.strictEqual(greens[0].geometry.getAttribute('position').count, 3, '最优路径 3 点');
    assert.strictEqual(greens[1].geometry.getAttribute('position').count, 3, '发展段覆盖全部 3 点（develop 足够长）');
    // 非最优：实线段（0.22）+ 淡出段（0.12），没有发展段（0.35 那种），整体更淡
    const red = ls.filter(l => hex(l) === 'ef4444');
    assert.strictEqual(red.length, 2, '非最优：实线段 + 淡出段各一条');
    for (const l of red) {
      assert.ok((l.material as THREE.LineBasicMaterial).opacity <= 0.25, '非最优候选不画发展段且更淡');
    }
  });

  it('未观测带子用灰蓝区分（调亮 + 描边）', () => {
    const scene = new THREE.Scene();
    updateCorridorViz(scene, makeViz({
      bands: [
        [{ x: 0, y: -4 }, { x: 0, y: 4 }, { x: 3, y: 4 }, { x: 3, y: -4 }],
        [{ x: 3, y: -4 }, { x: 3, y: 4 }, { x: 6, y: 4 }, { x: 6, y: -4 }],
      ],
      bandObserved: [true, false],
      obsLimitSegments: [[{ x: 3, y: -4 }, { x: 3, y: 4 }]],
      unobsBoundary: [[{ x: 3, y: -4 }, { x: 6, y: -4 }], [{ x: 6, y: -4 }, { x: 6, y: 4 }]],
    }));
    const g = scene.children[0] as THREE.Group;
    const ms = meshes(g) as THREE.Mesh[];
    const colors = ms.map(m => hex(m)).sort();
    assert.deepStrictEqual(colors, ['22dd77', '6b7a8f'], '观测=绿、未观测=灰蓝');
    assert.ok(lines(g).some(l => hex(l) === '9aa7b5'), '未观测区描边应画出');
  });

  it('候选线在首个阻塞点截断：阻塞段只画到硬阻塞点，之后不再有线', () => {
    const scene = new THREE.Scene();
    updateCorridorViz(scene, makeViz({
      candidates: [
        { targetL: 7.5, develop: 48, blocked: true, blockedHard: true, sBlock: 1, sBlockHard: 2,
          points: [{ s: 0, x: 0, y: 0 }, { s: 1, x: 1, y: 0.2 }, { s: 2, x: 2, y: 0.5 }, { s: 3, x: 3, y: 0.9 }] },
      ],
      bestTargetL: 7.5,
    }));
    const g = scene.children[0] as THREE.Group;
    const ls = lines(g);
    for (const l of ls) {
      const pos = l.geometry.getAttribute('position');
      for (let i = 0; i < pos.count; i++) {
        assert.ok(pos.getX(i) <= 2 + 1e-6, `硬阻塞点（s=2）之后不应再画线（x=${pos.getX(i)}）`);
      }
    }
    // 舒适阻塞点（s=1）之后、硬阻塞点（s=2）之前：应有淡出的续画
    assert.ok(ls.some(l => (l.material as THREE.LineBasicMaterial).opacity <= 0.2 && l.geometry.getAttribute('position').count >= 2),
      '舒适阻塞之后应有淡出段');
  });

  it('viz=null → 清除；resetOverlayCache 也应清掉该组', () => {
    const scene = new THREE.Scene();
    updateCorridorViz(scene, makeViz());
    assert.strictEqual(scene.children.length, 1);
    updateCorridorViz(scene, null);
    assert.strictEqual(scene.children.length, 0, '置空应移除');
    updateCorridorViz(scene, makeViz());
    resetOverlayCache(scene);
    assert.strictEqual(scene.children.length, 0, 'reset 应清理');
  });
});
