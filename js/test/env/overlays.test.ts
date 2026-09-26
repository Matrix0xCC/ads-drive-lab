// ==================== 测试：走廊可视化覆盖层（真实 THREE，无 WebGL） ====================
//
// 只验证"数据 → 场景对象"的契约（颜色/对象数/顶点数/清理），不涉及 WebGL 渲染。
// three 的核心几何/材质在 Node 下可直接构造（已在 node:test 环境验证）。

import { describe, it } from 'node:test';
import assert from 'node:assert';
import * as THREE from 'three';
import { updateCorridorViz, updateLidar, resetOverlayCache } from '../../env/renderer/overlays.js';
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
      { targetL: 7.5, develop: 48, blocked: false, blockedHard: false, sBlock: Infinity, sBlockHard: Infinity, cost: 1,
        points: [{ s: 0, x: 0, y: 0 }, { s: 1, x: 1, y: 0.2 }, { s: 2, x: 2, y: 0.5 }] },
      { targetL: -8, develop: 48, blocked: true, blockedHard: true, sBlock: 1, sBlockHard: 2, cost: 9,
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
        { targetL: 7.5, develop: 48, blocked: true, blockedHard: true, sBlock: 1, sBlockHard: 2, cost: 9,
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

describe('回放用走廊载荷（corridorVizForReplay）', () => {
  it('用扫描 + 车位姿直接重建载荷（不跑规划器）', async () => {
    const { corridorVizForReplay } = await import('../../algo/plan/planner.js');
    const { buildFrenetReference } = await import('../../algo/plan/frenet.js');
    const { MAX_SENSOR_DIST } = await import('../../algo/perception.js');
    const { px, rad, m } = await import('../../shared/dimensions.js');
    const ref = buildFrenetReference(
      Array.from({ length: 61 }, (_, i) => ({ x: px(i * 40), y: px(0) })), false);
    const angles = [], distances = [];
    for (let i = 0; i < 61; i++) {
      angles.push(rad(-Math.PI / 3 + (i / 60) * 2 * Math.PI / 3));
      distances.push(i === 30 ? px(160) : MAX_SENSOR_DIST);   // 正前方 40m 一个命中点
    }
    const scan = { angles, distances, rawDistances: [...distances], confidences: new Array(61).fill(1), timestamp: 0 };
    const viz = corridorVizForReplay(ref, scan, 0, 0);
    assert.ok(viz.bands.length > 0, '应生成自由带');
    assert.strictEqual(viz.bandObserved.length, viz.bands.length);
    assert.ok(viz.candidates.length >= 1, '应有候选（回放里也能看选路）');
    assert.ok(viz.pinchSegments.length > 0, '40m 处的点障碍应形成瓶颈横断线');
    for (const q of viz.bands) for (const p of q) assert.ok(Number.isFinite(p.x) && Number.isFinite(p.y));
  });
});

describe('LiDAR 射线开关', () => {
  it('scan=null（开关关闭/无数据）→ 已有射线必须全部隐藏，不得残留', () => {
    const scene = new THREE.Scene();
    const lines = [0, 1, 2].map(() => {
      const l = new THREE.Line(
        new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]),
        new THREE.LineBasicMaterial());
      scene.add(l);
      return l;
    });
    const scan = {
      angles: [0, 0.1], distances: [100, 120], rawDistances: [100, 120],
      confidences: [1, 1], timestamp: 0,
    } as any;
    updateLidar(scene, scan, lines, 0, 0);
    assert.ok(lines[0].visible && lines[1].visible, '有数据时应显示');
    assert.strictEqual(lines[2].visible, false, '多余射线池应隐藏');
    updateLidar(scene, null, lines, 0, 0);
    assert.ok(lines.every((l: any) => !l.visible), '关掉开关后不得残留上一帧射线');
  });
});
