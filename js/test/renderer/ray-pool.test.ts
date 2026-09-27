// ==================== 测试：LiDAR 射线池的渲染不变量 ====================
//
// 钉住一个"只有真跑才会暴露"的坑：射线池是预创建 + 每帧原地改顶点，而 Three.js 的包围球
// 只在第一次用到时算一次 → 用全 0 顶点创建时，包围球退化成世界原点的一个点，之后永不重算。
// 若开着视锥剔除，射线就会随相机平移/缩放时而出现时而消失（"动一下视角出来了，再动又没了"）。
// 因此池子里每一条都必须 `frustumCulled === false`。
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { createLidarRayPool } from '../../env/renderer/rayPool.js';

describe('renderer/rayPool：射线池不变量', () => {
  it('每条射线都关掉视锥剔除（否则过期包围球会造成视图相关的"闪没"）', () => {
    const { lines, parent } = createLidarRayPool(61);
    assert.strictEqual(lines.length, 61);
    assert.strictEqual(parent.children.length, 61);
    for (const l of lines) {
      assert.strictEqual(l.frustumCulled, false, '射线池必须 frustumCulled=false');
      assert.strictEqual(l.visible, false, '初始不可见（等第一帧数据）');
    }
  });

  it('每条射线是可原地改顶点的 6 浮点线段（池化契约）', () => {
    const { lines } = createLidarRayPool(3);
    for (const l of lines) {
      const pos = l.geometry.getAttribute('position');
      assert.strictEqual(pos.count, 2, '线段 = 2 个顶点');
      assert.strictEqual(pos.itemSize, 3);
      // 原地改顶点后仍是同一条（不需要重建几何）
      (pos.array as Float32Array).set([1, 2, 3, 4, 5, 6]);
      pos.needsUpdate = true;
      assert.strictEqual(l.geometry.getAttribute('position').array[3], 4);
    }
  });

  it('演示坑本身：原地改顶点后包围球不会自动重算（这就是必须关剔除的原因）', () => {
    const { lines } = createLidarRayPool(1);
    const g = lines[0].geometry;
    g.computeBoundingSphere();                       // 全 0 顶点 → 退化到原点
    const r0 = g.boundingSphere!.radius;
    const pos = g.getAttribute('position');
    (pos.array as Float32Array).set([1000, 0, 2000, 1000, 0, 2100]);
    pos.needsUpdate = true;
    const after = g.boundingSphere!.radius;          // 不重算 → 仍是旧值
    assert.ok(after < 1, `包围球不会自动更新（radius=${after}）；改顶点后必须靠 frustumCulled=false 兜住`);
    assert.ok(Number.isFinite(r0));
  });
});
