// ==================== 测试：渲染层的世界坐标系约定（右手系） ====================
//
// 【为什么需要】仿真平面是 (x=东, y=北)，而 Three/WebGL 的数学是右手系。
// 若把 sim y 直接映射成世界 +z，世界就是 (X=东, Y=上, Z=北) 这套**左手系**——
// 后果是"画面里北在上时东在左边"，即地面上**任何带手性的内容都会镜像**。
// 实测踩过：真实地图瓦片的文字左右反读（而仿真自己的几何不含手性，三年未暴露）。
//
// 修法：**场景根 z 镜像**（`scene.scale.z = WORLD_Z_SIGN`，sim y → 世界 −z = 北），
// 加进场景的内容照旧用仿真坐标；相机与拾取在场景之外，用 `simToWorldZ` / `worldZToSim` 换算。
//
// 本测试用 Three 自己的矩阵算"屏幕右/上"落在哪个地理方向——这正是当初能一眼定论的判据：
//   · 新约定（-1）：屏幕右 = 东 ✓、屏幕上 = 北 ✓
//   · 旧约定（+1）：屏幕右 = **西** ✗（镜像）——下面显式断言它会失败，把这个坑留在测试里。
import { describe, it } from 'node:test';
import assert from 'node:assert';
import * as THREE from 'three';
import { WORLD_Z_SIGN, simToWorldZ, worldZToSim, gridVisibleWhen } from '../../env/renderer/scene.js';

// 四种组合的实测结论（本测试逐条钉住）：
//   sign=+1 + 相机北侧 → 东在右、北在下   = 整幅上下颠倒（旧代码）
//   sign=−1 + 相机北侧 → 东在左、北在下   = 整幅左右镜像（只改了一半时的实测现象）
//   sign=−1 + 相机南侧 → 东在右、北在上   ✓ 正确
/** 仿真点 → 世界坐标（场景根镜像之后的位置） */
function simToWorld(x: number, simY: number, sign: number): THREE.Vector3 {
  return new THREE.Vector3(x, 0, sign * simY);
}

/**
 * 复现已"上方偏南看向北"的典型取景（与 frameLocal 同一套式子，但显式传入符号以便对照旧约定），
 * 返回"东 / 北 相对中心"在屏幕上的方向。
 */
function screenBasis(sign: number, camSideSim: number): { eastNdcX: number; northNdcY: number } {
  const scene = new THREE.Scene();
  scene.scale.z = sign;
  const span = 500, k = 1;
  const camera = new THREE.PerspectiveCamera(50, 1.5, 1, 5000);
  // camSideSim = 相机在**仿真坐标**里的哪一侧（+1 = 北侧、-1 = 南侧）
  camera.position.set(0, 0.9 * span * k, sign * (camSideSim * 0.6 * span * k));   // 与 app 的 simToWorldZ(cz ∓ 0.6span) 等价
  camera.lookAt(new THREE.Vector3(0, 0, 0));
  camera.updateMatrixWorld(true);

  const center = simToWorld(0, 0, sign);
  const east = simToWorld(100, 0, sign);        // 仿真 +x = 东
  const north = simToWorld(0, 100, sign);       // 仿真 +y = 北
  const p = (v: THREE.Vector3) => v.clone().project(camera);
  return { eastNdcX: p(east).x - p(center).x, northNdcY: p(north).y - p(center).y };
}

describe('渲染层世界约定：右手系（北 = −Z）', () => {
  it('常数与换算：simToWorldZ 取反，且 worldZToSim 是其逆', () => {
    assert.strictEqual(WORLD_Z_SIGN, -1, '世界 z 必须是仿真 y 的反号（北 = −Z）');
    for (const y of [0, 1, -1, 123.4, -9876]) {
      assert.strictEqual(simToWorldZ(y), -y);
      assert.strictEqual(worldZToSim(simToWorldZ(y)), y);
    }
  });

  it('**正确组合**：场景根镜像 + 相机在仿真南侧 → 北在上、东在右', () => {
    const { eastNdcX, northNdcY } = screenBasis(WORLD_Z_SIGN, -1);
    assert.ok(northNdcY > 0, `屏幕上应朝北（NDC y ${northNdcY.toFixed(3)} > 0）`);
    assert.ok(eastNdcX > 0, `屏幕右应朝东（NDC x ${eastNdcX.toFixed(3)} > 0）——否则地面影像会镜像、地图文字反读`);
  });

  it('**旧代码**（sign=+1 + 相机在北侧）：北在下 —— 整幅上下颠倒', () => {
    const { northNdcY } = screenBasis(+1, +1);
    assert.ok(northNdcY < 0, `旧代码下北应在下方（NDC y ${northNdcY.toFixed(3)}）——这是最初"地图反了"的现象`);
  });

  it('**只改一半**（sign=−1 + 相机仍在北侧）：东在左 —— 整幅左右镜像（实测踩过的中间态）', () => {
    const { eastNdcX, northNdcY } = screenBasis(WORLD_Z_SIGN, +1);
    assert.ok(eastNdcX < 0, `只改符号时东会跑到左边（NDC x ${eastNdcX.toFixed(3)}）`);
    assert.ok(northNdcY < 0, '而且北仍在下（相机没挪）');
  });

  it('场景根镜像会把"场景内用仿真坐标"的内容正确摆到世界 −y 处', () => {
    const scene = new THREE.Scene();
    scene.scale.z = WORLD_Z_SIGN;
    const obj = new THREE.Object3D();
    obj.position.set(10, 0, 250);            // 场景内容照旧用仿真坐标 (x=10, y=250 北)
    scene.add(obj);
    scene.updateMatrixWorld(true);
    const w = obj.getWorldPosition(new THREE.Vector3());
    assert.ok(Math.abs(w.x - 10) < 1e-9, '东向不变');
    assert.ok(Math.abs(w.z - (-250)) < 1e-9, `北 250m 应落到世界 z=-250，实际 ${w.z}`);
  });
});

describe('世界网格：有底图时让位', () => {
  it('底图真的显示时隐藏网格；其余情况保留（黑底下网格是空间参照）', () => {
    assert.strictEqual(gridVisibleWhen(true, true), false, '底图显示 → 网格必须隐藏（否则地图上叠一层黑线）');
    assert.strictEqual(gridVisibleWhen(true, false), true, '用户开了底图但这张地图没有瓦片 → 网格要保留');
    assert.strictEqual(gridVisibleWhen(false, true), true, '底图关掉 → 网格恢复');
    assert.strictEqual(gridVisibleWhen(false, false), true);
  });
});
