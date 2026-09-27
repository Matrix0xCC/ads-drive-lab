// ==================== 测试：罗盘的方位角/俯仰角数学 ====================
//
// 这些是"眼睛一看就知道对不对"，但**符号极易写反**的量（本会话已经在镜像/朝向上栽过几次）：
// 世界约定是 x = 东、**z 的负方向是北**（场景根 z 镜像，见 renderer/scene.ts 文件头），
// 而 OrbitControls 用球坐标 θ（自 +Z 起算、朝 +X 为正）表达相机位置。
// 推导：视线方向（水平分量）= −(sinθ, cosθ)，正北 = (0,−1) ⇒ **bearing = −θ**（θ = −bearing）。
// 这里把"相机在四个方向时罗盘读数是多少"逐条钉住。
import { describe, it } from 'node:test';
import assert from 'node:assert';
import * as THREE from 'three';
import {
  compassBearingDeg, viewPitchDeg, bearingToThetaRad, shortestAngleDeltaDeg,
  shortestAngleDeltaRad, bearingLabel, normalizeDeg,
  VIEW_PITCH_2D_DEG, VIEW_PITCH_3D_DEG,
} from '../../env/renderer/scene.js';

const D = 1000;
/** 相机放在目标的某个方向（世界坐标：+x 东、−z 北），看向目标；返回罗盘读数 */
function bearingFrom(camX: number, camY: number, camZ: number): number {
  const target = new THREE.Vector3(0, 0, 0);
  const cam = new THREE.PerspectiveCamera(45, 1.5, 10, 20000);
  cam.position.set(camX, camY, camZ);
  cam.lookAt(target);
  return compassBearingDeg(cam, target);
}

describe('罗盘：方位角（0=正北，顺时针）', () => {
  it('相机在**南侧**看向北 → 0°（这时画面是"北在上"，与 frameLocal 的默认取景一致）', () => {
    assert.ok(Math.abs(bearingFrom(0, 250, +D) - 0) < 1e-6, '相机在 +z（南）侧 → 视线朝北 → 0°');
  });
  it('相机在西侧看向东 → 90°（正东）', () => {
    assert.ok(Math.abs(bearingFrom(-D, 250, 0) - 90) < 1e-6);
  });
  it('相机在北侧看向南 → 180°（正南）', () => {
    assert.ok(Math.abs(bearingFrom(0, 250, -D) - 180) < 1e-6);
  });
  it('相机在东侧看向西 → 270°（正西）', () => {
    assert.ok(Math.abs(bearingFrom(+D, 250, 0) - 270) < 1e-6);
  });
  it('bearing = −θ（OrbitControls 的球坐标符号）', () => {
    for (const b of [0, 45, 90, 180, 270, 359]) {
      const theta = bearingToThetaRad(b);
      assert.ok(Math.abs(normalizeDeg(-theta * 180 / Math.PI) - b) < 1e-9, `bearing ${b} ↔ θ 不闭合`);
    }
    assert.strictEqual(bearingToThetaRad(90), -Math.PI / 2);
  });
});

describe('罗盘：俯仰角（0=垂直俯视 2D）', () => {
  it('正上方 → 0°；45° 斜视 → 45°', () => {
    const target = new THREE.Vector3(0, 0, 0);
    const cam = new THREE.PerspectiveCamera(45, 1.5, 10, 20000);
    cam.position.set(0, D, 0);
    assert.ok(viewPitchDeg(cam, target) < 1e-6, '正上方应为 0°');
    cam.position.set(0, D * Math.SQRT1_2, D * Math.SQRT1_2);
    assert.ok(Math.abs(viewPitchDeg(cam, target) - 45) < 1e-6, '等高/等距应为 45°');
  });
  it('2D/3D 的目标角：2D 接近 0（但不为 0，避免 lookAt 退化）；3D ≈ 33.7°（= atan2(0.6,0.9)）', () => {
    assert.ok(VIEW_PITCH_2D_DEG > 0 && VIEW_PITCH_2D_DEG < 5);
    assert.ok(Math.abs(VIEW_PITCH_3D_DEG - Math.atan2(0.6, 0.9) * 180 / Math.PI) < 1e-9);
  });
});

describe('罗盘：最短转向（不然从 350° 回正北会倒着转 340°）', () => {
  it('度：350→10 是 +20；10→350 是 −20；180 取 +180', () => {
    assert.strictEqual(shortestAngleDeltaDeg(350, 10), 20);
    assert.strictEqual(shortestAngleDeltaDeg(10, 350), -20);
    assert.strictEqual(shortestAngleDeltaDeg(0, 180), 180);
    assert.strictEqual(shortestAngleDeltaDeg(0, 181), -179);
  });
  it('弧度版与度版一致', () => {
    const d = shortestAngleDeltaRad(350 * Math.PI / 180, 10 * Math.PI / 180);
    assert.ok(Math.abs(d - 20 * Math.PI / 180) < 1e-12);
  });
});

describe('罗盘：方位文字', () => {
  it('四个正方向用"正X"', () => {
    assert.strictEqual(bearingLabel(0), '正北');
    assert.strictEqual(bearingLabel(90), '正东');
    assert.strictEqual(bearingLabel(180), '正南');
    assert.strictEqual(bearingLabel(270), '正西');
    assert.strictEqual(bearingLabel(359.5), '正北');   // 容差 1°
    assert.strictEqual(bearingLabel(-90), '正西');     // 负角也规范到 [0,360)
  });
  it('其余用"基准偏顺/逆时针相邻方向 + 偏移角"（偏移量一定 ≤45°）', () => {
    assert.strictEqual(bearingLabel(23), '北偏东 23°');
    assert.strictEqual(bearingLabel(113), '东偏南 23°');
    assert.strictEqual(bearingLabel(203), '南偏西 23°');
    assert.strictEqual(bearingLabel(293), '西偏北 23°');
    assert.strictEqual(bearingLabel(350), '北偏西 10°');
    // 45° 是"北偏东"与"东偏北"的**平局**（同一方向），两种说法都对：只要求它落在其一
    assert.ok(['北偏东 45°', '东偏北 45°'].includes(bearingLabel(45)), `45° 的标签：${bearingLabel(45)}`);
    for (let b = 0; b < 360; b += 7) {
      const label = bearingLabel(b);
      if (label.startsWith('正')) continue;            // 正方向不带偏移角
      const num = Number(/(\d+)°/.exec(label)?.[1] ?? 999);
      assert.ok(num <= 45, `偏移角必须 ≤45°，bearing ${b} 给出 ${label}`);
    }
  });
});
