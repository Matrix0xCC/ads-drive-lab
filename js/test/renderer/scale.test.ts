// ==================== 测试：比例尺与量距的数学 ====================
//
// 单位与方向都极易写错（本会话的教训）。这里钉住：1px 多少米、整数取值、刻度摆法、文字格式。
// 例：fov=45°、视口高 800px、目标离相机 1000 世界单位（=250m，4 单位/米）
//     → 每像素 2·1000·tan22.5°/800 = 1.0355 世界单位 = **0.2589 m**
import { describe, it } from 'node:test';
import assert from 'node:assert';
import * as THREE from 'three';
import {
  metersPerPixel, metersPerPixelAtGround, niceScaleBar, formatMeters, niceRulerInterval, rulerTicks,
} from '../../env/renderer/scale.js';
import { obstacleBoxYaw } from '../../env/renderer/geometry.js';

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

// ─────────────────────────────────────────────────────────────
// 比例尺必须"量出来就是标注"——**在它自己所在的位置上**
// 用户就是拿地面上的 s 刻度线（50m 一道）对照发现的：条长看着像 10m，标注却是 5m。
// 根因：透视相机下横向尺度随**深度**变化，而比例尺画在屏幕**底部**（那里地面近得多），
// 旧实现却用画面中心（OrbitControls 目标点）的尺度 → 条的实际覆盖 ≠ 标注。
// 下面用"把条两端的屏幕点打到地面、量真实距离"来判（与用户的做法一致，不依赖实现细节）。
// ─────────────────────────────────────────────────────────────
describe('比例尺：条的覆盖必须等于标注（在它自己的位置上）', () => {
  const W = 702, H = 378;
  /** 复现 app 的典型取景：frameLocal(span=250) + 北在上 */
  function appCamera(): THREE.PerspectiveCamera {
    const cam = new THREE.PerspectiveCamera(45, W / H, 10, 20000);
    cam.position.set(0, 0.9 * 250, 0 - 0.6 * 250);        // 世界：北 = −z → 相机在南侧（与 app 同）
    cam.lookAt(new THREE.Vector3(0, 0, 0));
    cam.updateMatrixWorld(true);
    return cam;
  }
  const plane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
  const ray = new THREE.Raycaster();
  /** 屏幕点 → 地面点（世界坐标） */
  function groundAt(cam: THREE.PerspectiveCamera, sx: number, sy: number): THREE.Vector3 | null {
    ray.setFromCamera(new THREE.Vector2((sx / W) * 2 - 1, -(sy / H) * 2 + 1), cam);
    const hit = new THREE.Vector3();
    return ray.ray.intersectPlane(plane, hit) ? hit : null;
  }
  /** 条两端打到地面后的**真实距离（米）** */
  function coveredMeters(cam: THREE.PerspectiveCamera, widthPx: number, sy: number): number {
    const a = groundAt(cam, W / 2 - widthPx / 2, sy), b = groundAt(cam, W / 2 + widthPx / 2, sy);
    assert.ok(a && b, '条两端都应能打到地面');
    return a!.distanceTo(b!) / 4;                          // 世界单位 → 米
  }
  const SY = H - 16;                                       // 比例尺所在高度（贴近底部）

  it('用**条自己所在处**的尺度 → 覆盖 = 标注（误差 <2%）', () => {
    const cam = appCamera();
    const point = groundAt(cam, W / 2, SY)!;
    const mpp = metersPerPixelAtGround(cam, point, W, H);
    const bar = niceScaleBar(mpp);
    const covered = coveredMeters(cam, bar.widthPx, SY);
    assert.ok(Math.abs(covered - bar.meters) / bar.meters < 0.02,
      `标注 ${bar.label}，实际覆盖 ${covered.toFixed(2)}m（差 ${(100 * Math.abs(covered - bar.meters) / bar.meters).toFixed(1)}%）`);
  });

  it('用**画面中心**的尺度（旧实现）→ 覆盖与标注明显不符（这就是被修掉的 bug）', () => {
    const cam = appCamera();
    const center = metersPerPixel(cam, new THREE.Vector3(0, 0, 0), H);
    const bottomPoint = groundAt(cam, W / 2, SY)!;
    const bottom = metersPerPixelAtGround(cam, bottomPoint, W, H);
    assert.ok(Math.abs(center / bottom - 1) > 0.1,
      `斜视时屏幕底部与中心的横向尺度应当明显不同（实测比 ${(center / bottom).toFixed(2)}）`);
    const bar = niceScaleBar(center);
    const covered = coveredMeters(cam, bar.widthPx, SY);
    assert.ok(Math.abs(covered - bar.meters) / bar.meters > 0.1,
      `旧实现应当明显不符：标注 ${bar.label} vs 实际覆盖 ${covered.toFixed(2)}m`);
  });

  it('正俯视（2D）时两者一致——没有斜视就没有深度差（守卫不能把正常情况改坏）', () => {
    const cam = new THREE.PerspectiveCamera(45, W / H, 10, 20000);
    cam.position.set(0, 250, 0.001);                       // 几乎正上方
    cam.lookAt(new THREE.Vector3(0, 0, 0));
    cam.updateMatrixWorld(true);
    const point = groundAt(cam, W / 2, SY)!;
    const at = metersPerPixelAtGround(cam, point, W, H);
    const center = metersPerPixel(cam, new THREE.Vector3(0, 0, 0), H);
    assert.ok(Math.abs(at / center - 1) < 0.05, `正俯视下应基本一致，实际比 ${(at / center).toFixed(3)}`);
  });
});

// ─────────────────────────────────────────────────────────────
// 障碍物/车辆盒子的朝向：**长轴必须指向行进方向**（实测踩过：写成 rotation.y = -angle 会转 90°，
// 车斜着横在路上；这是既有问题——静态障碍也一样歪，只是车流一多才显眼）。
// ─────────────────────────────────────────────────────────────
describe('障碍物盒子朝向（长轴 = 行进方向）', () => {
  function longAxisDot(simAngle: number, rotFn: (a: number) => number): number {
    const mesh = new THREE.Mesh(new THREE.BoxGeometry(8, 8, 18));   // 横向 2m、纵向 4.6m
    mesh.position.set(0, 4, 0);                                     // local z ↔ 仿真 y（见 geometry.ts）
    mesh.rotation.y = rotFn(simAngle);
    mesh.updateMatrixWorld(true);
    const longAxis = new THREE.Vector3(0, 0, 1).transformDirection(mesh.matrixWorld);
    const travel = new THREE.Vector3(Math.cos(simAngle), 0, Math.sin(simAngle));
    return longAxis.dot(travel);
  }

  it('八个行进方向上都对齐（长轴 · 行进方向 = 1）', () => {
    for (let k = 0; k < 8; k++) {
      const a = (k * Math.PI) / 4;
      const dot = longAxisDot(a, obstacleBoxYaw);
      assert.ok(Math.abs(dot - 1) < 1e-6, `朝向 ${(a * 180 / Math.PI).toFixed(0)}° 时点积 ${dot.toFixed(3)}（应为 1）`);
    }
  });

  it('旧的 `rotation.y = -angle` 会转 90°（点积 0）——把 bug 钉在原地', () => {
    for (const k of [1, 2, 3]) {
      const a = (k * Math.PI) / 4;
      assert.ok(Math.abs(longAxisDot(a, (x) => -x)) < 1e-6, '旧写法应当是"横着"的（点积 0）');
    }
  });
});
