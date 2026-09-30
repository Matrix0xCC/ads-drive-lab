// ==================== 测试：Frenet 投影精度（缺了很久的守卫） ====================
//
// 【为什么必须有这条测试】2026-09-27 的"幽灵刹车"根因就是**投影口径**：
// 旧实现找"最近**顶点**"并把偏移投到"该顶点的法向量"上 ⇒ 68m 处横向误差可达 **~1.2m** ⇒
// 相邻车道的车被投影进我这条车道 ⇒ 物理走廊判本车道过不去 ⇒ 提前急刹。
// 当时**没有任何测试**盯着"投影准不准"，所以这个误差潜伏了很久。
//
// 【判据】用能直接看懂的几何来判：
//   ① 往返一致：Frenet → 笛卡尔 → Frenet 必须回到原值（<5cm）；
//   ② 车体矩形：相邻车道一辆车（中心 l=±车道宽、宽 2m）的四个角投影回来，
//      必须落在它真正的横向位置上（<5cm）——这正是幽灵刹车里被算错的那个量。
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { buildFrenetReference, cartesianToFrenet, frenetToCartesian } from '../../algo/plan/frenet.js';
import { m, px } from '../../shared/dimensions.js';

/** 半径 500m 的圆弧（与 G4 高速最急 R=141m 同量级、更长），点距 ~4m（与真实地图重采样一致） */
function arcRef(radius = 500, span = Math.PI / 2, n = 200) {
  const pts = [];
  for (let i = 0; i < n; i++) {
    const th = -span / 2 + span * (i / (n - 1));
    // 【量纲】半径与坐标都是**米**，必须过 m() 再进 px()（直接 px(radius·sinθ) 会把米当仿真单位 ⇒
    // 弧长只剩 1/4，测试探针 s 会超出参考线总长被 clamp —— 这个错本轮真踩过 ✗）
    const rU = m(radius) as number;
    pts.push({ x: px(rU * Math.sin(th)), y: px(rU * (Math.cos(th) - 1)) });
  }
  return buildFrenetReference(pts, false);
}

describe('Frenet 投影精度（幽灵刹车根因的守卫）', () => {
  it('往返一致：Frenet → 笛卡尔 → Frenet 误差 <5cm（弯道上、含远处）', () => {
    const ref = arcRef();
    let worstS = 0, worstL = 0;
    for (const sM of [20, 100, 200, 300, 400]) {
      for (const lM of [-4, -2, 0, 2, 4]) {
        const p = frenetToCartesian(px(m(sM) as number), px(m(lM) as number), ref);
        const f = cartesianToFrenet(p.x, p.y, ref);
        worstS = Math.max(worstS, Math.abs((f.s as number) - (m(sM) as number)) / 4);
        worstL = Math.max(worstL, Math.abs((f.l as number) - (m(lM) as number)) / 4);
      }
    }
    console.log(`   [投影] 往返最大误差：纵向 ${worstS.toFixed(4)}m、横向 ${worstL.toFixed(4)}m`);
    assert.ok(worstS < 0.05, `纵向往返误差 ${worstS.toFixed(3)}m 应 <5cm`);
    assert.ok(worstL < 0.05, `横向往返误差 ${worstL.toFixed(3)}m 应 <5cm`);
  });

  it('相邻车道车辆的四个角：投影回来必须落在它真正的横向位置（<5cm）', () => {
    const ref = arcRef();
    const laneW = 3.75, halfW = 1.0, halfL = 2.3;
    let worst = 0;
    // 覆盖近处（12m）与远处（68m、幽灵刹车发生的距离）——旧口径在远处误差 ~1.2m
    for (const sM of [12, 30, 68, 120]) {
      for (const side of [-1, 1]) {
        const centerL = side * laneW;
        for (const dl of [-halfW, halfW]) {
          for (const ds of [-halfL, halfL]) {
            const p = frenetToCartesian(px(m(sM + ds) as number), px(m(centerL + dl) as number), ref);
            const f = cartesianToFrenet(p.x, p.y, ref);
            worst = Math.max(worst, Math.abs((f.l as number) / 4 - (centerL + dl)));
          }
        }
      }
    }
    console.log(`   [投影] 车体四角最大横向误差：${worst.toFixed(4)}m（旧口径在 68m 处可达 ~1.2m ✗）`);
    assert.ok(worst < 0.05, `车体角点横向误差 ${worst.toFixed(3)}m 应 <5cm`);
  });

  it('相邻车道的车**不该**被判进我这条车道（幽灵刹车的直接判据）', () => {
    const ref = arcRef();
    const laneW = 3.75, laneHalf = 1.875;
    const p = frenetToCartesian(px(m(68) as number), px(m(laneW - 1.0) as number), ref);  // 邻车道车的近侧角
    const f = cartesianToFrenet(p.x, p.y, ref);
    assert.ok(Math.abs(f.l as number) / 4 > laneHalf,
      `邻车道车近侧角 l=${((f.l as number) / 4).toFixed(2)}m 必须 > 本车道半宽 ${laneHalf}m`);
  });
});
