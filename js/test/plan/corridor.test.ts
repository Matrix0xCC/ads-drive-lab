// ==================== 测试：可行驶走廊（corridor，SL 空间） ====================
//
// 走廊 = 车辆中心可到达的横向区间集合：命中点/链段投影到 Frenet、按膨胀量
// （车半宽+裕度）膨胀为阻塞区间，再对每个纵向网格点求自由补集。
// 本文件覆盖：自由区间边界、低置信膨胀、偏置几何、链段切分、瓶颈定位。

import { describe, it } from 'node:test';
import assert from 'node:assert';
import { buildCorridor, deriveTargets, firstExit, corridorBandPolygons, corridorPinchSegments, corridorBandObserved, corridorCrossSectionSegments, bandBoundary, type Coverage } from '../../algo/plan/corridor.js';
import { buildFrenetReference } from '../../algo/plan/frenet.js';
import { px, m } from '../../shared/dimensions.js';

const ref = buildFrenetReference(
  Array.from({ length: 61 }, (_, i) => ({ x: px(i * 10), y: px(0) })), true);

const BASE = {
  ref, s0: 0,
  inflate: m(2.5) as number, inflateLow: m(4.5) as number,
  lMax: m(4.4) as number, horizon: m(120) as number, steps: 40, windowRel: m(60) as number,
};
const INFL = m(2.5) as number;   // = 10u
const LMAX = m(4.4) as number;   // = 17.6u

/** 取 s 最接近 targetS 的网格步 */
const stepAt = (c: any, targetS: number) =>
  c.steps.reduce((a: any, b: any) => Math.abs(b.s - targetS) < Math.abs(a.s - targetS) ? b : a);

describe('corridor: 自由区间', () => {
  it('无命中 → 全宽单段', () => {
    const c = buildCorridor({ ...BASE, hits: [], hitSegs: [] });
    for (const st of c.steps) {
      assert.strictEqual(st.free.length, 1);
      assert.ok(Math.abs(st.free[0].lmin + LMAX) < 1e-6);
      assert.ok(Math.abs(st.free[0].lmax - LMAX) < 1e-6);
    }
  });

  it('中央命中点（网格对齐 s=216u）→ 中线两侧各一段，边界=±膨胀量', () => {
    const c = buildCorridor({ ...BASE, hits: [{ x: px(216), y: px(0), d: px(216), low: false }], hitSegs: [] });
    const st = stepAt(c, 216);
    assert.strictEqual(st.free.length, 2);
    assert.ok(Math.abs(st.free[0].lmax + INFL) < 1e-6, `左段右边界=${st.free[0].lmax} 应=${-INFL}`);
    assert.ok(Math.abs(st.free[1].lmin - INFL) < 1e-6, `右段左边界=${st.free[1].lmin} 应=${INFL}`);
  });

  it('低置信命中 → 膨胀更大（低置信带更宽）', () => {
    const lowInfl = m(3.5) as number;   // 18u 会超出走廊半宽（全封死），取 14u 留出对比空间
    const c = buildCorridor({ ...BASE, inflateLow: lowInfl,
      hits: [{ x: px(216), y: px(0), d: px(216), low: true }], hitSegs: [] });
    const st = stepAt(c, 216);
    assert.ok(Math.abs(st.free[0].lmax + lowInfl) < 1e-6, `实际 ${st.free[0].lmax} 应=${-lowInfl}`);
    assert.ok(Math.abs(st.free[1].lmin - lowInfl) < 1e-6);
  });

  it('低置信膨胀超过走廊半宽 → 该步全封死（free 为空）', () => {
    const c = buildCorridor({ ...BASE,
      hits: [{ x: px(216), y: px(0), d: px(216), low: true }], hitSegs: [] });   // inflateLow=4.5m > 4.4m
    const st = stepAt(c, 216);
    assert.deepStrictEqual(st.free, []);
  });

  it('偏置命中（贴右 l≈1.6m）→ 左侧自由段更宽且右边界 = 命中 l − 膨胀（oval-s2 类几何）', () => {
    const c = buildCorridor({ ...BASE, hits: [{ x: px(216), y: px(6.4), d: px(216), low: false }], hitSegs: [] });
    const st = stepAt(c, 216);
    const widest = st.free.reduce((a: any, b: any) => (b.lmax - b.lmin) > (a.lmax - a.lmin) ? b : a);
    assert.ok(Math.abs(widest.lmax - (6.4 - INFL)) < 1e-6, `最宽段右边界=${widest.lmax} 应=${6.4 - INFL}`);
    assert.ok(Math.abs(widest.lmin + LMAX) < 1e-6);
  });

  it('链段横跨中线 → 链段处被切两段；链段之外全宽', () => {
    const c = buildCorridor({ ...BASE, hits: [],
      hitSegs: [{ ax: px(216), ay: px(-6), bx: px(216), by: px(6), low: false }] });
    assert.strictEqual(stepAt(c, 96).free.length, 1, '链段之前全宽');
    assert.strictEqual(stepAt(c, 216).free.length, 2, '链段处被膨胀区间切开');
  });

  it('圆形膨胀：命中所在 s 处阻塞最宽（±infl），超出半径的相邻步不受影响', () => {
    // 用两点命中（间距 > 2·网格步）让网格完全覆盖 ds ∈ {0, 12}：infl=10u
    const c = buildCorridor({ ...BASE,
      hits: [{ x: px(216), y: px(0), d: px(216), low: false },
             { x: px(456), y: px(0), d: px(456), low: false }], hitSegs: [] });
    const at = stepAt(c, 216);          // ds=0 → 半宽 = infl → 边界 ±10
    assert.ok(Math.abs(at.free[0].lmax + INFL) < 1e-6);
    assert.ok(Math.abs(at.free[1].lmin - INFL) < 1e-6);
    const far = stepAt(c, 108);         // 远离两个命中（>infl）→ 全宽
    assert.strictEqual(far.free.length, 1);
  });
});

describe('corridor: 候选派生（间隙中心必被覆盖）', () => {
  const LANE = px(7.5);   // 车道中心 1.875m
  const mk = (hits: any[]) => buildCorridor({ ...BASE, hits, hitSegs: [] });

  it('中央点障碍 → 左右两侧间隙中心都在候选中', () => {
    const c = mk([{ x: px(216), y: px(0), d: px(216), low: false }]);
    const t = deriveTargets({ corridor: c, laneOffset: LANE, l0: 0, lMax: LMAX, maxTargets: 7 });
    // 左段 [−17.6,−10] 中点 −13.8；右段 [10,17.6] 中点 13.8
    assert.ok(t.some(v => Math.abs(v - (-13.8)) < 0.5), `缺左侧间隙中心：${t.map(v => v / 4)}`);
    assert.ok(t.some(v => Math.abs(v - 13.8) < 0.5), `缺右侧间隙中心：${t.map(v => v / 4)}`);
    assert.ok(t.length <= 7);
  });

  it('障碍贴右（oval-s2 类）→ 左间隙中心 ≈ −2.65m 必在候选（旧版被迫贴边）', () => {
    const c = mk([{ x: px(216), y: px(6.4), d: px(216), low: false }]);
    const t = deriveTargets({ corridor: c, laneOffset: LANE, l0: 0, lMax: LMAX, maxTargets: 7 });
    // 左段 [−17.6, −3.6] 中点 −10.6u = −2.65m
    assert.ok(t.some(v => Math.abs(v - (-10.6)) < 0.5), `缺间隙中心：${t.map(v => v / 4)}`);
  });

  it('车道中心在自由段内 → 车道中心入选', () => {
    const c = mk([{ x: px(216), y: px(-6.4), d: px(216), low: false }]);   // 障碍贴左
    const t = deriveTargets({ corridor: c, laneOffset: LANE, l0: 0, lMax: LMAX, maxTargets: 7 });
    assert.ok(t.some(v => Math.abs(v - LANE) < 0.5), `缺车道中心：${t.map(v => v / 4)}`);
  });

  it('全封死 → 单候选 [laneOffset]', () => {
    const c = buildCorridor({ ...BASE, hits: [],
      hitSegs: [{ ax: px(216), ay: px(-40), bx: px(216), by: px(40), low: false }] });
    const t = deriveTargets({ corridor: c, laneOffset: LANE, l0: 0, lMax: LMAX, maxTargets: 7 });
    assert.deepStrictEqual(t, [LANE]);
  });

  it('确定性：同输入两次派生结果一致', () => {
    const c = mk([{ x: px(216), y: px(6.4), d: px(216), low: false }]);
    const a = deriveTargets({ corridor: c, laneOffset: LANE, l0: 0, lMax: LMAX, maxTargets: 7 });
    const b = deriveTargets({ corridor: c, laneOffset: LANE, l0: 0, lMax: LMAX, maxTargets: 7 });
    assert.deepStrictEqual(a, b);
  });
});

describe('corridor: firstExit 与旧逐命中距离判定对拍（不得漏判）', () => {
  // 参考实现 = 改造前的逐命中距离判定（仅测试内保留；直线夹具下路径点 (s,l) ↔ 笛卡尔 (s,l)）
  const CLEAR_U = 10;   // 车半宽 1m + 1.5m（u）
  function legacyFirstBlock(hits: { x: number; y: number }[], points: { s: number; l: number }[]): number {
    const segPointDist = (ax: number, ay: number, bx: number, by: number, px0: number, py0: number) => {
      const dx = bx - ax, dy = by - ay, l2 = dx * dx + dy * dy;
      if (l2 === 0) return Math.hypot(px0 - ax, py0 - ay);
      let t = ((px0 - ax) * dx + (py0 - ay) * dy) / l2;
      t = Math.max(0, Math.min(1, t));
      return Math.hypot(px0 - (ax + t * dx), py0 - (ay + t * dy));
    };
    for (let i = 0; i < points.length - 1; i++) {
      const x1 = points[i].s, y1 = points[i].l, x2 = points[i + 1].s, y2 = points[i + 1].l;
      for (const h of hits) if (segPointDist(x1, y1, x2, y2, h.x, h.y) <= CLEAR_U) return points[i].s;
    }
    return Infinity;
  }

  it('斜穿障碍带的路径：两者都检出，且 firstExit 不晚于旧判定+一个网格步（12u）', () => {
    const hits = [{ x: 216, y: 0, d: 216, low: false }];
    const c = buildCorridor({ ...BASE, hits, hitSegs: [] });
    // 路径：l = s/24（从原点斜穿；最短距离 ≈9u 出现在 s≈216，落在膨胀带内）
    const points = Array.from({ length: 41 }, (_, i) => ({ s: i * 12, l: i * 12 / 24 }));
    const exit = firstExit(c, points, Infinity);
    const leg = legacyFirstBlock(hits, points);
    assert.ok(Number.isFinite(leg), '参考实现应检出阻塞');
    assert.ok(Number.isFinite(exit), 'firstExit 应检出阻塞（不得漏判）');
    assert.ok(exit <= leg + 12 + 1, `firstExit=${exit} 晚于旧判定=${leg} 超过一个网格步`);
  });

  it('远离障碍的路径：两者都判无阻塞', () => {
    const hits = [{ x: 216, y: 0, d: 216, low: false }];
    const c = buildCorridor({ ...BASE, hits, hitSegs: [] });
    const points = Array.from({ length: 41 }, (_, i) => ({ s: i * 12, l: -16 }));
    assert.strictEqual(firstExit(c, points, Infinity), Infinity);
    assert.strictEqual(legacyFirstBlock(hits, points), Infinity);
  });
});

describe('corridor: 瓶颈（pinch）', () => {
  it('窗口内最窄处 → pinch 指向它', () => {
    const c = buildCorridor({ ...BASE, hits: [{ x: px(216), y: px(0), d: px(216), low: false }], hitSegs: [] });
    assert.ok(c.pinch, '应有瓶颈');
    assert.ok(Math.abs(c.pinch!.s - 216) <= 12, `pinch.s=${c.pinch!.s} 应≈216`);
    assert.ok(c.pinch!.width > 0 && c.pinch!.width < 2 * LMAX, `瓶颈宽度应小于全宽：${c.pinch!.width}`);
    assert.ok(c.pinch!.seg, '瓶颈处应有自由段');
  });

  it('全部被堵死（覆盖整个横向）→ pinch.seg=null, width=0', () => {
    const c = buildCorridor({ ...BASE, hits: [],
      hitSegs: [{ ax: px(216), ay: px(-40), bx: px(216), by: px(40), low: false }] });
    assert.strictEqual(c.pinch!.width, 0);
    assert.strictEqual(c.pinch!.seg, null);
  });

  it('窗口外的阻塞不影响 pinch 选择（只看 windowRel 内）', () => {
    const c = buildCorridor({ ...BASE,
      hits: [{ x: px(400), y: px(0), d: px(400), low: false }],   // 100m 处，窗口外（窗口 60m）
      hitSegs: [] });
    assert.ok(Math.abs(c.pinch!.width - 2 * LMAX) < 1e-6, `窗口内应全宽，实际 ${c.pinch!.width}`);
  });
});

describe('corridor: 开环参考线端点（幻影闭合段）', () => {
  // 半圆开环参考线 (0,0)→(50,50)→(100,0)：索引绕接 [n−1 → 0] 会把首尾连成弦 y=0。
  // 命中点 (50,−5) 距弦仅 5u、距真实弧线 55u——若允许绕接，它会投影到幻影弦上
  // （s≈totalS+50、l≈5），膨胀后在弦中部封死 l≈0 一带 → 弦上的路径被假判阻塞
  // （实测：路线终点之外的障碍物由此封死整条走廊 → 假停车，planner-safety 同源）。
  const arcRef = buildFrenetReference(
    Array.from({ length: 61 }, (_, i) => {
      const th = (i / 60) * Math.PI;
      return { x: px(50 - 50 * Math.cos(th)), y: px(50 * Math.sin(th)) };
    }), false);

  it('弦下方命中点不得封死弦上的路径（开环不绕接）', () => {
    const c = buildCorridor({ ...BASE, ref: arcRef, hits: [{ x: 50, y: -5, d: 55, low: false }], hitSegs: [] });
    const path = Array.from({ length: 41 }, (_, i) => ({ s: i * 12, l: 0 }));   // 沿弦 y=0
    assert.strictEqual(firstExit(c, path, Infinity), Infinity, '弦上路径不应被幻影弦命中点阻塞');
  });

  it('开环端点外延拓：终点延长线上的命中点落在 s>totalS（不塌到 totalS 上爆横向范围）', () => {
    // 直线开环参考线 x∈[0,490]；命中点在其延长线 60u 处（l=0）
    const lineRef = buildFrenetReference(
      Array.from({ length: 50 }, (_, i) => ({ x: px(i * 10), y: px(0) })), false);
    const c = buildCorridor({ ...BASE, ref: lineRef, s0: m(60) as number,
      hits: [{ x: 550, y: 0, d: 550, low: false }], hitSegs: [] });   // s0=240u → 网格覆盖到 720u
    const st = stepAt(c, 552);      // 550 ≈ 网格 552
    assert.ok(st.free.length === 2, `延长线上的命中点应只封住 l≈0 一带，实际 free=${JSON.stringify(st.free)}`);
    assert.ok(st.free[0].lmax < 0 && st.free[1].lmin > 0, '自由段应分居两侧（中央被堵）');
  });
});

describe('corridor: 可视化几何（自由带四边形 / 瓶颈横断线）', () => {
  // 直线参考线（间距 10u，沿 y=0）→ (s,l) 与笛卡尔一一对应（x=s, y=l），断言直观
  it('自由带 = 相邻网格步自由段的交集（笛卡尔四边形）', () => {
    const c = buildCorridor({ ...BASE, hits: [], hitSegs: [] });
    const quads = corridorBandPolygons(c, ref);
    assert.ok(quads.length > 0, '应有带子');
    for (const q of quads) {
      assert.strictEqual(q.length, 4, '每片带子是四边形');
      for (const p of q) assert.ok(Number.isFinite(p.x) && Number.isFinite(p.y), `坐标应有限：${JSON.stringify(p)}`);
      const ys = q.map(p => p.y);
      assert.ok(Math.max(...ys) - Math.min(...ys) <= 2 * LMAX + 1e-6, '带子不超出可行驶域');
    }
    // 中央有障碍：带子必须在障碍那一步"断开"——检查落在障碍 s（x=216）上的带子端点，
    // 不得有落在 l≈0 的（否则整条带子会跨过障碍连成一片）
    const cc = buildCorridor({ ...BASE, hits: [{ x: px(216), y: px(0), d: px(216), low: false }], hitSegs: [] });
    const atObstacle = corridorBandPolygons(cc, ref)
      .flatMap(q => q.filter(p => Math.abs(p.x - 216) < 1e-6));
    assert.ok(atObstacle.length >= 4, `障碍两侧各应有带子端点，实际 ${atObstacle.length}`);
    for (const p of atObstacle) {
      assert.ok(Math.abs(p.y) > 1e-6, `障碍 s 处的带子端点不应落在 l≈0（y=${p.y}）`);
    }
  });

  it('瓶颈横断线：有缝→自由段端点；全封死→满宽（±lMax）标记', () => {
    const c = buildCorridor({ ...BASE, hits: [{ x: px(216), y: px(0), d: px(216), low: false }], hitSegs: [] });
    const open = corridorPinchSegments(c, ref, LMAX);
    assert.strictEqual(open.length, 2, '两侧自由段各一条横断线');
    for (const seg of open) {
      assert.strictEqual(seg.length, 2);
      assert.ok(Math.abs(seg[0].x - 216) <= 12 && Math.abs(seg[1].x - 216) <= 12, '横断线在瓶颈所在 s 上');
    }
    const sealed = buildCorridor({ ...BASE, hits: [],
      hitSegs: [{ ax: px(216), ay: px(-40), bx: px(216), by: px(40), low: false }] });
    const closed = corridorPinchSegments(sealed, ref, LMAX);
    assert.strictEqual(closed.length, 1, '封死 → 一条满宽标记');
    const ys = closed[0].map(p => p.y);
    assert.ok(Math.abs(Math.min(...ys) + LMAX) < 1e-6 && Math.abs(Math.max(...ys) - LMAX) < 1e-6, '应为 ±lMax 满宽');
  });
});

describe('corridor: 观测覆盖（未观测区着色：HD Map 背景减除把墙滤掉了，遮挡阴影必须标出来）', () => {
  const fan = (len: number, n = 61, fov = 1.05) => {
    const rays = [];
    for (let i = 0; i < n; i++) rays.push({ angle: -fov + (i / (n - 1)) * 2 * fov, len });
    return rays;
  };
  it('射线整段够长 → 全观测；某方向被截断 → 该方向之外未观测', () => {
    const c = buildCorridor({ ...BASE, hits: [], hitSegs: [] });         // 直线参考：全宽自由带到 480u
    const quads = corridorBandPolygons(c, ref);
    const all = corridorBandObserved(quads, fan(600), 0, 0, 0.0175);
    assert.strictEqual(all.length, quads.length);
    assert.ok(all.every(x => x), '射线够长时应全观测');
    // 正前方方向射线只到 60u（打到墙/障碍）→ 60u 之外未观测
    const rays = fan(600).map(r => (Math.abs(r.angle) < 0.1 ? { ...r, len: 60 } : r));
    const obs = corridorBandObserved(quads, rays, 0, 0, 0.0175);
    for (let i = 0; i < quads.length; i++) {
      const cx = (quads[i][0].x + quads[i][1].x + quads[i][2].x + quads[i][3].x) / 4;
      assert.strictEqual(obs[i], cx <= 60 + 1e-6, `x≤60 应观测：quad ${i} 中心 x=${cx} obs=${obs[i]}`);
    }
    assert.ok(obs.some(x => !x), '应有未观测段');
  });

  it('射线只扫侧方（视锥内但不对着道路）→ 道路带子全部未观测；极短射线 → 只有近处观测', () => {
    const c = buildCorridor({ ...BASE, hits: [], hitSegs: [] });
    const quads = corridorBandPolygons(c, ref);
    const sideFan = [];
    for (let a = 0.6; a <= 1.05; a += 0.035) sideFan.push({ angle: a, len: 600 });   // 34°~60° 一带
    const obs = corridorBandObserved(quads, sideFan, 0, 0, 0.0175);
    for (let i = 0; i < quads.length; i++) {
      const cx = quads[i].reduce((a, p) => a + p.x, 0) / 4;
      const cy = quads[i].reduce((a, p) => a + p.y, 0) / 4;
      const d = Math.hypot(cx, cy);
      if (d > 24) assert.strictEqual(obs[i], false, `侧方扫描不应观察到 d=${(d / 4).toFixed(1)}m 的带子`);
    }
    assert.ok(obs.some(x => x), '车周（<5m 豁免）应视为已观测');
    const tiny = corridorBandObserved(quads, [{ angle: 0, len: 10 }], 0, 0, 0.0175);
    assert.ok(tiny[0], '近处（射线 10u 内）应观测');
    assert.ok(tiny.slice(3).every(x => !x), '更远（射线已到尽头）应未观测');
  });
});

describe('corridor: coverage 传入 → 自由段裁到"被观测"子区间（B：未知当不可通行）', () => {
  const fan = (len: number) => {
    const rays = [];
    for (let i = 0; i < 61; i++) rays.push({ angle: -1.05 + (i / 60) * 2.1, len });
    return rays;
  };
  it('射线只到 120u：120u 内的步保留自由，之外全封死', () => {
    const hits = [{ x: px(216), y: px(0), d: px(216), low: false }];
    const open = buildCorridor({ ...BASE, hits, hitSegs: [], coverage: { rays: fan(600), carX: 0, carY: 0, halfSpacing: 0.0175 } });
    const base = buildCorridor({ ...BASE, hits, hitSegs: [] });
    // 射线到量程时：横向位置也在视锥内的那些步（s≥240u 时 ±17.6u 的方位 ≈4°）应与未裁时一致
    for (let i = 20; i < open.steps.length; i++) {
      assert.deepStrictEqual(open.steps[i].free, base.steps[i].free, `step ${i}（s=${open.steps[i].s}）不应被裁`);
    }
    const gated = buildCorridor({ ...BASE, hits, hitSegs: [], coverage: { rays: fan(120), carX: 0, carY: 0, halfSpacing: 0.0175 } });
    for (let i = 0; i < gated.steps.length; i++) {
      const s = gated.steps[i].s;
      if (s < 120 - 1) {
        assert.ok(gated.steps[i].free.length > 0, `120u 内应保留自由（step ${i}, s=${s}）`);
      } else if (s > 132 + 1) {
        assert.strictEqual(gated.steps[i].free.length, 0, `120u 外应视为不可通行（step ${i}, s=${s}）`);
      }
    }
  });

  it('视锥外（射线只覆盖侧方）→ 全封死', () => {
    const sideRays = [];
    for (let a = 0.6; a <= 1.05; a += 0.035) sideRays.push({ angle: a, len: 600 });
    const c = buildCorridor({ ...BASE, hits: [], hitSegs: [], coverage: { rays: sideRays, carX: 0, carY: 0, halfSpacing: 0.0175 } });
    // 射线只覆盖 34°~60°：远处（方位角变小）全部判不可通行；近侧方可能残留窄条（物理上确实看得到）
    for (const st of c.steps) {
      if (st.s >= 60) assert.strictEqual(st.free.length, 0, `s=${st.s}（方位 <17°）应视为不可通行`);
    }
  });
});

describe('corridor: 带子集合的外边界（未观测区描边用）', () => {
  it('相邻四边形共享边只算一次；单块四边形 = 4 条边', () => {
    const single = bandBoundary([[
      { x: 0, y: 0 }, { x: 0, y: 2 }, { x: 3, y: 2 }, { x: 3, y: 0 },
    ]]);
    assert.strictEqual(single.length, 4, '单块四边形应有 4 条边');
    const pair = bandBoundary([
      [{ x: 0, y: 0 }, { x: 0, y: 2 }, { x: 3, y: 2 }, { x: 3, y: 0 }],
      [{ x: 3, y: 0 }, { x: 3, y: 2 }, { x: 6, y: 2 }, { x: 6, y: 0 }],
    ]);
    assert.strictEqual(pair.length, 6, '共享边（x=3）应被去掉，剩 6 条外边');
    for (const seg of pair) {
      assert.strictEqual(seg.length, 2);
      assert.ok(!(Math.abs(seg[0].x - 3) < 1e-6 && Math.abs(seg[1].x - 3) < 1e-6), '共享边不应出现');
    }
  });
});
