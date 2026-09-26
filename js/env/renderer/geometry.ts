// ==================== 渲染器：静态几何体构建 ====================
//
// 【职责】构建赛道、障碍物、车辆的 Three.js 几何体。
// 这些函数只在首次调用时执行一次（懒创建），后续帧只更新 position/rotation。
//
// 【坐标系】canvas(x,y) → Three.js(x, 0, z)

import * as THREE from 'three';
import { TrackGeometry, Obstacle } from '../../shared/types.js';
import { Px, m, px } from '../../shared/dimensions.js';
import { buildFrenetReference, frenetToCartesian } from '../../algo/plan/frenet.js';
import { TRACK_HALF_WIDTH } from '../track.js';
import type { FrenetReference } from '../../shared/types.js';

// ═══════════════════════════════════════════════════
// 赛道
// ═══════════════════════════════════════════════════

/**
 * 构建赛道 3D 模型
 * - 路面：ShapeGeometry（外边界挖掉内边界 = 环形路面）
 * - 内外边界线（实线）+ 中心线（虚线）
 */
export function buildTrackGroup(track: TrackGeometry): THREE.Group {
  const group = new THREE.Group();

  // 路面：ShapeGeometry 在 XY 平面创建，旋转 -PI/2 到 XZ 地面时 Z 轴翻转。
  // 因此 Y 取负值，保证旋转后 canvas(x,y) 正确映射到 Three.js(x,0,z)。
  // winding 用 ShapeUtils 检测并修正（外边界 CCW、孔 CW）。
  const toPoints = (pts: { x: Px; y: Px }[]) =>
    pts.map(p => new THREE.Vector2(p.x as number, -(p.y as number)));

  const outerPts = toPoints(track.outerBoundary);
  const innerPts = toPoints(track.innerBoundary);

  const shape = new THREE.Shape(
    THREE.ShapeUtils.isClockWise(outerPts) ? outerPts.reverse() : outerPts
  );
  // 开环道路（OSM）：outerBoundary 已是闭合的路面带多边形、innerBoundary 为空 → 不挖孔
  if (innerPts.length >= 3) {
    const hole = new THREE.Path(
      THREE.ShapeUtils.isClockWise(innerPts) ? innerPts : innerPts.reverse()
    );
    shape.holes.push(hole);
  }

  const roadGeom = new THREE.ShapeGeometry(shape);
  const roadMesh = new THREE.Mesh(roadGeom, new THREE.MeshStandardMaterial({ color: 0x3a3a3a }));
  roadMesh.rotation.x = -Math.PI / 2;
  roadMesh.position.y = 0.01;
  roadMesh.userData = { type: 'track', label: '赛道', width: 15, height: 0, length: '环形' };
  group.add(roadMesh);

  // 赛道轮廓线（辅助函数）
  function addLoopLine(points: { x: Px; y: Px }[], color: number, dashMode: boolean) {
    const pts3 = points.map(p => new THREE.Vector3(p.x as number, 0.02, p.y as number));
    const mat = dashMode
      ? new THREE.LineDashedMaterial({ color, dashSize: 8, gapSize: 12 })
      : new THREE.LineBasicMaterial({ color });
    const line = new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts3), mat);
    if (dashMode) line.computeLineDistances();
    group.add(line);
  }
  addLoopLine(track.outerBoundary, 0x888888, false);
  if (track.innerBoundary.length >= 2) addLoopLine(track.innerBoundary, 0x888888, false);
  addLoopLine(track.centerPoints, 0xaaaaaa, true);

  // 车道分隔线：按地图半宽与车道宽反推车道数（径向图 15m/3.75m = 4 车道 → 分隔线 ±3.75m；
  // OSM 图 11.25m/3.75m = 3 车道 → 分隔线 ±1.875m）。l=0 的中线由中心虚线承载，不重复画。
  const laneRef = buildFrenetReference(track.centerPoints, track.closed ?? true);
  const halfW = (track.halfWidth ?? TRACK_HALF_WIDTH) as number;
  const laneW = m(3.75) as number;
  const laneCount = Math.max(2, Math.round(2 * halfW / laneW));
  for (let k = 1; k < laneCount; k++) {
    const lv = px((-halfW + k * (2 * halfW / laneCount)));
    if (Math.abs(lv as number) < 1e-6) continue;                 // 中线不画（已有中心虚线）
    const pts: { x: Px; y: Px }[] = [];
    for (let i = 0; i < track.centerPoints.length; i++) {
      const cart = frenetToCartesian(laneRef.s[i], lv, laneRef);
      pts.push({ x: px(cart.x as number), y: px(cart.y as number) });
    }
    addLoopLine(pts, 0x666644, true);  // 浅黄虚线
  }

  return group;
}

// ═══════════════════════════════════════════════════
// 障碍物
// ═══════════════════════════════════════════════════

/** 构建障碍物组：红色 BoxGeometry + 边框线 */
export function buildObstacleGroup(obstacles: Obstacle[]): THREE.Group {
  const group = new THREE.Group();
  for (const obs of obstacles) {
    const zH = obs.zHeight || 8;
    const geom = new THREE.BoxGeometry(obs.width as number, zH, obs.height as number);
    const mat = new THREE.MeshStandardMaterial({ color: 0xdd3232, roughness: 0.4 });
    const mesh = new THREE.Mesh(geom, mat);
    mesh.position.set(obs.x as number, zH / 2, obs.y as number);
    mesh.rotation.y = -(obs.angle as number);
    mesh.userData = { type: 'obstacle', label: '障碍物', width: obs.width, height: zH, length: obs.height };

    const edgeGeom = new THREE.EdgesGeometry(geom);
    const edgeLine = new THREE.LineSegments(edgeGeom, new THREE.LineBasicMaterial({ color: 0xff4444 }));
    mesh.add(edgeLine);

    group.add(mesh);
  }
  return group;
}

// ═══════════════════════════════════════════════════
// 车辆
// ═══════════════════════════════════════════════════

/** 构建车辆模型：橙色车身 + 深橙驾驶舱 + 白色车头锥形箭头 */
export function buildCarGroup(): THREE.Group {
  const group = new THREE.Group();

  // 车身：2.0m 宽 × 4.5m 长（4px/m → 8 × 18 sim-units）
  const bodyGeom = new THREE.BoxGeometry(18, 3, 8);
  const bodyMesh = new THREE.Mesh(bodyGeom, new THREE.MeshStandardMaterial({ color: 0xffaa00 }));
  bodyMesh.position.set(0, 1.5, 0);
  bodyMesh.userData = { type: 'car', label: '车辆', width: 2.0, height: 0.75, length: 4.5 };
  group.add(bodyMesh);

  // 驾驶舱
  const cabinGeom = new THREE.BoxGeometry(7, 2.5, 7);
  const cabinMesh = new THREE.Mesh(cabinGeom, new THREE.MeshStandardMaterial({ color: 0xff8800 }));
  cabinMesh.position.set(2, 3.5, 0);
  group.add(cabinMesh);

  // 车头箭头
  const arrowGeom = new THREE.ConeGeometry(1.5, 4, 4);
  const arrowMesh = new THREE.Mesh(arrowGeom, new THREE.MeshStandardMaterial({ color: 0xffffff }));
  arrowMesh.rotation.z = -Math.PI / 2;
  arrowMesh.position.set(11, 1.5, 0);
  group.add(arrowMesh);

  return group;
}

// ═══════════════════════════════════════════════════
// Frenet 刻度（s 横线 / 横向参考线 / 数字标签）
// ═══════════════════════════════════════════════════
//
// 【用途】把规划器内部坐标系搬到路面上：debug 面板里的 "S=22.0m L=-2.6m" 能直接在地上读出来。
// 【参考线口径】与规划器一致——有路线用**路线开环参考线**（s 从路线起点算），否则用赛道闭环。
// 因此本组几何在"路线对象变化"时需要重建（见 renderer.updateScene 的缓存键）。

/** s 方向的横线刻度：每 10m 一条，每 50m 为 major（带数字标签位置）。
 *  端点横跨 ±roadHalf（路面半宽），直线参考下即垂直于路。 */
export function frenetSTicks(
  ref: FrenetReference, maxS: number, roadHalf: number,
): { s: number; major: boolean; a: { x: number; y: number }; b: { x: number; y: number };
     label?: { text: string; x: number; y: number } }[] {
  const out = [] as ReturnType<typeof frenetSTicks>;
  const STEP = 40;            // 10m = 40 仿真单位
  const MAJOR = 200;          // 50m
  for (let s = 0; s <= maxS + 1e-6; s += STEP) {
    const a = frenetToCartesian(s, px(-roadHalf), ref);
    const b = frenetToCartesian(s, px(roadHalf), ref);
    const major = Math.abs(s % MAJOR) < 1e-6;
    const tick: (typeof out)[number] = {
      s, major,
      a: { x: a.x as number, y: a.y as number },
      b: { x: b.x as number, y: b.y as number },
    };
    if (major) {
      const p = frenetToCartesian(s, px(roadHalf), ref);   // 标签放在路的外侧缘
      tick.label = { text: `s=${(s / 4).toFixed(0)}m`, x: p.x as number, y: p.y as number };
    }
    out.push(tick);
  }
  return out;
}

/** 横向参考线：给定 l 值，沿参考线采样成折线（0..maxS） */
export function frenetLLines(
  ref: FrenetReference, lValues: number[], s0: number, maxS: number, sample = 40,
): { l: number; points: { x: number; y: number }[] }[] {
  return lValues.map(l => {
    const points: { x: number; y: number }[] = [];
    for (let s = s0; s <= maxS + 1e-6; s += sample) {
      const c = frenetToCartesian(s, px(l), ref);
      points.push({ x: c.x as number, y: c.y as number });
    }
    return { l, points };
  });
}

/** 横向参考线的数值标签（纯函数，供渲染层画 Sprite）：**贴在 l 线上**（inset=0），
 *  沿参考线每 labelEvery 一处、从 sOffset 起算——默认与 s 刻度**错开半个站**
 *  （s 标签每 50m，本标签 25m 起步每 100m）→ 不会和 `s=…` 挤在一起。文本如 `l=+1.9m`。 */
export function frenetLLabels(
  ref: FrenetReference, lValues: number[], s0: number, maxS: number,
  labelEvery = 400, inset = 0, sOffset = 100,
): { l: number; s: number; text: string; x: number; y: number }[] {
  const out: { l: number; s: number; text: string; x: number; y: number }[] = [];
  for (const l of lValues) {
    const sgn = l >= 0 ? 1 : -1;
    for (let s = s0 + sOffset; s <= maxS + 1e-6; s += labelEvery) {
      const p = frenetToCartesian(s, px(l - sgn * inset), ref);
      out.push({
        l, s,
        text: `l=${l >= 0 ? '+' : ''}${(l / 4).toFixed(1)}m`,
        x: p.x as number, y: p.y as number,
      });
    }
  }
  return out;
}

/** 数字标签：canvas 贴图 Sprite（始终面向相机）。headless（node:test）无 DOM → 返回 null 跳过 */
const _spriteMatCache = new Map<string, THREE.SpriteMaterial>();

export function makeTextSprite(
  text: string, color = '#cfd8e3', scaleM = { w: 6, h: 2.25 },
): THREE.Sprite | null {
  if (typeof document === 'undefined') return null;
  const key = `${text}|${color}`;
  let mat = _spriteMatCache.get(key);
  if (!mat) {
    const canvas = document.createElement('canvas');
    canvas.width = 256; canvas.height = 96;
    const ctx = canvas.getContext('2d')!;
    ctx.font = 'bold 56px sans-serif';
    ctx.fillStyle = color;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(text, canvas.width / 2, canvas.height / 2);
    const tex = new THREE.CanvasTexture(canvas);
    tex.needsUpdate = true;
    mat = new THREE.SpriteMaterial({ map: tex, transparent: true, depthWrite: false });
    _spriteMatCache.set(key, mat);
  }
  const sprite = new THREE.Sprite(mat);
  sprite.scale.set(m(scaleM.w) as number, m(scaleM.h) as number, 1);
  return sprite;
}

/** 组装整组刻度（可整体显隐；路线变化时重建） */
export function buildFrenetTicksGroup(
  ref: FrenetReference, o: { maxS: number; roadHalf: number; domainHalf: number },
): THREE.Group {
  const g = new THREE.Group();
  const addLine = (pts: { x: number; y: number }[], color: number, opacity: number, y = 0.03) => {
    const geo = new THREE.BufferGeometry().setFromPoints(
      pts.map(p => new THREE.Vector3(p.x, y, p.y)));
    g.add(new THREE.Line(geo, new THREE.LineBasicMaterial({ color, transparent: true, opacity })));
  };

  // s 横线：每 10m（暗）、每 50m（亮）+ 数字
  for (const t of frenetSTicks(ref, o.maxS, o.roadHalf)) {
    addLine([t.a, t.b], t.major ? 0x9aa7b5 : 0x5b6472, t.major ? 0.55 : 0.28);
    if (t.label) {
      const sp = makeTextSprite(t.label.text);
      if (sp) {
        sp.position.set(t.label.x, m(1.2) as number, t.label.y);
        g.add(sp);
      }
    }
  }

  // 横向参考线：车道中心 ±1.875m（细）与规划域边 ±domainHalf（与走廊同色系）；
  // 每条线贴标签（l=+1.9m / l=±6.0m，颜色与线一致）：从 25m 起步每 100m 一处 →
  // 与 s 刻度（每 50m）错开半个站，不会挤在一起；标签 y 略低于 s 标签
  const laneL = [m(1.875) as number, -(m(1.875) as number)];
  for (const ln of frenetLLines(ref, laneL, 0, o.maxS)) {
    addLine(ln.points, 0x7d8894, 0.35);
  }
  for (const ln of frenetLLines(ref, [o.domainHalf, -o.domainHalf], 0, o.maxS)) {
    addLine(ln.points, 0x22dd77, 0.5, 0.05);
  }
  const lLabelGroups: { lValues: number[]; color: string }[] = [
    { lValues: laneL, color: '#9aa7b5' },
    { lValues: [o.domainHalf, -o.domainHalf], color: '#7ef0ab' },
  ];
  for (const grp of lLabelGroups) {
    for (const lb of frenetLLabels(ref, grp.lValues, 0, o.maxS)) {
      const sp = makeTextSprite(lb.text, grp.color, { w: 4.6, h: 1.35 });
      if (!sp) break;                       // headless 无 DOM：不画标签
      sp.position.set(lb.x, m(0.9) as number, lb.y);   // y 略低于 s 标签（s 标签在 1.2m）
      g.add(sp);
    }
  }

  return g;
}
