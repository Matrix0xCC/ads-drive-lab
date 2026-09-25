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
  const hole = new THREE.Path(
    THREE.ShapeUtils.isClockWise(innerPts) ? innerPts : innerPts.reverse()
  );
  shape.holes.push(hole);

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
  addLoopLine(track.innerBoundary, 0x888888, false);
  addLoopLine(track.centerPoints, 0xaaaaaa, true);

  // 4 车道分隔线（15m ÷ 4 = 3.75m/车道）
  // 分隔线位于 Frenet l = ±3.75m（半宽的 50%）
  const laneRef = buildFrenetReference(track.centerPoints, true);
  const halfLaneWidth = (m(3.75) as number);  // 3.75m → 15 sim-units
  for (const lSign of [-1, 1]) {
    const lv = px(lSign * halfLaneWidth);
    const pts: { x: Px; y: Px }[] = [];
    for (let i = 0; i < track.centerPoints.length; i++) {
      const s = laneRef.s[i];
      const cart = frenetToCartesian(s, lv, laneRef);
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
// 坐标网格与刻度尺（"跑道上的坐标刻度"）
// ═══════════════════════════════════════════════════
//
// 【为何覆盖"赛道包围盒"而非整个世界】用户需求是"在跑道上"能读坐标；世界 1800×800u
// 里赛道只占一块，按赛道包围盒 + 余量铺网格既够用又省线。
// 【单位】刻度文字用米（= 仿真单位 / 4，与 inspect 面板一致）。

export interface GridBounds { minX: number; minY: number; maxX: number; maxY: number }

/** 网格线段（仿真单位；不含边界线——边界由刻度尺画）：minorU 细、majorU 主 */
export function gridSegments(bounds: GridBounds, minorU: number, majorU: number) {
  const out: { ax: number; ay: number; bx: number; by: number; major: boolean }[] = [];
  const startX = Math.ceil((bounds.minX + 1e-6) / minorU) * minorU;
  for (let x = startX; x < bounds.maxX - 1e-6; x += minorU) {
    out.push({ ax: x, ay: bounds.minY, bx: x, by: bounds.maxY, major: Math.abs(x % majorU) < 1e-6 });
  }
  const startY = Math.ceil((bounds.minY + 1e-6) / minorU) * minorU;
  for (let y = startY; y < bounds.maxY - 1e-6; y += minorU) {
    out.push({ ax: bounds.minX, ay: y, bx: bounds.maxX, by: y, major: Math.abs(y % majorU) < 1e-6 });
  }
  return out;
}

/** 刻度尺：左边缘（axis='y'，竖排数字）与下边缘（axis='x'）每 majorU 一条内向短线，含两端 */
export function rulerTicks(bounds: GridBounds, majorU: number, tickU: number) {
  const nz = (v: number) => (v === 0 ? 0 : v);       // 归一化 -0（循环起点会产生 -0）
  const out: { ax: number; ay: number; bx: number; by: number; labelX: number; labelY: number; value: number; axis: 'x' | 'y' }[] = [];
  const startY = Math.ceil((bounds.minY - 1e-6) / majorU) * majorU;
  for (let y = startY; y <= bounds.maxY + 1e-6; y += majorU) {
    out.push({ ax: bounds.minX, ay: nz(y), bx: bounds.minX + tickU, by: nz(y),
      labelX: bounds.minX - tickU * 1.6, labelY: nz(y), value: nz(y), axis: 'y' });
  }
  const startX = Math.ceil((bounds.minX - 1e-6) / majorU) * majorU;
  for (let x = startX; x <= bounds.maxX + 1e-6; x += majorU) {
    out.push({ ax: nz(x), ay: bounds.minY, bx: nz(x), by: bounds.minY + tickU,
      labelX: nz(x), labelY: bounds.minY - tickU * 1.2, value: nz(x), axis: 'x' });
  }
  return out;
}

/** 赛道包围盒 + 余量（米→仿真单位由调用方换算） */
export function trackBounds(track: TrackGeometry, marginU: number): GridBounds {
  let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
  for (const p of [...track.centerPoints, ...track.innerBoundary, ...track.outerBoundary]) {
    minX = Math.min(minX, p.x as number); maxX = Math.max(maxX, p.x as number);
    minY = Math.min(minY, p.y as number); maxY = Math.max(maxY, p.y as number);
  }
  if (!Number.isFinite(minX)) return { minX: 0, minY: 0, maxX: 0, maxY: 0 };
  // 对齐到主网格，让刻度落在整数米上
  return {
    minX: Math.floor((minX - marginU) / 40) * 40, minY: Math.floor((minY - marginU) / 40) * 40,
    maxX: Math.ceil((maxX + marginU) / 40) * 40, maxY: Math.ceil((maxY + marginU) / 40) * 40,
  };
}

/** 文本标签 → Sprite（CanvasTexture）。Node/无 DOM 环境下由调用方注入工厂 */
export function labelSprite(text: string): THREE.Object3D {
  const canvas = document.createElement('canvas');
  canvas.width = 128; canvas.height = 64;
  const ctx = canvas.getContext('2d')!;
  ctx.clearRect(0, 0, 128, 64);
  ctx.fillStyle = 'rgba(180,200,220,0.95)';
  ctx.font = 'bold 40px ui-monospace, monospace';
  ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
  ctx.fillText(text, 64, 34);
  const tex = new THREE.CanvasTexture(canvas);
  tex.colorSpace = THREE.SRGBColorSpace;
  const mat = new THREE.SpriteMaterial({ map: tex, transparent: true, depthWrite: false });
  const sprite = new THREE.Sprite(mat);
  sprite.scale.set(40, 20, 1);         // 世界尺寸（仿真单位）：40u ≈ 10m 宽——默认整场视角下仍可读
  return sprite;
}

/**
 * 赛道坐标网格组：细网格（10m，淡）+ 主网格（50m，略亮）+ 边缘刻度尺 + 数字标签（米）。
 * labelFactory 可注入（单测/无 DOM 环境用），默认走 CanvasTexture Sprite。
 */
export function buildTrackGrid(
  track: TrackGeometry, labelFactory: (text: string) => THREE.Object3D = labelSprite,
): THREE.Group {
  const group = new THREE.Group();
  const bounds = trackBounds(track, m(10) as number);
  const MINOR = m(10) as number, MAJOR = m(50) as number, TICK = m(4) as number;
  const Y = 0.02;                                   // 贴地、在路面之上

  const pushLine = (list: THREE.Vector3[], s: { ax: number; ay: number; bx: number; by: number }) => {
    list.push(new THREE.Vector3(s.ax, Y, s.ay), new THREE.Vector3(s.bx, Y, s.by));
  };
  const segs = gridSegments(bounds, MINOR, MAJOR);
  const minorPts: THREE.Vector3[] = [], majorPts: THREE.Vector3[] = [];
  for (const s of segs) pushLine(s.major ? majorPts : minorPts, s);
  if (minorPts.length) group.add(new THREE.LineSegments(
    new THREE.BufferGeometry().setFromPoints(minorPts),
    new THREE.LineBasicMaterial({ color: 0x8899aa, transparent: true, opacity: 0.07 })));
  if (majorPts.length) group.add(new THREE.LineSegments(
    new THREE.BufferGeometry().setFromPoints(majorPts),
    new THREE.LineBasicMaterial({ color: 0x8899aa, transparent: true, opacity: 0.16 })));

  const tickPts: THREE.Vector3[] = [];
  for (const t of rulerTicks(bounds, MAJOR, TICK)) {
    pushLine(tickPts, t);
    const label = labelFactory(String(Math.round(t.value / 4)));
    label.position.set(t.labelX, 0.5, t.labelY);
    group.add(label);
  }
  if (tickPts.length) group.add(new THREE.LineSegments(
    new THREE.BufferGeometry().setFromPoints(tickPts),
    new THREE.LineBasicMaterial({ color: 0xaebdcc, transparent: true, opacity: 0.5 })));

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

/** 数字标签：canvas 贴图 Sprite（始终面向相机）。headless（node:test）无 DOM → 返回 null 跳过 */
function makeTextSprite(text: string, color = '#cfd8e3'): THREE.Sprite | null {
  if (typeof document === 'undefined') return null;
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
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, transparent: true, depthWrite: false }));
  sprite.scale.set(m(6) as number, m(2.25) as number, 1);
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

  // 横向参考线：车道中心 ±1.875m（细）与规划域边 ±domainHalf（与走廊同色系）
  for (const ln of frenetLLines(ref, [m(1.875) as number, -(m(1.875) as number)], 0, o.maxS)) {
    addLine(ln.points, 0x7d8894, 0.35);
  }
  for (const ln of frenetLLines(ref, [o.domainHalf, -o.domainHalf], 0, o.maxS)) {
    addLine(ln.points, 0x22dd77, 0.5, 0.05);
  }

  return g;
}
