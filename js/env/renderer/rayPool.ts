// ==================== 渲染器：LiDAR 射线池 ====================
//
// 【职责】预创建 N 条 `THREE.Line`（每条 2 个顶点），供每帧**原地改顶点**复用——
// 避免每帧 new/dispose 61 个几何体。
//
// 【为什么单独一个模块 + 为什么必须 frustumCulled = false】
// 这种"预创建 + 原地改顶点"的写法踩了 Three.js 的一个坑：**包围球只在第一次用到时算一次**
// （`Frustum.intersectsObject` 里 `if (geometry.boundingSphere === null) computeBoundingSphere()`），
// 而池子是先用**全 0 顶点**创建的 → 第一个包围球退化成"世界原点处的一个点"，之后**永不重算**。
// 于是视锥剔除会拿这个过期包围球判断，射线就随相机**平移/缩放**时而出现时而消失：
// 相机视野里含原点 → 出现；相机跟着车跑出几公里（OSM 长条图/路网图）→ 消失。
// 这正是"动一下视角/缩放就出来了，再动又没了"的根因。
//
// 射线永远在车周围几十~一百多米内、数量固定 61 条，**剔除本身毫无收益**，
// 所以直接关掉剔除（比每帧 computeBoundingSphere 61 次更省）。
// 单独成模块是为了让这条约束能被 node:test 直接钉住（不依赖 DOM/WebGL）。

import * as THREE from 'three';

/** 创建射线池：每条射线 = 一个 6 浮点（起终点）的 BufferGeometry */
export function createLidarRayPool(count: number): { lines: THREE.Line[]; parent: THREE.Group } {
  const parent = new THREE.Group();
  const lines: THREE.Line[] = [];
  for (let i = 0; i < count; i++) {
    const geom = new THREE.BufferGeometry();
    geom.setAttribute('position', new THREE.BufferAttribute(new Float32Array(6), 3));
    const mat = new THREE.LineBasicMaterial({ color: 0x00ff00, transparent: true, opacity: 0.25 });
    const line = new THREE.Line(geom, mat);
    line.visible = false;
    // 【不要删】见文件头：预创建 + 原地改顶点 → 包围球过期 → 视锥剔除会造成"闪没"
    line.frustumCulled = false;
    parent.add(line);
    lines.push(line);
  }
  return { lines, parent };
}
