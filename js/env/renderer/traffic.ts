// ==================== 渲染：交通流车辆 ====================
//
// 【为什么用 InstancedMesh】车流是**每帧变位置**的一批盒子（几十辆）：普通 Mesh 每帧重建
// 会产生大量 GPU 资源分配；InstancedMesh 只更新一个矩阵缓冲，一次 draw call 画完。
//
// 【必须 frustumCulled = false】与 LiDAR 射线池同一个坑：Three 的包围球只在第一次用到时算一次，
// 而我们是**按容量预创建**（初始矩阵全 0）→ 包围球退化成"世界原点一个点"且永不重算 →
// 开着剔除时车辆会随相机移动**时隐时现**（见 rayPool.ts 头注释与 ray-pool.test.ts）。
//
// 【坐标】与其它场景内容一样用**仿真坐标**（x 右、y 前）：`(x, 高度/2, simY)`，
// 场景根的 z 镜像会把它摆正（见 scene.ts 文件头）。

import * as THREE from 'three';
import { Obstacle } from '../../shared/types/interfaces/geometry.js';
import { obstacleBoxYaw } from './geometry.js';

const CAPACITY = 160;
const COLOR = 0x3f7fd8;          // 与静态障碍（红）区分：交通车是蓝的
const EDGE = 0x9fc4ff;

let _mesh: THREE.InstancedMesh | null = null;
let _group: THREE.Group | null = null;
const _m4 = new THREE.Matrix4();
const _q = new THREE.Quaternion();
const _pos = new THREE.Vector3();
const _scale = new THREE.Vector3();
const _up = new THREE.Vector3(0, 1, 0);

/** 每帧更新车辆实例（traffic 为空 → 整组隐藏）；返回是否画了东西（便于测试/调试） */
export function updateTraffic(scene: THREE.Scene, traffic: Obstacle[] | undefined): number {
  const list = traffic ?? [];
  if (list.length === 0 && !_group) return 0;
  if (!_group) {
    const geom = new THREE.BoxGeometry(1, 1, 1);
    const mat = new THREE.MeshStandardMaterial({ color: COLOR, roughness: 0.45 });
    _mesh = new THREE.InstancedMesh(geom, mat, CAPACITY);
    _mesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
    _mesh.frustumCulled = false;                     // 见文件头：否则车辆会时隐时现
    _group = new THREE.Group();
    _group.add(_mesh);
    scene.add(_group);
  }
  if (!_mesh) return 0;
  const n = Math.min(list.length, CAPACITY);
  for (let i = 0; i < n; i++) {
    const o = list[i];
    const zH = o.zHeight || 8;
    _pos.set(o.x as number, zH / 2, o.y as number);
    _q.setFromAxisAngle(_up, obstacleBoxYaw(o.angle as number));   // 与静态障碍同一约定（见 geometry.ts 的推导）
    _scale.set(o.width as number, zH, o.height as number);
    _m4.compose(_pos, _q, _scale);
    _mesh.setMatrixAt(i, _m4);
  }
  _mesh.count = n;
  _mesh.instanceMatrix.needsUpdate = true;
  _group.visible = n > 0;
  return n;
}
