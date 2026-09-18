// ==================== 渲染器：场景初始化 ====================
//
// 【坐标系映射】canvas(x,y) → Three.js(x, 0, z)
//   2D 地面平面是 xz，垂直轴是 y（Three.js 默认坐标系）
//
// 【相机】透视相机 + OrbitControls（右键旋转 / 滚轮缩放 / 中键平移）

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

/**
 * 初始化 Three.js 渲染器、场景、相机、光照、射线池、轨道控制器
 *
 * @param canvas HTML Canvas 元素
 * @param w      画布宽度
 * @param h      画布高度
 * @returns { renderer, scene, camera, controls, lidarLines }
 */
export function initScene(canvas: HTMLCanvasElement, w: number, h: number) {
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setSize(w, h);
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x1a1a1a);

  // 透视相机：从赛道中心上方倾斜俯视
  const camera = new THREE.PerspectiveCamera(45, w / h, 10, 4000);
  camera.position.set(w / 2, 600, h / 2 + 400);  // 上方偏南
  camera.lookAt(w / 2, 0, h / 2);

  // 轨道控制器：右键旋转、滚轮缩放、中键平移
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(w / 2, 0, h / 2);
  controls.enableDamping = true;          // 惯性衰减
  controls.dampingFactor = 0.08;
  controls.minDistance = 200;
  controls.maxDistance = 2000;
  controls.maxPolarAngle = Math.PI / 2 - 0.05;  // 不允许翻到底部
  controls.update();

  // 光照
  scene.add(new THREE.AmbientLight(0xffffff, 0.7));
  const dirLight = new THREE.DirectionalLight(0xffffff, 0.6);
  dirLight.position.set(0, 1, 0.5).normalize();
  scene.add(dirLight);

  // 深色地面 + 网格（居中于赛道中心）
  const tcX = w / 2, tcZ = h / 2;
  const ground = new THREE.Mesh(
    new THREE.PlaneGeometry(2000, 2000),
    new THREE.MeshStandardMaterial({ color: 0x111111 })
  );
  ground.rotation.x = -Math.PI / 2;
  ground.position.set(tcX, -0.1, tcZ);
  scene.add(ground);
  const grid = new THREE.GridHelper((w > h ? w : h) * 2, 40, 0x333333, 0x222222);
  grid.position.set(tcX, 0, tcZ);
  scene.add(grid);

  // LiDAR 射线池 —— 预创建 61 条 Line
  const MAX_RAYS = 61;
  const lidarLines: THREE.Line[] = [];
  const lidarParent = new THREE.Group();
  for (let i = 0; i < MAX_RAYS; i++) {
    const geom = new THREE.BufferGeometry();
    geom.setAttribute('position', new THREE.BufferAttribute(new Float32Array(6), 3));
    const mat = new THREE.LineBasicMaterial({ color: 0x00ff00, transparent: true, opacity: 0.25 });
    const line = new THREE.Line(geom, mat);
    line.visible = false;
    lidarParent.add(line);
    lidarLines.push(line);
  }
  scene.add(lidarParent);

  return { renderer, scene, camera, controls, lidarLines };
}
