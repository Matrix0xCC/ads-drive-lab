// ==================== 渲染器：场景初始化 ====================
//
// 【坐标系映射】canvas(x,y) → Three.js(x, 0, z)
//   2D 地面平面是 xz，垂直轴是 y（Three.js 默认坐标系）
//
// 【相机】透视相机 + OrbitControls（右键旋转 / 滚轮缩放 / 中键平移）

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

/**
 * 取景：让世界完整落在视口内（相机不裁切赛道）。
 * 原设计取景按世界宽高比（默认 1800/800 = 2.25，相机南侧上方 600/400 俯视）整定；
 * 视口更窄时按 世界宽高比/视口宽高比 等比拉远相机——水平世界覆盖与整定值一致，
 * 多出的垂直空间显示赛道周围地面；视口更宽时不回调（多显示两侧地面）。
 */
export function frameCamera(
  camera: THREE.PerspectiveCamera, controls: OrbitControls,
  viewW: number, viewH: number, worldW: number, worldH: number,
): void {
  const aspect = viewW / viewH;
  const k = Math.max(1, (worldW / worldH) / aspect);
  camera.aspect = aspect;
  camera.position.set(worldW / 2, 600 * k, worldH / 2 + 400 * k);  // 上方偏南
  camera.updateProjectionMatrix();
  controls.target.set(worldW / 2, 0, worldH / 2);
  controls.update();
}

/**
 * 初始化 Three.js 渲染器、场景、相机、光照、射线池、轨道控制器
 *
 * @param canvas HTML Canvas 元素
 * @param viewW  视口宽（渲染分辨率/CSS 尺寸，容器占满屏幕）
 * @param viewH  视口高
 * @param worldW 世界宽（仿真坐标域）
 * @param worldH 世界高
 * @returns { renderer, scene, camera, controls, lidarLines }
 */
export function initScene(
  canvas: HTMLCanvasElement, viewW: number, viewH: number, worldW: number, worldH: number,
) {
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  // updateStyle=false：不写入内联尺寸样式（显示尺寸由 CSS 100% 撑满容器），
  // 画布缓冲区按视口尺寸 × pixelRatio 设置。
  renderer.setSize(viewW, viewH, false);
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x1a1a1a);

  // 透视相机：从赛道中心上方倾斜俯视（取景见 frameCamera）
  const camera = new THREE.PerspectiveCamera(45, viewW / viewH, 10, 4000);

  // 轨道控制器：右键旋转、滚轮缩放、中键平移
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;          // 惯性衰减
  controls.dampingFactor = 0.08;
  controls.minDistance = 200;
  // 上限须容纳窄视口取景的拉远距离（k 最大 ≈ worldAspect/minAspect，600k/400k 缩放）
  controls.maxDistance = 3000;
  controls.maxPolarAngle = Math.PI / 2 - 0.05;  // 不允许翻到底部
  frameCamera(camera, controls, viewW, viewH, worldW, worldH);

  // 光照
  scene.add(new THREE.AmbientLight(0xffffff, 0.7));
  const dirLight = new THREE.DirectionalLight(0xffffff, 0.6);
  dirLight.position.set(0, 1, 0.5).normalize();
  scene.add(dirLight);

  // 深色地面 + 网格（居中于世界中心）
  const tcX = worldW / 2, tcZ = worldH / 2;
  const ground = new THREE.Mesh(
    new THREE.PlaneGeometry(2000, 2000),
    new THREE.MeshStandardMaterial({ color: 0x111111 })
  );
  ground.rotation.x = -Math.PI / 2;
  ground.position.set(tcX, -0.1, tcZ);
  scene.add(ground);
  const grid = new THREE.GridHelper((worldW > worldH ? worldW : worldH) * 2, 40, 0x333333, 0x222222);
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
