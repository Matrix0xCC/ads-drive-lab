// ==================== 渲染器：场景初始化 ====================
//
// 【坐标系映射】canvas(x,y) → Three.js(x, 0, z)
//   2D 地面平面是 xz，垂直轴是 y（Three.js 默认坐标系）
//
// 【相机】透视相机 + OrbitControls（右键旋转 / 滚轮缩放 / 中键平移）

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { createLidarRayPool } from './rayPool.js';

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

/** 局部取景：把 `spanM` 米见方的一块地取到视口里（中心 = (cx, cz)）。
 *  用于 2km 级长条地图——整图取景时车只是一个像素点，必须局部看。 */
export function frameLocal(
  camera: THREE.PerspectiveCamera, controls: OrbitControls,
  viewW: number, viewH: number, cx: number, cz: number, spanM: number,
): void {
  const aspect = viewW / viewH;
  camera.aspect = aspect;
  const k = Math.max(1, Math.max(1, spanM / 200) / aspect);
  camera.position.set(cx, 0.9 * spanM * k, cz + 0.6 * spanM * k);
  camera.updateProjectionMatrix();
  controls.target.set(cx, 0, cz);
  controls.update();
}

/** 相机跟随：把**相机与目标点一起**沿车的位移平移，保持用户已调整的旋转/俯仰/缩放。
 *  用帧内平滑（k = 3·dt）避免逐帧硬贴造成的抖动。 */
export function followCar(
  camera: THREE.PerspectiveCamera, controls: OrbitControls,
  carX: number, carZ: number, dt: number,
): void {
  const k = Math.min(1, Math.max(0, dt * 3));
  const dx = (carX - controls.target.x) * k;
  const dz = (carZ - controls.target.z) * k;
  if (Math.abs(dx) < 1e-9 && Math.abs(dz) < 1e-9) return;
  camera.position.x += dx; camera.position.z += dz;
  controls.target.x += dx; controls.target.z += dz;
}

/** 智能取景：小世界（径向图 450×200m）整图；大世界（OSM 长条 900×1840m）取车周围 ~250m
 *  局部窗口——否则车在屏幕上只是一个点。 */
export function frameSmart(
  camera: THREE.PerspectiveCamera, controls: OrbitControls,
  viewW: number, viewH: number, worldW: number, worldH: number, carX: number, carZ: number,
): void {
  if (Math.max(worldW, worldH) <= FOLLOW_WORLD_THRESHOLD) {
    frameCamera(camera, controls, viewW, viewH, worldW, worldH);
  } else {
    frameLocal(camera, controls, viewW, viewH, carX, carZ, 250);
  }
}

/** 世界"大"到需要相机跟随/局部取景的阈值（px，≈200m）：径向图（450×200m）不触发，
 *  OSM 高速图（900×1840m）触发——保证历史地图的取景行为一字不变。 */
export const FOLLOW_WORLD_THRESHOLD = 800;

/** 重设世界范围：地面与网格随地图尺寸重建（世界尺寸不再是一个全局常量）。 */
let _ground: THREE.Mesh | null = null;
let _grid: THREE.GridHelper | null = null;
export function setWorldExtent(scene: THREE.Scene, worldW: number, worldH: number): void {
  if (_ground) { scene.remove(_ground); _ground.geometry.dispose(); _ground = null; }
  if (_grid) { scene.remove(_grid); (_grid as unknown as { geometry?: { dispose(): void } }).geometry?.dispose?.(); _grid = null; }
  const cx = worldW / 2, cz = worldH / 2;
  const span = Math.max(worldW, worldH) * 1.5 + 2000;
  _ground = new THREE.Mesh(
    new THREE.PlaneGeometry(span, span),
    new THREE.MeshStandardMaterial({ color: 0x111111 }),
  );
  _ground.rotation.x = -Math.PI / 2;
  _ground.position.set(cx, -0.1, cz);
  scene.add(_ground);
  _grid = new THREE.GridHelper(Math.max(2000, Math.max(worldW, worldH) * 2), 40, 0x333333, 0x222222);
  _grid.position.set(cx, 0, cz);
  scene.add(_grid);
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
  // far 放开到 2 万：OSM 长条图（~1.8km）整图取景时相机距离可达数千
  const camera = new THREE.PerspectiveCamera(45, viewW / viewH, 10, 20000);

  // 轨道控制器：右键旋转、滚轮缩放、中键平移
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;          // 惯性衰减
  controls.dampingFactor = 0.08;
  controls.minDistance = 200;
  // 上限须容纳窄视口取景的拉远距离（k 最大 ≈ worldAspect/minAspect，600k/400k 缩放）
  controls.maxDistance = 15000;
  controls.maxPolarAngle = Math.PI / 2 - 0.05;  // 不允许翻到底部
  frameCamera(camera, controls, viewW, viewH, worldW, worldH);

  // 光照
  scene.add(new THREE.AmbientLight(0xffffff, 0.7));
  const dirLight = new THREE.DirectionalLight(0xffffff, 0.6);
  dirLight.position.set(0, 1, 0.5).normalize();
  scene.add(dirLight);

  // 深色地面 + 网格（按世界尺寸；地图切换时用 setWorldExtent 重建）
  setWorldExtent(scene, worldW, worldH);

  // LiDAR 射线池（预创建 + 每帧原地改顶点；**必须关视锥剔除**，见 rayPool.ts 头注释）
  const MAX_RAYS = 61;
  const { lines: lidarLines, parent: lidarParent } = createLidarRayPool(MAX_RAYS);
  scene.add(lidarParent);

  return { renderer, scene, camera, controls, lidarLines };
}
