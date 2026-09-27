// ==================== 渲染器：场景初始化 ====================
//
// 【坐标系映射】canvas(x,y) → Three.js(x, 0, z)
//   2D 地面平面是 xz，垂直轴是 y（Three.js 默认坐标系）
//
// 【相机】透视相机 + OrbitControls（**左键旋转** / 滚轮缩放 / 右键平移——未覆盖 mouseButtons，用默认映射）

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { createLidarRayPool } from './rayPool.js';
import { DEFAULT_SENSOR_RIG, rigRayCount } from '../../shared/sensors.js';

// ═══════════════════════════════════════════════════
// 【世界坐标的右手系约定】为什么要有这个常数
// ═══════════════════════════════════════════════════
//
// 仿真平面是 (x = 东, y = 北)。若**直接**把 sim y 映射成世界 +z，则世界为
// (X = 东, Y = 上, Z = 北)——而 Three/WebGL 的数学是右手系，这套轴序是**左手系**：
// 于是"画面里北在上"时，东会出现在**左边** → 地面上任何**带手性**的内容都会**镜像**
// （真实地图的文字反着读）。实测证据：复现 app 取景算相机基向量，得到 屏幕右·东 = −1.00。
//
// 仿真自己的几何不含手性（路线、车道线、赛道镜像后仍是它自己），所以这个镜像三年没暴露；
// 一旦贴上真实地图瓦片（有文字）就显形。
//
// 【修法（两件事必须一起做，少一件都会"还是反的"）】
//   ① **场景根 z 镜像**：`scene.scale.z = WORLD_Z_SIGN`，即 sim y → 世界 −z（北 = −Z）→ 恢复右手系；
//   ② **相机放在车的"南侧"**（仿真 y 更小的一侧）看向北 → 画面"北在上"。
// 为什么少一件都不行（四种组合实测，见 js/test/renderer/world-convention.test.ts）：
//     旧代码：sign=+1 + 相机在北侧 → 东在右、**北在下**（整幅**上下颠倒**）
//     只改①：sign=−1 + 相机在北侧 → **东在左**、北在下（整幅**左右镜像**）  ← 实测踩过
//     ①+② ：sign=−1 + 相机在南侧 → 东在右、北在上  ✓ 正确
//   · **加进 scene 的一切内容照旧用仿真坐标 (x, y)**——镜像由场景根统一施加，
//     所以路面/障碍/轨迹/路线/雷达/瓦片/刻度/走廊**一处都不用改**；
//   · **相机与 OrbitControls 不在场景里** → 它们的坐标必须写成世界坐标：z = simToWorldZ(simY)。
//     这就是下面所有 `simToWorldZ(...)` 的来源；
//   · **屏幕 → 世界的拾取**（`ui/interactions.ts`）要换回仿真 y：`worldZToSim(worldZ)`。
export const WORLD_Z_SIGN = -1;
/** 仿真 y（北正）→ 世界 z */
export function simToWorldZ(simY: number): number { return WORLD_Z_SIGN * simY; }
/** 世界 z → 仿真 y（自逆） */
export function worldZToSim(worldZ: number): number { return WORLD_Z_SIGN * worldZ; }

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
  // 注意：相机不在场景里 → 这里必须写**世界坐标**（z = simToWorldZ(仿真 y)）；
  // 且必须放在"仿真 y 更小"的一侧（南侧）→ 画面北在上（见文件头四种组合）。
  camera.position.set(worldW / 2, 600 * k, simToWorldZ(worldH / 2 - 400 * k));  // 上方偏南
  camera.updateProjectionMatrix();
  controls.target.set(worldW / 2, 0, simToWorldZ(worldH / 2));
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
  camera.position.set(cx, 0.9 * spanM * k, simToWorldZ(cz - 0.6 * spanM * k));   // 南侧看向北
  camera.updateProjectionMatrix();
  controls.target.set(cx, 0, simToWorldZ(cz));
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
  const dz = (simToWorldZ(carZ) - controls.target.z) * k;   // carZ 是仿真 y → 转世界 z
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
/** 世界网格是否显示（有底图时让位；见 `gridVisibleWhen`）——声明放这里，别放在函数后面（TDZ） */
let _gridVisible = true;
/**
 * 世界网格是否显示。
 * **底图打开时要隐藏网格**：网格是给"黑底"提供空间参照的调试辅助，而真实地图自己就有路网与
 * 地物参照；叠在一起只会让地图上多出一层黑线（用户实测反馈"地图上面的黑色网格线要去掉”）。
 */
export function gridVisibleWhen(showBasemap: boolean, hasBasemap: boolean): boolean {
  return !(showBasemap && hasBasemap);
}
export function setGridVisible(v: boolean): void {
  _gridVisible = v;
  if (_grid) _grid.visible = v;
}

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
  // 【分层与绘制顺序】地面放得足够低（层间距要给深度精度留余量，见 basemap.ts 的 depthTest 说明），
  // 并**最先绘制**（renderOrder 小于底图的 -1）→ 底图可以"不比深度"地覆在它上面。
  _ground.position.set(cx, -5, cz);
  _ground.renderOrder = -2;
  scene.add(_ground);
  _grid = new THREE.GridHelper(Math.max(2000, Math.max(worldW, worldH) * 2), 40, 0x333333, 0x222222);
  _grid.visible = _gridVisible;                    // 保持当前的网格可见性（底图打开时网格要让位）
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
  // 【世界右手系】见文件头：sim y → 世界 −z（北 = −Z），否则地面影像会镜像（文字反读）
  scene.scale.z = WORLD_Z_SIGN;
  scene.background = new THREE.Color(0x1a1a1a);

  // 透视相机：从赛道中心上方倾斜俯视（取景见 frameCamera）
  // far 放开到 2 万：OSM 长条图（~1.8km）整图取景时相机距离可达数千
  const camera = new THREE.PerspectiveCamera(45, viewW / viewH, 10, 20000);

  // 轨道控制器：**左键旋转**、滚轮缩放、右键平移（未覆盖 mouseButtons → OrbitControls 默认映射；
  // 注意"左键拖动 = 转视角"，所以量距/设路线这类吃左键的功能必须做成**模式**）
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;          // 惯性衰减
  controls.dampingFactor = 0.08;
  controls.minDistance = 200;
  // 上限须容纳窄视口取景的拉远距离（k 最大 ≈ worldAspect/minAspect，600k/400k 缩放）
  controls.maxDistance = 15000;
  controls.maxPolarAngle = Math.PI / 2 - 0.05;  // 不允许翻到底部
  // 用户一开始拖动，就放弃罗盘的平滑转动（否则动画会和手指抢方向盘）
  controls.addEventListener('start', () => { _viewSnap = null; });
  frameCamera(camera, controls, viewW, viewH, worldW, worldH);

  // 光照
  scene.add(new THREE.AmbientLight(0xffffff, 0.7));
  const dirLight = new THREE.DirectionalLight(0xffffff, 0.6);
  dirLight.position.set(0, 1, 0.5).normalize();
  scene.add(dirLight);

  // 深色地面 + 网格（按世界尺寸；地图切换时用 setWorldExtent 重建）
  setWorldExtent(scene, worldW, worldH);

  // LiDAR 射线池（预创建 + 每帧原地改顶点；**必须关视锥剔除**，见 rayPool.ts 头注释）
  // 射线池容量按**传感器套件**（多向之后不再是 61）。池子预创建、每帧原地改顶点，
  // 所以容量必须 ≥ 本帧射线数（见 rayPool.ts 的 frustumCulled 坑）。
  const MAX_RAYS = rigRayCount(DEFAULT_SENSOR_RIG);
  const { lines: lidarLines, parent: lidarParent } = createLidarRayPool(MAX_RAYS);
  scene.add(lidarParent);

  return { renderer, scene, camera, controls, lidarLines };
}

// ═══════════════════════════════════════════════════
// 罗盘（导航软件式）：方位角/俯仰角的读取与平滑转动
// ═══════════════════════════════════════════════════
//
// 【约定】世界：x = 东、y = 上、z 的**负方向**是北（场景根 z 镜像，见文件头）。
// 罗盘的"方位角"= **视线方向**在地面上的朝向，以正北为 0°、顺时针为正（东 90°、南 180°、西 270°）
// ——也就是"屏幕上方指向哪个方位"，这正是导航软件罗盘显示的量。
//
// 与 OrbitControls 的关系：控件用球坐标表达相机相对目标点的位置：
//   x = r·sinφ·sinθ, y = r·cosφ, z = r·sinφ·cosθ   （θ 自 +Z 起算、朝 +X 为正）
//   → 视线方向（水平分量）= −(sinθ, cosθ)，而正北 = (0, −1)
//   → 方位角 bearing = atan2(−sinθ, cosθ) = **−θ**
// 所以"转到方位 b"就是 **θ = −b**（下面 `bearingToThetaRad` 钉住了这个符号）。

/** 俯仰角（度）：0 = 垂直俯视（2D），90 = 与地面平行。取 OrbitControls 的 φ（自 +Y 起算）。 */
export function viewPitchDeg(camera: THREE.PerspectiveCamera, target: THREE.Vector3): number {
  const off = camera.position.clone().sub(target);
  const r = off.length();
  if (r < 1e-6) return 0;
  return Math.acos(Math.min(1, Math.max(-1, off.y / r))) * 180 / Math.PI;
}

/** 方位角（度，0=正北、顺时针）：即"屏幕上方"指向的方位。见上方推导。 */
export function compassBearingDeg(camera: THREE.PerspectiveCamera, target: THREE.Vector3): number {
  const off = camera.position.clone().sub(target);
  const theta = Math.atan2(off.x, off.z);           // OrbitControls 的 θ
  return normalizeDeg(-theta * 180 / Math.PI);
}

/** 把角度规范到 [0, 360) */
export function normalizeDeg(deg: number): number {
  return ((deg % 360) + 360) % 360;
}

/** 方位角 → OrbitControls 的 θ（弧度）。**符号在这里**：θ = −bearing。 */
export function bearingToThetaRad(bearingDeg: number): number {
  return -bearingDeg * Math.PI / 180;
}

/** 从 a 转到 b 的**最短**角度差（度，含符号，∈(−180, 180]）——避免"350°→10° 倒着转 340°" */
export function shortestAngleDeltaDeg(a: number, b: number): number {
  let d = normalizeDeg(b) - normalizeDeg(a);
  if (d > 180) d -= 360;
  if (d <= -180) d += 360;
  return d;
}

/** 同上，弧度版（转 θ 用） */
export function shortestAngleDeltaRad(a: number, b: number): number {
  return shortestAngleDeltaDeg(a * 180 / Math.PI, b * 180 / Math.PI) * Math.PI / 180;
}

/** 方位角的中文标签：正北 / 北偏东 23° / 东偏南 23° … */
export function bearingLabel(bearingDeg: number): string {
  const b = normalizeDeg(bearingDeg);
  const cardinals: [number, string][] = [[0, '正北'], [90, '正东'], [180, '正南'], [270, '正西']];
  for (const [deg, name] of cardinals) {
    if (Math.abs(shortestAngleDeltaDeg(b, deg)) < 1) return name;
  }
  // 以最近的象限基准（北/东/南/西）为参照，给出 ≤45° 的偏移
  const base = Math.round(b / 90) * 90;
  const diff = shortestAngleDeltaDeg(base, b);          // ∈ (−45, 45)
  const baseName = ({ 0: '北', 90: '东', 180: '南', 270: '西' } as Record<number, string>)[normalizeDeg(base)];
  const dir = diff > 0 ? '偏东' : '偏西';
  // "东偏南"？约定用**相邻顺时针/逆时针**的名字更符合习惯：北→东→南→西 顺时针
  const clockwiseOf: Record<string, string> = { 北: '东', 东: '南', 南: '西', 西: '北' };
  const ccwOf: Record<string, string> = { 北: '西', 西: '南', 南: '东', 东: '北' };
  const toward = diff > 0 ? clockwiseOf[baseName] : ccwOf[baseName];
  return `${baseName}偏${toward} ${Math.abs(diff).toFixed(0)}°`;
}

/** 2D（垂直俯视）与 3D（斜视）的目标俯仰角。3D 取 atan2(0.6, 0.9) ≈ 33.7°，
 *  与 `frameLocal` 的默认取景（0.9 高 / 0.6 后）一致，点 3D 即回到"看上去正常"的视角。 */
export const VIEW_PITCH_2D_DEG = 1.5;                 // 留一点余量：正上方 lookAt 会退化
export const VIEW_PITCH_3D_DEG = Math.atan2(0.6, 0.9) * 180 / Math.PI;
/** 判定"当前算 2D 还是 3D"：阈值之间留一段灰区（手拖到中间角度时不冒充任何一种） */
export const VIEW_PITCH_2D_MAX_DEG = 8;
export const VIEW_PITCH_3D_MIN_DEG = 20;
export function pitchModeOf(pitchDeg: number): '2d' | '3d' | '' {
  if (!Number.isFinite(pitchDeg)) return '';
  if (pitchDeg <= VIEW_PITCH_2D_MAX_DEG) return '2d';
  if (pitchDeg >= VIEW_PITCH_3D_MIN_DEG) return '3d';
  return '';
}
/** 单按钮切换的目标：在 2D → 去 3D；其余 → 回 2D（灰区里按"回 2D"处理，符合同一个按钮的心智） */
export function toggledPitchMode(pitchDeg: number): '2d' | '3d' {
  return pitchModeOf(pitchDeg) === '2d' ? '3d' : '2d';
}

interface ViewSnap {
  t: number; ms: number;
  fromTheta: number; dTheta: number; fromPhi: number; dPhi: number;
}

let _viewSnap: ViewSnap | null = null;

/**
 * 平滑转到目标方位/俯仰（只改朝向，**保持目标点与距离**）。
 * 用户一旦拖动就放弃（`initScene` 里注册的 controls 'start'）。
 */
export function snapView(
  camera: THREE.PerspectiveCamera, controls: OrbitControls,
  opts: { bearingDeg?: number; pitchDeg?: number }, ms = 250,
): void {
  const off = camera.position.clone().sub(controls.target);
  const r = off.length();
  if (r < 1e-6) return;
  const fromTheta = Math.atan2(off.x, off.z);
  const fromPhi = Math.acos(Math.min(1, Math.max(-1, off.y / r)));
  const toTheta = opts.bearingDeg === undefined
    ? fromTheta
    : fromTheta + shortestAngleDeltaRad(fromTheta, bearingToThetaRad(opts.bearingDeg));
  const toPhi = opts.pitchDeg === undefined
    ? fromPhi
    : Math.min(controls.maxPolarAngle, Math.max(0, opts.pitchDeg * Math.PI / 180));
  _viewSnap = { t: 0, ms: Math.max(1, ms), fromTheta, dTheta: toTheta - fromTheta, fromPhi, dPhi: toPhi - fromPhi };
}

/** 每帧推进平滑转动（主循环里调用；不在动画中则什么都不做） */
export function stepViewSnap(
  camera: THREE.PerspectiveCamera, controls: OrbitControls, dtMs: number,
): boolean {
  if (!_viewSnap) return false;
  const s = _viewSnap;
  s.t = Math.min(s.ms, s.t + dtMs);
  const u = s.t / s.ms;
  const e = u * u * (3 - 2 * u);                       // smoothstep 缓动
  const off = camera.position.clone().sub(controls.target);
  const r = off.length();
  const phi = s.fromPhi + s.dPhi * e;
  const theta = s.fromTheta + s.dTheta * e;
  camera.position.set(
    controls.target.x + r * Math.sin(phi) * Math.sin(theta),
    controls.target.y + r * Math.cos(phi),
    controls.target.z + r * Math.sin(phi) * Math.cos(theta),
  );
  controls.update();
  if (s.t >= s.ms) _viewSnap = null;
  return true;
}

/** 是否正在平滑转动（界面层可用它显示状态） */
export function isViewSnapping(): boolean { return _viewSnap !== null; }
