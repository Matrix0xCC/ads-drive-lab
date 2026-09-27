// ==================== 渲染层：真实地图底图（瓦片层） ====================
//
// 【定位】纯**表现层**：底图只给人看，**不参与感知/规划/控制**，也不进归档。
// 因此它必须满足两条硬约束：
//   ① **缺失即降级**：瓦片缓存是 gitignore 的（第三方数据不进仓库）→ 新克隆没有缓存是常态。
//      任何一步失败（清单 404 / 瓦片 404 / 解码失败）都只是"没有底图"，不许影响仿真。
//   ② **异步不阻塞**：加载是 fire-and-forget，瓦片就绪一张显示一张。
//
// 【摆放】（三个容易搞错的地方，都写在代码里）
//   · 坐标：用 `maps/<name>.tiles.json` 里的 `worldOrigin`（世界坐标 (0,0) 的经纬度）换算，
//     不是 `geometry.origin`（那是**投影**原点；地图点列做过世界平移，两者差几百米~几公里）。
//   · **朝向三件事必须同时满足**（少一件就会"看起来对但其实是镜像/翻转"，实测踩过）：
//       ① 法线朝**上**（`rotation.x = -π/2`）→ 从上方看到的是**正面**；
//          若用 `+π/2`，法线朝下，从上方看到的是**背面** → **整张图左右镜像**（文字反着读）。
//       ② 图象**上方朝北**（世界 +z）。`-π/2` 会把平面的 local +Y 转到世界 **-z**（南），
//          所以必须配 `texture.flipY = false`（图象首行 → local -Y → 世界 +z ✓）。
//       ③ 图象**右方朝东**（世界 +x）——PlaneGeometry 的 u 沿 local +X，`-π/2` 不动 X ✓。
//     三者合起来才等于"把瓦片平铺在地上、北在上"。
//   · **世界是右手系（北 = −Z）**：由 `scene.scale.z = -1`（见 scene.ts 文件头）统一施加，
//     所以**本模块照旧用仿真坐标**（`tileWorldRect` 给的是 x=东、y=北、单位米）即可——
//     场景根的镜像会把它落到世界 −z；配合 `flipY = false`，图象首行正好朝北、右方朝东。
//     若哪天把场景根的镜像去掉，这里会**立刻**变成镜像（文字反读）——`world-convention.test.ts` 钉住了这条。
//     （渲染层其它平面用 -π/2 是因为它们先用 `Vector2(x, -y)` 做了镜像，别混。）
import * as THREE from 'three';
import { m } from '../../shared/dimensions.js';
import { tileWorldRect, type LatLon } from '../../shared/geo.js';

/** `maps/<name>.tiles.json` 的 schema（由 `npm run tiles` 生成） */
export interface TileManifest {
  version: number;
  name: string;
  provider: string;
  urlTemplate: string;
  attribution: string;
  note?: string;
  zoom: number;
  tileSizePx: number;
  bboxLatLon: { north: number; south: number; west: number; east: number };
  /** 世界坐标 (0,0) 的经纬度 */
  worldOrigin: [number, number];
  /** 缓存相对路径模板（相对仓库根） */
  cache: string;
  tiles: { z: number; x: number; y: number; sha256: string; bytes: number }[];
  fetchedAt?: string;
  generator?: string;
}

export interface BasemapHandle {
  group: THREE.Group;
  attribution: string;
  zoom: number;
  stats: { loaded: number; total: number; failed: number };
  /** 底图平面的世界 y（在地面 -0.1 与路面 0.01 之间） */
  setVisible(v: boolean): void;
  setOpacity(o: number): void;
  dispose(): void;
}

/** 底图 y：地面在 -0.1、路面在 0.01、网格在 0 —— 取中间，既不被地面挡住也不压住路面 */
export const BASEMAP_Y = -0.05;

/** 摆放计算的结果（纯数据，便于离线单测：不碰 THREE、不碰 DOM、不联网） */
export interface BasemapQuad {
  z: number; x: number; y: number;
  /** 世界坐标中心与宽高（米） */
  cx: number; cy: number; width: number; height: number;
  /** 缓存里的瓦片相对路径 */
  url: string;
}

/**
 * 清单 → 逐张瓦片的摆放（纯函数）。
 * 用 `worldOrigin`（世界 (0,0) 的经纬度）换算——**不是** `geometry.origin`（投影原点）。
 */
export function planBasemapQuads(manifest: TileManifest): BasemapQuad[] {
  const origin: LatLon = { lat: manifest.worldOrigin[0], lon: manifest.worldOrigin[1] };
  return manifest.tiles.map(t => {
    const rect = tileWorldRect(t.z, t.x, t.y, origin);
    return {
      z: t.z, x: t.x, y: t.y, ...rect,
      url: manifest.cache.replace('{z}', String(t.z)).replace('{x}', String(t.x)).replace('{y}', String(t.y)),
    };
  });
}

const loader = new THREE.TextureLoader();

/** 产物 key 的形状：小写字母/数字/._-（地图下拉的 key 就是它，例如 `oval`、`bj-g4-dujiakan`） */
const ARTIFACT_KEY_RE = /^[a-z0-9][a-z0-9._-]*$/;

/**
 * 读取清单（缺失/损坏 → null，属正常情况：缓存是 gitignore 的）。
 *
 * @param mapKey **地图产物 key**（`MAP_PRESETS` 的键），**不是** `MapDefinition.name`——
 *   后者对合成地图是显示名（`Oval` / `Tech Circuit`）。实测踩过：拿显示名去找 `maps/Oval.tiles.json`
 *   → 控制台一条 404 噪音（OSM 地图恰好 key==name，所以只在切换回径向图时才暴露）。
 *   这里先做形状检查：不像 key 就**直接跳过**（并给出可读日志），而不是发一个注定 404 的请求。
 */
export async function fetchTileManifest(mapKey: string): Promise<TileManifest | null> {
  if (!ARTIFACT_KEY_RE.test(mapKey)) {
    console.info(`[basemap] "${mapKey}" 不是地图产物 key（显示名？）→ 跳过底图查找`);
    return null;
  }
  try {
    const res = await fetch(`maps/${mapKey}.tiles.json`, { cache: 'no-cache' });
    if (!res.ok) return null;
    const m = await res.json() as TileManifest;
    if (!Array.isArray(m.tiles) || m.tiles.length === 0) return null;
    if (!Array.isArray(m.worldOrigin) || m.worldOrigin.length !== 2) return null;
    return m;
  } catch {
    return null;
  }
}

/**
 * 加载底图（异步、逐张）。返回 handle 供显示/隐藏/释放；没有清单时返回 null。
 * @param mapName 地图名（找 `maps/<name>.tiles.json`）
 */
/**
 * 米制瓦片矩形 → **场景（仿真像素）**矩形。
 *
 * `planBasemapQuads` 给的是米（地理量纲，便于与地图产物/经纬度对照），而场景里的
 * 路面/障碍/雷达都用仿真像素（`SIM_UNITS_PER_METER = 4 px/m`）。**漏了换算就是
 * "地图比跑道小 4 倍"**（实测踩过：跑道很长、底图只有一小块）。抽成纯函数是为了让
 * `js/test/renderer/basemap.test.ts` 能直接钉住这条（旧测试只查了米制地理框 → 漏掉了这个 bug）。
 */
export function quadToSceneRect(q: BasemapQuad): { cx: number; cy: number; width: number; height: number } {
  return { cx: m(q.cx) as number, cy: m(q.cy) as number, width: m(q.width) as number, height: m(q.height) as number };
}

export async function loadBasemap(mapKey: string, opts?: { dim?: number }): Promise<BasemapHandle | null> {
  const manifest = await fetchTileManifest(mapKey);
  if (!manifest) {
    console.info(`[basemap] ${mapKey} 无底图清单（maps/${mapKey}.tiles.json）——按"无底图"运行。`
      + `如需底图：npm run tiles -- --map ${mapKey}`);
    return null;
  }
  const group = new THREE.Group();
  group.visible = false;
  const dim = opts?.dim ?? 0.55;                    // 压暗系数：底图是上下文，不该抢路面
  const material = (tex: THREE.Texture) => new THREE.MeshBasicMaterial({
    map: tex,
    color: new THREE.Color(dim, dim, dim),          // 乘法压暗（同时削弱"瓦片路面 vs 仿真路面"的双线感）
    side: THREE.FrontSide,                            // 法线朝上，正面可见（背面镜像的坑见文件头）
    // 【关键】瓦片**不参与深度比较**：它与地面只差 0.05 仿真像素（1.25cm），
    // 而相机视距 250m~2km，深度缓冲精度远不足以分辨 → 深色地面（0x111111）会成片"穿"出来，
    // 表现为地图上大片**黑色乱纹**（缩放会改变深度分布，所以缩放后尤其明显）。实测踩过。
    // 改为 `depthTest = false` + `depthWrite = false` + `renderOrder = -1`：在路面/网格之前画，
    // 由它们正常覆盖瓦片——顺序由 renderOrder 决定，不再依赖深度精度。
    depthTest: false,
    depthWrite: false,
    // 必须是**不透明**材质：半透明会被排到所有不透明物体（路面/网格）之后 → 反过来盖住路面。
    // 压暗由上面的 color 乘法完成，不靠透明度。
    transparent: false, opacity: 1,
  });
  const stats = { loaded: 0, total: manifest.tiles.length, failed: 0 };
  const geometries: THREE.BufferGeometry[] = [];

  // 逐张加载：先建平面（几何与位置是纯数学，立即完成），纹理就绪后再挂材质
  for (const q of planBasemapQuads(manifest)) {
    // 【单位】`planBasemapQuads` 给的是**米**（地理量纲，便于与地图产物/经纬度对照），
    // 而场景（路面/障碍/雷达…）用的是**仿真像素**（SIM_UNITS_PER_METER = 4 px/m）。
    // 必须在这里换算：漏了就是"地图比跑道小 4 倍"（实测踩过：跑道很长、地图只有一小块）。
    const rect = quadToSceneRect(q);
    const t = { z: q.z, x: q.x, y: q.y };
    const geom = new THREE.PlaneGeometry(rect.width, rect.height);
    const mesh = new THREE.Mesh(geom, new THREE.MeshBasicMaterial({ visible: false }));
    mesh.rotation.x = -Math.PI / 2;                 // 法线朝上（看正面 → 不镜像）；上方朝北靠 flipY=false，见文件头
    mesh.position.set(rect.cx, BASEMAP_Y, rect.cy);
    // 【绘制顺序】地面 -2 → 瓦片 -1 → 路面/网格/障碍/雷达 0（理由见下面 material 的 depthTest 注释）
    mesh.renderOrder = -1;
    mesh.userData = { type: 'basemap', label: `底图瓦片 ${t.z}/${t.x}/${t.y}`, width: rect.width, height: 0, length: `${manifest.provider}` };
    group.add(mesh);
    geometries.push(geom);
    loader.load(q.url, (tex) => {
      // 【必须】图象首行 → 平面 local -Y → 世界 +z（北）。默认 flipY=true 会让图象上方落到南边
      // （整张图南北翻转；配合"看背面"时还容易被误判成"已经对了"）。
      tex.flipY = false;
      tex.colorSpace = THREE.SRGBColorSpace;        // 不设会明显发灰
      tex.anisotropy = 8;                           // 斜视可读（上限由设备决定，超了会被夹住）
      tex.generateMipmaps = true;
      tex.minFilter = THREE.LinearMipmapLinearFilter;
      mesh.material.dispose();
      mesh.material = material(tex);
      stats.loaded++;
    }, undefined, () => { stats.failed++; });        // 单张失败只是留白，不影响别的瓦片
  }
  return {
    group, attribution: manifest.attribution, zoom: manifest.zoom, stats,
    setVisible(v) { group.visible = v; },
    setOpacity(o) { group.traverse(o2 => { const m = (o2 as THREE.Mesh).material as THREE.MeshBasicMaterial | undefined; if (m && 'opacity' in m) m.opacity = o; }); },
    dispose() {
      group.traverse(o2 => {
        const mesh = o2 as THREE.Mesh;
        if (mesh.material) {
          const m = mesh.material as THREE.MeshBasicMaterial;
          m.map?.dispose(); m.dispose();
        }
      });
      for (const g of geometries) g.dispose();
      group.clear();
    },
  };
}
