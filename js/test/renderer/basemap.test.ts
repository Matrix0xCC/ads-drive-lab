// ==================== 测试：底图瓦片的摆放与朝向 ====================
//
// 画面我在无头环境里看不到，所以把**能算的**都钉住：
//   ① 清单 schema 与缓存路径约定；
//   ② 每张瓦片的世界矩形落在世界范围内（含 buffer），尺寸 = 256px × 该纬度分辨率；
//   ③ **朝向**：图片上方必须指向北（世界 +z）——这条只能靠矩阵算出来验，肉眼很难发现南北颠倒；
//   ④ 缓存存在时，清单里的 sha256 必须与磁盘上的瓦片一致（清单的完整性声明）；
//   ⑤ 没有清单时优雅降级为 null（缓存是 gitignore 的，新克隆就是没有）。
import { describe, it } from 'node:test';
import assert from 'node:assert';
import * as fs from 'node:fs';
import * as path from 'node:path';
import * as THREE from 'three';
import { createHash } from 'node:crypto';
import { planBasemapQuads, fetchTileManifest, quadToSceneRect, BASEMAP_Y, type TileManifest } from '../../env/renderer/basemap.js';
import { SIM_UNITS_PER_METER } from '../../shared/dimensions.js';
import { MAP_PRESETS } from '../../env/mapDefinitions.js';
import { tileResolutionMetersPerPx, tileBounds } from '../../shared/geo.js';

const NAME = 'bj-g4-dujiakan';
const manifestPath = path.resolve('maps', `${NAME}.tiles.json`);
const mapPath = path.resolve('maps', `${NAME}.json`);
const manifest = JSON.parse(fs.readFileSync(manifestPath, 'utf8')) as TileManifest;
const map = JSON.parse(fs.readFileSync(mapPath, 'utf8'));

describe('basemap：清单与缓存约定', () => {
  it('清单字段齐全，且标明来源/署名（渲染时必须显示）', () => {
    assert.strictEqual(manifest.name, NAME);
    assert.match(manifest.provider, /^[a-z-]+$/);
    assert.match(manifest.attribution, /OpenStreetMap|CARTO/);
    assert.ok(manifest.urlTemplate.startsWith('https://'), '来源模板应是 https');
    assert.ok(Number.isInteger(manifest.zoom) && manifest.zoom >= 10 && manifest.zoom <= 19);
    assert.strictEqual(manifest.tileSizePx, 256);
    assert.ok(manifest.tiles.length > 0);
    assert.strictEqual(manifest.cache, `maps/tiles/${manifest.provider}/{z}/{x}/{y}.png`);
  });

  it('worldOrigin 是合法经纬度，且与地图产物的 originWorld 一致', () => {
    const [lat, lon] = manifest.worldOrigin;
    assert.ok(lat > 3 && lat < 54 && lon > 73 && lon < 135, `worldOrigin 非法：[${lat}, ${lon}]`);
    assert.strictEqual(lat, (map.geometry.originWorld as number[])[0]);
    assert.strictEqual(lon, (map.geometry.originWorld as number[])[1]);
  });
});

describe('basemap：瓦片摆放（纯函数）', () => {
  const quads = planBasemapQuads(manifest);

  it('每张瓦片一个矩形，尺寸 = 256px × 该纬度分辨率（±8%：墨卡托纬度压缩）', () => {
    assert.strictEqual(quads.length, manifest.tiles.length);
    for (const q of quads) {
      const b = tileBounds(q.z, q.x, q.y);
      const res = tileResolutionMetersPerPx(q.z, (b.north + b.south) / 2);
      assert.ok(Math.abs(q.width - 256 * res) / (256 * res) < 0.09, `宽 ${q.width.toFixed(1)} vs ${(256 * res).toFixed(1)}`);
      assert.ok(Math.abs(q.height - 256 * res) / (256 * res) < 0.09, `高 ${q.height.toFixed(1)} vs ${(256 * res).toFixed(1)}`);
      assert.ok(q.width > 1 && q.height > 1);
    }
  });

  it('缓存路径符合约定，且都在同一个 provider/z 目录下', () => {
    for (const q of quads) {
      assert.match(q.url, new RegExp(`^maps/tiles/${manifest.provider}/${q.z}/\\d+/\\d+\\.png$`));
    }
  });

  it('瓦片并集覆盖整条道路，且不超出"世界范围 + buffer + 一张瓦片"', () => {
    // 道路点的世界坐标范围（链式地图的 points 就是道路）
    const pts = map.geometry.points as [number, number][];
    let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
    for (const [x, y] of pts) {
      minX = Math.min(minX, x); maxX = Math.max(maxX, x);
      minY = Math.min(minY, y); maxY = Math.max(maxY, y);
    }
    const qminX = Math.min(...quads.map(q => q.cx - q.width / 2));
    const qmaxX = Math.max(...quads.map(q => q.cx + q.width / 2));
    const qminY = Math.min(...quads.map(q => q.cy - q.height / 2));
    const qmaxY = Math.max(...quads.map(q => q.cy + q.height / 2));
    // 覆盖道路
    assert.ok(qminX <= minX && qmaxX >= maxX && qminY <= minY && qmaxY >= maxY, '瓦片必须盖住整条道路');
    // 不无限外扩：外扩量 ≤ buffer(120m) + 一张瓦片
    const tile = 256 * tileResolutionMetersPerPx(manifest.zoom, manifest.worldOrigin[0]);
    const slack = 120 + tile + 1;
    assert.ok(minX - qminX <= slack && qminY - minY <= slack, `外扩过大：x ${(minX - qminX).toFixed(0)}m y ${(minY - qminY).toFixed(0)}m`);
  });

  it('缓存存在时：清单里的 sha256 与磁盘文件一致（清单的完整性声明）', () => {
    const haveCache = quads.every(q => fs.existsSync(path.resolve(q.url)));
    if (!haveCache) return;                     // 缓存是 gitignore 的：没有就跳过（不是失败）
    for (const q of quads) {
      const bytes = fs.readFileSync(path.resolve(q.url));
      const sha = createHash('sha256').update(bytes).digest('hex');
      const want = manifest.tiles.find(t => t.z === q.z && t.x === q.x && t.y === q.y)!;
      assert.strictEqual(sha, want.sha256, `${q.url} 与清单不符（缓存被改过？）`);
      assert.strictEqual(bytes.length, want.bytes);
    }
  });
});

describe('basemap：朝向（三条不变量，缺一条就会镜像/翻转）', () => {
  /** 与 basemap.ts 完全一致的摆放：rotation.x = -π/2 + texture.flipY = false */
  function place(q: { width: number; height: number; cx: number; cy: number }) {
    const mesh = new THREE.Mesh(new THREE.PlaneGeometry(q.width, q.height));
    mesh.rotation.x = -Math.PI / 2;
    mesh.position.set(q.cx, BASEMAP_Y, q.cy);
    mesh.updateMatrixWorld(true);
    const at = (x: number, y: number) => new THREE.Vector3(x, y, 0).applyMatrix4(mesh.matrixWorld);
    return {
      // texture.flipY = false ⇒ 图象**首行（上方）**对应平面 local -Y，末行对应 local +Y
      imageTop: at(0, -q.height / 2),
      imageBottom: at(0, q.height / 2),
      imageLeft: at(-q.width / 2, 0),
      imageRight: at(q.width / 2, 0),
      normal: new THREE.Vector3(0, 0, 1).transformDirection(mesh.matrixWorld),
    };
  }

  // 走**真实代码路径**取场景矩形（含"米 → 仿真像素"换算）：朝向与单位两件事一起被覆盖。
  // 注意场景根还会做一次 z 镜像（见 world-convention.test.ts）——镜像对所有内容一视同仁，
  // 所以这里在"地理帧"里的相对断言（图象上方 z 更大）在镜像后依然成立。
  const [q] = planBasemapQuads(manifest).map(quadToSceneRect);

  it('① 法线朝上 → 从上方看到的是**正面**（法线朝下会看到背面 → 整张图左右镜像，文字反读）', () => {
    const { normal } = place(q);
    assert.ok(normal.y > 0.99, `法线必须朝上（+Y），实际 (${normal.x.toFixed(2)}, ${normal.y.toFixed(2)}, ${normal.z.toFixed(2)})`);
    assert.ok(Math.abs(normal.x) < 1e-6 && Math.abs(normal.z) < 1e-6, '法线不应有水平分量');
  });

  it('② 图象上方朝北（世界 +z）、下方朝南', () => {
    const { imageTop, imageBottom } = place(q);
    assert.ok(imageTop.z > imageBottom.z, `图象上方应朝北：top.z=${imageTop.z} 必须 > bottom.z=${imageBottom.z}`);
    assert.ok(Math.abs(imageTop.z - (q.cy + q.height / 2)) < 1e-6, '上边缘 z = cy + 高/2');
  });

  it('③ 图象右方朝东（世界 +x）、左方朝西（镜像会让两者换位）', () => {
    const { imageLeft, imageRight } = place(q);
    assert.ok(imageRight.x > imageLeft.x, `图象右方应朝东：right.x=${imageRight.x} 必须 > left.x=${imageLeft.x}`);
  });

  it('底图平面贴在地面与路面之间（-0.1 < y < 0.01）', () => {
    const { imageTop } = place(q);
    assert.ok(Math.abs(imageTop.y - BASEMAP_Y) < 1e-6);
    assert.ok(BASEMAP_Y > -0.1 && BASEMAP_Y < 0.01, `BASEMAP_Y=${BASEMAP_Y} 应在（地面 -0.1, 路面 0.01）之间`);
  });
});

describe('basemap：无缓存时优雅降级', () => {
  it('没有清单 → fetchTileManifest 返回 null（不抛异常）', async () => {
    assert.strictEqual(await fetchTileManifest('__不存在的底图__'), null);
  });
});

// ─────────────────────────────────────────────────────────────
// 底图查找必须用**地图产物 key**，不能用显示名
// （实测：拿 `MapDefinition.name` 去找 → `GET /maps/Oval.tiles.json 404`；
//   OSM 地图恰好 key==name，所以只在切回径向图时才暴露）
// ─────────────────────────────────────────────────────────────
describe('basemap：查找 key 的约定', () => {
  it('不像产物 key 的名字（显示名）→ 直接跳过，**不发请求**', async () => {
    const orig = globalThis.fetch;
    let called = 0;
    globalThis.fetch = (async () => { called++; throw new Error('不该发请求'); }) as typeof fetch;
    try {
      for (const display of ['Oval', 'Tech Circuit', 'Hairpin']) {
        assert.strictEqual(await fetchTileManifest(display), null, `${display} 应返回 null`);
      }
      assert.strictEqual(called, 0, '显示名不该触发任何网络请求（否则控制台就是 404 噪音）');
    } finally { globalThis.fetch = orig; }
  });

  it('合法的 key 形状 → 会去查清单（查不到也只是 null，不抛）', async () => {
    const orig = globalThis.fetch;
    let called = 0;
    globalThis.fetch = (async () => { called++; return { ok: false } as Response; }) as typeof fetch;
    try {
      assert.strictEqual(await fetchTileManifest('no-such-map'), null);
      assert.strictEqual(called, 1, '形状合法的 key 应当真的去查（判据不能被过度拦截）');
    } finally { globalThis.fetch = orig; }
  });

  it('地图定义契约：非径向地图的 name 必须是合法产物 key；径向地图一律跳过底图', () => {
    const keyRe = /^[a-z0-9][a-z0-9._-]*$/;
    let radialWithDisplayName = 0;
    for (const [key, def] of Object.entries(MAP_PRESETS)) {
      assert.ok(keyRe.test(key), `preset key "${key}" 必须是合法产物 key`);
      if (def.kind === 'radial') {
        radialWithDisplayName++;                     // 径向图没有地理坐标 → 不查底图
      } else {
        assert.ok(keyRe.test(String(def.name)),
          `非径向地图 ${key} 的 name（"${def.name}"）必须是合法产物 key —— 底图/产物查找都用它`);
      }
    }
    assert.ok(radialWithDisplayName > 0, '应当存在径向（合成）地图，它们正是"没有底图"的那一类');
  });
});

// ─────────────────────────────────────────────────────────────
// 单位换算：米（地图产物）→ 仿真像素（场景）—— 实测踩过"地图比跑道小 4 倍"
// 旧测试只查了米制地理框，所以完全没发现摆放时漏了 ×4；这里补上场景量纲这一环。
// ─────────────────────────────────────────────────────────────
describe('basemap：单位换算（米 → 仿真像素）', () => {
  const quads = planBasemapQuads(manifest);
  const bbox = (rs: { cx: number; cy: number; width: number; height: number }[]) => ({
    w: Math.max(...rs.map(r => r.cx + r.width / 2)) - Math.min(...rs.map(r => r.cx - r.width / 2)),
    h: Math.max(...rs.map(r => r.cy + r.height / 2)) - Math.min(...rs.map(r => r.cy - r.height / 2)),
  });

  it('quadToSceneRect 恰好是米 × SIM_UNITS_PER_METER（不是"看着差不多"）', () => {
    const r0 = quadToSceneRect(quads[0]), q0 = quads[0];
    assert.strictEqual(r0.cx, q0.cx * SIM_UNITS_PER_METER);
    assert.strictEqual(r0.cy, q0.cy * SIM_UNITS_PER_METER);
    assert.strictEqual(r0.width, q0.width * SIM_UNITS_PER_METER);
    assert.strictEqual(r0.height, q0.height * SIM_UNITS_PER_METER);
  });

  it('瓦片覆盖在**仿真像素**下与地图世界尺寸相当（差 4 倍就是那个 bug）', () => {
    assert.strictEqual(SIM_UNITS_PER_METER, 4, '本测试的判据依赖 4 px/m');
    const scene = bbox(quads.map(quadToSceneRect));
    const worldW = (map.world.wM as number) * SIM_UNITS_PER_METER;
    const worldH = (map.world.hM as number) * SIM_UNITS_PER_METER;
    const ratioW = scene.w / worldW, ratioH = scene.h / worldH;
    assert.ok(ratioW > 0.9 && ratioW < 1.6, `底图覆盖宽 ${scene.w.toFixed(0)}px vs 世界 ${worldW.toFixed(0)}px（比值 ${ratioW.toFixed(2)}）——小于 0.9 说明忘了换算单位`);
    assert.ok(ratioH > 0.9 && ratioH < 1.6, `底图覆盖高 ${scene.h.toFixed(0)}px vs 世界 ${worldH.toFixed(0)}px（比值 ${ratioH.toFixed(2)}）`);
  });

});
