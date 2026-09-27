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
import { planBasemapQuads, fetchTileManifest, BASEMAP_Y, type TileManifest } from '../../env/renderer/basemap.js';
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

describe('basemap：朝向（图片上方必须朝北，即世界 +z）', () => {
  it('rotation.x = +π/2 时，平面的"上边缘"在世界 +z 方向', () => {
    const [q] = planBasemapQuads(manifest);
    // 与 basemap.ts 里完全一致的摆放方式
    const mesh = new THREE.Mesh(new THREE.PlaneGeometry(q.width, q.height));
    mesh.rotation.x = Math.PI / 2;
    mesh.position.set(q.cx, BASEMAP_Y, q.cy);
    mesh.updateMatrixWorld(true);
    const top = new THREE.Vector3(0, q.height / 2, 0).applyMatrix4(mesh.matrixWorld);    // 贴图上边缘
    const bottom = new THREE.Vector3(0, -q.height / 2, 0).applyMatrix4(mesh.matrixWorld);
    assert.ok(top.z > bottom.z, `贴图上方必须朝北（世界 +z）：top.z=${top.z} 应 > bottom.z=${bottom.z}`);
    // 并且正好等于矩形中心 ± 半高（没有意外的缩放/旋转）
    assert.ok(Math.abs(top.z - (q.cy + q.height / 2)) < 1e-6, `上边缘 z 应为 cy+高/2，实际 ${top.z}`);
    assert.ok(Math.abs(top.x - q.cx) < 1e-6);
    assert.ok(Math.abs(top.y - BASEMAP_Y) < 1e-6, '底图平面应贴在 BASEMAP_Y');
  });
});

describe('basemap：无缓存时优雅降级', () => {
  it('没有清单 → fetchTileManifest 返回 null（不抛异常）', async () => {
    assert.strictEqual(await fetchTileManifest('__不存在的底图__'), null);
  });
});
