// ==================== 测试：地图产物的地理配准契约 ====================
//
// 【为什么必须有】`geometry.origin` 曾经是"链式地图写 [lat,lon]、路网地图写 [0,0]（米）"——
// 同一个字段两种含义，而且路网那支恒为零。这类错误**不会影响任何仿真**（没人读它），
// 只会让"将来想做底图/瓦片/任何经纬度相关的事"时整体偏出几公里，且极难察觉。
//
// 判据：把地图的**世界 bbox** 通过 `originWorld` 换算回经纬度，必须落在**入库原始 OSM 响应**
// 的经纬度范围内（原始响应是超集：链式地图做过 bbox/长度裁剪、路网做过 clip）。
// 纯离线：只读 maps/ 下已入库的文件。
import { describe, it } from 'node:test';
import assert from 'node:assert';
import * as fs from 'node:fs';
import * as path from 'node:path';
import { localMetersToLatLon } from '../../shared/geo.js';

const NAMES = ['bj-g4-dujiakan', 'bj-g6-junction', 'bj-3rd-ring'];

/** 原始 OSM 响应里所有 way 顶点的经纬度 bbox（含裁剪前的范围，是超集） */
function rawBbox(name: string): { north: number; south: number; west: number; east: number } {
  const j = JSON.parse(fs.readFileSync(path.resolve('maps/raw', `${name}.osm.json`), 'utf8'));
  let north = -90, south = 90, west = 180, east = -180;
  for (const el of j.elements ?? []) {
    if (el.type !== 'way' || !Array.isArray(el.geometry)) continue;
    for (const p of el.geometry) {
      north = Math.max(north, p.lat); south = Math.min(south, p.lat);
      west = Math.min(west, p.lon); east = Math.max(east, p.lon);
    }
  }
  return { north, south, west, east };
}

describe('mapgen/入库地图的地理配准契约', () => {
  for (const name of NAMES) {
    it(`${name}：origin / originWorld 都是合法经纬度`, () => {
      const m = JSON.parse(fs.readFileSync(path.resolve('maps', `${name}.json`), 'utf8'));
      for (const key of ['origin', 'originWorld'] as const) {
        const o = m.geometry[key] as [number, number] | undefined;
        assert.ok(Array.isArray(o) && o.length === 2, `${name}.geometry.${key} 缺失`);
        const [lat, lon] = o;
        assert.ok(lat > 3 && lat < 54 && lon > 73 && lon < 135,
          `${name}.geometry.${key} 不像中国境内的经纬度：[${lat}, ${lon}]（曾把"米"写进这个字段）`);
        // 7 位小数 ≈ 1cm：4 位（≈11m）对配准太粗
        assert.ok(String(lat).split('.')[1]?.length >= 6, `${name}.geometry.${key} 纬度精度不足（应 ≥6 位小数）`);
      }
    });

    it(`${name}：世界 bbox 换算回经纬度后落在原始 OSM 范围内（能抓住原点写错）`, () => {
      const m = JSON.parse(fs.readFileSync(path.resolve('maps', `${name}.json`), 'utf8'));
      const ow = m.geometry.originWorld as [number, number];
      const origin = { lat: ow[0], lon: ow[1] };
      // 世界坐标系：x ∈ [0, wM]、y ∈ [0, hM]（y 向北）
      const w = m.world.wM as number, h = m.world.hM as number;
      const corners = [[0, 0], [w, 0], [0, h], [w, h]].map(([x, y]) => localMetersToLatLon(x, y, origin));
      const mapBox = {
        north: Math.max(...corners.map(c => c.lat)), south: Math.min(...corners.map(c => c.lat)),
        west: Math.min(...corners.map(c => c.lon)), east: Math.max(...corners.map(c => c.lon)),
      };
      const raw = rawBbox(name);
      // 容差 250m：世界尺寸含 60m 边距 + 平滑/裁剪；关键是"量级对上"，不是逐字节相等
      const dLat = 250 / 111320;
      const dLon = 250 / (111320 * Math.cos(origin.lat * Math.PI / 180));
      assert.ok(mapBox.north <= raw.north + dLat && mapBox.south >= raw.south - dLat,
        `${name} 纬度范围 [${mapBox.south.toFixed(5)}, ${mapBox.north.toFixed(5)}] 超出原始 [${raw.south.toFixed(5)}, ${raw.north.toFixed(5)}]`);
      assert.ok(mapBox.west >= raw.west - dLon && mapBox.east <= raw.east + dLon,
        `${name} 经度范围 [${mapBox.west.toFixed(5)}, ${mapBox.east.toFixed(5)}] 超出原始 [${raw.west.toFixed(5)}, ${raw.east.toFixed(5)}]`);
      // 世界尺寸不能退化
      assert.ok(w > 100 && h > 100, `${name} 世界尺寸异常 ${w}×${h}`);
    });
  }

  it('路网地图的世界坐标确实是"世界系"（第一个节点不在原点附近）', () => {
    for (const name of ['bj-g6-junction', 'bj-3rd-ring']) {
      const m = JSON.parse(fs.readFileSync(path.resolve('maps', `${name}.json`), 'utf8'));
      const n0 = m.network.nodes[0] as [number, number];
      assert.ok(Math.hypot(n0[0], n0[1]) > 1,
        `${name} 首节点 ${JSON.stringify(n0)} 落在世界原点附近——世界平移没生效？`);
      // 全部节点应落在 [0, wM] × [0, hM] 内
      for (const [x, y] of m.network.nodes as [number, number][]) {
        assert.ok(x >= -1 && x <= m.world.wM + 1 && y >= -1 && y <= m.world.hM + 1,
          `${name} 节点 (${x}, ${y}) 超出世界范围 ${m.world.wM}×${m.world.hM}`);
      }
    }
  });
});
