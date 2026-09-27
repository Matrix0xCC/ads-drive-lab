// ==================== 测试：共享地理换算 ====================
//
// 这些数字直接决定"底图会不会整体偏移"，而偏移是肉眼很难发现的错误 →
// 必须把公式、常数一致性与那个 30% 的墨卡托坑都钉住。
import { describe, it } from 'node:test';
import assert from 'node:assert';
import {
  EARTH_R_M, scaleFactors, latLonToLocalMeters, localMetersToLatLon,
  latLonToTile, tileBounds, tileResolutionMetersPerPx, tileRangeForBbox, tileWorldRect,
} from '../../shared/geo.js';
import { projectToEnu, EARTH_R_M as MAPGEN_EARTH_R_M } from '../../mapgen/osm.js';

const ORIGIN = { lat: 39.795, lon: 116.1937 };   // G4 杜家坎地图的投影原点

describe('geo：与地图管线同源', () => {
  it('地球半径常数与 mapgen 一致（防止两份公式漂移）', () => {
    assert.strictEqual(EARTH_R_M, MAPGEN_EARTH_R_M);
  });

  it('latLonToLocalMeters 与 mapgen 的 projectToEnu 逐点一致（< 1mm）', () => {
    const pts = [
      { lat: 39.795, lon: 116.1937 },
      { lat: 39.80, lon: 116.20 },
      { lat: 39.81, lon: 116.185 },
      { lat: 39.79, lon: 116.21 },
    ];
    const a = projectToEnu(pts, ORIGIN);
    for (let i = 0; i < pts.length; i++) {
      const b = latLonToLocalMeters(pts[i], ORIGIN);
      assert.ok(Math.hypot(a[i].x - b.x, a[i].y - b.y) < 1e-3,
        `第 ${i} 点不一致：projectToEnu=(${a[i].x},${a[i].y}) vs geo=(${b.x},${b.y})`);
    }
  });

  it('局部米制 → 经纬度 是单向的逆（往返 < 1cm）', () => {
    for (const p of [{ lat: 39.80, lon: 116.20 }, { lat: 39.78, lon: 116.17 }]) {
      const m = latLonToLocalMeters(p, ORIGIN);
      const back = localMetersToLatLon(m.x, m.y, ORIGIN);
      const d = Math.hypot((back.lat - p.lat) * 111320, (back.lon - p.lon) * 111320 * Math.cos(ORIGIN.lat * Math.PI / 180));
      assert.ok(d < 0.01, `往返误差 ${d}m`);
    }
  });
});

describe('geo：Web Mercator 瓦片', () => {
  it('**墨卡托的"米"不是米**：北京 39.9° 处比例因子 ≈ 1.30（差 30%）', () => {
    const { kx, ky } = scaleFactors({ lat: 39.9, lon: 0 });
    assert.ok(Math.abs(ky / kx - 1 / Math.cos(39.9 * Math.PI / 180)) < 1e-9, '纬度比例因子应为 1/cos φ');
    const eq = tileResolutionMetersPerPx(16, 0);
    const bj = tileResolutionMetersPerPx(16, 39.9);
    assert.ok(Math.abs(eq - 2.387) < 0.01, `赤道 z=16 应 ≈2.39，实际 ${eq}`);
    // cos(39.9°) = 0.767 → 北京比赤道小 23%
    assert.ok(Math.abs(bj - eq * Math.cos(39.9 * Math.PI / 180)) < 1e-9, `应按 cos φ 收缩，实际 ${bj}`);
    assert.ok(Math.abs(bj - 1.832) < 0.01, `北京 z=16 应 ≈1.83 m/px，实际 ${bj}`);
  });

  it('瓦片编号 ↔ 经纬度 往返一致（含小数坐标）', () => {
    for (const z of [14, 17, 18]) {
      for (const p of [{ lat: 39.795, lon: 116.1937 }, { lat: 39.9665, lon: 116.3743 }]) {
        const t = latLonToTile(p.lat, p.lon, z);
        const b = tileBounds(z, Math.floor(t.x), Math.floor(t.y));
        assert.ok(p.lat <= b.north + 1e-9 && p.lat >= b.south - 1e-9, `z=${z} 纬度不在瓦片内`);
        assert.ok(p.lon >= b.west - 1e-9 && p.lon <= b.east + 1e-9, `z=${z} 经度不在瓦片内`);
      }
    }
  });

  it('瓦片范围覆盖整个 bbox，且数量 = 网格数', () => {
    const bbox = { north: 39.99, south: 39.96, west: 116.35, east: 116.40 };
    const r = tileRangeForBbox(bbox, 17);
    assert.strictEqual(r.count, (r.x1 - r.x0 + 1) * (r.y1 - r.y0 + 1));
    for (const p of [
      { lat: bbox.north, lon: bbox.west }, { lat: bbox.south, lon: bbox.east },
      { lat: bbox.north, lon: bbox.east }, { lat: bbox.south, lon: bbox.west },
    ]) {
      const t = latLonToTile(p.lat, p.lon, 17);
      assert.ok(Math.floor(t.x) >= r.x0 && Math.floor(t.x) <= r.x1);
      assert.ok(Math.floor(t.y) >= r.y0 && Math.floor(t.y) <= r.y1);
    }
  });

  it('瓦片摆放矩形：中心与宽高落在世界坐标里，宽高与分辨率自洽', () => {
    const z = 17, x = 107878, y = 49683;
    const rect = tileWorldRect(z, x, y, ORIGIN);
    const b = tileBounds(z, x, y);
    const res = tileResolutionMetersPerPx(z, (b.north + b.south) / 2);
    assert.ok(Math.abs(rect.width - 256 * res) / (256 * res) < 0.08,
      `宽 ${rect.width.toFixed(1)} vs 256×${res.toFixed(2)}=${(256 * res).toFixed(1)}`);
    assert.ok(Math.abs(rect.height - 256 * res) / (256 * res) < 0.08,
      `高 ${rect.height.toFixed(1)} vs 256×${res.toFixed(2)}=${(256 * res).toFixed(1)}`);
    const c = latLonToLocalMeters({ lat: (b.north + b.south) / 2, lon: (b.west + b.east) / 2 }, ORIGIN);
    assert.ok(Math.abs(c.x - rect.cx) < 1e-6 && Math.abs(c.y - rect.cy) < 1e-6);
  });
});
