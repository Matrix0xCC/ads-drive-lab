// ==================== 测试：入库地图产物的一致性契约 ====================
// 守住三件事：① 地图与**入库的原始 OSM 响应**一致（sha256 → 谁改坏了原始数据/地图立刻暴露）；
// ② 线形与步长在界内（防"重新生成出一个坏地图还提交"）；③ 收费站桩号落在路径范围内。
// 纯离线：只读 maps/ 下已入库的两个文件。
import { describe, it } from 'node:test';
import assert from 'node:assert';
import * as fs from 'node:fs';
import * as path from 'node:path';
import { createHash } from 'node:crypto';
import { curvatureStats, pathLengthM, type Vec2M } from '../../mapgen/osm.js';

const NAME = 'bj-g4-dujiakan';
const map = JSON.parse(fs.readFileSync(path.resolve('maps', `${NAME}.json`), 'utf8'));
const pts: Vec2M[] = (map.geometry.points as [number, number][]).map(([x, y]) => ({ x, y }));

describe('mapgen/入库地图产物契约', () => {
  it('与入库的原始 OSM 响应一致（sha256 + 许可署名）', () => {
    const sha = createHash('sha256')
      .update(fs.readFileSync(path.resolve('maps/raw', `${NAME}.osm.json`), 'utf8')).digest('hex');
    assert.match(map.source.rawSha256, /^[0-9a-f]{64}$/);
    assert.strictEqual(map.source.rawSha256, sha, '地图与原始数据的 sha256 不一致（原始数据被改动？）');
    assert.strictEqual(map.source.license, 'ODbL-1.0');
    assert.ok(String(map.source.attribution).includes('OpenStreetMap'));
    assert.ok(Array.isArray(map.source.wayIds) && map.source.wayIds.length > 0);
  });

  it('开环 + 步长均匀 + 长度自洽', () => {
    assert.strictEqual(map.closed, false);
    assert.strictEqual(map.kind, 'osm-chain');
    assert.ok(pts.length > 500, `点数 ${pts.length} 太少`);
    const gaps = pts.slice(1).map((p, i) => Math.hypot(p.x - pts[i].x, p.y - pts[i].y));
    assert.ok(Math.max(...gaps) < map.geometry.stepM + 0.5, `最大步长 ${Math.max(...gaps)}m 超界`);
    assert.ok(Math.abs(pathLengthM(pts) - map.validation.lengthM) < 1,
      `长度不自洽：实测 ${pathLengthM(pts)} vs 记录 ${map.validation.lengthM}`);
  });

  it('世界尺寸包住点列（运行时按它取景/建地面）', () => {
    const xs = pts.map(p => p.x), ys = pts.map(p => p.y);
    const bboxW = Math.max(...xs) - Math.min(...xs);
    const bboxH = Math.max(...ys) - Math.min(...ys);
    assert.ok(map.world.wM >= bboxW && map.world.hM >= bboxH,
      `世界 ${map.world.wM}×${map.world.hM} 装不下点列包围盒 ${bboxW}×${bboxH}`);
    assert.ok(map.world.marginM > 0);
    assert.ok(Math.min(...xs) >= map.world.marginM - 1 && Math.min(...ys) >= map.world.marginM - 1,
      '点列应已归一化到世界坐标（含边距）');
  });

  it('生成的 TS 模块与地图文件同步（防止"改了 JSON 忘了提交 gen 模块"）', () => {
    const gen = fs.readFileSync(path.resolve('js/env/maps', `${NAME}.gen.ts`), 'utf8');
    assert.ok(gen.includes(map.source.rawSha256),
      'gen 模块里的 sha256 与地图文件不一致（重跑 npm run mapgen 生成）');
    assert.ok(gen.includes(`halfWidth: m(${map.road.halfWidthM})`), 'gen 模块半宽与地图不一致');
    assert.ok(gen.includes(`closed: ${map.closed}`), 'gen 模块开闭环标志与地图不一致');
    const genPts = (gen.match(/\{ x: m\(/g) ?? []).length;
    assert.strictEqual(genPts, pts.length, `gen 模块点数 ${genPts} ≠ 地图 ${pts.length}`);
  });

  it('线形在高速可接受范围（κ p95 对应半径 ≥ 250m）', () => {
    const k = curvatureStats(pts, 5);
    assert.ok(k.p95 < 0.004, `κ p95=${k.p95.toExponential(2)}（R≈${(1 / k.p95).toFixed(0)}m）`);
    assert.ok(k.max < 0.01, `κ max=${k.max.toExponential(2)}（R≈${(1 / k.max).toFixed(0)}m）`);
  });

  it('收费站桩号落在路径范围内；道路参数自洽', () => {
    assert.ok(Array.isArray(map.tollgates) && map.tollgates.length >= 1, '应至少锚定一个收费站');
    for (const g of map.tollgates as { sM: number; osmId: number }[]) {
      assert.ok(g.sM >= 0 && g.sM <= map.validation.lengthM + 1, `收费站 s=${g.sM} 超出 [0,${map.validation.lengthM}]`);
      assert.ok(Number.isFinite(g.osmId));
    }
    assert.ok(map.road.laneCount >= 1 && map.road.halfWidthM > 0);
    assert.strictEqual(map.road.oneway, true);
    assert.ok(Math.abs(map.road.halfWidthM - map.road.laneCount * map.road.laneWidthM / 2) < 1e-6);
  });
});
