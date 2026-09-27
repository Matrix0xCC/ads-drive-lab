// ==================== 底图瓦片取数（构建期；清单入库、瓦片入缓存） ====================
//
// 用法：
//   npm run tiles -- --map bj-g4-dujiakan                        # 默认 z=17、缓存到 maps/tiles/
//   npm run tiles -- --map bj-3rd-ring --zoom 17 --buffer 120
//   npm run tiles -- --map bj-g4-dujiakan --dry-run              # 只报数量和体积，不联网
//
// 产出：
//   maps/<name>.tiles.json                     ← **入库**（几百字节、可 diff）：来源/署名/范围/每张 sha256
//   maps/tiles/<provider>/<z>/<x>/<y>.png      ← **gitignore 缓存**（第三方数据不进仓库历史）
//
// 【为什么这样分工】瓦片是**别人的数据 + 外部世界的快照**：
//   ① 许可：OSM 官方瓦片政策不允许批量下载，把几百张别人的瓦片提交进 git 属于再分发灰色地带；
//   ② 仓库卫生：二进制不可 diff，每次重生成都会永久增加历史体积（而 maps/*.json 的"逐字节可复现"
//      性质瓦片本来就没有——明天同一张瓦片就可能变）；
//   ③ 可用性：本项目所在的机器连不上官方站（实测超时），取数本身不保证成功。
// 所以：**清单入库（能回答"我们用的是哪一批"）、瓦片入缓存（缺失时仿真优雅降级为无底图）**。
//
// 【量纲】瓦片是 Web Mercator，其"米"不是米（cos φ 修正，北京差 30%）——所有换算都走
// `js/shared/geo.ts`，不要把瓦片像素当米。
import fs from 'node:fs';
import path from 'node:path';
import { createHash } from 'node:crypto';
import {
  localMetersToLatLon, tileRangeForBbox, tileResolutionMetersPerPx, type LatLon,
} from '../shared/geo.js';

interface Provider {
  template: string;
  attribution: string;
  /** 使用须知（进清单，提醒下一个人） */
  note: string;
}

const PROVIDERS: Record<string, Provider> = {
  'osm-de': {
    template: 'https://tile.openstreetmap.de/{z}/{x}/{y}.png',
    attribution: '© OpenStreetMap contributors',
    note: 'OSM 社区镜像（实测本项目可达）。请遵守 OSM 瓦片政策：不要批量抓取、要带可识别的 UA。',
  },
  osm: {
    template: 'https://tile.openstreetmap.org/{z}/{x}/{y}.png',
    attribution: '© OpenStreetMap contributors',
    note: 'OSMF 官方瓦片服务：其政策**明确禁止**批量/自动化大量下载；仅在小范围教学用途下少量取用。',
  },
  'carto-light': {
    template: 'https://basemaps.cartocdn.com/light_all/{z}/{x}/{y}.png',
    attribution: '© OpenStreetMap contributors, © CARTO',
    note: 'CARTO 公共底图（浅色样式，适合当上下文底图）。',
  },
};

const TILE_SIZE_PX = 256;
const UA = 'ads-drive-lab/1.0 (offline driving simulator; educational; tiles fetched manually)';

interface Args {
  map: string; provider: string; zoom: number; bufferM: number;
  outDir: string; manifestDir: string; max: number; gapMs: number; dryRun: boolean;
}

function parseArgs(argv: string[]): Args {
  const a: Args = {
    map: '', provider: 'osm-de', zoom: 17, bufferM: 120,
    outDir: path.join('maps', 'tiles'), manifestDir: 'maps',
    max: 600, gapMs: 500, dryRun: false,
  };
  for (let i = 0; i < argv.length; i++) {
    const k = argv[i];
    if (k === '--map') a.map = argv[++i];
    else if (k === '--provider') a.provider = argv[++i];
    else if (k === '--zoom') a.zoom = Number(argv[++i]);
    else if (k === '--buffer') a.bufferM = Number(argv[++i]);
    else if (k === '--out') a.outDir = argv[++i];
    else if (k === '--max') a.max = Number(argv[++i]);
    else if (k === '--gap-ms') a.gapMs = Number(argv[++i]);
    else if (k === '--dry-run') a.dryRun = true;
    else throw new Error(`未知参数: ${k}`);
  }
  if (!a.map) throw new Error('需要 --map <name>（maps/<name>.json）');
  if (!PROVIDERS[a.provider]) throw new Error(`未知 provider: ${a.provider}（可选 ${Object.keys(PROVIDERS).join(' / ')}）`);
  if (!Number.isInteger(a.zoom) || a.zoom < 10 || a.zoom > 19) throw new Error('--zoom 需在 10..19');
  return a;
}

/** 地图产物里所有道路几何的**世界坐标**点（链式：points；路网：所有链） */
function roadWorldPoints(map: any): [number, number][] {
  if (Array.isArray(map.geometry?.points)) return map.geometry.points as [number, number][];
  const out: [number, number][] = [];
  for (const ch of map.network?.chains ?? []) for (const p of ch) out.push(p as [number, number]);
  return out;
}

async function fetchTile(url: string): Promise<Uint8Array> {
  const res = await fetch(url, {
    headers: { 'User-Agent': UA, Accept: 'image/png,image/*' },
    signal: AbortSignal.timeout(30000),
  });
  if (!res.ok) throw new Error(`HTTP ${res.status}`);
  const buf = new Uint8Array(await res.arrayBuffer());
  if (buf.length < 100) throw new Error(`响应过小（${buf.length} 字节）`);
  return buf;
}

async function main(): Promise<void> {
  const args = parseArgs(process.argv.slice(2));
  const mapPath = path.join('maps', `${args.map}.json`);
  if (!fs.existsSync(mapPath)) throw new Error(`找不到 ${mapPath}`);
  const map = JSON.parse(fs.readFileSync(mapPath, 'utf8'));
  const ow = map.geometry?.originWorld as [number, number] | undefined;
  if (!ow || Math.abs(ow[0]) < 1e-6) {
    throw new Error(`${mapPath} 缺少合法的 geometry.originWorld（世界坐标系的经纬度原点）——先重跑 mapgen`);
  }
  const origin: LatLon = { lat: ow[0], lon: ow[1] };

  // ── 1. 道路 bbox（世界米制 → 经纬度），外扩 buffer ──
  const pts = roadWorldPoints(map);
  if (pts.length === 0) throw new Error(`${mapPath} 里没有道路几何`);
  let north = -90, south = 90, west = 180, east = -180;
  for (const [x, y] of pts) {
    const ll = localMetersToLatLon(x, y, origin);
    north = Math.max(north, ll.lat); south = Math.min(south, ll.lat);
    west = Math.min(west, ll.lon); east = Math.max(east, ll.lon);
  }
  const dLat = args.bufferM / 111320;
  const dLon = args.bufferM / (111320 * Math.cos(origin.lat * Math.PI / 180));
  const bboxLatLon = {
    north: north + dLat, south: south - dLat, west: west - dLon, east: east + dLon,
  };

  // ── 2. 瓦片范围 ──
  const range = tileRangeForBbox(bboxLatLon, args.zoom);
  const res = tileResolutionMetersPerPx(args.zoom, (bboxLatLon.north + bboxLatLon.south) / 2);
  console.log(`底图取数：${args.map}  来源 ${args.provider}  z=${args.zoom}（${res.toFixed(2)} m/px）`);
  console.log(`  道路 bbox 外扩 ${args.bufferM}m → 瓦片 ${range.x0}..${range.x1} × ${range.y0}..${range.y1} = **${range.count} 张**`
    + `（约 ${(range.count * 9 / 1024).toFixed(1)}MB，按 9KB/张估）`);
  if (range.count > args.max) {
    throw new Error(`瓦片数 ${range.count} 超过 --max ${args.max}：降 zoom、减小 --buffer，或显式提高 --max`);
  }
  if (args.dryRun) { console.log('  --dry-run：不联网、不写文件'); return; }

  // ── 3. 逐张取（已缓存的复用：可重复执行、断点续传）──
  const prov = PROVIDERS[args.provider];
  const dir = path.join(args.outDir, args.provider, String(args.zoom));
  fs.mkdirSync(dir, { recursive: true });
  const tiles: { z: number; x: number; y: number; sha256: string; bytes: number }[] = [];
  let fetched = 0, reused = 0, failed = 0;
  for (let x = range.x0; x <= range.x1; x++) {
    for (let y = range.y0; y <= range.y1; y++) {
      const file = path.join(dir, String(x), `${y}.png`);
      let bytes: Uint8Array;
      if (fs.existsSync(file)) {
        bytes = new Uint8Array(fs.readFileSync(file));
        reused++;
      } else {
        const url = prov.template.replace('{z}', String(args.zoom)).replace('{x}', String(x)).replace('{y}', String(y));
        try {
          bytes = await fetchTile(url);
          fs.mkdirSync(path.dirname(file), { recursive: true });
          fs.writeFileSync(file, bytes);
          fetched++;
          if (fetched % 20 === 0) console.log(`  已取 ${fetched} 张…`);
          await new Promise(r => setTimeout(r, args.gapMs));       // 限速：体面地对待公共瓦片服务
        } catch (e) {
          failed++;
          console.warn(`  ⚠ ${args.zoom}/${x}/${y} 取数失败：${(e as Error).message}`);
          continue;
        }
      }
      tiles.push({
        z: args.zoom, x, y, bytes: bytes.length,
        sha256: createHash('sha256').update(bytes).digest('hex'),
      });
    }
  }

  // ── 4. 清单入库 ──
  const manifest = {
    version: 1,
    name: args.map,
    provider: args.provider,
    urlTemplate: prov.template,
    attribution: prov.attribution,
    note: prov.note,
    zoom: args.zoom,
    tileSizePx: TILE_SIZE_PX,
    bboxLatLon,
    /** 世界坐标 (0,0) 的经纬度：渲染层用它把瓦片摆进世界坐标 */
    worldOrigin: [ow[0], ow[1]],
    cache: `maps/tiles/${args.provider}/{z}/{x}/{y}.png`,
    tiles,
    fetchedAt: new Date().toISOString(),
    generator: 'npm run tiles',
  };
  const manifestPath = path.join(args.manifestDir, `${args.map}.tiles.json`);
  fs.writeFileSync(manifestPath, JSON.stringify(manifest, null, 1) + '\n');
  const total = tiles.reduce((a, t) => a + t.bytes, 0);
  console.log(`\n清单入库：${manifestPath}`);
  console.log(`  瓦片 ${tiles.length} 张（新取 ${fetched}、复用缓存 ${reused}、失败 ${failed}），共 ${(total / 1024).toFixed(1)}KB`);
  console.log(`  署名（渲染时必须显示）：${prov.attribution}`);
  if (failed > 0) {
    console.warn(`  ⚠ 有 ${failed} 张失败：缓存的瓦片不完整，渲染时缺失处会留空（不影响仿真）`);
    process.exitCode = 1;
  }
}

main().catch(e => { console.error(`失败：${(e as Error).message}`); process.exitCode = 1; });
