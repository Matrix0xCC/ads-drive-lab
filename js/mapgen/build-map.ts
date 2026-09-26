// ==================== 地图生成 CLI：OSM 高速主线 → maps/<name>.json ====================
//
// 【用法】
//   npm run mapgen -- --name bj-g4 --ref G4 --bbox 39.86,116.28,39.92,116.40 \
//                     --min-length 1500 --step 3 --toll-span
//   离线复跑（不联网，用已入库的原始响应）：
//   npm run mapgen -- --name bj-g4 --from-raw maps/raw/bj-g4.osm.json ...
//
// 【产出】（均入库，供"运行时零联网 + 同种子逐字节可复现"）
//   maps/raw/<name>.osm.json   原始 Overpass 响应（pretty-print，便于 code review）
//   maps/<name>.json           派生地图：开环中心线（米，局部 ENU）+ 来源 + 校验统计
//
// 【为什么原始数据也入库】① 仿真确定性的地基是"输入不变"；② 不反复压 Overpass（实测会过载
// 返回 Dispatcher_Client::timeout）；③ ODbL 要求来源可追溯。
//
// 【确定性】地图文件不含任何时间戳（时间戳会破坏"重新生成逐字节相同"）：
// 只记录原始响应的 sha256 与 OSM 数据基准时间（`osm3s.timestamp_osm_base`，是数据本身的时间）。
//
// 【Overpass 实测】必须带 User-Agent（不带直接 406）；主站会因过载失败 → 多端点 failover。

import * as fs from 'node:fs';
import * as path from 'node:path';
import { createHash } from 'node:crypto';
import process from 'node:process';
import {
  parseOverpass, chainWays, wayPoints, selectCarriageway, projectToEnu,
  resampleUniform, smoothByArcLength, curvatureStats, pathLengthM,
  projectOntoPath, sliceByArcLength, speedCapFromCurvature, lateralAccel, A_LAT_MAX_MPS2,
  type LatLon, type Vec2M,
} from './osm.js';
import { buildTopology, largestComponent, topologyStats, type Topology } from './network.js';
import { buildGraphFromEdges, buildNetworkRoute } from '../algo/routePlanner.js';

/**
 * Overpass 端点。**默认只用主站并重试，不在端点之间静默切换**——
 * 实测教训：主站过载时返回 HTML 错误页，failover 到镜像后拿回的是**另一份（更旧）OSM 数据**：
 * 同一个立交 bbox，主站有 33 条 way（16 主线 + 17 匝道），镜像只回 16 条主线且**没有 remark**，
 * 于是"这个 bbox 没有匝道"的错误结论被静默接受（路网因此没有路口）。
 * 需要换端点时用 `--endpoint <url>` 显式指定；`MIRRORS` 只用于提示。
 */
const PRIMARY_ENDPOINT = 'https://overpass-api.de/api/interpreter';
const MIRRORS = [
  'https://overpass.private.coffee/api/interpreter',
  'https://maps.mail.ru/osm/tools/overpass/api/interpreter',
];
/** 同端点重试次数与退避（毫秒） */
const RETRY_BACKOFF_MS = [2000, 6000, 15000];
const USER_AGENT = 'ads-drive-lab-mapgen/1.0 (offline driving simulator; OSM data)';
/** 客户端超时（> 查询里的服务端 timeout + 余量；见 fetchOverpass 的注释） */
const CLIENT_TIMEOUT_MS = 240000;
/** 收费站与所选链的横向容差（米）：超出即认为不在本车行道上 */
const TOLL_CORRIDOR_M = 40;
/** --toll-span 时在首末收费站之外额外留的余量（米） */
const TOLL_MARGIN_M = 60;
/** 世界边距（米）：点列归一化到 [margin, w-margin] 内，给路侧留出渲染/复位空间 */
const WORLD_MARGIN_M = 60;

interface Args {
  name: string; ref?: string; bbox?: [number, number, number, number];
  minLengthM: number; stepM: number; smoothM: number; lengthM?: number;
  tollSpan: boolean; startAtToll: boolean; reverse: boolean;
  /** 路网模式：产出多链 + 节点/边图（可路由），而非"选一条车行道" */
  network: boolean;
  /** Overpass 端点（缺省主站；换端点会换数据版本，见 fetchOverpass 注释） */
  endpoint: string;
  fromRaw?: string; writeRaw: boolean; outDir: string;
}

function parseArgs(argv: string[]): Args {
  const a: Args = { name: 'osm-map', minLengthM: 1500, stepM: 3, smoothM: 30, tollSpan: false,
    startAtToll: false, reverse: false, network: false, endpoint: PRIMARY_ENDPOINT,
    writeRaw: true, outDir: 'maps' };
  for (let i = 0; i < argv.length; i++) {
    const k = argv[i];
    if (k === '--name') a.name = argv[++i];
    else if (k === '--ref') a.ref = argv[++i];
    else if (k === '--bbox') {
      const v = argv[++i].split(',').map(Number);
      if (v.length !== 4 || v.some(n => !Number.isFinite(n))) throw new Error('--bbox 需要 S,W,N,E 四个数字');
      a.bbox = [v[0], v[1], v[2], v[3]];
    } else if (k === '--min-length') a.minLengthM = Number(argv[++i]);
    else if (k === '--step') a.stepM = Number(argv[++i]);
    else if (k === '--smooth') a.smoothM = Number(argv[++i]);
    else if (k === '--length') a.lengthM = Number(argv[++i]);
    else if (k === '--toll-span') a.tollSpan = true;
    else if (k === '--start-at-toll') a.startAtToll = true;
    else if (k === '--network') a.network = true;
    else if (k === '--endpoint') a.endpoint = argv[++i];
    else if (k === '--reverse') a.reverse = true;
    else if (k === '--from-raw') a.fromRaw = argv[++i];
    else if (k === '--no-raw') a.writeRaw = false;
    else if (k === '--out-dir') a.outDir = argv[++i];
    else throw new Error(`未知参数: ${k}`);
  }
  if (!a.fromRaw && !a.bbox) throw new Error('需要 --bbox（联网抓取）或 --from-raw（离线复跑）');
  if (/[\\/]/.test(a.name)) throw new Error('--name 不能含路径分隔符');
  if (!(a.stepM > 0)) throw new Error('--step 必须为正');
  if (a.smoothM < 0) throw new Error('--smooth 不能为负');
  if (a.tollSpan && a.startAtToll) throw new Error('--toll-span 与 --start-at-toll 互斥');
  return a;
}

function buildNetworkQuery(bbox: [number, number, number, number]): string {
  const [s, w, n, e] = bbox;
  // motorway（主线）+ motorway_link（匝道）：路网的"路口"正是两者相接处。
  // **用显式 union 而不是 `~"^(motorway|motorway_link)$"`**：实测该正则在 Overpass 上
  // 只匹配到 motorway（匝道 0 条，原始响应里可验证），显式等值查询才可靠。
  return `[out:json][timeout:180];(
way["highway"="motorway"](${s},${w},${n},${e});
way["highway"="motorway_link"](${s},${w},${n},${e});
node["barrier"="toll_booth"](${s},${w},${n},${e});
);out tags geom;`;
}

function buildQuery(bbox: [number, number, number, number]): string {
  const [s, w, n, e] = bbox;
  // 只按 bbox 抓，ref 过滤在本地做（本地 refMatches 是 `;` 精确匹配；Overpass 的 ~ 是子串正则，
  // 会把 G40 也当成 G4 抓进来）。收费站一并抓（M2 的收费口→出口要用）。
  return `[out:json][timeout:60];(
way["highway"="motorway"](${s},${w},${n},${e});
node["barrier"="toll_booth"](${s},${w},${n},${e});
way["barrier"="toll_booth"](${s},${w},${n},${e});
);out tags geom;`;
}

/** 抓取（同一端点重试，不静默换端点）；返回解析后的 JSON 与服务它的端点 */
async function fetchOverpass(query: string, endpoint: string): Promise<{ json: any; via: string }> {
  let lastErr = '';
  for (let attempt = 0; attempt <= RETRY_BACKOFF_MS.length; attempt++) {
    {
      const ep = endpoint;
      try {
        const url = `${ep}?data=${encodeURIComponent(query)}`;
        // 客户端超时必须**长于**查询里的服务端 timeout（180s）：否则"慢但会成功"的查询被客户端
        // 掐断 → failover 到别的端点，可能拿到**另一份（更旧/不同）**数据而毫无察觉。
        const res = await fetch(url, {
          headers: { 'User-Agent': USER_AGENT },
          signal: AbortSignal.timeout(CLIENT_TIMEOUT_MS),
        });
        // 显式看状态码：429 = 限流（实测主站在忙时对本机限流，返回 HTML 错误页；
        // 此时"重试"可能拿回**不同内容**的响应，必须当成失败并提示等待，而不是当成一次成功抓取）
        if (!res.ok) {
          lastErr = `${ep}: HTTP ${res.status}${res.status === 429 ? '（限流：稍后重试或改用 --from-raw 复用已入库原始数据）' : ''}`;
          if (attempt < RETRY_BACKOFF_MS.length) {
            const wait = RETRY_BACKOFF_MS[attempt];
            console.warn(`  抓取失败，${wait / 1000}s 后重试同一端点…（${lastErr}）`);
            await new Promise(r => setTimeout(r, wait));
          }
          continue;
        }
        const text = await res.text();
        if (!text.trimStart().startsWith('{')) {
          lastErr = `${ep}: 非 JSON 响应（${text.replace(/<[^>]+>/g, ' ').replace(/\s+/g, ' ').slice(0, 140)}）`;
          continue;
        }
        const json = JSON.parse(text);
        if (!Array.isArray(json.elements)) { lastErr = `${ep}: 响应缺少 elements`; continue; }
        // 【关键】Overpass 在超时/内存受限时返回的是"**部分结果** + remark"，HTTP 仍 200、JSON 也合法。
        // 只检查 elements 是数组会**静默接受截断数据**（实测：本应 33 条 way 的立交 bbox 只回了 16 条
        // motorway、匝道全丢 → 路网没有路口，看起来"这个 bbox 没匝道"）。必须显式失败并换端点。
        if (typeof json.remark === 'string' && json.remark.length > 0) {
          lastErr = `${ep}: Overpass 结果不完整（remark: ${json.remark.slice(0, 120)}）`;
          continue;
        }
        if (json.elements.length === 0) { lastErr = `${ep}: 响应为空`; continue; }
        return { json, via: ep };
      } catch (e) {
        lastErr = `${ep}: ${(e as Error).message}`;
      }
      if (attempt < RETRY_BACKOFF_MS.length) {
        const wait = RETRY_BACKOFF_MS[attempt];
        console.warn(`  抓取失败，${wait / 1000}s 后重试同一端点…（${lastErr}）`);
        await new Promise(r => setTimeout(r, wait));
      }
    }
  }
  throw new Error(`Overpass 抓取失败（端点 ${endpoint}，已重试 ${RETRY_BACKOFF_MS.length} 次）：${lastErr}`
    + `\n  提示：主站过载时可换端点，但**不同端点的 OSM 数据版本可能不同**（会静默改变地图内容）：`
    + `\n        --endpoint ${MIRRORS[0]}`);
}

/** 取众数（并列时取较小值）——用于 lanes / maxspeed 这类同一条路上可能不一致的 tag */
function mode(values: number[]): number | null {
  if (values.length === 0) return null;
  const c = new Map<number, number>();
  for (const v of values) c.set(v, (c.get(v) ?? 0) + 1);
  return [...c.entries()].sort((a, b) => b[1] - a[1] || a[0] - b[0])[0][0];
}

function leadingInt(s: string | undefined): number | null {
  if (!s) return null;
  const m = /^\s*(\d+)/.exec(s);
  return m ? Number(m[1]) : null;
}

/** 定点到 4 位小数（0.1mm）——保证重新生成逐字节相同 */
const r4 = (v: number): number => Math.round(v * 1e4) / 1e4;

interface TollGate { sM: number; lM: number; osmId: number; kind: 'node' | 'way' }

async function main(): Promise<void> {
  const args = parseArgs(process.argv.slice(2));
  fs.mkdirSync(args.outDir, { recursive: true });
  const rawPath = path.join(args.outDir, 'raw', `${args.name}.osm.json`);
  const metaPath = path.join(args.outDir, 'raw', `${args.name}.meta.json`);
  const mapPath = path.join(args.outDir, `${args.name}.json`);

  // ── 1. 取原始数据（联网 or 离线复用）──
  let json: any;
  let via: string;
  if (args.fromRaw) {
    json = JSON.parse(fs.readFileSync(args.fromRaw, 'utf8'));
    via = `file:${args.fromRaw}`;
    console.log(`离线复跑：${args.fromRaw}`);
  } else {
    const query = buildQuery(args.bbox!);
    console.log(`抓取 Overpass… bbox=${args.bbox!.join(',')}`);
    const got = await fetchOverpass(query, args.endpoint);
    json = got.json;
    via = got.via;
    console.log(`  端点 ${via}｜OSM 基准时间 ${json?.osm3s?.timestamp_osm_base ?? '?'}｜`
      + `元素 ${json.elements.length}（其中 way ${json.elements.filter((e: any) => e.type === 'way').length}）`);
  }
  // 规范化原始文本：入库的字节 = 计算 sha256 的字节（离线复跑才可能逐字节一致）
  const rawText = args.fromRaw ? fs.readFileSync(args.fromRaw, 'utf8') : JSON.stringify(json, null, 1) + '\n';
  if (!args.fromRaw && args.writeRaw) {
    fs.mkdirSync(path.dirname(rawPath), { recursive: true });
    fs.writeFileSync(rawPath, rawText);
    console.log(`原始数据入库：${rawPath}`);
  }
  const rawSha = createHash('sha256').update(rawText).digest('hex');
  // 取数过程信息（端点/时间/查询语句）只进侧车 meta，**不进地图文件**——
  // 否则"联网抓取"与"离线复跑"两条路径产出的地图文件不同，"重新生成逐字节相同"就不成立。
  if (!args.fromRaw) {
    fs.writeFileSync(metaPath, JSON.stringify({
      name: args.name, via, fetchedAt: new Date().toISOString(),
      bbox: args.bbox, query: buildQuery(args.bbox!), rawSha256: rawSha,
    }, null, 2) + '\n');
  }

  // ── 1b. 路网模式（多链 + 节点/边图）──
  if (args.network) {
    buildNetworkMap(args, json, via, rawSha, rawText, mapPath, worldH0 => worldH0);
    return;
  }

  // ── 2. 链合并 + 选车行道 ──
  const ways = parseOverpass(json);
  const chains = chainWays(ways);
  const onewayChains = chains.filter(c => c.every(w => w.tags.oneway === 'yes' || w.tags.oneway === '1'));
  console.log(`way ${ways.length} → 链 ${chains.length}（单向 ${onewayChains.length}）`);
  const picked = selectCarriageway(chains, { ref: args.ref, minLengthM: args.minLengthM });
  if (!picked) {
    console.log('候选链（按里程）：');
    for (const c of chains.map(c => {
      const len = pathLengthM(projectToEnu(wayPoints(c)));
      return { c, len, refs: [...new Set(c.map(w => w.tags.ref).filter(Boolean))].join('|') || '-' };
    }).sort((a, b) => b.len - a.len).slice(0, 8)) {
      console.log(`  ${c.len.toFixed(0).padStart(7)}m  ref=${c.refs}  way数=${c.c.length}  首id=${c.c[0].id}`);
    }
    throw new Error(`没有满足条件的车行道链（ref=${args.ref ?? '-'}，最短 ${args.minLengthM}m）`);
  }
  let chainPts = wayPoints(picked);
  if (args.reverse) chainPts = [...chainPts].reverse();
  const origin = chainPts[0];
  const proj = projectToEnu(chainPts);
  const rawLen = pathLengthM(proj);
  console.log(`选中链：${picked.length} way / ${rawLen.toFixed(0)}m / ref=${[...new Set(picked.map(w => w.tags.ref).filter(Boolean))].join('|')}`);

  // ── 3. 收费站桩号（节点 / way 均收，投影到同一 ENU 帧）──
  const gates: TollGate[] = [];
  for (const el of json.elements as any[]) {
    let pts: LatLon[] | null = null;
    let kind: TollGate['kind'] = 'node';
    if (el?.type === 'node' && el.tags?.barrier === 'toll_booth'
      && typeof el.lat === 'number' && typeof el.lon === 'number') {
      pts = [{ lat: el.lat, lon: el.lon }];
    } else if (el?.type === 'way' && el.tags?.barrier === 'toll_booth' && Array.isArray(el.geometry)) {
      pts = (el.geometry as LatLon[]).filter(p => typeof p?.lat === 'number' && typeof p?.lon === 'number');
      kind = 'way';
    }
    if (!pts || pts.length === 0) continue;
    const c: Vec2M = (() => {                       // way 取几何中点，node 取自身
      const enu = projectToEnu(pts, origin);
      return { x: enu.reduce((s, p) => s + p.x, 0) / enu.length, y: enu.reduce((s, p) => s + p.y, 0) / enu.length };
    })();
    const { s, l } = projectOntoPath(proj, c);
    if (Math.abs(l) <= TOLL_CORRIDOR_M) gates.push({ sM: r4(s), lM: r4(l), osmId: el.id, kind });
  }
  gates.sort((a, b) => a.sM - b.sM);
  console.log(`本车行道上的收费站：${gates.length} 个${gates.length ? `（首个 s=${gates[0].sM}m，末个 s=${gates[gates.length - 1].sM}m）` : ''}`);

  // ── 4. 裁剪（收费站跨度 / 指定长度）→ 平滑 → 重采样 ──
  let sA = 0, sB = rawLen;
  let worldW = 0, worldH = 0;
  if (args.tollSpan) {
    if (gates.length >= 2) {
      sA = Math.max(0, gates[0].sM - TOLL_MARGIN_M);
      sB = Math.min(rawLen, gates[gates.length - 1].sM + TOLL_MARGIN_M);
      console.log(`按收费站裁剪：s ∈ [${sA.toFixed(0)}, ${sB.toFixed(0)}]m（含 ${TOLL_MARGIN_M}m 余量）`);
    } else {
      console.warn('  ⚠ --toll-span 需要 ≥2 个收费站，跳过裁剪');
    }
  }
  if (args.startAtToll) {
    if (gates.length >= 1) {
      sA = Math.max(0, gates[0].sM - TOLL_MARGIN_M);
      console.log(`起点对齐收费口：s0 = ${sA.toFixed(0)}m（首个收费站 s=${gates[0].sM.toFixed(0)}m，余量 ${TOLL_MARGIN_M}m）`);
    } else {
      console.warn('  ⚠ --start-at-toll 需要 ≥1 个收费站，跳过对齐');
    }
  }
  if (args.lengthM !== undefined) {
    const b = Math.min(sB, sA + args.lengthM);
    if (b < sB) console.log(`按 --length 裁剪：s ∈ [${sA.toFixed(0)}, ${b.toFixed(0)}]m`);
    sB = b;
  }
  const control = sliceByArcLength(proj, sA, sB);
  // 归一化到**世界坐标**（点列平移使包围盒落在 [margin, ...] 内）：
  // 运行时不需再做 bbox 计算，地图文件本身就是"世界坐标 + 世界尺寸"。
  {
    let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
    for (const p of control) {
      minX = Math.min(minX, p.x); maxX = Math.max(maxX, p.x);
      minY = Math.min(minY, p.y); maxY = Math.max(maxY, p.y);
    }
    const dx = WORLD_MARGIN_M - minX, dy = WORLD_MARGIN_M - minY;
    for (const p of control) { p.x += dx; p.y += dy; }
    for (const p of proj) { p.x += dx; p.y += dy; }          // 收费站已按 s 参数记录，平移不影响 s
    worldW = maxX - minX + 2 * WORLD_MARGIN_M;
    worldH = maxY - minY + 2 * WORLD_MARGIN_M;
  }
  // 线形净化三步（依据见 spec 2026-09-26 §3.3）：
  //   ① 线性等距重采样（把非均匀 OSM 节点变成 3m 网格；不引入过冲）
  //   ② 按弧长窗口平滑（治单节点位置抖动造成的假急弯）
  //   ③ 再等距重采样（平滑会让总长略缩，恢复严格等距）
  const rough = resampleUniform(control, args.stepM);
  const road = args.smoothM > 0
    ? resampleUniform(smoothByArcLength(rough, args.smoothM), args.stepM)
    : rough;

  // ── 5. 校验统计 ──
  const lenM = pathLengthM(road);
  const kappa = curvatureStats(road, 5);
  const minRadiusM = kappa.max > 1e-9 ? 1 / kappa.max : Infinity;
  const laneCandidates = picked.map(w => leadingInt(w.tags.lanes)).filter((v): v is number => v !== null && v > 0);
  const laneCount = mode(laneCandidates) ?? 3;
  const laneWidthM = 3.75;
  const maxspeedKph = mode(picked.map(w => leadingInt(w.tags.maxspeed))
    .filter((v): v is number => v !== null && v > 0)) ?? 120;
  const maxspeedMps = maxspeedKph / 3.6;
  const warnings: string[] = [];
  if (kappa.p95 > 0.002) warnings.push(`线形偏弯：κ p95=${kappa.p95.toExponential(2)}（R≈${(1 / kappa.p95).toFixed(0)}m）`);
  if (minRadiusM < 150) warnings.push(`存在急弯：最小半径 ≈ ${minRadiusM.toFixed(0)}m（检查是否原始数据假象）`);
  if (lateralAccel(kappa.max, maxspeedMps) > A_LAT_MAX_MPS2) {
    warnings.push(`按 maxspeed ${maxspeedKph}km/h 过最急弯需 ${lateralAccel(kappa.max, maxspeedMps).toFixed(1)}m/s² > A_LAT_MAX ${A_LAT_MAX_MPS2}`
      + `（本项目车辆为运动学模型、无附着极限，该约束只作用在规划器 → 需靠限速通过，可行上限 ${(speedCapFromCurvature(kappa.max) * 3.6).toFixed(0)}km/h）`);
  }
  if (new Set(laneCandidates).size > 1) warnings.push(`lanes tag 不一致（${[...new Set(laneCandidates)].join('/')}），取众数 ${laneCount}`);
  if (args.tollSpan && gates.length >= 2 && lenM < args.minLengthM) {
    warnings.push(`裁剪后仅 ${lenM.toFixed(0)}m < min-length ${args.minLengthM}m（收费站太近）`);
  }

  // ── 6. 写地图文件（键序固定 + 定点 4 位 → 逐字节可复现）──
  const out = {
    version: 1,
    name: args.name,
    kind: 'osm-chain',
    closed: false,
    source: {
      provider: 'OpenStreetMap', license: 'ODbL-1.0',
      attribution: '© OpenStreetMap contributors',
      rawFileName: `${args.name}.osm.json`,
      ref: args.ref ?? null,
      wayIds: picked.map(w => w.id),
      rawSha256: rawSha,
      osmBaseTimestamp: json?.osm3s?.timestamp_osm_base ?? null,
    },
    world: { wM: r4(worldW), hM: r4(worldH), marginM: WORLD_MARGIN_M },
    geometry: {
      stepM: r4(args.stepM),
      origin: [r4(origin.lat), r4(origin.lon)],
      points: road.map(p => [r4(p.x), r4(p.y)]),
    },
    road: {
      laneCount, laneWidthM, halfWidthM: r4(laneCount * laneWidthM / 2),
      maxspeedKph, oneway: true,
    },
    tollgates: gates
      .filter(g => g.sM >= sA && g.sM <= sB)
      .map(g => ({ sM: r4(g.sM - sA), lM: g.lM, osmId: g.osmId, kind: g.kind })),
    validation: {
      lengthM: r4(lenM),
      kappa: { p50: kappa.p50, p95: kappa.p95, max: kappa.max },
      minRadiusM: Number.isFinite(minRadiusM) ? r4(minRadiusM) : null,
      aLatAtMaxspeed: r4(lateralAccel(kappa.max, maxspeedMps)),
      maxspeedFeasibleMps: Number.isFinite(speedCapFromCurvature(kappa.max)) ? r4(speedCapFromCurvature(kappa.max)) : null,
      warnings,
    },
  };
  fs.writeFileSync(mapPath, JSON.stringify(out, null, 2) + '\n');

  // 生成 TS 模块：浏览器与 headless 都以**同步 import** 的方式拿到地图（避免运行时 fetch 与异步装配）。
  // 数据源仍是 maps/<name>.json（人可读、可 diff、有 sha256）；本文件是它的机械转换，勿手改。
  const genDir = path.join('js', 'env', 'maps');
  fs.mkdirSync(genDir, { recursive: true });
  const constName = args.name.toUpperCase().replace(/[^A-Z0-9]+/g, '_');
  const pts = road.map(p => `    { x: m(${r4(p.x)}), y: m(${r4(p.y)}) },`).join('\n');
  const gen = `// ==================== 自动生成：OSM 地图 → MapDefinition（勿手改） ====================
// 由 \`npm run mapgen\` 从 maps/${args.name}.json 生成；改地图请改生成器参数并重跑。
// 源文件 sha256: ${rawSha}
// 数据来源：${out.source.provider}（${out.source.license}）${out.source.attribution}
import { m } from '../../shared/dimensions.js';
import type { MapDefinition } from '../mapDefinitions.js';

export const ${constName}: MapDefinition = {
  name: ${JSON.stringify(args.name)},
  kind: 'polyline',
  closed: ${out.closed},
  halfWidth: m(${out.road.halfWidthM}),
  worldW: m(${r4(worldW)}),
  worldH: m(${r4(worldH)}),
  laneWidthM: ${out.road.laneWidthM},
  maxspeedKph: ${out.road.maxspeedKph},
  tollgates: [
${out.tollgates.map(g => `    { sM: ${g.sM}, lM: ${g.lM}, osmId: ${g.osmId} },`).join('\n')}
  ],
  points: [
${pts}
  ],
};
`;
  const genPath = path.join(genDir, `${args.name}.gen.ts`);
  fs.writeFileSync(genPath, gen);
  console.log(`生成模块：${genPath}（${road.length} 点）`);

  console.log(`\n写出 ${mapPath}`);
  console.log(`  长度 ${lenM.toFixed(0)}m 步长 ${args.stepM}m 点数 ${road.length} 世界 ${worldW.toFixed(0)}×${worldH.toFixed(0)}m`);
  if (Math.max(worldW, worldH) > 800) console.log(`  ℹ 世界长边 ${Math.max(worldW, worldH).toFixed(0)}m > 屏幕可读范围：相机需跟随（M1-b）`);
  console.log(`  车道 ${laneCount}×${laneWidthM}m → 半宽 ${(laneCount * laneWidthM / 2).toFixed(2)}m；maxspeed ${maxspeedKph}km/h`);
  console.log(`  κ p50/p95/max = ${kappa.p50.toExponential(2)} / ${kappa.p95.toExponential(2)} / ${kappa.max.toExponential(2)}（R_max=${(1 / kappa.max).toFixed(0)}m）`);
  console.log(`  收费站 ${out.tollgates.length} 个${out.tollgates.length ? `：s=${out.tollgates.map(g => g.sM.toFixed(0)).join(', ')}m` : ''}`);
  for (const w of warnings) console.log(`  ⚠ ${w}`);
}

/**
 * 路网地图：way → 拓扑（节点/边）→ 最大连通分量 → 逐边几何净化 → 世界坐标 → 地图 + TS 模块。
 *
 * 【与链模式的差别】链模式只选"一条车行道"，路由退化成沿链走；路网模式保留**整片路网**，
 * 于是 Dijkstra 第一次真的有得选（直行 vs 走匝道）。地图里同时带 nodes / edges / chains，
 * 运行时按 `kind: 'network'` 走多链分支。
 */
function buildNetworkMap(
  args: Args, json: any, via: string, rawSha: string, rawText: string,
  mapPath: string, _pass: (x: number) => number,
): void {
  const all = parseOverpass(json).filter(w => /^(motorway|motorway_link)$/.test(w.tags.highway ?? ''));
  if (all.length === 0) throw new Error('bbox 内没有 motorway / motorway_link');
  const rawTopo = buildTopology(all);
  const comp = largestComponent(rawTopo);
  const st0 = topologyStats(comp);
  console.log(`路网：way ${all.length} → 节点 ${st0.nodes} / 边 ${st0.edges}`
    + `（路口 ${st0.junctions}、死端 ${st0.deadEnds}）/ 里程 ${(st0.lengthM / 1000).toFixed(2)}km`
    + `  ${Object.entries(st0.byHighway).map(([k, v]) => `${k}:${(v.lengthM / 1000).toFixed(1)}km`).join(' ')}`);
  if (st0.junctions === 0) console.warn('  ⚠ 该 bbox 内没有路口（度≥3）——路由不会有"选择"，换一个含立交的 bbox');
  if (!st0.byHighway['motorway_link']) {
    console.warn('  ⚠ 结果里没有任何 motorway_link：可能是**截断响应**（Overpass 超时返回部分结果）'
      + ' 或该 bbox 确实没有匝道 —— 请用 --from-raw 复核原始响应');
  }

  // 逐边几何净化（与链模式同一套三步）+ 世界坐标偏移
  let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
  const chains: Vec2M[][] = comp.edges.map(e => {
    const rough = resampleUniform(e.geom, args.stepM);
    const road = args.smoothM > 0 ? resampleUniform(smoothByArcLength(rough, args.smoothM), args.stepM) : rough;
    for (const p of road) {
      minX = Math.min(minX, p.x); maxX = Math.max(maxX, p.x);
      minY = Math.min(minY, p.y); maxY = Math.max(maxY, p.y);
    }
    return road;
  });
  const dx = WORLD_MARGIN_M - minX, dy = WORLD_MARGIN_M - minY;
  for (const ch of chains) for (const p of ch) { p.x += dx; p.y += dy; }
  const nodes = comp.nodes.map(n => ({ x: n.x + dx, y: n.y + dy }));
  const worldW = maxX - minX + 2 * WORLD_MARGIN_M;
  const worldH = maxY - minY + 2 * WORLD_MARGIN_M;

  const edges = comp.edges.map((e, i) => ({
    from: e.from, to: e.to, chain: i,
    highway: e.highway, ref: e.ref, oneway: e.oneway,
    lengthM: r4(pathLengthM(chains[i])),
  }));

  // 道路属性：主线车道数取众数、限速取最小（保守）
  const mw = all.filter(w => w.tags.highway === 'motorway');
  const laneCount = mode(mw.map(w => leadingInt(w.tags.lanes)).filter((v): v is number => v !== null && v > 0)) ?? 3;
  const laneWidthM = 3.75;
  const maxspeedKph = mode(mw.map(w => leadingInt(w.tags.maxspeed)).filter((v): v is number => v !== null && v > 0)) ?? 120;

  // 路线抽样（验收证据）：取两个相距最远的死端/端点，跑真 Dijkstra，看是否经过匝道
  const deg = new Array(nodes.length).fill(0);
  for (const e of comp.edges) { deg[e.from]++; deg[e.to]++; }
  const ends = nodes.map((_, i) => i).filter(i => deg[i] === 1);
  let routeInfo: Record<string, unknown> = { available: false };
  if (ends.length >= 2) {
    const graph = buildGraphFromEdges(nodes.length, edges);
    const chPts = chains.map(ch => ch.map(p => ({ x: p.x as any, y: p.y as any })));
    // 单向路网：不是任意两个死端都可达（可能逆着单行方向）→ 按"距离从远到近"试若干对，
    // 取第一条可路由的（确定性：候选按确定顺序生成）
    const cands: [number, number, number][] = [];
    for (const i of ends) for (const j of ends) {
      if (i >= j) continue;
      cands.push([i, j, Math.hypot(nodes[i].x - nodes[j].x, nodes[i].y - nodes[j].y)]);
    }
    cands.sort((x, y) => y[2] - x[2]);
    let r: ReturnType<typeof buildNetworkRoute> = null;
    let a0 = -1, b0 = -1;
    for (const [i, j] of cands) {
      // 单向路网：正向不通就试反向；**记录真正走通的那个方向**（否则复算不可达）
      r = buildNetworkRoute(graph, edges, chPts, i, j);
      if (r) { a0 = i; b0 = j; break; }
      r = buildNetworkRoute(graph, edges, chPts, j, i);
      if (r) { a0 = j; b0 = i; break; }
    }
    if (r) {
      const used = r.chainSeq.map(ci => edges[ci].highway);
      const links = used.filter(h => h === 'motorway_link').length;
      routeInfo = {
        available: true, fromNode: a0, toNode: b0, edges: r.chainSeq.length,
        linksUsed: links, lengthM: r4(pathLengthM(r.waypoints as Vec2M[])),
        usesRamp: links > 0,
      };
      console.log(`路线抽样：节点 ${a0}→${b0}｜${r.chainSeq.length} 条边 / ${(pathLengthM(r.waypoints as Vec2M[]) / 1000).toFixed(2)}km`
        + `｜匝道 ${links} 段${links > 0 ? '（✓ 路线经过匝道）' : '（⚠ 未经过匝道）'}`);
    }
  }

  const out = {
    version: 1, name: args.name, kind: 'network', closed: false,
    source: {
      provider: 'OpenStreetMap', license: 'ODbL-1.0',
      attribution: '© OpenStreetMap contributors',
      rawFileName: `${args.name}.osm.json`, ref: args.ref ?? null,
      wayIds: all.map(w => w.id), rawSha256: rawSha,
      osmBaseTimestamp: json?.osm3s?.timestamp_osm_base ?? null,
    },
    world: { wM: r4(worldW), hM: r4(worldH), marginM: WORLD_MARGIN_M },
    geometry: { stepM: r4(args.stepM), origin: [r4(comp.nodes[0]?.x ?? 0), r4(comp.nodes[0]?.y ?? 0)] },
    road: { laneCount, laneWidthM, halfWidthM: r4(laneCount * laneWidthM / 2), maxspeedKph, oneway: true },
    network: {
      nodes: nodes.map(n => [r4(n.x), r4(n.y)]),
      edges,
      chains: chains.map(ch => ch.map(p => [r4(p.x), r4(p.y)])),
    },
    validation: {
      lengthM: r4(st0.lengthM), junctions: st0.junctions, deadEnds: st0.deadEnds,
      byHighway: Object.fromEntries(Object.entries(st0.byHighway)
        .map(([k, v]) => [k, { count: v.count, lengthM: r4(v.lengthM) }])),
      routeSample: routeInfo,
      /** 边数 ≥ 节点数 ⇔ 图里有环 ⇔ 存在**替代路径**（真正的"选路"） */
      hasCycle: comp.edges.length >= nodes.length,
      warnings: [
        ...(st0.junctions === 0 ? ['bbox 内无路口：路由不会有选择'] : []),
        ...(comp.edges.length < nodes.length ? ['图中无环（树）：路线唯一，"选路"体现在走哪条匝道而非替代路径'] : []),
      ],
    },
  };
  fs.writeFileSync(mapPath, JSON.stringify(out, null, 2) + '\n');

  // TS 模块（机械转换；勿手改）
  const constName = args.name.toUpperCase().replace(/[^A-Z0-9]+/g, '_');
  const genDir = path.join('js', 'env', 'maps');
  fs.mkdirSync(genDir, { recursive: true });
  const gen = `// ==================== 自动生成：OSM 路网 → MapDefinition（勿手改） ====================
// 由 \`npm run mapgen -- --network\` 从 maps/${args.name}.json 生成。
// 源文件 sha256: ${rawSha}
// 数据来源：OpenStreetMap（ODbL-1.0）© OpenStreetMap contributors
import { m } from '../../shared/dimensions.js';
import type { MapDefinition } from '../mapDefinitions.js';

export const ${constName}: MapDefinition = {
  name: ${JSON.stringify(args.name)},
  kind: 'network',
  closed: false,
  halfWidth: m(${out.road.halfWidthM}),
  worldW: m(${r4(worldW)}),
  worldH: m(${r4(worldH)}),
  laneWidthM: ${laneWidthM},
  maxspeedKph: ${maxspeedKph},
  nodes: [
${nodes.map(n => `    { x: m(${r4(n.x)}), y: m(${r4(n.y)}) },`).join('\n')}
  ],
  chains: [
${chains.map(ch => `    [${ch.map(p => `{ x: m(${r4(p.x)}), y: m(${r4(p.y)}) }`).join(', ')}],`).join('\n')}
  ],
  edges: [
${edges.map(e => `    { from: ${e.from}, to: ${e.to}, chain: ${e.chain}, lengthM: ${e.lengthM}, oneway: ${e.oneway} },`).join('\n')}
  ],
};
`;
  const genPath = path.join(genDir, `${args.name}.gen.ts`);
  fs.writeFileSync(genPath, gen);
  // meta 无条件写（联网抓取与离线复跑都留溯源；取数过程信息不进地图文件，见 M1 设计发现 2）
  fs.writeFileSync(path.join(args.outDir, 'raw', `${args.name}.meta.json`), JSON.stringify({
    name: args.name, mode: 'network', via,
    fetchedAt: args.fromRaw ? null : new Date().toISOString(),
    bbox: args.bbox ?? null, query: args.bbox ? buildNetworkQuery(args.bbox) : '(离线复跑)',
    rawSource: args.fromRaw ?? null, rawSha256: rawSha,
  }, null, 2) + '\n');
  console.log(`\n写出 ${mapPath}`);
  console.log(`  链 ${chains.length} 段 / 节点 ${nodes.length} 个 / 世界 ${worldW.toFixed(0)}×${worldH.toFixed(0)}m`);
  console.log(`  车道 ${laneCount}×${laneWidthM}m → 半宽 ${(laneCount * laneWidthM / 2).toFixed(2)}m；maxspeed ${maxspeedKph}km/h`);
  console.log(`  生成模块：${genPath}`);
}

main().catch(e => { console.error(`失败：${(e as Error).message}`); process.exitCode = 1; });
