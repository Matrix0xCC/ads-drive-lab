// ==================== 共享地理换算（地图管线 + 渲染层共用） ====================
//
// 【为什么要有这个模块】地图管线（`js/mapgen/`）在构建期把经纬度投影成局部米制坐标；
// 渲染层要在**运行时**把底图瓦片放到同一个坐标系里。两边各写一份公式迟早会漂移，
// 而漂移的表现是"底图整体偏移几十米"这种很难一眼看出的错误。
// 所以：公式只有这一份，`projectToEnu` 与本模块的一致性由单测钉住（< 1mm）。
//
// 【两个必须记住的坑（都实测踩过）】
//   ① **Web Mercator 的"米"不是米**：瓦片在墨卡托平面里是正方形，而墨卡托在纬度 φ 处的
//      比例因子是 1/cos φ（北京 39.9° → 1.30）。同样的 z，赤道 2.39 m/px、北京 1.83 m/px。
//      把瓦片像素当米 → 差 30% 且随纬度漂移。
//   ② **瓦片在"等距近似"里仍是轴对齐矩形**（本模块的局部系是等距的：x=Δlon·kx, y=Δlat·ky），
//      所以只要把瓦片的**四个角**各自投影一次，就得到精确的摆放矩形——不必假设瓦片是正方形。
// 本模块**不得**依赖 js/mapgen（那是构建期工具，不进浏览器包）→ 常数在这里定义，
// 与 mapgen 的一致性由单测断言（`geo.test.ts` 比对 mapgen 导出的 EARTH_R_M）。
export const EARTH_R_M = 6371000;

export interface LatLon { lat: number; lon: number }

/** 度 → 米 的两个比例常数（与 `projectToEnu` 完全一致） */
export function scaleFactors(origin: LatLon): { kx: number; ky: number } {
  return {
    kx: Math.cos(origin.lat * Math.PI / 180) * EARTH_R_M * Math.PI / 180,
    ky: EARTH_R_M * Math.PI / 180,
  };
}

/** 经纬度 → 局部米制（x 东、y 北）——与 `js/mapgen/osm.ts:projectToEnu` 同一套式子 */
export function latLonToLocalMeters(p: LatLon, origin: LatLon): { x: number; y: number } {
  const { kx, ky } = scaleFactors(origin);
  return { x: (p.lon - origin.lon) * kx, y: (p.lat - origin.lat) * ky };
}

/** 局部米制 → 经纬度（上式的逆） */
export function localMetersToLatLon(x: number, y: number, origin: LatLon): LatLon {
  const { kx, ky } = scaleFactors(origin);
  return { lat: origin.lat + y / ky, lon: origin.lon + x / kx };
}

// ═══════════════════════════════════════════════════
// Web Mercator 瓦片（slippy map / XYZ 方案）
// ═══════════════════════════════════════════════════

/** 瓦片的纬度上界（y=0 是北极端；标准 XYZ 方案） */
function tileYToLat(y: number, z: number): number {
  const n = Math.PI - 2 * Math.PI * y / 2 ** z;
  return 180 / Math.PI * Math.atan(Math.sinh(n));
}

/** 瓦片 x → 经度（西边界） */
function tileXToLon(x: number, z: number): number {
  return x / 2 ** z * 360 - 180;
}

/** 经纬度 → 瓦片坐标（**含小数**；取整即瓦片编号） */
export function latLonToTile(lat: number, lon: number, z: number): { x: number; y: number } {
  const n = 2 ** z;
  const x = (lon + 180) / 360 * n;
  const latRad = lat * Math.PI / 180;
  const y = (1 - Math.log(Math.tan(latRad) + 1 / Math.cos(latRad)) / Math.PI) / 2 * n;
  return { x, y };
}

/** 瓦片编号 → 该瓦片覆盖的经纬度矩形（西/东/北/南） */
export function tileBounds(z: number, x: number, y: number): {
  west: number; east: number; north: number; south: number;
} {
  return {
    west: tileXToLon(x, z), east: tileXToLon(x + 1, z),
    north: tileYToLat(y, z), south: tileYToLat(y + 1, z),
  };
}

/**
 * 一个 z 层级上、某瓦片的**地面分辨率**（米/像素，真实距离）。
 *
 * ⚠ 这是"墨卡托像素对应的真实米数"，已经含 1/cos φ 的修正——**底图配准必须用它**，
 * 不能拿墨卡托平面距离当米（那正是本文开头坑 ①）。
 */
export function tileResolutionMetersPerPx(z: number, lat: number): number {
  return 156543.03392 * Math.cos(lat * Math.PI / 180) / 2 ** z;
}

/** 覆盖给定经纬度 bbox 的瓦片范围（含端点） */
export function tileRangeForBbox(
  bbox: { north: number; south: number; west: number; east: number }, z: number,
): { x0: number; y0: number; x1: number; y1: number; count: number } {
  const nw = latLonToTile(bbox.north, bbox.west, z);
  const se = latLonToTile(bbox.south, bbox.east, z);
  const x0 = Math.floor(nw.x), x1 = Math.floor(se.x);
  const y0 = Math.floor(nw.y), y1 = Math.floor(se.y);
  return { x0, y0, x1, y1, count: (x1 - x0 + 1) * (y1 - y0 + 1) };
}

/** 把瓦片矩形（经纬度）投影到局部米制 → 底图摆放用的轴对齐矩形（中心 + 宽高） */
export function tileWorldRect(
  z: number, x: number, y: number, origin: LatLon,
): { cx: number; cy: number; width: number; height: number } {
  const b = tileBounds(z, x, y);
  // 四角各自投影：局部系是等距的 → 瓦片仍是轴对齐矩形，用中心 + 宽高即可精确表示
  const nw = latLonToLocalMeters({ lat: b.north, lon: b.west }, origin);
  const se = latLonToLocalMeters({ lat: b.south, lon: b.east }, origin);
  return {
    cx: (nw.x + se.x) / 2, cy: (nw.y + se.y) / 2,
    width: Math.abs(se.x - nw.x), height: Math.abs(nw.y - se.y),
  };
}
