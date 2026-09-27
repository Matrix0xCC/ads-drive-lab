// ==================== UI：调试面板（Inspect，画布右下） ====================
// 渲染（节流 10Hz）：感知/规划/控制模块的内部状态，写入 #inspect-body。
// 展开/收起由状态 debugOpen 控制（btn-debug-toggle / I 键）。

import { debug } from '../shared/debug.js';
import { SIM_UNITS_PER_METER } from '../shared/dimensions.js';

let _inspectTimer = 0;

export function updateInspectPanel(dt: number) {
  _inspectTimer += dt;
  if (_inspectTimer < 0.1) return;
  _inspectTimer = 0;

  const body = document.getElementById('inspect-body');
  if (!body) return;

  const toM = (v: number, d = 1) => (v / SIM_UNITS_PER_METER).toFixed(d);

  const p = debug.perception;
  const pl = debug.planning;
  const c = debug.control;

  let html = '<b>Perception</b>\n';
  if (p) {
    html += `  射线: ${p.rayCount} | 前方: ${toM(p.frontDist)}m\n`;
    html += `  最近命中: ${toM(p.minDist)}m\n`;
    html += `  过滤: ${p.rawHits}→${p.filteredHits} (背景减除)\n`;
    // 【S3】物体层：聚类 + 跟踪得到的物体（字段已是 m / m·s⁻¹）。列出最近的三个已确认物体，
    // 便于在浏览器里直接核对"旁边那辆车被跟成了什么速度"。
    if (p.objects && p.objects.length) {
      const conf = p.objects.filter(o => o.confirmed);
      html += `  物体: ${conf.length}/${p.objects.length} 已确认（聚类+跟踪）\n`;
      for (const o of conf.slice(0, 3)) {
        html += `    #${o.id} s=${o.s.toFixed(0)}m l=${o.l.toFixed(1)}m v=${o.vS.toFixed(1)}m/s 射线${o.rays} 传感器${o.sensor}\n`;
      }
    }
  }
  html += pl?.source === 'replay' ? '\n<b>Planning（回放·归档+重算）</b>\n' : '\n<b>Planning</b>\n';
  if (pl) {
    html += `  状态: ${pl.state} | S=${toM(pl.frenetS)}m L=${toM(pl.frenetL, 2)}m\n`;
    if (pl.cruiseV) html += `  巡航目标: ${toM(pl.cruiseV)}m/s（地图限速 ∩ 本车道前方最近命中）\n`;
    html += `  dl/ds=${pl.dl_ds.toFixed(4)} | Δheading=${(pl.headingDiff*180/Math.PI).toFixed(1)}°\n`;
    if (pl.remainingDist > 0) html += `  剩余: ${toM(pl.remainingDist)}m\n`;
    html += `  速度: 目标 ${toM(pl.speedTarget)}m/s 模式 ${pl.speedMode}\n`;
    if (pl.mapSpeedLimitMps != null) html += `  地图限速: ${toM(pl.mapSpeedLimitMps)}m/s（maxspeed）\n`;
    if (pl.laneOffsetU != null) {
      const cs = (pl.laneCentersU ?? []).map(v => toM(v, 2)).join(' / ');
      html += `  车道: 名义 ${toM(pl.laneOffsetU, 2)}m${cs ? `｜中心 [${cs}]m` : ''}\n`;
    }
    if (pl.rounds !== undefined && pl.planMs !== undefined && pl.vAssume !== undefined) {
      html += `  紧间隙: ${pl.tight ?? '—'} | 迭代 ${pl.rounds} 轮 ${pl.planMs.toFixed(1)}ms (v假设 ${toM(pl.vAssume)}m/s)\n`;
    }
    html += `  最优路径: L=${toM(pl.bestTargetL, 1)}m cost=${pl.bestCost.toFixed(1)}\n`;
    html += '  路径候选（编号同画布 L 标签）:\n';
    for (let i = 0; i < pl.paths.length; i++) {
      const p = pl.paths[i];
      const mark = p.targetL === pl.bestTargetL ? '★' : ' ';
      html += `    #${String(i + 1).padStart(2)}${mark} L=${toM(p.targetL, 1).padStart(5)}m cost=${p.cost.toFixed(1)}${p.blocked ? ' BLOCK' : ''}\n`;
    }
  }
  html += '\n<b>Control</b>\n';
  if (c) {
    html += `  预瞄: ${toM(c.lookaheadDist)}m → (${toM(c.lookaheadX)},${toM(c.lookaheadY)})\n`;
    html += `  α=${(c.alpha*180/Math.PI).toFixed(1)}° | δ=${(c.steerAngle*180/Math.PI).toFixed(2)}° | 油门=${c.throttle.toFixed(3)}\n`;
  }
  body.innerHTML = html;
}

/** 折叠/展开调试面板（控制内容区显隐与按钮箭头） */
export function setInspectorOpen(open: boolean): void {
  const body = document.getElementById('inspect-body');
  const btn = document.getElementById('btn-debug-toggle');
  if (body) body.style.display = open ? 'block' : 'none';
  if (btn) btn.textContent = open ? '▾' : '▸';
}
