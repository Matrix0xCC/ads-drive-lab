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
  }
  html += '\n<b>Planning</b>\n';
  if (pl) {
    html += `  状态: ${pl.state} | S=${toM(pl.frenetS)}m L=${toM(pl.frenetL, 2)}m\n`;
    html += `  dl/ds=${pl.dl_ds.toFixed(4)} | Δheading=${(pl.headingDiff*180/Math.PI).toFixed(1)}°\n`;
    if (pl.remainingDist > 0) html += `  剩余: ${toM(pl.remainingDist)}m\n`;
    html += `  速度: 目标 ${toM(pl.speedTarget)}m/s 模式 ${pl.speedMode}\n`;
    html += `  紧间隙: ${pl.tight} | 迭代 ${pl.rounds} 轮 ${pl.planMs.toFixed(1)}ms (v假设 ${toM(pl.vAssume)}m/s)\n`;
    html += `  最优路径: L=${toM(pl.bestTargetL, 1)}m cost=${pl.bestCost.toFixed(1)}\n`;
    html += '  路径候选:\n';
    for (const p of pl.paths) {
      const mark = p.targetL === pl.bestTargetL ? '★' : ' ';
      html += `    ${mark} L=${toM(p.targetL, 1).padStart(5)}m cost=${p.cost.toFixed(1)}${p.blocked ? ' BLOCK' : ''}\n`;
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
