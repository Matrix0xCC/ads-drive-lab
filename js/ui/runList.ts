// ==================== UI：回放运行选择框（分页列表） ====================
// 「数据 → 回放运行…」打开应用内对话框：列出 runs/index.json 的全部运行
// （时间/场景/地图/判定，每页 10 条），点条目进入回放；Esc 或 × 关闭。
// 数据来自服务器端文件（天然支持 WSL 目录），不使用浏览器文件选择器。
// 开关状态在 UiState.runPickerOpen；当前页与列表数据是本模块的视图状态。

import type { UiController } from './controller.js';

/** runs/index.json 条目（与 ReplayController.listRuns 的返回结构一致） */
export interface RunEntry {
  id: string;
  scenario: string;
  map: string;
  seed: number;
  createdAt: string;
  verdict: string;
  high: number;
  medium: number;
}

const PAGE_SIZE = 10;

/** 页数（空列表也算 1 页，用于空态展示） */
export function pageCount(total: number, size = PAGE_SIZE): number {
  return Math.max(1, Math.ceil(total / size));
}

/** 夹紧页码后切片（负数→首页，超界→末页） */
export function pageSlice<T>(items: T[], page: number, size = PAGE_SIZE): T[] {
  const p = Math.max(0, Math.min(page, pageCount(items.length, size) - 1));
  return items.slice(p * size, p * size + size);
}

/** ISO 时间 → 本地 "MM-DD HH:mm"（目录名按本地时间生成，展示保持一致） */
export function formatRunTime(iso: string): string {
  const d = new Date(iso);
  if (Number.isNaN(d.getTime())) return iso;
  const p = (n: number) => String(n).padStart(2, '0');
  return `${p(d.getMonth() + 1)}-${p(d.getDate())} ${p(d.getHours())}:${p(d.getMinutes())}`;
}

let _runs: RunEntry[] = [];
let _page = 0;
let _onPick: ((id: string) => void) | null = null;
/** 清理按钮两次点击确认（第一次进入待确认态，3s 未确认自动复位） */
let _clearArmed = false;
let _clearBusy = false;
let _message = '';

function escapeHtml(s: string): string {
  return s.replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]!));
}

/** 装入运行列表（最新在前）并回到第一页；打开前调用 */
export function setRunList(runs: RunEntry[]): void {
  _runs = [...runs].reverse();   // index.json 按时间追加 → 反转使最新在前
  _page = 0;
  _clearArmed = false;
  _message = '';
}

/** 接线：关闭按钮 / 分页按钮 / 行点击（事件委托） */
export function wireRunPicker(ctl: UiController, onPick: (id: string) => void): void {
  _onPick = onPick;
  document.getElementById('btn-run-picker-close')?.addEventListener('click', () => {
    ctl.dispatch({ type: 'CLOSE_RUN_PICKER' });
  });
  document.getElementById('btn-run-prev')?.addEventListener('click', () => {
    _page -= 1; _render();
  });
  document.getElementById('btn-run-next')?.addEventListener('click', () => {
    _page += 1; _render();
  });
  document.getElementById('run-picker-body')?.addEventListener('click', (e) => {
    const row = (e.target as HTMLElement).closest('[data-run-id]') as HTMLElement | null;
    if (!row?.dataset.runId) return;
    const id = row.dataset.runId;
    ctl.dispatch({ type: 'CLOSE_RUN_PICKER' });
    _onPick?.(id);
  });
  document.getElementById('btn-run-clear')?.addEventListener('click', () => { void onClearClick(); });
}

/** 清理归档：两次点击确认 → DELETE /api/runs（需 npm run serve 启动） */
async function onClearClick(): Promise<void> {
  if (_clearBusy || _runs.length === 0) return;
  if (!_clearArmed) {
    _clearArmed = true;
    _render();
    setTimeout(() => { if (_clearArmed) { _clearArmed = false; _message = ''; _render(); } }, 3000);
    return;
  }
  _clearBusy = true;
  _render();
  try {
    const res = await fetch('/api/runs', { method: 'DELETE' });
    const body: any = await res.json().catch(() => null);
    if (!res.ok || !body?.ok) throw new Error(body?.error ?? `HTTP ${res.status}`);
    _runs = [];
    _message = `已清理 ${body.deleted} 项归档`;
  } catch (e) {
    _message = `清理失败：${(e as Error).message}（需用 npm run serve 启动才有清理接口）`;
  } finally {
    _clearBusy = false;
    _clearArmed = false;
    _render();
  }
}

/** 状态投影：显示/隐藏对话框；打开时渲染当前页 */
export function setRunPickerOpen(open: boolean): void {
  const el = document.getElementById('run-picker');
  if (el) el.style.display = open ? 'flex' : 'none';
  if (open) _render();
}

function _render(): void {
  const body = document.getElementById('run-picker-body');
  if (!body) return;
  const pages = pageCount(_runs.length);
  _page = Math.max(0, Math.min(_page, pages - 1));

  body.innerHTML = _runs.length === 0
    ? '<div class="rp-empty">没有可回放的运行。<br>先执行 headless 生成归档：<br>'
      + 'npm run sim -- --scenario scenarios/batch/oval-s1.json</div>'
    : pageSlice(_runs, _page).map(r => `
        <div class="rp-row" data-run-id="${escapeHtml(r.id)}">
          <span class="rp-time">${escapeHtml(formatRunTime(r.createdAt))}</span>
          <span class="rp-name">${escapeHtml(r.scenario)}</span>
          <span class="rp-map">${escapeHtml(r.map)} #${r.seed}</span>
          <span class="rp-verdict ${r.verdict === 'PASS' ? 'ok' : 'warn'}">${escapeHtml(r.verdict)}</span>
        </div>`).join('');

  const label = document.getElementById('run-picker-page');
  if (label) label.textContent = `${_page + 1} / ${pages}（共 ${_runs.length} 条）`;
  const prev = document.getElementById('btn-run-prev') as HTMLButtonElement | null;
  const next = document.getElementById('btn-run-next') as HTMLButtonElement | null;
  if (prev) prev.disabled = _page <= 0;
  if (next) next.disabled = _page >= pages - 1;

  const clearBtn = document.getElementById('btn-run-clear') as HTMLButtonElement | null;
  if (clearBtn) {
    clearBtn.textContent = _clearArmed ? '确认清理？（不可恢复）' : (_clearBusy ? '清理中…' : '清理归档');
    clearBtn.classList.toggle('armed', _clearArmed);
    clearBtn.disabled = _clearBusy || _runs.length === 0;
  }
  const msg = document.getElementById('run-picker-msg');
  if (msg) msg.textContent = _message;
}
