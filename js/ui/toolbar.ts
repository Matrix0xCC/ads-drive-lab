// ==================== UI：顶栏（世界簇 + 控制簇） ====================
// 世界：地图/场景选择；控制：暂停/回放/导出/导入/视角。
// 只做事件接线 → 状态机 dispatch 或 deps.actions；不持有状态。

import { UiController } from './controller.js';
import type { UiDeps } from './index.js';
import { ReplayController } from '../replay.js';
import { parseBundle } from '../headless/bundle.js';
import { frameCamera } from '../env/renderer.js';
import { SIM_W, SIM_H } from '../sim/engine.js';

export interface ToolbarHelpers {
  enterReplay: (load: () => Promise<void> | void) => Promise<void>;
  exitReplayIfActive: () => void;
}

export function wireToolbar(
  deps: UiDeps, ctl: UiController,
  helpers: ToolbarHelpers,
): void {
  const { enterReplay, exitReplayIfActive } = helpers;

  // ── 地图选择（保留场景配置仅换地图；回放中先退出回放）──
  // 下拉初值由 main 的 applyScenario 同步（它在 setupUI 之前执行）
  const mapSelect = document.getElementById('map-dropdown') as HTMLSelectElement | null;
  if (mapSelect) {
    mapSelect.addEventListener('change', () => {
      exitReplayIfActive();
      deps.actions.switchMap(mapSelect.value);
      ctl.dispatch({ type: 'SCENARIO_CHANGED', name: `browser-${mapSelect.value}` });
    });
  }

  // ── 场景加载（与 headless 同源：scenarios/index.json 列表） ──
  const scenarioSelect = document.getElementById('scenario-dropdown') as HTMLSelectElement | null;
  if (scenarioSelect) {
    fetch('scenarios/index.json')
      .then(r => (r.ok ? r.json() : { scenarios: [] }))
      .then((idx: any) => {
        for (const entry of idx.scenarios ?? []) {
          const opt = document.createElement('option');
          opt.value = entry.file;
          opt.textContent = `${entry.name}（${entry.map}）`;
          scenarioSelect.appendChild(opt);
        }
      })
      .catch(() => { /* 索引缺失不阻塞页面 */ });
    scenarioSelect.addEventListener('change', async () => {
      const file = scenarioSelect.value;
      exitReplayIfActive();
      if (!file) {
        deps.actions.loadDefaultScenario(deps.engine.mapDef.name);
        ctl.dispatch({ type: 'SCENARIO_CHANGED', name: '随机场景' });
        return;
      }
      try {
        const res = await fetch(`scenarios/${file}`);
        if (!res.ok) throw new Error(`HTTP ${res.status}`);
        const name = deps.actions.loadScenario(await res.json());
        if (name) ctl.dispatch({ type: 'SCENARIO_CHANGED', name });
      } catch (e) {
        alert(`场景加载失败: ${(e as Error).message}`);
      }
    });
  }

  // ── 控制簇 ──
  document.getElementById('btn-pause')?.addEventListener('click', () => {
    // 回放中切回放播放；LIVE 中切实时暂停（与 Space 语义一致）
    if (ctl.state.mode === 'REPLAY') deps.replay.toggle();
    else ctl.dispatch({ type: 'TOGGLE_PAUSE' });
  });

  document.getElementById('btn-replay-mode')?.addEventListener('click', async () => {
    const runs = await ReplayController.listRuns();
    if (runs.length === 0) {
      alert('没有可回放的运行。先执行: npm run sim -- --scenario scenarios/oval-cruise.json');
      return;
    }
    const choice = prompt('选择运行（输入编号）:\n' + runs.map((r, i) => `${i}: ${r.label}`).join('\n'), '0');
    if (choice === null) return;
    const idx = Number(choice);
    if (Number.isNaN(idx) || !runs[idx]) return;
    await enterReplay(() => deps.replay.load(runs[idx].id));
  });

  document.getElementById('btn-export')?.addEventListener('click', () => deps.actions.exportRun());

  document.getElementById('btn-import')?.addEventListener('click', () => {
    const input = document.createElement('input');
    input.type = 'file'; input.accept = '.json';
    input.onchange = (e: any) => {
      const file = e.target?.files?.[0];
      if (!file) return;
      const reader = new FileReader();
      reader.onload = () => {
        try {
          const bundle = parseBundle(JSON.parse(reader.result as string));
          void enterReplay(() => deps.replay.loadBundle(bundle));
        } catch (err) {
          alert(`归档导入失败: ${(err as Error).message}`);
        }
      };
      reader.readAsText(file);
    };
    input.click();
  });

  document.getElementById('btn-reset-view')?.addEventListener('click', () => {
    frameCamera(deps.camera, deps.controls, deps.canvas.clientWidth, deps.canvas.clientHeight, SIM_W, SIM_H);
  });
}
