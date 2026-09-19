// ==================== UI：顶栏（世界簇 + 控制簇） ====================
// 世界：地图/场景选择；控制：暂停/回放/导出/导入/视角。
// 事件接线，状态逻辑见 state.ts/controller.ts（T5 接线）。

import type { UIContext } from './index.js';
import { DEFAULT_MAP } from '../env/mapDefinitions.js';
import { parseBundle } from '../headless/bundle.js';

export interface ToolbarHelpers {
  enterReplay: (load: () => Promise<void> | void) => Promise<void>;
  resetCamera: () => void;
  togglePause: () => void;
}

export function wireToolbar(ctx: UIContext, helpers: ToolbarHelpers): void {
  // ── 地图选择 ──
  const mapSelect = document.getElementById('map-dropdown') as HTMLSelectElement;
  if (mapSelect) {
    mapSelect.value = DEFAULT_MAP;
    mapSelect.addEventListener('change', () => {
      ctx.onMapChange(mapSelect.value);
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
      if (!file) { ctx.onScenarioReset(); return; }
      try {
        const res = await fetch(`scenarios/${file}`);
        if (!res.ok) throw new Error(`HTTP ${res.status}`);
        ctx.onScenarioLoad(await res.json());
      } catch (e) {
        alert(`场景加载失败: ${(e as Error).message}`);
      }
    });
  }

  // ── 运行归档导出/导入（与 headless 同格式：.run.json 归档包） ──
  document.getElementById('btn-export')?.addEventListener('click', () => {
    ctx.onExportRun();
  });

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
          void helpers.enterReplay(() => ctx.replay.loadBundle(bundle));
        } catch (err) {
          alert(`归档导入失败: ${(err as Error).message}`);
        }
      };
      reader.readAsText(file);
    };
    input.click();
  });

  // ── 暂停 / 视角重置按钮 ──
  document.getElementById('btn-pause')?.addEventListener('click', () => helpers.togglePause());
  document.getElementById('btn-reset-view')?.addEventListener('click', () => helpers.resetCamera());
}
