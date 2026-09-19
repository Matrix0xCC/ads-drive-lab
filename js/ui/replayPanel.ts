// ==================== UI：回放栏 ====================
// 时间轴/问题标记/播放/速度/步进/关闭。
// 播放内部状态归 ReplayController；模式（LIVE/REPLAY）接线见 index.ts。

import type { UIContext } from './index.js';
import { ReplayController } from '../replay.js';
import { debug } from '../shared/debug.js';

export interface ReplayPanelApi {
  show(): void;
  hide(): void;
  renderMarkers(): void;
}

export function setupReplayPanel(
  ctx: UIContext,
  enterReplay: (load: () => Promise<void> | void) => Promise<void>,
): ReplayPanelApi {
  const replay = ctx.replay;
  const replayBar = document.getElementById('replay-bar') as HTMLDivElement | null;
  const timeline = document.getElementById('replay-timeline') as HTMLInputElement | null;
  const stepLabel = document.getElementById('replay-step-label');
  const markersBox = document.getElementById('replay-markers');

  function renderMarkers() {
    if (!markersBox) return;
    markersBox.innerHTML = '';
    const total = replay.state.totalSteps || 1;
    for (const step of replay.state.markers) {
      const dot = document.createElement('div');
      dot.className = 'replay-marker';
      dot.style.left = `${(step / total) * 100}%`;
      dot.title = `问题帧 ${step}`;
      dot.addEventListener('click', () => replay.seekStep(step));
      markersBox.appendChild(dot);
    }
  }

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
    await enterReplay(() => replay.load(runs[idx].id));
  });

  document.getElementById('btn-replay-play')?.addEventListener('click', () => replay.toggle());
  document.getElementById('btn-replay-prev')?.addEventListener('click', () => replay.seekStep(replay.state.currentStep - 1));
  document.getElementById('btn-replay-next')?.addEventListener('click', () => replay.seekStep(replay.state.currentStep + 1));
  (document.getElementById('replay-speed') as HTMLSelectElement | null)
    ?.addEventListener('change', (e: any) => replay.setSpeed(Number(e.target.value)));
  document.getElementById('btn-replay-close')?.addEventListener('click', () => {
    replay.close();
    if (replayBar) replayBar.style.display = 'none';
    debug.paused = false;                      // 恢复实时仿真（T5 由状态机接管后删除）
  });
  timeline?.addEventListener('input', () => replay.seekStep(Number(timeline.value)));
  replay.onChange = () => {
    if (timeline) {
      timeline.value = String(Math.floor(replay.state.currentStep));
      timeline.max = String(replay.state.totalSteps);
    }
    if (stepLabel) stepLabel.textContent = `${Math.floor(replay.state.currentStep)} / ${replay.state.totalSteps}`;
  };
  window.addEventListener('keydown', (e) => {
    if (!replay.state.active || e.target instanceof HTMLInputElement) return;
    if (e.key === 'ArrowLeft') replay.seekStep(replay.state.currentStep - 1);
    if (e.key === 'ArrowRight') replay.seekStep(replay.state.currentStep + 1);
  });

  return {
    show() { if (replayBar) replayBar.style.display = 'flex'; },
    hide() { if (replayBar) replayBar.style.display = 'none'; },
    renderMarkers,
  };
}
