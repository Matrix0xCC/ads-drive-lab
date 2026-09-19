// ==================== UI：回放栏 ====================
// 时间轴/问题标记/播放/速度/步进/关闭。播放内部状态归 ReplayController；
// 模式（LIVE/REPLAY）归 UiState——本模块只接线。
// 键盘（Space/←→/Esc）统一在 ui/index.ts 的分发器中按模式路由。

import type { UiDeps } from './index.js';

export interface ReplayPanelApi {
  show(): void;
  hide(): void;
  renderMarkers(): void;
}

export function setupReplayPanel(
  deps: UiDeps,
  enterReplay: (load: () => Promise<void> | void) => Promise<void>,
  exitReplay: () => void,
): ReplayPanelApi {
  const replay = deps.replay;
  const replayBar = document.getElementById('replay-bar') as HTMLDivElement | null;
  const timeline = document.getElementById('replay-timeline') as HTMLInputElement | null;
  const stepLabel = document.getElementById('replay-step-label');
  const markersBox = document.getElementById('replay-markers');

  function renderMarkers(): void {
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

  document.getElementById('btn-replay-play')?.addEventListener('click', () => replay.toggle());
  document.getElementById('btn-replay-prev')?.addEventListener('click', () => replay.seekStep(replay.state.currentStep - 1));
  document.getElementById('btn-replay-next')?.addEventListener('click', () => replay.seekStep(replay.state.currentStep + 1));
  (document.getElementById('replay-speed') as HTMLSelectElement | null)
    ?.addEventListener('change', (e: any) => replay.setSpeed(Number(e.target.value)));
  document.getElementById('btn-replay-close')?.addEventListener('click', () => exitReplay());
  timeline?.addEventListener('input', () => replay.seekStep(Number(timeline.value)));
  replay.onChange = () => {
    if (timeline) {
      timeline.value = String(Math.floor(replay.state.currentStep));
      timeline.max = String(replay.state.totalSteps);
    }
    if (stepLabel) stepLabel.textContent = `${Math.floor(replay.state.currentStep)} / ${replay.state.totalSteps}`;
  };

  return {
    show() { if (replayBar) replayBar.style.display = 'flex'; },
    hide() { if (replayBar) replayBar.style.display = 'none'; },
    renderMarkers,
  };
}
