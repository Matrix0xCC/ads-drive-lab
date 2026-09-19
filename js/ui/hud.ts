// ==================== UI：提示条 ====================
// 底部上下文提示（#hint-bar）渲染；文案由 state.ts 的 deriveHint 派生。
// HUD 顶部「地图/场景」标签由 main.ts 的 applyScenario 写入（与真实世界状态同处）。

export function renderHint(text: string): void {
  const el = document.getElementById('hint-bar');
  if (el) el.textContent = text;
}
