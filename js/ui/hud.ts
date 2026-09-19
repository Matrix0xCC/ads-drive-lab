// ==================== UI：提示条 + 右侧信息面板 ====================
// 底部上下文提示（#hint-bar）渲染；文案由 state.ts 的 deriveHint 派生。
// 「信息」面板（#info-panel，画布右上）承载详细遥测内容区折叠；
// 遥测数值本身由渲染层按元素 id 更新（env/renderer/hud.ts）。
// 「地图/场景」标签由 main.ts 的 applyScenario 写入（与真实世界状态同处）。

export function renderHint(text: string): void {
  const el = document.getElementById('hint-bar');
  if (el) el.textContent = text;
}

/** 折叠/展开信息面板（控制内容区显隐与按钮箭头） */
export function setInfoOpen(open: boolean): void {
  const body = document.getElementById('info-body');
  const btn = document.getElementById('btn-info-toggle');
  if (body) body.style.display = open ? 'block' : 'none';
  if (btn) btn.textContent = open ? '▾' : '▸';
}
