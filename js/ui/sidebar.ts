// ==================== UI：左侧菜单栏（手风琴） ====================
// 菜单头点击 → dispatch TOGGLE_SECTION；内容区显隐/箭头由 setSidebarSection
// 按状态投影（单开）。DOM 结构见 index.html（section-<id> / menu-<id>）。
// 「信息」「调试」不在菜单内——它们是画布右侧的独立面板（见 hud.ts/inspector.ts）。

import { UiController } from './controller.js';
import { SidebarSection } from './state.js';

const SECTIONS: SidebarSection[] = ['world', 'sim', 'data'];

export function wireSidebar(ctl: UiController): void {
  for (const id of SECTIONS) {
    document.getElementById(`menu-${id}`)?.addEventListener('click', () => {
      ctl.dispatch({ type: 'TOGGLE_SECTION', section: id });
    });
  }
}

/** 状态投影：展开对应分区内容，更新菜单头箭头（▾ 展开 / ▸ 收起） */
export function setSidebarSection(section: SidebarSection | null): void {
  for (const id of SECTIONS) {
    const body = document.getElementById(`section-${id}`);
    const arrow = document.getElementById(`arrow-${id}`);
    if (body) body.style.display = section === id ? 'block' : 'none';
    if (arrow) arrow.textContent = section === id ? '▾' : '▸';
  }
}
