// ==================== 测试：换地图时的路线合法性 ====================
//
// 【背景】路线声明是**地图专属**的：路网地图用 fromNode/toNode（节点序号），链式/径向用 startS/goalS（弧长）。
// 而 UI 上"地点（地图）"是一个可以单独切换的维度 → 切换后旧路线会被套到新地图上。
// 实测（本次修复前）：
//   · 三环路线(节点 2→93) 套到 G6(25 节点) → 建图内部爆 **RangeError: Invalid array length**（不可读）
//   · 弧长路线套到另一张图 → centerPoints[i] 变 undefined → 路线带着坏点继续（长度算成 NaN）
// 现在这两类都必须是**可读的 ScenarioError**，并且带上"可能来自别的地图"这个诊断。
import { describe, it } from 'node:test';
import assert from 'node:assert';
import * as fs from 'node:fs';
import * as path from 'node:path';
import { buildScenarioWorld } from '../../sim/scenario.js';

const SIM_W = 1200, SIM_H = 800;
const load = (f: string) => JSON.parse(fs.readFileSync(path.resolve('scenarios', f + '.json'), 'utf8'));

/** 复现 main.ts 的 switchMap：保留场景配置，只换地图 */
function buildWithMap(file: string, map: string) {
  return buildScenarioWorld({ ...load(file), map }, SIM_W, SIM_H);
}

describe('换地图：路线声明不兼容时必须给可读错误', () => {
  it('路网路线套到另一张路网（节点序号越界）→ ScenarioError（不是 RangeError），且提示"可能来自别的地图"', () => {
    let err: Error | null = null;
    try { buildWithMap('bj-3rd-ring-route', 'bj-g6-junction'); } catch (e) { err = e as Error; }
    assert.ok(err, '跨路网套用旧路线必须失败');
    assert.strictEqual(err!.constructor.name, 'ScenarioError', `必须是可读的 ScenarioError，实际 ${err!.constructor.name}`);
    assert.match(err!.message, /节点序号/, '要说清是节点序号问题');
    assert.match(err!.message, /别的地图/, '要给出"路线来自别的地图"这个诊断');
    assert.doesNotMatch(err!.message, /Invalid array length/, '不能把内部异常原样抛给用户');
  });

  it('弧长路线（{startS,goalS}）套到路网地图 → 可读错误（要求 fromNode/toNode）', () => {
    assert.throws(() => buildWithMap('bj-g4-dujiakan-cruise', 'bj-g6-junction'),
      (e: Error) => e.constructor.name === 'ScenarioError' && /fromNode/.test(e.message));
  });

  it('路网路线套到链式/径向地图 → 可读错误（不是 undefined 的 startS/goalS）', () => {
    assert.throws(() => buildWithMap('bj-3rd-ring-route', 'oval'),
      (e: Error) => e.constructor.name === 'ScenarioError' && /路线不可达/.test(e.message));
  });

  it('同图加载仍然正常（守卫不能误伤）', () => {
    for (const f of ['oval-cruise', 'bj-g6-junction-route', 'bj-3rd-ring-route', 'bj-g4-dujiakan-cruise']) {
      const s = load(f);
      const b = buildScenarioWorld(s, SIM_W, SIM_H);
      assert.ok(b.route, `${f} 应当有路线`);
      assert.ok(Number.isFinite(b.route!.length as unknown as number), `${f} 路线长度必须是有限值`);
      assert.ok((b.route!.length as unknown as number) > 0, `${f} 路线长度必须为正`);
    }
  });

  it('路线长度必须是有限值（NaN 兜底守卫）', () => {
    for (const f of ['oval-cruise', 'bj-g6-junction-route']) {
      const b = buildScenarioWorld(load(f), SIM_W, SIM_H);
      assert.ok(Number.isFinite(b.route!.length as unknown as number));
    }
  });
});
