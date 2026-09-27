// ==================== 测试：main.ts 的模块初始化顺序（防 TDZ） ====================
//
// 【为什么需要】浏览器里踩过两次同一个坑：给"**模块初始化阶段就会被调用**的函数"引用了
// **声明位置更靠后**的模块级绑定，于是刷新即崩：
//   `Uncaught ReferenceError: Cannot access 'xxx' before initialization`
// 两次都出在 `applyScenario`（定义在前、顶层第 112 行调用）：
//   ① 我新加的 `let basemapAttribution`（声明在 145 行，用在 70 行）
//   ② `const ui = setupUI(...)`（116 行才初始化，applyScenario 里却用了它）
// TypeScript **不会**报这种错（跨函数体的引用不做"先声明后使用"检查），只在浏览器里炸。
//
// 判据（精确版）：**只在顶层代码里被调用的**函数，其函数体不得引用分界之后声明的模块级名字。
// 只在回调/定时器里被调用的函数（例如 renderBasemapAttribution 只由总线回调调用）不在检查范围——
// 那时 `ui` 早已初始化，安全。
// 纯静态检查：只读源码，不需要 DOM/WebGL。
import { describe, it } from 'node:test';
import assert from 'node:assert';
import * as fs from 'node:fs';
import * as path from 'node:path';

const SRC = path.resolve('js/main.ts');

/** 从 `{` 起按花括号配对取出函数体（粗粒度，够用：源码里没有含花括号的字符串） */
function bodyEnd(src: string, openBraceIdx: number): number {
  let depth = 0;
  for (let i = openBraceIdx; i < src.length; i++) {
    if (src[i] === '{') depth++;
    else if (src[i] === '}') { depth--; if (depth === 0) return i + 1; }
  }
  return src.length;
}

/** 顶层函数定义（名字 + 函数体区间） */
function topLevelFunctions(src: string): { name: string; start: number; end: number; bodyOpen: number }[] {
  const out: { name: string; start: number; end: number; bodyOpen: number }[] = [];
  for (const m of src.matchAll(/^(?:export\s+)?(?:async\s+)?function\s+([\w$]+)\s*\([^)]*\)[^{]*\{/gm)) {
    const bodyOpen = (m.index ?? 0) + m[0].length - 1;
    out.push({ name: m[1], start: m.index ?? 0, end: bodyEnd(src, bodyOpen), bodyOpen });
  }
  return out;
}

/** 去掉所有函数体后的源码（剩下的就是"模块初始化时真正执行的语句"） */
function topLevelCodeOnly(src: string): string {
  let out = '';
  let cursor = 0;
  for (const f of topLevelFunctions(src)) {
    out += src.slice(cursor, f.bodyOpen + 1).replace(/\{[\s\S]*$/, '{');   // 保留签名部分，丢掉函数体
    cursor = f.end;
  }
  out += src.slice(cursor);
  return out;
}

describe('main.ts 模块初始化顺序', () => {
  const src = fs.readFileSync(SRC, 'utf8');
  const cutMatch = /^const ui = setupUI\(/m.exec(src);
  assert.ok(cutMatch, '找不到 `const ui = setupUI(`（结构变了就同步更新本测试）');
  const cut = cutMatch!.index;

  /** 分界之后声明的模块级名字（含分界那一行本身） */
  const declaredAfter = [...src.slice(cut).matchAll(/^(?:let|const|var)\s+([A-Za-z_$][\w$]*)/gm)].map(m => m[1]);

  /** 模块初始化阶段（分界之前）就被调用的函数名 */
  const initCode = topLevelCodeOnly(src.slice(0, cut));
  // 注意：`topLevelCodeOnly` 会保留函数**签名**，而签名里就含 `name(` —— 不剔除的话每个函数都会
  // 被误判成"被调用"（假阳性；实测踩过）。匹配前先把"自己的定义"从待搜文本里去掉。
  const calledAtInit = topLevelFunctions(src.slice(0, cut))
    .map(f => f.name)
    .filter(name => new RegExp(`\\b${name}\\s*\\(`)
      .test(initCode.replace(new RegExp(`function\\s+${name}\\s*\\(`, 'g'), '')));

  it('分界之后确实还有模块级声明（否则本测试失去意义）', () => {
    assert.ok(declaredAfter.length > 0, `declaredAfter=${declaredAfter.join(',')}`);
    assert.ok(calledAtInit.includes('applyScenario'), `模块初始化阶段应当调用 applyScenario，实际：${calledAtInit.join(',')}`);
  });

  it('模块初始化阶段被调用的函数，不引用之后声明的模块级绑定', () => {
    const offenders: string[] = [];
    for (const f of topLevelFunctions(src.slice(0, cut))) {
      if (!calledAtInit.includes(f.name)) continue;               // 只查初始化阶段会被调用的
      const body = src.slice(f.bodyOpen, f.end);
      for (const name of declaredAfter) {
        if (new RegExp(`\\b${name}\\b`).test(body)) offenders.push(`${f.name}() 引用了 ${name}`);
      }
    }
    assert.deepStrictEqual(offenders, [],
      `发现 TDZ 风险（模块初始化时这些绑定还不存在）：\n  ${offenders.join('\n  ')}`);
  });
});
