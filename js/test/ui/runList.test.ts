// ==================== 测试：回放运行选择框（分页纯函数） ====================
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { pageCount, pageSlice, formatRunTime } from '../../ui/runList.js';

describe('runList 分页', () => {
  it('页数计算：空列表也算 1 页（空态展示），末页不满', () => {
    assert.strictEqual(pageCount(0, 10), 1);
    assert.strictEqual(pageCount(1, 10), 1);
    assert.strictEqual(pageCount(25, 10), 3);
  });

  it('切片：页码越界夹紧（负数→首页，超界→末页）', () => {
    const items = [1, 2, 3, 4, 5];
    assert.deepStrictEqual(pageSlice(items, 0, 2), [1, 2]);
    assert.deepStrictEqual(pageSlice(items, 1, 2), [3, 4]);
    assert.deepStrictEqual(pageSlice(items, 2, 2), [5]);
    assert.deepStrictEqual(pageSlice(items, 9, 2), [5]);
    assert.deepStrictEqual(pageSlice(items, -1, 2), [1, 2]);
  });
});

describe('runList 时间显示', () => {
  it('ISO → 本地 MM-DD HH:mm', () => {
    const d = new Date(2026, 8, 19, 11, 42);   // 本地 2026-09-19 11:42
    assert.strictEqual(formatRunTime(d.toISOString()), '09-19 11:42');
  });

  it('非法时间原文返回（不崩溃）', () => {
    assert.strictEqual(formatRunTime('garbage'), 'garbage');
  });
});
