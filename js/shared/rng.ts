// ==================== 可播种随机数（PRNG） ====================
//
// 【为什么需要？】
// headless 模式要求"同种子 → 逐帧一致"，Math.random() 无法复现。
// 所有随机子系统（障碍物布局 / GPS 噪声 / 随机赛道）都接收 Rng 注入。
//
// 【mulberry32】32 位种子、周期 2^32、统计质量足够仿真使用，
// 实现仅 6 行且完全确定性（无内部隐式状态）。
//
// 【子种子派生】同一根种子的不同子系统必须用不同 RNG 实例，
// 否则一个子系统的取数次数会偏移另一个子系统的序列（顺序耦合）。
// deriveSeed(base, 'gps') 用 FNV-1a 从根种子派生独立子种子。

export type Rng = () => number;

/** mulberry32 — 返回 [0,1) 均匀分布随机数 */
export function createRng(seed: number): Rng {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6D2B79F5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

/** FNV-1a 派生子种子 — 与调用顺序、次数无关 */
export function deriveSeed(baseSeed: number, label: string): number {
  let h = (2166136261 ^ (baseSeed >>> 0)) >>> 0;
  for (let i = 0; i < label.length; i++) {
    h ^= label.charCodeAt(i);
    h = Math.imul(h, 16777619) >>> 0;
  }
  return h >>> 0;
}
