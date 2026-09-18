// ==================== 算法：规划模块 ====================
//
// Frenet 轨迹规划 — 由 4 个子模块组成：
//   plan/frenet.ts     — Frenet 坐标系构建 + Cartesian 互转 + 碰撞检测
//   plan/polynomial.ts — 五次多项式求解器（横向 l(s) + 纵向 s(t)）
//   plan/planner.ts    — 候选轨迹生成 + 代价评估 + 路径终点处理

export { plan, getPlannerState } from './planner.js';
