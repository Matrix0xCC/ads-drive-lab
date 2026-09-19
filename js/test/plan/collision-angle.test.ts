// ==================== 测试：isLidarCollision 角度环绕回归 ====================
//
// 背景：扫描角数组存未归一化的绝对角（car.angle + offset，car.angle 单调累积），
// 而轨迹点方位角来自 atan2（恒在 [-π,π]）。历史上归一化公式未处理
// 差值 > 2π 的情况（产生负差值 → 错误的"最近射线"），导致车辆朝向
// 积累过 180° 后碰撞检测在部分方位完全失明（2026-09-19 批量测试定位）。
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { isLidarCollision } from '../../algo/plan/collision.js';
import { MAX_SENSOR_DIST } from '../../algo/perception.js';
import { rad, px } from '../../shared/dimensions.js';

/**
 * 构造指定车头朝向下（未归一化）的扫描：仅车头正前方（idx 30）有命中。
 * @param carAngleRad 车头朝向原始值（可超过 ±π）
 * @param hitDist     正前方命中距离（仿真单位）
 */
function scanForwardHit(carAngleRad: number, hitDist: number | null) {
  const n = 61;
  const angles: any[] = [], distances: any[] = [], rawDistances: any[] = [], confidences: number[] = [];
  for (let i = 0; i < n; i++) {
    const off = (-60 + i * 2) * Math.PI / 180;
    angles.push(rad(carAngleRad + off));           // 未归一化绝对角（与真实数据同形态）
    const hit = hitDist !== null && i === 30;
    distances.push(px(hit ? hitDist : (MAX_SENSOR_DIST as number)));
    rawDistances.push(px(hit ? hitDist : (MAX_SENSOR_DIST as number)));
    confidences.push(hit ? 1 : 0);
  }
  return { angles, distances, rawDistances, confidences, timestamp: 0 };
}

/** 车头前方 dist 单位处、沿实际前进方向（世界朝向 = carAngle）的轨迹点 */
function pointAheadOf(carAngleRad: number, dist: number): { x: number; y: number } {
  return { x: Math.cos(carAngleRad) * dist, y: Math.sin(carAngleRad) * dist };
}

const MARGIN = 20;  // 仿真单位（5m）

describe('isLidarCollision 锥形窗口随命中距离缩放', () => {
  // 命中距离 200u（50m）处目标角宽很窄（±1.25m ≈ ±1.4°）：
  // 轨迹点在 4.5° 方位（侧向绕行）不应被固定 ±4° 宽锥牵连误判；
  // 正对（同方位）的轨迹点仍应判碰撞。
  const scanHitAt0 = (d: number) => {
    const n = 61;
    const angles: any[] = [], distances: any[] = [], rawDistances: any[] = [], confidences: number[] = [];
    for (let i = 0; i < n; i++) {
      const off = (-60 + i * 2) * Math.PI / 180;
      angles.push(rad(off));
      const hit = Math.abs(off) < 0.025;   // 单射线（现实量级：50m 处 2.5m 宽）
      distances.push(px(hit ? d : (MAX_SENSOR_DIST as number)));
      rawDistances.push(px(hit ? d : (MAX_SENSOR_DIST as number)));
      confidences.push(hit ? 1 : 0);
    }
    return { angles, distances, rawDistances, confidences, timestamp: 0 };
  };

  it('远距离目标的窄角宽度：4.5° 侧偏的轨迹点不判碰撞', () => {
    const scan = scanHitAt0(200);
    const pt = pointAheadOf((4.5 * Math.PI) / 180, 240);   // 越过 200u 处目标 40u（> margin 20）
    assert.strictEqual(isLidarCollision(pt.x, pt.y, 0, 0, scan as any, MARGIN), false);
  });

  it('同方位（正对）的轨迹点仍判碰撞（对照）', () => {
    const scan = scanHitAt0(200);
    const pt = pointAheadOf(0, 240);   // 200 + MARGIN(20) < 240
    assert.strictEqual(isLidarCollision(pt.x, pt.y, 0, 0, scan as any, MARGIN), true);
  });
});

describe('isLidarCollision 角度环绕', () => {
  it('car.angle=190°（原始值 > π）：正前方障碍 + 轨迹越过 → 判碰撞', () => {
    const carAngle = 190 * Math.PI / 180;          // 3.316 rad，超过 π 的原始值
    const scan = scanForwardHit(carAngle, 100);
    const pt = pointAheadOf(carAngle, 200);        // 越过 100 单位处的障碍
    assert.strictEqual(isLidarCollision(pt.x, pt.y, 0, 0, scan as any, MARGIN), true);
  });

  it('car.angle=-200°（原始值 < -π）：同样判碰撞', () => {
    const carAngle = -200 * Math.PI / 180;         // -3.491 rad
    const scan = scanForwardHit(carAngle, 100);
    const pt = pointAheadOf(carAngle, 200);
    assert.strictEqual(isLidarCollision(pt.x, pt.y, 0, 0, scan as any, MARGIN), true);
  });

  it('car.angle=190° 但无障碍命中 → 不判碰撞（负例）', () => {
    const carAngle = 190 * Math.PI / 180;
    const scan = scanForwardHit(carAngle, null);
    const pt = pointAheadOf(carAngle, 200);
    assert.strictEqual(isLidarCollision(pt.x, pt.y, 0, 0, scan as any, MARGIN), false);
  });

  it('car.angle=190° 障碍在 150 单位但余量未覆盖（100+20 > 150）→ 不判碰撞（边界负例）', () => {
    const carAngle = 190 * Math.PI / 180;
    const scan = scanForwardHit(carAngle, 100);
    const pt = pointAheadOf(carAngle, 110);        // 越过障碍仅 10 < margin 20
    assert.strictEqual(isLidarCollision(pt.x, pt.y, 0, 0, scan as any, MARGIN), false);
  });
});
