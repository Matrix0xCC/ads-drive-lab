// ==================== 测试：感知的背景减除参考线随世界更新 ====================
//
// 回归背景（浏览器实测）：模块级 `_bgFrenetRef` 只在首次构建、永不失效。
// headless 每次新进程 → 从不暴露；浏览器是"同一进程内引擎 rebuildWorld 切换
// 地图/场景" → 切到新地图后背景减除仍用旧地图参考线 → 障碍命中被 |l| 误判为
// 边界并过滤 → 规划器"看不见"障碍 → 小车直穿障碍物（batch-hairpin-s4 实测）。

import { describe, it } from 'node:test';
import assert from 'node:assert';
import { sense, MAX_SENSOR_DIST } from '../../algo/perception.js';
import { CarState, WorldSnapshot } from '../../shared/types.js';
import { rad, m, mps, mps2, px, pxps, dimless } from '../../shared/dimensions.js';

/** 直线参考线（中心线 y = yOffset），无围墙（隔离分类逻辑） */
function straightTrack(yOffset: number): any {
  const pts: any[] = [];
  for (let i = 0; i <= 200; i++) pts.push({ x: px(i * 10), y: px(yOffset) });
  return { centerPoints: pts, innerBoundary: [], outerBoundary: [] };
}

function makeCar(x: number, y: number): CarState {
  return {
    x: px(x), y: px(y), angle: rad(0), speed: pxps(0),
    steerAngle: rad(0), maxSpeed: mps(42.5), acceleration: mps2(8),
    friction: dimless(0.12), width: m(2.0), length: m(4.5),
    collisionRadius: px(Math.hypot(8, 18) / 2),
  };
}

/** 车正前方 50m 的方形障碍 */
function snapshotOn(yOffset: number): WorldSnapshot {
  return {
    car: makeCar(200, yOffset),
    track: straightTrack(yOffset),
    obstacles: [{ x: px(400), y: px(yOffset), width: px(8), height: px(8), angle: rad(0), zHeight: 8 } as any],
    timestamp: 0,
  };
}

const MID = 30;   // 车头正前方射线（61 条、±60° FOV 的中间一条）

describe('感知：背景减除参考线缓存', () => {
  it('换地图后 sense 必须用新地图参考线（障碍不能被误判为边界过滤）', () => {
    // 地图 A（中心线 y=1000u）：先感知一次，使模块缓存建立在 A 上
    const scanA = sense(snapshotOn(1000));
    assert.ok((scanA.distances[MID] as number) < (MAX_SENSOR_DIST as number) - 1,
      '前提：地图 A 上正前方 50m 障碍应被识别');

    // 切到地图 B（中心线 y=400u，同一相对几何）——模拟浏览器切地图/场景
    const scanB = sense(snapshotOn(400));
    assert.ok((scanB.distances[MID] as number) < (MAX_SENSOR_DIST as number) - 1,
      `地图 B 上障碍应被识别（实际 ${(scanB.distances[MID] as number) / 4}m —— 若为 125m 说明仍在用旧地图参考线）`);
    assert.strictEqual(scanB.confidences![MID], 1);
  });
});
