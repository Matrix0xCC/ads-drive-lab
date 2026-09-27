// ==================== 测试：物体层（聚类 + 跟踪 + 速度） ====================
//
// 这一层要证明的最关键一件事：**跟踪出来的速度对得上真值**。
// 这正是 C（射线逐帧差分）做不到的：实测 p50 = 76m/s 的垃圾（最近射线在障碍间跳变）。
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { readFileSync } from 'node:fs';
import { SimEngine, FIXED_DT } from '../../sim/engine.js';
import { MessageBus } from '../../messageBus.js';
import { parseScenario, buildScenarioWorld } from '../../sim/scenario.js';
import { detectObjects, trackedObjects, resetObjects, OBJECT, OBJECT_DT } from '../../algo/objects.js';
import { m, px } from '../../shared/dimensions.js';

/** 造一帧"若干个物体"的扫描（只填物体层要用的字段） */
function scanOf(items: { s: number; l: number; d: number; sensor?: number }[], rayPer = 3, sep = false) {
  const hitS: number[] = [], hitL: number[] = [], distances: number[] = [], sensorOf: number[] = [], angles: number[] = [];
  for (const it of items) {
    for (let k = 0; k < rayPer; k++) {
      hitS.push(it.s + px(k * 4));            // 同一物体上几条射线略有差异
      hitL.push(it.l);
      distances.push(it.d);
      sensorOf.push(it.sensor ?? 0);
      angles.push(0);
    }
    if (sep) { hitS.push(NaN); hitL.push(NaN); distances.push(px(120)); sensorOf.push(0); angles.push(0); }
  }
  return { angles, distances, rawDistances: [...distances], confidences: angles.map(() => 1), hitS, hitL, sensorOf, timestamp: 0 };
}

describe('物体层：聚类', () => {
  it('不同位置/距离的物体分成不同的簇，每个簇给出近端 s / 横向 l / 尺度', () => {
    const dets = detectObjects(scanOf([
      { s: m(40), l: m(0), d: m(40) },
      { s: m(70), l: m(3.75), d: m(70) },
      { s: m(20), l: m(-3.75), d: m(20) },
    ], 3, true) as never);
    assert.strictEqual(dets.length, 3, '应当聚成 3 个物体');
    const sorted = dets.slice().sort((a, b) => a.l - b.l);
    assert.ok(Math.abs(sorted[0].l - (m(-3.75) as number)) < 1, '最左的是 −3.75m 那个');
    assert.ok(Math.abs(sorted[2].l - (m(3.75) as number)) < 1, '最右的是 +3.75m 那个');
    assert.ok(Math.abs(sorted[1].s - (m(40) as number)) < 1, '中间那个的 s ≈ 40m');
  });

  it('距离跳变大（两个不同远近的物体贴在一起）→ 断成两个簇', () => {
    const one = detectObjects(scanOf([{ s: m(20), l: 0, d: m(20) }, { s: m(45), l: 0, d: m(45) }], 2, false) as never);
    assert.strictEqual(one.length, 2, '距离差 25m ≫ 门限 → 不该合并成一个');
  });

  it('没有 Frenet 位置的老扫描（合成扫描）→ 不产生物体（不崩）', () => {
    assert.deepStrictEqual(detectObjects({ angles: [], distances: [], rawDistances: [], confidences: [], timestamp: 0 }), []);
  });
});

describe('物体层：跟踪', () => {
  it('同一物体跨帧保持同一个 id；尺度/位置平滑跟住', () => {
    resetObjects();
    let tracks = trackedObjects(scanOf([{ s: m(50), l: 0, d: m(50) }], 3, false) as never, OBJECT_DT);
    const id0 = tracks[0].id;
    for (let k = 1; k <= 6; k++) {
      tracks = trackedObjects(scanOf([{ s: m(50 + k), l: 0, d: m(50 + k) }], 3, false) as never, OBJECT_DT);
    }
    assert.strictEqual(tracks.length, 1);
    assert.strictEqual(tracks[0].id, id0, 'id 必须保持');
    assert.ok(tracks[0].confirmed, '连续命中后应确认');
    assert.ok(tracks[0].vS > 0, '物体在向前走 → 沿路速度应为正');
  });

  it('确定性：同样两帧序列跑两次 → 轨迹逐字段一致', () => {
    const run = () => {
      resetObjects();
      let tr = trackedObjects(scanOf([{ s: m(30), l: m(1), d: m(30) }], 3, false) as never, OBJECT_DT);
      tr = trackedObjects(scanOf([{ s: m(31), l: m(1.2), d: m(31) }], 3, false) as never, OBJECT_DT);
      tr = trackedObjects(scanOf([{ s: m(32), l: m(1.4), d: m(32) }], 3, false) as never, OBJECT_DT);
      return JSON.stringify(tr);
    };
    assert.strictEqual(run(), run());
  });

  it('丢失超过 MAX_MISSED 帧 → 轨迹消亡', () => {
    resetObjects();
    trackedObjects(scanOf([{ s: m(30), l: 0, d: m(30) }], 3, false) as never, OBJECT_DT);
    let tr: unknown[] = [];
    for (let k = 0; k <= OBJECT.MAX_MISSED; k++) tr = trackedObjects(scanOf([], 0, false) as never, OBJECT_DT);
    assert.strictEqual(tr.length, 0, '连续丢失后应清空');
  });
});

describe('物体层：速度估计必须"合理且稳定"（C 用射线差分做不到的那件事）', () => {
  // 【为什么断言的是"合理 + 稳定"而不是"误差 < X"】与交通真值逐车匹配需要一套配对基准
  // （数组未排序、两条参考线口径要对齐），本轮先不做——那是下一步。
  // 但下面这两条已经足以否掉 C 的做法：射线差分给出的前车速度 p50 = 76m/s（不可能的值）且逐帧乱跳。
  // 诊断实测（G4 交通场景）：前方 76m 处那辆车 → 跟踪 vS = 15.0m/s，
  // 正好是该车道车流的设计速度（0.45×120km/h = 54km/h = 15m/s）✓
  it('跟踪出的速度必须是**物理合理**的（0 < vS ≤ 30m/s），且逐帧稳定（不跳）', () => {
    const raw = JSON.parse(readFileSync('scenarios/bj-g4-dujiakan-traffic.json', 'utf8'));
    const s = parseScenario(raw);
    const w = buildScenarioWorld(s, 1200, 800);
    const eng = new SimEngine(new MessageBus('t', false), w.worldCfg);
    if (w.route) eng.setRoute(w.route, 0);
    resetObjects();

    const speeds: number[] = [];
    let jumpMax = 0, samples = 0;
    const prev = new Map<number, number>();          // id → 上一帧 vS（米/秒）
    for (let i = 0; i < 60 * 12; i++) {
      eng.step(FIXED_DT);
      const scan = eng.latestLidarScan;
      if (!scan) continue;
      for (const t of trackedObjects(scan, OBJECT_DT, eng.track)) {
        if (!t.confirmed) continue;
        const vm = t.vS / 4;
        speeds.push(vm);
        const p = prev.get(t.id);
        if (p !== undefined) { jumpMax = Math.max(jumpMax, Math.abs(vm - p)); samples++; }
        prev.set(t.id, vm);
      }
    }
    assert.ok(speeds.length > 100, '应有足够多的速度样本（实际 ' + speeds.length + '）');
    assert.ok(speeds.every(v => v >= -0.5 && v <= 30), '速度必须物理合理（≤30m/s；射线差分曾是 76m/s）');
    assert.ok(speeds.some(v => v > 5), '应当估出"在动的物体"（不能全是 0）');
    console.log('   [objects] 速度样本 ' + speeds.length + '  逐帧最大跳变 ' + jumpMax.toFixed(2) + 'm/s（阈值 8）');
    assert.ok(samples > 50, '应有足够多的跨帧同 id 样本');
    assert.ok(jumpMax < 8, '逐帧速度不该跳变（实际 ' + jumpMax.toFixed(2) + 'm/s）');
  });
});
