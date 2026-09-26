// ==================== 测试：规划器 ====================

import { describe, it } from 'node:test';
import assert from 'node:assert';
import { plan, getPlannerState } from '../../algo/plan/index.js';
import { buildFrenetReference, cartesianToFrenet } from '../../algo/plan/frenet.js';
import { solvePathLayer, lateralSpeedCap, TIGHT_SPEED, PATH_HORIZON, PATH_STEPS } from '../../algo/plan/paths.js';
import { CarState, WorldSnapshot, LidarScan, Route } from '../../shared/types.js';
import { rad, m, mps, mps2, px, pxps, pxps2, dimless, perpx } from '../../shared/dimensions.js';
import { MAX_SENSOR_DIST } from '../../algo/perception.js';
import { debug } from '../../shared/debug.js';

// ─── 辅助 ───

function makeCar(x: number, y: number, angle: number, speed: number): CarState {
  return {
    x: px(x), y: px(y), angle: rad(angle), speed: pxps(speed),
    steerAngle: rad(0), maxSpeed: mps(42.5), acceleration: mps2(8),
    friction: dimless(0.12), width: m(2.0), length: m(4.5), collisionRadius: px(Math.hypot(m(2.0) as number, m(4.5) as number) / 2),
  };
}

/** 创建一条沿 x 轴的直线参考线（测试车辆在 l≈0 上，障碍物也在 l≈0） */
function makeSnapshot(car: CarState): WorldSnapshot {
  const pts: any[] = [];
  for (let i = 0; i <= 200; i++) pts.push({ x: px(i * 10), y: px(0) });
  pts.push(pts[0]);
  return { car, track: { centerPoints: pts, innerBoundary: [], outerBoundary: [] }, obstacles: [], timestamp: 0 };
}

function makeClearLidar(): LidarScan {
  const n = 61;
  const angles: any[] = [];
  const distances: any[] = [];
  for (let i = 0; i < n; i++) {
    angles.push(rad(-Math.PI / 3 + (i / (n - 1)) * 2 * Math.PI / 3));
    distances.push(MAX_SENSOR_DIST);
  }
  const rawDistances = [...distances];
  return { angles, distances, rawDistances, confidences: new Array(61).fill(1), timestamp: 0 };
}

function makeBlockedLidar(blockedAngles: number[], blockedDist: number, arcHalf = 0.05): LidarScan {
  const n = 61;
  const angles: any[] = [];
  const distances: any[] = [];
  for (let i = 0; i < n; i++) {
    const a = -Math.PI / 3 + (i / (n - 1)) * 2 * Math.PI / 3;
    angles.push(rad(a));
    const blocked = blockedAngles.some(ba => Math.abs(a - ba) < arcHalf);
    distances.push(blocked ? px(blockedDist) : MAX_SENSOR_DIST);
  }
  const rawDistances = [...distances];
  return { angles, distances, rawDistances, confidences: new Array(61).fill(1), timestamp: 0 };
}

// ═══════════════════════════════════════════════════
// 巡航 + 避障
// ═══════════════════════════════════════════════════

describe('planner 巡航', () => {
  it('全清 LiDAR → 正常轨迹', () => {
    const car = makeCar(100, 0, 0, 120);
    const traj = plan(makeSnapshot(car), makeClearLidar());
    assert.ok(traj.points.length > 0);
  });

  it('前方 75m 障碍（提交窗口外）→ 选路与无障碍一致：不提前选边（横向耐心）', () => {
    const car = makeCar(100, 0, 0, 120);
    plan(makeSnapshot(car), makeClearLidar());
    const freeTargetL = debug.planning!.bestTargetL;
    plan(makeSnapshot(car), makeBlockedLidar([0], 300, 0.025));   // 单射线（现实尺寸）
    assert.strictEqual(debug.planning!.bestTargetL, freeTargetL);
    assert.strictEqual(debug.planning!.speedMode, 'cruise');
  });

  it('前方 50m 障碍（进入提交窗口）→ 选侧避让', () => {
    const car = makeCar(100, 0, 0, 120);
    const traj = plan(makeSnapshot(car), makeBlockedLidar([0], 200, 0.025));   // 单射线
    assert.ok(traj.type !== 'lane_keep');
    assert.ok(traj.type !== 'emergency');
  });

  it('前方 20m 障碍且未开始横移 → 物理上无法绕开，紧急制动（诚实语义）', () => {
    const car = makeCar(100, 0, 0, 120);
    const traj = plan(makeSnapshot(car), makeBlockedLidar([0], 80));
    assert.strictEqual(traj.type, 'emergency');
  });

  it('已停住（v=0）且前方封死 → 轨迹保持有限值（不产生 NaN，实测停车后崩溃根因）', () => {
    const car = makeCar(100, 0, 0, 0);   // 停住
    const traj = plan(makeSnapshot(car), makeBlockedLidar([0], 60));
    for (const p of traj.points) {
      assert.ok(Number.isFinite(p.x as number) && Number.isFinite(p.y as number) && Number.isFinite(p.v as number),
        `轨迹点应有限：(${p.x},${p.y},v=${p.v})`);
    }
  });
});

describe('planner 双阈值（紧间隙慢行）', () => {
  it('舒适层全阻塞但物理可过（且可达）→ 不停车，限速 8m/s 慢行', () => {
    // 与 paths.test 双阈值夹具一致：P1 压舒适阻塞、−0.625m 档硬阈值可过。
    // 车用 10m/s（可达性：紧段 ~34m 外，减速到 8m/s 只需 ~3m，可行）。
    const n = 61, angles: any[] = [], distances: any[] = [];
    for (let i = 0; i < n; i++) {
      const a = -Math.PI / 3 + (i / (n - 1)) * 2 * Math.PI / 3;
      angles.push(rad(a));
      distances.push(i === 31 ? px(143.3) : i === 33 ? px(191) : i === 28 ? px(214.8) : MAX_SENSOR_DIST);
    }
    const scan: LidarScan = { angles, distances, rawDistances: [...distances], confidences: new Array(61).fill(1), timestamp: 0 };
    const car = makeCar(100, 0, 0, 40);
    plan(makeSnapshot(car), scan);
    assert.strictEqual(debug.planning!.speedMode, 'cruise', '物理可过不应触发停车线');
    assert.strictEqual(debug.planning!.speedTarget, mps(8) as number, '应限速 8m/s');
  });

  it('物理可过但不可达（30m/s、紧段 34m 外）→ 停车线生效（不可达=真无解）', () => {
    const n = 61, angles: any[] = [], distances: any[] = [];
    for (let i = 0; i < n; i++) {
      const a = -Math.PI / 3 + (i / (n - 1)) * 2 * Math.PI / 3;
      angles.push(rad(a));
      distances.push(i === 31 ? px(143.3) : i === 33 ? px(191) : i === 28 ? px(214.8) : MAX_SENSOR_DIST);
    }
    const scan: LidarScan = { angles, distances, rawDistances: [...distances], confidences: new Array(61).fill(1), timestamp: 0 };
    const car = makeCar(100, 0, 0, 120);   // 30m/s：需 ~70m 减速距离 > 34m
    plan(makeSnapshot(car), scan);
    assert.notStrictEqual(debug.planning!.speedMode, 'cruise', '不可达应进入停车/紧急');
  });
});

describe('planner 速度目标（vCap 单值化）', () => {
  it('障碍 30m 内 → speedTarget ≤ 17.5m/s；50m → ≤ 25m/s；无障碍 → 35m/s', () => {
    // 车放在车道中心（l=+1.875m=y 7.5u）：本用例只测"障碍距离 → 巡航目标"这一条梯度，
    // 车道横向偏移会额外触发收敛迭代的横向机动上限（速度-几何耦合，另有用例覆盖）。
    const car = makeCar(100, m(1.875) as number, 0, 150);
    plan(makeSnapshot(car), makeBlockedLidar([0], 120));
    assert.ok(debug.planning!.speedTarget <= (mps(17.5) as number) + 1e-6,
      `30m 内应限 17.5，实际 ${debug.planning!.speedTarget / 4}`);
    plan(makeSnapshot(car), makeBlockedLidar([0], 200));
    assert.ok(debug.planning!.speedTarget <= (mps(25) as number) + 1e-6,
      `50m 应限 25，实际 ${debug.planning!.speedTarget / 4}`);
    plan(makeSnapshot(car), makeClearLidar());
    assert.strictEqual(debug.planning!.speedTarget, mps(35) as number);
  });
});

describe('planner 一致性', () => {
  it('相同输入 → 相同类型', () => {
    const car = makeCar(100, 0, 0, 120);
    const lidar = makeClearLidar();
    let firstType = '';
    for (let i = 0; i < 10; i++) {
      const traj = plan(makeSnapshot(car), lidar);
      if (i === 0) firstType = traj.type;
      assert.strictEqual(traj.type, firstType);
      assert.ok(traj.points.length > 0);
    }
  });

  it('30 帧内避障方向切换 ≤2 次', () => {
    const car = makeCar(100, 0, 0, 120);
    const lidar = makeBlockedLidar([0], 300, 0.025);   // 单射线
    let switches = 0;
    let lastType = '';
    for (let i = 0; i < 30; i++) {
      const traj = plan(makeSnapshot(car), lidar);
      if (lastType && traj.type !== lastType) switches++;
      lastType = traj.type;
    }
    assert.ok(switches <= 2, `实际切换 ${switches} 次`);
  });
});

describe('planner 世界/路线切换（模块级参考线缓存失效）', () => {
  /** 中心线 y = yOffset 的直线轨道快照（无路线） */
  function snapshotAt(yOffset: number): WorldSnapshot {
    const pts: any[] = [];
    for (let i = 0; i <= 200; i++) pts.push({ x: px(i * 10), y: px(yOffset) });
    return { car: makeCar(200, yOffset, 0, 100), track: { centerPoints: pts, innerBoundary: [], outerBoundary: [] }, obstacles: [], timestamp: 0 };
  }

  it('无路线：换轨道后闭环参考线随世界重建（不沿用旧地图）', () => {
    plan(snapshotAt(1000), makeClearLidar());
    assert.ok(Math.abs(debug.planning!.frenetL as number) < 10, '地图 A：车应贴中心线');
    plan(snapshotAt(400), makeClearLidar());
    assert.ok(Math.abs(debug.planning!.frenetL as number) < 10,
      `换地图后车仍应贴新中心线（实际 |L|=${Math.abs(debug.planning!.frenetL as number) / 4}m —— 若 ~150m 说明仍在用旧地图参考线）`);
  });

  it('有路线：换路线对象（索引相同、几何不同）→ 参考线重建', () => {
    const mkRoute = (yOffset: number): Route => {
      const pts: any[] = [];
      for (let i = 0; i <= 100; i++) pts.push({ x: px(i * 10), y: px(yOffset) });
      const ref = buildFrenetReference(pts, false);
      return { waypoints: ref.points, waypointIndices: [], startIndex: 0, goalIndex: 100, length: px(ref.totalS) };
    };
    plan(snapshotAt(1000), makeClearLidar(), mkRoute(1000));
    assert.ok(Math.abs(debug.planning!.frenetL as number) < 10, '路线 A：车应贴路线中心线');
    plan(snapshotAt(400), makeClearLidar(), mkRoute(400));
    assert.ok(Math.abs(debug.planning!.frenetL as number) < 10,
      `换路线后车仍应贴新路线（实际 |L|=${Math.abs(debug.planning!.frenetL as number) / 4}m —— 若 ~150m 说明路线索引相同时沿用了旧几何缓存）`);
  });
});

describe('planner 路线处理', () => {
  it('接近终点 → 到达检测', () => {
    const pts = [];
    for (let i = 0; i <= 20; i++) pts.push({ x: px(i * 5), y: px(0) });
    const ref = buildFrenetReference(pts, false);
    const route: Route = {
      waypoints: ref.points, waypointIndices: [],
      startIndex: 0, goalIndex: 20, length: px(ref.totalS),
    };
    const car = makeCar(90, 0, 0, 80);
    const traj = plan(makeSnapshot(car), makeClearLidar(), route);
    assert.strictEqual(traj.type, 'arrived');
  });

  it('路线中途 → 轨迹正常且未到达', () => {
    const pts = [];
    for (let i = 0; i <= 100; i++) pts.push({ x: px(i * 10), y: px(0) });
    const ref = buildFrenetReference(pts, false);
    const route: Route = {
      waypoints: ref.points, waypointIndices: [],
      startIndex: 0, goalIndex: 100, length: px(ref.totalS),
    };
    const car = makeCar(300, 0, 0, 180);
    const traj = plan(makeSnapshot(car), makeClearLidar(), route);
    assert.notStrictEqual(traj.type, 'arrived');
    assert.ok(traj.points.length > 0);
  });
});

describe('规划器：收敛迭代（path-speed iteration）', () => {
  /** 与 plan() 同口径重建路径层（契约断言用；prevTargetL 取 null 消除帧间耦合） */
  function pathLayerAt(snap: WorldSnapshot, scan: LidarScan, v0: number) {
    const ref = buildFrenetReference(snap.track.centerPoints, true);
    const fl = cartesianToFrenet(snap.car.x, snap.car.y, ref);
    const r = solvePathLayer({
      ref, l0: fl.l, dl0: dimless(0), s0: fl.s, v0,
      carX: snap.car.x as number, carY: snap.car.y as number, lidarScan: scan,
      carWidth: snap.car.width, collisionRadius: snap.car.collisionRadius,
      laneOffset: px(m(1.875)), prevTargetL: null,
    });
    return { fl, r };
  }

  it('静止车（v=0）→ 几何评估速度不低于慢行档（破"v=0 裕度最大"自锁）', () => {
    const car = makeCar(100, 0, 0, 0);
    plan(makeSnapshot(car), makeBlockedLidar([0], 100, 0.025));
    assert.ok((debug.planning!.vAssume ?? -1) >= (TIGHT_SPEED as number),
      `几何评估速度应 ≥ 慢行档，实际 ${(debug.planning!.vAssume as number) / 4}m/s`);
    assert.ok((debug.planning!.rounds ?? 0) >= 1, '应记录迭代轮数');
  });

  it('高速 + 大横移需求 → 速度目标被横向机动上限压低（路径几何反馈成速度）', () => {
    const car = makeCar(100, 0, 0, 140);              // 35 m/s
    plan(makeSnapshot(car), makeBlockedLidar([0], 200, 0.025));
    assert.ok((debug.planning!.speedTarget as number) < (mps(20) as number),
      `速度目标应被横向上限压低，实际 ${(debug.planning!.speedTarget as number) / 4}m/s`);
  });

  it('速度目标满足横向机动上限契约（speedTarget ≤ lateralSpeedCap）', () => {
    const snap = makeSnapshot(makeCar(100, 0, 0, 140));
    const scan = makeBlockedLidar([0], 200, 0.025);
    plan(snap, scan);
    const { fl, r } = pathLayerAt(snap, scan, debug.planning!.vAssume as number);
    const cap = lateralSpeedCap(Math.abs((r.best.targetL as number) - (fl.l as number)), r.best.develop);
    assert.ok((debug.planning!.speedTarget as number) <= cap + 4,
      `速度目标 ${(debug.planning!.speedTarget as number) / 4}m/s 应 ≤ 横向上限 ${(cap / 4).toFixed(1)}m/s`);
  });

  it('确定性：同输入两次规划 → 轮数/目标/速度目标一致；轮数 ∈ [1,3]', () => {
    const scan = makeBlockedLidar([0], 200, 0.025);
    // 新 snapshot = 新 track 对象 → 参考线与迭代/迟滞记忆重建，两次都从干净状态出发
    plan(makeSnapshot(makeCar(100, 0, 0, 120)), scan);
    const a = { r: debug.planning!.rounds, t: debug.planning!.bestTargetL, u: debug.planning!.speedTarget };
    plan(makeSnapshot(makeCar(100, 0, 0, 120)), scan);
    const b = { r: debug.planning!.rounds, t: debug.planning!.bestTargetL, u: debug.planning!.speedTarget };
    assert.deepStrictEqual(a, b);
    assert.ok((a.r ?? 0) >= 1 && (a.r ?? 0) <= 3, `轮数应 ∈ [1,3]，实际 ${a.r}`);
  });

  it('迭代计数与修正标记：求值次数 ≥2；横向上限改变假设 → revised=true', () => {
    const snap = makeSnapshot(makeCar(100, 0, 0, 140));            // 35 m/s + 大横移需求
    const scan = makeBlockedLidar([0], 200, 0.025);
    plan(snap, scan);
    assert.ok((debug.planning!.rounds ?? 0) >= 2, `求值次数应 ≥2（确认轮），实际 ${debug.planning!.rounds}`);
    assert.strictEqual(debug.planning!.revised, true, '横向上限压低了速度假设 → 应标"需修正"');
    const snap2 = makeSnapshot(makeCar(100, m(1.875) as number, 0, 120));   // 畅通、已在车道中心
    plan(snap2, makeClearLidar());
    assert.strictEqual(debug.planning!.revised, false, '自洽假设不应标"需修正"');
  });

  it('debug.planning.viz：自由带/瓶颈/候选齐备（渲染层数据源，零拷贝引用路径层）', () => {
    const snap = makeSnapshot(makeCar(100, 0, 0, 120));
    plan(snap, makeClearLidar());
    const d = debug.planning!;
    const viz = d.viz;
    assert.ok(viz, '应挂载 viz');
    assert.ok(viz!.bands.length > 0, '畅通时应有一条完整自由带');
    assert.strictEqual(viz!.candidates.length, d.paths.length, '候选数与 inspector 列表一致');
    assert.strictEqual(viz!.bestTargetL, d.bestTargetL);
    assert.ok(viz!.candidates.every(c => (c.points?.length ?? 0) >= 2), '候选带路径点（供画线）');
    // 可视化走廊用 2× 采样（1.5m）——带子边界更贴曲率，不改规划器自身 3m 网格
    assert.strictEqual(viz!.stepU, (PATH_HORIZON as number) / PATH_STEPS / 2, '可视化网格步长应为规划网格的一半');
    // 观测覆盖：与带子一一对应（畅通场景射线到量程 → 全观测，无观测极限线）
    assert.strictEqual(viz!.bandObserved.length, viz!.bands.length, '观测标记应与带子一一对应');
    assert.ok(viz!.bandObserved.every(x => x), '全清 LiDAR（射线到量程）→ 应全观测');
    assert.strictEqual(viz!.obsLimitSegments.length, 0, '无未观测区 → 不画观测极限线');
    for (const q of viz!.bands) for (const pt of q) assert.ok(Number.isFinite(pt.x) && Number.isFinite(pt.y));
    for (const seg of viz!.pinchSegments) for (const pt of seg) assert.ok(Number.isFinite(pt.x) && Number.isFinite(pt.y));
  });

  it('前方被截断（障碍挡住视线）→ 障碍之外的带子标为未观测，并给出观测极限线', () => {
    const snap = makeSnapshot(makeCar(100, 0, 0, 120));
    plan(snap, makeBlockedLidar([0], 60, 0.05));      // 单方向命中 15m（挡视线），其余到量程
    const viz = debug.planning!.viz!;
    assert.ok(viz.bandObserved.some(x => !x), '障碍之外应标未观测');
    assert.ok(viz.obsLimitSegments.length > 0, '应有观测极限横断线');
    for (const seg of viz.obsLimitSegments) for (const p of seg) assert.ok(Number.isFinite(p.x) && Number.isFinite(p.y));
  });

  it('性能烟测：30 帧平均单帧规划 <5ms（预热后稳态；超时只记录，不改变行为）', () => {
    const snap = makeSnapshot(makeCar(100, 0, 0, 120));
    const scan = makeBlockedLidar([0], 200, 0.025);
    for (let i = 0; i < 5; i++) plan(snap, scan);      // 预热（JIT 首帧 ~30ms，归档指标里会如实记录该尖峰）
    const t0 = Date.now();
    for (let i = 0; i < 30; i++) plan(snap, scan);
    const avg = (Date.now() - t0) / 30;
    // 上界放宽到 25ms：node --test 按文件并行，全量套件下本用例与其它文件争 CPU，
    // 绝对值不可靠（隔离运行时实测 ~0.5ms）。它只做"无病态爆炸"烟测——真正的
    // 15Hz 预算（p95 <5ms）由归档指标 planMs 在报告里跟踪（spec §5.2/§5.3）。
    assert.ok(avg < 25, `平均单帧规划耗时 ${avg.toFixed(2)}ms（隔离运行时约 0.5ms）`);
  });
});
