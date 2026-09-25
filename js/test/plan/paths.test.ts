// ==================== 测试：路径层（SL 空间） ====================
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { solvePathLayer, PATH_HORIZON } from '../../algo/plan/paths.js';
import { buildFrenetReference } from '../../algo/plan/frenet.js';
import { CarState, LidarScan } from '../../shared/types.js';
import { Px, rad, m, mps, mps2, px, pxps, dimless } from '../../shared/dimensions.js';
import { MAX_SENSOR_DIST } from '../../algo/perception.js';

const LO = m(1.875) as number;

function straightRef() {
  const pts: any[] = [];
  for (let i = 0; i <= 600; i++) pts.push({ x: px(i * 10), y: px(0) });
  return buildFrenetReference(pts, true);
}
function makeCar(x: number, y: number, angle: number, speed: number): CarState {
  return { x: px(x), y: px(y), angle: rad(angle), speed: pxps(speed), steerAngle: rad(0),
    maxSpeed: mps(42.5), acceleration: mps2(8), friction: dimless(0.12),
    width: m(2.0), length: m(4.5), collisionRadius: px(Math.hypot(8, 18) / 2) };
}
function clearScan(): LidarScan {
  const n = 61, angles: any[] = [], distances: any[] = [];
  for (let i = 0; i < n; i++) { angles.push(rad(-Math.PI/3 + (i/(n-1))*2*Math.PI/3)); distances.push(MAX_SENSOR_DIST); }
  return { angles, distances, rawDistances: [...distances], confidences: new Array(61).fill(1), timestamp: 0 };
}
function blockedScan(dist: number, arcHalf = 0.05): LidarScan {
  const n = 61, angles: any[] = [], distances: any[] = [];
  for (let i = 0; i < n; i++) {
    const a = -Math.PI/3 + (i/(n-1))*2*Math.PI/3;
    angles.push(rad(a));
    distances.push(Math.abs(a) < arcHalf ? px(dist) : MAX_SENSOR_DIST);
  }
  return { angles, distances, rawDistances: [...distances], confidences: new Array(61).fill(1), timestamp: 0 };
}

function solve(scan: LidarScan, opts: { s0?: number; l0?: number; prev?: number | null; dl0?: number; speed?: number } = {}) {
  // 直线参考线从 x=0 起算、间距 10u：车与 s0 必须一致（车在参考线该弧长处）
  const car = makeCar(opts.s0 ?? 0, 0, 0, opts.speed ?? 100);
  return solvePathLayer({
    ref: straightRef(), l0: px(opts.l0 ?? 0), dl0: dimless(opts.dl0 ?? 0), s0: opts.s0 ?? 0,
    v0: car.speed as number,
    carX: car.x as number, carY: car.y as number, lidarScan: scan,
    carWidth: car.width as Px, collisionRadius: car.collisionRadius as Px,
    laneOffset: px(LO), prevTargetL: opts.prev ?? null,
  });
}

describe('路径层：候选生成与域约束', () => {
  it('全部候选在可行驶域内，采样覆盖整个视程', () => {
    const r = solve(clearScan());
    assert.ok(r.ranked.length >= 3);
    for (const p of r.ranked) {
      assert.ok(Math.abs(p.targetL as number) <= 17.6 + 1e-6, `越界 targetL=${p.targetL}`);
      assert.strictEqual(p.points.length, 41);   // PATH_STEPS + 1
      assert.ok((p.points[40].s - p.points[0].s) > (PATH_HORIZON as number) - 1);
    }
  });

  it('畅通：最优为车道中心路径（代价最低）', () => {
    const r = solve(clearScan());
    assert.strictEqual(r.allBlocked, false);
    assert.ok(Math.abs((r.best.targetL as number) - LO) < 0.5, `best=${r.best.targetL}`);
    assert.strictEqual(r.best.blocked, false);
  });
});

describe('路径层：碰撞判定与选路', () => {
  it('中央窄障碍（100u）：本车道阻塞，最优选侧向路径', () => {
    const r = solve(blockedScan(100, 0.025));
    assert.strictEqual(r.allBlocked, false);
    assert.strictEqual(r.best.blocked, false);
    assert.ok(Math.abs((r.best.targetL as number) - LO) > 2 * 4, '应选侧向路径'); // >2m
    const inLane = r.ranked.find(p => Math.abs((p.targetL as number) - LO) < 1);
    assert.ok(inLane && inLane.blocked, '本车道应被判阻塞');
    // 判阻塞发生在"路径进入命中点 CLEAR(2.5m) 范围内"处：sBlock ≈ 60-130
    assert.ok(inLane.sBlock >= 60 && inLane.sBlock <= 130, `sBlock 应≈60-130，实际 ${inLane.sBlock}`);
  });

  it('近距宽障碍（60u）全阻塞：allBlocked=true，按最晚碰撞排序', () => {
    const r = solve(blockedScan(60));
    assert.strictEqual(r.allBlocked, true);
    assert.strictEqual(r.best.blocked, true);
    const ord = r.ranked.map(p => p.sBlock);
    for (let i = 1; i < ord.length; i++) assert.ok(ord[i] <= ord[i - 1], '应按 sBlock 降序');
    assert.ok(r.sBlock === ord[0]);
    assert.strictEqual(r.allBlockedHard, true, '物理阈值下也全阻塞（真无解）');
    assert.strictEqual(r.sBlockHard, r.best.sBlockHard, '停车基准=最晚硬阻塞候选');
  });
});

describe('路径层：提交窗口（横向耐心）', () => {
  it('窗口外障碍（300u=75m）→ 本车道不判近距阻塞；停车线不触发', () => {
    const r = solve(blockedScan(300, 0.025));
    const inLane = r.ranked.find(p => Math.abs((p.targetL as number) - LO) < 1)!;
    assert.ok(inLane && !inLane.blocked, '窗口外不应判近距阻塞（不提前选边）');
    assert.strictEqual(r.allBlocked, false);
    assert.strictEqual(r.best.targetL, inLane.targetL, '近距全清 → 最优应为本车道');
    assert.strictEqual(r.allBlockedHard, false, '物理层畅通 → 停车线不触发');
    assert.strictEqual(r.sBlockHard, Infinity);
  });

  it('窗口内障碍（200u=50m）→ 本车道判阻塞，最优选侧', () => {
    const r = solve(blockedScan(200, 0.025));
    const inLane = r.ranked.find(p => Math.abs((p.targetL as number) - LO) < 1)!;
    assert.ok(inLane && inLane.blocked, '窗口内应判阻塞');
    assert.ok(Math.abs((r.best.targetL as number) - LO) > 8, '应选侧向路径');   // >2m
  });
});

describe('路径层：横向路径不越出 [l0, target]', () => {
  it('车贴左侧、航向斜率指向外侧（dl0<0）→ 路径不得越过 l0 继续左漂', () => {
    // 实测（hairpin-s5 批量）：车在 l≈−4.4 走廊边缘、航向角误差 dl0<0 时，五次
    // 多项式为满足初值斜率会先向 −l 侧下沉，逐帧重读 l0 形成棘轮 → 撞 |l|=5.0
    // 复位崖 → 复位到中心线正对障碍 → 连锁碰撞。路径必须夹在 [min(l0,target),
    // max(l0,target)] 内（斜率自然过渡，不允许越过端点）。
    const r = solve(clearScan(), { l0: -17, dl0: -0.1 });
    for (const p of r.ranked) {
      const lmin = Math.min(-17, p.targetL as number);
      const worst = Math.min(...p.points.map(pt => pt.l as number));
      assert.ok(worst >= lmin - 1e-6, `路径左漂越过端点：worst=${worst} < ${lmin} (target=${p.targetL})`);
    }
  });
});

describe('路径层：自适应发展段（adaptive development）', () => {
  it('障碍落在默认发展段内 → 缩短发展段提前完成横移，候选恢复可过', () => {
    // 车在 l=+3.5m 偏右；前方 25m 有横跨 l≈±0.9m 的障碍链（命中链段）。
    // 默认 40m 发展段：路径到障碍处才横移约 1/3（l≈−2.2m）→ 距链段仅 ~1.3m，
    // 硬阈值（HARD_CLEAR=1.4m）也过不去 → 停车线（hairpin-s7 实测停车根因）。
    // 自适应后：发展段缩到 ~16m，到障碍前已到位 (−4.4m) → 可通过。
    const n = 61, angles: any[] = [], distances: any[] = [];
    for (let i = 0; i < n; i++) {
      const a = -Math.PI / 3 + (i / (n - 1)) * 2 * Math.PI / 3;
      angles.push(rad(a));
      distances.push((i === 29 || i === 30 || i === 31) ? px(100) : MAX_SENSOR_DIST);
    }
    const scan: LidarScan = { angles, distances, rawDistances: [...distances], confidences: new Array(61).fill(1), timestamp: 0 };
    const r = solve(scan, { l0: 14, speed: 20 });   // 车在 l=+3.5m、低速 5m/s（停车后起步的情境：允许急横移）
    const left = r.ranked.find(p => Math.abs((p.targetL as number) + 17.6) < 0.5)!;
    assert.ok(left, '应有 −4.4m 走廊候选');
    assert.strictEqual(left.blocked, false,
      `−4.4 候选应通过（实际 comfort=${left.blocked} hard=${left.blockedHard}）`);
    assert.strictEqual(r.allBlockedHard, false, '物理层应有可过候选（不得触发停车线）');
  });
});

describe('路径层：双阈值（舒适选路 / 物理通行）', () => {
  it('全部候选低于舒适间隙、但存在物理可过候选 → allBlockedHard=false，最优为可过候选', () => {
    // 夹具（笛卡尔命中点，直线参考、车在 l0=0）：
    //   P1 右中 (143.2, 5.0)  —— 把车道/右侧候选压进舒适阻塞，但 −0.625m 档与它留 7.5u（>硬阈值 5.6u）
    //   P2 远右 (189.9, 20.0) —— 堵 +4.375/+4.4
    //   P3 左  (214.3, −15.0) —— 堵 −3.125/−4.4
    const n = 61, angles: any[] = [], distances: any[] = [];
    for (let i = 0; i < n; i++) {
      const a = -Math.PI / 3 + (i / (n - 1)) * 2 * Math.PI / 3;
      angles.push(rad(a));
      distances.push(i === 31 ? px(143.3) : i === 33 ? px(191) : i === 28 ? px(214.8) : MAX_SENSOR_DIST);
    }
    const scan: LidarScan = { angles, distances, rawDistances: [...distances], confidences: new Array(61).fill(1), timestamp: 0 };
    const r = solve(scan);
    assert.strictEqual(r.allBlocked, true, '舒适层：应全部候选阻塞');
    assert.strictEqual(r.allBlockedHard, false, '物理层：存在可通过候选（不停车）');
    assert.strictEqual(r.best.targetL, px(-2.5), '最优应为物理可过的 −0.625m 档');
    assert.strictEqual(r.best.blocked, true);
    assert.strictEqual(r.best.blockedHard, false);
    assert.ok(r.ranked[1].blockedHard, '硬阻塞候选应排在其后');
  });
});

describe('路径层：候选覆盖间隙', () => {
  it('障碍贴右、左侧留有中间间隙 → 最优为间隙内候选（非走廊边缘）', () => {
    // 链段从 l≈0 延到 l≈+5.2m（右车道+右侧）：左侧可通行区间 ≈ [−4.4, −1]，间隙中心 ≈ −3.1m。
    // 若候选只有走廊边缘 −4.4（距复位崖 |l|=5.0 仅 0.6m 跟踪余量），实际车会贴边
    // 漂移越界触发复位（oval-s2 批量实证）；间隙内候选（车道中心−5m）才是稳妥选择。
    const n = 61, angles: any[] = [], distances: any[] = [];
    for (let i = 0; i < n; i++) {
      const a = -Math.PI / 3 + (i / (n - 1)) * 2 * Math.PI / 3;
      angles.push(rad(a));
      distances.push((i >= 30 && i <= 33) ? px(200) : MAX_SENSOR_DIST);    // 相邻射线 → 命中链段（l≈0..+5.2m）
    }
    const scan: LidarScan = { angles, distances, rawDistances: [...distances], confidences: new Array(61).fill(1), timestamp: 0 };
    const r = solve(scan);
    assert.strictEqual(r.best.blocked, false);
    assert.ok(Math.abs((r.best.targetL as number) - (LO - 20)) < 2,
      `最优应在间隙内（车道中心−5m），实际 ${(r.best.targetL as number) / 4}m`);
  });
});
