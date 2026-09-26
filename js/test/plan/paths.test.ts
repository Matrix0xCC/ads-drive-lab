// ==================== 测试：路径层（SL 空间） ====================
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { solvePathLayer, PATH_HORIZON, PATH_DEVELOP, TIGHT_SPEED, lateralSpeedCap, DOMAIN_OVERSHOOT_RESERVE } from '../../algo/plan/paths.js';
import { marginFor } from '../../algo/plan/margin.js';
import { buildFrenetReference } from '../../algo/plan/frenet.js';
import { CarState, LidarScan } from '../../shared/types.js';
import { Px, rad, m, mps, mps2, px, pxps, dimless } from '../../shared/dimensions.js';
import { MAX_SENSOR_DIST } from '../../algo/perception.js';
import { TRACK_HALF_WIDTH } from '../../env/track.js';
import { BOUNDS_BUFFER } from '../../env/car.js';

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
    // 规划域 = TRACK_HALF_WIDTH − (车半宽 + 复位缓冲 BOUNDS_BUFFER) − 0.6m 过冲预留
    const lMaxU = (TRACK_HALF_WIDTH as number) - ((m(1.0) as number) + (BOUNDS_BUFFER as number)) - (DOMAIN_OVERSHOOT_RESERVE as number);
    for (const p of r.ranked) {
      assert.ok(Math.abs(p.targetL as number) <= lMaxU + 1e-6, `越界 targetL=${p.targetL}（域 ±${lMaxU}）`);
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
    // 障碍在 l=0（道路中心）把瓶颈处切成左右两段自由区，候选 = 段中心（±3.66m）。
    // 旧的"侧移 >2m"档位断言随候选口径（固定档位 → 间隙中心）失效：段中心距车道中心
    // 仅 1.78m，但已在障碍膨胀边（l=0 ± 1.65m）之外。要判的是"最优落在自由段内部
    // （两侧留内缩余量），既非车道中心、也非贴边档"。
    const bestL = r.best.targetL as number;
    const seg = r.corridor.pinch!.free.find(sg => bestL >= sg.lmin && bestL <= sg.lmax);
    assert.ok(seg, `最优应落在瓶颈自由段内，实际 ${bestL / 4}m`);
    assert.ok(bestL - seg.lmin > (m(0.3) as number) && seg.lmax - bestL > (m(0.3) as number),
      `最优应为间隙中心（两侧 ≥0.3m 内缩余量），实际 ${bestL / 4}m（自由段 [${seg.lmin / 4}, ${seg.lmax / 4}]m）`);
    assert.ok(bestL > (LO as number), `应选在障碍右侧的自由段，实际 ${bestL / 4}m`);
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

  it('窗口内障碍（200u=50m）→ 本车道判阻塞，最优选侧（间隙中心）', () => {
    // 候选取自走廊瓶颈处自由段（间隙中心 + 边界内缩），不再是从固定档位里挑——
    // 窄障碍的避让幅度只比间隙边界多出"内缩前的中心"，故"侧移 >2m"这类旧断言
    // 不再成立；要判的是"最优不在本车道、且舒适畅通"。
    const r = solve(blockedScan(200, 0.025));
    const inLane = r.ranked.find(p => Math.abs((p.targetL as number) - LO) < 1)!;
    assert.ok(inLane && inLane.blocked, '窗口内应判阻塞');
    assert.strictEqual(r.allBlocked, false, '存在舒适畅通候选');
    assert.strictEqual(r.best.blocked, false, '最优应舒适畅通');
    assert.ok((r.best.targetL as number) > (inLane.targetL as number), '最优应位于本车道右侧（间隙中心）');
    assert.strictEqual(r.sBlockHard, Infinity, '物理层畅通 → 停车线不触发');
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
  it('障碍落在默认发展段内 → 缩短发展段提前完成横移，左侧候选恢复可过', () => {
    // 车在 l=+3.5m 偏右；前方 25m 有横跨 l≈±0.9m 的障碍链（命中链段）。
    // 默认 40m 发展段：路径到障碍处才横移约 1/3 → 连硬阈值也过不去 → 停车线
    // （hairpin-s7 实测停车根因）。自适应后发展段缩到 ~18m，到障碍前已到位。
    // 反事实核验（临时禁用自适应分支重跑）：左侧候选 sBlockHard=96（硬阻塞）——
    // 自适应分支是可过性的必要条件，本测试确实卡在该机制上。
    const n = 61, angles: any[] = [], distances: any[] = [];
    for (let i = 0; i < n; i++) {
      const a = -Math.PI / 3 + (i / (n - 1)) * 2 * Math.PI / 3;
      angles.push(rad(a));
      distances.push((i === 29 || i === 30 || i === 31) ? px(100) : MAX_SENSOR_DIST);
    }
    const scan: LidarScan = { angles, distances, rawDistances: [...distances], confidences: new Array(61).fill(1), timestamp: 0 };
    const r = solve(scan, { l0: 14, speed: 20 });   // 车在 l=+3.5m、低速 5m/s（停车后起步的情境：允许急横移）
    const leftmost = r.ranked.reduce((a, b) => ((b.targetL as number) < (a.targetL as number) ? b : a));
    assert.ok((leftmost.targetL as number) < -(m(3) as number), `应存在左侧走廊候选，最左 ${leftmost.targetL}`);
    assert.strictEqual(leftmost.blocked, false,
      `最左候选 ${(leftmost.targetL as number) / 4}m 应舒适通过（实际 comfort=${leftmost.blocked} hard=${leftmost.blockedHard}）`);
    assert.strictEqual(r.allBlockedHard, false, '物理层应有可过候选（不得触发停车线）');
    // 候选记录实际使用的发展段（自适应缩短后）——规划器按它反解横向机动可行速度
    assert.ok((leftmost.develop as number) > 0 && (leftmost.develop as number) < (PATH_DEVELOP as number),
      `发展段应被自适应缩短，实际 ${leftmost.develop}`);
  });
});

describe('路径层：双阈值（舒适选路 / 物理通行）', () => {
  it('全部候选低于舒适间隙、但存在物理可过候选 → allBlockedHard=false，最优为可过候选', () => {
    // 夹具（笛卡尔命中点，直线参考、车在 l0=0）：
    //   P1 右中 (143.2, 5.0)  —— 把车道/右侧候选压进舒适阻塞，但右侧间隙中心硬阈值可过
    //   P2 远右 (189.9, 20.0) —— 堵 +4.375/+4.4
    //   P3 左  (214.3, −15.0) —— 堵 −3.125/−4.4
    // 实测：最优 = 右侧间隙中心 +3.19m（舒适阻塞、物理可过）；左侧 −2.39/−4.10 与右岸 3.64/4.10 硬阻塞。
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
    assert.strictEqual(r.best.blocked, true, '最优为"紧间隙慢行"候选（舒适阻塞）');
    assert.strictEqual(r.best.blockedHard, false, '最优必须物理可过');
    assert.strictEqual(r.sBlockHard, Infinity, '存在可过候选 → 停车线不触发');
    // 分级互斥且分层：全部硬阻塞候选排在所有可过候选之后
    const firstHard = r.ranked.findIndex(c => c.blockedHard);
    assert.ok(firstHard > 0, '应存在硬阻塞候选且不在首位');
    for (let i = firstHard; i < r.ranked.length; i++) {
      assert.ok(r.ranked[i].blockedHard, `ranked[${i}] 应为硬阻塞（分级不得重复计入）`);
    }
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
    // 候选口径已改为"瓶颈自由段中心"：左侧自由段 = [走廊左边界 −6.0m, 障碍膨胀边 −1.31m]，
    // 段中心 ≈ −3.66m（不再是固定档位"车道中心−5m"）。贴边档（lmin+0.3m = −5.7m）距
    // 复位崖 |l|=6.0 只剩 0.3m 跟踪余量，故断言最优是段中心、而非贴边。
    const seg = r.corridor.pinch!.free.reduce((a, b) =>
      Math.abs((a.lmin + a.lmax) / 2 - (LO as number)) <= Math.abs((b.lmin + b.lmax) / 2 - (LO as number)) ? a : b);
    const mid = (seg.lmin + seg.lmax) / 2;
    assert.ok(Math.abs((r.best.targetL as number) - mid) < (m(0.05) as number),
      `最优应为间隙中心 ${mid / 4}m，实际 ${(r.best.targetL as number) / 4}m`);
    assert.ok(Math.abs((r.best.targetL as number) - (seg.lmin + (m(0.3) as number))) > (m(0.5) as number),
      '最优不应是贴边档（距复位崖仅 0.3m）');
  });
});

describe('路径层：横向机动速度上限（lateralSpeedCap，供收敛迭代反解）', () => {
  it('无横移需求 → Infinity；有横移 → 与 developMin 互为反解', () => {
    assert.strictEqual(lateralSpeedCap(0, 160), Infinity, '无横移 → 不设速度上限');
    // 与 paths.ts 的 developMin 同式：S_min = √(5.7735·|Δl|·v²/A_LAT_MAX)
    // 反解得 v = √(A_LAT_MAX·S²/(5.7735·|Δl|))。量纲核对（mps2(3.5)=14 u/s²，
    // 单位换算后恰等于 SI 的 A_LAT_MAX=3.5 m/s²）：|Δl|=22u=5.5m、S=160u=40m
    // → v = 53.12 u/s = 13.28 m/s，此时峰值横向加速度 5.7735·5.5·13.28²/40² = 3.5 ✓
    const cap = lateralSpeedCap(22, 160);
    assert.ok(Math.abs(cap - 53.12) < 0.05, `实际 ${cap.toFixed(2)} u/s`);
    // 横移越大 → 上限越低；发展段越长 → 上限越高
    assert.ok(lateralSpeedCap(44, 160) < cap, '横移加倍 → 上限下降');
    assert.ok(lateralSpeedCap(22, 320) > cap, '发展段加倍 → 上限上升');
  });
});

describe('路径层：物理走廊裕度（标定表 + 几何评估速度下限）', () => {
  it('裕度 = marginFor(max(v0, 慢行档))：静止车取慢行档 0.91m；高速取表值', () => {
    // 回归测试（评审 Critical）：曾把 TIGHT_SPEED=mps(8)=32 直接与 v0/4（m/s）比较 →
    // marginFor(32)=0.615m 恒定，标定表实际失效（8m/s 慢行档应为 0.91m）。
    const stop = solve(clearScan(), { speed: 0 });
    assert.ok(Math.abs((stop.marginU as number) / 4 - marginFor((TIGHT_SPEED as number) / 4)) < 1e-9,
      `静止车应取慢行档裕度 ${marginFor((TIGHT_SPEED as number) / 4).toFixed(3)}m，实际 ${((stop.marginU as number) / 4).toFixed(3)}m`);
    const fast = solve(clearScan(), { speed: 35 * 4 });
    assert.ok(Math.abs((fast.marginU as number) / 4 - marginFor(35)) < 1e-9,
      `35m/s 应取表值 ${marginFor(35).toFixed(3)}m，实际 ${((fast.marginU as number) / 4).toFixed(3)}m`);
    assert.ok((stop.marginU as number) > (fast.marginU as number), '低速裕度应大于高速（标定表随速度下降）');
  });
});
