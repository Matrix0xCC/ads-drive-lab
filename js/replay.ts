// ==================== 浏览器回放控制器 ====================
//
// 【职责】读取 runs/<id>/ 归档，以 step（60Hz）为时钟驱动现有渲染层：
// 各记录类型取「t ≤ 当前时刻的最近一条」，构造与实时管线相同的输入，
// 调用 updateScene（复用 overlays/HUD，不写第二套渲染）。
//
// 【数据流】fetch(run.json/world.json/issues.json/frames.jsonl) →
// 内存索引 Map<step, FrameRecord[]> → update(realDt) 推进时钟 → _render。
//
// 【模式互斥】回放期间由 main.ts 的 rAF 循环调用 update()，不步进 engine；
// 关闭回放时 main.ts 恢复实时模式。

import * as THREE from 'three';
import { updateScene, resetSceneCache } from './env/renderer.js';
import { corridorVizForReplay } from './algo/plan/planner.js';
import { buildFrenetReference } from './algo/plan/frenet.js';
import type { PlanningViz } from './shared/debug.js';
import { debug } from './shared/debug.js';
import { FrameRecord, RunMeta, WorldFile, Issue } from './shared/types.js';
import { RunBundle } from './headless/bundle.js';
import { px, rad } from './shared/dimensions.js';

export interface ReplayState {
  active: boolean;
  currentStep: number;
  totalSteps: number;
  playing: boolean;
  speed: number;
  /** issues 的 firstFrame 列表（时间轴标记） */
  markers: number[];
  runId: string | null;
}

export class ReplayController {
  readonly state: ReplayState = {
    active: false, currentStep: 0, totalSteps: 0,
    playing: false, speed: 1, markers: [], runId: null,
  };
  onChange: (() => void) | null = null;

  private _meta: RunMeta | null = null;
  private _world: WorldFile | null = null;
  /** 视觉开关（由 UI 状态投影写入；回放也支持走廊/刻度，走廊用归档扫描重算） */
  showCorridor = false;
  showFrenetTicks = true;
  showLidar = true;
  private _vizRef: ReturnType<typeof buildFrenetReference> | null = null;
  private _issues: Issue[] = [];
  private _stepIndex: Map<number, FrameRecord[]> = new Map();
  private _lastRenderedStep = -1;

  constructor(private scene: THREE.Scene, private lidarLines: THREE.Line[]) {}

  /** 运行列表（runs/index.json，供回放运行选择框） */
  static async listRuns(): Promise<{
    id: string; scenario: string; map: string; seed: number;
    createdAt: string; verdict: string; high: number; medium: number;
  }[]> {
    try {
      // no-store：index.json 是可变资源（新增运行/清理归档都会变），
      // 默认缓存模式会在清理后仍返回旧列表（浏览器实测需整页刷新才恢复）
      const res = await fetch('runs/index.json', { cache: 'no-store' });
      if (!res.ok) return [];
      const idx = await res.json();
      return (idx.runs ?? []).map((r: any) => ({
        id: String(r.id),
        scenario: String(r.scenario ?? ''),
        map: String(r.map ?? ''),
        seed: Number(r.seed ?? 0),
        createdAt: String(r.createdAt ?? ''),
        verdict: String(r.verdict ?? ''),
        high: Number(r.issues?.high ?? 0),
        medium: Number(r.issues?.medium ?? 0),
      }));
    } catch { return []; }
  }

  /** 从服务器 runs/<id>/ 加载 */
  async load(runId: string): Promise<void> {
    const base = `runs/${encodeURIComponent(runId)}`;
    const meta = await (await fetch(`${base}/run.json`)).json();
    const world = await (await fetch(`${base}/world.json`)).json();
    const issues = await (await fetch(`${base}/issues.json`)).json();
    const text = await (await fetch(`${base}/frames.jsonl`)).text();
    const frames: FrameRecord[] = text.trim().split('\n').map(l => JSON.parse(l));
    this.applyBundle({ meta, world, issues, frames });
  }

  /** 导入本地归档包（.run.json，与服务器归档同构）直接回放 */
  loadBundle(bundle: RunBundle): void {
    this.applyBundle(bundle);
  }

  private applyBundle(src: { meta: RunMeta; world: WorldFile; issues: Issue[]; frames: FrameRecord[] }): void {
    this._meta = src.meta;
    this._world = src.world;
    const pts = (arr: [number, number][]) => arr.map(([x, y]) => ({ x: px(x), y: px(y) }));
    this._vizRef = src.world.route && src.world.route.waypoints.length >= 2
      ? buildFrenetReference(pts(src.world.route.waypoints), false)
      : buildFrenetReference(pts(src.world.track.centerPoints), src.world.track.closed ?? true);
    this._lastRenderedStep = -1;      // 强制重绘（开关切换后立即生效）
    this._issues = src.issues;

    this._stepIndex = new Map();
    let maxStep = 0;
    for (const r of src.frames) {
      const arr = this._stepIndex.get(r.step) ?? [];
      arr.push(r);
      this._stepIndex.set(r.step, arr);
      if (r.step > maxStep) maxStep = r.step;
    }

    this.state.active = true;
    this.state.runId = src.meta?.id ?? null;
    this.state.totalSteps = maxStep;
    this.state.currentStep = 0;
    this.state.markers = this._issues.map(i => i.firstFrame);
    this.state.playing = false;
    this._lastRenderedStep = -1;
    resetSceneCache(this.scene);   // 换运行 → 静态几何懒缓存重建
    this._render();
    this.onChange?.();
  }

  close(): void {
    this.state.active = false;
    this.state.playing = false;
    resetSceneCache(this.scene);   // 恢复实时模式时重建实时世界的几何
    this.onChange?.();
  }

  play(): void { this.state.playing = true; this.onChange?.(); }
  pause(): void { this.state.playing = false; this.onChange?.(); }
  toggle(): void { this.state.playing ? this.pause() : this.play(); }

  seekStep(n: number): void {
    this.state.currentStep = Math.max(0, Math.min(this.state.totalSteps, Math.round(n)));
    this.state.playing = false;
    this._render();
    this.onChange?.();
  }

  setSpeed(x: number): void { this.state.speed = x; this.onChange?.(); }

  /** 由 main.ts 的 rAF 循环每帧调用（仅回放激活时） */
  update(realDt: number): void {
    if (!this.state.playing) return;
    this.state.currentStep += realDt * this.state.speed * 60;
    if (this.state.currentStep >= this.state.totalSteps) {
      this.state.currentStep = this.state.totalSteps;
      this.state.playing = false;
    }
    this._render();
    this.onChange?.();
  }

  /** 取 ≤ atStep 的最近一条指定类型记录（回溯 3s 覆盖低频记录） */
  private _lastOf(type: string, atStep: number): any {
    for (let s = atStep; s >= Math.max(0, atStep - 180); s--) {
      for (const r of this._stepIndex.get(s) ?? []) {
        if (r.type === type) return r;
      }
    }
    return null;
  }

  /** 请求重绘当前帧（暂停中切换开关时也要立即生效） */
  requestRedraw(): void { this._lastRenderedStep = -1; }

  private _render(): void {
    const world = this._world;
    if (!world || !this._meta) return;
    const step = Math.floor(this.state.currentStep);
    if (step === this._lastRenderedStep) return;
    this._lastRenderedStep = step;

    const stepRec = this._lastOf('step', step);
    if (!stepRec) return;
    const planRec = this._lastOf('plan', step);
    const percRec = this._lastOf('perception', step);
    const gpsRec = this._lastOf('gps', step);

    const toPts = (a: [number, number][]) => a.map(([x, y]) => ({ x: px(x), y: px(y) }));
    const snapshot = {
      car: {
        ...stepRec.car,
        // 渲染只读 x/y/angle；其余字段补零类型占位
        maxSpeed: 0, acceleration: 0, friction: 0, width: 0, length: 0, collisionRadius: 0,
      },
      track: {
        centerPoints: toPts(world.track.centerPoints),
        innerBoundary: toPts(world.track.innerBoundary),
        outerBoundary: toPts(world.track.outerBoundary),
      },
      obstacles: world.obstacles,
      timestamp: 0,
    };

    const lidarScan = percRec ? {
      angles: percRec.scan.angles.map((a: number) => rad(a)),
      distances: percRec.scan.distances.map((d: number) => px(d)),
      rawDistances: percRec.scan.rawDistances.map((d: number) => px(d)),
      confidences: percRec.scan.confidences,
      timestamp: 0,
    } : null;

    const trajectory = planRec ? {
      points: planRec.trajectory.points.map((p: any) => ({
        t: p.t, x: px(p.x), y: px(p.y), yaw: rad(0), v: px(p.v), kappa: 0,
      })),
      cost: planRec.trajectory.cost,
      type: planRec.trajectory.type,
      timestamp: 0,
    } : null;

    const route = world.route ? {
      waypoints: toPts(world.route.waypoints),
      waypointIndices: [],
      startIndex: world.route.startIndex,
      goalIndex: world.route.goalIndex,
      length: px(world.route.length),
    } : null;

    const gps = gpsRec
      ? { x: px(gpsRec.x), y: px(gpsRec.y), uncertainty: px(5), timestamp: 0 }
      : null;

    // 走廊可视化在回放里没有现成数据（回放不跑规划器）→ 用该帧归档的扫描 + 车位姿重算
    let corridorViz: PlanningViz | null = null;
    if (this.showCorridor && lidarScan && this._vizRef) {
      corridorViz = corridorVizForReplay(
        this._vizRef, lidarScan, stepRec.car.x, stepRec.car.y,
        world.route ? px(world.route.length) : undefined,
        world.track.laneOffset,          // 名义车道也来自归档（地图语义）
      );
    }

    // 面板对齐：回放没有实时规划数据，调试面板原来显示的是"实时残留"。**每渲染帧**都写回
    // debug.planning（不能只在开走廊时写——那样面板就跟着走廊开关才刷新）。
    debug.planning = replayPlanningDebug(planRec, corridorViz, world.route?.length ?? null);

    updateScene(
      this.scene, snapshot as any, this.showLidar ? lidarScan : null,
      planRec?.state ?? 'NONE', this.lidarLines, trajectory, route as any, gps as any,
      corridorViz, this.showFrenetTicks,
    );
  }
}


/**
 * 回放帧 → 调试面板数据（纯函数，便于单测）。
 *   - 基础字段取归档 plan 记录（状态/frenet/速度目标/轨迹类型）；
 *   - 候选列表：**开了走廊重算**时用重算结果（编号与画布 L1..Ln 同序）；
 *     否则退回归档记录的候选（collision 标记即 blocked）；
 *   - 实时专有字段（迭代轮数/耗时/v假设/紧间隙）不填 → 面板自动隐藏该行。
 */
export function replayPlanningDebug(
  planRec: any, corridorViz: PlanningViz | null, routeLenU: number | null,
): NonNullable<typeof debug.planning> {
  const archCands: any[] = planRec?.candidates ?? [];
  const archBest = planRec?.best;
  const vizCands = corridorViz?.candidates ?? null;
  const paths = vizCands
    ? vizCands.map(c => ({ targetL: c.targetL, blocked: c.blocked, cost: c.cost }))
    : archCands.map(c => ({ targetL: c.targetL, blocked: !!c.collision, cost: c.cost }));
  const s = (planRec?.frenet?.s ?? 0) as number;
  return {
    source: 'replay',
    state: planRec?.state ?? 'NONE',
    frenetS: s, frenetL: planRec?.frenet?.l ?? 0,
    dl_ds: planRec?.frenet?.dl_ds ?? 0, headingDiff: planRec?.frenet?.headingDiff ?? 0,
    paths,
    bestCost: vizCands
      ? (vizCands.find(c => c.targetL === corridorViz!.bestTargetL)?.cost ?? 0)
      : (archBest?.cost ?? 0),
    bestTargetL: vizCands ? corridorViz!.bestTargetL : (archBest?.targetL ?? 0),
    speedTarget: archBest?.targetV ?? 0,
    speedMode: planRec?.trajectory?.type ?? '—',
    remainingDist: routeLenU === null ? 0 : Math.max(0, routeLenU - s),
    viz: corridorViz,
  };
}
