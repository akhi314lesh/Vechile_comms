import * as THREE from 'three';
import { smokeTex } from './env.js';
import { surfaceAt, T, latHeight } from './track.js';

/* ================================================================
   VEHICLE SIM — dynamic bicycle model.
   Frames: heading θ, forward = (-sinθ, -cosθ), right = (cosθ, -sinθ).
   Body-frame velocities (vx forward, vy right), yaw ω positive = LEFT.
     v̇x = Fx/m - vy·ω      v̇y = Fy/m + vx·ω      ω̇ = (b·Fyr - a·Fyf·cosδ)/Iz
   Tires: Pacejka-style curve, friction-circle combined slip, load
   transfer, ABS / TC / ESC (each disableable → fully raw manual),
   wheelspin drifts, locked-wheel plows, handbrake, auto gearbox.
   ================================================================ */
const P = a => Math.sin(1.35 * Math.atan(8.5 * a));          // tire curve, peak ≈ 7.5°
const GEARS = [3.6, 2.1, 1.4, 1.0, 0.8];
const clamp = THREE.MathUtils.clamp;

export class VehicleSim {
  constructor() {
    this.pos = new THREE.Vector3();
    this.heading = 0;
    this.vx = 0; this.vy = 0; this.omega = 0;
    this.steer = 0;
    this.gear = 1; this.rpm = 900; this.reverse = false; this.revT = 0;
    this.inThrottle = 0; this.inBrake = 0; this.inSteer = 0; this.inHand = false;
    this.m = 1350; this.Iz = 2100;
    this.a = 1.32; this.b = 1.55; this.L = 2.87; this.h = 0.52;
    this.wheelR = 0.32;
    this.aids = { esc: true, abs: true, tc: true };
    this.mu = 1.05; this.s = 0; this.lat = 0; this.off = false;
    this.latAcc = 0; this.lonAcc = 0; this.lonAccS = 0;
    this.escActive = false; this.absActive = false; this.tcActive = false;
    this.spin = 0; this.lockF = false; this.lockR = false;
    this.slipF = 0; this.slipR = 0;
    this.impact = 0; this.scrape = 0; this.groundY = 0;
  }
  place(x, z, heading) {
    this.pos.set(x, 0, z);
    this.heading = heading;
    this.vx = this.vy = this.omega = 0;
    this.gear = 1; this.rpm = 900; this.reverse = false;
  }
  get vAbs() { return Math.hypot(this.vx, this.vy); }
  get fw() { return { x: -Math.sin(this.heading), z: -Math.cos(this.heading) }; }
  get rt() { return { x: Math.cos(this.heading), z: -Math.sin(this.heading) }; }

  step(dt) {
    const S = this;
    /* steering: rate-limited, full lock available at any speed */
    const tgt = clamp(S.inSteer, -1, 1) * 0.58;
    S.steer += clamp(tgt - S.steer, -3.4 * dt, 3.4 * dt);

    const surf = surfaceAt(S.pos.x, S.pos.z);
    S.mu = surf.mu; S.s = surf.s; S.lat = surf.lat; S.off = surf.off;
    const vAbs0 = S.vAbs;

    /* reverse engagement: hold brake at a stop */
    if (!S.reverse && vAbs0 < 0.6 && S.inBrake > 0.5) S.revT += dt; else S.revT = 0;
    if (S.revT > 0.35) { S.reverse = true; S.revT = 0; }
    if (S.reverse && (S.inThrottle > 0.3 || S.vx > 1.2)) S.reverse = false;
    const gasIn = S.reverse ? S.inBrake : S.inThrottle;
    const brkIn = S.reverse ? S.inThrottle : S.inBrake;

    /* drivetrain */
    const wheelRps = Math.abs(S.vx) / (2 * Math.PI * S.wheelR);
    let rpm = wheelRps * 60 * GEARS[S.gear - 1] * 3.9;
    if (!S.reverse) {
      if (rpm > 6000 && S.gear < 5) S.gear++;
      else if (rpm < 2300 && S.gear > 1) S.gear--;
      rpm = wheelRps * 60 * GEARS[S.gear - 1] * 3.9;
    }
    S.rpm += (clamp(Math.max(rpm, gasIn > 0.1 ? 1600 : 900), 900, 6600) - S.rpm) * Math.min(1, dt * 6);
    const Tq = (S.reverse ? 150 : 235 * (0.6 + 0.4 * Math.sin(Math.PI * clamp((S.rpm - 800) / 5200, 0, 1)))) * gasIn;
    let Fdrive = Tq * GEARS[S.gear - 1] * (S.reverse ? 3.2 : 3.9) * 0.85 / S.wheelR * (S.reverse ? -1 : 1);
    if (S.reverse && S.vx < -6) Fdrive = 0;
    if (gasIn < 0.05 && vAbs0 > 1) Fdrive = -Math.min(500, 60 * vAbs0) * (S.reverse ? -1 : 1);

    /* axle loads with longitudinal weight transfer */
    const g = 9.81;
    const Fzf = Math.max(600, S.m * g * S.b / S.L - S.m * S.lonAccS * S.h / S.L);
    const Fzr = Math.max(600, S.m * g * S.a / S.L + S.m * S.lonAccS * S.h / S.L);

    /* brakes + wheel lock states */
    const MAXB = S.m * 1.15 * g;
    let FbF = brkIn * MAXB * 0.62, FbR = brkIn * MAXB * 0.38;
    S.lockF = !S.aids.abs && brkIn > 0.5 && vAbs0 > 3;
    S.lockR = (S.inHand && vAbs0 > 1.5) || (!S.aids.abs && brkIn > 0.8 && vAbs0 > 5);
    S.absActive = false;
    if (S.aids.abs && brkIn > 0.1 && vAbs0 > 2) {
      const cap = S.mu * (Fzf + Fzr);
      if (FbF + FbR > cap * 0.93) {
        const k = cap * 0.93 / (FbF + FbR);
        FbF *= k; FbR *= k;
        S.absActive = true;
      }
    }

    /* axle velocities → slip angles */
    const vfx = S.vx, vfy = S.vy - S.a * S.omega;
    const vrx = S.vx, vry = S.vy + S.b * S.omega;
    const vden = Math.abs(S.vx) + 0.5;
    const af = Math.atan2(vfy, vden) - S.steer;
    const ar = Math.atan2(vry, vden);
    S.slipF = Math.abs(af); S.slipR = Math.abs(ar);

    let FxF = 0, FxR = 0, Fyf = 0, Fyr = 0, spin = 0;
    if (S.lockF) {                                     // locked front: no steering, slides straight
      const va = Math.hypot(vfx, vfy) || 1;
      FxF = -S.mu * Fzf * vfx / va; Fyf = -S.mu * Fzf * vfy / va;
    } else {
      FxF = -Math.sign(S.vx || 1) * Math.min(FbF, vAbs0 * 400);
      const capF = S.mu * Fzf;
      const latCapF = Math.sqrt(Math.max(capF * capF * 0.04, capF * capF - FxF * FxF));
      Fyf = -latCapF * P(af);
    }
    if (S.lockR) {                                     // handbrake / rear lock: drift entry
      const va = Math.hypot(vrx, vry) || 1;
      FxR = -S.mu * Fzr * vrx / va; Fyr = -S.mu * Fzr * vry / va;
    } else {
      const capR = S.mu * Fzr;
      if (Fdrive > capR) {                             // traction limit
        if (S.aids.tc) { Fdrive = capR * 0.96; S.tcActive = true; }
        else { spin = Fdrive / capR - 1; Fdrive = capR * 0.9; }
      }
      S.spin = spin;
      FxR = Fdrive - Math.sign(S.vx || 1) * Math.min(FbR, vAbs0 * 300);
      let latCapR = Math.sqrt(Math.max(capR * capR * 0.05, capR * capR - Math.min(FxR * FxR, capR * capR)));
      latCapR *= 1 - 0.55 * Math.min(spin, 1);         // wheelspin destabilizes the rear
      Fyr = -latCapR * P(ar);
    }

    /* ESC: yaw-rate reference vs actual → corrective moment + throttle cut */
    let Mesc = 0;
    S.escActive = false;
    if (S.aids.esc && vAbs0 > 4) {
      const wRef = -S.vx * Math.tan(S.steer) / S.L;
      const err = S.omega - wRef;
      if (Math.abs(err) > 0.1) {
        Mesc = clamp(-err * 4200, -3600, 3600);
        S.escActive = true;
        if (S.inThrottle > 0.3 && Math.abs(err) > 0.25) FxR *= 0.6;
      }
    }

    /* resistances */
    const Fdrag = 0.42;
    const Frr = surf.off ? 1100 : 190;
    const Fx = FxF * Math.cos(S.steer) + FxR - Math.sign(S.vx || 0) * (vAbs0 > 0.2 ? Frr : 0) - Fdrag * vAbs0 * S.vx;
    const Fy = Fyf * Math.cos(S.steer) + Fyr - Fdrag * vAbs0 * S.vy;
    const Mz = S.b * Fyr - S.a * Fyf * Math.cos(S.steer) + Mesc;

    /* integrate */
    S.vx += (Fx / S.m - S.vy * S.omega) * dt;
    S.vy += (Fy / S.m + S.vx * S.omega) * dt;
    S.omega += (Mz / S.Iz) * dt;
    S.omega *= Math.max(0, 1 - 0.55 * dt);

    if (vAbs0 < 3) {                                   // low-speed kinematic blend (parking)
      const wk = S.vx * Math.tan(S.steer) / S.L;
      S.omega += (wk - S.omega) * (1 - vAbs0 / 3) * Math.min(1, dt * 10);
      S.vy *= Math.max(0, 1 - dt * (3 - vAbs0) * 2);
    }
    if (S.vAbs < 0.25 && gasIn < 0.05 && brkIn < 0.05) { S.vx *= 0.5; S.vy *= 0.5; S.omega *= 0.4; }

    S.heading += S.omega * dt;
    const sn = Math.sin(S.heading), cs = Math.cos(S.heading);
    S.pos.x += (S.vx * (-sn) + S.vy * cs) * dt;
    S.pos.z += (S.vx * (-cs) + S.vy * (-sn)) * dt;

    S.latAcc = Fy / S.m / g;
    S.lonAcc = Fx / S.m / g;
    S.lonAccS += (S.lonAcc - S.lonAccS) * Math.min(1, dt * 8);

    /* ground height (berm ride when off the deck) */
    const gy = latHeight(S.lat);
    S.groundY += (gy - S.groundY) * Math.min(1, dt * 6);

    collideTrack(S);
  }

  /* world-space per-wheel contact points (for skid marks & smoke) */
  wheelContacts() {
    const S = this, cs = Math.cos(S.heading), sn = Math.sin(S.heading);
    const out = [];
    for (const [lx, lz] of [[-0.8, -1.46], [0.8, -1.46], [-0.8, 1.46], [0.8, 1.46]]) {
      out.push({
        x: S.pos.x + lx * cs + lz * sn,
        z: S.pos.z - lx * sn + lz * cs,
        front: lz < 0, left: lx < 0,
        y: S.groundY + 0.02
      });
    }
    return out;
  }
}

/* wall / guardrail collision — the track's lat is authoritative */
export function collideTrack(S) {
  const t = T;
  if (!t || !isFinite(S.lat)) return;
  const isWall = t.bar && t.inWin(S.s) && Math.sign(S.lat) === t.bar.side;
  const limit = isWall ? t.deckHalf - 1.0 : t.deckHalf + 0.18 - 0.55;
  if (Math.abs(S.lat) <= limit) return;
  const sgn = Math.sign(S.lat);
  const rx = -surfaceTx(S), rz = surfaceTz(S);
  const over = Math.abs(S.lat) - limit;
  S.pos.x -= rx * sgn * over;
  S.pos.z -= rz * sgn * over;
  const sn = Math.sin(S.heading), cs = Math.cos(S.heading);
  let wvx = S.vx * (-sn) + S.vy * cs, wvz = S.vx * (-cs) + S.vy * (-sn);
  const nx = rx * sgn, nz = rz * sgn;
  const vn = wvx * nx + wvz * nz;
  if (vn > 0) {
    const rest = isWall ? 0.25 : 0.35;
    wvx -= nx * vn * (1 + rest); wvz -= nz * vn * (1 + rest);
    const tx = -nz, tz = nx, vt = wvx * tx + wvz * tz;
    wvx -= tx * vt * 0.3; wvz -= tz * vt * 0.3;
    S.vx = wvx * (-sn) + wvz * (-cs);
    S.vy = wvx * cs + wvz * (-sn);
    S.omega *= 0.6;
    S.impact = Math.max(S.impact, vn);
    S.scrape = 0.35;
    S.scrapePos = { x: S.pos.x + nx * 1.0, z: S.pos.z + nz * 1.0 };
  }
}
let lastSurf = { tx: 1, tz: 0 };
function surfaceTx() { return lastSurf.tx; }
function surfaceTz() { return lastSurf.tz; }
export function cacheSurf(surf) { lastSurf = surf; }

/* ================================================================
   SKID MARKS — ring buffer of quads, per-vertex alpha fade
   ================================================================ */
export class SkidTrail {
  constructor(max = 1100) {
    this.max = max;
    this.pos = new Float32Array(max * 4 * 3);
    this.alpha = new Float32Array(max * 4);
    const idx = new Uint32Array(max * 6);
    for (let i = 0; i < max; i++) {
      const v = i * 4;
      idx.set([v, v + 1, v + 2, v, v + 2, v + 3], i * 6);
    }
    const geo = new THREE.BufferGeometry();
    this.posAttr = new THREE.BufferAttribute(this.pos, 3);
    this.aAttr = new THREE.BufferAttribute(this.alpha, 1);
    this.posAttr.setUsage(THREE.DynamicDrawUsage);
    this.aAttr.setUsage(THREE.DynamicDrawUsage);
    geo.setAttribute('position', this.posAttr);
    geo.setAttribute('aA', this.aAttr);
    geo.setIndex(new THREE.BufferAttribute(idx, 1));
    const mat = new THREE.ShaderMaterial({
      transparent: true, depthWrite: false,
      polygonOffset: true, polygonOffsetFactor: -2,
      vertexShader: `attribute float aA; varying float vA;
        void main(){ vA = aA; gl_Position = projectionMatrix * modelViewMatrix * vec4(position,1.0); }`,
      fragmentShader: `varying float vA;
        void main(){ gl_FragColor = vec4(0.02, 0.02, 0.025, vA * 0.7); }`
    });
    this.mesh = new THREE.Mesh(geo, mat);
    this.mesh.frustumCulled = false;
    this.mesh.renderOrder = 2;
    this.head = 0;
    this.last = new Map();
  }
  wheel(id, x, y, z, slipping, strength = 1, width = 0.22) {
    const l = this.last.get(id);
    if (!slipping) { this.last.set(id, null); return; }
    if (!l) { this.last.set(id, { x, y, z }); return; }
    const d = Math.hypot(x - l.x, z - l.z);
    if (d < 0.35 || d > 4) { this.last.set(id, { x, y, z }); return; }
    const nx = -(z - l.z) / d * width, nz = (x - l.x) / d * width;
    const i = this.head;
    const P = this.pos, A = this.alpha;
    const pts = [[l.x + nx, l.y, l.z + nz], [l.x - nx, l.y, l.z - nz], [x - nx, y, z - nz], [x + nx, y, z + nz]];
    for (let k = 0; k < 4; k++) {
      P[(i * 4 + k) * 3] = pts[k][0]; P[(i * 4 + k) * 3 + 1] = pts[k][1]; P[(i * 4 + k) * 3 + 2] = pts[k][2];
      A[i * 4 + k] = strength;
    }
    this.head = (this.head + 1) % this.max;
    this.last.set(id, { x, y, z });
    this.posAttr.needsUpdate = true;
    this.aAttr.needsUpdate = true;
  }
  update(dt) {
    const A = this.alpha;
    let any = false;
    for (let i = 0; i < A.length; i++) if (A[i] > 0) { A[i] = Math.max(0, A[i] - dt * 0.05); any = true; }
    if (any) this.aAttr.needsUpdate = true;
  }
  reset() {
    this.alpha.fill(0);
    this.aAttr.needsUpdate = true;
    this.last.clear();
  }
}

/* ================================================================
   SMOKE — GPU points pool, grows and fades
   ================================================================ */
export class Smoke {
  constructor(max = 260) {
    this.max = max;
    this.vel = new Float32Array(max * 3);
    this.life = new Float32Array(max);
    this.size = new Float32Array(max);
    const pos = new Float32Array(max * 3);
    const aLife = new Float32Array(max);
    const aSize = new Float32Array(max);
    const geo = new THREE.BufferGeometry();
    this.posAttr = new THREE.BufferAttribute(pos, 3);
    this.lifeAttr = new THREE.BufferAttribute(aLife, 1);
    this.sizeAttr = new THREE.BufferAttribute(aSize, 1);
    this.posAttr.setUsage(THREE.DynamicDrawUsage);
    this.lifeAttr.setUsage(THREE.DynamicDrawUsage);
    this.sizeAttr.setUsage(THREE.DynamicDrawUsage);
    geo.setAttribute('position', this.posAttr);
    geo.setAttribute('aLife', this.lifeAttr);
    geo.setAttribute('aSize', this.sizeAttr);
    const mat = new THREE.ShaderMaterial({
      transparent: true, depthWrite: false,
      uniforms: { tex: { value: smokeTex } },
      vertexShader: `attribute float aLife; attribute float aSize; varying float vL;
        void main(){ vL = aLife; vec4 mv = modelViewMatrix * vec4(position, 1.0);
          gl_PointSize = aSize * (170.0 / max(1.0, -mv.z)); gl_Position = projectionMatrix * mv; }`,
      fragmentShader: `uniform sampler2D tex; varying float vL;
        void main(){ vec4 t = texture2D(tex, gl_PointCoord);
          gl_FragColor = vec4(vec3(0.5, 0.53, 0.58), t.a * vL * 0.30); }`
    });
    this.points = new THREE.Points(geo, mat);
    this.points.frustumCulled = false;
    this.head = 0;
  }
  spawn(x, y, z, strength = 1) {
    const i = this.head;
    this.head = (this.head + 1) % this.max;
    this.posAttr.setXYZ(i, x, y, z);
    this.vel[i * 3] = (Math.random() - 0.5) * 1.2;
    this.vel[i * 3 + 1] = 0.8 + Math.random() * 1.4;
    this.vel[i * 3 + 2] = (Math.random() - 0.5) * 1.2;
    this.life[i] = 1;
    this.size[i] = (1.2 + Math.random() * 1.4) * strength;
    this.lifeAttr.needsUpdate = this.sizeAttr.needsUpdate = this.posAttr.needsUpdate = true;
  }
  update(dt) {
    const P = this.posAttr.array, V = this.vel, L = this.life;
    let any = false;
    for (let i = 0; i < this.max; i++) {
      if (L[i] <= 0) continue;
      L[i] -= dt * 0.55;
      P[i * 3] += V[i * 3] * dt; P[i * 3 + 1] += V[i * 3 + 1] * dt; P[i * 3 + 2] += V[i * 3 + 2] * dt;
      this.size[i] += dt * 3.5;
      any = true;
    }
    if (any) { this.posAttr.needsUpdate = this.lifeAttr.needsUpdate = this.sizeAttr.needsUpdate = true; }
    this.lifeAttr.array === L; // noop keep
  }
  reset() { this.life.fill(0); this.lifeAttr.needsUpdate = true; }
}