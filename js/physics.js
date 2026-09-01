/**
 * js/physics.js — Vehicle dynamics (bicycle model), collision detection
 */
import { clamp, GRAV, PHYS_H } from './utils.js';

export function safeCurveSpeed(kappa, mu, load = 0.55) {
  const k = Math.abs(kappa);
  if (k < 1e-5) return 200;
  return Math.sqrt(mu * GRAV * load / k);
}

export function obbOverlap2D(A, B) {
  const mk = o => {
    const fx = -Math.sin(o.psi), fz = -Math.cos(o.psi);
    return { x: o.x, z: o.z, fx, fz, rx: -fz, rz: fx, hl: o.hl, hw: o.hw };
  };
  const a = mk(A), b = mk(B);
  const proj = (r, ux, uz) => {
    const c = r.x * ux + r.z * uz;
    const e = Math.abs(r.fx * ux + r.fz * uz) * r.hl + Math.abs(r.rx * ux + r.rz * uz) * r.hw;
    return { min: c - e, max: c + e };
  };
  for (const [ux, uz] of [[a.fx, a.fz], [a.rx, a.rz], [b.fx, b.fz], [b.rx, b.rz]]) {
    const pa = proj(a, ux, uz), pb = proj(b, ux, uz);
    if (pa.max < pb.min || pb.max < pa.min) return false;
  }
  return true;
}

export class VehicleDynamics {
  constructor(p = {}) {
    this.m = p.m ?? 1400;
    this.Iz = p.Iz ?? 2100;
    this.a = p.a ?? 1.18;
    this.b = p.b ?? 1.52;
    this.h = p.h ?? 0.55;
    this.L = this.a + this.b;
    this.Cf = p.Cf ?? 82000;
    this.Cr = p.Cr ?? 95000;
    this.P = p.P ?? 108000;
    this.Fmax = p.Fmax ?? 5600;
    this.cDrag = p.cDrag ?? 0.42;
    this.cRR = p.cRR ?? 220;
    this.brakeF = p.brakeF ?? 12500;
    this.steerMax = p.steerMax ?? 0.56;
    this.steerRate = p.steerRate ?? 3.4;
    this.x = 0; this.z = 0; this.psi = 0;
    this.u = 0; this.w = 0; this.om = 0;
    this.delta = 0;
    this.ax = 0; this.ay = 0;
    this.satF = this.satR = false;
    this.slipF = this.slipR = false;
    this.lockF = this.lockR = false;
    this.wheelSpin = false;
    this.escActive = false;
  }

  reset(x, z, psi) {
    this.x = x; this.z = z; this.psi = psi;
    this.u = this.w = this.om = this.delta = 0;
    this.ax = this.ay = 0;
  }

  forward() { return { x: -Math.sin(this.psi), z: -Math.cos(this.psi) }; }
  right() { return { x: Math.cos(this.psi), z: -Math.sin(this.psi) }; }
  get beta() { return Math.atan2(this.w, Math.max(Math.abs(this.u), 2)); }

  step(dt, ctrl, env) {
    const dLim = Math.min(this.steerMax, 0.16 + 4.5 / Math.max(Math.abs(this.u), 3));
    const dTar = clamp(ctrl.steer, -1, 1) * dLim;
    const dMax = this.steerRate * dt;
    this.delta += clamp(dTar - this.delta, -dMax, dMax);
    const d = this.delta;
    const kin = clamp(1 - Math.abs(this.u) / 4, 0, 1);
    const mu = env.mu;

    const rev = ctrl.throttle < 0;
    let Fdrive = ctrl.throttle *
      Math.min(rev ? this.Fmax * 0.4 : this.Fmax, this.P / Math.max(Math.abs(this.u), 3));
    const Fdrag = this.cDrag * this.u * Math.abs(this.u);
    const Frr = this.cRR * (env.offroad ? 2.6 : 1) * Math.sign(this.u || 0);

    const Fzf = Math.max(this.m * GRAV * this.b / this.L - this.m * this.ax * this.h / this.L, this.m * GRAV * 0.12);
    const Fzr = Math.max(this.m * GRAV * this.a / this.L + this.m * this.ax * this.h / this.L, this.m * GRAV * 0.12);
    const gripF = mu * Fzf, gripR = mu * Fzr;

    const bSign = Math.sign(this.u) || 1;
    let Fxf = Fdrive;
    let Fxbf = -bSign * Math.abs(ctrl.brake) * this.brakeF * 0.62;
    let Fxbr = -bSign * Math.abs(ctrl.brake) * this.brakeF * 0.38;
    this.lockF = this.lockR = false; this.wheelSpin = false;

    if (env.abs && Math.abs(Fxbr) > 0.55 * gripR) {
      const excess = Math.abs(Fxbr) - 0.55 * gripR;
      Fxbr = -bSign * 0.55 * gripR;
      Fxbf -= bSign * excess;
    }
    if (Math.abs(Fxf) > gripF * 0.96) { Fxf = Math.sign(Fxf) * gripF * 0.96; this.wheelSpin = true; }
    const lockThr = env.abs ? 1.02 : 0.97;
    if (Math.abs(Fxbf) > gripF * lockThr) { Fxbf = -bSign * gripF * (env.abs ? 0.92 : 1); this.lockF = !env.abs; }
    if (Math.abs(Fxbr) > gripR * lockThr) { Fxbr = -bSign * gripR * (env.abs ? 0.92 : 1); this.lockR = !env.abs; }

    const uG = Math.max(Math.abs(this.u), 2.2) * (Math.sign(this.u) || 1);
    const af = Math.atan2(this.w + this.a * this.om, uG) - d;
    const ar = Math.atan2(this.w - this.b * this.om, uG);

    const latCapF = 0.96 * Math.sqrt(Math.max(gripF * gripF - Fxf * Fxf - Fxbf * Fxbf, 0));
    const latCapR = Math.sqrt(Math.max(gripR * gripR - Fxbr * Fxbr, 0));
    const keepF = this.lockF ? 0.12 : 1;
    const keepR = this.lockR ? 0.12 : 1;
    const rawF = -this.Cf * af, rawR = -this.Cr * ar;
    let Fyf = clamp(rawF, -latCapF, latCapF) * keepF;
    let Fyr = clamp(rawR, -latCapR, latCapR) * keepR;
    Fyf *= (1 - kin); Fyr *= (1 - kin);
    this.satF = Math.abs(this.u) > 5 && Math.abs(rawF) > latCapF * 1.02 + 1;
    this.satR = Math.abs(this.u) > 5 && Math.abs(rawR) > latCapR * 1.02 + 1;
    this.slipF = Math.abs(this.u) > 4 && Math.abs(rawF) > latCapF * 1.35;
    this.slipR = Math.abs(this.u) > 4 && Math.abs(rawR) > latCapR * 1.35;

    let Mesc = 0;
    this.escActive = false;
    if (env.esc && Math.abs(this.u) > 4) {
      const beta = this.beta;
      const omMax = 0.8 * mu * GRAV / Math.max(Math.abs(this.u), 4);
      const omRef = clamp(this.u * Math.tan(d) / this.L, -omMax, omMax);
      const err = this.om - omRef;
      if (Math.abs(beta) > 0.12 || Math.abs(err) > 0.45 || (this.satR && Math.abs(err) > 0.15)) {
        this.escActive = true;
        Fdrive = 0;
        Mesc = clamp(-(1400 * beta + 5200 * err), -5600, 5600);
        Fxbr -= bSign * 0.1 * gripR;
      }
    }

    const Fx = Fxf + Fxbf + Fxbr - Fdrag - Frr;
    const Fy = Fyf * Math.cos(d) + Fyr;
    const du = (Fx - Fyf * Math.sin(d)) / this.m + this.w * this.om;
    const dw = Fy / this.m - this.u * this.om;
    const dom = (this.a * Fyf * Math.cos(d) - this.b * Fyr + Mesc) / this.Iz;

    this.u += du * dt; this.w += dw * dt; this.om += dom * dt;
    this.om = clamp(this.om, -2.6, 2.6);
    this.w = clamp(this.w, -26, 26);

    if (kin > 0) {
      const k2 = kin * (1 - Math.exp(-dt * 10));
      let uSteer = this.u;
      if (Math.abs(ctrl.throttle || 0) > 0.05 && Math.abs(ctrl.steer || 0) > 0.05) {
        const uDir = (ctrl.throttle || 0) >= 0 ? 1 : -1;
        uSteer = uDir * Math.max(Math.abs(this.u), 2.8 * Math.abs(ctrl.throttle || 0));
      }
      const omK = uSteer * Math.tan(d) / this.L;
      this.om += (omK - this.om) * k2;
      this.w *= 1 - k2;
      if (Math.abs(this.u) < 0.35 && Math.abs(ctrl.throttle) < 0.05) {
        const st = Math.exp(-dt * 9);
        this.u *= st; this.w *= st; this.om *= st;
      }
    }
    if (Math.abs(this.u) < 0.02 && Math.abs(ctrl.throttle) < 0.02) this.u = 0;

    this.psi += this.om * dt;
    const f = this.forward(), r = this.right();
    this.x += (this.u * f.x + this.w * r.x) * dt;
    this.z += (this.u * f.z + this.w * r.z) * dt;

    const filt = 1 - Math.exp(-dt / 0.1);
    this.ax += (du - this.ax) * filt;
    this.ay += (Fy / this.m - this.ay) * filt;
    return this;
  }
}
