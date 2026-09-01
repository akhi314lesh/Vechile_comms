/**
 * js/lka.js — Stanley-style lane centering / LKA with curvature feedforward.
 *
 * Spec conventions:
 *   e_lat : + = vehicle RIGHT of lane center
 *   e_psi : + = vehicle heading LEFT of road tangent
 *   kappa : + = road curves LEFT
 *   omega : + = yaw LEFT (CCW)
 *   steer : + = turn LEFT
 *
 * Set steerPositive='left' (Three.js standard: +delta turns left).
 */

const TAU = 2.0 * Math.PI;

export function wrapAngle(a) {
  return ((a + Math.PI) % TAU + TAU) % TAU - Math.PI;
}

const clamp = (x, lo, hi) => Math.min(hi, Math.max(lo, x));

export class LaneKeepController {
  constructor(config = {}) {
    this.dt = config.dt ?? 1.0 / 30.0;
    this.kLatGain = config.kLatGain ?? 0.42;
    this.kLatSpeed = config.kLatSpeed ?? 0.07;
    this.kHead = config.kHead ?? 0.70;
    this.wheelbase = config.wheelbase ?? 3.2;
    this.kUndersteer = config.kUndersteer ?? 0.16;
    this.kDamp = config.kDamp ?? 0.0;
    this.maxSlew = config.maxSlew ?? 6.0;
    this.s = (config.steerPositive === 'right') ? -1.0 : 1.0;
    this.deadband = config.deadband ?? 0.0;
    this.diag = {};
    this.reset(0.0);
  }

  reset(steer = 0.0) {
    this.steer = clamp(this.s * steer, -1.0, 1.0);
    this.diag = { raw: 0.0, kLat: 0.0, ff: 0.0, saturated: false, slewLimited: false };
  }

  update(eLat, ePsi, kappa, v, omega) {
    if (![eLat, ePsi, kappa, v, omega].every(Number.isFinite)) {
      return this.s * this.steer;
    }
    v = Math.max(v, 0.0);

    let ep = ePsi, kp = kappa, om = omega;
    if (this.s < 0.0) {
      ep = -ep; kp = -kp; om = -om;
    }

    // (3a) speed-scaled cross-track gain: responsive low, stable high
    const kLat = this.kLatGain / Math.max(1.0, v * this.kLatSpeed);

    // (3c) curvature feedforward — proactive steer before error accumulates
    const ff = kp * (this.wheelbase + v * this.kUndersteer);

    // (3b) heading alignment / damping (ePsi wrapped!)
    let raw = kLat * eLat - this.kHead * wrapAngle(ep) + ff;

    // Optional derivative damping on EXCESS yaw rate only:
    if (this.kDamp) {
      raw -= this.kDamp * (om - v * kp);
    }

    // Optional micro-deadband
    if (this.deadband > 0.0 && Math.abs(eLat) < this.deadband) {
      raw -= kLat * eLat;
    }

    const cmd = clamp(raw, -1.0, 1.0);
    const step = this.maxSlew * this.dt;
    this.diag = {
      raw, kLat, ff,
      saturated: Math.abs(raw) > 1.0,
      slewLimited: Math.abs(cmd - this.steer) > step + 1e-12
    };
    this.steer = clamp(this.steer + clamp(cmd - this.steer, -step, step), -1.0, 1.0);
    return this.s * this.steer;
  }
}
