/**
 * js/lka.js — High-Precision Stanley Path Follower with Post-Curve Stabilization
 * 
 * Solves Post-Turn Zigzagging / Steering Oscillation:
 *   1. Excess-Yaw-Rate Damping: -kDamp * (omega - v * kappa) prevents overshoot on curve exit
 *   2. Speed-Scaled Slew Rate Limiting: smooth actuation prevents instantaneous wheel snap
 *   3. Asymptotic Heading Alignment: avoids over-correction when heading already points to lane center
 *   4. Smooth Spline Deadband: suppresses micro-oscillation near lane center
 */

const TAU = 2.0 * Math.PI;

export function wrapAngle(a) {
  return ((a + Math.PI) % TAU + TAU) % TAU - Math.PI;
}

const clamp = (x, lo, hi) => Math.min(hi, Math.max(lo, x));

export class LaneKeepController {
  constructor(config = {}) {
    this.dt = config.dt ?? 1.0 / 30.0;
    this.kLatGain = config.kLatGain ?? 0.44;
    this.kLatSpeed = config.kLatSpeed ?? 0.055;
    this.kHead = config.kHead ?? 0.72;
    this.wheelbase = config.wheelbase ?? 3.2;
    this.kUndersteer = config.kUndersteer ?? 0.16;
    this.kDamp = config.kDamp ?? 0.12;             // Excess yaw damping (critically damped exit)
    this.maxSlew = config.maxSlew ?? 5.5;          // Slew rate limit (rad/s)
    this.s = (config.steerPositive === 'right') ? -1.0 : 1.0;
    this.deadbandLat = config.deadbandLat ?? 0.02; // m (suppress micro-hunting)
    this.deadbandHead = config.deadbandHead ?? 0.010;// rad
    this.prevCmd = 0.0;
    this.diag = {};
    this.reset(0.0);
  }

  reset(steer = 0.0) {
    this.steer = clamp(this.s * steer, -1.0, 1.0);
    this.prevCmd = this.steer;
    this.diag = {
      raw: 0.0, kLat: 0.0, ff: 0.0, damping: 0.0,
      saturated: false, slewLimited: false, curveExit: false
    };
  }

  /**
   * One control step
   * @param {number} eLat - Cross-track error (+ = vehicle RIGHT of center)
   * @param {number} ePsi - Heading error (+ = vehicle pointing LEFT of road tangent)
   * @param {number} kappa - Road curvature (+ = curve LEFT)
   * @param {number} v - Vehicle speed (m/s)
   * @param {number} omega - Vehicle yaw rate (rad/s, + = CCW / turn LEFT)
   * @returns {number} Normalized steering command in [-1, 1]
   */
  update(eLat, ePsi, kappa, v, omega) {
    if (![eLat, ePsi, kappa, v, omega].every(Number.isFinite)) {
      return this.s * this.steer;
    }
    v = Math.max(v, 0.0);

    let ep = ePsi, kp = kappa, om = omega;
    if (this.s < 0.0) {
      ep = -ep; kp = -kp; om = -om;
    }

    const wrappedHeading = wrapAngle(ep);

    // 1. Speed-scaled cross-track gain (responsive at low speed, rock-solid at high speed)
    const kLat = this.kLatGain / Math.max(1.0, v * this.kLatSpeed);

    // 2. Curvature feedforward (proactive angle before error accumulates)
    const ff = kp * (this.wheelbase + v * this.kUndersteer);

    // 3. Asymptotic Heading Alignment Guard:
    // If vehicle is already angling toward the centerline, soften the cross-track gain
    // to prevent overshooting past the centerline.
    let effectiveKLat = kLat;
    const isConverging = (eLat > 0 && wrappedHeading < -0.01) || (eLat < 0 && wrappedHeading > 0.01);
    if (isConverging && Math.abs(eLat) < 0.45) {
      effectiveKLat *= 0.60;
    }

    // Smooth spline deadband on cross-track error near lane center
    if (Math.abs(eLat) < this.deadbandLat) {
      const u = Math.abs(eLat) / this.deadbandLat;
      effectiveKLat *= (u * u);
    }

    // 4. Raw Stanley control law
    let raw = effectiveKLat * eLat - this.kHead * wrappedHeading + ff;

    // 5. Excess-Yaw-Rate Damping:
    // (om - v * kp) is 0 during steady cornering, but non-zero when exiting curves.
    // This actively damps residual yaw rotation without fighting the steady turn.
    const excessYawRate = om - v * kp;
    const damping = this.kDamp * excessYawRate;
    raw -= damping;

    // 7. Actuator slew-rate limiting (speed-adaptive)
    // Slew limit is tighter at high speed to prevent high-g lateral snap
    const speedSlewFactor = clamp(1.0 - (v / 45.0) * 0.35, 0.65, 1.0);
    const maxStep = this.maxSlew * speedSlewFactor * this.dt;

    const cmd = clamp(raw, -1.0, 1.0);
    const slewLimited = Math.abs(cmd - this.steer) > maxStep;
    this.steer = clamp(this.steer + clamp(cmd - this.steer, -maxStep, maxStep), -1.0, 1.0);

    this.diag = {
      raw,
      kLat: effectiveKLat,
      ff,
      damping,
      saturated: Math.abs(raw) > 1.0,
      slewLimited,
      curveExit: Math.abs(kp) < 1e-4 && Math.abs(om) > 0.05
    };

    return this.s * this.steer;
  }
}
