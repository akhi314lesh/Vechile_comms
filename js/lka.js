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
    this.wheelbase = config.wheelbase ?? 3.2;        // m
    this.kUndersteer = config.kUndersteer ?? 0.16;   // s
    this.tauSteer = config.tauSteer ?? 0.08;         // s (actuator lag)
    this.maxSlew = config.maxSlew ?? 5.5;            // Slew rate limit (rad/s)
    this.deadbandLat = config.deadbandLat ?? 0.02;   // m (suppress micro-hunting)
    this.deadbandHead = config.deadbandHead ?? 0.010;// rad
    this.s = (config.steerPositive === 'right') ? -1.0 : 1.0;
    this.prevRaw = 0.0;
    this.diag = {};
    this.reset(0.0);
  }

  reset(steer = 0.0) {
    this.steer = clamp(this.s * steer, -1.0, 1.0);
    this.prevRaw = this.steer;
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
    const vKmh = v * 3.6;

    let ep = ePsi, kp = kappa, om = omega;
    if (this.s < 0.0) {
      ep = -ep; kp = -kp; om = -om;
    }

    const wrappedHeading = wrapAngle(ep);

    // 1. Continuous speed-decay cross-track gain (resolves 0-60 km/h constant-gain bug)
    const kLat = 0.44 / (1.0 + 0.022 * v);

    // 2. Heading alignment gain
    const kHead = 0.72 + 0.002 * vKmh;

    // 3. Speed-scheduled excess yaw damping (critically damps yaw rate at high speeds)
    const kDamp = 0.12 + 0.003 * vKmh;

    // 4. Curvature feedforward
    const ff = kp * (this.wheelbase + v * this.kUndersteer);

    // 5. Smooth C^inf Convergence Scaling (prevents threshold chattering near 0.45m)
    let effectiveKLat = kLat;
    const isConverging = (eLat > 0 && wrappedHeading < -0.01) || (eLat < 0 && wrappedHeading > 0.01);
    if (isConverging && Math.abs(eLat) < 0.45) {
      effectiveKLat *= (0.60 + 0.40 * Math.tanh(Math.abs(eLat) / 0.45));
    }

    // Smooth spline deadband on cross-track error near lane center
    if (Math.abs(eLat) < this.deadbandLat) {
      const u = Math.abs(eLat) / this.deadbandLat;
      effectiveKLat *= (u * u);
    }

    // 6. Raw control law with excess-yaw-rate damping
    let raw = effectiveKLat * eLat - kHead * wrappedHeading + ff;
    const excessYawRate = om - v * kp;
    const damping = kDamp * excessYawRate;
    raw -= damping;

    // 7. Actuator phase lead compensation
    const rawDelta = raw - this.prevRaw;
    const leadAlpha = 0.40 * (this.tauSteer / this.dt);
    const leadRaw = raw + clamp(leadAlpha * rawDelta, -0.05, 0.05);
    this.prevRaw = raw;

    // 8. Actuator slew-rate limiting (speed-adaptive)
    const speedSlewFactor = clamp(1.0 - (v / 45.0) * 0.35, 0.65, 1.0);
    const maxStep = this.maxSlew * speedSlewFactor * this.dt;

    const cmd = clamp(leadRaw, -1.0, 1.0);
    const slewLimited = Math.abs(cmd - this.steer) > maxStep;
    this.steer = clamp(this.steer + clamp(cmd - this.steer, -maxStep, maxStep), -1.0, 1.0);

    this.diag = {
      raw: leadRaw,
      kLat: effectiveKLat,
      kHead,
      kDamp,
      ff,
      damping,
      saturated: Math.abs(leadRaw) > 1.0,
      slewLimited,
      curveExit: Math.abs(kp) < 1e-4 && Math.abs(om) > 0.05
    };

    return this.s * this.steer;
  }
}
