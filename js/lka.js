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
    this.tauFilt = config.tauFilt ?? 0.15;           // s (curvature filter time constant)
    this.maxSlew = config.maxSlew ?? 6.0;            // Slew rate limit (rad/s)
    this.ki = config.ki ?? 0.15;                     // Integral trim gain (spec: 0.15)
    this.iLimit = config.iLimit ?? 0.30;             // Anti-windup clamp (spec: +/- 0.3)
    this.deadbandLat = config.deadbandLat ?? 0.02;   // m (suppress micro-hunting)
    this.deadbandHead = config.deadbandHead ?? 0.008;// rad
    this.s = (config.steerPositive === 'right') ? -1.0 : 1.0;

    this.integral = 0.0;
    this.filteredKappa = 0.0;
    this.prevRaw = null;
    this.steer = 0.0;
    this.diag = {};
    this.reset(0.0);
  }

  reset(steer = 0.0) {
    this.steer = clamp(this.s * steer, -1.0, 1.0);
    this.prevRaw = null;
    this.integral = 0.0;
    this.filteredKappa = 0.0;
    this.diag = {
      raw: 0.0, kLat: 0.0, ff: 0.0, damping: 0.0, iTrim: 0.0,
      saturated: false, slewLimited: false, curveExit: false
    };
  }

  /**
   * One control step
   * @param {number} eLat - Cross-track error (+ = vehicle RIGHT of center)
   * @param {number} ePsi - Heading error (+ = vehicle pointing LEFT of road tangent)
   * @param {number} kappaPreview - Average previewed road curvature (+ = curve LEFT)
   * @param {number} v - Vehicle speed (m/s)
   * @param {number} omega - Vehicle yaw rate (rad/s, + = CCW / turn LEFT)
   * @returns {number} Normalized steering command in [-1, 1]
   */
  update(eLat, ePsi, kappaPreview, v, omega) {
    if (![eLat, ePsi, kappaPreview, v, omega].every(Number.isFinite)) {
      return this.s * this.steer;
    }
    v = Math.max(v, 0.0);
    const vKmh = v * 3.6;
    const vKmhOver10 = vKmh / 10.0;

    let ep = ePsi, kp = kappaPreview, om = omega;
    if (this.s < 0.0) {
      ep = -ep; kp = -kp; om = -om;
    }

    const wrappedHeading = wrapAngle(ep);

    // 1. Cross-Track Gain (smooth speed decay up to 80 km/h)
    const kLat = 0.44 / (1.0 + 0.022 * v);

    // 2. Heading Alignment Gain
    const kHead = 0.76 + 0.003 * vKmh;

    // 3. Speed-Scheduled Yaw Damping (critically damps yaw rate at high speeds)
    const kDamp = 0.16 + 0.003 * vKmh;

    // 4. Low-pass filter kappa_preview with tau_filt = 0.15s
    const filtAlpha = clamp(this.dt / Math.max(this.dt, this.tauFilt), 0.0, 1.0);
    this.filteredKappa += (kp - this.filteredKappa) * filtAlpha;

    // 5. Curvature Feedforward
    const ff = this.filteredKappa * (this.wheelbase + v * this.kUndersteer);

    // 6. Smooth C^inf Convergence Scaling (tanh blend)
    let effectiveKLat = kLat;
    const isConverging = (eLat > 0 && wrappedHeading < -0.01) || (eLat < 0 && wrappedHeading > 0.01);
    if (isConverging) {
      effectiveKLat *= (0.60 + 0.40 * Math.tanh(Math.abs(eLat) / 0.45));
    }

    // Smooth quadratic deadband near lane center
    if (Math.abs(eLat) < this.deadbandLat) {
      const u = Math.abs(eLat) / this.deadbandLat;
      effectiveKLat *= (u * u);
    }

    // 7. Integral Trim on Cross-Track Error (active in near-center band with anti-windup)
    if (Math.abs(eLat) < 0.35) {
      this.integral += eLat * this.dt;
      const maxIntegral = this.iLimit / this.ki;
      this.integral = clamp(this.integral, -maxIntegral, maxIntegral);
    } else {
      this.integral *= 0.85;
    }
    const iTrim = this.ki * this.integral;

    // 8. Raw Feedback Law
    let raw = effectiveKLat * eLat - kHead * wrappedHeading + ff + iTrim;

    // 9. Excess-Yaw Damping (stabilizes dominant pole pair without fighting steady turn)
    const excessYawRate = om - v * this.filteredKappa;
    const damping = kDamp * excessYawRate;
    raw -= damping;

    // 10. Actuator Phase Lead Compensation (cancels tau_steer lag pole)
    let leadRaw = raw;
    if (this.prevRaw !== null) {
      const rawDelta = raw - this.prevRaw;
      const leadAlpha = 0.40 * (this.tauSteer / this.dt);
      leadRaw = raw + clamp(leadAlpha * rawDelta, -0.06, 0.06);
    }
    this.prevRaw = raw;

    // 11. Speed-Adaptive Slew Rate Limiter
    const speedSlewFactor = clamp(1.0 - (v / 60.0) * 0.35, 0.65, 1.0);
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
      iTrim,
      saturated: Math.abs(leadRaw) > 1.0,
      slewLimited,
      curveExit: Math.abs(kp) < 1e-4 && Math.abs(om) > 0.05
    };

    return this.s * this.steer;
  }
}
