/**
 * js/aeb-policy.js — 360-Degree Surround Perception & Autonomous Emergency Braking (AEB) Policy
 * 
 * This module monitors all 360° sensor raycasts around the ego vehicle,
 * calculates Time-to-Collision (TTC) and sector proximity distances, and executes
 * progressive deceleration or aggressive emergency braking whenever thresholds are breached.
 */

export class SurroundAEBPolicy {
  constructor(config = {}) {
    this.criticalDist = config.criticalDist ?? 6.5; // Meters: instant full emergency stop
    this.warningDist = config.warningDist ?? 16.0;   // Meters: progressive deceleration
    this.criticalTTC = config.criticalTTC ?? 1.35;   // Seconds: time-to-collision hard stop
    this.warningTTC = config.warningTTC ?? 2.6;     // Seconds: warning slowdown threshold
    this.sideSafetyDist = config.sideSafetyDist ?? 2.2; // Meters: lateral blind spot clearance
  }

  /**
   * Evaluates 360° proximity rays and returns control adjustments
   * @param {Object} sensorFrame - { ego, proximity: { fwd, left, right, rear, fwdTtc }, rays }
   * @param {Object} baseControl - { steer, throttle, brake }
   * @returns {Object} { steer, throttle, brake, alert, severity, aebTriggered }
   */
  evaluate(sensorFrame, baseControl = { steer: 0, throttle: 0, brake: 0 }) {
    const { ego, proximity } = sensorFrame;
    const speed = Math.max(ego ? ego.u : 0, 0);

    let steer = baseControl.steer;
    let throttle = baseControl.throttle;
    let brake = baseControl.brake;
    let alert = null;
    let severity = 'amber';
    let aebTriggered = false;

    if (!proximity) {
      return { steer, throttle, brake, alert, severity, aebTriggered };
    }

    const { fwd, left, right, rear, fwdTtc } = proximity;

    // ─── 1. CRITICAL EMERGENCY STOP THRESHOLD ───
    if (fwd <= this.criticalDist || (fwd < 20.0 && fwdTtc <= this.criticalTTC)) {
      throttle = 0.0;
      brake = 1.0; // 100% full emergency braking force
      aebTriggered = true;
      alert = `⚠️ AEB · CRITICAL PROXIMITY (${fwd.toFixed(1)}M) — EMERGENCY STOP`;
      severity = 'red';
      return { steer, throttle, brake, alert, severity, aebTriggered };
    }

    // ─── 2. WARNING / PROGRESSIVE DECELERATION THRESHOLD ───
    if (fwd <= this.warningDist || (fwd < 32.0 && fwdTtc <= this.warningTTC)) {
      const brakeIntensity = Math.min(1.0, Math.max(0.2, (this.warningDist - fwd) / (this.warningDist - this.criticalDist)));
      throttle = Math.min(throttle, 0.1);
      brake = Math.max(brake, brakeIntensity);
      aebTriggered = true;
      alert = `PROXIMITY WARNING (${fwd.toFixed(1)}M) — DECELERATING`;
      severity = 'amber';
    }

    // ─── 3. 360° BLIND SPOT & SIDE PROXIMITY STEERING OVERRIDE ───
    if (left < this.sideSafetyDist && steer > 0.05) {
      // Vehicle is dangerously close on the left; inhibit left turn
      steer = Math.min(steer, 0.0);
      alert = alert || `BLIND SPOT · LEFT CLEARANCE BREACH (${left.toFixed(1)}M)`;
      severity = 'amber';
    }

    if (right < this.sideSafetyDist && steer < -0.05) {
      // Vehicle is dangerously close on the right; inhibit right turn
      steer = Math.max(steer, 0.0);
      alert = alert || `BLIND SPOT · RIGHT CLEARANCE BREACH (${right.toFixed(1)}M)`;
      severity = 'amber';
    }

    return { steer, throttle, brake, alert, severity, aebTriggered };
  }
}
