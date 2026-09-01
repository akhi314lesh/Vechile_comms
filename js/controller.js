/**
 * js/controller.js — Deterministic Autonomous Vehicle Control Stack
 * 
 * Provides:
 *   1. LongitudinalPID — Anti-windup PID speed & acceleration controller
 *   2. IDMController   — Intelligent Driver Model for car following & headway
 *   3. CurvatureGovernor — Proactive curve deceleration envelope
 *   4. LaneChangeStateMachine — Deterministic multi-state lane changing
 *   5. AutonomousDrivingStack — Complete integrated driving controller
 * 
 * NOTE: RL is completely untouched and independent.
 */

import { clamp, wrapAngle } from './utils.js';
import { LaneKeepController } from './lka.js';
import { applyAEB, makeAEBShield } from './aeb.js';

// Physical constants
const GRAV = 9.81;

/**
 * ─── 1. LONGITUDINAL PID CONTROLLER ───
 * Controls speed and acceleration with integral anti-windup,
 * smooth slew rate limiting, and dedicated throttle/brake separation.
 */
export class LongitudinalPID {
  constructor(config = {}) {
    this.kp = config.kp ?? 0.45;
    this.ki = config.ki ?? 0.08;
    this.kd = config.kd ?? 0.04;
    this.integralLimit = config.integralLimit ?? 4.0;
    this.maxAccel = config.maxAccel ?? 3.5;    // m/s²
    this.maxDecel = config.maxDecel ?? 7.0;    // m/s²
    this.deadband = config.deadband ?? 0.35;   // m/s error deadband

    this.integral = 0.0;
    this.prevError = 0.0;
    this.prevSpeed = 0.0;
    this.throttle = 0.0;
    this.brake = 0.0;
  }

  reset() {
    this.integral = 0.0;
    this.prevError = 0.0;
    this.prevSpeed = 0.0;
    this.throttle = 0.0;
    this.brake = 0.0;
  }

  /**
   * Update speed control
   * @param {number} currentSpeed - Current forward speed in m/s
   * @param {number} targetSpeed - Target speed in m/s
   * @param {number} dt - Time step in seconds
   * @returns {{ throttle: number, brake: number, accelCmd: number }}
   */
  update(currentSpeed, targetSpeed, dt) {
    if (dt <= 0) return { throttle: this.throttle, brake: this.brake, accelCmd: 0 };
    currentSpeed = Math.max(0, currentSpeed);
    targetSpeed = Math.max(0, targetSpeed);

    const error = targetSpeed - currentSpeed;

    // Integral with anti-windup
    if (Math.abs(error) > this.deadband) {
      this.integral += error * dt;
      this.integral = clamp(this.integral, -this.integralLimit, this.integralLimit);
    } else {
      this.integral *= 0.95; // Decay near target
    }

    // Derivative term (on measured speed to avoid derivative kick)
    const dSpeed = (currentSpeed - this.prevSpeed) / dt;
    this.prevSpeed = currentSpeed;
    this.prevError = error;

    // Raw acceleration demand
    const rawAccel = this.kp * error + this.ki * this.integral - this.kd * dSpeed;
    const accelCmd = clamp(rawAccel, -this.maxDecel, this.maxAccel);

    // Smooth actuation mapping with deadband
    let targetThrottle = 0.0;
    let targetBrake = 0.0;

    if (accelCmd > 0.15) {
      targetThrottle = clamp(accelCmd / this.maxAccel, 0.0, 1.0);
      targetBrake = 0.0;
    } else if (accelCmd < -0.4) {
      targetThrottle = 0.0;
      targetBrake = clamp(-accelCmd / this.maxDecel, 0.0, 1.0);
    } else {
      targetThrottle = 0.05; // Idle roll / coasting
      targetBrake = 0.0;
    }

    // Actuator slew rate limiting to prevent jerky transitions
    const maxThrottleRate = 2.5 * dt;
    const maxBrakeRate = 5.0 * dt;
    this.throttle += clamp(targetThrottle - this.throttle, -maxThrottleRate * 1.5, maxThrottleRate);
    this.brake += clamp(targetBrake - this.brake, -maxBrakeRate, maxBrakeRate);

    this.throttle = clamp(this.throttle, 0.0, 1.0);
    this.brake = clamp(this.brake, 0.0, 1.0);

    return { throttle: this.throttle, brake: this.brake, accelCmd };
  }
}

/**
 * ─── 2. INTELLIGENT DRIVER MODEL (IDM) CONTROLLER ───
 * Calculates safe dynamic following gap and speed adjustments:
 * s*(v, Δv) = s0 + v*T + (v*Δv) / (2*sqrt(a*b))
 */
export class IDMController {
  constructor(config = {}) {
    this.s0 = config.s0 ?? 5.5;               // Minimum standstill distance (m)
    this.timeHeadway = config.timeHeadway ?? 1.4; // Safe time gap (s)
    this.maxAccel = config.maxAccel ?? 2.8;   // Comfortable acceleration (m/s²)
    this.comfDecel = config.comfDecel ?? 3.2; // Comfortable deceleration (m/s²)
    this.delta = config.delta ?? 4.0;         // Acceleration exponent
  }

  /**
   * Computes desired target speed and following acceleration
   * @param {number} egoSpeed - Ego vehicle speed (m/s)
   * @param {number} cruiseSpeed - Desired cruise speed (m/s)
   * @param {number} leadDist - Distance to lead vehicle (m)
   * @param {number} leadRelSpeed - Relative speed (leadSpeed - egoSpeed) (m/s)
   * @param {boolean} isLeadBraking - Whether lead vehicle is actively braking
   * @returns {{ targetSpeed: number, freeRoadRatio: number, desiredGap: number }}
   */
  evaluate(egoSpeed, cruiseSpeed, leadDist = Infinity, leadRelSpeed = 0.0, isLeadBraking = false) {
    egoSpeed = Math.max(0.01, egoSpeed);
    cruiseSpeed = Math.max(0.01, cruiseSpeed);

    if (!Number.isFinite(leadDist) || leadDist > 180.0) {
      // Free road driving
      return { targetSpeed: cruiseSpeed, freeRoadRatio: 1.0, desiredGap: this.s0 };
    }

    const deltaV = -leadRelSpeed; // Positive when ego is closing on lead
    const sStar = this.s0 + egoSpeed * this.timeHeadway + (egoSpeed * deltaV) / (2 * Math.sqrt(this.maxAccel * this.comfDecel));
    const effectiveGap = Math.max(0.5, leadDist);

    // IDM acceleration formula
    const freeRoadTerm = 1 - Math.pow(egoSpeed / cruiseSpeed, this.delta);
    const interactionTerm = Math.pow(sStar / effectiveGap, 2);
    const idmAccel = this.maxAccel * (freeRoadTerm - interactionTerm);

    // Target speed adjustment
    let targetSpeed = cruiseSpeed;
    if (interactionTerm > 0.05) {
      const leadSpeed = Math.max(0, egoSpeed + leadRelSpeed);
      // Adjust target speed smoothly to match lead + gap error
      targetSpeed = Math.min(cruiseSpeed, Math.max(0, leadSpeed + 0.45 * (effectiveGap - sStar)));
    }

    // Early reaction if lead vehicle is actively braking (via V2V or brake light)
    if (isLeadBraking && leadDist < 45.0) {
      targetSpeed = Math.min(targetSpeed, egoSpeed * 0.65);
    }

    return { targetSpeed: Math.max(0, targetSpeed), freeRoadRatio: Math.max(0, freeRoadTerm), desiredGap: sStar };
  }
}

/**
 * ─── 3. PROACTIVE CURVATURE GOVERNOR ───
 * Pre-emptively calculates safe curve speeds before entering turns:
 * v_safe(d) = sqrt(v_curve^2 + 2 * a_comf * d)
 */
export class CurvatureGovernor {
  constructor(config = {}) {
    this.mu = config.mu ?? 0.88;             // Road friction coefficient
    this.lateralAccLimit = config.latAcc ?? 4.2; // Max comfortable lateral acceleration (m/s²)
    this.comfDecel = config.comfDecel ?? 2.8; // Comfortable approach braking (m/s²)
    this.lookaheadDist = config.lookahead ?? 85.0; // Max lookahead distance (m)
  }

  /**
   * Evaluate road curvature profile ahead and compute speed ceiling
   * @param {Object} track - Track definition with kappaAt(s)
   * @param {number} egoS - Vehicle longitudinal road coordinate (m)
   * @param {number} currentSpeed - Vehicle speed in m/s
   * @param {number} cruiseSpeed - Base cruise speed in m/s
   * @returns {{ safeSpeed: number, upcomingCurvature: number, governorActive: boolean }}
   */
  evaluate(track, egoS, currentSpeed, cruiseSpeed) {
    if (!track || typeof track.kappaAt !== 'function') {
      return { safeSpeed: cruiseSpeed, upcomingCurvature: 0, governorActive: false };
    }

    let minAllowedSpeed = cruiseSpeed;
    let maxUpcomingK = 0;
    let governorActive = false;

    // Scan ahead along track in steps of 6 meters
    const scanEnd = Math.min(this.lookaheadDist, Math.max(25, currentSpeed * 3.2 + 20));
    for (let d = 5; d <= scanEnd; d += 6) {
      const sAhead = (egoS + d) % (track.L || 1000);
      const k = Math.abs(track.kappaAt(sAhead));
      if (k < 1e-4) continue;

      if (k > maxUpcomingK) maxUpcomingK = k;

      // Steady-state curve speed limit
      const vCurve = Math.sqrt(Math.min(this.lateralAccLimit, this.mu * GRAV * 0.6) / k);
      // Approach speed limit allowing comfortable deceleration over distance d
      const vApproach = Math.sqrt(vCurve * vCurve + 2 * this.comfDecel * d);

      if (vApproach < minAllowedSpeed) {
        minAllowedSpeed = vApproach;
        governorActive = true;
      }
    }

    return {
      safeSpeed: Math.max(4.0, Math.min(cruiseSpeed, minAllowedSpeed)),
      upcomingCurvature: maxUpcomingK,
      governorActive: governorActive && minAllowedSpeed < cruiseSpeed - 1.5
    };
  }
}

/**
 * ─── 4. DETERMINISTIC LANE CHANGE STATE MACHINE ───
 * States: KEEP_LANE, PREPARE_LANE_CHANGE, CHECK_GAP, LANE_CHANGE, COMPLETE, ABORT
 */
export const LaneChangeState = {
  KEEP_LANE: 'KEEP_LANE',
  PREPARE: 'PREPARE',
  CHECK_GAP: 'CHECK_GAP',
  LANE_CHANGE: 'LANE_CHANGE',
  COMPLETE: 'COMPLETE',
  ABORT: 'ABORT'
};

export class LaneChangeStateMachine {
  constructor(config = {}) {
    this.state = LaneChangeState.KEEP_LANE;
    this.currentLane = 0;
    this.targetLane = 0;
    this.latPosition = 0.0;
    this.timer = 0.0;
    this.cooldown = 0.0;
    this.lateralSpeed = config.latSpeed ?? 1.4; // m/s lateral transition rate
    this.minFrontGap = config.minFrontGap ?? 25.0; // m
    this.minRearGap = config.minRearGap ?? 20.0;   // m
    this.minTTC = config.minTTC ?? 3.5;           // s
  }

  reset() {
    this.state = LaneChangeState.KEEP_LANE;
    this.currentLane = 0;
    this.targetLane = 0;
    this.latPosition = 0.0;
    this.timer = 0.0;
    this.cooldown = 0.0;
  }

  /**
   * Evaluates lane safety from perceived objects
   */
  isLaneSafe(targetLane, perceivedObjects, egoS, egoLat, egoSpeed, track) {
    if (!track || !track.laneLat) return true;
    const targetCenter = track.laneLat('fwd', targetLane);

    for (const obj of perceivedObjects || []) {
      // Check lateral overlap with target lane corridor (±2.0m from target lane center)
      if (Math.abs(obj.lat - targetCenter) > 2.0) continue;

      const relS = track.wrapS ? track.wrapS(obj.s - egoS) : (obj.s - egoS);
      const relV = (obj.vAlong ?? obj.speed ?? 0) - egoSpeed;

      // Front safety gap check (must be clear ahead)
      if (relS >= 0 && relS < this.minFrontGap) {
        return false;
      }

      // Rear safety gap check (must be clear behind)
      if (relS < 0 && relS > -this.minRearGap) {
        return false;
      }

      // Dynamic closing TTC check
      if (relS > 0 && relV < -0.5) {
        const ttc = relS / Math.abs(relV);
        if (ttc < this.minTTC) return false;
      }
    }
    return true;
  }

  /**
   * Step the state machine
   */
  update(dt, reqDir, egoLat, egoS, egoSpeed, totalLanes, perceivedObjects, track) {
    this.cooldown = Math.max(0, this.cooldown - dt);
    this.timer += dt;

    const currentLaneIdx = Math.round((egoLat - (track.laneLat ? track.laneLat('fwd', 0) : 0)) / 3.6);
    this.currentLane = clamp(currentLaneIdx, 0, totalLanes - 1);

    switch (this.state) {
      case LaneChangeState.KEEP_LANE:
        this.latPosition = track.laneLat ? track.laneLat('fwd', this.currentLane) : egoLat;
        if (reqDir !== 0 && this.cooldown <= 0) {
          const desiredLane = this.currentLane + reqDir;
          if (desiredLane >= 0 && desiredLane < totalLanes) {
            this.targetLane = desiredLane;
            this.state = LaneChangeState.PREPARE;
            this.timer = 0;
          }
        }
        break;

      case LaneChangeState.PREPARE:
        this.latPosition = track.laneLat ? track.laneLat('fwd', this.currentLane) : egoLat;
        if (this.isLaneSafe(this.targetLane, perceivedObjects, egoS, egoLat, egoSpeed, track)) {
          this.state = LaneChangeState.LANE_CHANGE;
          this.timer = 0;
        } else if (this.timer > 3.0) {
          // Timeout waiting for gap
          this.state = LaneChangeState.ABORT;
        }
        break;

      case LaneChangeState.LANE_CHANGE: {
        const goalLat = track.laneLat ? track.laneLat('fwd', this.targetLane) : egoLat;
        const dir = Math.sign(goalLat - this.latPosition);
        this.latPosition += dir * this.lateralSpeed * dt;

        // Abort check if maneuver just started and obstacle suddenly enters
        const progress = Math.abs(egoLat - (track.laneLat ? track.laneLat('fwd', this.currentLane) : 0));
        if (progress < 1.0 && !this.isLaneSafe(this.targetLane, perceivedObjects, egoS, egoLat, egoSpeed, track)) {
          this.state = LaneChangeState.ABORT;
          break;
        }

        // Check completion
        if (dir === 0 || (dir > 0 && this.latPosition >= goalLat) || (dir < 0 && this.latPosition <= goalLat)) {
          this.latPosition = goalLat;
          this.state = LaneChangeState.COMPLETE;
          this.timer = 0;
        }
        break;
      }

      case LaneChangeState.COMPLETE:
        this.currentLane = this.targetLane;
        this.cooldown = 2.0;
        this.state = LaneChangeState.KEEP_LANE;
        break;

      case LaneChangeState.ABORT: {
        const originalLat = track.laneLat ? track.laneLat('fwd', this.currentLane) : egoLat;
        const dir = Math.sign(originalLat - this.latPosition);
        this.latPosition += dir * this.lateralSpeed * 1.5 * dt; // Rapid return
        if (dir === 0 || (dir > 0 && this.latPosition >= originalLat) || (dir < 0 && this.latPosition <= originalLat)) {
          this.latPosition = originalLat;
          this.cooldown = 3.0;
          this.state = LaneChangeState.KEEP_LANE;
        }
        break;
      }
    }

    return {
      state: this.state,
      targetLateral: this.latPosition,
      active: this.state === LaneChangeState.LANE_CHANGE || this.state === LaneChangeState.ABORT,
      targetLane: this.targetLane
    };
  }
}

/**
 * ─── 5. INTEGRATED AUTONOMOUS DRIVING CONTROLLER ───
 * Coordinates the full deterministic driving stack:
 * Perception -> Decision -> Stanley Steering -> PID Speed -> Safety Shield
 */
export class AutonomousDrivingStack {
  constructor(config = {}) {
    this.lka = new LaneKeepController({ dt: 1.0 / 30.0, kDamp: 0.12, steerPositive: 'left' });
    this.pid = new LongitudinalPID();
    this.idm = new IDMController();
    this.gov = new CurvatureGovernor();
    this.laneMachine = new LaneChangeStateMachine();
    this.shield = makeAEBShield(1.5, 2.0);
  }

  reset() {
    this.lka.reset(0.0);
    this.pid.reset();
    this.laneMachine.reset();
  }

  /**
   * Execute full control step
   * @param {Object} context - All sensor, vehicle, track, and ADAS state
   * @returns {{ steer: number, throttle: number, brake: number, alert: string, severity: number }}
   */
  step(context) {
    const {
      egoState,
      track,
      perceivedObjects = [],
      proximityRays = [],
      baseCruiseSpeed = 14.0,
      laneRequest = 0,
      dt = 1.0 / 30.0,
      adas = { lka: true, aeb: true, gov: true, alc: true }
    } = context;

    const u = Math.max(egoState.u ?? 0, 0.0);
    const egoS = egoState.s ?? 0;
    const egoLat = egoState.lat ?? 0;
    const egoPsi = egoState.psi ?? 0;
    const roadPsi = egoState.roadPsi ?? 0;
    const kappa = egoState.kappa ?? 0;
    const omega = egoState.om ?? 0;
    const totalLanes = track.def ? track.def.lanesF : 2;

    // ── 1. Lane-Change Decision ──
    let targetLat = egoLat;
    let lcActive = false;
    if (adas.alc) {
      const lcResult = this.laneMachine.update(
        dt, laneRequest, egoLat, egoS, u, totalLanes, perceivedObjects, track
      );
      targetLat = lcResult.targetLateral;
      lcActive = lcResult.active;
    } else {
      const curLane = Math.round((egoLat - (track.laneLat ? track.laneLat('fwd', 0) : 0)) / 3.6);
      targetLat = track.laneLat ? track.laneLat('fwd', clamp(curLane, 0, totalLanes - 1)) : egoLat;
    }

    // ── 2. Curvature-Aware Speed Envelope ──
    let targetSpeed = baseCruiseSpeed;
    if (adas.gov) {
      const govResult = this.gov.evaluate(track, egoS, u, baseCruiseSpeed);
      targetSpeed = Math.min(targetSpeed, govResult.safeSpeed);
    }

    // ── 3. Lead Vehicle Following (IDM) ──
    let nearestLead = null;
    let minLeadDist = Infinity;
    for (const obj of perceivedObjects) {
      if ((obj.vAlong ?? obj.speed ?? 0) < -0.5) continue; // Skip oncoming
      if (Math.abs(obj.lat - targetLat) > 2.0) continue;     // In target corridor
      const dist = track.wrapS ? track.wrapS(obj.s - egoS) : (obj.s - egoS);
      if (dist > 1.0 && dist < minLeadDist) {
        minLeadDist = dist;
        nearestLead = obj;
      }
    }

    let isLeadBraking = false;
    let leadRelSpeed = 0.0;
    if (nearestLead) {
      leadRelSpeed = (nearestLead.vAlong ?? nearestLead.speed ?? 0) - u;
      isLeadBraking = nearestLead.braking || false;
      const idmResult = this.idm.evaluate(u, targetSpeed, minLeadDist, leadRelSpeed, isLeadBraking);
      targetSpeed = Math.min(targetSpeed, idmResult.targetSpeed);
    }

    // ── 4. Lateral Steering (Stanley Controller) ──
    const eLat = egoLat - targetLat;
    const ePsi = wrapAngle(egoPsi - roadPsi);
    let steerCmd = this.lka.update(eLat, ePsi, kappa, u, omega);

    // ── 5. Longitudinal Control (Anti-Windup PID) ──
    const pidResult = this.pid.update(u, targetSpeed, dt);

    // ── 6. Deterministic Safety Override Shield (AEB + Blind Spot) ──
    const candidateCtrl = {
      steer: steerCmd,
      throttle: pidResult.throttle,
      brake: pidResult.brake
    };

    const closingSpeed = Math.max(u, 0.5);
    const ttc = minLeadDist < 100.0 ? minLeadDist / closingSpeed : Infinity;

    let finalControl = candidateCtrl;
    if (adas.aeb && proximityRays.length >= 36) {
      finalControl = this.shield(candidateCtrl, proximityRays, u, ttc);
    }

    return {
      steer: finalControl.steer,
      throttle: finalControl.throttle,
      brake: finalControl.brake,
      targetSpeed,
      alert: finalControl.alert || (lcActive ? 'LANE CHANGE ACTIVE' : 'CRUISE'),
      severity: finalControl.severity || 0
    };
  }
}
