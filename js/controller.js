/**
 * js/controller.js — Complete Deterministic Predictive Autonomous Driving Stack
 * 
 * Implements:
 *   1. True 2D Closing Velocity & Multi-Stage TTC Prediction
 *   2. Kinematic Stopping-Distance Prediction Model
 *   3. Predictive Curvature Speed Governor (Pre-Curve Deceleration)
 *   4. Intelligent Driver Model (IDM) Car Following
 *   5. Unified Hierarchical Safe-Speed Governor
 *   6. Multi-Stage Progressive Longitudinal Controller (Anti-Windup PID + Brake Modulator)
 *   7. Curvature-Lookahead Stanley Path Follower (No Oscillation)
 *   8. Deterministic Lane Change & Obstacle Avoidance State Machine
 *   9. Last-Resort AEB Safety Override Shield
 * 
 * NOTE: RL is 100% untouched and independent.
 */

import { clamp, wrapAngle } from './utils.js';
import { LaneKeepController } from './lka.js';
import { makeAEBShield } from './aeb.js';

const GRAV = 9.81;

/**
 * ─── 1. PREDICTIVE COLLISION & CLOSING VELOCITY CALCULATOR ───
 */
export function calculateClosingDynamics(ego, target) {
  // 2D distance vector
  const dx = (target.wx ?? target.x ?? 0) - (ego.wx ?? ego.x ?? 0);
  const dz = (target.wz ?? target.z ?? 0) - (ego.wz ?? ego.z ?? 0);
  const dist = Math.hypot(dx, dz);

  if (dist < 1e-3) {
    return { dist: 0, closingVelocity: 0, ttc: Infinity, isClosing: false };
  }

  // Unit vector from ego toward target
  const uDirX = dx / dist;
  const uDirZ = dz / dist;

  // 2D velocities
  const egoVx = -(ego.u ?? 0) * Math.sin(ego.psi ?? 0);
  const egoVz = -(ego.u ?? 0) * Math.cos(ego.psi ?? 0);

  const targetHeading = target.heading ?? target.psi ?? ego.psi ?? 0;
  const targetSpeed = target.vAlong ?? target.speed ?? target.v ?? 0;
  const targetVx = -targetSpeed * Math.sin(targetHeading);
  const targetVz = -targetSpeed * Math.cos(targetHeading);

  // Relative velocity (ego closing on target)
  const relVx = egoVx - targetVx;
  const relVz = egoVz - targetVz;

  // Closing velocity = dot(v_rel, u_dir)
  const closingVelocity = relVx * uDirX + relVz * uDirZ;
  const isClosing = closingVelocity > 0.15;
  const ttc = isClosing ? dist / closingVelocity : Infinity;

  return { dist, closingVelocity, ttc, isClosing };
}

/**
 * ─── 2. STOPPING-DISTANCE PREDICTION MODEL ───
 * reactionDistance = v * t_react
 * brakingDistance = (v_ego^2 - v_lead^2) / (2 * a_comf)
 * requiredDistance = reactionDistance + brakingDistance + safetyMargin
 */
export class StoppingDistanceModel {
  constructor(config = {}) {
    this.reactionTime = config.reactionTime ?? 0.65; // s (autonomous perception + actuation delay)
    this.comfDecel = config.comfDecel ?? 2.8;       // m/s² (comfortable deceleration)
    this.safetyMargin = config.safetyMargin ?? 6.0;   // m (standstill safety buffer)
  }

  calculateRequiredDistance(egoSpeed, targetSpeed = 0) {
    egoSpeed = Math.max(0, egoSpeed);
    targetSpeed = Math.max(0, targetSpeed);
    const reactDist = egoSpeed * this.reactionTime;
    const brakeDist = Math.max(0, egoSpeed * egoSpeed - targetSpeed * targetSpeed) / (2 * this.comfDecel);
    return reactDist + brakeDist + this.safetyMargin;
  }

  /**
   * Calculates maximum safe speed to be able to stop comfortably within availableDistance
   */
  calculateSafeSpeed(availableDistance, targetSpeed = 0) {
    const dAvail = Math.max(0, availableDistance - this.safetyMargin);
    if (dAvail <= 0.5) return 0.0;

    // Solve: v * t_react + (v^2 - v_tar^2) / (2*a) = dAvail
    // v^2 + 2*a*t_react*v - (2*a*dAvail + v_tar^2) = 0
    const a = this.comfDecel;
    const tr = this.reactionTime;
    const c = 2 * a * dAvail + targetSpeed * targetSpeed;
    const disc = (2 * a * tr) * (2 * a * tr) + 4 * c;
    const vSafe = (-2 * a * tr + Math.sqrt(Math.max(0, disc))) / 2;
    return Math.max(0, vSafe);
  }
}

/**
 * ─── 3. PROACTIVE CURVATURE SPEED GOVERNOR ───
 * Computes speed ceiling along lookahead envelope before entering curves:
 * v_safe(d) = sqrt(v_curve^2 + 2 * a_comf * d)
 */
export class CurvatureGovernor {
  constructor(config = {}) {
    this.mu = config.mu ?? 0.88;
    this.latAccLimit = config.latAcc ?? 3.8;       // Max lateral acceleration (m/s²)
    this.comfDecel = config.comfDecel ?? 2.4;     // Comfortable approach deceleration (m/s²)
    this.lookaheadDist = config.lookahead ?? 90.0;// Max lookahead (m)
  }

  evaluate(track, egoS, currentSpeed, cruiseSpeed) {
    if (!track || typeof track.kappaAt !== 'function') {
      return { safeSpeed: cruiseSpeed, upcomingCurvature: 0, lookaheadCurvature: 0, governorActive: false };
    }

    let minAllowedSpeed = cruiseSpeed;
    let maxUpcomingK = 0;
    let governorActive = false;

    // Adaptive lookahead distance scaling with speed: D = max(35, v * 3.5 + 20)
    const scanEnd = Math.min(this.lookaheadDist, Math.max(35, currentSpeed * 3.6 + 25));
    const dLook = Math.max(8.0, currentSpeed * 0.65);
    const lookaheadCurvature = track.kappaAt ? Math.abs(track.kappaAt((egoS + dLook) % (track.L || 1000))) : 0;

    for (let d = 4; d <= scanEnd; d += 5) {
      const sAhead = (egoS + d) % (track.L || 1000);
      const k = Math.abs(track.kappaAt(sAhead));
      if (k < 1e-4) continue;

      if (k > maxUpcomingK) maxUpcomingK = k;

      // Safe steady-state curve speed
      const vCurve = Math.sqrt(Math.min(this.latAccLimit, this.mu * GRAV * 0.55) / k);
      // Safe approach speed allowing comfortable deceleration over distance d
      const vApproach = Math.sqrt(vCurve * vCurve + 2 * this.comfDecel * d);

      if (vApproach < minAllowedSpeed) {
        minAllowedSpeed = vApproach;
        governorActive = true;
      }
    }

    return {
      safeSpeed: Math.max(3.5, Math.min(cruiseSpeed, minAllowedSpeed)),
      upcomingCurvature: maxUpcomingK,
      lookaheadCurvature,
      governorActive: governorActive && minAllowedSpeed < cruiseSpeed - 1.2
    };
  }
}

/**
 * ─── 4. INTELLIGENT DRIVER MODEL (IDM) WITH MULTI-STAGE TTC ───
 */
export class IDMController {
  constructor(config = {}) {
    this.s0 = config.s0 ?? 6.0;               // Minimum standstill gap (m)
    this.timeHeadway = config.timeHeadway ?? 1.5; // Desired time headway (s)
    this.maxAccel = config.maxAccel ?? 2.8;   // Comfortable acceleration (m/s²)
    this.comfDecel = config.comfDecel ?? 3.2; // Comfortable deceleration (m/s²)
    this.delta = config.delta ?? 4.0;
  }

  evaluate(egoSpeed, cruiseSpeed, leadDist, leadRelSpeed, isLeadBraking = false, ttc = Infinity) {
    egoSpeed = Math.max(0.01, egoSpeed);
    cruiseSpeed = Math.max(0.01, cruiseSpeed);

    if (!Number.isFinite(leadDist) || leadDist > 180.0) {
      return { targetSpeed: cruiseSpeed, desiredGap: this.s0, stage: 'FREE' };
    }

    const deltaV = -leadRelSpeed; // Positive when ego is closing on lead
    const sStar = this.s0 + egoSpeed * this.timeHeadway + (egoSpeed * deltaV) / (2 * Math.sqrt(this.maxAccel * this.comfDecel));
    const effectiveGap = Math.max(0.5, leadDist);
    const interactionTerm = Math.pow(sStar / effectiveGap, 2);

    let targetSpeed = cruiseSpeed;
    const leadSpeed = Math.max(0, egoSpeed + leadRelSpeed);

    // Progressive Multi-Stage Following
    let stage = 'FOLLOW';
    if (ttc < 1.8 || leadDist < 7.0) {
      // Critical emergency zone
      targetSpeed = 0.0;
      stage = 'CRITICAL';
    } else if (ttc < 3.0 || leadDist < 12.0) {
      // Strong braking zone
      targetSpeed = Math.min(targetSpeed, Math.max(0, leadSpeed * 0.5));
      stage = 'STRONG_BRAKE';
    } else if (ttc < 5.0 || leadDist < sStar) {
      // Moderate deceleration zone
      targetSpeed = Math.min(targetSpeed, Math.max(0, leadSpeed + 0.35 * (effectiveGap - sStar)));
      stage = 'DECELERATE';
    } else if (interactionTerm > 0.04) {
      // Standard headway tracking
      targetSpeed = Math.min(cruiseSpeed, Math.max(0, leadSpeed + 0.5 * (effectiveGap - sStar)));
      stage = 'FOLLOW';
    }

    // Early reaction to lead vehicle V2V brake broadcast
    if (isLeadBraking && leadDist < 60.0) {
      targetSpeed = Math.min(targetSpeed, egoSpeed * 0.65);
    }

    return { targetSpeed: Math.max(0, targetSpeed), desiredGap: sStar, stage };
  }
}

/**
 * ─── 5. LONGITUDINAL ANTI-WINDUP PID & ACTUATION MODULATOR ───
 */
export class LongitudinalPID {
  constructor(config = {}) {
    this.kp = config.kp ?? 0.52;
    this.ki = config.ki ?? 0.07;
    this.kd = config.kd ?? 0.05;
    this.integralLimit = config.integralLimit ?? 4.0;
    this.maxAccel = config.maxAccel ?? 3.4;
    this.maxDecel = config.maxDecel ?? 7.5;
    this.deadband = config.deadband ?? 0.20;

    this.integral = 0.0;
    this.prevSpeed = 0.0;
    this.throttle = 0.0;
    this.brake = 0.0;
  }

  reset() {
    this.integral = 0.0;
    this.prevSpeed = 0.0;
    this.throttle = 0.0;
    this.brake = 0.0;
  }

  update(currentSpeed, targetSpeed, dt, forceBrake = 0) {
    if (dt <= 0) return { throttle: this.throttle, brake: this.brake, accelCmd: 0 };
    currentSpeed = Math.max(0, currentSpeed);
    targetSpeed = Math.max(0, targetSpeed);

    if (forceBrake > 0) {
      this.throttle = 0.0;
      this.brake = Math.max(this.brake, forceBrake);
      this.integral = 0.0;
      return { throttle: this.throttle, brake: this.brake, accelCmd: -this.maxDecel * forceBrake };
    }

    const error = targetSpeed - currentSpeed;

    // Integral with anti-windup clamping
    if (Math.abs(error) > this.deadband) {
      this.integral += error * dt;
      this.integral = clamp(this.integral, -this.integralLimit, this.integralLimit);
    } else {
      this.integral *= 0.94;
    }

    const dSpeed = (currentSpeed - this.prevSpeed) / dt;
    this.prevSpeed = currentSpeed;

    const rawAccel = this.kp * error + this.ki * this.integral - this.kd * dSpeed;
    const accelCmd = clamp(rawAccel, -this.maxDecel, this.maxAccel);

    let targetThrottle = 0.0;
    let targetBrake = 0.0;

    if (accelCmd > 0.12) {
      targetThrottle = clamp(accelCmd / this.maxAccel, 0.0, 1.0);
      targetBrake = 0.0;
    } else if (accelCmd < -0.32) {
      targetThrottle = 0.0;
      targetBrake = clamp(-accelCmd / this.maxDecel, 0.0, 1.0);
    } else {
      targetThrottle = 0.04; // Idle roll
      targetBrake = 0.0;
    }

    // Actuator slew rate limits
    const maxThrottleRate = 2.8 * dt;
    const maxBrakeRate = 6.0 * dt;
    this.throttle += clamp(targetThrottle - this.throttle, -maxThrottleRate * 1.5, maxThrottleRate);
    this.brake += clamp(targetBrake - this.brake, -maxBrakeRate, maxBrakeRate);

    this.throttle = clamp(this.throttle, 0.0, 1.0);
    this.brake = clamp(this.brake, 0.0, 1.0);

    return { throttle: this.throttle, brake: this.brake, accelCmd };
  }
}

/**
 * ─── 6. DETERMINISTIC LANE CHANGE & OBSTACLE AVOIDANCE STATE MACHINE ───
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
    this.lateralSpeed = config.latSpeed ?? 1.45;
    this.minFrontGap = config.minFrontGap ?? 25.0; // m
    this.minRearGap = config.minRearGap ?? 18.0;   // m
    this.minTTC = config.minTTC ?? 3.2;           // s
  }

  reset() {
    this.state = LaneChangeState.KEEP_LANE;
    this.currentLane = 0;
    this.targetLane = 0;
    this.latPosition = 0.0;
    this.timer = 0.0;
    this.cooldown = 0.0;
  }

  isLaneSafe(targetLane, perceivedObjects, egoS, egoLat, egoSpeed, track) {
    if (!track || !track.laneLat) return true;
    const targetCenter = track.laneLat('fwd', targetLane);

    for (const obj of perceivedObjects || []) {
      if (Math.abs(obj.lat - targetCenter) > 2.0) continue;

      const relS = track.wrapS ? track.wrapS(obj.s - egoS) : (obj.s - egoS);
      const relV = (obj.vAlong ?? obj.speed ?? 0) - egoSpeed;

      // Front safety gap
      if (relS >= 0 && relS < this.minFrontGap) return false;

      // Rear safety gap
      if (relS < 0 && relS > -this.minRearGap) return false;

      // Dynamic closing TTC
      if (relS > 0 && relV < -0.5) {
        const ttc = relS / Math.abs(relV);
        if (ttc < this.minTTC) return false;
      }
    }
    return true;
  }

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
          this.state = LaneChangeState.ABORT;
        }
        break;

      case LaneChangeState.LANE_CHANGE: {
        const goalLat = track.laneLat ? track.laneLat('fwd', this.targetLane) : egoLat;
        const dir = Math.sign(goalLat - this.latPosition);
        this.latPosition += dir * this.lateralSpeed * dt;

        const progress = Math.abs(egoLat - (track.laneLat ? track.laneLat('fwd', this.currentLane) : 0));
        if (progress < 1.0 && !this.isLaneSafe(this.targetLane, perceivedObjects, egoS, egoLat, egoSpeed, track)) {
          this.state = LaneChangeState.ABORT;
          break;
        }

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
        const origLat = track.laneLat ? track.laneLat('fwd', this.currentLane) : egoLat;
        const dir = Math.sign(origLat - this.latPosition);
        this.latPosition += dir * this.lateralSpeed * 1.5 * dt;
        if (dir === 0 || (dir > 0 && this.latPosition >= origLat) || (dir < 0 && this.latPosition <= origLat)) {
          this.latPosition = origLat;
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
 * ─── 7. COMPLETE AUTONOMOUS DRIVING CONTROLLER STACK ───
 */
export class AutonomousDrivingStack {
  constructor(config = {}) {
    this.lka = new LaneKeepController({ dt: 1.0 / 30.0, kDamp: 0.12, steerPositive: 'left' });
    this.pid = new LongitudinalPID();
    this.idm = new IDMController();
    this.gov = new CurvatureGovernor();
    this.stopModel = new StoppingDistanceModel();
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

    // ── 1. Target Lane & Obstacle / Slow-Vehicle Lane Change Decision ──
    let targetLat = egoLat;
    let lcActive = false;
    let autoLaneReq = laneRequest;

    // Evaluate leading vehicle in current lane for automatic safe lane change
    const curLane = Math.round((egoLat - (track.laneLat ? track.laneLat('fwd', 0) : 0)) / 3.6);
    const curLaneCenter = track.laneLat ? track.laneLat('fwd', curLane) : egoLat;

    let leadInCurrentLane = null;
    let leadInCurrentDist = Infinity;
    for (const obj of perceivedObjects) {
      if ((obj.vAlong ?? obj.speed ?? 0) < -0.5) continue;
      if (Math.abs(obj.lat - curLaneCenter) > 2.0) continue;
      const d = track.wrapS ? track.wrapS(obj.s - egoS) : (obj.s - egoS);
      if (d > 1.0 && d < leadInCurrentDist) {
        leadInCurrentDist = d;
        leadInCurrentLane = obj;
      }
    }

    // Auto lane change decision if autoPass enabled, slower vehicle detected, and safe adjacent lane exists
    if (adas.alc && adas.autoPass && autoLaneReq === 0 && leadInCurrentLane && leadInCurrentDist < 65.0) {
      const leadSpeed = leadInCurrentLane.vAlong ?? leadInCurrentLane.speed ?? 0;
      if (leadSpeed < baseCruiseSpeed - 2.5) {
        // Check adjacent lanes
        if (curLane > 0 && this.laneMachine.isLaneSafe(curLane - 1, perceivedObjects, egoS, egoLat, u, track)) {
          autoLaneReq = -1;
        } else if (curLane + 1 < totalLanes && this.laneMachine.isLaneSafe(curLane + 1, perceivedObjects, egoS, egoLat, u, track)) {
          autoLaneReq = 1;
        }
      }
    }

    if (adas.alc) {
      const lcResult = this.laneMachine.update(
        dt, autoLaneReq, egoLat, egoS, u, totalLanes, perceivedObjects, track
      );
      targetLat = lcResult.targetLateral;
      lcActive = lcResult.active;
    } else {
      targetLat = track.laneLat ? track.laneLat('fwd', clamp(curLane, 0, totalLanes - 1)) : egoLat;
    }

    // ── 2. Unified Safe-Speed Governor ──
    // safeSpeed = min(roadSpeed, curveSpeed, followingSpeed, stoppingDistanceSpeed, ttcSpeed)
    let safeSpeed = baseCruiseSpeed;

    // A. Curvature Governor (Decelerate smoothly BEFORE curve entry)
    let lookaheadK = kappa;
    if (adas.gov) {
      const govResult = this.gov.evaluate(track, egoS, u, baseCruiseSpeed);
      safeSpeed = Math.min(safeSpeed, govResult.safeSpeed);
      lookaheadK = Math.max(kappa, govResult.lookaheadCurvature);
    }

    // B. Lead Vehicle Following (IDM + 2D Closing Dynamics + Stopping Distance)
    let nearestLead = null;
    let minLeadDist = Infinity;
    let minLeadTTC = Infinity;
    let minLeadClosingV = 0;

    for (const obj of perceivedObjects) {
      if ((obj.vAlong ?? obj.speed ?? 0) < -0.5) continue;
      if (Math.abs(obj.lat - targetLat) > 2.0) continue;
      const d = track.wrapS ? track.wrapS(obj.s - egoS) : (obj.s - egoS);
      if (d > 0.5 && d < minLeadDist) {
        minLeadDist = d;
        nearestLead = obj;

        const dyn2D = calculateClosingDynamics(egoState, obj);
        minLeadTTC = dyn2D.ttc;
        minLeadClosingV = dyn2D.closingVelocity;
      }
    }

    let isLeadBraking = false;
    let leadRelSpeed = 0.0;
    if (nearestLead) {
      leadRelSpeed = (nearestLead.vAlong ?? nearestLead.speed ?? 0) - u;
      isLeadBraking = nearestLead.braking || false;

      // IDM Target Speed
      const idmResult = this.idm.evaluate(u, safeSpeed, minLeadDist, leadRelSpeed, isLeadBraking, minLeadTTC);
      safeSpeed = Math.min(safeSpeed, idmResult.targetSpeed);

      // Stopping Distance Model Speed Ceiling
      const stopSpeedCeil = this.stopModel.calculateSafeSpeed(minLeadDist, Math.max(0, u + leadRelSpeed));
      safeSpeed = Math.min(safeSpeed, stopSpeedCeil);
    }

    // ── 3. Multi-Stage Progressive Braking & Control Output ──
    let forceBrake = 0.0;
    let alertMsg = lcActive ? 'LANE CHANGE ACTIVE' : 'CRUISE';
    let alertSev = 0;

    if (minLeadDist < 6.5 || minLeadTTC < 1.35) {
      // Last-resort emergency stop
      forceBrake = 1.0;
      safeSpeed = 0.0;
      alertMsg = 'AEB · CRITICAL DISTANCE — EMERGENCY STOP';
      alertSev = 2;
    } else if (minLeadTTC < 2.5 || minLeadDist < 12.0) {
      // Strong progressive deceleration
      forceBrake = clamp((2.5 - minLeadTTC) / 1.5, 0.45, 0.85);
      alertMsg = 'FCW · IMMINENT COLLISION RISK — BRAKING';
      alertSev = 2;
    } else if (minLeadTTC < 4.5 || (isLeadBraking && minLeadDist < 45.0)) {
      // Moderate deceleration
      forceBrake = clamp((4.5 - minLeadTTC) / 2.5, 0.15, 0.45);
      alertMsg = 'ACC · DECELERATING FOR TRAFFIC';
      alertSev = 1;
    }

    // ── 4. Longitudinal Controller (PID with Anti-Windup) ──
    const pidResult = this.pid.update(u, safeSpeed, dt, forceBrake);

    // ── 5. Curvature-Lookahead Stanley Path Follower ──
    const eLat = egoLat - targetLat;
    const ePsi = wrapAngle(egoPsi - roadPsi);
    const steerCmd = this.lka.update(eLat, ePsi, lookaheadK, u, omega);

    // ── 6. Deterministic 360° Safety Shield (Blind Spot Protection) ──
    const candidateCtrl = {
      steer: steerCmd,
      throttle: pidResult.throttle,
      brake: pidResult.brake
    };

    let finalControl = candidateCtrl;
    if (adas.aeb && proximityRays.length >= 36) {
      finalControl = this.shield(candidateCtrl, proximityRays, u, minLeadTTC);
    }

    return {
      steer: finalControl.steer,
      throttle: finalControl.throttle,
      brake: finalControl.brake,
      targetSpeed: safeSpeed,
      alert: finalControl.alert !== 'NONE' ? finalControl.alert : alertMsg,
      severity: Math.max(alertSev, finalControl.severity || 0)
    };
  }
}
