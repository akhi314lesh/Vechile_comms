/**
 * js/controller.js — Complete Deterministic Autonomous Driving Stack
 * 
 * Dynamic Speed Behavior & Safety Architecture:
 *   - Absolute Maximum Speed Hard Ceiling: 60 km/h (16.6667 m/s)
 *   - Hierarchical Speed Planning: min(cruise, road, curve, traffic, closing, stoppingDist, signal, obstacle)
 *   - Smooth Dynamic Rate-Limited Speed Transitions (No Step Jumps)
 *   - Early Deceleration for Distant Closing Traffic
 *   - Pre-Curve Deceleration and Smooth Post-Curve Re-Acceleration
 *   - Intelligent Multi-Lane Traffic Utility Selection
 *   - Post-Curve Stabilized Stanley Path Follower (Zero Post-Turn Zigzag)
 *   - Last-Resort AEB Safety Override Shield
 * 
 * NOTE: RL is 100% untouched.
 */

import { clamp, wrapAngle } from './utils.js';
import { LaneKeepController } from './lka.js';
import { makeAEBShield } from './aeb.js';

const GRAV = 9.81;

/** Absolute Hard Speed Ceiling: 80 km/h */
export const MAX_AUTONOMOUS_SPEED = 80.0 / 3.6; // 22.2222 m/s (80 km/h)

/**
 * ─── 1. PREDICTIVE COLLISION & CLOSING VELOCITY CALCULATOR ───
 */
export function calculateClosingDynamics(ego, target) {
  const dx = (target.wx ?? target.x ?? 0) - (ego.wx ?? ego.x ?? 0);
  const dz = (target.wz ?? target.z ?? 0) - (ego.wz ?? ego.z ?? 0);
  const dist = Math.hypot(dx, dz);

  if (dist < 1e-3) {
    return { dist: 0, closingVelocity: 0, ttc: Infinity, isClosing: false };
  }

  const uDirX = dx / dist;
  const uDirZ = dz / dist;

  const egoVx = -(ego.u ?? 0) * Math.sin(ego.psi ?? 0);
  const egoVz = -(ego.u ?? 0) * Math.cos(ego.psi ?? 0);

  const targetHeading = target.heading ?? target.psi ?? ego.psi ?? 0;
  const targetSpeed = target.vAlong ?? target.speed ?? target.v ?? 0;
  const targetVx = -targetSpeed * Math.sin(targetHeading);
  const targetVz = -targetSpeed * Math.cos(targetHeading);

  const relVx = egoVx - targetVx;
  const relVz = egoVz - targetVz;

  const closingVelocity = relVx * uDirX + relVz * uDirZ;
  const isClosing = closingVelocity > 0.15;
  const ttc = isClosing ? dist / closingVelocity : Infinity;

  return { dist, closingVelocity, ttc, isClosing };
}

/**
 * ─── 2. STOPPING-DISTANCE & DISTANT CLOSING PREDICTION MODEL ───
 */
export class StoppingDistanceModel {
  constructor(config = {}) {
    this.reactionTime = config.reactionTime ?? 0.65; // s (autonomous system latency)
    this.comfDecel = config.comfDecel ?? 2.8;       // m/s² (comfortable deceleration)
    this.safetyMargin = config.safetyMargin ?? 6.0;   // m (standstill buffer)
  }

  calculateRequiredDistance(egoSpeed, targetSpeed = 0) {
    egoSpeed = Math.max(0, egoSpeed);
    targetSpeed = Math.max(0, targetSpeed);
    const reactDist = egoSpeed * this.reactionTime;
    const brakeDist = Math.max(0, egoSpeed * egoSpeed - targetSpeed * targetSpeed) / (2 * this.comfDecel);
    return reactDist + brakeDist + this.safetyMargin;
  }

  /**
   * Calculates maximum safe speed to stop comfortably within available distance
   */
  calculateSafeSpeed(availableDistance, targetSpeed = 0) {
    const dAvail = Math.max(0, availableDistance - this.safetyMargin);
    if (dAvail <= 0.5) return 0.0;

    const a = this.comfDecel;
    const tr = this.reactionTime;
    const c = 2 * a * dAvail + targetSpeed * targetSpeed;
    const disc = (2 * a * tr) * (2 * a * tr) + 4 * c;
    const vSafe = (-2 * a * tr + Math.sqrt(Math.max(0, disc))) / 2;
    return Math.max(0, vSafe);
  }

  /**
   * Calculates dynamic speed ceiling for distant closing traffic (e.g. 50-120m away)
   * Prevents accelerating when closing on slower vehicles ahead.
   */
  calculateClosingSpeedCeiling(leadDist, leadSpeed) {
    leadDist = Math.max(0, leadDist);
    leadSpeed = Math.max(0, leadSpeed);
    const desiredGap = this.safetyMargin + leadSpeed * 1.5;
    const availableClosingDist = Math.max(0, leadDist - desiredGap);
    // v_catch = v_lead + sqrt(2 * a_comf * d_avail)
    const vClosingCap = leadSpeed + Math.sqrt(2 * this.comfDecel * availableClosingDist);
    return Math.min(MAX_AUTONOMOUS_SPEED, vClosingCap);
  }
}

/**
 * ─── 3. PROACTIVE CURVATURE SPEED GOVERNOR ───
 */
export class CurvatureGovernor {
  constructor(config = {}) {
    this.mu = config.mu ?? 0.88;
    this.latAccLimit = config.latAcc ?? 3.2;       // Max comfortable lateral acceleration (m/s²)
    this.comfDecel = config.comfDecel ?? 2.8;     // Comfortable approach deceleration (m/s²)
    this.lookaheadDist = config.lookahead ?? 110.0;// Max lookahead (m)
  }

  evaluate(track, egoS, currentSpeed, cruiseSpeed) {
    const baseSpeed = Math.min(MAX_AUTONOMOUS_SPEED, cruiseSpeed);
    if (!track || typeof track.kappaAt !== 'function') {
      return { safeSpeed: baseSpeed, upcomingCurvature: 0, lookaheadK: 0, governorActive: false };
    }

    let minAllowedSpeed = baseSpeed;
    let maxUpcomingK = 0;
    let governorActive = false;

    const scanEnd = Math.min(this.lookaheadDist, Math.max(40, currentSpeed * 3.8 + 30));
    const dLook = Math.max(8.0, currentSpeed * 0.70);
    const lookaheadK = track.kappaAt ? Math.abs(track.kappaAt((egoS + dLook) % (track.L || 1000))) : 0;

    for (let d = 2; d <= scanEnd; d += 3) {
      const sAhead = (egoS + d) % (track.L || 1000);
      const k = Math.abs(track.kappaAt(sAhead));
      if (k < 1e-4) continue;

      if (k > maxUpcomingK) maxUpcomingK = k;

      const vCurve = Math.sqrt(Math.min(this.latAccLimit, this.mu * GRAV * 0.50) / k);
      const vApproach = Math.sqrt(vCurve * vCurve + 2 * this.comfDecel * d);

      if (vApproach < minAllowedSpeed) {
        minAllowedSpeed = vApproach;
        governorActive = true;
      }
    }

    return {
      safeSpeed: Math.max(3.5, Math.min(baseSpeed, minAllowedSpeed)),
      upcomingCurvature: maxUpcomingK,
      lookaheadK,
      governorActive: governorActive && minAllowedSpeed < baseSpeed - 1.0
    };
  }
}

/**
 * ─── 4. INTELLIGENT DRIVER MODEL (IDM) ───
 */
export class IDMController {
  constructor(config = {}) {
    this.s0 = config.s0 ?? 6.0;               // Standstill buffer (m)
    this.timeHeadway = config.timeHeadway ?? 1.5; // Desired headway (s)
    this.maxAccel = config.maxAccel ?? 2.4;   // Comfortable acceleration (m/s²)
    this.comfDecel = config.comfDecel ?? 3.0; // Comfortable deceleration (m/s²)
    this.delta = config.delta ?? 4.0;
  }

  evaluate(egoSpeed, cruiseSpeed, leadDist, leadRelSpeed, isLeadBraking = false, ttc = Infinity) {
    egoSpeed = Math.max(0.01, egoSpeed);
    cruiseSpeed = Math.min(MAX_AUTONOMOUS_SPEED, Math.max(0.01, cruiseSpeed));

    if (!Number.isFinite(leadDist) || leadDist > 180.0) {
      return { targetSpeed: cruiseSpeed, desiredGap: this.s0, stage: 'FREE' };
    }

    const deltaV = -leadRelSpeed;
    const sStar = this.s0 + egoSpeed * this.timeHeadway + (egoSpeed * deltaV) / (2 * Math.sqrt(this.maxAccel * this.comfDecel));
    const effectiveGap = Math.max(0.5, leadDist);
    const interactionTerm = Math.pow(sStar / effectiveGap, 2);

    let targetSpeed = cruiseSpeed;
    const leadSpeed = Math.max(0, egoSpeed + leadRelSpeed);

    let stage = 'FOLLOW';
    if (ttc < 1.8 || leadDist < 7.0) {
      targetSpeed = 0.0;
      stage = 'CRITICAL';
    } else if (ttc < 3.0 || leadDist < 12.0) {
      targetSpeed = Math.min(targetSpeed, Math.max(0, leadSpeed * 0.5));
      stage = 'STRONG_BRAKE';
    } else if (ttc < 5.0 || leadDist < sStar) {
      targetSpeed = Math.min(targetSpeed, Math.max(0, leadSpeed + 0.35 * (effectiveGap - sStar)));
      stage = 'DECELERATE';
    } else if (interactionTerm > 0.04) {
      targetSpeed = Math.min(cruiseSpeed, Math.max(0, leadSpeed + 0.5 * (effectiveGap - sStar)));
      stage = 'FOLLOW';
    }

    if (isLeadBraking && leadDist < 60.0) {
      targetSpeed = Math.min(targetSpeed, egoSpeed * 0.65);
    }

    return { targetSpeed: clamp(targetSpeed, 0, MAX_AUTONOMOUS_SPEED), desiredGap: sStar, stage };
  }
}

/**
 * ─── 5. LONGITUDINAL PID WITH SMOOTH DYNAMIC SPEED FILTER ───
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
    this.filteredTarget = 0.0;
    this.throttle = 0.0;
    this.brake = 0.0;
  }

  reset() {
    this.integral = 0.0;
    this.prevSpeed = 0.0;
    this.filteredTarget = 0.0;
    this.throttle = 0.0;
    this.brake = 0.0;
  }

  /**
   * Updates longitudinal controls with dynamic acceleration/deceleration smoothing
   */
  update(currentSpeed, rawTargetSpeed, dt, forceBrake = 0) {
    if (dt <= 0) return { throttle: this.throttle, brake: this.brake, accelCmd: 0, filteredTarget: this.filteredTarget };
    currentSpeed = Math.max(0, currentSpeed);
    
    // Absolute 60 km/h Hard Limit
    const targetSpeed = clamp(rawTargetSpeed, 0, MAX_AUTONOMOUS_SPEED);

    if (forceBrake > 0) {
      this.throttle = 0.0;
      this.brake = Math.max(this.brake, forceBrake);
      this.integral = 0.0;
      this.filteredTarget = currentSpeed;
      return { throttle: this.throttle, brake: this.brake, accelCmd: -this.maxDecel * forceBrake, filteredTarget: 0 };
    }

    // Dynamic Smooth Target Speed Profiling (No instant step jumps):
    // Smoothly ramp up at max 2.2 m/s² on empty road; ramp down at max 4.0 m/s² in traffic.
    if (this.filteredTarget === 0 && currentSpeed > 0) {
      this.filteredTarget = currentSpeed;
    }
    const maxSpeedInc = 2.2 * dt;
    const maxSpeedDec = 4.2 * dt;
    this.filteredTarget += clamp(targetSpeed - this.filteredTarget, -maxSpeedDec, maxSpeedInc);
    this.filteredTarget = clamp(this.filteredTarget, 0, MAX_AUTONOMOUS_SPEED);

    const error = this.filteredTarget - currentSpeed;

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
      targetThrottle = 0.04;
      targetBrake = 0.0;
    }

    const maxThrottleRate = 2.8 * dt;
    const maxBrakeRate = 6.0 * dt;
    this.throttle += clamp(targetThrottle - this.throttle, -maxThrottleRate * 1.5, maxThrottleRate);
    this.brake += clamp(targetBrake - this.brake, -maxBrakeRate, maxBrakeRate);

    this.throttle = clamp(this.throttle, 0.0, 1.0);
    this.brake = clamp(this.brake, 0.0, 1.0);

    return { throttle: this.throttle, brake: this.brake, accelCmd, filteredTarget: this.filteredTarget };
  }
}

/**
 * ─── 6. INTELLIGENT MULTI-LANE TRAFFIC UTILITY MODEL ───
 */
export class LaneUtilityModel {
  constructor(config = {}) {
    this.minImprovement = config.minImprovement ?? 0.18; // Hysteresis threshold
    this.minFrontGap = config.minFrontGap ?? 25.0;       // m
    this.minRearGap = config.minRearGap ?? 20.0;         // m
    this.minRearTTC = config.minRearTTC ?? 4.5;          // s (catches fast cars closing from behind)
  }

  evaluate(curLane, egoS, egoLat, egoSpeed, cruiseSpeed, totalLanes, perceivedObjects, track, upcomingCurvature = 0) {
    const laneStats = [];
    const effectiveCruise = Math.min(MAX_AUTONOMOUS_SPEED, cruiseSpeed);

    for (let laneIdx = 0; laneIdx < totalLanes; laneIdx++) {
      const laneCenter = track.laneLat ? track.laneLat('fwd', laneIdx) : laneIdx * 3.6;
      let count = 0;
      let frontLead = null, frontDist = Infinity, frontSpeed = effectiveCruise, frontTTC = Infinity;
      let rearLead = null, rearDist = Infinity, rearSpeed = 0, rearTTC = Infinity;
      let speedSum = 0;

      for (const obj of perceivedObjects || []) {
        const objLat = obj.lat ?? 0;
        if (Math.abs(objLat - laneCenter) > 1.9) continue;

        const relS = track.wrapS ? track.wrapS(obj.s - egoS) : (obj.s - egoS);
        const objSpeed = obj.vAlong ?? obj.speed ?? obj.v ?? 0;

        if (relS > -50 && relS < 130) {
          count++;
          speedSum += objSpeed;
        }

        if (relS > 0.5 && relS < frontDist) {
          frontDist = relS;
          frontLead = obj;
          frontSpeed = objSpeed;
          const closing = egoSpeed - objSpeed;
          frontTTC = closing > 0.2 ? relS / closing : Infinity;
        }

        if (relS < -0.5 && Math.abs(relS) < rearDist) {
          rearDist = Math.abs(relS);
          rearLead = obj;
          rearSpeed = objSpeed;
          const rearClosing = objSpeed - egoSpeed;
          rearTTC = rearClosing > 0.2 ? Math.abs(relS) / rearClosing : Infinity;
        }
      }

      const avgSpeed = count > 0 ? speedSum / count : effectiveCruise;

      const speedBenefit = clamp(avgSpeed / Math.max(5.0, effectiveCruise), 0.0, 1.0);
      const spaceBenefit = clamp(frontDist / 80.0, 0.0, 1.0);
      const densityBenefit = clamp(1.0 - (count / 5.0), 0.0, 1.0);

      let switchPenalty = (laneIdx !== curLane) ? 0.22 : 0.0;
      let curveRisk = Math.abs(upcomingCurvature) > 0.0075 ? 0.40 : 0.0;
      let rearFastRisk = (rearLead && rearTTC < this.minRearTTC) ? 1.0 : 0.0;

      const score = (0.42 * speedBenefit + 0.35 * spaceBenefit + 0.23 * densityBenefit)
                    - switchPenalty - curveRisk - rearFastRisk;

      let isSafe = true;
      let unsafeReason = 'NONE';

      if (frontDist < 18.0) {
        isSafe = false;
        unsafeReason = 'FRONT_TOO_CLOSE';
      } else if (frontDist < this.minFrontGap && (egoSpeed - frontSpeed) > -0.5) {
        isSafe = false;
        unsafeReason = 'FRONT_GAP_TOO_SMALL';
      } else if (rearDist < 18.0) {
        isSafe = false;
        unsafeReason = 'REAR_TOO_CLOSE';
      } else if (rearDist < this.minRearGap && (rearSpeed - egoSpeed) > -0.5) {
        isSafe = false;
        unsafeReason = 'REAR_GAP_TOO_SMALL';
      } else if (rearTTC < this.minRearTTC) {
        isSafe = false;
        unsafeReason = 'FAST_REAR_VEHICLE_APPROACHING';
      } else if (frontTTC < 2.5) {
        isSafe = false;
        unsafeReason = 'FRONT_TTC_CRITICAL';
      } else if (Math.abs(upcomingCurvature) > 0.009) {
        isSafe = false;
        unsafeReason = 'CURVATURE_UNSTABLE';
      }

      laneStats.push({
        laneIdx,
        laneCenter,
        vehicleCount: count,
        frontDist,
        frontSpeed,
        frontTTC,
        rearDist,
        rearSpeed,
        rearTTC,
        avgSpeed,
        score,
        isSafe,
        unsafeReason
      });
    }

    const currentLaneStat = laneStats[curLane] || laneStats[0];
    let bestLane = curLane;
    let highestScore = currentLaneStat ? currentLaneStat.score : 0;
    let shouldChange = false;
    let changeReason = 'KEEP_LANE';

    for (let laneIdx = 0; laneIdx < totalLanes; laneIdx++) {
      if (laneIdx === curLane) continue;
      const cand = laneStats[laneIdx];
      if (cand.isSafe && cand.score > highestScore + this.minImprovement) {
        highestScore = cand.score;
        bestLane = laneIdx;
        shouldChange = true;
        changeReason = `LANE_${laneIdx + 1}_FASTER_AND_CLEAR (Score: ${cand.score.toFixed(2)} vs ${currentLaneStat.score.toFixed(2)})`;
      }
    }

    return {
      currentLane: curLane,
      bestLane,
      shouldChange,
      changeReason,
      laneStats
    };
  }
}

/**
 * ─── 7. DETERMINISTIC SMOOTH LANE CHANGE STATE MACHINE ───
 */
export const LaneChangeState = {
  KEEP_LANE: 'KEEP_LANE',
  PREPARE: 'PREPARE',
  LANE_CHANGE: 'LANE_CHANGE',
  COMPLETE: 'COMPLETE',
  ABORT: 'ABORT'
};

export class LaneChangeStateMachine {
  constructor(config = {}) {
    this.state = LaneChangeState.KEEP_LANE;
    this.currentLane = 0;
    this.targetLane = 0;
    this.startLat = 0.0;
    this.targetLat = 0.0;
    this.latPosition = 0.0;
    this.timer = 0.0;
    this.transitionDuration = config.duration ?? 2.2;
    this.cooldown = 0.0;
    this.utilityModel = new LaneUtilityModel();
  }

  reset() {
    this.state = LaneChangeState.KEEP_LANE;
    this.currentLane = 0;
    this.targetLane = 0;
    this.latPosition = 0.0;
    this.timer = 0.0;
    this.cooldown = 0.0;
  }

  update(dt, reqDir, egoLat, egoS, egoSpeed, cruiseSpeed, totalLanes, perceivedObjects, track, upcomingCurvature = 0, autoPass = false) {
    this.cooldown = Math.max(0, this.cooldown - dt);
    this.timer += dt;

    const currentLaneIdx = Math.round((egoLat - (track.laneLat ? track.laneLat('fwd', 0) : 0)) / 3.6);
    this.currentLane = clamp(currentLaneIdx, 0, totalLanes - 1);
    const curCenter = track.laneLat ? track.laneLat('fwd', this.currentLane) : egoLat;

    const utilResult = this.utilityModel.evaluate(
      this.currentLane, egoS, egoLat, egoSpeed, cruiseSpeed, totalLanes, perceivedObjects, track, upcomingCurvature
    );

    switch (this.state) {
      case LaneChangeState.KEEP_LANE:
        this.latPosition = curCenter;
        if (this.cooldown <= 0) {
          let desiredLane = this.currentLane;
          if (reqDir !== 0) {
            desiredLane = clamp(this.currentLane + reqDir, 0, totalLanes - 1);
          } else if (autoPass && utilResult.shouldChange) {
            desiredLane = utilResult.bestLane;
          }

          if (desiredLane !== this.currentLane) {
            const targetStat = utilResult.laneStats[desiredLane];
            if (targetStat && targetStat.isSafe) {
              this.targetLane = desiredLane;
              this.startLat = curCenter;
              this.targetLat = track.laneLat ? track.laneLat('fwd', desiredLane) : desiredLane * 3.6;
              this.state = LaneChangeState.PREPARE;
              this.timer = 0.0;
            }
          }
        }
        break;

      case LaneChangeState.PREPARE:
        this.latPosition = curCenter;
        const targetStat = utilResult.laneStats[this.targetLane];
        if (targetStat && targetStat.isSafe) {
          this.state = LaneChangeState.LANE_CHANGE;
          this.startLat = curCenter;
          this.targetLat = track.laneLat ? track.laneLat('fwd', this.targetLane) : this.targetLane * 3.6;
          this.timer = 0.0;
        } else if (this.timer > 2.5) {
          this.state = LaneChangeState.ABORT;
          this.timer = 0.0;
        }
        break;

      case LaneChangeState.LANE_CHANGE: {
        const tau = clamp(this.timer / this.transitionDuration, 0.0, 1.0);
        const smoothU = tau * tau * tau * (10 - 15 * tau + 6 * tau * tau);
        this.latPosition = this.startLat + (this.targetLat - this.startLat) * smoothU;

        if (tau < 0.4) {
          const checkStat = utilResult.laneStats[this.targetLane];
          if (checkStat && !checkStat.isSafe) {
            this.state = LaneChangeState.ABORT;
            this.startLat = egoLat;
            this.targetLat = curCenter;
            this.timer = 0.0;
            break;
          }
        }

        if (tau >= 1.0) {
          this.latPosition = this.targetLat;
          this.state = LaneChangeState.COMPLETE;
          this.timer = 0.0;
        }
        break;
      }

      case LaneChangeState.COMPLETE:
        this.currentLane = this.targetLane;
        this.cooldown = 3.5;
        this.state = LaneChangeState.KEEP_LANE;
        break;

      case LaneChangeState.ABORT: {
        const tau = clamp(this.timer / 1.5, 0.0, 1.0);
        const smoothU = tau * tau * (3 - 2 * tau);
        this.latPosition = this.startLat + (curCenter - this.startLat) * smoothU;
        if (tau >= 1.0) {
          this.latPosition = curCenter;
          this.cooldown = 4.0;
          this.state = LaneChangeState.KEEP_LANE;
        }
        break;
      }
    }

    return {
      state: this.state,
      targetLateral: this.latPosition,
      active: this.state === LaneChangeState.LANE_CHANGE || this.state === LaneChangeState.ABORT,
      targetLane: this.targetLane,
      utilResult
    };
  }
}

/**
 * ─── 8. COMPLETE AUTONOMOUS DRIVING CONTROLLER STACK ───
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

  step(context) {
    const {
      egoState,
      track,
      perceivedObjects = [],
      proximityRays = [],
      baseCruiseSpeed = 14.0,
      laneRequest = 0,
      dt = 1.0 / 30.0,
      adas = { lka: true, aeb: true, gov: true, alc: true, autoPass: false }
    } = context;

    const u = Math.max(egoState.u ?? 0, 0.0);
    const egoS = egoState.s ?? 0;
    const egoLat = egoState.lat ?? 0;
    const egoPsi = egoState.psi ?? 0;
    const roadPsi = egoState.roadPsi ?? 0;
    const kappa = egoState.kappa ?? 0;
    const omega = egoState.om ?? 0;
    const totalLanes = track.def ? track.def.lanesF : 2;

    // ── 1. Absolute Maximum Speed Ceiling Clamp (60 km/h) ──
    const effectiveCruise = clamp(baseCruiseSpeed, 0, MAX_AUTONOMOUS_SPEED);

    // ── 2. Curvature Lookahead Profile ──
    let safeSpeed = effectiveCruise;
    let lookaheadK = kappa;
    let upcomingCurvature = 0;

    if (adas.gov) {
      const govResult = this.gov.evaluate(track, egoS, u, effectiveCruise);
      safeSpeed = Math.min(safeSpeed, govResult.safeSpeed);
      lookaheadK = Math.max(kappa, govResult.lookaheadK);
      upcomingCurvature = govResult.upcomingCurvature;
    }

    // ── 3. Intelligent Lane Selection & State Machine ──
    let targetLat = egoLat;
    let lcActive = false;
    let utilInfo = null;

    if (adas.alc) {
      const lcResult = this.laneMachine.update(
        dt, laneRequest, egoLat, egoS, u, effectiveCruise, totalLanes, perceivedObjects, track, upcomingCurvature, adas.autoPass
      );
      targetLat = lcResult.targetLateral;
      lcActive = lcResult.active;
      utilInfo = lcResult.utilResult;
    } else {
      const curLane = Math.round((egoLat - (track.laneLat ? track.laneLat('fwd', 0) : 0)) / 3.6);
      targetLat = track.laneLat ? track.laneLat('fwd', clamp(curLane, 0, totalLanes - 1)) : egoLat;
    }

    // ── 4. Lead Vehicle Following, Stopping Distance & Distant Closing Prediction ──
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
      const leadSpeed = nearestLead.vAlong ?? nearestLead.speed ?? 0;
      leadRelSpeed = leadSpeed - u;
      isLeadBraking = nearestLead.braking || false;

      // A. IDM Target Speed
      const idmResult = this.idm.evaluate(u, safeSpeed, minLeadDist, leadRelSpeed, isLeadBraking, minLeadTTC);
      safeSpeed = Math.min(safeSpeed, idmResult.targetSpeed);

      // B. Kinematic Stopping Distance Ceiling
      const stopSpeedCeil = this.stopModel.calculateSafeSpeed(minLeadDist, Math.max(0, leadSpeed));
      safeSpeed = Math.min(safeSpeed, stopSpeedCeil);

      // C. Distant Closing Prediction Ceiling (prevents accelerating when rapidly catching traffic ahead)
      const closingCeil = this.stopModel.calculateClosingSpeedCeiling(minLeadDist, leadSpeed);
      safeSpeed = Math.min(safeSpeed, closingCeil);
    }

    // ── 5. Hierarchical Minimum Clamp (Max 60 km/h) ──
    safeSpeed = clamp(safeSpeed, 0, MAX_AUTONOMOUS_SPEED);

    // ── 6. Multi-Stage Progressive Braking ──
    let forceBrake = 0.0;
    let alertMsg = lcActive ? 'LANE CHANGE ACTIVE' : 'CRUISE';
    let alertSev = 0;

    if (minLeadDist < 6.5 || minLeadTTC < 1.35) {
      forceBrake = 1.0;
      safeSpeed = 0.0;
      alertMsg = 'AEB · CRITICAL DISTANCE — EMERGENCY STOP';
      alertSev = 2;
    } else if (minLeadTTC < 2.5 || minLeadDist < 12.0) {
      forceBrake = clamp((2.5 - minLeadTTC) / 1.5, 0.45, 0.85);
      alertMsg = 'FCW · IMMINENT COLLISION RISK — BRAKING';
      alertSev = 2;
    } else if (minLeadTTC < 4.5 || (isLeadBraking && minLeadDist < 45.0)) {
      forceBrake = clamp((4.5 - minLeadTTC) / 2.5, 0.15, 0.45);
      alertMsg = 'ACC · DECELERATING FOR TRAFFIC';
      alertSev = 1;
    }

    // ── 7. Longitudinal PID Controller (Smooth Dynamic Acceleration/Deceleration) ──
    const pidResult = this.pid.update(u, safeSpeed, dt, forceBrake);

    // ── 8. Post-Curve Stabilized Stanley Path Follower (Zero Post-Turn Zigzag) ──
    const eLat = egoLat - targetLat;
    const ePsi = wrapAngle(egoPsi - roadPsi);
    const previewDist = Math.max(8.0, u * 0.8);
    const sampleSteps = 5;
    let kappaSum = 0;
    for (let i = 0; i < sampleSteps; i++) {
      const sSample = egoS + (i / (sampleSteps - 1)) * previewDist;
      kappaSum += track.kappaAt ? track.kappaAt(sSample) : kappa;
    }
    const kappaPreview = kappaSum / sampleSteps;
    const steerCmd = this.lka.update(eLat, ePsi, kappaPreview, u, omega);

    // ── 9. Deterministic 360° Safety Shield (Blind Spot Protection) ──
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
      filteredTargetSpeed: pidResult.filteredTarget,
      alert: finalControl.alert !== 'NONE' ? finalControl.alert : alertMsg,
      severity: Math.max(alertSev, finalControl.severity || 0),
      diag: {
        lka: this.lka.diag,
        util: utilInfo
      }
    };
  }
}
