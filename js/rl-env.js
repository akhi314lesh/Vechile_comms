/**
 * js/rl-env.js — Reinforcement Learning Environment
 *
 * Provides the standard RL interface:
 *   env.reset(seed)        → initial observation
 *   env.step(action)       → { observation, reward, done, truncated, info }
 *   env.getObservation()   → agent-visible observation only
 *   env.getGroundTruth()   → privileged data (for reward/eval only)
 *   env.getMetrics()       → episode statistics
 *
 * The simulation update order follows Section 30:
 *   1. Receive control action
 *   2. Apply control smoother
 *   3. Update ego vehicle physics
 *   4. Update other vehicle behavior
 *   5. Update vehicle positions
 *   6. Update local sensors for EVERY vehicle
 *   7. Generate/update V2V BSMs
 *   8. Receive V2V messages for EVERY equipped vehicle
 *   9. Fuse local + V2V perception for EVERY vehicle
 *   10. Calculate hazards for EVERY vehicle
 *   11. Run autonomous policy / RL policy
 *   12. Apply resulting controls on next control step
 *   13. Check collisions
 *   14. Calculate reward
 *   15. Generate observation
 *   16. Return step result
 */
import { clamp, createRNG, PHYS_H, SENSOR_RANGES, OBS_TOTAL } from './utils.js';
import { VehicleDynamics, obbOverlap2D, safeCurveSpeed } from './physics.js';
import { BSMBroadcastBuffer } from './v2v.js';
import {
  calculateEgoLocalPerception,
  calculateAILocalPerception,
  fusePerception,
  assessHazards
} from './perception.js';
import { computeVehicleBehavior } from './behavior.js';
import { buildObservation, preparePerceivedForObs } from './rl-observation.js';
import { calculateReward, checkTermination } from './rl-reward.js';
import { EpisodeMetrics } from './metrics.js';
import { generateScenario } from './scenario-gen.js';

/**
 * V2V RL Environment
 *
 * Can run in two modes:
 *   - VISUAL mode: integrated with Three.js rendering (browser)
 *   - HEADLESS mode: physics + perception only, no rendering
 */
export class V2VRLEnvironment {
  constructor(config = {}) {
    this.headless = config.headless ?? false;
    this.maxSteps = config.maxSteps ?? 2000;
    this.v2vEnabled = config.v2vEnabled ?? true;
    this.curriculumStage = config.curriculumStage ?? 1;
    this.dt = config.dt ?? (1 / 30); // simulation timestep

    // Will be set by connectToSim() or created internally
    this.sim = null;        // reference to running simulation
    this.track = null;
    this.dyn = null;
    this.scenario = [];
    this.bsmBuffer = new BSMBroadcastBuffer();

    // Episode state
    this.episodeNum = 0;
    this.stepNum = 0;
    this.metrics = new EpisodeMetrics();
    this.prevAction = null;
    this.prevState = null;
    this.currentObs = null;
    this.done = false;
    this.totalProgress = 0;
    this.offRoadAccum = 0;
    this.stuckAccum = 0;

    // Perception state
    this.egoLocalTracks = [];
    this.egoWorldModel = [];
    this.egoHazards = [];
  }

  /**
   * Connect to a running simulation (visual mode).
   * The sim object should provide access to:
   *   - track, scenario, dyn, etc.
   */
  connectToSim(simRef) {
    this.sim = simRef;
  }

  /**
   * Reset the environment with a seed for deterministic episodes.
   * Returns the initial observation.
   */
  reset(seed) {
    this.episodeNum++;
    this.stepNum = 0;
    this.done = false;
    this.prevAction = null;
    this.prevState = null;
    this.totalProgress = 0;
    this.offRoadAccum = 0;
    this.stuckAccum = 0;
    this.egoLocalTracks = [];
    this.egoWorldModel = [];
    this.egoHazards = [];

    // Reset metrics
    this.metrics.reset();
    this.metrics.episode = this.episodeNum;
    this.metrics.startTime = performance.now() / 1000;

    // Generate scenario
    const scenarioCfg = generateScenario(
      seed ?? (this.episodeNum * 7919 + 42),
      this.curriculumStage
    );

    this.metrics.v2vEnabled = scenarioCfg.tags.v2vEnabled;
    this.metrics.hasErratic = scenarioCfg.tags.hasErratic;
    this.metrics.hasOccluded = scenarioCfg.tags.hasOccluded;
    this.metrics.curriculumStage = this.curriculumStage;

    // If connected to a visual sim, use its reset mechanism
    if (this.sim) {
      this.sim.resetWithScenario(scenarioCfg);
      this.track = this.sim.track;
      this.dyn = this.sim.dyn;
      this.scenario = this.sim.scenario;
    }

    // Clear BSM buffer
    this.bsmBuffer.clear();

    // Build initial observation
    this.currentObs = this._buildCurrentObs({ steer: 0, throttle: 0, brake: 0 });
    return this.currentObs;
  }

  /**
   * Execute one step in the environment.
   * Action: { steering: [-1,1], throttle: [0,1], brake: [0,1] }
   * 
   * Returns: { observation, reward, done, truncated, info }
   */
  step(action) {
    if (this.done) {
      throw new Error('Environment is done. Call reset() first.');
    }

    this.stepNum++;
    const dt = this.dt;

    // Normalize action
    const ctrl = {
      steer: clamp(action.steering ?? action.steer ?? 0, -1, 1),
      throttle: clamp(action.throttle ?? 0, 0, 1),
      brake: clamp(action.brake ?? 0, 0, 1)
    };

    // Save previous state for reward calculation
    const prevEgoState = this._getEgoState();

    // ---- STEP 1-2: Apply control (with smoothing) ----
    // Control smoothing is handled by VehicleDynamics.step()

    // ---- STEP 3: Update ego physics ----
    if (this.dyn && this.track) {
      const proj = this.track.project(this.dyn.x, this.dyn.z, 0);
      const onP = Math.abs(proj.lat) < this.track.metrics.pavedHalf + 0.35;
      const env = { mu: onP ? 0.92 : 0.5, offroad: !onP, abs: true, esc: true };

      let acc = dt;
      while (acc >= PHYS_H) {
        this.dyn.step(PHYS_H, ctrl, env);
        acc -= PHYS_H;
      }

      // Track off-road time
      if (!onP) {
        this.offRoadAccum += dt;
      } else {
        this.offRoadAccum = 0;
      }
    }

    // ---- STEP 4-5: Update AI vehicles ----
    if (this.sim) {
      // Delegate to sim's AI update
      this.sim.updateAIVehicles(dt);
    } else if (this.track) {
      this._updateAIVehicles(dt);
    }

    // ---- STEP 6: Update local sensors for EVERY vehicle ----
    // (Handled in perception pipeline below)

    // ---- STEP 7: Generate/update V2V BSMs ----
    const currentTime = this.stepNum * dt;
    this.bsmBuffer.clear();
    for (const ai of this.scenario) {
      if (ai.radio) {
        this.bsmBuffer.transmit({
          id: ai.id,
          x: ai.car?.position?.x ?? ai.x ?? 0,
          z: ai.car?.position?.z ?? ai.z ?? 0,
          psi: ai.psi ?? 0,
          v: ai.v ?? 0,
          ax: 0,
          braking: !!(ai.braking || ai.forceT > 0 || ai.behavior === 'stopped'),
          lane: ai.lane ?? 0,
          s: ai.s ?? 0,
          lat: ai.lat ?? 0,
          radio: true
        }, currentTime);
      }
    }

    // ---- STEP 8-10: Perception pipeline for every vehicle ----
    const egoState = this._getEgoState();

    // Ego perception
    if (this.sim) {
      // Use sim's raycasting infrastructure
      const egoPerception = this.sim.runEgoPerception(currentTime, this.bsmBuffer);
      this.egoWorldModel = egoPerception.unifiedModel;
      this.egoHazards = egoPerception.hazards;
    } else {
      // Simplified (headless)
      this._runEgoPerceptionSimplified(egoState, currentTime);
    }

    // AI perception (simplified for performance)
    for (const ai of this.scenario) {
      const aiLocal = calculateAILocalPerception(
        ai, this.scenario, this.track?.obstacleData ?? [], this.track, egoState
      );

      let v2vObjects = [];
      if (ai.radio) {
        // Receive BSMs
        const receiverState = {
          id: ai.id,
          x: ai.car?.position?.x ?? ai.x ?? 0,
          z: ai.car?.position?.z ?? ai.z ?? 0,
          radio: true
        };
        // Inline BSM reception for AI
        for (const bsm of this.bsmBuffer.getAll()) {
          if (bsm.vehicleId === ai.id) continue;
          const dx = bsm.position.x - receiverState.x;
          const dz = bsm.position.z - receiverState.z;
          const dist = Math.hypot(dx, dz);
          if (dist > SENSOR_RANGES.v2v) continue;
          v2vObjects.push({
            vehicleId: bsm.vehicleId,
            estimatedX: bsm.position.x,
            estimatedZ: bsm.position.z,
            bsmPosition: bsm.position,
            speed: bsm.speed,
            heading: bsm.heading,
            braking: bsm.braking,
            distance: dist,
            messageAge: 0,
            source: 'v2v'
          });
        }
      }

      ai.worldModel = fusePerception(aiLocal.detectedObjects, v2vObjects, this.track);
      ai.hazards = assessHazards(ai.worldModel,
        { s: ai.s ?? 0, u: ai.v ?? 0, lat: ai.lat ?? 0 }, this.track);
    }

    // ---- STEP 11: RL policy already applied (ctrl) ----
    // ---- STEP 12: Controls applied above ----

    // ---- STEP 13: Check collisions ----
    const collisionResult = this._checkCollisions();

    // ---- STEP 14: Calculate reward ----
    const currentEgoState = this._getEgoState();
    const groundTruth = this._getGroundTruth(collisionResult);

    const reward = calculateReward(
      currentEgoState, prevEgoState ?? currentEgoState,
      ctrl, this.prevAction,
      groundTruth
    );

    // ---- STEP 15: Build observation ----
    this.currentObs = this._buildCurrentObs(ctrl);

    // ---- STEP 16: Check termination ----
    const termination = checkTermination(
      { ...currentEgoState, offRoadTime: this.offRoadAccum, stuckTime: this.stuckAccum, lapsCompleted: 0 },
      groundTruth, this.stepNum, this.maxSteps
    );
    this.done = termination.done || termination.truncated;

    // Record metrics
    const detectionCounts = { v2v: 0, local: 0, fused: 0 };
    for (const obj of this.egoWorldModel) {
      if (obj.source === 'v2v') detectionCounts.v2v++;
      else if (obj.source === 'local') detectionCounts.local++;
      else if (obj.source === 'v2v+local') detectionCounts.fused++;
    }

    this.metrics.recordStep({
      reward,
      speed: currentEgoState.u,
      laneDev: currentEgoState.laneDev,
      progress: this.totalProgress,
      minTTC: groundTruth.minTTC,
      minDistance: groundTruth.minDistance,
      v2vReceived: this.egoWorldModel.filter(o => o.source === 'v2v' || o.source === 'v2v+local').length,
      detectionCounts,
      offRoad: !currentEgoState.onPavement,
      braking: ctrl.brake > 0.3,
      dt,
      time: currentTime
    });

    if (typeof document !== 'undefined') {
      const elR = document.getElementById('rlMetricReward');
      if (elR) elR.textContent = `EP REWARD · ${this.metrics.totalReward.toFixed(1)}`;
      const elS = document.getElementById('rlMetricSteps');
      if (elS) elS.textContent = `STEPS · ${this.stepNum} / ${this.maxSteps}`;
      const elT = document.getElementById('rlMetricTTC');
      if (elT) elT.textContent = `MIN TTC · ${groundTruth.minTTC < 20 ? groundTruth.minTTC.toFixed(1) + 's' : '—'}`;
      const elC = document.getElementById('rlMetricCol');
      if (elC) {
        elC.textContent = `COLLISION · ${collisionResult.collided ? 'YES (' + collisionResult.type + ')' : 'NO'}`;
        elC.style.color = collisionResult.collided ? '#ff5340' : '#69f0ae';
      }
    }

    if (collisionResult.collided) {
      this.metrics.recordCollision(collisionResult.type);
    }

    if (this.done) {
      this.metrics.finalize(currentTime);
    }

    // Save for next step
    this.prevAction = ctrl;
    this.prevState = currentEgoState;

    return {
      observation: this.currentObs,
      reward,
      done: termination.done,
      truncated: termination.truncated,
      info: {
        reason: termination.reason,
        step: this.stepNum,
        speed: Math.abs(currentEgoState.u ?? 0),
        collision: collisionResult.collided,
        minTTC: groundTruth.minTTC,
        v2vDetections: detectionCounts.v2v + detectionCounts.fused,
        localDetections: detectionCounts.local + detectionCounts.fused
      }
    };
  }

  /** Get observation (agent-visible ONLY) */
  getObservation() {
    return this.currentObs;
  }

  /** Get ground truth (privileged, NEVER passed to agent) */
  getGroundTruth() {
    return this._getGroundTruth(this._checkCollisions());
  }

  /** Get episode metrics */
  getMetrics() {
    return this.metrics;
  }

  /*INTERNAL METHODS */

  _getEgoState() {
    if (!this.dyn) return {};
    const proj = this.track ? this.track.project(this.dyn.x, this.dyn.z, 0) : {};
    const onP = proj.lat !== undefined ? Math.abs(proj.lat) < (this.track?.metrics?.pavedHalf ?? 10) + 0.35 : true;
    return {
      x: this.dyn.x,
      z: this.dyn.z,
      psi: this.dyn.psi,
      u: this.dyn.u,
      w: this.dyn.w,
      om: this.dyn.om,
      ax: this.dyn.ax,
      ay: this.dyn.ay,
      beta: this.dyn.beta,
      s: proj.s ?? 0,
      lat: proj.lat ?? 0,
      laneDev: proj.lat ?? 0,
      onPavement: onP,
      offRoad: !onP,
      targetSpeed: 14
    };
  }

  _buildCurrentObs(ctrl) {
    const egoState = this._getEgoState();

    // Road info from map
    const roadInfo = {
      curvatureAhead: this.track ? this.track.kappaAt((egoState.s ?? 0) + 30) : 0,
      laneCount: this.track?.def?.lanesF ?? 2,
      oncomingLanes: this.track?.def?.lanesO ?? 0,
      signalDist: 200,
      signalState: 'green'
    };

    // Prepare perceived objects in ego-relative frame
    const perceivedRel = preparePerceivedForObs(this.egoWorldModel, egoState);

    return buildObservation(egoState, roadInfo, perceivedRel, ctrl);
  }

  _getGroundTruth(collisionResult) {
    const egoState = this._getEgoState();
    let minDist = Infinity;
    let minTTC = Infinity;

    // Calculate ground-truth distances to all vehicles
    for (const ai of this.scenario) {
      const ax = ai.car?.position?.x ?? ai.x ?? 0;
      const az = ai.car?.position?.z ?? ai.z ?? 0;
      const dist = Math.hypot(ax - (egoState.x ?? 0), az - (egoState.z ?? 0)) - 4.7; // subtract car lengths
      if (dist < minDist) minDist = dist;

      const closing = Math.abs(egoState.u ?? 0) - (ai.v ?? 0);
      if (closing > 0.25 && dist > 0) {
        const ttc = dist / closing;
        if (ttc < minTTC) minTTC = ttc;
      }
    }

    return {
      collision: collisionResult?.collided ?? false,
      collisionType: collisionResult?.type ?? '',
      minDistance: Math.max(0, minDist),
      minTTC: minTTC === Infinity ? 99 : minTTC
    };
  }

  _checkCollisions() {
    if (!this.dyn) return { collided: false, type: '' };

    const A = { x: this.dyn.x, z: this.dyn.z, psi: this.dyn.psi, hl: 2.35, hw: 0.92 };

    for (const ai of this.scenario) {
      const ax = ai.car?.position?.x ?? ai.x ?? 0;
      const az = ai.car?.position?.z ?? ai.z ?? 0;
      if (obbOverlap2D(A, { x: ax, z: az, psi: ai.psi ?? 0, hl: 2.35, hw: 0.92 })) {
        return { collided: true, type: `vehicle:${ai.name}`, headOn: ai.dir === 'onc' };
      }
    }

    if (this.track) {
      for (const ob of this.track.obstacleData ?? []) {
        if (obbOverlap2D(A, { x: ob.x, z: ob.z, psi: ob.psi, hl: ob.hl, hw: ob.hw })) {
          return { collided: true, type: `obstacle:${ob.type}`, headOn: false };
        }
      }
    }

    return { collided: false, type: '' };
  }

  _updateAIVehicles(dt) {
    if (!this.track) return;
    const egoState = this._getEgoState();

    for (const ai of this.scenario) {
      // Use behavior system
      const context = {
        track: this.track,
        allVehicles: this.scenario,
        egoState,
        obstacles: this.track.obstacleData ?? [],
        crossFrames: this.track.crossFrames ?? [],
        signalPhase: null // simplified
      };

      const result = computeVehicleBehavior(ai, dt, context);
      const want = result.desiredSpeed;
      const rate = want < ai.v ? 6.5 : 2.2;
      ai.v = Math.max(0, ai.v + clamp(want - ai.v, -rate * dt, rate * dt));
      ai.s = ai.dir === 'fwd' ? (ai.s + ai.v * dt) % this.track.L : (ai.s - ai.v * dt + this.track.L) % this.track.L;
      ai.braking = result.braking;

      // Update position from track
      if (this.track.frameAt) {
        const f = this.track.frameAt(ai.s);
        const lat = this.track.laneLat ? this.track.laneLat(ai.dir, ai.lane) : 0;
        if (ai.car) {
          ai.car.position.set(f.p.x + f.r.x * lat, 0, f.p.z + f.r.z * lat);
        }
        ai.x = f.p.x + f.r.x * lat;
        ai.z = f.p.z + f.r.z * lat;
        const psiT = Math.atan2(-f.t.x, -f.t.z);
        ai.psi = ai.dir === 'onc' ? psiT + Math.PI : psiT;
        if (ai.car) ai.car.rotation.y = ai.psi;
      }
    }
  }

  _runEgoPerceptionSimplified(egoState, currentTime) {
    // Simplified perception without raycasting (headless mode)
    const localDetections = [];
    const fwdX = -Math.sin(egoState.psi ?? 0);
    const fwdZ = -Math.cos(egoState.psi ?? 0);

    for (const ai of this.scenario) {
      const ax = ai.car?.position?.x ?? ai.x ?? 0;
      const az = ai.car?.position?.z ?? ai.z ?? 0;
      const dx = ax - (egoState.x ?? 0);
      const dz = az - (egoState.z ?? 0);
      const dist = Math.hypot(dx, dz);

      if (dist > SENSOR_RANGES.camera) continue;

      const dot = (dx * fwdX + dz * fwdZ) / (dist || 1);
      if (dot > -0.17 || dist < SENSOR_RANGES.ultrasonic) {
        // TODO: occlusion check for headless mode
        localDetections.push({
          source: 'local',
          vehicleId: ai.id,
          x: ax, z: az,
          distance: dist,
          speed: Math.abs(ai.v ?? 0),
          heading: ai.psi ?? 0,
          braking: !!(ai.braking || ai.forceT > 0),
          tag: 'vehicle',
          s: ai.s ?? 0,
          lat: ai.lat ?? 0,
          vAlong: ai.v ?? 0
        });
      }
    }

    // V2V reception
    let v2vObjects = [];
    if (this.v2vEnabled) {
      const egoReceiver = {
        id: -1,
        x: egoState.x ?? 0,
        z: egoState.z ?? 0,
        radio: true
      };
      for (const bsm of this.bsmBuffer.getAll()) {
        const dx = bsm.position.x - egoReceiver.x;
        const dz = bsm.position.z - egoReceiver.z;
        const dist = Math.hypot(dx, dz);
        if (dist > SENSOR_RANGES.v2v) continue;
        v2vObjects.push({
          vehicleId: bsm.vehicleId,
          estimatedX: bsm.position.x,
          estimatedZ: bsm.position.z,
          bsmPosition: bsm.position,
          speed: bsm.speed,
          heading: bsm.heading,
          braking: bsm.braking,
          distance: dist,
          messageAge: currentTime - bsm.timestamp,
          trackPosition: bsm.trackPosition,
          lateralPosition: bsm.lateralPosition,
          source: 'v2v'
        });
      }
    }

    // Fuse
    this.egoWorldModel = fusePerception(localDetections, v2vObjects, this.track);

    // Assess hazards
    this.egoHazards = assessHazards(this.egoWorldModel, egoState, this.track);
  }
}

/**
 * Get the action space specification for RL configuration.
 */
export function getActionSpaceInfo() {
  return {
    shape: [3],
    names: ['steering', 'throttle', 'brake'],
    low: [-1, 0, 0],
    high: [1, 1, 1],
    type: 'continuous'
  };
}
