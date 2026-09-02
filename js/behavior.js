/**
 * js/behavior.js — Vehicle behavior types: normal, erratic
 * 
 * Behavior is INDEPENDENT of communication capability (radio flag).
 * The underlying data model keeps them separate:
 *   { radio: true/false, behavior: 'normal'/'erratic' }
 * 
 * Four valid combinations:
 *   1. V2V + NORMAL    — predictable, broadcasts BSMs
 *   2. V2V + ERRATIC   — unpredictable, still broadcasts BSMs
 *   3. NO V2V + NORMAL  — predictable, invisible to radio
 *   4. NO V2V + ERRATIC — unpredictable, invisible to radio
 * 
 * Erratic behavior still obeys physical vehicle model.
 * No teleporting or physics violations.
 */
import { clamp, createRNG } from './utils.js';
import { safeCurveSpeed } from './physics.js';

/* NORMAL BEHAVIOR*/

/**
 * Normal vehicle: constant desired speed, normal road-following,
 * normal braking behavior, safe curve speed.
 */
export function computeNormalBehavior(vehicle, dt, context) {
  const { track, allVehicles, egoState, signalPhase } = context;
  
  let want = vehicle.speed;
  const look = vehicle.v * 2.4 + 30;

  // Safe curve speed
  const k = track.kappaAt(vehicle.dir === 'fwd' ? vehicle.s + look : vehicle.s - look);
  want = Math.min(want, Math.max(safeCurveSpeed(k, 0.85, 0.55), 3));

  // Follow leader (uses perception if available, otherwise arc-length check)
  if (vehicle.dir === 'fwd') {
    // Check hazards from perception (if available)
    if (vehicle.hazards && vehicle.hazards.length > 0) {
      for (const h of vehicle.hazards) {
        if (!h.isLaneThreat) continue;
        if (h.relS > 0.5 && h.relS < 80) {
          want = Math.min(want, Math.max(0, h.speed + 0.5 * (h.relS - (7 + vehicle.v * 1.2))));
        }
      }
    } else {
      // Fallback: use ground-truth distance (for backward compat during migration)
      for (const o of allVehicles) {
        if (o === vehicle || o.dir !== 'fwd' || o.lane !== vehicle.lane) continue;
        const d = track.wrapS(o.s - vehicle.s);
        if (d > 0.5 && d < 80) {
          const leadV = Math.abs(o.v);
          want = Math.min(want, Math.max(0, leadV + 0.5 * (d - (7 + vehicle.v * 1.2))));
        }
      }
    }

    // Ego following (normal vehicles only; erratic vehicles do not yield)
    if (egoState && vehicle.behavior !== 'erratic') {
      const gp = track.wrapS(egoState.s - vehicle.s);
      if (gp > 0.5 && gp < 25) {
        const egoLat = egoState.lat ?? 0;
        const myLat = track.laneLat ? track.laneLat(vehicle.dir, vehicle.lane) : 0;
        if (Math.abs(myLat - egoLat) < 2.4) {
          want = Math.min(want, Math.max(0, Math.abs(egoState.u ?? 0) + 0.5 * (gp - (7 + vehicle.v * 1.2))));
        }
      }
    }

    // Obstacle avoidance
    if (context.obstacles) {
      for (const ob of context.obstacles) {
        const myLat = track.laneLat ? track.laneLat('fwd', vehicle.lane) : 0;
        if (Math.abs(ob.lat - myLat) > 2.2) continue;
        const d = track.wrapS(ob.s - vehicle.s);
        if (d > 0.5 && d < 70) want = Math.min(want, Math.max(0, 0.5 * (d - 7)));
      }
    }
  }

  // Signal stop
  if (signalPhase && context.crossFrames) {
    for (const cf of context.crossFrames) {
      const d = vehicle.dir === 'fwd'
        ? track.wrapS(cf.s - 14 - vehicle.s)
        : track.wrapS(vehicle.s - cf.s - 14);
      if (d <= 0.5 || d > 90) continue;
      if (signalPhase.main !== 'green') {
        const canStop = d > (vehicle.v * vehicle.v) / 7 + 1;
        if (signalPhase.main === 'red' || canStop) {
          want = Math.min(want, Math.sqrt(Math.max(0, 2 * 2.5 * (d - 2.5))));
        }
      }
    }
  }

  return { desiredSpeed: want, braking: want < vehicle.v - 2.5 };
}

/* ERRATIC BEHAVIOR*/

/**
 * Erratic vehicle state — tracks current perturbation.
 * Created once per erratic vehicle.
 */
export function createErraticState(seed) {
  const rng = createRNG(seed);
  return {
    rng,
    // Current perturbation
    accelPulse: 0,          // m/s² perturbation
    accelPulseTimer: 0,     // remaining time
    accelPulseCooldown: rng.randFloat(8, 20),
    // Unexpected braking
    brakePulse: false,
    brakePulseTimer: 0,
    brakePulseCooldown: rng.randFloat(15, 45),
    // Speed oscillation
    speedOscPhase: rng.random() * Math.PI * 2,
    speedOscPeriod: rng.randFloat(5, 10),
    speedOscAmplitude: rng.randFloat(0.1, 0.2), // fraction of base speed
    // Accumulated time
    t: 0
  };
}

/**
 * Erratic vehicle: unpredictable but physically valid.
 * Builds on normal behavior, then adds perturbations.
 * 
 * Possible perturbations:
 *   - Random acceleration/deceleration bursts
 *   - Unexpected braking events
 *   - Speed oscillation
 *   
 * All perturbations pass through vehicle dynamics (no teleporting).
 */
export function computeErraticBehavior(vehicle, dt, context) {
  // Start with normal behavior as baseline
  const normal = computeNormalBehavior(vehicle, dt, context);
  let want = normal.desiredSpeed;
  let braking = normal.braking;

  // Initialize erratic state if needed
  if (!vehicle.erraticState) {
    vehicle.erraticState = createErraticState(vehicle.id * 7919 + 42);
  }
  const es = vehicle.erraticState;
  es.t += dt;

  // --- Speed oscillation (continuous) ---
  const oscFrac = Math.sin(es.t * (2 * Math.PI / es.speedOscPeriod) + es.speedOscPhase);
  want *= (1 + oscFrac * es.speedOscAmplitude);

  // --- Random acceleration pulse ---
  es.accelPulseCooldown -= dt;
  if (es.accelPulseCooldown <= 0 && es.accelPulseTimer <= 0) {
    // Trigger new pulse
    es.accelPulse = es.rng.randFloat(-4, 4); // ±4 m/s²
    es.accelPulseTimer = es.rng.randFloat(0.5, 2.0);
    es.accelPulseCooldown = es.rng.randFloat(8, 20);
  }
  if (es.accelPulseTimer > 0) {
    es.accelPulseTimer -= dt;
    want += es.accelPulse * dt * 8; // Convert acceleration to speed change
    want = Math.max(0, want);
  }

  // --- Unexpected braking ---
  es.brakePulseCooldown -= dt;
  if (es.brakePulseCooldown <= 0 && !es.brakePulse) {
    es.brakePulse = true;
    es.brakePulseTimer = es.rng.randFloat(1.0, 3.0);
    es.brakePulseCooldown = es.rng.randFloat(15, 45);
  }
  if (es.brakePulse) {
    es.brakePulseTimer -= dt;
    want = Math.max(vehicle.speed * 0.15, 1.5);
    braking = true;
    if (es.brakePulseTimer <= 0) {
      es.brakePulse = false;
    }
  }

  // Clamp to reasonable bounds
  want = clamp(want, 0, vehicle.speed * 1.4);

  return { desiredSpeed: want, braking };
}

/* BEHAVIOR DISPATCHER*/

/**
 * Compute behavior for a vehicle based on its behavior type.
 * Returns { desiredSpeed, braking }.
 * 
 * Behavior types:
 *   'normal'  — predictable driving
 *   'erratic' — random perturbations
 *   'stopped' — stationary (hazard lights)
 *   'brake'   — legacy brake-event mode (maps to periodic braking)
 */
export function computeVehicleBehavior(vehicle, dt, context) {
  if (vehicle.behavior === 'stopped') {
    return { desiredSpeed: 0, braking: true };
  }

  if (vehicle.behavior === 'erratic') {
    return computeErraticBehavior(vehicle, dt, context);
  }

  if (vehicle.behavior === 'brake') {
    // Legacy brake-event behavior: periodic braking
    const normal = computeNormalBehavior(vehicle, dt, context);
    // The timer/braking toggling is handled at the vehicle level
    if (vehicle.braking || vehicle.forceT > 0) {
      return {
        desiredSpeed: Math.max(vehicle.speed * 0.22, 2.5),
        braking: true
      };
    }
    return normal;
  }

  // Default: normal
  return computeNormalBehavior(vehicle, dt, context);
}
