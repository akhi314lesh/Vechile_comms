/**
 * js/perception.js — Per-Vehicle Perception System
 *
 * ARCHITECTURE:
 *   Every vehicle independently calculates:
 *     1. Local perception (camera/radar/ultrasonic via raycasting)
 *     2. V2V perception (BSM reception, if radio equipped)
 *     3. Sensor fusion (merge local + V2V into unified world model)
 *     4. Hazard assessment (from unified world model ONLY)
 *
 *   A vehicle with no radio still gets steps 1, 3 (trivial), 4.
 *   V2V is an ADDITIONAL source, NOT a prerequisite.
 *
 *   Ground truth is NEVER used in perception or hazard assessment.
 *   It is used ONLY for: collision detection, reward calculation, debugging.
 */
import { clamp, wrapAngle, SENSOR_RANGES, MAX_BSM_AGE } from './utils.js';

/*ULTRASONIC / PROXIMITY SENSOR — 12 channels covering full car (360°) */
const ULTRA_DIRS = [
  0,                  // F  (Front)
  0.26,               // FR1 (Front-Right Narrow)
  -0.26,              // FL1 (Front-Left Narrow)
  0.52,               // FR2 (Front-Right Wide)
  -0.52,              // FL2 (Front-Left Wide)
  Math.PI / 3,        // SR1 (Side-Right Front)
  -Math.PI / 3,       // SL1 (Side-Left Front)
  Math.PI / 2,        // RGT (Right Side)
  -Math.PI / 2,       // LFT (Left Side)
  2 * Math.PI / 3,    // SR2 (Side-Right Rear)
  -2 * Math.PI / 3,   // SL2 (Side-Left Rear)
  Math.PI             // RR  (Rear)
];
const ULTRA_LABELS = ['F', 'FR1', 'FL1', 'FR2', 'FL2', 'SR1', 'SL1', 'RGT', 'LFT', 'SR2', 'SL2', 'RR'];

export { ULTRA_DIRS, ULTRA_LABELS };

/**
 * Cast a single ray and return the hit (or null).
 * Uses Three.js raycaster against provided targets.
 */
function castRay(raycaster, camera, ox, oy, oz, dx, dz, far, targets, _ro, _rd) {
  raycaster.camera = camera;
  raycaster.set(_ro.set(ox, oy, oz), _rd.set(dx, 0, dz).normalize());
  raycaster.far = far;
  const h = raycaster.intersectObjects(targets, true);
  return h.length ? h[0] : null;
}

function tagOf(obj) {
  let o = obj;
  while (o) { if (o.userData && o.userData.tag) return o.userData.tag; o = o.parent; }
  return 'unknown';
}

/* LOCAL PERCEPTION — raycasting for ego (full) or AI (simplified)*/

/**
 * Calculate full local perception for the ego vehicle.
 * Uses actual raycasting: 25-ray wide radar/camera fan + 12-channel 360° proximity rays.
 * Covers the full vehicle body and surroundings up to 30m (ultrasonic) / 160m (radar).
 */
export function calculateEgoLocalPerception(egoState, raycaster, camera, rayTargets, THREE) {
  const _ro = new THREE.Vector3();
  const _rd = new THREE.Vector3();
  const f = { x: -Math.sin(egoState.psi), z: -Math.cos(egoState.psi) };
  const r = { x: Math.cos(egoState.psi), z: -Math.sin(egoState.psi) };

  const result = {
    radarNearest: null,
    ultraDist: new Array(ULTRA_DIRS.length).fill(SENSOR_RANGES.ultrasonic),
    ultraTypes: new Array(ULTRA_DIRS.length).fill('clear'),
    rayHits: []
  };

  /* Forward radar & vision ray grid — 25 rays across full vehicle width */
  const fanOx = egoState.x + f.x * 1.9;
  const fanOz = egoState.z + f.z * 1.9;
  for (let k = -12; k <= 12; k++) {
    const a = k * 3.5 * Math.PI / 180;
    const ca = Math.cos(a), sa = Math.sin(a);
    const dx = f.x * ca - f.z * sa, dz = f.x * sa + f.z * ca;
    // Offset ray origin across car width (-0.9m to +0.9m) for complete car body coverage
    const latOffset = (k / 12) * 0.9;
    const rx = fanOx + r.x * latOffset;
    const rz = fanOz + r.z * latOffset;

    const h = castRay(raycaster, camera, rx, 0.55, rz, dx, dz, SENSOR_RANGES.radar, rayTargets, _ro, _rd);
    if (h) {
      const wx = rx + dx * h.distance, wz = rz + dz * h.distance;
      const latOff = Math.sin(a) * h.distance;
      if (Math.abs(latOff) < 3.2 && (!result.radarNearest || h.distance < result.radarNearest.dist)) {
        result.radarNearest = { dist: h.distance, latOff, tag: tagOf(h.object) };
      }
      result.rayHits.push({ x: wx, z: wz, tag: tagOf(h.object) });
    }
  }

  /* 360° Proximity / Ultrasonic Ray Grid — 12 channels up to 30m */
  for (let k = 0; k < ULTRA_DIRS.length; k++) {
    const a = ULTRA_DIRS[k];
    const ca = Math.cos(a), sa = Math.sin(a);
    const dx = f.x * ca - f.z * sa, dz = f.x * sa + f.z * ca;
    const h = castRay(raycaster, camera, egoState.x, 0.55, egoState.z, dx, dz, SENSOR_RANGES.ultrasonic, rayTargets, _ro, _rd);
    if (h) {
      result.ultraDist[k] = Math.max(0.15, h.distance);
      result.ultraTypes[k] = tagOf(h.object);
      result.rayHits.push({ x: egoState.x + dx * h.distance, z: egoState.z + dz * h.distance, tag: result.ultraTypes[k] });
    } else {
      result.ultraDist[k] = SENSOR_RANGES.ultrasonic;
      result.ultraTypes[k] = 'clear';
    }
  }

  return result;
}

/**
 * Calculate simplified local perception for AI vehicles.
 * Instead of full raycasting (expensive × N vehicles), use arc-length
 * distance checks with a simple forward ray for occlusion.
 * 
 * This preserves the correct architecture (AI uses perception, not ground truth)
 * while keeping the cost reasonable.
 */
export function calculateAILocalPerception(aiVehicle, allVehicles, obstacles, track, egoState) {
  const result = { detectedObjects: [] };
  const myX = aiVehicle.car?.position?.x ?? aiVehicle.x ?? 0;
  const myZ = aiVehicle.car?.position?.z ?? aiVehicle.z ?? 0;
  const myPsi = aiVehicle.psi;
  const myS = aiVehicle.s;
  const fwdX = -Math.sin(myPsi);
  const fwdZ = -Math.cos(myPsi);

  // Check each other vehicle
  for (const other of allVehicles) {
    if (other === aiVehicle) continue;
    if (other.id === aiVehicle.id) continue;

    const otherX = other.car?.position?.x ?? other.x ?? 0;
    const otherZ = other.car?.position?.z ?? other.z ?? 0;
    const dx = otherX - myX;
    const dz = otherZ - myZ;
    const dist = Math.hypot(dx, dz);

    // Range check: AI uses camera-like range forward, ultrasonic short-range all around
    if (dist > SENSOR_RANGES.camera) continue;

    // FOV check: is the other vehicle in front? (within ~120° FOV)
    const dot = (dx * fwdX + dz * fwdZ) / (dist || 1);
    const inFrontFOV = dot > -0.17; // ~100° half-angle
    const inShortRange = dist < SENSOR_RANGES.ultrasonic;

    if (!inFrontFOV && !inShortRange) continue;

    // Simplified occlusion: for now, no wall occlusion check for AI
    // (full raycasting would be too expensive for N vehicles)
    // The ego gets proper occlusion via raycasting
    result.detectedObjects.push({
      source: 'local',
      vehicleId: other.id,
      x: otherX,
      z: otherZ,
      distance: dist,
      speed: Math.abs(other.v ?? 0),
      heading: other.psi ?? 0,
      braking: !!(other.braking || other.forceT > 0),
      tag: 'vehicle'
    });
  }

  // Check ego vehicle
  if (egoState) {
    const dx = egoState.x - myX;
    const dz = egoState.z - myZ;
    const dist = Math.hypot(dx, dz);
    if (dist < SENSOR_RANGES.camera) {
      const dot = (dx * fwdX + dz * fwdZ) / (dist || 1);
      if (dot > -0.17 || dist < SENSOR_RANGES.ultrasonic) {
        result.detectedObjects.push({
          source: 'local',
          vehicleId: -1, // ego
          x: egoState.x,
          z: egoState.z,
          distance: dist,
          speed: Math.abs(egoState.u ?? 0),
          heading: egoState.psi ?? 0,
          braking: false,
          tag: 'vehicle'
        });
      }
    }
  }

  // Check obstacles
  for (const ob of obstacles) {
    const dx = ob.x - myX;
    const dz = ob.z - myZ;
    const dist = Math.hypot(dx, dz);
    if (dist > SENSOR_RANGES.camera) continue;
    const dot = (dx * fwdX + dz * fwdZ) / (dist || 1);
    if (dot > -0.17 || dist < SENSOR_RANGES.ultrasonic) {
      result.detectedObjects.push({
        source: 'local',
        vehicleId: -99,
        x: ob.x,
        z: ob.z,
        distance: dist,
        speed: 0,
        heading: ob.psi ?? 0,
        braking: false,
        tag: 'obstacle'
      });
    }
  }

  return result;
}

/* SENSOR FUSION — merge local + V2V into unified world model */

/**
 * Fuse local perception and V2V objects.
 * Each object in the unified model has a source:
 *   'local'     — only local sensors detect it
 *   'v2v'       — only V2V reports it
 *   'v2v+local' — both local and V2V detect it (fused)
 * 
 * When fused:
 *   - Position comes from local (fresher measurement)
 *   - V2V metadata (ID, message age, etc.) is retained
 */
export function fusePerception(localTracks, v2vObjects, track) {
  const unified = [];
  const matchedV2V = new Set();

  // Start with V2V objects
  for (let vi = 0; vi < v2vObjects.length; vi++) {
    const v = v2vObjects[vi];
    let matched = false;

    // Try to match with a local track
    for (const local of localTracks) {
      const dx = (local.x ?? local.wx ?? 0) - (v.estimatedX ?? v.bsmPosition?.x ?? 0);
      const dz = (local.z ?? local.wz ?? 0) - (v.estimatedZ ?? v.bsmPosition?.z ?? 0);
      const dist = Math.hypot(dx, dz);

      if (dist < 4.0) { // association threshold
        // FUSED: use local position (fresher), keep V2V metadata
        unified.push({
          source: 'v2v+local',
          vehicleId: v.vehicleId,
          x: local.x ?? local.wx ?? v.estimatedX,
          z: local.z ?? local.wz ?? v.estimatedZ,
          distance: local.distance ?? Math.hypot((local.x ?? 0), (local.z ?? 0)),
          speed: local.speed ?? v.speed ?? 0,
          heading: local.heading ?? v.heading ?? 0,
          braking: local.braking || v.braking,
          messageAge: v.messageAge ?? 0,
          tag: local.tag ?? 'vehicle',
          // Track-projected fields (filled by caller)
          s: local.s ?? 0,
          lat: local.lat ?? 0,
          vAlong: local.vAlong ?? 0,
          seen: true
        });
        matched = true;
        matchedV2V.add(vi);
        break;
      }
    }

    if (!matched) {
      // V2V only — not locally visible
      unified.push({
        source: 'v2v',
        vehicleId: v.vehicleId,
        x: v.estimatedX ?? v.bsmPosition?.x ?? 0,
        z: v.estimatedZ ?? v.bsmPosition?.z ?? 0,
        distance: v.distance ?? 0,
        speed: v.speed ?? 0,
        heading: v.heading ?? 0,
        braking: v.braking ?? false,
        messageAge: v.messageAge ?? 0,
        tag: 'vehicle',
        s: v.trackPosition ?? 0,
        lat: v.lateralPosition ?? 0,
        vAlong: 0,
        seen: false
      });
    }
  }

  // Add unmatched local tracks
  for (const local of localTracks) {
    let alreadyFused = false;
    for (const u of unified) {
      if (u.source === 'v2v+local') {
        const dx = (local.x ?? 0) - u.x;
        const dz = (local.z ?? 0) - u.z;
        if (Math.hypot(dx, dz) < 2.0) { alreadyFused = true; break; }
      }
    }
    if (!alreadyFused) {
      unified.push({
        source: 'local',
        vehicleId: local.vehicleId ?? -1,
        x: local.x ?? local.wx ?? 0,
        z: local.z ?? local.wz ?? 0,
        distance: local.distance ?? 0,
        speed: local.speed ?? Math.abs(local.vAlong ?? 0),
        heading: local.heading ?? 0,
        braking: local.braking ?? false,
        messageAge: 0,
        tag: local.tag ?? 'unknown',
        s: local.s ?? 0,
        lat: local.lat ?? 0,
        vAlong: local.vAlong ?? 0,
        seen: true // locally visible by definition
      });
    }
  }

  return unified;
}

/* ================================================================
   HAZARD ASSESSMENT — operates on unified world model ONLY
   Never directly reads ground truth.
   ================================================================ */

/**
 * Calculate hazards from the unified world model.
 * For each perceived object, compute:
 *   - distance, relative speed, closing speed
 *   - TTC (time-to-collision)
 *   - braking state, message age, source
 */
export function assessHazards(unifiedModel, vehicleState, track) {
  const hazards = [];
  const myS = vehicleState.s ?? 0;
  const mySpeed = Math.abs(vehicleState.u ?? vehicleState.v ?? 0);
  const myLat = vehicleState.lat ?? 0;

  for (const obj of unifiedModel) {
    // Calculate relative longitudinal position
    let relS = 0;
    if (track && track.wrapS) {
      relS = track.wrapS(obj.s - myS);
    } else {
      relS = obj.s - myS;
    }

    const relLat = obj.lat - myLat;

    // Closing speed (positive = getting closer)
    const closingSpeed = mySpeed - (obj.vAlong ?? obj.speed ?? 0);

    // TTC calculation
    let ttc = Infinity;
    if (closingSpeed > 0.25 && obj.distance > 0) {
      ttc = obj.distance / closingSpeed;
    }

    // Lateral overlap check
    const lateralSeparation = Math.abs(relLat);

    hazards.push({
      ...obj,
      relS,
      relLat,
      closingSpeed,
      ttc,
      lateralSeparation,
      // Hazard severity
      isLaneThreat: lateralSeparation < 2.4,
      isImmediate: ttc < 2.5 && lateralSeparation < 2.4,
      isWarning: ttc < 7.0 && lateralSeparation < 3.5
    });
  }

  // Sort by relevance (immediate threats first, then by distance)
  hazards.sort((a, b) => {
    if (a.isImmediate && !b.isImmediate) return -1;
    if (!a.isImmediate && b.isImmediate) return 1;
    return a.distance - b.distance;
  });

  return hazards;
}

/* PERCEPTION MANAGER — orchestrates the full pipeline per vehicle */

/**
 * Full perception pipeline for a single vehicle.
 * This is the correct architecture:
 *   localPerception = calculateLocalPerception(vehicle)
 *   if vehicle.radio:
 *     v2vPerception = receiveV2V(vehicle)
 *   else:
 *     v2vPerception = []
 *   unifiedPerception = fuse(localPerception, v2vPerception)
 *   hazards = calculateHazards(unifiedPerception)
 */
export function runPerceptionPipeline(vehicle, config) {
  const {
    isEgo,
    allVehicles,
    obstacles,
    track,
    bsmBuffer,
    currentTime,
    v2vRange,
    egoState,
    // Ego-only raycasting deps
    raycaster,
    camera,
    rayTargets,
    THREE,
    // Existing local tracks (for ego track persistence)
    existingLocalTracks
  } = config;

  let localDetections = [];
  let localResult = null;

  // Step 1: Local perception
  if (isEgo && raycaster && camera && rayTargets && THREE) {
    // Full raycasting for ego
    localResult = calculateEgoLocalPerception(
      { x: egoState.x, z: egoState.z, psi: egoState.psi },
      raycaster, camera, rayTargets, THREE
    );
    // Convert ray hits to tracked objects using the existing tracker
    if (existingLocalTracks) {
      localDetections = existingLocalTracks;
    }
  } else {
    // Simplified perception for AI
    const aiResult = calculateAILocalPerception(
      vehicle, allVehicles, obstacles, track, egoState
    );
    localDetections = aiResult.detectedObjects;
  }

  // Step 2: V2V reception (only if radio equipped)
  let v2vObjects = [];
  if (vehicle.radio && bsmBuffer) {
    const { receiveBSMs } = require_v2v();
    v2vObjects = receiveBSMs(vehicle, bsmBuffer, currentTime, v2vRange);
  }

  // Step 3: Fusion
  const unifiedModel = fusePerception(localDetections, v2vObjects, track);

  // Step 4: Hazard assessment
  const vehicleState = isEgo
    ? { s: egoState.s ?? 0, u: egoState.u ?? 0, lat: egoState.lat ?? 0 }
    : { s: vehicle.s ?? 0, v: vehicle.v ?? 0, lat: vehicle.lat ?? 0, u: vehicle.v ?? 0 };

  const hazards = assessHazards(unifiedModel, vehicleState, track);

  return {
    localResult,    // Raw sensor data (ego only)
    localDetections,
    v2vObjects,
    unifiedModel,
    hazards
  };
}

// Lazy import helper to avoid circular deps
let _v2v = null;
function require_v2v() {
  if (!_v2v) {
    // Direct import would be circular; we use the passed bsmBuffer directly
    // The receiveBSMs function is re-exported inline
    _v2v = {
      receiveBSMs(receiverVehicle, bsmBuffer, currentTime, v2vRange) {
        if (!receiverVehicle.radio) return [];
        const range = v2vRange ?? SENSOR_RANGES.v2v;
        const receiverX = receiverVehicle.x ?? receiverVehicle.car?.position?.x ?? 0;
        const receiverZ = receiverVehicle.z ?? receiverVehicle.car?.position?.z ?? 0;
        const received = [];
        for (const bsm of bsmBuffer.getAll()) {
          if (bsm.vehicleId === receiverVehicle.id) continue;
          const dx = bsm.position.x - receiverX;
          const dz = bsm.position.z - receiverZ;
          const dist = Math.hypot(dx, dz);
          if (dist > range) continue;
          const messageAge = currentTime - bsm.timestamp;
          if (messageAge > MAX_BSM_AGE) continue;
          const fwdX = -Math.sin(bsm.heading);
          const fwdZ = -Math.cos(bsm.heading);
          received.push({
            vehicleId: bsm.vehicleId,
            bsmPosition: { x: bsm.position.x, z: bsm.position.z },
            heading: bsm.heading,
            speed: bsm.speed,
            acceleration: bsm.acceleration,
            braking: bsm.braking,
            lane: bsm.lane,
            trackPosition: bsm.trackPosition,
            lateralPosition: bsm.lateralPosition,
            distance: dist,
            messageAge,
            estimatedX: bsm.position.x + bsm.speed * fwdX * messageAge,
            estimatedZ: bsm.position.z + bsm.speed * fwdZ * messageAge,
            source: 'v2v'
          });
        }
        return received;
      }
    };
  }
  return _v2v;
}
