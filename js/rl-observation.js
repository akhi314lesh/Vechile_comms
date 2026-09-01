/**
 * js/rl-observation.js — RL Observation Vector Construction
 *
 * The observation is derived ONLY from:
 *   - Ego vehicle state (IMU)
 *   - Local perception (camera, ultrasonic, radar)
 *   - V2V perception (BSM messages)
 *   - Fused perception (unified world model)
 *   - Road/map information
 *
 * It must NEVER contain:
 *   - Hidden vehicle positions
 *   - Hidden vehicle behavior type
 *   - Ground-truth collision distance
 *   - Ground-truth TTC
 *   - Scenario array
 */
import {
  clamp, normalize01, normalizeSymmetric,
  MAX_PERCEIVED_OBJECTS, OBS_EGO_FEATURES, OBS_ROAD_FEATURES,
  OBS_OBJ_FEATURES, OBS_TOTAL
} from './utils.js';

/* Normalization constants */
const MAX_SPEED = 50;         // m/s (~180 km/h)
const MAX_ACCEL = 12;         // m/s²
const MAX_YAW_RATE = 2.6;    // rad/s
const MAX_SLIP = 0.5;         // rad
const MAX_LANE_DEV = 5;       // m
const MAX_CURVATURE = 0.1;    // 1/m
const MAX_SIGNAL_DIST = 200;  // m
const MAX_DISTANCE = 250;     // m (V2V range)
const MAX_REL_SPEED = 60;     // m/s
const MAX_TTC = 15;           // s
const MAX_MSG_AGE = 2.0;      // s

/**
 * Build the normalized observation vector from perception data.
 *
 * @param {object} egoState - Ego vehicle state (from IMU/dynamics)
 * @param {object} roadInfo - Road/map information
 * @param {array}  perceivedObjects - From unified world model (fusion output)
 * @param {object} ctrl - Current control inputs
 * @returns {Float32Array} - Normalized observation vector of size OBS_TOTAL
 */
export function buildObservation(egoState, roadInfo, perceivedObjects, ctrl) {
  const obs = new Float32Array(OBS_TOTAL);
  let idx = 0;

  /* ---- Ego State (9 features) ---- */
  obs[idx++] = normalize01(Math.abs(egoState.u ?? 0), 0, MAX_SPEED);                // speed [0,1]
  obs[idx++] = normalizeSymmetric(egoState.ax ?? 0, -MAX_ACCEL, MAX_ACCEL);         // long accel [-1,1]
  obs[idx++] = normalizeSymmetric(egoState.ay ?? 0, -MAX_ACCEL, MAX_ACCEL);         // lat accel [-1,1]
  obs[idx++] = normalizeSymmetric(egoState.om ?? 0, -MAX_YAW_RATE, MAX_YAW_RATE);  // yaw rate [-1,1]
  obs[idx++] = normalizeSymmetric(egoState.beta ?? 0, -MAX_SLIP, MAX_SLIP);         // slip angle [-1,1]
  obs[idx++] = normalizeSymmetric(egoState.laneDev ?? 0, -MAX_LANE_DEV, MAX_LANE_DEV); // lane deviation [-1,1]
  obs[idx++] = clamp(ctrl?.steer ?? 0, -1, 1);                                      // steering [-1,1]
  obs[idx++] = clamp(ctrl?.throttle ?? 0, 0, 1);                                    // throttle [0,1]
  obs[idx++] = clamp(ctrl?.brake ?? 0, 0, 1);                                       // brake [0,1]

  /* ---- Road Info (4 features) ---- */
  obs[idx++] = normalizeSymmetric(roadInfo.curvatureAhead ?? 0, -MAX_CURVATURE, MAX_CURVATURE); // curvature [-1,1]
  obs[idx++] = normalize01(roadInfo.laneCount ?? 2, 1, 4);                           // lane count [0,1]
  obs[idx++] = normalize01(roadInfo.oncomingLanes ?? 0, 0, 4);                       // oncoming lanes [0,1]
  // Signal: encode as distance * state. 0 = no signal or green, positive = distance to red/yellow
  const sigDist = roadInfo.signalDist ?? MAX_SIGNAL_DIST;
  const sigState = roadInfo.signalState ?? 'green';
  obs[idx++] = sigState === 'green' ? 0 : normalize01(sigDist, 0, MAX_SIGNAL_DIST);  // signal [0,1]

  /* ---- Perceived Objects (MAX_PERCEIVED_OBJECTS × OBS_OBJ_FEATURES) ---- */
  // Sort by distance, take closest N
  const sorted = (perceivedObjects || [])
    .slice()
    .sort((a, b) => (a.distance ?? Infinity) - (b.distance ?? Infinity))
    .slice(0, MAX_PERCEIVED_OBJECTS);

  for (let i = 0; i < MAX_PERCEIVED_OBJECTS; i++) {
    if (i < sorted.length) {
      const obj = sorted[i];
      obs[idx++] = normalizeSymmetric(obj.relX ?? 0, -MAX_DISTANCE, MAX_DISTANCE);    // relative X [-1,1]
      obs[idx++] = normalizeSymmetric(obj.relZ ?? 0, -MAX_DISTANCE, MAX_DISTANCE);    // relative Z [-1,1]
      obs[idx++] = normalizeSymmetric(obj.relSpeed ?? 0, -MAX_REL_SPEED, MAX_REL_SPEED); // rel speed [-1,1]
      obs[idx++] = normalize01(clamp(obj.distance ?? MAX_DISTANCE, 0, MAX_DISTANCE), 0, MAX_DISTANCE); // dist [0,1]
      obs[idx++] = obj.braking ? 1 : 0;                                                // braking {0,1}
      // Source encoding: local=0, v2v=0.5, v2v+local=1
      obs[idx++] = obj.source === 'v2v+local' ? 1 : obj.source === 'v2v' ? 0.5 : 0;  // source [0,1]
      obs[idx++] = normalize01(clamp(obj.messageAge ?? 0, 0, MAX_MSG_AGE), 0, MAX_MSG_AGE); // msg age [0,1]
    } else {
      // Pad with zeros for missing objects
      obs[idx++] = 0; // relX
      obs[idx++] = 0; // relZ
      obs[idx++] = 0; // relSpeed
      obs[idx++] = 1; // distance = max (far away)
      obs[idx++] = 0; // braking = no
      obs[idx++] = 0; // source = none
      obs[idx++] = 0; // msg age = 0
    }
  }

  return obs;
}

/**
 * Prepare perceived objects with ego-relative coordinates for the observation.
 * Converts world-space objects to ego-relative frame.
 */
export function preparePerceivedForObs(perceivedObjects, egoState) {
  const egoX = egoState.x ?? 0;
  const egoZ = egoState.z ?? 0;
  const egoPsi = egoState.psi ?? 0;
  const egoSpeed = Math.abs(egoState.u ?? 0);
  const cosPsi = Math.cos(egoPsi);
  const sinPsi = Math.sin(egoPsi);

  return (perceivedObjects || []).map(obj => {
    const dx = (obj.x ?? 0) - egoX;
    const dz = (obj.z ?? 0) - egoZ;
    // Rotate to ego frame (forward = -Z in world)
    const relX = dx * cosPsi + dz * sinPsi;
    const relZ = -dx * sinPsi + dz * cosPsi;
    const distance = Math.hypot(dx, dz);
    const relSpeed = egoSpeed - (obj.speed ?? 0);

    return {
      ...obj,
      relX,
      relZ,
      relSpeed,
      distance
    };
  });
}

/**
 * Get the observation space dimensions for RL configuration.
 */
export function getObservationSpaceInfo() {
  return {
    shape: [OBS_TOTAL],
    egoFeatures: OBS_EGO_FEATURES,
    roadFeatures: OBS_ROAD_FEATURES,
    objectFeatures: OBS_OBJ_FEATURES,
    maxObjects: MAX_PERCEIVED_OBJECTS,
    total: OBS_TOTAL,
    // Feature names for debugging/logging
    featureNames: [
      // Ego
      'ego_speed', 'ego_ax', 'ego_ay', 'ego_yaw_rate', 'ego_slip',
      'ego_lane_dev', 'ego_steer', 'ego_throttle', 'ego_brake',
      // Road
      'road_curvature', 'road_lanes', 'road_oncoming', 'road_signal',
      // Objects (repeated MAX_PERCEIVED_OBJECTS times)
      ...Array.from({ length: MAX_PERCEIVED_OBJECTS }, (_, i) => [
        `obj${i}_relX`, `obj${i}_relZ`, `obj${i}_relSpeed`,
        `obj${i}_dist`, `obj${i}_braking`, `obj${i}_source`, `obj${i}_msgAge`
      ]).flat()
    ]
  };
}
