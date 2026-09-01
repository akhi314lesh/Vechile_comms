/**
 * js/test/test-all.js — Comprehensive unit test suite for V2V simulator logic & RL
 */
import { createRNG, clamp, wrapAngle, OBS_TOTAL } from '../utils.js';
import { generateBSM, BSMBroadcastBuffer, receiveBSMs } from '../v2v.js';
import { fusePerception, assessHazards, calculateAILocalPerception } from '../perception.js';
import { computeVehicleBehavior, computeNormalBehavior, computeErraticBehavior } from '../behavior.js';
import { buildObservation, preparePerceivedForObs } from '../rl-observation.js';
import { calculateReward, checkTermination } from '../rl-reward.js';
import { EpisodeMetrics, TrainingMetrics } from '../metrics.js';
import { generateScenario, generateComparisonPair } from '../scenario-gen.js';
import { VehicleDynamics } from '../physics.js';

let passed = 0;
let failed = 0;

function assert(condition, message) {
  if (!condition) {
    console.error(`❌ FAIL: ${message}`);
    failed++;
  } else {
    console.log(`✅ PASS: ${message}`);
    passed++;
  }
}

console.log('--- STARTING V2V SIMULATOR & RL UNIT TESTS ---\n');

// Test 1: Seeded PRNG Determinism
{
  const rng1 = createRNG(12345);
  const rng2 = createRNG(12345);
  const vals1 = Array.from({ length: 10 }, () => rng1.random());
  const vals2 = Array.from({ length: 10 }, () => rng2.random());
  assert(JSON.stringify(vals1) === JSON.stringify(vals2), 'Seeded PRNG produces identical sequences for same seed');
}

// Test 2: BSM Transmission & Integrity
{
  const veh = { id: 5, x: 10, z: 20, psi: 0.5, v: 12.5, ax: 0.2, braking: true, lane: 1, s: 100, lat: 1.8, radio: true };
  const bsm = generateBSM(veh, 1.5);
  assert(bsm.vehicleId === 5, 'BSM contains vehicleId');
  assert(bsm.speed === 12.5, 'BSM contains vehicle speed');
  assert(bsm.braking === true, 'BSM contains braking state');
  assert(!('perception' in bsm) && !('seenVehicles' in bsm), 'BSM does NOT contain perception or ground-truth leak');
}

// Test 3: V2V Reception Range & Staleness
{
  const buffer = new BSMBroadcastBuffer();
  const v1 = { id: 1, x: 0, z: 0, psi: 0, v: 10, braking: false, lane: 0, s: 0, lat: 0, radio: true };
  const v2Near = { id: 2, x: 50, z: 0, psi: 0, v: 10, braking: false, lane: 0, s: 50, lat: 0, radio: true };
  const v3Far = { id: 3, x: 300, z: 0, psi: 0, v: 10, braking: false, lane: 0, s: 300, lat: 0, radio: true };
  
  buffer.transmit(v1, 1.0);
  buffer.transmit(v2Near, 1.0);
  buffer.transmit(v3Far, 1.0);

  const receivedFresh = receiveBSMs(v1, buffer, 1.0, 250);
  assert(receivedFresh.length === 1 && receivedFresh[0].vehicleId === 2, 'V2V receives nearby vehicle and excludes out-of-range vehicle');

  const receivedStale = receiveBSMs(v1, buffer, 4.0, 250); // 3 seconds later (MAX_BSM_AGE = 2.0)
  assert(receivedStale.length === 0, 'V2V discards stale messages older than MAX_BSM_AGE');
}

// Test 4: Perception Fusion (Local + V2V)
{
  const localTracks = [{ x: 10, z: 20, distance: 22.3, speed: 12, s: 20, lat: 0, tag: 'vehicle', seen: true }];
  const v2vObjects = [{ vehicleId: 4, bsmPosition: { x: 10.2, z: 20.1 }, estimatedX: 10.2, estimatedZ: 20.1, speed: 12, heading: 0, distance: 22.4, messageAge: 0.1, source: 'v2v' }];

  const fused = fusePerception(localTracks, v2vObjects, null);
  assert(fused.length === 1, 'Local track and V2V message within threshold are associated into a single object');
  assert(fused[0].source === 'v2v+local', 'Associated object source is correctly set to v2v+local');
}

// Test 5: Independent Hazard Assessment
{
  const unifiedModel = [
    { source: 'v2v', vehicleId: 2, x: 0, z: -30, distance: 30, speed: 5, heading: 0, braking: true, s: 50, lat: 0, vAlong: 5 }
  ];
  const vehicleState = { s: 0, u: 15, lat: 0 };
  const hazards = assessHazards(unifiedModel, vehicleState, null);
  
  assert(hazards.length === 1, 'Hazard assessment operates on unified model');
  assert(hazards[0].closingSpeed === 10, 'Hazard closing speed calculated correctly (15 - 5 = 10 m/s)');
  assert(Math.abs(hazards[0].ttc - 3.0) < 1e-4, 'Hazard TTC calculated correctly (30 / 10 = 3.0 s)');
}

// Test 6: Vehicle Behavior Combinations
{
  const ctx = { track: { kappaAt: () => 0, wrapS: s => s }, allVehicles: [], egoState: null };
  const vehNormal = { id: 10, speed: 15, v: 15, behavior: 'normal', radio: false, s: 0, lane: 0 };
  const vehErratic = { id: 11, speed: 15, v: 15, behavior: 'erratic', radio: true, s: 0, lane: 0 };

  const resNormal = computeVehicleBehavior(vehNormal, 0.1, ctx);
  const resErratic = computeVehicleBehavior(vehErratic, 0.1, ctx);

  assert(typeof resNormal.desiredSpeed === 'number', 'Normal vehicle computes valid desired speed');
  assert(typeof resErratic.desiredSpeed === 'number', 'Erratic vehicle computes valid desired speed');
  assert(vehErratic.radio === true, 'Radio capability is independent of vehicle behavior mode');
}

// Test 7: RL Observation Vector & Dimension
{
  const egoState = { u: 15, ax: 0.5, ay: 0.1, om: 0.02, beta: 0.01, laneDev: 0.2 };
  const roadInfo = { curvatureAhead: 0.001, laneCount: 2, oncomingLanes: 2, signalDist: 100, signalState: 'green' };
  const perceived = [{ relX: 0, relZ: -20, relSpeed: 5, distance: 20, braking: true, source: 'v2v+local', messageAge: 0.05 }];
  const ctrl = { steer: 0.05, throttle: 0.4, brake: 0 };

  const obs = buildObservation(egoState, roadInfo, perceived, ctrl);
  assert(obs instanceof Float32Array, 'Observation is a Float32Array');
  assert(obs.length === OBS_TOTAL, `Observation length is exactly ${OBS_TOTAL}`);
  assert(!obs.some(isNaN), 'Observation vector contains no NaN values');
}

// Test 8: RL Reward Function
{
  const state = { s: 10.5, u: 14, laneDev: 0.1, offRoad: false };
  const prevState = { s: 10.0, u: 14, laneDev: 0.1, offRoad: false };
  const action = { steer: 0, throttle: 0.5, brake: 0 };
  const prevAction = { steer: 0, throttle: 0.5, brake: 0 };
  const groundTruth = { collision: false, minDistance: 15, minTTC: 5.0 };

  const reward = calculateReward(state, prevState, action, prevAction, groundTruth);
  assert(typeof reward === 'number' && reward > 0, 'Positive reward for progress and smooth driving');

  const crashGT = { collision: true, minDistance: 0, minTTC: 0 };
  const crashReward = calculateReward(state, prevState, action, prevAction, crashGT);
  assert(crashReward < -50, 'Large negative penalty for collision');
}

// Test 9: Scenario Generation & Determinism
{
  const sc1 = generateScenario(999, 6);
  const sc2 = generateScenario(999, 6);
  assert(JSON.stringify(sc1) === JSON.stringify(sc2), 'Scenario generator is deterministic given the same seed and stage');
  assert(sc1.vehicles.length >= 3, 'Stage 6 generates sufficient vehicle traffic for V2V occlusion testing');
}

// Test 10: Vehicle Dynamics & Steering Authority
{
  const dyn = new VehicleDynamics();
  dyn.u = 0.0;
  const ctrl = { steer: -1.0, throttle: 1.0, brake: 0.0 };
  const env = { mu: 0.92, offroad: false, abs: true, esc: true };
  for (let i = 0; i < 30; i++) dyn.step(0.016, ctrl, env);
  assert(dyn.u > 0.5, 'Vehicle accelerates forward with throttle from standstill');
  assert(dyn.om < -0.3, `Low-speed steering generates responsive yaw rate (om=${dyn.om.toFixed(3)})`);
  assert(dyn.psi < -0.05, 'Vehicle turns right into negative heading on steer D');

  // Stationary check
  const dynIdle = new VehicleDynamics();
  for (let i = 0; i < 60; i++) dynIdle.step(0.016, { steer: 1.0, throttle: 0, brake: 0 }, env);
  assert(dynIdle.u === 0 && dynIdle.psi === 0, 'Stationary vehicle with zero throttle does not drift');
}

console.log(`\n--- TEST SUMMARY: ${passed} PASSED, ${failed} FAILED ---`);
if (failed > 0) process.exit(1);
