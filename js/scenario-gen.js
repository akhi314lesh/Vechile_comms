/**
 * js/scenario-gen.js — Seeded Scenario Generation for RL Training
 *
 * Each episode generates a deterministic scenario from a seed.
 * Curriculum stages progressively increase difficulty:
 *   1. Empty road, normal vehicles
 *   2. Normal traffic, different speeds
 *   3. Sudden braking
 *   4. Erratic vehicles
 *   5. Occluded vehicles
 *   6. V2V + occlusion
 *   7. Mixed V2V/non-V2V, normal/erratic
 */
import { createRNG, clamp } from './utils.js';

const PAINTS = [0x39404a, 0x9aa2ac, 0xc9ced4, 0x59626e, 0x7f8894, 0xe8631e, 0x3a5f8c, 0x8c3a3a];

/**
 * Generate a scenario configuration from a seed and curriculum stage.
 *
 * @param {number} seed - Deterministic seed
 * @param {number} stage - Curriculum stage (1-7)
 * @param {object} trackDef - Base track definition
 * @returns {object} Scenario configuration
 */
export function generateScenario(seed, stage, trackDef) {
  const rng = createRNG(seed);

  const scenario = {
    seed,
    stage,
    egoV2V: true, // ego always has V2V (can be toggled for comparison)
    vehicles: [],
    egoSpawnU: rng.randFloat(0.0, 0.2),
    trackMods: {},
    tags: {
      v2vEnabled: true,
      hasErratic: false,
      hasOccluded: false
    }
  };

  switch (stage) {
    case 1:
      // Empty road, 1-2 normal vehicles
      generateStage1(rng, scenario);
      break;
    case 2:
      // Normal traffic, varied speeds, 2-4 vehicles
      generateStage2(rng, scenario);
      break;
    case 3:
      // Sudden braking events, 2-4 vehicles
      generateStage3(rng, scenario);
      break;
    case 4:
      // Erratic vehicles, 2-4 vehicles
      generateStage4(rng, scenario);
      break;
    case 5:
      // Occluded vehicles (behind barrier), 2-5 vehicles
      generateStage5(rng, scenario);
      break;
    case 6:
      // V2V + occlusion, 3-5 vehicles
      generateStage6(rng, scenario);
      break;
    case 7:
      // Full mixed scenario
      generateStage7(rng, scenario);
      break;
    default:
      generateStage1(rng, scenario);
  }

  return scenario;
}

function makeVehicle(rng, overrides = {}) {
  return {
    id: rng.randInt(1, 99999),
    name: overrides.name ?? `CAR ${rng.randInt(1, 99)}`,
    dir: overrides.dir ?? 'fwd',
    lane: overrides.lane ?? 0,
    spawnU: overrides.spawnU ?? rng.randFloat(0.1, 0.9),
    speed: overrides.speed ?? rng.randFloat(8, 18),
    behavior: overrides.behavior ?? 'normal',
    radio: overrides.radio ?? true,
    hatch: overrides.hatch ?? rng.chance(0.5),
    paint: overrides.paint ?? rng.pick(PAINTS),
    ...overrides
  };
}

/* Stage 1: Empty road, 1-2 normal vehicles, all V2V */
function generateStage1(rng, scenario) {
  const n = rng.randInt(1, 2);
  for (let i = 0; i < n; i++) {
    scenario.vehicles.push(makeVehicle(rng, {
      behavior: 'normal',
      radio: true,
      speed: rng.randFloat(10, 16),
      spawnU: rng.randFloat(0.2 + i * 0.15, 0.4 + i * 0.15)
    }));
  }
}

/* Stage 2: Normal traffic, varied speeds */
function generateStage2(rng, scenario) {
  const n = rng.randInt(2, 4);
  for (let i = 0; i < n; i++) {
    scenario.vehicles.push(makeVehicle(rng, {
      behavior: 'normal',
      radio: true,
      speed: rng.randFloat(6, 22), // wide speed range
      lane: rng.randInt(0, 1),
      spawnU: rng.randFloat(0.1 + i * 0.1, 0.3 + i * 0.15)
    }));
  }
}

/* Stage 3: Sudden braking events */
function generateStage3(rng, scenario) {
  const n = rng.randInt(2, 4);
  for (let i = 0; i < n; i++) {
    const willBrake = i === 0 || rng.chance(0.4);
    scenario.vehicles.push(makeVehicle(rng, {
      behavior: willBrake ? 'brake' : 'normal',
      radio: true,
      speed: rng.randFloat(10, 18),
      spawnU: rng.randFloat(0.12 + i * 0.08, 0.25 + i * 0.12)
    }));
  }
}

/* Stage 4: Erratic vehicles */
function generateStage4(rng, scenario) {
  const n = rng.randInt(2, 4);
  scenario.tags.hasErratic = true;
  for (let i = 0; i < n; i++) {
    const isErratic = i === 0 || rng.chance(0.5);
    scenario.vehicles.push(makeVehicle(rng, {
      behavior: isErratic ? 'erratic' : 'normal',
      radio: true,
      speed: rng.randFloat(8, 20),
      spawnU: rng.randFloat(0.1 + i * 0.1, 0.3 + i * 0.12)
    }));
  }
}

/* Stage 5: Occluded vehicles (no V2V — pure local perception test) */
function generateStage5(rng, scenario) {
  const n = rng.randInt(2, 5);
  scenario.tags.hasOccluded = true;
  scenario.tags.v2vEnabled = false; // no V2V advantage
  scenario.egoV2V = false;

  // One vehicle behind barrier
  scenario.vehicles.push(makeVehicle(rng, {
    name: 'OCCLUDED',
    behavior: rng.chance(0.5) ? 'stopped' : 'normal',
    radio: false,
    speed: rng.randFloat(0, 10),
    spawnU: rng.randFloat(0.35, 0.55), // near barrier area
    lane: 0
  }));

  // Other normal vehicles
  for (let i = 1; i < n; i++) {
    scenario.vehicles.push(makeVehicle(rng, {
      behavior: 'normal',
      radio: false,
      speed: rng.randFloat(10, 16),
      spawnU: rng.randFloat(0.1 + i * 0.1, 0.3 + i * 0.12)
    }));
  }
}

/* Stage 6: V2V + occlusion (the core experiment) */
function generateStage6(rng, scenario) {
  const n = rng.randInt(3, 5);
  scenario.tags.hasOccluded = true;
  scenario.tags.v2vEnabled = true;
  scenario.egoV2V = true;

  // Key vehicle: behind barrier, WITH V2V — should be detectable
  scenario.vehicles.push(makeVehicle(rng, {
    name: 'V2V-OCCLUDED',
    behavior: rng.chance(0.4) ? 'erratic' : rng.chance(0.5) ? 'stopped' : 'brake',
    radio: true, // V2V enabled!
    speed: rng.randFloat(0, 14),
    spawnU: rng.randFloat(0.35, 0.55), // near barrier
    lane: 0
  }));
  if (scenario.vehicles[0].behavior === 'erratic') {
    scenario.tags.hasErratic = true;
  }

  // Other vehicles
  for (let i = 1; i < n; i++) {
    scenario.vehicles.push(makeVehicle(rng, {
      behavior: 'normal',
      radio: rng.chance(0.7),
      speed: rng.randFloat(10, 18),
      spawnU: rng.randFloat(0.1 + i * 0.08, 0.25 + i * 0.12)
    }));
  }
}

/* Stage 7: Full mixed scenario */
function generateStage7(rng, scenario) {
  const n = rng.randInt(3, 6);
  scenario.tags.v2vEnabled = rng.chance(0.7);
  scenario.egoV2V = scenario.tags.v2vEnabled;

  for (let i = 0; i < n; i++) {
    const behavior = rng.pick(['normal', 'normal', 'normal', 'erratic', 'brake', 'stopped']);
    const radio = rng.chance(0.5);

    if (behavior === 'erratic') scenario.tags.hasErratic = true;

    scenario.vehicles.push(makeVehicle(rng, {
      behavior,
      radio,
      speed: behavior === 'stopped' ? 0 : rng.randFloat(5, 22),
      lane: rng.randInt(0, 1),
      spawnU: rng.randFloat(0.05 + i * 0.06, 0.15 + i * 0.1)
    }));
  }

  // Potentially add an occluded vehicle
  if (rng.chance(0.6)) {
    scenario.tags.hasOccluded = true;
    scenario.vehicles.push(makeVehicle(rng, {
      name: 'OCCLUDED',
      behavior: rng.pick(['normal', 'erratic', 'stopped', 'brake']),
      radio: rng.chance(0.5),
      speed: rng.randFloat(0, 12),
      spawnU: rng.randFloat(0.35, 0.55),
      lane: 0
    }));
  }
}

/**
 * Generate a matched pair of scenarios for V2V comparison.
 * Same seed, same vehicles, but one with V2V enabled and one without.
 */
export function generateComparisonPair(seed, stage) {
  const withV2V = generateScenario(seed, stage);
  // Force all vehicles to have V2V
  withV2V.egoV2V = true;
  withV2V.tags.v2vEnabled = true;
  for (const v of withV2V.vehicles) v.radio = true;

  const noV2V = generateScenario(seed, stage);
  // Force all vehicles to have no V2V
  noV2V.egoV2V = false;
  noV2V.tags.v2vEnabled = false;
  for (const v of noV2V.vehicles) v.radio = false;

  return { withV2V, noV2V };
}

/**
 * Get curriculum stage progression thresholds.
 * Based on rolling average reward over recent episodes.
 */
export function shouldAdvanceStage(metrics, currentStage) {
  const recent = metrics.recent(50);
  if (recent.length < 30) return false;

  const avgReward = recent.reduce((s, e) => s + e.reward, 0) / recent.length;
  const collisionRate = recent.filter(e => e.collision).length / recent.length;

  // Stage advancement criteria: low collision rate + decent reward
  const thresholds = {
    1: { minReward: 50, maxCollisionRate: 0.15 },
    2: { minReward: 40, maxCollisionRate: 0.20 },
    3: { minReward: 30, maxCollisionRate: 0.25 },
    4: { minReward: 25, maxCollisionRate: 0.30 },
    5: { minReward: 20, maxCollisionRate: 0.30 },
    6: { minReward: 15, maxCollisionRate: 0.35 },
    7: { minReward: 10, maxCollisionRate: 0.40 } // final stage, always stays
  };

  const t = thresholds[currentStage] || thresholds[7];
  return avgReward >= t.minReward && collisionRate <= t.maxCollisionRate && currentStage < 7;
}
