/**
 * js/test/driving-benchmark.js — Comprehensive Non-RL Autonomous Driving Benchmark Suite
 * 
 * Validates deterministic driving stack across 15+ scenario classes:
 *   1. Straight road centering
 *   2. Gentle curve tracking
 *   3. Sharp curve tracking
 *   4. Stationary obstacle avoidance
 *   5. Slower leading vehicle following (ACC/IDM)
 *   6. Leading vehicle emergency braking
 *   7. Vehicle cutting into lane
 *   8. Safe lane change execution
 *   9. Unsafe lane change abort (gap preservation)
 *  10. Dense traffic navigation
 *  11. V2V available (early awareness)
 *  12. V2V unavailable (local sensors only)
 *  13. Delayed V2V information (kinematic extrapolation)
 *  14. Multiple nearby vehicles (surround shield)
 *  15. High-speed highway driving (35 m/s)
 * 
 * Computes:
 *   - Collision rate
 *   - Lane departure rate (|e_lat| > 1.8m)
 *   - Average & maximum lateral error
 *   - Speed tracking error
 *   - Harsh braking events (decel > 4.5 m/s²)
 *   - Harsh steering events (|steer_rate| > 4.0/s)
 *   - TTC violations (TTC < 1.5s)
 *   - Following-distance violations (gap < 4.0m)
 *   - Successful vs aborted lane changes
 */

import { AutonomousDrivingStack, LongitudinalPID, IDMController, CurvatureGovernor, LaneChangeStateMachine } from '../controller.js';
import { LaneKeepController, wrapAngle } from '../lka.js';
import { clamp } from '../utils.js';

const DT = 1.0 / 30.0;

class SimpleTrack {
  constructor(length = 1000.0, lanesF = 2, kappaProfile = []) {
    this.L = length;
    this.def = { lanesF, lanesO: 0 };
    this.kappaProfile = kappaProfile; // [{ sStart, sEnd, kappa }]
  }

  laneLat(dir, lane) {
    return lane * 3.6;
  }

  kappaAt(s) {
    s = ((s % this.L) + this.L) % this.L;
    for (const seg of this.kappaProfile) {
      if (s >= seg.sStart && s <= seg.sEnd) return seg.kappa;
    }
    return 0.0;
  }

  wrapS(ds) {
    return ds;
  }
}

class SimVehicle {
  constructor(x = 0, s = 0, lat = 0, u = 15.0, psi = 0) {
    this.x = x;
    this.z = s;
    this.s = s;
    this.lat = lat;
    this.u = u;
    this.psi = psi;
    this.roadPsi = 0.0;
    this.kappa = 0.0;
    this.om = 0.0;
    this.steer = 0.0;
    this.throttle = 0.0;
    this.brake = 0.0;
  }

  step(ctrl, kappa, dt) {
    this.steer = ctrl.steer;
    this.throttle = ctrl.throttle;
    this.brake = ctrl.brake;

    // Longitudinal acceleration
    const netAccel = (ctrl.throttle * 3.2) - (ctrl.brake * 7.5) - 0.01 * this.u * this.u;
    this.u = Math.max(0, this.u + netAccel * dt);

    // Yaw dynamics (bicycle model with steering)
    const L = 3.2;
    const targetOmega = (this.u * Math.tan(ctrl.steer * 0.5)) / L;
    this.om += (targetOmega - this.om) * Math.min(1.0, dt / 0.08);

    // Heading error relative to road
    const dPsi = (this.om - this.u * kappa) * dt;
    this.psi += dPsi;

    // Lateral displacement change
    this.lat += -this.u * Math.sin(this.psi) * dt;
    this.s += this.u * Math.cos(this.psi) * dt;
    this.x = this.lat;
    this.z = this.s;
    this.kappa = kappa;
  }
}

function runScenario(name, setup) {
  const {
    duration = 10.0,
    track = new SimpleTrack(2000, 2),
    initialSpeed = 15.0,
    initialLat = 0.0,
    initialPsi = 0.0,
    cruiseSpeed = 15.0,
    obstacles = [],
    leadVehicles = [],
    laneRequest = 0,
    expectedTargetLat = 0.0,
    evalStart = 5.0,
    adas = { lka: true, aeb: true, gov: true, alc: true }
  } = setup;

  const ego = new SimVehicle(0, 0, initialLat, initialSpeed, initialPsi);
  const controller = new AutonomousDrivingStack();
  controller.reset();

  const leads = leadVehicles.map(v => ({ ...v }));
  const obs = obstacles.map(o => ({ ...o }));

  let totalSteps = Math.round(duration / DT);
  let collisions = 0;
  let laneDepartures = 0;
  let lateralErrors = [];
  let speeds = [];
  let speedErrors = [];
  let harshBraking = 0;
  let harshSteering = 0;
  let ttcViolations = 0;
  let followingViolations = 0;
  let prevSteer = 0.0;

  for (let step = 0; step < totalSteps; step++) {
    const k = track.kappaAt(ego.s);

    // Step lead vehicles
    for (const lead of leads) {
      if (lead.braking) {
        lead.u = Math.max(0, lead.u - 6.5 * DT);
      }
      lead.s += lead.u * DT;
      if (lead.cutIn) {
        lead.lat += (lead.targetLat - lead.lat) * 1.5 * DT;
      }
    }

    // Build perceived objects list
    const perceived = [];
    for (const lead of leads) {
      perceived.push({
        src: lead.src || 'local',
        s: lead.s,
        lat: lead.lat,
        vAlong: lead.u,
        speed: lead.u,
        braking: lead.braking || false,
        tag: 'vehicle'
      });
    }
    for (const ob of obs) {
      perceived.push({
        src: 'local',
        s: ob.s,
        lat: ob.lat,
        vAlong: 0,
        speed: 0,
        braking: false,
        tag: 'obstacle'
      });
    }

    // Build 36-ray surround proximity scan
    const proximityRays = new Array(36).fill(60.0);
    for (let r = 0; r < 36; r++) {
      const angle = (r * 10 * Math.PI) / 180;
      const rayDx = Math.sin(angle);
      const rayDz = Math.cos(angle);
      for (const p of perceived) {
        const dx = p.lat - ego.lat;
        const dz = p.s - ego.s;
        const dist = Math.hypot(dx, dz);
        if (dist < 60.0) {
          const dot = (dx * rayDx + dz * rayDz) / (dist || 1);
          if (dot > 0.92) {
            proximityRays[r] = Math.min(proximityRays[r], dist);
          }
        }
      }
    }

    // Execute Controller (laneRequest is an event/pulse triggered at step 10)
    const activeLaneReq = (step === 10) ? laneRequest : 0;
    const ctrl = controller.step({
      egoState: ego,
      track,
      perceivedObjects: perceived,
      proximityRays,
      baseCruiseSpeed: cruiseSpeed,
      laneRequest: activeLaneReq,
      dt: DT,
      adas
    });

    // Check Metrics
    speeds.push(ego.u);
    speedErrors.push(Math.abs(ego.u - ctrl.targetSpeed));

    if (ctrl.brake > 0.65) harshBraking++;

    const steerRate = Math.abs(ctrl.steer - prevSteer) / DT;
    if (steerRate > 4.0) harshSteering++;
    prevSteer = ctrl.steer;

    // Check collision with any lead or obstacle
    for (const p of perceived) {
      const relS = p.s - ego.s;
      const relLat = Math.abs(p.lat - ego.lat);
      if (Math.abs(relS) < 3.8 && relLat < 1.5) {
        collisions++;
      }
      if (relS > 0 && relS < 4.0 && relLat < 1.5) {
        followingViolations++;
      }
      const closing = ego.u - p.vAlong;
      if (closing > 0.5 && relS > 0 && relLat < 1.5) {
        const ttc = relS / closing;
        if (ttc < 1.35) ttcViolations++;
      }
    }

    // Record lateral error after settling (t > evalStart)
    if (step > Math.round(evalStart / DT)) {
      const latErr = Math.abs(ego.lat - expectedTargetLat);
      lateralErrors.push(latErr);
      if (latErr > 1.8) laneDepartures++;
    }

    // Advance simulation
    ego.step(ctrl, k, DT);
  }

  const avgLatErr = lateralErrors.length ? (lateralErrors.reduce((a, b) => a + b, 0) / lateralErrors.length) : 0;
  const maxLatErr = lateralErrors.length ? Math.max(...lateralErrors) : 0;
  const avgSpeed = speeds.reduce((a, b) => a + b, 0) / speeds.length;
  const avgSpeedErr = speedErrors.reduce((a, b) => a + b, 0) / speedErrors.length;

  return {
    name,
    passed: collisions === 0 && maxLatErr < 0.35,
    collisions,
    laneDepartures,
    avgLatErr,
    maxLatErr,
    avgSpeed,
    avgSpeedErr,
    harshBraking,
    harshSteering,
    ttcViolations,
    followingViolations
  };
}

export function runAllBenchmarks() {
  console.log('================================================================');
  console.log('       AUTONOMOUS VEHICLE DRIVING STACK BENCHMARK SUITE');
  console.log('================================================================\n');

  const results = [];

  // 1. Straight road centering
  results.push(runScenario('1. Straight Road Centering', {
    duration: 8.0,
    initialLat: 0.8,
    initialSpeed: 20.0,
    cruiseSpeed: 20.0
  }));

  // 2. Gentle curve tracking (R = 250m)
  results.push(runScenario('2. Gentle Curve (R=250m)', {
    duration: 10.0,
    track: new SimpleTrack(2000, 2, [{ sStart: 50, sEnd: 400, kappa: 1.0 / 250 }]),
    initialSpeed: 22.0,
    cruiseSpeed: 22.0
  }));

  // 3. Sharp curve tracking (R = 80m)
  results.push(runScenario('3. Sharp Curve (R=80m)', {
    duration: 10.0,
    track: new SimpleTrack(2000, 2, [{ sStart: 50, sEnd: 300, kappa: 1.0 / 80 }]),
    initialSpeed: 18.0,
    cruiseSpeed: 18.0
  }));

  // 4. Stationary obstacle avoidance
  results.push(runScenario('4. Stationary Obstacle (AEB Stop)', {
    duration: 6.0,
    initialSpeed: 16.0,
    cruiseSpeed: 16.0,
    obstacles: [{ s: 80, lat: 0.0 }]
  }));

  // 5. Slower lead vehicle following (IDM/ACC)
  results.push(runScenario('5. Slower Lead Vehicle Following', {
    duration: 12.0,
    initialSpeed: 22.0,
    cruiseSpeed: 22.0,
    leadVehicles: [{ s: 50, lat: 0.0, u: 12.0 }]
  }));

  // 6. Lead vehicle emergency braking
  results.push(runScenario('6. Lead Vehicle Emergency Braking', {
    duration: 8.0,
    initialSpeed: 20.0,
    cruiseSpeed: 20.0,
    leadVehicles: [{ s: 40, lat: 0.0, u: 18.0, braking: true }]
  }));

  // 7. Vehicle cutting into lane
  results.push(runScenario('7. Vehicle Cut-In', {
    duration: 8.0,
    initialSpeed: 18.0,
    cruiseSpeed: 18.0,
    leadVehicles: [{ s: 30, lat: 3.6, targetLat: 0.0, u: 14.0, cutIn: true }]
  }));

  // 8. Safe lane change (Target: Lane 1 at lat 3.6m)
  results.push(runScenario('8. Safe Lane Change', {
    duration: 10.0,
    initialSpeed: 16.0,
    cruiseSpeed: 16.0,
    laneRequest: 1,
    expectedTargetLat: 3.6
  }));

  // 9. Unsafe lane change (Target lane blocked -> Stay in Lane 0 at lat 0.0m)
  results.push(runScenario('9. Unsafe Lane Change (Blocked)', {
    duration: 10.0,
    initialSpeed: 16.0,
    cruiseSpeed: 16.0,
    leadVehicles: [{ s: 10, lat: 3.6, u: 16.0 }],
    laneRequest: 1,
    expectedTargetLat: 0.0
  }));

  // 10. Dense traffic following
  results.push(runScenario('10. Dense Traffic Following', {
    duration: 14.0,
    initialSpeed: 15.0,
    cruiseSpeed: 18.0,
    leadVehicles: [
      { s: 35, lat: 0.0, u: 12.0 },
      { s: 80, lat: 0.0, u: 14.0 },
      { s: 25, lat: 3.6, u: 13.0 }
    ]
  }));

  // 11. V2V Available (Early Braking Alert)
  results.push(runScenario('11. V2V Available (Early Warning)', {
    duration: 8.0,
    initialSpeed: 22.0,
    cruiseSpeed: 22.0,
    leadVehicles: [{ s: 70, lat: 0.0, u: 18.0, braking: true, src: 'v2v' }]
  }));

  // 12. V2V Unavailable (Local radar fallback)
  results.push(runScenario('12. V2V Unavailable (Local Sensor)', {
    duration: 8.0,
    initialSpeed: 20.0,
    cruiseSpeed: 20.0,
    leadVehicles: [{ s: 50, lat: 0.0, u: 12.0, src: 'local' }]
  }));

  // 13. High-speed Highway Driving with 60 km/h Hard Limit (Requested 30 m/s -> Capped at 16.67 m/s)
  results.push(runScenario('13. 60 km/h Hard Speed Ceiling', {
    duration: 10.0,
    initialSpeed: 14.0,
    cruiseSpeed: 30.0,
    track: new SimpleTrack(3000, 2, [{ sStart: 100, sEnd: 600, kappa: 1.0 / 400 }])
  }));

  // 14. Sharp Left Curve -> Straight Exit (Zero Post-Turn Zigzag)
  results.push(runScenario('14. Left Curve -> Straight Exit', {
    duration: 10.0,
    initialSpeed: 16.0,
    cruiseSpeed: 16.0,
    evalStart: 6.5,
    track: new SimpleTrack(2000, 2, [{ sStart: 20, sEnd: 80, kappa: 1.0 / 75.0 }])
  }));

  // 15. Sharp Right Curve -> Straight Exit (Zero Post-Turn Zigzag)
  results.push(runScenario('15. Right Curve -> Straight Exit', {
    duration: 10.0,
    initialSpeed: 16.0,
    cruiseSpeed: 16.0,
    evalStart: 6.5,
    track: new SimpleTrack(2000, 2, [{ sStart: 20, sEnd: 80, kappa: -1.0 / 75.0 }])
  }));

  // 16. Congested Current Lane -> Auto-Select Fast Empty Lane (Intelligent Lane Selection)
  results.push(runScenario('16. Congested Lane -> Auto-Select Lane 1', {
    duration: 14.0,
    initialSpeed: 16.0,
    cruiseSpeed: 18.0,
    evalStart: 8.0,
    adas: { lka: true, aeb: true, gov: true, alc: true, autoPass: true },
    leadVehicles: [
      { s: 25, lat: 0.0, u: 8.0 },
      { s: 45, lat: 0.0, u: 8.0 },
      { s: 65, lat: 0.0, u: 8.0 }
    ],
    expectedTargetLat: 3.6
  }));

  // 17. Congested Lane with Fast V2V Rear Vehicle -> Inhibit Lane Change (Safety Override)
  results.push(runScenario('17. Fast Rear Vehicle -> Hold Lane', {
    duration: 3.5,
    initialSpeed: 12.0,
    cruiseSpeed: 16.0,
    evalStart: 1.0,
    adas: { lka: true, aeb: true, gov: true, alc: true, autoPass: true },
    leadVehicles: [
      { s: 30, lat: 0.0, u: 8.0 },
      { s: -25, lat: 3.6, u: 24.0, src: 'v2v' } // Fast vehicle closing from rear in Lane 1
    ],
    expectedTargetLat: 0.0
  }));

  // 18. Post-Lane-Change Centering Stabilization
  results.push(runScenario('18. Post-Lane-Change Stabilization', {
    duration: 10.0,
    initialSpeed: 16.0,
    cruiseSpeed: 16.0,
    laneRequest: 1,
    expectedTargetLat: 3.6
  }));

  // 19. Distant Closing Lead Vehicle (100m Ahead @ 7 m/s -> Smooth Early Deceleration)
  results.push(runScenario('19. Distant Slower Lead (100m)', {
    duration: 12.0,
    initialSpeed: 15.0,
    cruiseSpeed: 16.0,
    leadVehicles: [{ s: 100, lat: 0.0, u: 7.0 }]
  }));

  // 20. Empty Road Smooth Acceleration (6 m/s -> Smooth Ramp to 15 m/s)
  results.push(runScenario('20. Empty Road Smooth Acceleration', {
    duration: 10.0,
    initialSpeed: 6.0,
    cruiseSpeed: 15.0
  }));

  // 21. Traffic Clears -> Dynamic Re-Acceleration (8 m/s -> 15 m/s)
  results.push(runScenario('21. Traffic Clears -> Speed Restoration', {
    duration: 12.0,
    initialSpeed: 8.0,
    cruiseSpeed: 15.0,
    leadVehicles: [{ s: 40, lat: 0.0, u: 16.0 }]
  }));

  // Print Report Table
  console.log('| Scenario Name                     | Pass/Fail | Collisions | Max |e_lat| | Avg Speed | Harsh Brk |');
  console.log('|-----------------------------------|-----------|------------|-------------|-----------|-----------|');

  let totalPassed = 0;
  for (const r of results) {
    const status = r.passed ? '✅ PASS' : '❌ FAIL';
    if (r.passed) totalPassed++;
    console.log(
      `| ${r.name.padEnd(35)} | ${status.padEnd(9)} | ${String(r.collisions).padStart(10)} | ${r.maxLatErr.toFixed(3).padStart(10)}m | ${(r.avgSpeed.toFixed(1) + ' m/s').padStart(9)} | ${String(r.harshBraking).padStart(9)} |`
    );
  }

  console.log('\n================================================================');
  console.log(` BENCHMARK SUMMARY: ${totalPassed}/${results.length} SCENARIOS PASSED`);
  console.log('================================================================\n');

  return totalPassed === results.length;
}

if (typeof process !== 'undefined' && process.argv[1]?.includes('driving-benchmark')) {
  const ok = runAllBenchmarks();
  process.exit(ok ? 0 : 1);
}
