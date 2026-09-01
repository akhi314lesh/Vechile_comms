import assert from 'node:assert';
import { VehicleDynamics } from '../physics.js';

console.log('--- TESTING VEHICLE DYNAMICS & STEERING DYNAMICS ---');

// 1. Low-speed steering with throttle generates responsive yaw rate
{
  const dyn = new VehicleDynamics();
  dyn.u = 0.0;
  const ctrl = { steer: -1.0, throttle: 1.0, brake: 0.0 }; // Turn right (D) + throttle (W)
  const env = { mu: 0.92, offroad: false, abs: true, esc: true };
  const dt = 0.016;

  // Step 30 frames (0.5s)
  for (let i = 0; i < 30; i++) {
    dyn.step(dt, ctrl, env);
  }

  assert(dyn.u > 0.5, 'Vehicle should accelerate forward with throttle');
  assert(dyn.om < -0.3, `Yaw rate should be responsive: got om=${dyn.om}`);
  assert(dyn.psi < -0.05, `Heading should turn right (negative psi): got psi=${dyn.psi}`);
  console.log(`✅ PASS: Low-speed steering responsive (psi=${(dyn.psi * 180 / Math.PI).toFixed(2)}°, om=${dyn.om.toFixed(3)}, u=${dyn.u.toFixed(2)} m/s)`);
}

// 2. Stationary without throttle does not drift or spin
{
  const dyn = new VehicleDynamics();
  dyn.u = 0.0;
  const ctrl = { steer: 1.0, throttle: 0.0, brake: 0.0 }; // Full lock, zero throttle
  const env = { mu: 0.92, offroad: false, abs: true, esc: true };
  const dt = 0.016;

  for (let i = 0; i < 60; i++) {
    dyn.step(dt, ctrl, env);
  }

  assert.strictEqual(dyn.u, 0, 'Stationary car without throttle should remain at 0 speed');
  assert.strictEqual(dyn.psi, 0, 'Stationary car without throttle should not drift or rotate');
  console.log('✅ PASS: Stationary vehicle with zero throttle does not drift');
}

// 3. Reverse steering
{
  const dyn = new VehicleDynamics();
  dyn.u = 0.0;
  const ctrl = { steer: -1.0, throttle: -0.4, brake: 0.0 }; // Reverse + steer right
  const env = { mu: 0.92, offroad: false, abs: true, esc: true };
  const dt = 0.016;

  for (let i = 0; i < 30; i++) {
    dyn.step(dt, ctrl, env);
  }

  assert(dyn.u < -0.2, 'Vehicle should accelerate backwards');
  assert(dyn.om > 0.1, 'In reverse, turning right should pivot nose right/om positive');
  console.log(`✅ PASS: Reverse steering responsive (psi=${(dyn.psi * 180 / Math.PI).toFixed(2)}°, om=${dyn.om.toFixed(3)}, u=${dyn.u.toFixed(2)} m/s)`);
}

console.log('--- ALL BARRIER & STEERING PHYSICS TESTS PASSED ---');
