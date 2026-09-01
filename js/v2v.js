/**
 * js/v2v.js — V2V Communication Model
 * 
 * BSM (Basic Safety Message) contains ONLY information the transmitting
 * vehicle legitimately knows about ITSELF. No perception data, no other
 * vehicle info.
 * 
 * V2V is an additional information source, NOT a prerequisite for
 * perception or hazard detection.
 */
import { MAX_BSM_AGE, SENSOR_RANGES, clamp } from './utils.js';

/**
 * Generate a BSM from a vehicle's own state.
 * This is the ONLY valid content for a BSM.
 */
export function generateBSM(vehicle, timestamp) {
  return {
    vehicleId: vehicle.id,
    timestamp,
    position: { x: vehicle.x ?? vehicle.car?.position?.x ?? 0, z: vehicle.z ?? vehicle.car?.position?.z ?? 0 },
    heading: vehicle.psi,
    speed: Math.abs(vehicle.v ?? 0),
    acceleration: vehicle.ax ?? 0,
    braking: !!(vehicle.braking || vehicle.forceT > 0 || vehicle.behavior === 'stopped'),
    lane: vehicle.lane ?? 0,
    trackPosition: vehicle.s ?? 0,
    lateralPosition: vehicle.lat ?? 0
  };
}

/**
 * BSM broadcast buffer — each equipped vehicle stores its latest BSM here.
 * Cleared and regenerated each tick.
 */
export class BSMBroadcastBuffer {
  constructor() {
    this.messages = new Map(); // vehicleId -> BSM
  }

  clear() {
    this.messages.clear();
  }

  /** Transmit: only vehicles with radio=true call this */
  transmit(vehicle, timestamp) {
    if (!vehicle.radio) return;
    const bsm = generateBSM(vehicle, timestamp);
    this.messages.set(vehicle.id, bsm);
  }

  /** Get all currently broadcast BSMs */
  getAll() {
    return Array.from(this.messages.values());
  }
}

/**
 * Receive BSMs for a specific vehicle.
 * Filters by: 
 *   - Not self
 *   - Within communication range
 *   - Not stale (message age < MAX_BSM_AGE)
 * Returns V2V objects with estimated positions (extrapolated by message age).
 */
export function receiveBSMs(receiverVehicle, bsmBuffer, currentTime, v2vRange) {
  if (!receiverVehicle.radio) return []; // No radio = no V2V

  const range = v2vRange ?? SENSOR_RANGES.v2v;
  const receiverX = receiverVehicle.x ?? receiverVehicle.car?.position?.x ?? 0;
  const receiverZ = receiverVehicle.z ?? receiverVehicle.car?.position?.z ?? 0;

  const received = [];

  for (const bsm of bsmBuffer.getAll()) {
    // Don't receive own BSM
    if (bsm.vehicleId === receiverVehicle.id) continue;

    // Range check
    const dx = bsm.position.x - receiverX;
    const dz = bsm.position.z - receiverZ;
    const dist = Math.hypot(dx, dz);
    if (dist > range) continue;

    // Staleness check
    const messageAge = currentTime - bsm.timestamp;
    if (messageAge > MAX_BSM_AGE) continue;

    // Extrapolate position based on message age
    const fwdX = -Math.sin(bsm.heading);
    const fwdZ = -Math.cos(bsm.heading);
    const estimatedX = bsm.position.x + bsm.speed * fwdX * messageAge;
    const estimatedZ = bsm.position.z + bsm.speed * fwdZ * messageAge;

    received.push({
      vehicleId: bsm.vehicleId,
      // Original BSM data
      bsmPosition: { x: bsm.position.x, z: bsm.position.z },
      heading: bsm.heading,
      speed: bsm.speed,
      acceleration: bsm.acceleration,
      braking: bsm.braking,
      lane: bsm.lane,
      trackPosition: bsm.trackPosition,
      lateralPosition: bsm.lateralPosition,
      // Computed fields
      distance: dist,
      messageAge,
      estimatedX,
      estimatedZ,
      // Source tag for fusion
      source: 'v2v'
    });
  }

  return received;
}
