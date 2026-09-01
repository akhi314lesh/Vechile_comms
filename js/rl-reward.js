/**
 * js/rl-reward.js — RL Reward Function
 *
 * The reward encourages safe, smooth driving and hazard avoidance.
 * Ground truth is used ONLY for reward calculation (never in observation).
 *
 * Positive:
 *   - Forward progress
 *   - Reasonable speed
 *   - Lane keeping
 *   - Smooth controls
 *   - Successful collision avoidance
 *
 * Negative:
 *   - Collision (large penalty)
 *   - Near collision
 *   - Low TTC
 *   - Off-road driving
 *   - Violent controls
 */
import { clamp } from './utils.js';

/**
 * Calculate the reward for a single timestep.
 *
 * @param {object} state      - Current ego state
 * @param {object} prevState  - Previous ego state
 * @param {object} action     - Control action applied { steer, throttle, brake }
 * @param {object} prevAction - Previous control action
 * @param {object} groundTruth - Privileged data (collisions, min TTC, etc.)
 * @param {object} config     - Reward weights (optional)
 * @returns {number} reward
 */
export function calculateReward(state, prevState, action, prevAction, groundTruth, config = {}) {
  const w = {
    progress: config.progress ?? 1.0,
    speed: config.speed ?? 0.3,
    lane: config.lane ?? 0.5,
    smooth: config.smooth ?? 0.2,
    collision: config.collision ?? -100,
    nearMiss: config.nearMiss ?? -5.0,
    lowTTC: config.lowTTC ?? -2.0,
    offRoad: config.offRoad ?? -3.0,
    alive: config.alive ?? 0.1,
    ...config
  };

  let reward = 0;

  /* ---- Forward progress ---- */
  const progressDist = (state.s ?? 0) - (prevState.s ?? 0);
  // Normalize: ~0.5 reward per meter at cruise speed
  reward += w.progress * clamp(progressDist * 0.5, -0.5, 2.0);

  /* ---- Speed reward ---- */
  // Encourage maintaining a reasonable speed (10-20 m/s = 36-72 km/h)
  const speed = Math.abs(state.u ?? 0);
  const targetSpeed = state.targetSpeed ?? 14; // ~50 km/h default
  const speedErr = Math.abs(speed - targetSpeed) / targetSpeed;
  reward += w.speed * Math.max(0, 1 - speedErr);

  /* ---- Lane keeping ---- */
  const laneDev = Math.abs(state.laneDev ?? 0);
  // Smooth penalty: 0 at center, -1 at 2m deviation
  reward += w.lane * Math.max(0, 1 - laneDev / 2.0);

  /* ---- Smoothness ---- */
  if (prevAction) {
    const dSteer = Math.abs((action.steer ?? 0) - (prevAction.steer ?? 0));
    const dThrottle = Math.abs((action.throttle ?? 0) - (prevAction.throttle ?? 0));
    const dBrake = Math.abs((action.brake ?? 0) - (prevAction.brake ?? 0));
    const jerk = dSteer + dThrottle * 0.5 + dBrake * 0.5;
    reward += w.smooth * Math.max(0, 1 - jerk * 3);
  }

  /* ---- Alive bonus ---- */
  reward += w.alive;

  /* ---- Collision penalty ---- */
  if (groundTruth.collision) {
    reward += w.collision;
  }

  /* ---- Near-miss penalty ---- */
  if (groundTruth.minDistance !== undefined && groundTruth.minDistance < 3.0 && !groundTruth.collision) {
    const severity = 1 - clamp(groundTruth.minDistance / 3.0, 0, 1);
    reward += w.nearMiss * severity;
  }

  /* ---- Low TTC penalty ---- */
  if (groundTruth.minTTC !== undefined && groundTruth.minTTC < 3.0 && !groundTruth.collision) {
    const severity = 1 - clamp(groundTruth.minTTC / 3.0, 0, 1);
    reward += w.lowTTC * severity;
  }

  /* ---- Off-road penalty ---- */
  if (state.offRoad) {
    reward += w.offRoad;
  }

  return reward;
}

/**
 * Check termination conditions.
 * Returns { done, truncated, reason }
 */
export function checkTermination(state, groundTruth, step, maxSteps) {
  if (groundTruth.collision) {
    return { done: true, truncated: false, reason: 'collision' };
  }

  if (step >= maxSteps) {
    return { done: false, truncated: true, reason: 'timeout' };
  }

  // Off-road for too long
  if (state.offRoadTime > 3.0) {
    return { done: true, truncated: false, reason: 'off_road' };
  }

  // Stuck (speed near zero for too long)
  if (state.stuckTime > 5.0) {
    return { done: true, truncated: false, reason: 'stuck' };
  }

  // Completed a full lap
  if (state.lapsCompleted >= 1) {
    return { done: true, truncated: false, reason: 'lap_complete' };
  }

  return { done: false, truncated: false, reason: '' };
}
