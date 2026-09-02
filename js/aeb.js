/* 
 * js/aeb.js — Autonomous Emergency Braking + 360° collision-avoidance shield
 *
 * Rays  : 36 @ 10°, index 0 = dead ahead.  1..17 right (9 = 90°R),
 *         18 = rear, 19..35 left (27 = 90°L, 35 = 10° left of bow).
 *
 *              0 (front)
 *        35 (−10°)   1 (+10°)
 *      27 (−90°)       9 (+90°)      ← blind-spot flanks
 *              18 (rear)
 */

export const AEB = {
  CRIT_DIST:          6.5,   // m — hard stop below this forward distance
  CRIT_TTC:           1.35,  // s — hard stop below this TTC
  WARN_FAR:           16.0,  // m — modulated braking begins
  WARN_BRAKE_MIN:     0.2,
  WARN_BRAKE_MAX:     0.85,
  WARN_THROTTLE_CAP:  0.3,   // throttle ceiling inside warning band
  BLIND_DIST:         2.2,   // m — flank inhibition range
  CORRIDOR_HALF_W:    1.15,  // m — path-relevant half-width (car ≈ 0.9 + margin)
  POS_STEER_IS_RIGHT: false, // In Three.js: +steer = left, -steer = right
};

export const FRONT_RAYS  = [35, 0, 1];    // ±10°
export const RIGHT_FLANK = [8, 9, 10];    // 80°–100° right
export const LEFT_FLANK  = [26, 27, 28];  // 80°–100° left
export const REAR_RAYS   = [17, 18, 19];  // used when reversing

const clamp = (x, lo, hi) => Math.min(hi, Math.max(lo, x));
const valid = (d) => Number.isFinite(d) && d > 0;     // 0 / Inf / NaN = no return

export const rayAngleDeg = (i) => (i <= 18 ? i * 10 : (i - 36) * 10);

/* Min distance over `idxs`, counting only rays whose lateral offset lies
 * inside the driving corridor (kills adjacent-lane false positives in curves). */
export function corridorMin(rays, idxs, halfWidth = AEB.CORRIDOR_HALF_W) {
  let m = Infinity;
  for (const i of idxs) {
    const d = rays[i];
    if (valid(d) && Math.abs(Math.sin(rayAngleDeg(i) * Math.PI / 180)) * d <= halfWidth && d < m) m = d;
  }
  return m;
}

export function rawMin(rays, idxs) {
  let m = Infinity;
  for (const i of idxs) { const d = rays[i]; if (valid(d) && d < m) m = d; }
  return m;
}

/**
 * Safety shield. `control` = candidate {steer, throttle, brake} from the
 * policy / driver / ADAS. Returns {steer, throttle, brake, alert, severity}.
 */
export function applyAEB(control, rays, speed, ttc) {
  const out = {
    steer:    clamp(control.steer   ?? 0, -1, 1),
    throttle: clamp(control.throttle ?? 0,  0, 1),
    brake:    clamp(control.brake   ?? 0,  0, 1),
    alert: 'NONE', severity: 0,
  };
  if (!rays || rays.length < 36) return out;          // sensor fault → pass-through

  // Normalize TTC: non-finite / non-positive = not closing
  const t = (Number.isFinite(ttc) && ttc > 0) ? ttc : Infinity;

  // Guard the rear corridor instead of the front when reversing
  const sector = (speed < -0.2) ? REAR_RAYS : FRONT_RAYS;
  const d = corridorMin(rays, sector);

  // ── (2) CRITICAL: hard stop ─────────────────────────────────────
  if (d < AEB.CRIT_DIST || t < AEB.CRIT_TTC) {
    out.throttle = 0.0;
    out.brake    = 1.0;
    out.alert    = 'DANGER';
    out.severity = 2;
  }
  // ── (3) WARNING: modulated braking, inversely proportional to d ─
  else if (d < AEB.WARN_FAR) {
    const b = clamp((AEB.WARN_FAR - d) / (AEB.WARN_FAR - AEB.CRIT_DIST),   // = (16−d)/9.5
                    AEB.WARN_BRAKE_MIN, AEB.WARN_BRAKE_MAX);
    out.brake    = Math.max(out.brake, b);            // only tighten
    out.throttle = Math.min(out.throttle, AEB.WARN_THROTTLE_CAP);
    out.alert    = 'WARNING';
    out.severity = 1;
  }

  // ── (4) 360° blind-spot steering inhibition ─────────────────────
  const rightObs = rawMin(rays, RIGHT_FLANK) < AEB.BLIND_DIST;
  const leftObs  = rawMin(rays, LEFT_FLANK)  < AEB.BLIND_DIST;
  const posRight = AEB.POS_STEER_IS_RIGHT;

  if (rightObs && (posRight ? out.steer > 0 : out.steer < 0)) out.steer = 0;
  if (leftObs  && (posRight ? out.steer < 0 : out.steer > 0)) out.steer = 0;

  if ((rightObs || leftObs) && out.severity < 1) {
    out.alert = 'BLIND_SPOT';
    out.severity = 1;
  }
  return out;
}

/* ── TTC helpers ────────────────────────────────────────────────── */
export function ttcFromSpeeds(dist, egoSpeed, obstacleSpeed) {
  const closing = egoSpeed - obstacleSpeed;           // + when closing
  return closing > 1e-3 ? dist / closing : Infinity;
}
export function ttcFromScans(prevDist, dist, dt) {
  const closing = (prevDist - dist) / dt;             // + when closing
  return closing > 1e-3 ? dist / closing : Infinity;
}

/* ── Optional: latched shield with hysteresis ───────────────────────
 * Prevents brake/alert chatter when d oscillates around 6.5 m: once a
 * critical stop fires, it holds until the scene is verifiably clear. */
export function makeAEBShield(hysteresisM = 1.5, ttcRecover = 2.0) {
  let latched = false;
  return function shield(control, rays, speed, ttc) {
    const out = applyAEB(control, rays, speed, ttc);
    const d = corridorMin(rays, speed < -0.2 ? REAR_RAYS : FRONT_RAYS);
    const t = (Number.isFinite(ttc) && ttc > 0) ? ttc : Infinity;
    if (out.severity === 2) latched = true;
    else if (latched) {
      if (d > AEB.CRIT_DIST + hysteresisM && t > ttcRecover) latched = false;
      else Object.assign(out, { throttle: 0, brake: 1, alert: 'DANGER', severity: 2 });
    }
    return out;
  };
}
