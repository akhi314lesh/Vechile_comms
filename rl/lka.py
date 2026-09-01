"""
rl/lka.py — Stanley-style lane centering / LKA with curvature feedforward.

Spec conventions:
  e_lat : + = vehicle RIGHT of lane center
  e_psi : + = vehicle heading LEFT of road tangent
  kappa : + = road curves LEFT
  omega : + = yaw LEFT (CCW)
  steer : + = turn LEFT

Set steer_positive='right' for the index.html sim (+1 steers right);
inputs are then expected in that mirrored convention (e_psi/omega/kappa
signs flip; e_lat stays + = right). Output +1 = steer right.
"""

import math

TAU = 2.0 * math.pi

def wrap_angle(a):
    """Wrap to [-pi, pi). Critical for e_psi: an unwrapped ~2*pi heading
    error would command full lock instead of a small correction."""
    return (a + math.pi) % TAU - math.pi

def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


class LaneKeepController:
    def __init__(self,
                 dt=1.0 / 30.0,          # control period (s) — set to your tick rate
                 k_lat_gain=0.42,        # cross-track gain numerator      (spec)
                 k_lat_speed=0.07,       # speed normalizer: k = g/max(1, v*s) (spec)
                 k_head=0.70,            # heading alignment/damping, 0.65–0.75 (spec)
                 wheelbase=3.2,          # m   — feedforward length term    (spec)
                 k_understeer=0.16,      # s   — feedforward speed term     (spec)
                 k_damp=0.0,             # excess-yaw damping; 0 = literal spec formula
                 max_slew=6.0,           # normalized units / s             (spec)
                 steer_positive='left',  # 'left' (spec) | 'right' (your sim)
                 deadband=0.0):          # m, optional anti-hunt on noisy lane estimate
        assert steer_positive in ('left', 'right')
        self.dt = float(dt)
        self.k_lat_gain, self.k_lat_speed = k_lat_gain, k_lat_speed
        self.k_head = k_head
        self.wheelbase, self.k_understeer = wheelbase, k_understeer
        self.k_damp = k_damp
        self.max_slew = float(max_slew)
        self.s = -1.0 if steer_positive == 'right' else 1.0
        self.deadband = deadband
        self.diag = {}
        self.reset(0.0)

    def reset(self, steer=0.0):
        """steer is in the OUTPUT convention (what update() returns),
        e.g. the actuator's current value after a mode handover."""
        self.steer = _clamp(self.s * float(steer), -1.0, 1.0)
        self.diag = {'raw': 0.0, 'k_lat': 0.0, 'ff': 0.0,
                     'saturated': False, 'slew_limited': False}

    def update(self, e_lat, e_psi, kappa, v, omega):
        """One control step. Returns normalized steer in [-1, 1]."""
        if not all(math.isfinite(x) for x in (e_lat, e_psi, kappa, v, omega)):
            return self.s * self.steer          # lane estimate dropout → hold
        e_lat, e_psi, kappa, v, omega = map(float, (e_lat, e_psi, kappa, v, omega))
        v = max(v, 0.0)                         # LKA assumes forward motion

        if self.s < 0.0:                        # mirror sim-space → spec-space
            e_psi, kappa, omega = -e_psi, -kappa, -omega

        # (3a) speed-scaled cross-track gain: responsive low, stable high
        k_lat = self.k_lat_gain / max(1.0, v * self.k_lat_speed)

        # (3c) curvature feedforward — proactive steer before error accumulates
        ff = kappa * (self.wheelbase + v * self.k_understeer)

        # (3b) heading alignment / damping (e_psi wrapped!)
        raw = k_lat * e_lat - self.k_head * wrap_angle(e_psi) + ff

        # Optional derivative damping on EXCESS yaw rate only:
        # (omega - v*kappa) is zero in perfect steady cornering, so this
        # term adds phase margin without fighting the curve itself.
        if self.k_damp:
            raw -= self.k_damp * (omega - v * kappa)

        # Optional micro-deadband (suppress hunting on noisy e_lat)
        if self.deadband > 0.0 and abs(e_lat) < self.deadband:
            raw -= k_lat * e_lat

        cmd = _clamp(raw, -1.0, 1.0)                       # (5) saturation
        step = self.max_slew * self.dt                     # (5) slew limit
        self.diag = {'raw': raw, 'k_lat': k_lat, 'ff': ff,
                     'saturated': abs(raw) > 1.0,
                     'slew_limited': abs(cmd - self.steer) > step + 1e-12}
        self.steer = _clamp(self.steer + _clamp(cmd - self.steer, -step, step),
                            -1.0, 1.0)
        return self.s * self.steer

if __name__ == '__main__':
    lka = LaneKeepController(steer_positive='left')
    steer = lka.update(e_lat=0.5, e_psi=0.05, kappa=0.01, v=15.0, omega=0.0)
    print(f"LKA test output: steer = {steer:.4f}, diag = {lka.diag}")
