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
                 tau_steer=0.08,         # s   — actuator lag
                 max_slew=6.0,           # normalized units / s             (spec)
                 steer_positive='left',  # 'left' (spec) | 'right' (your sim)
                 deadband=0.0):          # m, optional anti-hunt on noisy lane estimate
        assert steer_positive in ('left', 'right')
        self.dt = float(dt)
        self.k_lat_gain, self.k_lat_speed = k_lat_gain, k_lat_speed
        self.k_head = k_head
        self.wheelbase, self.k_understeer = wheelbase, k_understeer
        self.k_damp = k_damp
        self.tau_steer = tau_steer
        self.max_slew = float(max_slew)
        self.s = -1.0 if steer_positive == 'right' else 1.0
        self.deadband = deadband
        self.prev_raw = None
        self.diag = {}
        self.reset(0.0)

    def reset(self, steer=0.0):
        self.steer = _clamp(self.s * float(steer), -1.0, 1.0)
        self.prev_raw = None
        self.diag = {'raw': 0.0, 'k_lat': 0.0, 'ff': 0.0,
                     'saturated': False, 'slew_limited': False}

    def update(self, e_lat, e_psi, kappa, v, omega):
        if not all(math.isfinite(x) for x in (e_lat, e_psi, kappa, v, omega)):
            return self.s * self.steer
        e_lat, e_psi, kappa, v, omega = map(float, (e_lat, e_psi, kappa, v, omega))
        v = max(v, 0.0)
        v_kmh = v * 3.6

        if self.s < 0.0:
            e_psi, kappa, omega = -e_psi, -kappa, -omega

        wrapped_heading = wrap_angle(e_psi)

        # (1) Speed-decay cross-track gain
        if self.k_lat_speed >= 0.05:
            k_lat = self.k_lat_gain / max(1.0, v * self.k_lat_speed)
        else:
            k_lat = self.k_lat_gain / (1.0 + self.k_lat_speed * v)

        # (2) Heading and damping
        k_head = self.k_head
        k_damp = self.k_damp

        # (3) Curvature feedforward
        ff = kappa * (self.wheelbase + v * self.k_understeer)

        # (4) Smooth C^inf convergence scaling
        effective_k_lat = k_lat
        if self.k_damp > 0:
            is_converging = (e_lat > 0 and wrapped_heading < -0.01) or (e_lat < 0 and wrapped_heading > 0.01)
            if is_converging and abs(e_lat) < 0.45:
                effective_k_lat *= (0.60 + 0.40 * math.tanh(abs(e_lat) / 0.45))

        if self.deadband > 0.0 and abs(e_lat) < self.deadband:
            u = abs(e_lat) / self.deadband
            effective_k_lat *= (u * u)

        # (5) Raw feedback with excess yaw damping
        raw = effective_k_lat * e_lat - k_head * wrapped_heading + ff
        if k_damp > 0:
            excess_yaw_rate = omega - v * kappa
            raw -= k_damp * excess_yaw_rate

        # (6) Actuator phase lead compensation
        if self.k_damp > 0 and self.tau_steer > 0 and self.prev_raw is not None:
            raw_delta = raw - self.prev_raw
            lead_alpha = 0.40 * (self.tau_steer / self.dt)
            lead_raw = raw + _clamp(lead_alpha * raw_delta, -0.05, 0.05)
        else:
            lead_raw = raw
        self.prev_raw = raw

        # (7) Speed-adaptive slew limiting
        speed_slew_factor = _clamp(1.0 - (v / 45.0) * 0.35, 0.65, 1.0) if self.k_damp > 0 else 1.0
        max_step = self.max_slew * speed_slew_factor * self.dt

        cmd = _clamp(lead_raw, -1.0, 1.0)
        self.diag = {'raw': lead_raw, 'k_lat': effective_k_lat, 'ff': ff,
                     'saturated': abs(lead_raw) > 1.0,
                     'slew_limited': abs(cmd - self.steer) > max_step + 1e-12}
        self.steer = _clamp(self.steer + _clamp(cmd - self.steer, -max_step, max_step),
                            -1.0, 1.0)
        return self.s * self.steer

if __name__ == '__main__':
    lka = LaneKeepController(steer_positive='left')
    steer = lka.update(e_lat=0.5, e_psi=0.05, kappa=0.01, v=15.0, omega=0.0)
    print(f"LKA test output: steer = {steer:.4f}, diag = {lka.diag}")
