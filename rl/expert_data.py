# rl/expert_data.py — expert demonstration generator v3
import math
import numpy as np

DT       = 1.0 / 30.0   # matches 30 Hz sim
GAMMA    = 0.99
OBS_DIM  = 97
LANE_W   = 3.6
N_SLOTS  = 12
LAT_LIM  = LANE_W * 1.2

# ─── Expert controller (calibrated to Three.js simulator coordinates) ───
def expert_steering(lat, yaw_rate, kappa, speed):
    """
    Stanley steering matching Three.js coordinate system in index.html:
      +lat = car is to the right of lane center -> steer > 0 (turn left back to center)
      -lat = car is to the left of lane center  -> steer < 0 (turn right back to center)
      +yaw_rate = car rotating left -> damping steer < 0 (turn right to stabilize)
      +kappa = road curves left -> feedforward steer > 0 (turn left along curve)
    """
    k_lat = 0.45 / max(1.0, speed * 0.08)
    steer = k_lat * lat - 0.35 * yaw_rate + kappa * max(3.5, speed) * 0.60
    return float(np.clip(steer, -1.0, 1.0))

def expert_longitudinal(speed, v_des, lead_dist, lead_rel_speed, is_braking_ahead):
    if lead_dist < 6.0:
        return 0.0, 1.0
    ttc = lead_dist / max(0.01, -lead_rel_speed) if lead_rel_speed < -0.5 else 99.0
    if ttc < 1.6:
        return 0.0, float(np.clip(1.6 / max(0.1, ttc), 0.6, 1.0))
    if is_braking_ahead and lead_dist < 45.0:
        v_des = min(v_des, speed * 0.5)
    if lead_dist < 999:
        s_star = max(4.5, 4.5 + speed * 1.5 + speed * max(0.0, -lead_rel_speed) / (2 * math.sqrt(6.0)))
        if lead_dist < s_star:
            v_des = min(v_des, speed * (lead_dist / s_star) ** 2)
    err = v_des - speed
    if err > 0.3:   return float(np.clip(err / 5.0, 0.08, 0.85)), 0.0
    elif err < -1.5: return 0.0, float(np.clip(-err / 8.0, 0.08, 0.7))
    return 0.12, 0.0

def safe_curve_speed(kappa):
    if abs(kappa) < 0.002: return 30.0
    return min(30.0, math.sqrt(0.8 * 9.81 / abs(kappa)))

# ─── ONE consistent world model (domain-randomized per episode) ───
def make_dynamics(rng):
    return {'steer_gain': rng.uniform(0.06, 0.10), 'steer_tau': rng.uniform(0.12, 0.22),
            'accel': rng.uniform(2.8, 4.2), 'brake': rng.uniform(5.5, 7.5),
            'drag': rng.uniform(0.010, 0.022)}

def step_ego(s, steer, throttle, brake, kappa, dyn, rng):
    ax_t = dyn['accel'] * throttle - dyn['brake'] * brake - dyn['drag'] * s['speed']
    s['ax'] += (ax_t - s['ax']) * min(1.0, DT / 0.15)
    s['speed'] = float(np.clip(s['speed'] + s['ax'] * DT, 0.0, 33.0))
    yr_t = steer * dyn['steer_gain'] * s['speed'] + kappa * 0.5 * s['speed']
    s['yaw_rate'] += (yr_t - s['yaw_rate']) * min(1.0, DT / dyn['steer_tau']) + rng.normal(0, 0.008)
    # When turning left (steer > 0, yaw_rate > 0), lateral deviation decreases (moves left toward 0)
    s['lat'] += -s['yaw_rate'] * s['speed'] * 0.25 * DT - steer * 0.010 \
                + kappa * s['speed'] * 0.01 + rng.normal(0, 0.006)
    s['lat'] = float(np.clip(s['lat'], -LAT_LIM, LAT_LIM))
    s['ay'] = s['yaw_rate'] * s['speed'] * 0.4
    s['beta'] = math.atan2(s['yaw_rate'] * 1.5, max(s['speed'], 2.0))

def make_road(rng, kind):
    if kind == 'straight':  return lambda x: 0.0
    k0 = {'gentle': rng.uniform(0.005, 0.015), 'curve': rng.uniform(0.015, 0.045),
          'tight':  rng.uniform(0.030, 0.060)}[kind] * rng.choice([-1, 1])
    s1, hold = rng.uniform(20, 60), rng.uniform(60, 140)
    entry = 30.0 if kind != 'tight' else 15.0
    def kappa(x):   # smooth (clothoid-like) entry/hold/exit
        e = min(max(min(x - s1, s1 + hold + entry - x, entry) / entry, 0.0), 1.0)
        return k0 * (e * e * (3.0 - 2.0 * e))
    return kappa

# ─── Traffic objects with events (braking, cut-in, stop-and-go waves) ───
class TrafficObj:
    def __init__(self, rng, dist, speed, rel_x=0.0, v2v=True, brake_at=None,
                 brake_rate=0.35, cut_at=None, wave=None, reveal_dist=999.0):
        self.dist, self.speed, self.rel_x = float(dist), float(speed), float(rel_x)
        self.v2v, self.reveal_dist = v2v, reveal_dist
        self.brake_at, self.brake_rate, self.brake_t0 = brake_at, brake_rate, None
        self.cut_at, self.cut_dur, self._x0 = cut_at, rng.uniform(1.2, 2.8), float(rel_x)
        self.wave, self.braking = wave, False
    def step(self, t, ego_speed, rng):
        if self.wave is not None:
            base, amp, per = self.wave
            self.speed = float(np.clip(base + amp * math.sin(2 * math.pi * t / per), 0, base + amp))
        elif self.brake_at is not None and t >= self.brake_at:
            self.brake_t0 = self.brake_t0 or t
            self.braking = self.speed > 0.1
            self.speed = max(0.0, self.speed - self.brake_rate)
        else:
            self.speed = max(0.0, self.speed + rng.normal(0, 0.15 * DT))
        if self.cut_at is not None and t >= self.cut_at:
            f = min(1.0, (t - self.cut_at) / self.cut_dur); f = f * f * (3 - 2 * f)
            self.rel_x = self._x0 * (1.0 - f)     # cut toward ego lane
        self.dist = float(np.clip(self.dist + (self.speed - ego_speed) * DT, 0.5, 300.0))

# ─── Structured perception: dropout, staleness, V2V loss, brake-light lag ───
class Perception:
    def __init__(self, rng, p_detect=0.98, p_v2v=0.95, dist_noise=0.4,
                 relv_noise=0.3, lat_noise=0.3, brake_delay=0.3, max_age=0.8):
        self.rng, self.p_detect, self.p_v2v = rng, p_detect, p_v2v
        self.dn, self.vn, self.ln = dist_noise, relv_noise, lat_noise
        self.brake_delay, self.max_age = brake_delay, max_age
        self.tracks = {}
    def sense(self, objs, t, ego_speed):
        rng, out = self.rng, []
        for o in objs:
            tr = self.tracks.get(id(o))
            detectable = o.dist < o.reveal_dist and abs(o.rel_x) < 2.5 * LANE_W
            detected = detectable and rng.random() < self.p_detect
            v2v_rx = o.v2v and rng.random() < self.p_v2v
            if tr is None:
                if not (detected or v2v_rx):
                    continue                     # not yet observed → absent from obs
                tr = {'braking': False, 'v2v': False, 'age': 0.0, 'last_v2v': -9.0,
                      'dist': o.dist + rng.normal(0, 8.0 if not detected else self.dn),
                      'rel_x': o.rel_x + rng.normal(0, self.ln),
                      'rel_speed': (o.speed - ego_speed) + rng.normal(0, self.vn)}
                self.tracks[id(o)] = tr
            if detected:
                tr['dist'] = o.dist + rng.normal(0, self.dn + 0.01 * o.dist)
                tr['rel_x'] = o.rel_x + rng.normal(0, self.ln)
                tr['rel_speed'] = (o.speed - ego_speed) + rng.normal(0, self.vn)
                lag = o.braking and o.brake_t0 is not None and (t - o.brake_t0) > self.brake_delay
                tr['braking'] = (not lag) if rng.random() < 0.02 else lag   # rare misflag
            if v2v_rx:
                tr['v2v'], tr['last_v2v'] = True, t
                tr['braking'] = o.braking
                tr['rel_speed'] = (o.speed - ego_speed) + rng.normal(0, 0.15)
                tr['dist'] = tr['dist'] + rng.normal(0, 0.2)  # v2v range is coarse
                tr['age'] = 0.0
            elif detected:
                tr['age'] = 0.0
            else:
                tr['age'] += DT
                if tr['age'] > self.max_age:
                    del self.tracks[id(o)]       # track lost → object vanishes
                    continue
            if t - tr['last_v2v'] > 0.5:
                tr['v2v'] = False
            out.append(tr)
        return out

# ─── Expert wrapper: rate-limited actions, in-lane lead selection, measured state ───
class Expert:
    RATE = np.array([8.0, 6.0, 12.0])            # steer/throttle/brake slew (units/s)
    def __init__(self): self.prev = np.zeros(3)
    def act(self, m, tracks, v_des):
        ld, lr, lb = 999.0, 0.0, False
        for tr in tracks:                        # lead = nearest IN-LANE track
            if abs(tr['rel_x']) < 1.5 and tr['dist'] < ld:
                ld, lr, lb = tr['dist'], tr['rel_speed'], tr['braking']
        steer = expert_steering(m['lat'], m['yaw_rate'], m['kappa'], m['speed'])
        thr, brk = expert_longitudinal(m['speed'], v_des, ld, lr, lb)
        raw = np.array([steer, thr, brk], dtype=np.float32)
        out = self.prev + np.clip(raw - self.prev, -Expert.RATE * DT, Expert.RATE * DT)
        out[0] = np.clip(out[0], -1, 1); out[1:] = np.clip(out[1:], 0, 1)
        self.prev = out
        return out

# ─── Reward proxy + obs builder ───
def reward_step(s, objs, v_des, a):
    r = 0.5 * min(s['speed'] / max(v_des, 1.0), 1.15)
    r -= 0.35 * min(abs(s['lat']) / 3.0, 1.5)
    r -= 0.05 * (abs(s['ax']) / 6.0 + abs(s['ay']) / 6.0)
    r -= 0.02 * (abs(a[0]) + a[2])
    if objs:
        d = min(o.dist for o in objs)
        if d < 4.0:   r -= 3.0 * (4.0 - d)
        else:
            hz = 4.0 + s['speed'] * 1.2
            if d < hz: r -= 0.3 * (1.0 - d / hz)
    return r

def build_obs(m, tracks, prev_a):
    obs = np.zeros(OBS_DIM, dtype=np.float32)
    obs[0] = np.clip(m['speed'] / 50.0, 0, 1)
    obs[1] = np.clip(m['ax'] / 12.0, -1, 1)
    obs[2] = np.clip(m['ay'] / 12.0, -1, 1)
    obs[3] = np.clip(m['yaw_rate'] / 2.6, -1, 1)
    obs[4] = np.clip(m.get('beta', 0.0) / 0.5, -1, 1)
    obs[5] = np.clip(m['lat'] / 5.0, -1, 1)
    obs[6] = np.clip(prev_a[0], -1, 1)
    obs[7] = np.clip(prev_a[1], 0, 1)
    obs[8] = np.clip(prev_a[2], 0, 1)
    obs[9]  = np.clip(m['kappa'] / 0.1, -1, 1)
    obs[10] = m['lanes_f']; obs[11] = m['lanes_o']; obs[12] = 0.0
    for i, tr in enumerate(sorted(tracks, key=lambda x: x['dist'])[:N_SLOTS]):
        b = 13 + i * 7
        obs[b+0] = np.clip(tr['rel_x'] / 250.0, -1, 1)
        obs[b+1] = np.clip(tr['dist'] / 250.0, -1, 1)
        obs[b+2] = np.clip(tr['rel_speed'] / 60.0, -1, 1)
        obs[b+3] = np.clip(tr['dist'] / 250.0, 0, 1)
        obs[b+4] = 1.0 if tr['braking'] else 0.0
        obs[b+5] = 1.0 if (tr.get('v2v', False) and tr.get('seen', True)) else (0.5 if tr.get('v2v', False) else 0.0)
        obs[b+6] = np.clip(tr['age'] / 2.0, 0, 1)
    for i in range(len(tracks), N_SLOTS):
        obs[13 + i * 7 + 3] = 1.0
    return obs

# ─── Episode rollout ───
def run_episode(rng, cfg, n_steps=200):
    road, dyn = make_road(rng, cfg['road']), make_dynamics(rng)
    v_des = rng.uniform(*cfg.get('v_des', (13.0, 15.0)))
    s = {'speed': rng.uniform(*cfg.get('v0', (6.0, 16.0))),
         'lat': rng.uniform(*cfg.get('lat0', (-1.0, 1.0))),
         'yaw_rate': rng.uniform(-0.05, 0.05), 'ax': 0.0, 'ay': 0.0, 'beta': 0.0, 's_pos': 0.0}
    objs = cfg.get('objects', lambda r: [])(rng)
    q = cfg.get('perception', 'good')
    perc = {'good':     Perception(rng, 0.98, 0.95, brake_delay=0.3),
            'nov2v':    Perception(rng, 0.97, 0.0,  brake_delay=0.6),
            'degraded': Perception(rng, 0.85, 0.4,  dist_noise=1.2, relv_noise=0.8,
                                   brake_delay=1.0, max_age=1.2)}[q]
    lanes = rng.choice([1, 2, 2, 3])
    lat_bias, lat_bias_v = 0.0, rng.uniform(-0.05, 0.05)

    expert, prev_a = Expert(), np.zeros(3)
    O = np.zeros((n_steps, OBS_DIM), np.float32)
    A = np.zeros((n_steps, 3), np.float32)
    R = np.zeros(n_steps, np.float32)

    for t in range(n_steps):
        for o in objs: o.step(t, s['speed'], rng)
        tracks = perc.sense(objs, t, s['speed'])
        lat_bias += lat_bias_v * DT + rng.normal(0, 0.002)
        m = {'speed': max(0.0, s['speed'] + rng.normal(0, 0.15)),
             'lat': s['lat'] + lat_bias + rng.normal(0, 0.03),
             'yaw_rate': s['yaw_rate'] + rng.normal(0, 0.02),
             'ax': s['ax'] + rng.normal(0, 0.1), 'ay': s['ay'] + rng.normal(0, 0.1),
             'beta': s.get('beta', 0.0),
             'kappa': road(s['s_pos']) + rng.normal(0, 0.0015),
             'lanes_f': lanes / 4.0, 'lanes_o': lanes / 4.0}
        a = expert.act(m, tracks, min(v_des, safe_curve_speed(m['kappa'])))
        O[t], A[t], R[t], prev_a = build_obs(m, tracks, prev_a), a, reward_step(s, objs, v_des, a), a

        a_apply = a.copy()
        if cfg.get('ctrl_noise', 0) and rng.random() < cfg['ctrl_noise']:
            a_apply[0] = np.clip(a_apply[0] + rng.normal(0, 0.35), -1, 1)
        if rng.random() < cfg.get('perturb', 0.008):
            k = rng.integers(0, 3)
            if k == 0:   s['lat'] = float(np.clip(s['lat'] + rng.choice([-1,1]) * rng.uniform(0.5, 2.2), -LAT_LIM, LAT_LIM))
            elif k == 1: s['yaw_rate'] += rng.choice([-1, 1]) * rng.uniform(0.2, 0.9)
            else:        s['speed'] = float(np.clip(s['speed'] + rng.uniform(-4, 3), 0, 30))
        step_ego(s, a_apply[0], a_apply[1], a_apply[2], road(s['s_pos']), dyn, rng)
        s['s_pos'] += s['speed'] * DT

    V = np.zeros(n_steps, np.float32)
    v = 0.0
    for t in range(n_steps - 1, -1, -1):
        v = R[t] + GAMMA * v
        V[t] = v
    return O, A, V

# ─── Scenario mix ───
SCENARIOS = [
    ('cruise', 160, lambda r: dict(road=r.choice(['straight','gentle']), v0=(6,18), v_des=(12,18))),
    ('curve', 220, lambda r: dict(road='curve', v0=(8,16), v_des=(12,16))),
    ('tight_curve', 140, lambda r: dict(road='tight', v0=(10,16), v_des=(10,14), perturb=0.015)),
    ('start_from_stop', 120, lambda r: dict(road=r.choice(['straight','gentle']), v0=(0,0), v_des=(12,16), perturb=0.004)),
    ('high_speed', 100, lambda r: dict(road=r.choice(['straight','gentle']), v0=(16,22), v_des=(18,24))),
    ('lead_follow', 220, lambda r: dict(road=r.choice(['straight','gentle','curve']),
        objects=lambda r: [TrafficObj(r, r.uniform(15,60), r.uniform(7,17), 0.0, r.random() < 0.8)])),
    ('cut_in', 240, lambda r: dict(road=r.choice(['straight','gentle']), v_des=(13,17),
        objects=lambda r: [TrafficObj(r, r.uniform(60,110), r.uniform(10,16), 0.0),
                           TrafficObj(r, r.uniform(12,35), r.uniform(9,15),
                                      r.choice([-1,1]) * LANE_W, r.random() < 0.5,
                                      cut_at=int(r.integers(20, 90)))])),
    ('emergency_brake_v2v', 240, lambda r: dict(road='straight', v0=(12,18), perturb=0.002,
        objects=lambda r: [TrafficObj(r, r.uniform(14,32), r.uniform(12,18), 0.0, True,
                                      brake_at=int(r.integers(25,70)), brake_rate=r.uniform(0.25,0.55))])),
    ('emergency_brake_nov2v', 180, lambda r: dict(road='straight', perception='nov2v', v0=(12,18), perturb=0.002,
        objects=lambda r: [TrafficObj(r, r.uniform(14,30), r.uniform(12,18), 0.0, False,
                                      brake_at=int(r.integers(25,60)), brake_rate=r.uniform(0.25,0.5))])),
    ('stopped_vehicle', 150, lambda r: dict(road=r.choice(['straight','gentle']), v0=(10,18), perturb=0.002,
        objects=lambda r: [TrafficObj(r, r.uniform(25,70), 0.0, 0.0, r.random() < 0.5)])),
    ('stop_and_go', 180, lambda r: dict(road='straight', v0=(4,12), v_des=(10,14), perturb=0.003,
        objects=lambda r: [TrafficObj(r, r.uniform(10,30), 6.0, 0.0, True,
                                      wave=(r.uniform(2,6), r.uniform(3,6), r.uniform(10,25)))])),
    ('multi_traffic', 240, lambda r: dict(road=r.choice(['straight','gentle','curve']), v0=(8,16),
        objects=lambda r: [TrafficObj(r, r.uniform(15,100), r.uniform(6,17),
                                      r.choice([0.0, 0.0, LANE_W, -LANE_W]) + r.normal(0, 0.4),
                                      r.random() < 0.7,
                                      brake_at=int(r.integers(40,140)) if r.random() < 0.4 else None)
                           for _ in range(int(r.integers(2, 5)))])),
    ('occluded_v2v', 160, lambda r: dict(road='straight', v0=(10,16), perturb=0.002,
        objects=lambda r: [TrafficObj(r, r.uniform(40,80), r.uniform(0,4), 0.0, True,
                                      reveal_dist=r.uniform(18,28))])),
    ('degraded_perception', 160, lambda r: dict(road=r.choice(['straight','gentle']),
        perception='degraded', v0=(8,16), ctrl_noise=0.03,
        objects=lambda r: [TrafficObj(r, r.uniform(20,60), r.uniform(5,14), 0.0, r.random() < 0.3,
                                      brake_at=int(r.integers(50,150)) if r.random() < 0.5 else None)])),
    ('recovery_mix', 220, lambda r: dict(road=r.choice(['straight','gentle','curve','tight']),
        v0=(0,20), v_des=(10,18), perturb=0.05, ctrl_noise=0.05,
        objects=lambda r: ([TrafficObj(r, r.uniform(15,80), r.uniform(4,16), 0.0, r.random() < 0.7)]
                           if r.random() < 0.6 else []))),
]

def generate_dataset(seed=42, traj_len=200):
    rng = np.random.default_rng(seed)
    O, A, V, EP = [], [], [], []
    for name, n, cfg_fn in SCENARIOS:
        for _ in range(n):
            o, a, v = run_episode(rng, cfg_fn(rng), traj_len)
            O.append(o); A.append(a); V.append(v); EP.append(np.full(traj_len, len(EP)))
        print(f"  {name:24s} {n * traj_len:>7,} steps")
    return np.concatenate(O), np.concatenate(A), np.concatenate(V), np.concatenate(EP)

if __name__ == '__main__':
    print("Generating sample dataset with expert_data.py...")
    O, A, V, EP = generate_dataset(seed=42, traj_len=50)
    print(f"Dataset generated successfully! Shape: O={O.shape}, A={A.shape}, V={V.shape}")
