"""rl/lka_test.py — run: python rl/lka_test.py"""
import math
import sys
from pathlib import Path

# Add current dir and parent dir to sys.path
sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).parent.parent))

try:
    from lka import LaneKeepController, wrap_angle
except ImportError:
    from rl.lka import LaneKeepController, wrap_angle

DT = 1.0 / 30.0

def _smooth(u): u = max(0.0, min(1.0, u)); return u * u * (3.0 - 2.0 * u)

class Road:
    """Piecewise curvature, [(length_m, kappa), ...], smoothstep transitions."""
    def __init__(self, segs, trans=20.0):
        self.edges = [0.0]
        for L, _ in segs: self.edges.append(self.edges[-1] + L)
        self.k = [kk for _, kk in segs]
        self.T = trans
    def kappa(self, s):
        s = min(max(s, 0.0), self.edges[-1] - 1e-9)
        for j in range(len(self.k) - 1):
            e = self.edges[j + 1]
            if e - self.T / 2 <= s <= e + self.T / 2:
                u = (s - (e - self.T / 2)) / self.T
                return self.k[j] + (self.k[j + 1] - self.k[j]) * _smooth(u)
        i = 0
        while i < len(self.k) - 1 and s >= self.edges[i + 1]: i += 1
        return self.k[i]

class LaneSim:
    """Road-frame bicycle + actuator lag. +steer/+omega/+kappa = LEFT, e_lat + = RIGHT."""
    def __init__(self, e_lat=0.0, e_psi=0.0, L=3.2, delta_max=0.5, tau=0.10):
        self.e_lat, self.e_psi = e_lat, e_psi
        self.L, self.delta_max, self.tau = L, delta_max, tau
        self.act, self.s, self.omega = 0.0, 0.0, 0.0
    def step(self, steer_cmd, v, kappa):
        self.act += (steer_cmd - self.act) * min(1.0, DT / self.tau)
        self.omega = v * math.tan(self.act * self.delta_max) / self.L
        self.e_psi += (self.omega - v * kappa) * DT
        self.e_lat += -v * math.sin(self.e_psi) * DT
        self.s += v * DT

def run(name, road, v, e0, psi0, T, k_damp=0.12):
    ctrl = LaneKeepController(dt=DT, k_damp=k_damp)   # recommended config w/ lag
    sim = LaneSim(e_lat=e0, e_psi=psi0)
    es, us = [], []
    for _ in range(int(T / DT)):
        k = road.kappa(sim.s)
        u = ctrl.update(sim.e_lat, sim.e_psi, k, v, sim.omega)
        sim.step(u, v, k)
        es.append(sim.e_lat); us.append(u)

    skip = int(3.0 / DT)
    max_after = max(abs(e) for e in es[skip:])
    tail = max(abs(e) for e in es[-int(2.0 / DT):])
    zc = sum(1 for a, b in zip(es[skip:], es[skip + 1:])
             if a * b < 0 and max(abs(a), abs(b)) > 0.03)          # significant crossings
    bad = [i for i, e in enumerate(es) if abs(e) > 0.05]
    settle = (bad[-1] + 1) * DT if bad else 0.0
    slew = max((abs(b - a) for a, b in zip([0.0] + us[:-1], us)), default=0.0) / DT
    assert slew <= 6.0 + 1e-6 and -1.0 <= min(us) and max(us) <= 1.0
    print(f"  {name:34s} v={v:4.1f}  max|e|={max_after:6.3f}  tail|e|={tail:6.3f}  "
          f"settle={settle:5.1f}s  zero-x={zc}  slew={slew:5.2f}/s")
    return {'max_after': max_after, 'tail': tail, 'settle': settle, 'zc': zc}

def unit_tests():
    c = LaneKeepController(dt=DT, k_damp=0.0)          # literal spec formula
    c.reset(); assert c.update(0.5, 0, 0, 20, 0) > 0   # (4) right of center → +
    c.reset(); assert c.update(-0.5, 0, 0, 20, 0) < 0  # (4) left of center  → −
    c.reset(); assert c.update(0, 0, 0.01, 20, 0) > 0  # (4) left curve      → +ff
    c.reset(); assert c.update(0, 0.05, 0, 20, 0) < 0  # heading left → correct right
    # exact spec values
    c.reset(); assert abs(c.update(0, 0, 0.01, 20, 0) - 0.064) < 1e-9   # κ(3.2+0.16·20)
    c.reset(); assert abs(c.update(0.5, 0, 0, 30, 0) - 0.10) < 1e-9     # (0.42/2.1)·0.5
    # (5) saturation + slew: |raw|>1 clamps, one step moves exactly 6·dt
    c.reset(); assert abs(c.update(10, 0, 0, 20, 0) - 6.0 * DT) < 1e-9
    # heading wrap: 6.1 rad ≡ −0.183 rad
    c.reset(); a = c.update(0.3, 6.1, 0, 20, 0)
    c.reset(); b = c.update(0.3, wrap_angle(6.1), 0, 20, 0)
    assert abs(a - b) < 1e-9
    # damping never fights steady cornering (ω = v·κ exactly)
    d = LaneKeepController(dt=DT, k_damp=0.12); d.reset()
    assert abs(d.update(0, 0, 0.01, 20, 0.2) - 0.064) < 1e-9
    # 'right' mode is an exact mirror of 'left'
    Lc = LaneKeepController(dt=DT); Rc = LaneKeepController(dt=DT, steer_positive='right')
    Lc.reset(); Rc.reset()
    ul = Lc.update(0.4, 0.06, 0.008, 24, 0.15)
    ur = Rc.update(0.4, -0.06, -0.008, 24, -0.15)      # sim-space (+ = right)
    assert abs(ul + ur) < 1e-9
    # NaN → hold
    c.reset(); u0 = c.update(0.3, 0, 0, 20, 0)
    assert c.update(float('nan'), 0, 0, 20, 0) == u0
    print("unit tests: all passed ✔\n")

def main():
    print("Closed-loop lane centering (30 Hz, actuator tau = 100 ms, k_damp = 0.12)\n")
    r = run("straight centering +0.8 m", Road([(600, 0.0)]), 25.0, 0.8, 0.04, 12.0)
    assert r['tail'] < 0.05 and r['settle'] < 6.0 and r['zc'] <= 6      # no oscillation
    r = run("mirror start      −0.8 m", Road([(600, 0.0)]), 25.0, -0.8, -0.04, 12.0)
    assert r['tail'] < 0.05 and r['settle'] < 6.0
    r = run("highway S  R250L → R150R", Road([(100, 0.0), (300, 1/250), (80, 0.0),
                                              (250, -1/150), (100, 0.0)]), 33.0, 0.0, 0.0, 26.0)
    assert r['max_after'] < 0.15                                        # no corner cutting
    r = run("sharp ramp curve  R80L", Road([(60, 0.0), (250, 1/80), (60, 0.0)]), 20.0, 0.3, 0.0, 17.0)
    assert r['max_after'] < 0.15
    r = run("low-speed recovery +1.2 m", Road([(400, 0.0)]), 6.0, 1.2, 0.0, 16.0)
    assert r['settle'] < 15.0 and r['tail'] < 0.05
    print("\nlka_test: all closed-loop assertions passed ✔")

if __name__ == '__main__':
    unit_tests()
    main()
