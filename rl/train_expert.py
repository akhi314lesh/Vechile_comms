"""
rl/train_expert.py — Generate high-quality pre-trained weights via behavioral cloning.

Improvements over naive random-sample approach:
  1. Generates coherent driving TRAJECTORIES (not i.i.d. samples)
  2. Covers 8 distinct driving scenarios with proper weighting
  3. Uses cosine-annealing learning rate schedule
  4. Trains for 25 epochs with validation tracking
  5. Also trains the critic head on value targets
  6. Adds Gaussian noise augmentation for robustness
"""

import os, math
from pathlib import Path
import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np

from ppo import PPOTrainer, ActorCritic


# ─── Expert controller (mirrors the ADAS policy in index.html) ───

def expert_steering(lat, yaw_rate, kappa, speed):
    """Stanley-style lane-keeping + curvature feedforward."""
    # Crosstrack error correction (proportional + derivative)
    k_lat = 2.0 / max(1.0, speed * 0.15)
    steer = -k_lat * lat
    # Yaw rate damping
    steer -= 0.45 * yaw_rate
    # Curvature feedforward
    steer += kappa * max(3.0, speed) * 0.65
    return np.clip(steer, -1.0, 1.0)


def expert_longitudinal(speed, v_des, lead_dist, lead_rel_speed, is_braking_ahead):
    """IDM-style adaptive cruise + emergency brake."""
    throttle, brake = 0.0, 0.0

    # Emergency zone
    if lead_dist < 6.0:
        return 0.0, 1.0
    
    # TTC-based braking
    ttc = lead_dist / max(0.01, -lead_rel_speed) if lead_rel_speed < -0.5 else 99.0
    if ttc < 1.5:
        return 0.0, np.clip(1.5 / max(0.1, ttc), 0.6, 1.0)
    
    # Braking vehicle ahead warning (V2V)
    if is_braking_ahead and lead_dist < 40.0:
        v_des = min(v_des, speed * 0.6)

    # Following distance control (IDM-inspired)
    if lead_dist < 999:
        s_star = 4.0 + speed * 1.5 + speed * (-lead_rel_speed) / (2 * math.sqrt(2.0 * 3.0))
        s_star = max(s_star, 4.0)
        if lead_dist < s_star:
            v_des = min(v_des, speed * (lead_dist / s_star) ** 2)

    speed_err = v_des - speed
    if speed_err > 0.3:
        throttle = np.clip(speed_err / 5.0, 0.08, 0.85)
    elif speed_err < -1.5:
        brake = np.clip(-speed_err / 8.0, 0.08, 0.7)
    else:
        throttle = 0.12  # cruise maintenance

    return float(throttle), float(brake)


def safe_curve_speed(kappa):
    """Max safe speed through a curve (lateral accel limited)."""
    if abs(kappa) < 0.002:
        return 30.0
    return min(30.0, math.sqrt(0.8 * 9.81 / abs(kappa)))


# ─── Scenario generators (produce coherent trajectories) ───

def make_trajectory(n_steps, scenario_fn):
    """Roll out a trajectory using an expert controller in a given scenario."""
    obs_seq = np.zeros((n_steps, 97), dtype=np.float32)
    act_seq = np.zeros((n_steps, 3), dtype=np.float32)
    val_seq = np.zeros(n_steps, dtype=np.float32)

    state = scenario_fn('init')
    for t in range(n_steps):
        state = scenario_fn('step', state, t)
        obs_seq[t] = state['obs']
        act_seq[t] = state['action']
        val_seq[t] = state['value']
    return obs_seq, act_seq, val_seq


def scenario_straight_cruise():
    """Straight road, no traffic, cruise at 14 m/s."""
    def fn(cmd, state=None, t=0):
        if cmd == 'init':
            return {'speed': 0.5, 'lat': np.random.uniform(-0.3, 0.3), 'yaw_rate': 0.0,
                    'kappa': 0.0, 'ax': 0.0, 'ay': 0.0}
        s = state.copy()
        v_des = 14.0
        steer = expert_steering(s['lat'], s['yaw_rate'], s['kappa'], s['speed'])
        throttle, brake = expert_longitudinal(s['speed'], v_des, 999, 0, False)
        # Simulate dynamics
        s['speed'] = np.clip(s['speed'] + (throttle * 4.0 - brake * 6.0 - 0.02 * s['speed']) * 0.033, 0, 25)
        s['lat'] += (s['yaw_rate'] * 0.033 * s['speed'] * 0.3 + steer * 0.02)
        s['lat'] = np.clip(s['lat'] + np.random.normal(0, 0.01), -3.5, 3.5)
        s['yaw_rate'] = steer * s['speed'] * 0.08 + np.random.normal(0, 0.02)
        s['ax'] = throttle * 3.0 - brake * 5.0
        s['ay'] = s['yaw_rate'] * s['speed'] * 0.3
        obs = build_obs(s, [])
        action = np.array([steer, throttle, brake], dtype=np.float32)
        value = s['speed'] / 14.0 * 0.4 - abs(s['lat']) * 0.1
        s['obs'] = obs; s['action'] = action; s['value'] = value
        return s
    return fn


def scenario_curve_negotiation():
    """Track with varying curvature, no traffic."""
    kappa_base = np.random.choice([-1, 1]) * np.random.uniform(0.015, 0.055)
    def fn(cmd, state=None, t=0):
        if cmd == 'init':
            return {'speed': np.random.uniform(8, 16), 'lat': np.random.uniform(-0.5, 0.5),
                    'yaw_rate': 0.0, 'kappa': kappa_base, 'ax': 0.0, 'ay': 0.0}
        s = state.copy()
        # Slowly vary curvature
        s['kappa'] = kappa_base * (1.0 + 0.3 * math.sin(t * 0.02))
        v_des = min(14.0, safe_curve_speed(s['kappa']))
        steer = expert_steering(s['lat'], s['yaw_rate'], s['kappa'], s['speed'])
        throttle, brake = expert_longitudinal(s['speed'], v_des, 999, 0, False)
        s['speed'] = np.clip(s['speed'] + (throttle * 3.5 - brake * 6.0 - 0.015 * s['speed']) * 0.033, 0, 25)
        s['lat'] += (s['yaw_rate'] * 0.033 * s['speed'] * 0.3 + steer * 0.015 - s['kappa'] * s['speed'] * 0.01)
        s['lat'] = np.clip(s['lat'] + np.random.normal(0, 0.015), -3.5, 3.5)
        s['yaw_rate'] = steer * s['speed'] * 0.07 + s['kappa'] * s['speed'] * 0.5 + np.random.normal(0, 0.03)
        s['ax'] = throttle * 3.0 - brake * 5.0; s['ay'] = s['yaw_rate'] * s['speed'] * 0.4
        obs = build_obs(s, [])
        action = np.array([steer, throttle, brake], dtype=np.float32)
        value = s['speed'] / 14.0 * 0.35 - abs(s['lat']) * 0.12 - abs(steer) * 0.05
        s['obs'] = obs; s['action'] = action; s['value'] = value
        return s
    return fn


def scenario_lead_vehicle():
    """Following a lead vehicle at constant speed."""
    lead_speed = np.random.uniform(8, 16)
    lead_dist_init = np.random.uniform(15, 50)
    def fn(cmd, state=None, t=0):
        if cmd == 'init':
            return {'speed': np.random.uniform(10, 18), 'lat': np.random.uniform(-0.3, 0.3),
                    'yaw_rate': 0.0, 'kappa': np.random.uniform(-0.01, 0.01),
                    'ax': 0.0, 'ay': 0.0, 'lead_dist': lead_dist_init}
        s = state.copy()
        rel_speed = lead_speed - s['speed']
        s['lead_dist'] = np.clip(s['lead_dist'] + rel_speed * 0.033, 2, 150)
        v_des = min(14.0, safe_curve_speed(s['kappa']))
        steer = expert_steering(s['lat'], s['yaw_rate'], s['kappa'], s['speed'])
        throttle, brake = expert_longitudinal(s['speed'], v_des, s['lead_dist'], rel_speed, False)
        s['speed'] = np.clip(s['speed'] + (throttle * 3.5 - brake * 6.0 - 0.015 * s['speed']) * 0.033, 0, 25)
        s['lat'] += steer * 0.015 + np.random.normal(0, 0.008)
        s['lat'] = np.clip(s['lat'], -3.5, 3.5)
        s['yaw_rate'] = steer * s['speed'] * 0.07 + np.random.normal(0, 0.02)
        s['ax'] = throttle * 3.0 - brake * 5.0; s['ay'] = s['yaw_rate'] * s['speed'] * 0.3
        lead_obj = {'dist': s['lead_dist'], 'rel_speed': rel_speed, 'braking': False, 'v2v': True}
        obs = build_obs(s, [lead_obj])
        action = np.array([steer, throttle, brake], dtype=np.float32)
        value = s['speed'] / 14.0 * 0.3 - abs(s['lat']) * 0.1 + min(s['lead_dist'], 30) / 30 * 0.2
        s['obs'] = obs; s['action'] = action; s['value'] = value
        return s
    return fn


def scenario_emergency_brake():
    """Lead vehicle suddenly brakes hard."""
    lead_speed_init = np.random.uniform(12, 18)
    brake_start = np.random.randint(30, 80)
    def fn(cmd, state=None, t=0):
        if cmd == 'init':
            return {'speed': lead_speed_init - 1, 'lat': np.random.uniform(-0.2, 0.2),
                    'yaw_rate': 0.0, 'kappa': np.random.uniform(-0.008, 0.008),
                    'ax': 0.0, 'ay': 0.0, 'lead_dist': np.random.uniform(20, 35),
                    'lead_speed': lead_speed_init}
        s = state.copy()
        if t > brake_start:
            s['lead_speed'] = max(0, s['lead_speed'] - 0.25)
        rel_speed = s['lead_speed'] - s['speed']
        s['lead_dist'] = np.clip(s['lead_dist'] + rel_speed * 0.033, 1, 150)
        is_braking = t > brake_start
        steer = expert_steering(s['lat'], s['yaw_rate'], s['kappa'], s['speed'])
        throttle, brake_cmd = expert_longitudinal(s['speed'], 14.0, s['lead_dist'], rel_speed, is_braking)
        s['speed'] = np.clip(s['speed'] + (throttle * 3.5 - brake_cmd * 7.0 - 0.015 * s['speed']) * 0.033, 0, 25)
        s['lat'] += steer * 0.012 + np.random.normal(0, 0.006)
        s['lat'] = np.clip(s['lat'], -3.5, 3.5)
        s['yaw_rate'] = steer * s['speed'] * 0.07 + np.random.normal(0, 0.02)
        s['ax'] = throttle * 3.0 - brake_cmd * 6.0; s['ay'] = s['yaw_rate'] * s['speed'] * 0.3
        lead_obj = {'dist': s['lead_dist'], 'rel_speed': rel_speed, 'braking': is_braking, 'v2v': True}
        obs = build_obs(s, [lead_obj])
        action = np.array([steer, throttle, brake_cmd], dtype=np.float32)
        value = 0.3 if s['lead_dist'] > 5 else -1.0
        s['obs'] = obs; s['action'] = action; s['value'] = value
        return s
    return fn


def scenario_v2v_occluded():
    """V2V-warned occluded vehicle behind barrier."""
    occ_dist_init = np.random.uniform(40, 80)
    occ_speed = np.random.uniform(0, 5)
    def fn(cmd, state=None, t=0):
        if cmd == 'init':
            return {'speed': np.random.uniform(10, 16), 'lat': np.random.uniform(-0.3, 0.3),
                    'yaw_rate': 0.0, 'kappa': np.random.uniform(-0.02, 0.02),
                    'ax': 0.0, 'ay': 0.0, 'occ_dist': occ_dist_init}
        s = state.copy()
        rel_speed = occ_speed - s['speed']
        s['occ_dist'] = np.clip(s['occ_dist'] + rel_speed * 0.033, 1, 200)
        steer = expert_steering(s['lat'], s['yaw_rate'], s['kappa'], s['speed'])
        throttle, brake = expert_longitudinal(s['speed'], 13.0, s['occ_dist'], rel_speed, occ_speed < 2)
        s['speed'] = np.clip(s['speed'] + (throttle * 3.5 - brake * 6.0 - 0.015 * s['speed']) * 0.033, 0, 25)
        s['lat'] += steer * 0.012 + np.random.normal(0, 0.008)
        s['lat'] = np.clip(s['lat'], -3.5, 3.5)
        s['yaw_rate'] = steer * s['speed'] * 0.07 + np.random.normal(0, 0.02)
        s['ax'] = throttle * 3.0 - brake * 5.0; s['ay'] = s['yaw_rate'] * s['speed'] * 0.3
        occ_obj = {'dist': s['occ_dist'], 'rel_speed': rel_speed, 'braking': occ_speed < 2, 'v2v': True}
        obs = build_obs(s, [occ_obj])
        action = np.array([steer, throttle, brake], dtype=np.float32)
        value = 0.3 if s['occ_dist'] > 8 else -0.5
        s['obs'] = obs; s['action'] = action; s['value'] = value
        return s
    return fn


def scenario_multi_vehicle():
    """2-3 vehicles in mixed traffic."""
    n_cars = np.random.randint(2, 4)
    car_speeds = [np.random.uniform(8, 18) for _ in range(n_cars)]
    car_dists = sorted([np.random.uniform(15, 80) for _ in range(n_cars)])
    def fn(cmd, state=None, t=0):
        if cmd == 'init':
            return {'speed': np.random.uniform(8, 16), 'lat': np.random.uniform(-0.5, 0.5),
                    'yaw_rate': 0.0, 'kappa': np.random.uniform(-0.02, 0.02),
                    'ax': 0.0, 'ay': 0.0, 'car_dists': list(car_dists)}
        s = state.copy()
        objects = []
        min_dist = 999
        min_rel = 0
        for i in range(n_cars):
            rel = car_speeds[i] - s['speed']
            s['car_dists'][i] = np.clip(s['car_dists'][i] + rel * 0.033, 2, 200)
            objects.append({'dist': s['car_dists'][i], 'rel_speed': rel,
                          'braking': car_speeds[i] < 5, 'v2v': np.random.rand() < 0.7})
            if s['car_dists'][i] < min_dist:
                min_dist = s['car_dists'][i]; min_rel = rel
        steer = expert_steering(s['lat'], s['yaw_rate'], s['kappa'], s['speed'])
        throttle, brake = expert_longitudinal(s['speed'], 14.0, min_dist, min_rel, False)
        s['speed'] = np.clip(s['speed'] + (throttle * 3.5 - brake * 6.0 - 0.015 * s['speed']) * 0.033, 0, 25)
        s['lat'] += steer * 0.012 + np.random.normal(0, 0.01)
        s['lat'] = np.clip(s['lat'], -3.5, 3.5)
        s['yaw_rate'] = steer * s['speed'] * 0.07 + np.random.normal(0, 0.025)
        s['ax'] = throttle * 3.0 - brake * 5.0; s['ay'] = s['yaw_rate'] * s['speed'] * 0.35
        obs = build_obs(s, objects)
        action = np.array([steer, throttle, brake], dtype=np.float32)
        value = s['speed'] / 14.0 * 0.3 + min(min_dist, 30) / 30 * 0.2 - abs(s['lat']) * 0.1
        s['obs'] = obs; s['action'] = action; s['value'] = value
        return s
    return fn


def scenario_start_from_stop():
    """Starting from standstill, accelerating to cruise speed."""
    def fn(cmd, state=None, t=0):
        if cmd == 'init':
            return {'speed': 0.0, 'lat': np.random.uniform(-0.2, 0.2), 'yaw_rate': 0.0,
                    'kappa': np.random.uniform(-0.01, 0.01), 'ax': 0.0, 'ay': 0.0}
        s = state.copy()
        v_des = 14.0
        steer = expert_steering(s['lat'], s['yaw_rate'], s['kappa'], max(s['speed'], 1.0))
        speed_err = v_des - s['speed']
        throttle = np.clip(speed_err / 4.0, 0.1, 0.9) if speed_err > 0 else 0.0
        brake = 0.0
        s['speed'] = np.clip(s['speed'] + (throttle * 4.5 - 0.01 * s['speed']) * 0.033, 0, 25)
        s['lat'] += steer * 0.01 + np.random.normal(0, 0.005)
        s['lat'] = np.clip(s['lat'], -3.5, 3.5)
        s['yaw_rate'] = steer * max(s['speed'], 0.5) * 0.06 + np.random.normal(0, 0.01)
        s['ax'] = throttle * 4.0; s['ay'] = 0.0
        obs = build_obs(s, [])
        action = np.array([steer, throttle, brake], dtype=np.float32)
        value = s['speed'] / 14.0 * 0.5
        s['obs'] = obs; s['action'] = action; s['value'] = value
        return s
    return fn


def scenario_tight_curve_with_traffic():
    """Sharp curve with a slow lead vehicle."""
    kappa = np.random.choice([-1, 1]) * np.random.uniform(0.03, 0.06)
    lead_speed = np.random.uniform(6, 11)
    def fn(cmd, state=None, t=0):
        if cmd == 'init':
            return {'speed': np.random.uniform(10, 16), 'lat': np.random.uniform(-0.4, 0.4),
                    'yaw_rate': 0.0, 'kappa': kappa, 'ax': 0.0, 'ay': 0.0,
                    'lead_dist': np.random.uniform(18, 40)}
        s = state.copy()
        s['kappa'] = kappa * (1.0 + 0.2 * math.sin(t * 0.03))
        rel_speed = lead_speed - s['speed']
        s['lead_dist'] = np.clip(s['lead_dist'] + rel_speed * 0.033, 2, 150)
        v_des = min(safe_curve_speed(s['kappa']), 14.0)
        steer = expert_steering(s['lat'], s['yaw_rate'], s['kappa'], s['speed'])
        throttle, brake = expert_longitudinal(s['speed'], v_des, s['lead_dist'], rel_speed, False)
        s['speed'] = np.clip(s['speed'] + (throttle * 3.5 - brake * 6.0 - 0.015 * s['speed']) * 0.033, 0, 25)
        s['lat'] += (s['yaw_rate'] * 0.033 * s['speed'] * 0.2 + steer * 0.012 - s['kappa'] * s['speed'] * 0.008)
        s['lat'] = np.clip(s['lat'] + np.random.normal(0, 0.012), -3.5, 3.5)
        s['yaw_rate'] = steer * s['speed'] * 0.07 + s['kappa'] * s['speed'] * 0.5 + np.random.normal(0, 0.03)
        s['ax'] = throttle * 3.0 - brake * 5.0; s['ay'] = s['yaw_rate'] * s['speed'] * 0.45
        lead_obj = {'dist': s['lead_dist'], 'rel_speed': rel_speed, 'braking': False, 'v2v': True}
        obs = build_obs(s, [lead_obj])
        action = np.array([steer, throttle, brake], dtype=np.float32)
        value = s['speed'] / 14.0 * 0.25 + min(s['lead_dist'], 20) / 20 * 0.2 - abs(s['lat']) * 0.15
        s['obs'] = obs; s['action'] = action; s['value'] = value
        return s
    return fn


# ─── Observation builder (matches index.html _buildObs exactly) ───

def build_obs(state, objects):
    """Build 97-dim observation matching the browser's format."""
    obs = np.zeros(97, dtype=np.float32)
    speed = state['speed']

    # Ego state (0..8)
    obs[0] = np.clip(abs(speed) / 50.0, 0, 1)
    obs[1] = np.clip(state['ax'] / 12.0, -1, 1)
    obs[2] = np.clip(state['ay'] / 12.0, -1, 1)
    obs[3] = np.clip(state['yaw_rate'] / 2.6, -1, 1)
    obs[4] = 0.0  # slip angle
    obs[5] = np.clip(state['lat'] / 5.0, -1, 1)
    obs[6] = 0.0  # prev steer (will be overwritten)
    obs[7] = 0.3  # prev throttle
    obs[8] = 0.0  # prev brake

    # Road (9..12)
    obs[9]  = np.clip(state['kappa'] / 0.1, -1, 1)
    obs[10] = 0.5   # lanes_f / 4
    obs[11] = 0.5   # lanes_o / 4
    obs[12] = 0.0   # signal

    # Objects (13..96) — 12 slots × 7 features
    sorted_obj = sorted(objects, key=lambda o: o['dist'])[:12]
    for i in range(12):
        base = 13 + i * 7
        if i < len(sorted_obj):
            o = sorted_obj[i]
            d = o['dist']
            obs[base + 0] = np.clip(np.random.uniform(-2, 2) / 250.0, -1, 1)   # relX (small lateral offset)
            obs[base + 1] = np.clip(d / 250.0, -1, 1)                           # relZ (ahead)
            obs[base + 2] = np.clip(o['rel_speed'] / 60.0, -1, 1)               # relative speed
            obs[base + 3] = np.clip(d / 250.0, 0, 1)                            # distance
            obs[base + 4] = 1.0 if o['braking'] else 0.0                        # braking flag
            obs[base + 5] = 1.0 if o.get('v2v') else 0.0                        # source (v2v+local)
            obs[base + 6] = np.clip(np.random.uniform(0, 0.3) / 2.0, 0, 1)      # message age
        else:
            obs[base + 3] = 1.0  # empty slot = max distance
    return obs


# ─── Main training loop ───

def main():
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    print(f"[Expert Model v2] Using device: {device}")

    trainer = PPOTrainer(obs_dim=97, act_dim=3, device=device)
    model = trainer.model

    # Generate trajectory dataset
    scenario_factories = [
        (scenario_straight_cruise, 250),
        (scenario_curve_negotiation, 350),
        (scenario_lead_vehicle, 300),
        (scenario_emergency_brake, 400),
        (scenario_v2v_occluded, 300),
        (scenario_multi_vehicle, 350),
        (scenario_start_from_stop, 200),
        (scenario_tight_curve_with_traffic, 350),
    ]

    traj_len = 200
    all_obs, all_act, all_val = [], [], []

    print("[Expert Model v2] Generating expert driving trajectories...")
    np.random.seed(42)
    for factory_fn, count in scenario_factories:
        for _ in range(count):
            fn = factory_fn()
            o, a, v = make_trajectory(traj_len, fn)
            all_obs.append(o)
            all_act.append(a)
            all_val.append(v)

    obs_data = torch.FloatTensor(np.concatenate(all_obs))
    act_data = torch.FloatTensor(np.concatenate(all_act))
    val_data = torch.FloatTensor(np.concatenate(all_val))

    total_samples = len(obs_data)
    print(f"[Expert Model v2] Generated {total_samples:,} samples from {sum(c for _, c in scenario_factories)} trajectories")

    # Split 95/5 train/val
    perm = torch.randperm(total_samples)
    n_val = total_samples // 20
    val_idx, train_idx = perm[:n_val], perm[n_val:]
    train_ds = torch.utils.data.TensorDataset(obs_data[train_idx], act_data[train_idx], val_data[train_idx])
    val_ds = torch.utils.data.TensorDataset(obs_data[val_idx], act_data[val_idx], val_data[val_idx])
    train_loader = torch.utils.data.DataLoader(train_ds, batch_size=256, shuffle=True, drop_last=True)
    val_loader = torch.utils.data.DataLoader(val_ds, batch_size=512, shuffle=False)

    optimizer = torch.optim.AdamW(model.parameters(), lr=3e-3, weight_decay=1e-4)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=25, eta_min=1e-5)

    print("[Expert Model v2] Training ActorCritic (25 epochs, cosine LR)...")
    best_val_loss = float('inf')

    for epoch in range(1, 26):
        model.train()
        epoch_actor_loss, epoch_critic_loss, n = 0.0, 0.0, 0
        for b_obs, b_act, b_val in train_loader:
            b_obs = b_obs.to(device)
            b_act = b_act.to(device)
            b_val = b_val.to(device)

            # Noise augmentation on obs (robustness)
            b_obs_aug = b_obs + torch.randn_like(b_obs) * 0.01

            features = model.shared(b_obs_aug)
            pred_act = model.actor_mean(features)
            pred_val = model.critic(features).squeeze(-1)

            actor_loss = F.mse_loss(pred_act, b_act)
            critic_loss = F.mse_loss(pred_val, b_val)
            loss = actor_loss + 0.5 * critic_loss

            optimizer.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
            optimizer.step()

            epoch_actor_loss += actor_loss.item() * len(b_obs)
            epoch_critic_loss += critic_loss.item() * len(b_obs)
            n += len(b_obs)

        scheduler.step()

        # Validation
        model.eval()
        val_loss, vn = 0.0, 0
        with torch.no_grad():
            for b_obs, b_act, b_val in val_loader:
                features = model.shared(b_obs.to(device))
                pred = model.actor_mean(features)
                val_loss += F.mse_loss(pred, b_act.to(device)).item() * len(b_obs)
                vn += len(b_obs)

        avg_train = epoch_actor_loss / n
        avg_val = val_loss / vn
        lr = scheduler.get_last_lr()[0]
        tag = ""
        if avg_val < best_val_loss:
            best_val_loss = avg_val
            tag = " ★ best"

        print(f"  Epoch {epoch:2d}/25 — Actor: {avg_train:.5f} | Critic: {epoch_critic_loss/n:.5f} | "
              f"Val: {avg_val:.5f} | LR: {lr:.2e}{tag}")

    # Set exploration noise for evaluation (low = deterministic)
    with torch.no_grad():
        model.actor_log_std.data.fill_(-2.0)

    # Save
    log_dir = Path('rl/logs')
    log_dir.mkdir(parents=True, exist_ok=True)

    for name in ['best_model.pt', 'pretrained_model.pt', 'final_model.pt']:
        trainer.save(str(log_dir / name))

    print(f"\n{'='*60}")
    print(f"  Expert model v2 training complete!")
    print(f"  Total samples: {total_samples:,}")
    print(f"  Best val MSE:  {best_val_loss:.5f}")
    print(f"  Saved to:      rl/logs/best_model.pt")
    print(f"{'='*60}")


if __name__ == '__main__':
    main()
