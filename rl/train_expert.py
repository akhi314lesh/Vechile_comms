"""
rl/train_expert.py — Generate high-quality pre-trained weights via behavioral cloning.
Fixed steering conventions and longitudinal control matching simulator physics.
"""

import os, math, sys
from pathlib import Path
import numpy as np

# Force unbuffered output
sys.stdout.reconfigure(line_buffering=True)

import torch
import torch.nn as nn
import torch.nn.functional as F

from ppo import PPOTrainer, ActorCritic


# ─── Expert controllers ───

def expert_steer_vec(lat, yaw_rate, kappa, speed):
    """
    Stanley steering matching Three.js coordinate system in index.html:
      +lat = car is to the right of lane center -> steer > 0 (turn left back to center)
      -lat = car is to the left of lane center  -> steer < 0 (turn right back to center)
      +yaw_rate = car rotating left -> damping steer < 0 (turn right to stabilize)
      +kappa = road curves left -> feedforward steer > 0 (turn left along curve)
    """
    k_lat = 0.42 / np.maximum(1.0, speed * 0.08)
    steer = k_lat * lat - 0.28 * yaw_rate + kappa * np.maximum(3.5, speed) * 0.60
    return np.clip(steer, -1.0, 1.0)


def expert_long_vec(speed, v_des, lead_dist, lead_rel_speed, is_braking):
    """Vectorized IDM-style longitudinal control."""
    n = len(speed)
    throttle = np.zeros(n, dtype=np.float32)
    brake = np.zeros(n, dtype=np.float32)

    # Emergency zone
    emerg = lead_dist < 6.0
    brake[emerg] = 1.0

    # TTC braking
    closing = (lead_rel_speed < -0.5) & ~emerg
    ttc = np.full(n, 99.0)
    ttc[closing] = lead_dist[closing] / np.maximum(0.01, -lead_rel_speed[closing])
    ttc_brake = (ttc < 1.8) & ~emerg
    brake[ttc_brake] = np.clip(1.8 / np.maximum(0.1, ttc[ttc_brake]), 0.6, 1.0)

    # V2V braking warning
    v2v_warn = is_braking & (lead_dist < 45.0) & ~emerg & ~ttc_brake
    v_des_adj = v_des.copy()
    v_des_adj[v2v_warn] = np.minimum(v_des[v2v_warn], speed[v2v_warn] * 0.5)

    # IDM following
    has_lead = (lead_dist < 999) & ~emerg & ~ttc_brake
    s_star = 5.0 + speed * 1.6 + speed * np.maximum(0, -lead_rel_speed) / (2 * math.sqrt(6.0))
    s_star = np.maximum(s_star, 5.0)
    too_close = has_lead & (lead_dist < s_star)
    v_des_adj[too_close] = np.minimum(v_des_adj[too_close],
                                       speed[too_close] * (lead_dist[too_close] / s_star[too_close]) ** 2)

    # Speed control (only for non-emergency/ttc cases)
    normal = ~emerg & ~ttc_brake
    speed_err = v_des_adj - speed
    accel = normal & (speed_err > 0.2)
    decel = normal & (speed_err < -1.0)
    cruise = normal & ~accel & ~decel

    throttle[accel] = np.clip(speed_err[accel] * 0.35, 0.15, 0.9)
    brake[decel] = np.clip(-speed_err[decel] * 0.25, 0.1, 0.7)
    throttle[cruise] = 0.22  # maintain cruising momentum

    # Zero throttle where braking
    throttle[brake > 0.05] = 0.0

    return throttle, brake


def safe_curve_speed_vec(kappa):
    """Vectorized max safe speed through curve."""
    straight = np.abs(kappa) < 0.002
    v_max = np.where(straight, 30.0, np.minimum(30.0, np.sqrt(0.8 * 9.81 / np.maximum(np.abs(kappa), 1e-6))))
    return v_max


# ─── Vectorized dataset generation ───

def generate_dataset(n_total=500000):
    """Generate expert driving data covering 8 scenarios, fully vectorized."""
    print(f"[Expert v2] Generating {n_total:,} expert samples (vectorized)...", flush=True)
    np.random.seed(42)

    obs_all = np.zeros((n_total, 97), dtype=np.float32)
    act_all = np.zeros((n_total, 3), dtype=np.float32)
    val_all = np.zeros(n_total, dtype=np.float32)

    idx = 0
    scenarios = [
        ('straight_cruise', int(n_total * 0.10)),
        ('curve', int(n_total * 0.15)),
        ('lead_follow', int(n_total * 0.15)),
        ('emergency_brake', int(n_total * 0.15)),
        ('v2v_occluded', int(n_total * 0.12)),
        ('multi_vehicle', int(n_total * 0.13)),
        ('start_stop', int(n_total * 0.08)),
        ('tight_curve_traffic', int(n_total * 0.12)),
    ]

    for name, n in scenarios:
        end = min(idx + n, n_total)
        actual_n = end - idx
        if actual_n <= 0:
            break

        speed = np.random.uniform(2, 22, actual_n).astype(np.float32)
        lat = np.random.uniform(-2.5, 2.5, actual_n).astype(np.float32)
        yaw_rate = np.random.uniform(-1.0, 1.0, actual_n).astype(np.float32)
        kappa = np.random.uniform(-0.05, 0.05, actual_n).astype(np.float32)
        lead_dist = np.full(actual_n, 999.0, dtype=np.float32)
        lead_rel = np.zeros(actual_n, dtype=np.float32)
        is_braking = np.zeros(actual_n, dtype=bool)

        if name == 'straight_cruise':
            speed[:] = np.random.uniform(0.5, 20, actual_n)
            kappa[:] = np.random.uniform(-0.005, 0.005, actual_n)
            lat[:] = np.random.uniform(-1.0, 1.0, actual_n)

        elif name == 'curve':
            kappa[:] = np.random.choice([-1, 1], actual_n) * np.random.uniform(0.01, 0.06, actual_n)
            speed[:] = np.random.uniform(5, 18, actual_n)
            lat[:] = np.random.uniform(-1.5, 1.5, actual_n)

        elif name == 'lead_follow':
            lead_dist[:] = np.random.uniform(8, 60, actual_n)
            lead_rel[:] = np.random.uniform(-8, 5, actual_n)
            speed[:] = np.random.uniform(6, 18, actual_n)
            kappa[:] = np.random.uniform(-0.015, 0.015, actual_n)

        elif name == 'emergency_brake':
            lead_dist[:] = np.random.uniform(4, 30, actual_n)
            lead_rel[:] = np.random.uniform(-15, -2, actual_n)  # closing fast
            is_braking[:] = np.random.rand(actual_n) > 0.3
            speed[:] = np.random.uniform(8, 20, actual_n)

        elif name == 'v2v_occluded':
            lead_dist[:] = np.random.uniform(10, 80, actual_n)
            lead_rel[:] = np.random.uniform(-12, 3, actual_n)
            is_braking[:] = np.random.rand(actual_n) > 0.4
            speed[:] = np.random.uniform(8, 18, actual_n)

        elif name == 'multi_vehicle':
            lead_dist[:] = np.random.uniform(10, 50, actual_n)
            lead_rel[:] = np.random.uniform(-10, 5, actual_n)
            speed[:] = np.random.uniform(6, 18, actual_n)
            kappa[:] = np.random.uniform(-0.03, 0.03, actual_n)

        elif name == 'start_stop':
            speed[:] = np.random.uniform(0, 6, actual_n)
            lat[:] = np.random.uniform(-0.5, 0.5, actual_n)
            kappa[:] = np.random.uniform(-0.01, 0.01, actual_n)

        elif name == 'tight_curve_traffic':
            kappa[:] = np.random.choice([-1, 1], actual_n) * np.random.uniform(0.025, 0.06, actual_n)
            lead_dist[:] = np.random.uniform(12, 45, actual_n)
            lead_rel[:] = np.random.uniform(-6, 3, actual_n)
            speed[:] = np.random.uniform(6, 16, actual_n)
            lat[:] = np.random.uniform(-1.5, 1.5, actual_n)

        # Compute expert actions
        v_safe = safe_curve_speed_vec(kappa)
        v_des = np.minimum(14.0, v_safe)
        steer = expert_steer_vec(lat, yaw_rate, kappa, speed)
        throttle, brake = expert_long_vec(speed, v_des, lead_dist, lead_rel, is_braking)

        # Derived quantities
        ax = throttle * 3.5 - brake * 6.0 - 0.015 * speed
        ay = yaw_rate * speed * 0.35

        # Build observations matching index.html _buildObs() exactly
        obs = np.zeros((actual_n, 97), dtype=np.float32)
        obs[:, 0] = np.clip(np.abs(speed) / 50.0, 0, 1)
        obs[:, 1] = np.clip(ax / 12.0, -1, 1)
        obs[:, 2] = np.clip(ay / 12.0, -1, 1)
        obs[:, 3] = np.clip(yaw_rate / 2.6, -1, 1)
        obs[:, 4] = 0.0  # slip
        obs[:, 5] = np.clip(lat / 5.0, -1, 1)
        obs[:, 6] = np.clip(steer * 0.9, -1, 1)   # prev steer
        obs[:, 7] = np.clip(throttle * 0.85, 0, 1) # prev throttle
        obs[:, 8] = np.clip(brake * 0.85, 0, 1)    # prev brake
        obs[:, 9] = np.clip(kappa / 0.1, -1, 1)
        obs[:, 10] = 0.5  # lanes_f
        obs[:, 11] = 0.5  # lanes_o
        obs[:, 12] = 0.0  # signal

        # Object slots
        has_obj = lead_dist < 900
        for slot in range(3):
            base = 13 + slot * 7
            if slot == 0:
                mask = has_obj
                d = lead_dist
                rv = lead_rel
                brk = is_braking
            else:
                mask = np.random.rand(actual_n) < 0.3
                d = np.random.uniform(20, 120, actual_n).astype(np.float32)
                rv = np.random.uniform(-8, 5, actual_n).astype(np.float32)
                brk = np.random.rand(actual_n) > 0.7

            obs[mask, base + 0] = np.clip(np.random.uniform(-2, 2, mask.sum()) / 250.0, -1, 1)
            obs[mask, base + 1] = np.clip(d[mask] / 250.0, -1, 1)
            obs[mask, base + 2] = np.clip(rv[mask] / 60.0, -1, 1)
            obs[mask, base + 3] = np.clip(d[mask] / 250.0, 0, 1)
            obs[mask, base + 4] = brk[mask].astype(np.float32)
            obs[mask, base + 5] = (np.random.rand(mask.sum()) < 0.7).astype(np.float32)
            obs[mask, base + 6] = np.clip(np.random.uniform(0, 0.3, mask.sum()) / 2.0, 0, 1)
            obs[~mask, base + 3] = 1.0  # empty slot

        # Remaining empty slots
        for slot in range(3, 12):
            obs[:, 13 + slot * 7 + 3] = 1.0

        # Actions
        act = np.stack([steer, throttle, brake], axis=1).astype(np.float32)

        # Value targets
        val = (speed / 14.0 * 0.35
               - np.abs(lat) * 0.12
               + np.where(lead_dist < 900, np.minimum(lead_dist, 30) / 30 * 0.2, 0.1)
               - np.abs(steer) * 0.05).astype(np.float32)

        obs_all[idx:end] = obs
        act_all[idx:end] = act
        val_all[idx:end] = val
        idx = end

        print(f"  {name:25s} — {actual_n:>7,} samples", flush=True)

    if idx < n_total:
        obs_all = obs_all[:idx]
        act_all = act_all[:idx]
        val_all = val_all[:idx]

    return obs_all, act_all, val_all


def main():
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    print(f"[Expert Model v2] Using device: {device}", flush=True)

    trainer = PPOTrainer(obs_dim=97, act_dim=3, device=device)
    model = trainer.model

    obs_np, act_np, val_np = generate_dataset(500000)
    total = len(obs_np)

    obs_data = torch.FloatTensor(obs_np)
    act_data = torch.FloatTensor(act_np)
    val_data = torch.FloatTensor(val_np)

    perm = torch.randperm(total)
    n_val = total // 20
    train_idx, val_idx = perm[n_val:], perm[:n_val]
    train_ds = torch.utils.data.TensorDataset(obs_data[train_idx], act_data[train_idx], val_data[train_idx])
    val_ds = torch.utils.data.TensorDataset(obs_data[val_idx], act_data[val_idx], val_data[val_idx])
    train_loader = torch.utils.data.DataLoader(train_ds, batch_size=512, shuffle=True, drop_last=True,
                                                num_workers=0, pin_memory=(device == 'cuda'))
    val_loader = torch.utils.data.DataLoader(val_ds, batch_size=1024, shuffle=False, num_workers=0)

    optimizer = torch.optim.AdamW(model.parameters(), lr=3e-3, weight_decay=1e-4)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=25, eta_min=1e-5)

    print(f"[Expert Model v2] Training on {total:,} samples (25 epochs, cosine LR)...", flush=True)
    best_val_loss = float('inf')

    for epoch in range(1, 26):
        model.train()
        ep_actor, ep_critic, n = 0.0, 0.0, 0
        for b_obs, b_act, b_val in train_loader:
            b_obs = b_obs.to(device)
            b_act = b_act.to(device)
            b_val = b_val.to(device)

            b_obs_aug = b_obs + torch.randn_like(b_obs) * 0.012

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

            ep_actor += actor_loss.item() * len(b_obs)
            ep_critic += critic_loss.item() * len(b_obs)
            n += len(b_obs)

        scheduler.step()

        model.eval()
        vl, vn = 0.0, 0
        with torch.no_grad():
            for b_obs, b_act, b_val in val_loader:
                features = model.shared(b_obs.to(device))
                pred = model.actor_mean(features)
                vl += F.mse_loss(pred, b_act.to(device)).item() * len(b_obs)
                vn += len(b_obs)

        avg_train = ep_actor / n
        avg_val = vl / vn
        tag = ""
        if avg_val < best_val_loss:
            best_val_loss = avg_val
            tag = " ★ best"

        print(f"  Epoch {epoch:2d}/25 — Actor: {avg_train:.5f} | Critic: {ep_critic/n:.5f} | "
              f"Val: {avg_val:.5f} | LR: {scheduler.get_last_lr()[0]:.2e}{tag}", flush=True)

    with torch.no_grad():
        model.actor_log_std.data.fill_(-2.0)

    log_dir = Path('rl/logs')
    log_dir.mkdir(parents=True, exist_ok=True)
    for name in ['best_model.pt', 'pretrained_model.pt', 'final_model.pt']:
        trainer.save(str(log_dir / name))

    print(f"\n{'='*60}", flush=True)
    print(f"  Expert Model v2 — Training Complete!", flush=True)
    print(f"  Total samples:  {total:,}", flush=True)
    print(f"  Best val MSE:   {best_val_loss:.5f}", flush=True)
    print(f"  Models saved:   rl/logs/best_model.pt", flush=True)
    print(f"{'='*60}", flush=True)


if __name__ == '__main__':
    main()
