"""
rl/ppo.py — Proximal Policy Optimization (PPO) for continuous control

Actor-Critic architecture:
  Input: 97-dim observation
  Actor: MLP → 3 continuous actions (steering, throttle, brake)
  Critic: MLP → 1 scalar value

Uses:
  - Clipped surrogate objective
  - GAE advantage estimation
  - Entropy bonus for exploration
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.distributions import Normal
import numpy as np


class ActorCritic(nn.Module):
    """Actor-Critic network for PPO."""

    def __init__(self, obs_dim=97, act_dim=3, hidden=256):
        super().__init__()

        # Shared feature extractor
        self.shared = nn.Sequential(
            nn.Linear(obs_dim, hidden),
            nn.Tanh(),
            nn.Linear(hidden, hidden),
            nn.Tanh(),
        )

        # Actor head (mean + log_std for each action)
        self.actor_mean = nn.Linear(hidden, act_dim)
        self.actor_log_std = nn.Parameter(torch.zeros(act_dim))

        # Critic head
        self.critic = nn.Linear(hidden, 1)

        # Action bounds
        # steering: [-1, 1], throttle: [0, 1], brake: [0, 1]
        self.register_buffer('act_low', torch.tensor([-1.0, 0.0, 0.0]))
        self.register_buffer('act_high', torch.tensor([1.0, 1.0, 1.0]))

        self._init_weights()

    def _init_weights(self):
        for m in self.modules():
            if isinstance(m, nn.Linear):
                nn.init.orthogonal_(m.weight, gain=0.5)
                nn.init.zeros_(m.bias)
        # Smaller init for actor output
        nn.init.orthogonal_(self.actor_mean.weight, gain=0.01)

    def forward(self, obs):
        features = self.shared(obs)
        return features

    def get_action(self, obs, deterministic=False):
        """Sample an action from the policy."""
        features = self.forward(obs)
        mean = self.actor_mean(features)
        std = self.actor_log_std.exp().expand_as(mean)

        if deterministic:
            action = mean
        else:
            dist = Normal(mean, std)
            action = dist.rsample()

        # Clip to valid range
        action = torch.max(torch.min(action, self.act_high), self.act_low)
        return action

    def evaluate(self, obs, actions):
        """Evaluate actions: log_prob, entropy, value."""
        features = self.forward(obs)
        mean = self.actor_mean(features)
        std = self.actor_log_std.exp().expand_as(mean)

        dist = Normal(mean, std)
        log_prob = dist.log_prob(actions).sum(dim=-1)
        entropy = dist.entropy().sum(dim=-1)
        value = self.critic(features).squeeze(-1)

        return log_prob, entropy, value

    def get_value(self, obs):
        """Get value estimate."""
        features = self.forward(obs)
        return self.critic(features).squeeze(-1)


class PPOTrainer:
    """PPO training algorithm."""

    def __init__(self, obs_dim=97, act_dim=3, device='cpu', **kwargs):
        self.device = torch.device(device)
        self.model = ActorCritic(obs_dim, act_dim).to(self.device)
        self.optimizer = torch.optim.Adam(self.model.parameters(), lr=kwargs.get('lr', 3e-4))

        # Hyperparameters
        self.gamma = kwargs.get('gamma', 0.99)
        self.gae_lambda = kwargs.get('gae_lambda', 0.95)
        self.clip_eps = kwargs.get('clip_eps', 0.2)
        self.entropy_coef = kwargs.get('entropy_coef', 0.01)
        self.value_coef = kwargs.get('value_coef', 0.5)
        self.max_grad_norm = kwargs.get('max_grad_norm', 0.5)
        self.epochs = kwargs.get('epochs', 10)
        self.batch_size = kwargs.get('batch_size', 64)

        # Rollout buffer
        self.buffer = RolloutBuffer()

    def select_action(self, obs, deterministic=False):
        """Select action for a single observation."""
        with torch.no_grad():
            obs_t = torch.FloatTensor(obs).unsqueeze(0).to(self.device)
            action = self.model.get_action(obs_t, deterministic)
            value = self.model.get_value(obs_t)
            _, _, _ = self.model.evaluate(obs_t, action)
        return action.squeeze(0).cpu().numpy(), value.item()

    def store_transition(self, obs, action, reward, done, value, log_prob=None):
        """Store a transition in the buffer."""
        self.buffer.store(obs, action, reward, done, value)

    def compute_gae(self, last_value):
        """Compute GAE advantages."""
        rewards = np.array(self.buffer.rewards)
        values = np.array(self.buffer.values + [last_value])
        dones = np.array(self.buffer.dones)

        advantages = np.zeros_like(rewards)
        last_gae = 0

        for t in reversed(range(len(rewards))):
            delta = rewards[t] + self.gamma * values[t + 1] * (1 - dones[t]) - values[t]
            advantages[t] = last_gae = delta + self.gamma * self.gae_lambda * (1 - dones[t]) * last_gae

        returns = advantages + values[:-1]
        return advantages, returns

    def update(self, last_value):
        """Perform PPO update."""
        advantages, returns = self.compute_gae(last_value)

        # Normalize advantages
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)

        obs = torch.FloatTensor(np.array(self.buffer.observations)).to(self.device)
        actions = torch.FloatTensor(np.array(self.buffer.actions)).to(self.device)
        old_values = torch.FloatTensor(np.array(self.buffer.values)).to(self.device)
        advantages_t = torch.FloatTensor(advantages).to(self.device)
        returns_t = torch.FloatTensor(returns).to(self.device)

        # Get old log probs
        with torch.no_grad():
            old_log_probs, _, _ = self.model.evaluate(obs, actions)

        # PPO epochs
        total_loss = 0
        n_updates = 0

        for epoch in range(self.epochs):
            indices = np.random.permutation(len(obs))

            for start in range(0, len(obs), self.batch_size):
                end = start + self.batch_size
                idx = indices[start:end]

                b_obs = obs[idx]
                b_actions = actions[idx]
                b_advantages = advantages_t[idx]
                b_returns = returns_t[idx]
                b_old_log_probs = old_log_probs[idx]

                log_probs, entropy, values = self.model.evaluate(b_obs, b_actions)

                # Policy loss (clipped surrogate)
                ratio = (log_probs - b_old_log_probs).exp()
                surr1 = ratio * b_advantages
                surr2 = torch.clamp(ratio, 1 - self.clip_eps, 1 + self.clip_eps) * b_advantages
                policy_loss = -torch.min(surr1, surr2).mean()

                # Value loss
                value_loss = F.mse_loss(values, b_returns)

                # Entropy bonus
                entropy_loss = -entropy.mean()

                # Total loss
                loss = policy_loss + self.value_coef * value_loss + self.entropy_coef * entropy_loss

                self.optimizer.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(self.model.parameters(), self.max_grad_norm)
                self.optimizer.step()

                total_loss += loss.item()
                n_updates += 1

        self.buffer.clear()
        return total_loss / max(n_updates, 1)

    def save(self, path):
        """Save model checkpoint."""
        torch.save({
            'model_state': self.model.state_dict(),
            'optimizer_state': self.optimizer.state_dict(),
        }, path)
        print(f'[PPO] Model saved to {path}')

    def load(self, path):
        """Load model checkpoint."""
        checkpoint = torch.load(path, map_location=self.device)
        self.model.load_state_dict(checkpoint['model_state'])
        self.optimizer.load_state_dict(checkpoint['optimizer_state'])
        print(f'[PPO] Model loaded from {path}')


class RolloutBuffer:
    """Simple rollout buffer for PPO."""

    def __init__(self):
        self.clear()

    def clear(self):
        self.observations = []
        self.actions = []
        self.rewards = []
        self.dones = []
        self.values = []

    def store(self, obs, action, reward, done, value):
        self.observations.append(obs)
        self.actions.append(action)
        self.rewards.append(reward)
        self.dones.append(float(done))
        self.values.append(value)

    def __len__(self):
        return len(self.observations)
