"""
rl/evaluate.py — Evaluation script for V2V vs non-V2V comparison

Runs trained models through standardized scenarios and generates
comparison metrics between V2V-enabled and V2V-disabled conditions.
"""

import argparse
import asyncio
import json
import os
from pathlib import Path

import numpy as np

from ppo import PPOTrainer
from ws_bridge import SimulatorBridge


async def evaluate(args):
    bridge = SimulatorBridge(host=args.host, port=args.port)
    await bridge.start_server()

    trainer = PPOTrainer(obs_dim=97, act_dim=3, device=args.device)
    trainer.load(args.model)

    results = {
        'v2v_normal': [], 'v2v_erratic': [],
        'nov2v_normal': [], 'nov2v_erratic': [],
        'v2v_occluded': [], 'nov2v_occluded': []
    }

    configs = [
        ('v2v_normal',    True,  4, 'Normal + V2V'),
        ('nov2v_normal',  False, 4, 'Normal + No V2V'),
        ('v2v_erratic',   True,  4, 'Erratic + V2V'),
        ('nov2v_erratic', False, 4, 'Erratic + No V2V'),
        ('v2v_occluded',  True,  6, 'Occluded + V2V'),
        ('nov2v_occluded', False, 5, 'Occluded + No V2V'),
    ]

    print(f'\n{"="*70}')
    print(f'  V2V Evaluation — {args.episodes} episodes per config')
    print(f'  Model: {args.model}')
    print(f'{"="*70}')

    for config_name, v2v_enabled, stage, label in configs:
        print(f'\n--- {label} (stage {stage}) ---')
        await bridge.set_v2v(v2v_enabled)
        await bridge.set_curriculum(stage)

        for ep in range(args.episodes):
            seed = ep * 12345 + hash(config_name) % 10000
            obs = await bridge.reset(seed)
            episode_reward = 0
            steps = 0
            done = False

            while not done:
                action, _ = trainer.select_action(obs, deterministic=True)
                obs, reward, terminated, truncated, info = await bridge.step(action)
                done = terminated or truncated
                episode_reward += reward
                steps += 1

            metrics = await bridge.get_metrics()
            results[config_name].append({
                'reward': episode_reward,
                'steps': steps,
                'collision': info.get('collision', False),
                'reason': info.get('reason', ''),
                **metrics
            })

            if (ep + 1) % 10 == 0:
                avg_r = np.mean([r['reward'] for r in results[config_name]])
                col = sum(1 for r in results[config_name] if r['collision']) / len(results[config_name])
                print(f'  EP {ep+1:4d} | AvgR={avg_r:7.1f} | Col={col:.1%}')

    # Print comparison table
    print(f'\n{"="*70}')
    print(f'  RESULTS COMPARISON')
    print(f'{"="*70}')
    print(f'{"Config":<25} {"AvgReward":>10} {"ColRate":>10} {"AvgMinTTC":>10} {"AvgMinDist":>10}')
    print(f'{"-"*65}')

    for config_name, _, _, label in configs:
        data = results[config_name]
        if not data:
            continue
        avg_reward = np.mean([d['reward'] for d in data])
        col_rate = sum(1 for d in data if d.get('collision')) / len(data)
        ttcs = [d.get('minTTC', 99) for d in data if d.get('minTTC') is not None]
        avg_ttc = np.mean(ttcs) if ttcs else 'N/A'
        dists = [d.get('minDistance', 99) for d in data if d.get('minDistance') is not None]
        avg_dist = np.mean(dists) if dists else 'N/A'

        ttc_str = f'{avg_ttc:.2f}' if isinstance(avg_ttc, float) else avg_ttc
        dist_str = f'{avg_dist:.2f}' if isinstance(avg_dist, float) else avg_dist
        print(f'{label:<25} {avg_reward:>10.1f} {col_rate:>10.1%} {ttc_str:>10} {dist_str:>10}')

    # Save results
    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, 'w') as f:
        json.dump(results, f, indent=2, default=str)
    print(f'\nResults saved to {out_path}')

    await bridge.close()


def main():
    parser = argparse.ArgumentParser(description='V2V RL Evaluation')
    parser.add_argument('--model', required=True, help='Path to trained model')
    parser.add_argument('--episodes', type=int, default=50, help='Episodes per config')
    parser.add_argument('--host', default='localhost')
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--device', default='cpu')
    parser.add_argument('--output', default='rl/logs/eval_results.json')
    args = parser.parse_args()

    asyncio.run(evaluate(args))


if __name__ == '__main__':
    main()
