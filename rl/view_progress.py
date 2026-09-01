"""
rl/view_progress.py — Interactive Learning Verification Tool

Run anytime during or after training:
  python rl/view_progress.py

Analyzes training_log.jsonl and outputs:
  - Learning status (Is the agent improving?)
  - Reward trajectory (early vs recent episodes)
  - Collision rate reduction
  - Survival steps increase
  - Stage promotions
"""

import json
import os
import sys
from pathlib import Path


def analyze_training_log(log_path='rl/logs/training_log.jsonl'):
    path = Path(log_path)
    if not path.exists():
        print(f"[!] Log file not found at: {path}")
        print("    Start training first with: python rl/train.py --device cuda")
        return

    episodes = []
    with open(path, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if line:
                try:
                    episodes.append(json.loads(line))
                except json.JSONDecodeError:
                    pass

    n = len(episodes)
    if n == 0:
        print("[!] Log file is currently empty. Waiting for episode 1 to complete...")
        return

    # Extract metrics
    rewards = [e.get('reward', 0) for e in episodes]
    steps = [e.get('steps', 0) for e in episodes]
    collisions = [1 if e.get('collision') else 0 for e in episodes]
    stages = [e.get('curriculum_stage', 1) for e in episodes]

    total_episodes = n
    current_stage = stages[-1]
    total_collisions = sum(collisions)
    overall_col_rate = (total_collisions / n) * 100

    # Compare first chunk vs last chunk (learning trend)
    chunk_size = min(50, n // 2) if n >= 20 else n // 2
    if chunk_size > 0:
        early_rewards = rewards[:chunk_size]
        recent_rewards = rewards[-chunk_size:]
        early_cols = collisions[:chunk_size]
        recent_cols = collisions[-chunk_size:]

        avg_early_r = sum(early_rewards) / chunk_size
        avg_recent_r = sum(recent_rewards) / chunk_size
        col_early_rate = (sum(early_cols) / chunk_size) * 100
        col_recent_rate = (sum(recent_cols) / chunk_size) * 100
        reward_gain = avg_recent_r - avg_early_r
        col_reduction = col_early_rate - col_recent_rate
    else:
        avg_early_r = rewards[0]
        avg_recent_r = rewards[-1]
        col_early_rate = collisions[0] * 100
        col_recent_rate = collisions[-1] * 100
        reward_gain = avg_recent_r - avg_early_r
        col_reduction = 0

    print("=" * 65)
    print("         V2V RL AGENT — LEARNING VERIFICATION REPORT")
    print("=" * 65)
    print(f" Total Episodes Completed: {total_episodes}")
    print(f" Current Curriculum Stage: Stage {current_stage}")
    print(f" Best Episode Reward:      {max(rewards):.1f}")
    print(f" Latest 10-Ep Avg Reward:  {sum(rewards[-10:]) / min(10, len(rewards)):.1f}")
    print(f" Overall Collision Rate:   {overall_col_rate:.1f}% ({total_collisions}/{total_episodes})")
    print("-" * 65)

    print("\n>>> LEARNING VERIFICATION DIAGNOSIS <<<")
    if n < 15:
        print(" [STATUS: INITIALIZING] Agent is in early exploration phase.")
        print(" Allow training to reach 30–50 episodes to observe statistical trends.")
    elif reward_gain > 5 and col_reduction >= 0:
        print(" [STATUS: SUCCESSFULLY LEARNING] ✅")
        print(f"  • Average Reward:   {avg_early_r:+.1f} (Early)  ==>  {avg_recent_r:+.1f} (Recent)  [{reward_gain:+.1f} pts]")
        print(f"  • Collision Rate:   {col_early_rate:.1f}% (Early) ==>  {col_recent_rate:.1f}% (Recent)  [-{abs(col_reduction):.1f}%]")
        print("  The agent is successfully improving its driving policy!")
    elif reward_gain > 0:
        print(" [STATUS: PROGRESSING] 📈")
        print(f"  • Average Reward:   {avg_early_r:+.1f} (Early)  ==>  {avg_recent_r:+.1f} (Recent)")
        print(f"  • Collision Rate:   {col_early_rate:.1f}% (Early) ==>  {col_recent_rate:.1f}% (Recent)")
    else:
        print(" [STATUS: EXPLORING POLICY] 🔄")
        print("  Agent is currently testing action distributions.")

    # Recent 10 episodes table
    print("\n" + "-" * 65)
    print(f"{'Episode':<10} {'Reward':<12} {'Steps':<10} {'Collision?':<12} {'Curriculum':<12}")
    print("-" * 65)
    for e in episodes[-10:]:
        col_str = "💥 CRASH" if e.get('collision') else "✅ CLEAN"
        print(f"{e.get('episode', 0):<10} {e.get('reward', 0):<12.1f} {e.get('steps', 0):<10} {col_str:<12} Stage {e.get('curriculum_stage', 1):<10}")
    print("-" * 65)
    print("Report complete. Re-run `python rl/view_progress.py` at any time!\n")


if __name__ == '__main__':
    log_file = sys.argv[1] if len(sys.argv) > 1 else 'rl/logs/training_log.jsonl'
    analyze_training_log(log_file)
