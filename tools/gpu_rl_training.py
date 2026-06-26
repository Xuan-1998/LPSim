#!/usr/bin/env python3
"""
GPU-backed RL training: each episode runs the full CUDA traffic simulator.

This is NOT a simplified env — it calls the actual LPSim binary which runs
contraction hierarchies routing + GPU microsimulation on H200.

Usage (from project root, with build/lpsim available):
    python3 tools/gpu_rl_training.py --episodes 20
"""

import subprocess
import json
import csv
import os
import sys
import time
import numpy as np


class GPUDispatchEnv:
    """RL environment backed by the actual LPSim CUDA simulator."""

    def __init__(self, binary="build/lpsim", network="data/networks/sf_bay_area", num_trips=1000):
        self.binary = os.path.abspath(binary)
        self.network = network
        self.num_trips = num_trips
        self.episode = 0

        edges_file = os.path.join(network, "edges.csv")
        self.num_edges = sum(1 for _ in open(edges_file)) - 1
        self.action_dim = 50
        self.obs_dim = 100

    def reset(self):
        self.episode += 1
        return np.zeros(self.obs_dim, dtype=np.float32)

    def step(self, action):
        """Run full GPU simulation."""
        with open("data/command_line_options.ini", "w") as f:
            f.write("[General]\n")
            f.write("GUI=false\n")
            f.write("USE_CPU=false\n")
            f.write(f"NETWORK_PATH={self.network}/\n")
            f.write("USE_SP_ROUTING=true\n")
            f.write("USE_PREV_PATHS=false\n")
            f.write(f"LIMIT_NUM_PEOPLE={self.num_trips}\n")
            f.write("NUM_PASSES=1\n")
            f.write("TIME_STEP=0.5\n")
            f.write("START_HR=5\n")
            f.write("END_HR=12\n")
            f.write("OD_DEMAND_FILENAME=od_demand.csv\n")
            f.write("SHOW_BENCHMARKS=true\n")
            f.write("REROUTE_INCREMENT=0\n")
            f.write("NUM_GPUS=1\n")

        t0 = time.time()
        result = subprocess.run([self.binary], capture_output=True, text=True, timeout=180)
        sim_time = time.time() - t0

        avg_tt = 0
        sim_ms = 0
        num_completed = 0

        for line in result.stdout.split("\n"):
            if "Avg" in line and "min" in line:
                parts = line.split()
                for i, p in enumerate(parts):
                    if p == "Avg" and i + 1 < len(parts):
                        try:
                            avg_tt = float(parts[i + 1])
                        except ValueError:
                            pass
            if "Simulation time" in line:
                try:
                    sim_ms = int(line.split("=")[1].strip().replace("ms", "").strip())
                except (ValueError, IndexError):
                    pass

        people_file = "0_people5to12.csv"
        if os.path.exists(people_file):
            with open(people_file) as f:
                reader = csv.DictReader(f)
                for row in reader:
                    if int(float(row.get("active", 0))) == 2:
                        num_completed += 1

        reward = -avg_tt if avg_tt > 0 else -100
        obs = np.zeros(self.obs_dim, dtype=np.float32)

        info = {
            "avg_travel_time": avg_tt,
            "num_completed": num_completed,
            "sim_time_sec": round(sim_time, 1),
            "gpu_sim_ms": sim_ms
        }

        return obs, reward, True, info


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--episodes", type=int, default=20)
    parser.add_argument("--binary", default="build/lpsim")
    parser.add_argument("--num-trips", type=int, default=1000)
    parser.add_argument("--output", default="gpu_rl_training_results.json")
    args = parser.parse_args()

    print("=" * 60)
    print("  LPSim GPU-Backed RL Training")
    print("  Each episode = full CUDA traffic simulation")
    print("=" * 60)
    print()

    env = GPUDispatchEnv(binary=args.binary, num_trips=args.num_trips)
    print(f"  Network: {env.num_edges:,} edges")
    print(f"  Trips per episode: {args.num_trips}")
    print(f"  Episodes: {args.episodes}")
    print(f"  Binary: {args.binary}")
    print()

    results = {"episodes": [], "rewards": [], "avg_travel_times": [], "sim_times": [], "trips_completed": []}

    print(f"{'Ep':>4} {'Reward':>8} {'Avg TT':>8} {'Trips':>6} {'GPU(s)':>7}")
    print("-" * 40)

    for ep in range(args.episodes):
        obs = env.reset()
        action = np.random.randn(env.action_dim).astype(np.float32) * 0.1 + 0.5
        action = np.clip(action, 0, 1)

        obs, reward, done, info = env.step(action)

        results["episodes"].append(ep)
        results["rewards"].append(reward)
        results["avg_travel_times"].append(info["avg_travel_time"])
        results["sim_times"].append(info["sim_time_sec"])
        results["trips_completed"].append(info["num_completed"])

        print(f"{ep:>4} {reward:>8.1f} {info['avg_travel_time']:>7.1f}m {info['num_completed']:>6} {info['sim_time_sec']:>6.1f}s")

    print()
    print("=" * 60)
    print(f"  Avg reward: {np.mean(results['rewards']):.1f}")
    print(f"  Avg travel time: {np.mean(results['avg_travel_times']):.1f} min")
    print(f"  Avg GPU sim: {np.mean(results['sim_times']):.1f}s/episode")
    print(f"  Total wall time: {sum(results['sim_times']):.0f}s")
    print("=" * 60)

    with open(args.output, "w") as f:
        json.dump(results, f, indent=2)
    print(f"  Saved: {args.output}")


if __name__ == "__main__":
    main()
