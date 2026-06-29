"""
LPSim Gymnasium Environment

Wraps the C++/CUDA simulator as a Gymnasium-compatible
environment for reinforcement learning research on city-scale fleets.

The environment exposes:
  - Observation: per-edge traffic state (speed, density, flow)
  - Action: toll/pricing vector or routing directives per edge
  - Reward: configurable (total travel time, revenue, congestion)

Architecture:
  Python (Gym API) → subprocess → LivingCity binary → GPU simulation
  State exchange via CSV files (fast enough for epoch-level RL).
"""

import csv
import os
import subprocess
import tempfile
from typing import Any, Optional

import gymnasium as gym
import numpy as np
from gymnasium import spaces


class LPSimEnv(gym.Env):
    """Large-scale network simulation environment for RL.

    Supports dispatch, pricing, and routing policy optimization
    over networks with hundreds of thousands of edges.
    """

    metadata = {"render_modes": ["human"], "render_fps": 1}

    def __init__(
        self,
        network_path: str = "LivingCity/data/networks/sf_bay_area",
        simulator_dir: str = "LivingCity",
        num_gpus: int = 1,
        start_hour: float = 5.0,
        end_hour: float = 6.0,
        time_step: float = 0.5,
        num_trips: int = 5000,
        action_edges: Optional[list] = None,
        reward_type: str = "travel_time",
        render_mode: Optional[str] = None,
    ):
        super().__init__()

        self.network_path = network_path
        self.simulator_dir = simulator_dir
        self.num_gpus = num_gpus
        self.start_hour = start_hour
        self.end_hour = end_hour
        self.time_step = time_step
        self.num_trips = num_trips
        self.reward_type = reward_type
        self.render_mode = render_mode

        # Load network topology to determine observation/action dimensions
        edges_file = os.path.join(network_path, "edges.csv")
        with open(edges_file) as f:
            self.num_edges = sum(1 for _ in f) - 1  # subtract header

        # Action space: toll/speed multiplier per controllable edge
        self.action_edges = action_edges or list(range(min(100, self.num_edges)))
        self.num_actions = len(self.action_edges)

        self.action_space = spaces.Box(
            low=0.0, high=1.0, shape=(self.num_actions,), dtype=np.float32
        )

        # Observation space: per-edge [avg_speed, num_vehicles, flow_rate]
        self.observation_space = spaces.Box(
            low=0.0, high=np.inf, shape=(self.num_edges, 3), dtype=np.float32
        )

        self._episode_count = 0
        self._last_results = None

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self._episode_count += 1
        obs = np.zeros(self.observation_space.shape, dtype=np.float32)
        return obs, {}

    def step(self, action: np.ndarray):
        # Write action as toll vector to edges CSV (modify speed/toll column)
        self._apply_action(action)

        # Run simulation
        results = self._run_simulation()
        self._last_results = results

        # Compute observation from results
        obs = self._extract_observation(results)

        # Compute reward
        reward = self._compute_reward(results)

        terminated = True  # single-shot episode (one sim run)
        truncated = False

        info = {
            "avg_travel_time": results.get("avg_travel_time", 0.0),
            "total_co": results.get("total_co", 0.0),
            "num_completed": results.get("num_completed", 0),
        }

        return obs, reward, terminated, truncated, info

    def _apply_action(self, action: np.ndarray):
        """Apply RL action as toll/pricing modifications to the network."""
        # For now, store action for the simulator to read
        action_file = os.path.join(self.simulator_dir, "rl_action.csv")
        with open(action_file, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["edge_id", "toll"])
            for i, edge_id in enumerate(self.action_edges):
                writer.writerow([edge_id, f"{action[i]:.4f}"])

    def _run_simulation(self) -> dict:
        """Execute the GPU simulator and parse results."""
        config = {
            "GUI": "false",
            "USE_CPU": "false",
            "NETWORK_PATH": self.network_path + "/",
            "USE_SP_ROUTING": "true",
            "USE_PREV_PATHS": "false",
            "NUM_PASSES": "1",
            "TIME_STEP": str(self.time_step),
            "START_HR": str(int(self.start_hour)),
            "END_HR": str(int(self.end_hour)),
            "OD_DEMAND_FILENAME": "od_demand.csv",
            "SHOW_BENCHMARKS": "false",
            "REROUTE_INCREMENT": "0",
            "NUM_GPUS": str(self.num_gpus),
            "LIMIT_NUM_PEOPLE": str(self.num_trips),
        }

        ini_path = os.path.join(self.simulator_dir, "command_line_options.ini")
        with open(ini_path, "w") as f:
            f.write("[General]\n")
            for k, v in config.items():
                f.write(f"{k}={v}\n")

        binary = os.path.join(self.simulator_dir, "LivingCity")
        result = subprocess.run(
            [binary],
            cwd=self.simulator_dir,
            capture_output=True,
            text=True,
            timeout=300,
        )

        return self._parse_output(result.stdout)

    def _parse_output(self, stdout: str) -> dict:
        """Parse simulator stdout for key metrics."""
        results = {"avg_travel_time": 0.0, "total_co": 0.0, "num_completed": 0}
        for line in stdout.split("\n"):
            if "Avg" in line and "min" in line:
                parts = line.split()
                for i, p in enumerate(parts):
                    if p == "Avg" and i + 1 < len(parts):
                        try:
                            results["avg_travel_time"] = float(parts[i + 1])
                        except ValueError:
                            pass
                    if p == "CO" and i + 1 < len(parts):
                        try:
                            results["total_co"] = float(parts[i + 1])
                        except ValueError:
                            pass
            if "People" in line:
                parts = line.split()
                for i, p in enumerate(parts):
                    if p == "People":
                        try:
                            results["num_completed"] = int(parts[i + 1])
                        except (ValueError, IndexError):
                            pass
        return results

    def _extract_observation(self, results: dict) -> np.ndarray:
        """Build observation from simulation output files."""
        obs = np.zeros(self.observation_space.shape, dtype=np.float32)
        results_file = os.path.join(self.simulator_dir, "results.csv")
        if os.path.exists(results_file):
            with open(results_file) as f:
                reader = csv.DictReader(f)
                for row in reader:
                    edge_id = int(row.get("edge_id", -1))
                    if 0 <= edge_id < self.num_edges:
                        obs[edge_id, 0] = float(row.get("avg_speed", 0))
                        obs[edge_id, 1] = float(row.get("num_vehicles", 0))
                        obs[edge_id, 2] = float(row.get("flow_rate", 0))
        return obs

    def _compute_reward(self, results: dict) -> float:
        """Compute reward based on configured objective."""
        if self.reward_type == "travel_time":
            return -results.get("avg_travel_time", 100.0)
        elif self.reward_type == "emissions":
            return -results.get("total_co", 1000.0)
        elif self.reward_type == "throughput":
            return float(results.get("num_completed", 0))
        return 0.0

    def render(self):
        if self.render_mode == "human" and self._last_results:
            print(f"[LPSim] Episode {self._episode_count}: "
                  f"avg_tt={self._last_results.get('avg_travel_time', '?')} min, "
                  f"CO={self._last_results.get('total_co', '?')}")

    def close(self):
        pass
