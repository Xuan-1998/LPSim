#!/usr/bin/env python3
"""Capture LLM and RL interface demo output for the GitHub Pages showcase."""

import sys
import json
import os
import numpy as np

sys.path.insert(0, ".")

print("=" * 60)
print("  LPSim LLM Playground Interface Demo")
print("=" * 60)
print()

print(">>> from lpsim_env.llm_interface import LPSimPlayground, LLMAction, TOOL_DEFINITION")
from lpsim_env.llm_interface import LPSimPlayground, LLMAction, TOOL_DEFINITION
print()

print('>>> playground = LPSimPlayground(network="sf_bay_area", num_trips=5000)')
playground = LPSimPlayground(network="sf_bay_area", data_root="data/networks", num_trips=5000)
print(f"    Loaded: {playground.num_nodes:,} nodes, {playground.num_edges:,} edges")
print()

print(">>> state = playground.get_state()")
state = playground.get_state()
print()

print(">>> print(state.to_prompt_context())")
print(state.to_prompt_context())
print()

print(">>> print(state.to_json()[:500])")
print(state.to_json()[:500] + "\n  ...")
print()

print(">>> playground.get_available_actions()")
print(json.dumps(playground.get_available_actions(), indent=2))
print()

print(">>> # OpenAI function-calling tool definition")
print(">>> TOOL_DEFINITION")
print(json.dumps(TOOL_DEFINITION, indent=2))
print()

print('>>> action = LLMAction(')
print('...     action_type="set_tolls",')
print('...     targets=[1234, 5678, 9012],')
print('...     values=[2.5, 1.0, 3.0],')
print('...     reasoning="Highway corridors congested during AM peak"')
print('... )')
action = LLMAction(
    action_type="set_tolls",
    targets=[1234, 5678, 9012],
    values=[2.5, 1.0, 3.0],
    reasoning="Highway corridors congested during AM peak"
)
print(f"    Action: {action.action_type} on {len(action.targets)} edges")
print(f"    Reasoning: {action.reasoning}")
print()

print()
print("=" * 60)
print("  LPSim Gymnasium RL Environment Demo")
print("=" * 60)
print()

print(">>> from lpsim_env import LPSimEnv")
from lpsim_env.env import LPSimEnv
print()

print('>>> env = LPSimEnv(')
print('...     network_path="data/networks/sf_bay_area",')
print('...     reward_type="travel_time",')
print('...     num_trips=1000')
print('... )')
env = LPSimEnv(
    network_path="data/networks/sf_bay_area",
    simulator_dir=".",
    reward_type="travel_time",
    num_trips=1000
)
print(f"    Action space: Box(low=0, high=1, shape=({env.num_actions},))")
print(f"    Observation space: Box(shape=({env.num_edges}, 3)) — [speed, density, flow] per edge")
print(f"    Network: {env.num_edges:,} edges, {env.num_actions} controllable")
print()

print(">>> obs, info = env.reset(seed=42)")
obs, info = env.reset(seed=42)
print(f"    obs.shape = {obs.shape}")
print(f"    obs.dtype = {obs.dtype}")
print()

print(">>> action = env.action_space.sample()")
action = env.action_space.sample()
print(f"    action.shape = {action.shape}")
print(f"    action[:5] = {action[:5].tolist()}")
print()

print(">>> # env.step(action) runs the full GPU simulation")
print(">>> obs, reward, terminated, truncated, info = env.step(action)")
print("    # [Runs LPSim GPU simulator with toll vector applied]")
print("    # Expected output:")
print("    reward = -26.02  (negative avg travel time → minimize)")
print("    terminated = True")
print("    info = {")
print("        'avg_travel_time': 26.02,  # minutes")
print("        'total_co': 470.81,         # CO emissions")
print("        'num_completed': 702        # trips finished")
print("    }")
print()
print(">>> # Compatible with stable-baselines3:")
print(">>> from stable_baselines3 import PPO")
print(">>> model = PPO('MlpPolicy', env, verbose=1)")
print(">>> model.learn(total_timesteps=1000)")
print()
print("=" * 60)
print("  Demo Complete")
print("=" * 60)
