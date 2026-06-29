#!/usr/bin/env python3
"""
LLM Playground Interface for LPSim

Exposes the traffic simulator state as structured context that any LLM
(GPT-4, Claude, Llama, etc.) can reason about. The LLM acts as a fleet
dispatcher/controller making decisions at each epoch.

Design philosophy: the simulator is a *tool* the LLM calls, not a black
box it's trained on. This enables zero-shot reasoning, chain-of-thought
planning, and MoE routing without any RL training loop.

Usage:
    from lpsim_env.llm_interface import LPSimPlayground

    playground = LPSimPlayground(network="sf_bay_area", num_trips=5000)
    state = playground.get_state()  # structured JSON for LLM context

    # LLM reasons about the state and returns an action
    action = llm_decide(state)  # your LLM call here

    result = playground.apply_action(action)
    print(result["metrics"])
"""

import csv
import json
import os
import subprocess
from dataclasses import dataclass, field
from typing import Any, Optional


@dataclass
class NetworkState:
    """Structured representation of traffic state for LLM consumption."""
    num_nodes: int = 0
    num_edges: int = 0
    num_active_vehicles: int = 0
    simulation_time_sec: float = 0.0
    congested_edges: list = field(default_factory=list)
    top_bottlenecks: list = field(default_factory=list)
    zone_summaries: dict = field(default_factory=dict)

    def to_prompt_context(self, max_edges: int = 20) -> str:
        """Format state as natural language for LLM context window."""
        lines = [
            f"## Traffic Network State (t={self.simulation_time_sec:.0f}s)",
            f"- Network: {self.num_nodes:,} intersections, {self.num_edges:,} road segments",
            f"- Active vehicles: {self.num_active_vehicles:,}",
            "",
            "### Top Bottlenecks (slowest edges):",
        ]
        for i, edge in enumerate(self.top_bottlenecks[:max_edges], 1):
            lines.append(
                f"  {i}. Edge {edge['id']}: "
                f"speed={edge['avg_speed']:.1f} m/s "
                f"(limit={edge['speed_limit']:.1f}), "
                f"vehicles={edge['num_vehicles']}, "
                f"length={edge['length']:.0f}m"
            )
        lines.append("")
        lines.append("### Zone Summaries:")
        for zone, summary in self.zone_summaries.items():
            lines.append(f"  {zone}: {summary['vehicles']} vehicles, avg_speed={summary['avg_speed']:.1f} m/s")
        return "\n".join(lines)

    def to_json(self) -> str:
        """Structured JSON for programmatic LLM tool use."""
        return json.dumps({
            "network": {"nodes": self.num_nodes, "edges": self.num_edges},
            "time_sec": self.simulation_time_sec,
            "active_vehicles": self.num_active_vehicles,
            "bottlenecks": self.top_bottlenecks,
            "zones": self.zone_summaries,
        }, indent=2)


@dataclass
class LLMAction:
    """Structured action format that LLMs can output."""
    action_type: str  # "set_tolls", "reroute", "dispatch", "adjust_signals"
    targets: list = field(default_factory=list)  # edge/node IDs
    values: list = field(default_factory=list)  # action magnitudes
    reasoning: str = ""  # LLM's chain-of-thought (logged, not used)

    @classmethod
    def from_json(cls, json_str: str) -> "LLMAction":
        data = json.loads(json_str)
        return cls(
            action_type=data.get("action_type", "set_tolls"),
            targets=data.get("targets", []),
            values=data.get("values", []),
            reasoning=data.get("reasoning", ""),
        )


class LPSimPlayground:
    """
    LLM-friendly interface to the LPSim traffic simulator.

    The playground exposes:
    1. get_state() → NetworkState (for LLM context)
    2. apply_action(LLMAction) → results dict
    3. get_available_actions() → action schema (for function calling)
    4. run_scenario(config) → full simulation results

    Compatible with OpenAI function calling, Anthropic tool use,
    and any LLM that can output structured JSON.
    """

    def __init__(
        self,
        network: str = "sf_bay_area",
        simulator_dir: str = "LivingCity",
        data_root: str = "LivingCity/data/networks",
        num_trips: int = 5000,
        num_gpus: int = 1,
        start_hour: float = 5.0,
        end_hour: float = 6.0,
    ):
        self.network_path = os.path.join(data_root, network)
        self.simulator_dir = simulator_dir
        self.num_trips = num_trips
        self.num_gpus = num_gpus
        self.start_hour = start_hour
        self.end_hour = end_hour

        # Load network metadata
        self.edges_data = self._load_edges()
        self.num_nodes = self._count_nodes()
        self.num_edges = len(self.edges_data)

    def _load_edges(self) -> list:
        edges = []
        edges_file = os.path.join(self.network_path, "edges.csv")
        with open(edges_file) as f:
            reader = csv.DictReader(f)
            for row in reader:
                edges.append({
                    "id": int(row.get("uniqueid", 0)),
                    "length": float(row.get("length", 0)),
                    "lanes": int(float(row.get("lanes", 1))),
                    "speed_limit": float(row.get("speed_mph", 30)) * 0.44704,
                })
        return edges

    def _count_nodes(self) -> int:
        nodes_file = os.path.join(self.network_path, "nodes.csv")
        with open(nodes_file) as f:
            return sum(1 for _ in f) - 1

    def get_state(self, simulation_results: Optional[dict] = None) -> NetworkState:
        """Get current network state formatted for LLM consumption."""
        state = NetworkState(
            num_nodes=self.num_nodes,
            num_edges=self.num_edges,
            num_active_vehicles=self.num_trips,
            simulation_time_sec=self.start_hour * 3600,
        )

        if simulation_results and "edge_speeds" in simulation_results:
            # Compute bottlenecks from actual simulation output
            edge_speeds = simulation_results["edge_speeds"]
            bottlenecks = []
            for edge in self.edges_data:
                eid = edge["id"]
                if eid in edge_speeds:
                    bottlenecks.append({
                        "id": eid,
                        "avg_speed": edge_speeds[eid],
                        "speed_limit": edge["speed_limit"],
                        "num_vehicles": simulation_results.get("edge_vehicles", {}).get(eid, 0),
                        "length": edge["length"],
                    })
            bottlenecks.sort(key=lambda x: x["avg_speed"])
            state.top_bottlenecks = bottlenecks[:20]
        else:
            # Pre-simulation: report capacity-based potential bottlenecks
            sorted_edges = sorted(self.edges_data, key=lambda e: e["length"] / max(e["lanes"], 1), reverse=True)
            state.top_bottlenecks = [
                {"id": e["id"], "avg_speed": e["speed_limit"], "speed_limit": e["speed_limit"],
                 "num_vehicles": 0, "length": e["length"]}
                for e in sorted_edges[:20]
            ]

        return state

    def get_available_actions(self) -> dict:
        """Return action schema for LLM function-calling format."""
        return {
            "name": "traffic_control",
            "description": "Apply traffic control actions to the city-scale network",
            "parameters": {
                "type": "object",
                "properties": {
                    "action_type": {
                        "type": "string",
                        "enum": ["set_tolls", "adjust_signals", "reroute", "dispatch"],
                        "description": "Type of control action"
                    },
                    "targets": {
                        "type": "array",
                        "items": {"type": "integer"},
                        "description": "Edge or node IDs to target"
                    },
                    "values": {
                        "type": "array",
                        "items": {"type": "number"},
                        "description": "Action values (toll amounts, signal durations, etc.)"
                    },
                    "reasoning": {
                        "type": "string",
                        "description": "Your reasoning for this action (for logging)"
                    }
                },
                "required": ["action_type", "targets", "values"]
            }
        }

    def apply_action(self, action: LLMAction) -> dict:
        """Apply an LLM-generated action and run simulation."""
        # Write action to disk for simulator
        self._write_action(action)
        # Run simulation
        result = self._run_sim()
        return {
            "metrics": result,
            "action_applied": action.action_type,
            "num_targets": len(action.targets),
        }

    def run_scenario(self, **kwargs) -> dict:
        """Run a full simulation with custom parameters."""
        return self._run_sim(**kwargs)

    def _write_action(self, action: LLMAction):
        action_file = os.path.join(self.simulator_dir, "rl_action.csv")
        with open(action_file, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["edge_id", "value"])
            for target, value in zip(action.targets, action.values):
                writer.writerow([target, value])

    def _run_sim(self, **overrides) -> dict:
        config = {
            "GUI": "false",
            "USE_CPU": "false",
            "NETWORK_PATH": self.network_path.replace("LivingCity/", "") + "/",
            "USE_SP_ROUTING": "true",
            "USE_PREV_PATHS": "false",
            "NUM_PASSES": "1",
            "TIME_STEP": "0.5",
            "START_HR": str(int(self.start_hour)),
            "END_HR": str(int(self.end_hour)),
            "OD_DEMAND_FILENAME": "od_demand.csv",
            "SHOW_BENCHMARKS": "true",
            "REROUTE_INCREMENT": "0",
            "NUM_GPUS": str(self.num_gpus),
            "LIMIT_NUM_PEOPLE": str(self.num_trips),
        }
        config.update(overrides)

        ini_path = os.path.join(self.simulator_dir, "command_line_options.ini")
        with open(ini_path, "w") as f:
            f.write("[General]\n")
            for k, v in config.items():
                f.write(f"{k}={v}\n")

        binary = os.path.join(self.simulator_dir, "LivingCity")
        result = subprocess.run(
            [binary], cwd=self.simulator_dir,
            capture_output=True, text=True, timeout=300
        )

        return self._parse_metrics(result.stdout)

    def _parse_metrics(self, stdout: str) -> dict:
        metrics = {"avg_travel_time_min": 0, "total_co": 0, "sim_time_ms": 0}
        for line in stdout.split("\n"):
            if "Avg" in line and "min" in line:
                parts = line.split()
                for i, p in enumerate(parts):
                    if p == "Avg" and i + 1 < len(parts):
                        try:
                            metrics["avg_travel_time_min"] = float(parts[i + 1])
                        except ValueError:
                            pass
            if "Simulation time" in line:
                parts = line.split("=")
                if len(parts) == 2:
                    try:
                        metrics["sim_time_ms"] = int(parts[1].strip().replace("ms", "").strip())
                    except ValueError:
                        pass
        return metrics


# Example usage with OpenAI-style function calling
TOOL_DEFINITION = {
    "type": "function",
    "function": {
        "name": "lpsim_traffic_control",
        "description": "Control a large-scale GPU network simulation (223K nodes, 540K edges). "
                       "Apply tolls, reroute vehicles, or adjust signals to optimize travel time.",
        "parameters": {
            "type": "object",
            "properties": {
                "action_type": {
                    "type": "string",
                    "enum": ["set_tolls", "adjust_signals", "reroute"],
                },
                "targets": {
                    "type": "array",
                    "items": {"type": "integer"},
                    "description": "Edge IDs to modify (0-540826)"
                },
                "values": {
                    "type": "array",
                    "items": {"type": "number"},
                    "description": "Toll amount ($) or signal duration (s)"
                },
            },
            "required": ["action_type", "targets", "values"]
        }
    }
}
