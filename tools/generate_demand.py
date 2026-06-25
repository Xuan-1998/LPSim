#!/usr/bin/env python3
"""
Synthetic OD Demand Generator for LPSim

Generates configurable origin-destination trip matrices for any network
in the LPSim data format. Supports uniform random, gravity model, and
Poisson arrival patterns.

Usage:
    python tools/generate_demand.py --network LivingCity/data/networks/sf_bay_area \
                                    --num-trips 10000 \
                                    --start-hour 5 --end-hour 12 \
                                    --model uniform \
                                    --seed 42

Output: writes od_demand.csv to the specified network directory.
"""

import argparse
import csv
import math
import os
import random
import sys


def load_nodes(network_path):
    """Load node indices from the network's nodes.csv."""
    nodes_file = os.path.join(network_path, "nodes.csv")
    if not os.path.exists(nodes_file):
        print(f"Error: {nodes_file} not found", file=sys.stderr)
        sys.exit(1)

    indices = []
    with open(nodes_file) as f:
        reader = csv.DictReader(f)
        has_index = "index" in reader.fieldnames
        for i, row in enumerate(reader):
            idx = int(row["index"]) if has_index else i
            indices.append(idx)

    print(f"Loaded {len(indices)} nodes from {nodes_file}")
    return indices


def load_edges(network_path):
    """Load edges for gravity model (need distances between connected nodes)."""
    edges_file = os.path.join(network_path, "edges.csv")
    if not os.path.exists(edges_file):
        return {}

    adjacency = {}
    with open(edges_file) as f:
        reader = csv.DictReader(f)
        has_uv = "u" in reader.fieldnames and "v" in reader.fieldnames
        if not has_uv:
            return {}
        for row in reader:
            u, v = int(row["u"]), int(row["v"])
            length = float(row["length"])
            adjacency.setdefault(u, []).append((v, length))
            adjacency.setdefault(v, []).append((u, length))

    return adjacency


def generate_uniform(nodes, num_trips, start_sec, end_sec, rng):
    """Uniform random: random O/D pairs with uniform departure times."""
    trips = []
    for _ in range(num_trips):
        o, d = rng.sample(nodes, 2)
        dep = rng.uniform(start_sec, end_sec)
        trips.append((dep, o, d))
    return sorted(trips)


def generate_gravity(nodes, num_trips, start_sec, end_sec, rng, adjacency, beta=0.001):
    """Gravity model: probability of trip (i,j) inversely proportional to distance."""
    if not adjacency:
        print("Warning: no edge data for gravity model, falling back to uniform")
        return generate_uniform(nodes, num_trips, start_sec, end_sec, rng)

    node_set = set(nodes)
    connected = [n for n in nodes if n in adjacency]
    if len(connected) < 2:
        return generate_uniform(nodes, num_trips, start_sec, end_sec, rng)

    trips = []
    for _ in range(num_trips):
        o = rng.choice(connected)
        neighbors = [(v, l) for v, l in adjacency.get(o, []) if v in node_set]
        if not neighbors:
            d = rng.choice([n for n in connected if n != o])
        else:
            weights = [math.exp(-beta * l) for _, l in neighbors]
            d = rng.choices([v for v, _ in neighbors], weights=weights, k=1)[0]
        dep = rng.uniform(start_sec, end_sec)
        trips.append((dep, o, d))
    return sorted(trips)


def generate_poisson(nodes, num_trips, start_sec, end_sec, rng):
    """Poisson arrivals: exponential inter-arrival times, uniform O/D."""
    duration = end_sec - start_sec
    rate = num_trips / duration

    trips = []
    t = start_sec
    while len(trips) < num_trips:
        t += rng.expovariate(rate)
        if t >= end_sec:
            break
        o, d = rng.sample(nodes, 2)
        trips.append((t, o, d))

    while len(trips) < num_trips:
        dep = rng.uniform(start_sec, end_sec)
        o, d = rng.sample(nodes, 2)
        trips.append((dep, o, d))

    return sorted(trips[:num_trips])


def main():
    parser = argparse.ArgumentParser(description="Generate synthetic OD demand for LPSim")
    parser.add_argument("--network", required=True, help="Path to network directory")
    parser.add_argument("--num-trips", type=int, default=5000, help="Number of trips")
    parser.add_argument("--start-hour", type=float, default=5.0, help="Start hour (24h)")
    parser.add_argument("--end-hour", type=float, default=12.0, help="End hour (24h)")
    parser.add_argument("--model", choices=["uniform", "gravity", "poisson"], default="uniform")
    parser.add_argument("--seed", type=int, default=42, help="Random seed")
    parser.add_argument("--output", default=None, help="Output file (default: network/od_demand.csv)")
    parser.add_argument("--beta", type=float, default=0.001, help="Gravity model decay (for --model gravity)")
    args = parser.parse_args()

    rng = random.Random(args.seed)
    nodes = load_nodes(args.network)
    start_sec = args.start_hour * 3600
    end_sec = args.end_hour * 3600

    if args.model == "uniform":
        trips = generate_uniform(nodes, args.num_trips, start_sec, end_sec, rng)
    elif args.model == "gravity":
        adjacency = load_edges(args.network)
        trips = generate_gravity(nodes, args.num_trips, start_sec, end_sec, rng, adjacency, args.beta)
    elif args.model == "poisson":
        trips = generate_poisson(nodes, args.num_trips, start_sec, end_sec, rng)

    output_file = args.output or os.path.join(args.network, "od_demand.csv")
    with open(output_file, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["dep_time", "origin", "destination"])
        for dep, o, d in trips:
            writer.writerow([f"{dep:.1f}", o, d])

    print(f"Generated {len(trips)} trips ({args.model} model) → {output_file}")
    print(f"  Time window: {args.start_hour:.0f}h – {args.end_hour:.0f}h")
    print(f"  Seed: {args.seed}")


if __name__ == "__main__":
    main()
