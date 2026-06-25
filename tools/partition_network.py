#!/usr/bin/env python3
"""
Graph Partitioning Tool for Multi-GPU LPSim

Produces a vertex partition file (partitions.txt) that assigns each node
to a GPU partition. Uses METIS for min-cut balanced partitioning when
available, falls back to spatial bisection (longitude-based) otherwise.

Usage:
    python tools/partition_network.py \
        --network LivingCity/data/networks/sf_bay_area \
        --num-parts 4 \
        --method metis

Output: writes partitions.txt to the network directory.
"""

import argparse
import csv
import os
import sys
from collections import defaultdict


def load_graph(network_path):
    """Load graph as adjacency list from edges.csv."""
    edges_file = os.path.join(network_path, "edges.csv")
    nodes_file = os.path.join(network_path, "nodes.csv")

    # Count nodes
    num_nodes = 0
    node_coords = {}
    with open(nodes_file) as f:
        reader = csv.DictReader(f)
        has_index = "index" in reader.fieldnames
        for i, row in enumerate(reader):
            idx = int(row["index"]) if has_index else i
            node_coords[idx] = (float(row["x"]), float(row["y"]))
            num_nodes = max(num_nodes, idx + 1)

    # Build adjacency
    adjacency = defaultdict(list)
    with open(edges_file) as f:
        reader = csv.DictReader(f)
        cols = reader.fieldnames
        u_col = "u" if "u" in cols else None
        v_col = "v" if "v" in cols else None
        if u_col is None:
            print("Error: edges.csv must have 'u' and 'v' columns", file=sys.stderr)
            sys.exit(1)
        for row in reader:
            u, v = int(row[u_col]), int(row[v_col])
            if u < num_nodes and v < num_nodes:
                adjacency[u].append(v)
                adjacency[v].append(u)

    return num_nodes, adjacency, node_coords


def partition_metis(num_nodes, adjacency, num_parts):
    """Use METIS for balanced min-cut partitioning."""
    try:
        import pymetis
    except ImportError:
        print("pymetis not installed. Install with: pip install pymetis")
        print("Falling back to spatial partitioning.")
        return None

    # Convert to CSR format for pymetis
    xadj = [0]
    adjncy = []
    for i in range(num_nodes):
        neighbors = list(set(adjacency.get(i, [])))
        adjncy.extend(neighbors)
        xadj.append(len(adjncy))

    print(f"Running METIS: {num_nodes} vertices, {len(adjncy)} adjacencies, {num_parts} partitions")
    _, membership = pymetis.part_graph(num_parts, xadj=xadj, adjncy=adjncy)
    return membership


def partition_spatial(num_nodes, node_coords, num_parts):
    """Spatial bisection based on longitude coordinate."""
    # Sort nodes by x-coordinate (longitude) and split evenly
    nodes_with_coord = [(idx, coord[0]) for idx, coord in node_coords.items()]
    nodes_with_coord.sort(key=lambda x: x[1])

    partition = [0] * num_nodes
    chunk_size = len(nodes_with_coord) // num_parts

    for rank, (idx, _) in enumerate(nodes_with_coord):
        part_id = min(rank // chunk_size, num_parts - 1)
        partition[idx] = part_id

    return partition


def partition_random(num_nodes, num_parts, seed=42):
    """Random balanced assignment (baseline for comparison)."""
    import random
    rng = random.Random(seed)
    partition = [i % num_parts for i in range(num_nodes)]
    rng.shuffle(partition)
    return partition


def compute_edge_cut(adjacency, partition):
    """Count edges that cross partition boundaries."""
    cut = 0
    for u, neighbors in adjacency.items():
        for v in neighbors:
            if partition[u] != partition[v]:
                cut += 1
    return cut // 2  # each edge counted twice


def main():
    parser = argparse.ArgumentParser(description="Partition road network for multi-GPU simulation")
    parser.add_argument("--network", required=True, help="Path to network directory")
    parser.add_argument("--num-parts", type=int, default=2, help="Number of GPU partitions")
    parser.add_argument("--method", choices=["metis", "spatial", "random"], default="metis")
    parser.add_argument("--output", default=None, help="Output file (default: network/partitions.txt)")
    args = parser.parse_args()

    num_nodes, adjacency, node_coords = load_graph(args.network)
    print(f"Graph: {num_nodes} nodes, {sum(len(v) for v in adjacency.values())//2} edges")

    if args.method == "metis":
        partition = partition_metis(num_nodes, adjacency, args.num_parts)
        if partition is None:
            partition = partition_spatial(num_nodes, node_coords, args.num_parts)
    elif args.method == "spatial":
        partition = partition_spatial(num_nodes, node_coords, args.num_parts)
    elif args.method == "random":
        partition = partition_random(num_nodes, args.num_parts)

    # Compute quality metrics
    edge_cut = compute_edge_cut(adjacency, partition)
    part_sizes = defaultdict(int)
    for p in partition:
        part_sizes[p] += 1
    imbalance = max(part_sizes.values()) / min(part_sizes.values()) if part_sizes else 0

    print(f"Partition quality ({args.method}):")
    print(f"  Edge cut: {edge_cut} ({100*edge_cut/max(1,sum(len(v) for v in adjacency.values())//2):.1f}% of edges)")
    print(f"  Imbalance ratio: {imbalance:.3f}")
    for p in sorted(part_sizes):
        print(f"  Partition {p}: {part_sizes[p]} nodes")

    # Write output
    output_file = args.output or os.path.join(args.network, "partitions.txt")
    with open(output_file, "w") as f:
        for p in partition:
            f.write(f"{p}\n")

    print(f"Written to: {output_file}")


if __name__ == "__main__":
    main()
