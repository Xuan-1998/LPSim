#!/usr/bin/env python3
"""
Compute per-GPU partition weights based on available device memory.

On heterogeneous GPU nodes (e.g., mixed A100-40G and A100-80G), this
computes the proportional share each GPU should handle based on its
free memory. Output is a weights file that the partitioner uses.

Usage:
    python tools/gpu_partition_weights.py --num-gpus 4

Requires: pynvml (pip install pynvml)
"""

import argparse
import json
import sys

def get_gpu_memory_info(num_gpus):
    """Query NVIDIA GPU memory using pynvml."""
    try:
        import pynvml
        pynvml.nvmlInit()

        device_count = pynvml.nvmlDeviceGetCount()
        if num_gpus > device_count:
            print(f"Warning: requested {num_gpus} GPUs but only {device_count} available")
            num_gpus = device_count

        gpu_info = []
        for i in range(num_gpus):
            handle = pynvml.nvmlDeviceGetHandleByIndex(i)
            mem_info = pynvml.nvmlDeviceGetMemoryInfo(handle)
            name = pynvml.nvmlDeviceGetName(handle)
            gpu_info.append({
                "index": i,
                "name": name,
                "total_mb": mem_info.total // (1024 * 1024),
                "free_mb": mem_info.free // (1024 * 1024),
            })

        pynvml.nvmlShutdown()
        return gpu_info
    except ImportError:
        print("pynvml not available, using uniform weights")
        return [{"index": i, "name": f"GPU {i}", "total_mb": 80000, "free_mb": 80000}
                for i in range(num_gpus)]


def compute_weights(gpu_info):
    """Compute partition weight per GPU proportional to free memory."""
    total_free = sum(g["free_mb"] for g in gpu_info)
    if total_free == 0:
        return [1.0 / len(gpu_info)] * len(gpu_info)
    return [g["free_mb"] / total_free for g in gpu_info]


def main():
    parser = argparse.ArgumentParser(description="Compute GPU partition weights")
    parser.add_argument("--num-gpus", type=int, default=4)
    parser.add_argument("--output", default=None, help="Output JSON file")
    args = parser.parse_args()

    gpu_info = get_gpu_memory_info(args.num_gpus)
    weights = compute_weights(gpu_info)

    print("GPU Memory Analysis:")
    for g, w in zip(gpu_info, weights):
        print(f"  GPU {g['index']} ({g['name']}): {g['free_mb']} MB free → weight {w:.3f}")

    result = {
        "num_gpus": len(gpu_info),
        "weights": weights,
        "gpus": gpu_info,
    }

    if args.output:
        with open(args.output, "w") as f:
            json.dump(result, f, indent=2)
        print(f"Written to {args.output}")
    else:
        print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
