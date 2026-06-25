#!/bin/bash
# Profile LPSim GPU simulation with NVIDIA Nsight Systems and Nsight Compute
#
# Usage:
#   ./tools/profile_sim.sh [nsys|ncu|both] [output_prefix]
#
# Prerequisites: nsys and/or ncu in PATH (CUDA Toolkit 12.x)
# Run from the LivingCity/ directory after building.

set -euo pipefail

MODE=${1:-nsys}
PREFIX=${2:-lpsim_profile}
BINARY="./LivingCity"

if [ ! -f "$BINARY" ]; then
    echo "Error: $BINARY not found. Build first with: qmake && make -j"
    exit 1
fi

echo "=== LPSim GPU Profiler ==="
echo "Mode: $MODE | Output prefix: $PREFIX"
echo ""

profile_nsys() {
    echo "[nsys] Capturing timeline trace..."
    nsys profile \
        --output "${PREFIX}_timeline" \
        --trace cuda,nvtx,osrt \
        --force-overwrite true \
        --stats true \
        $BINARY 2>&1 | tee "${PREFIX}_nsys.log"
    echo ""
    echo "[nsys] Timeline saved to: ${PREFIX}_timeline.nsys-rep"
    echo "  View with: nsys-ui ${PREFIX}_timeline.nsys-rep"
}

profile_ncu() {
    echo "[ncu] Capturing per-kernel metrics..."
    echo "  Targeting: kernel_trafficSimulation"
    ncu \
        --output "${PREFIX}_kernels" \
        --set full \
        --kernel-name "kernel_trafficSimulation" \
        --launch-count 5 \
        --target-processes all \
        $BINARY 2>&1 | tee "${PREFIX}_ncu.log"
    echo ""
    echo "[ncu] Kernel metrics saved to: ${PREFIX}_kernels.ncu-rep"
    echo "  View with: ncu-ui ${PREFIX}_kernels.ncu-rep"
    echo ""
    echo "Key metrics to check:"
    echo "  - sm__throughput.avg_pct_of_peak_sustained_elapsed (compute util)"
    echo "  - dram__throughput.avg_pct_of_peak_sustained_elapsed (memory BW)"
    echo "  - sm__warps_active.avg_per_cycle_active (occupancy)"
    echo "  - l1tex__t_sectors_pipe_lsu_mem_global_op_ld.sum (global loads)"
}

case "$MODE" in
    nsys)  profile_nsys ;;
    ncu)   profile_ncu ;;
    both)  profile_nsys; echo ""; profile_ncu ;;
    *)     echo "Unknown mode: $MODE. Use nsys, ncu, or both."; exit 1 ;;
esac

echo ""
echo "=== Profiling complete ==="
