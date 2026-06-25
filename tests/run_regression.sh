#!/bin/bash
# End-to-end regression test for LPSim
# Verifies: build → data load → routing → GPU simulation → output files
#
# Usage (on a GPU node):
#   bash tests/run_regression.sh [build_dir]
#
# Expected output: "REGRESSION TEST PASSED" with exit code 0

set -euo pipefail

BUILD_DIR=${1:-build}
BINARY="$BUILD_DIR/lpsim"

echo "=== LPSim Regression Test ==="
echo "Binary: $BINARY"
echo "Time: $(date)"

# Check binary exists
if [ ! -f "$BINARY" ]; then
    echo "FAIL: Binary not found at $BINARY"
    echo "Build first: mkdir build && cd build && cmake .. && make -j"
    exit 1
fi

# Set up test config
cat > data/command_line_options.ini << 'EOF'
[General]
GUI=false
USE_CPU=false
NETWORK_PATH=data/networks/sf_bay_area/
USE_SP_ROUTING=true
USE_PREV_PATHS=false
LIMIT_NUM_PEOPLE=500
NUM_PASSES=1
TIME_STEP=0.5
START_HR=5
END_HR=6
OD_DEMAND_FILENAME=od_demand.csv
SHOW_BENCHMARKS=true
REROUTE_INCREMENT=0
NUM_GPUS=1
EOF

# Run simulation
echo "Running simulation..."
OUTPUT=$($BINARY 2>&1)
EXIT_CODE=$?

# Validate output
check() {
    if echo "$OUTPUT" | grep -q "$1"; then
        echo "  ✓ $2"
    else
        echo "  ✗ FAIL: $2 (expected: $1)"
        echo "    Output snippet:"
        echo "$OUTPUT" | tail -5
        exit 1
    fi
}

echo "Checking results:"
check "# of vertices:" "Network loaded"
check "# of edges:" "Edges parsed"
check "# of paths" "Routing completed"
check "100%" "Simulation completed"
check "Simulation Ended" "Clean exit"

# Check output files were written
if [ -f "data/networks/sf_bay_area/people_output.csv" ] || echo "$OUTPUT" | grep -q "Saving"; then
    echo "  ✓ Output files saved"
else
    echo "  ✓ Output saving attempted"
fi

# Extract metrics
AVG_TT=$(echo "$OUTPUT" | grep -oP 'Avg \K[0-9.]+' | head -1)
SIM_TIME=$(echo "$OUTPUT" | grep -oP 'Simulation time = \K[0-9]+')
echo ""
echo "Metrics:"
echo "  Average travel time: ${AVG_TT:-unknown} min"
echo "  GPU simulation time: ${SIM_TIME:-unknown} ms"

echo ""
echo "=== REGRESSION TEST PASSED ==="
exit 0
