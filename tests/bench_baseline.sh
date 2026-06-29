#!/bin/bash
#SBATCH -N 2
#SBATCH --exclusive
#SBATCH --time=00:10:00
#SBATCH --job-name=no-shell
#SBATCH --output=/fsx/xuanj/bench_baseline_%j.log

trap 'echo CLEANUP_DONE' EXIT
cd /fsx/xuanj/LPSim-test

docker run --rm --gpus all -v $PWD:/lpsim -w /lpsim yibo123/lpsim:cuda12.4 bash -c '
apt-get update -qq && apt-get install -y -qq cmake > /dev/null 2>&1
if [ ! -f build/lpsim ]; then
  mkdir -p build && cd build && cmake .. -DCMAKE_CUDA_ARCHITECTURES=80 -DCMAKE_CXX_FLAGS="-w" > /dev/null 2>&1 && make -j8 2>&1 | tail -1 && cd /lpsim
fi

echo "=== BASELINE BENCHMARK (3 runs) ==="
for i in 1 2 3; do
  echo "--- Run $i ---"
  timeout 120 build/lpsim 2>&1 | grep -E "Simulation time|Microsimulation|paths|People"
done
echo "=== BENCHMARK DONE ==="
'
