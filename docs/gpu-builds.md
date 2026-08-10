# Portable NVIDIA GPU builds

LPSim can package native CUDA machine code (cubin) and driver-JIT-compatible
PTX in the same executable. At runtime the NVIDIA driver selects the best image
for the installed GPU.

## Default build profiles

When no architecture option is supplied, CMake selects a profile based on the
CUDA compiler:

| CUDA toolkit | Native cubins | Embedded PTX | Blackwell behavior |
| --- | --- | --- | --- |
| 12.8 or newer | `sm_80`, `sm_89`, `sm_90`, `sm_100` | `compute_80`, `compute_100` | Native `sm_100` |
| Older than 12.8 | `sm_80`, `sm_89`, `sm_90` | `compute_80`, `compute_90` | Driver JIT from PTX |

This covers A100 (8.0), L40/L40S (8.9), H100/H200 (9.0), and B100/B200
(10.0). A recent NVIDIA driver is required to JIT older PTX on a newer GPU.

Use the standard CMake option for a machine-specific build:

```bash
cmake -S . -B build -DCMAKE_CUDA_ARCHITECTURES="90-real;90-virtual"
cmake --build build -j
```

Or use the LPSim-specific override, which is convenient in Docker build args:

```bash
cmake -S . -B build \
  -DLPSIM_CUDA_ARCHITECTURES="80-real;89-real;90-real;100-real;80-virtual;100-virtual"
cmake --build build -j
```

The tracked Dockerfile uses CUDA 12.9.1 and the portable-native profile by
default. A smaller GPU-specific image can override the same setting:

```bash
docker build -t lpsim .
docker build -t lpsim:h100 \
  --build-arg LPSIM_CUDA_ARCHITECTURES="90-real;90-virtual" .
```

Native `sm_100` compilation requires CUDA 12.8 or newer. Architecture-specific
targets such as `sm_100a` are intentionally not enabled because their cubins do
not run on every compute-capability 10.0 GPU.

## Inspect a build

The artifact checker wraps `cuobjdump` and fails when a required image is absent:

```bash
python3 tools/check_cuda_artifacts.py build/lpsim \
  --require-real 80 89 90 100 \
  --require-virtual 80 100
```

LPSim also prints the nvcc version, embedded architectures, driver API/runtime
versions, selected GPU names, compute capabilities, memory, SM counts, and P2P
availability during CUDA initialization.

## Validate both loading paths

Run a representative regression once with PTX disabled to prove the matching
cubin is present, and once with JIT forced to prove the fallback is usable:

```bash
CUDA_DISABLE_PTX_JIT=1 tests/run_regression.sh build
CUDA_FORCE_PTX_JIT=1 tests/run_regression.sh build
```

The first run should be used for native H100 and B200 validation. The forced-JIT
run is slower on first launch because the driver compiles and caches the PTX.
Floating-point reductions and atomic update order can vary between GPU models,
so cross-GPU tests should compare invariants and tolerances rather than requiring
byte-identical output files.
