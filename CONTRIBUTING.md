# Contributing to LPSim

LPSim welcomes contributions in GPU optimization, traffic modeling, RL integration, and multi-modal simulation.

## Quick Start

```bash
docker pull yibo123/lpsim:cuda12.4
git clone git@github.com:Xuan-1998/LPSim.git && cd LPSim
docker run -it --rm --gpus all -v "$PWD":/lpsim -w /lpsim yibo123/lpsim:cuda12.4 bash
qmake LivingCity/LivingCity.pro && make -j
cd LivingCity && ./LivingCity
```

## Development Workflow

1. Create a feature branch: `git checkout -b feature/your-feature`
2. Make changes, verify build passes in the Docker container
3. Test with the bundled `sf_bay_area` network (5K synthetic trips)
4. Push and create a PR against `multi-gpu-multimode`
5. CI will verify compilation automatically

## Code Style

- C++/CUDA: use existing formatting (spaces, braces on same line)
- CUDA kernels: prefer `__restrict__` on pointer params, use `gpuErrchk()` macro
- Python tools: PEP 8, type hints where helpful

## Areas for Contribution

- **Multi-GPU optimization**: stream pipelining, dynamic repartitioning, NCCL integration
- **Traffic modeling**: new vehicle types, intersection logic, signal control
- **RL/AI integration**: per-timestep state export, Gymnasium env improvements
- **Profiling**: roofline analysis, memory bandwidth optimization

## Testing

GPU simulation requires NVIDIA GPU hardware. The CI pipeline verifies
compilation on CPU-only runners. For full end-to-end testing:

```bash
# Generate test demand
python tools/generate_demand.py --network LivingCity/data/networks/sf_bay_area --num-trips 1000

# Run simulation (requires GPU)
cd LivingCity && ./LivingCity
```

## Reporting Issues

Include: GPU model, CUDA version, Docker image version, and the full error output.
