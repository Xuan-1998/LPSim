# LPSim: Large-scale Parallel Traffic Simulator

GPU-accelerated, multi-GPU traffic microsimulator for HPC and AI research at city scale: powers ICML 2026 deep-RL ride-hailing dispatch, MoE routing policies, and closed-loop LLM-driven fleet optimization.

<img width="1200" alt="LPSim Bay Area" src="https://github.com/Xuan-1998/LPSim/assets/58761221/1c41f659-aee0-4887-99e0-39b0133154ce">

---

## Key Features

| Feature | Description |
|---------|-------------|
| **Multi-GPU** | Graph-partitioned simulation across 1-8 GPUs with ghost-zone vehicle migration |
| **City-scale** | 223K+ nodes, 540K+ edges (SF Bay Area) in seconds on a single GPU |
| **Multi-modal** | Cars + UAM (urban air mobility) with vertiport queueing |
| **RL-ready** | Gymnasium environment wrapper for training dispatch/pricing policies |
| **LLM-ready** | Structured state API compatible with OpenAI/Anthropic tool-use |
| **Web Viz** | Browser-based 3D network visualization via deck.gl |

---

## Quick Start

```bash
# Clone and build
git clone https://github.com/Xuan-1998/LPSim.git && cd LPSim
docker run -it --rm --gpus all -v "$PWD":/lpsim -w /lpsim yibo123/lpsim:cuda12.4 bash

# Inside container:
apt-get install -y cmake
mkdir build && cd build
cmake .. -DCMAKE_CUDA_ARCHITECTURES=80
make -j
cd .. && build/lpsim
```

## Web Visualizer

```bash
python3 viz/server.py --network data/networks/sf_bay_area
# Open http://localhost:8080
```

## Generate Synthetic Demand

```bash
python3 tools/generate_demand.py \
    --network data/networks/sf_bay_area \
    --num-trips 10000 --model poisson --seed 42
```

---

## Project Structure

```
src/
  simulator/     CUDA kernels + multi-GPU orchestration
  routing/       Contraction hierarchies shortest path
  io/            Network and demand CSV loaders
include/lpsim/  Public headers (vehicle, edge, intersection structs)
data/           Network data + configuration
tools/          Python utilities (demand gen, partitioner, profiler)
lpsim_env/      RL (Gymnasium) + LLM playground interfaces
viz/            Web-based traffic visualizer
tests/          Unit tests
```

---

## Performance

| Metric | Value |
|--------|-------|
| Network | SF Bay Area: 223K nodes, 540K edges |
| Routing | 5,000 paths via CH in 2.0s (192 threads) |
| Simulation | 1 hour simulated → 12.1s GPU time |
| Hardware | NVIDIA H200 (143 GB HBM3e) |
| GPU Memory | 2.4 GB for full network |

---

## LLM Integration

```python
from lpsim_env.llm_interface import LPSimPlayground, TOOL_DEFINITION

playground = LPSimPlayground(network="sf_bay_area", num_trips=5000)
state = playground.get_state()
print(state.to_prompt_context())  # Natural language for LLM context

# Use TOOL_DEFINITION with OpenAI/Anthropic function calling
```

## RL Environment

```python
from lpsim_env import LPSimEnv

env = LPSimEnv(network_path="data/networks/sf_bay_area", reward_type="travel_time")
obs, info = env.reset()
obs, reward, done, trunc, info = env.step(action)
```

---

## Citing LPSim

**Multi-GPU traffic assignment (TR-C 2024)**
> Jiang, X., Sengupta, R., Demmel, J., & Williams, S. (2024). *Large scale multi-GPU based parallel traffic simulation for accelerated traffic assignment and propagation.* Transportation Research Part C, 169, 104873.

**Deep RL ride-hailing dispatch (ICML 2026)**
> Tang, Y., Cui, K., Park, J. H., Zhao, Y., Jiang, X., et al. (2026). *RAST-MoE-RL: A Regime-Aware Spatio-Temporal MoE Framework for Deep Reinforcement Learning in Ride-Hailing.* ICML 2026.

---

## License

MIT
