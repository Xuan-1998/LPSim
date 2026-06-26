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
| Routing | 5,000 paths via CH in 2.0s |
| Simulation | 1 hour simulated in seconds on modern GPUs |
| GPU Memory | ~2.4 GB for full network |

## Live Demo

[**Open the interactive network visualizer →**](https://xuan-1998.github.io/LPSim/)

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

If LPSim contributes to your work, please cite the relevant paper(s):

**Multi-GPU Traffic Simulation (Transportation Research Part C, 2024)**
> Jiang, X., Sengupta, R., Demmel, J., & Williams, S. (2024). *Large scale multi-GPU based parallel traffic simulation for accelerated traffic assignment and propagation.* Transportation Research Part C: Emerging Technologies, 169, 104873. [link](https://www.sciencedirect.com/science/article/pii/S0968090X24003942)

**Deep RL for Ride-Hailing Dispatch (ICML 2026)**
> Tang, Y., Cui, K., Park, J. H., Zhao, Y., Jiang, X.†, He, H., Zhuang, D., Wang, S., Yu, J., Koutsopoulos, H., & Zhao, J. (2026). *RAST-MoE-RL: A Regime-Aware Spatio-Temporal MoE Framework for Deep Reinforcement Learning in Ride-Hailing.* International Conference on Machine Learning (ICML). [arXiv:2512.13727](https://arxiv.org/abs/2512.13727)
>
> † Corresponding author

**Urban Air Mobility Integration (Journal of Air Transportation, 2023)**
> Jiang, X., Tang, Y., Cao, J., Bulusu, V., Yang, H., Peng, X., Zheng, Y., Zhao, J., & Sengupta, R. (2023). *Simulating Integration of Urban Air Mobility into Existing Transportation Systems: Survey.* Journal of Air Transportation, 32(3), 97-107. [link](https://arc.aiaa.org/doi/10.2514/1.D0431)

**Time-Driven Simulation Framework (ACM SIGSIM, 2024)**
> Jiang, X. (2024). *Designing a Time-Driven Simulation Framework for Large-Scale Traffic Networks.* Proceedings of the 38th ACM SIGSIM Conference on Principles of Advanced Discrete Simulation (PADS).

---

## License

MIT
