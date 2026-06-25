# LPSim — Large-scale Parallel traffic Simulation

LPSim is a GPU-accelerated, multi-GPU traffic microsimulator for HPC and AI research. It scales city-region networks with hundreds of thousands of nodes and millions of agents to run in minutes on commodity GPUs, and it is built to serve as a high-throughput environment for reinforcement-learning policies in mobility (dispatch, routing, pricing, vertiport siting).

The platform underpins our ICML 2026 paper on regime-aware deep RL for ride-hailing dispatch, our T-RC 2024 paper on multi-GPU traffic assignment, and ongoing work on urban air mobility (UAM) integration.

<img width="1200" alt="LPSim Bay Area" src="https://github.com/Xuan-1998/LPSim/assets/58761221/1c41f659-aee0-4887-99e0-39b0133154ce">

---

## Why LPSim

- **City-scale, in minutes.** The full nine-county SF Bay Area network (≈220k nodes, ≈550k edges, millions of trips) runs in single-digit minutes on a small GPU cluster. Half-second time steps; discrete time-driven kernels.
- **Multi-GPU by design.** The road network is partitioned across GPUs; each GPU advances its subgraph independently for many time steps before exchanging boundary state, amortizing communication over compute.
- **HPC-friendly.** Single-source CUDA, OpenMP for host parallelism, CSR graph layout, contraction-hierarchies routing via [Pandana](https://github.com/UDST/pandana) — works on a workstation, scales to a node, scales to multi-node.
- **AI/RL-ready.** Stateful, fast resets and deterministic rollouts make LPSim usable as a closed-loop environment for deep RL on dispatch, routing, and matching problems at metropolitan scale.

![multi-GPU partitioning](https://github.com/Xuan-1998/LPSim/assets/58761221/a8ddbab9-8e17-405e-b392-28044c1d917a)

---

## Quick start (Docker)

The Docker image bundles CUDA 12.4, Qt5, Boost, Pandana, and all build deps. This is the recommended path.

```bash
# Prerequisites: Docker + NVIDIA Container Toolkit + a working `nvidia-smi`
docker pull yibo123/lpsim:cuda12.4

git clone https://github.com/Xuan-1998/LPSim.git && cd LPSim
docker run -it --rm --gpus all -v "$PWD":/lpsim -w /lpsim yibo123/lpsim:cuda12.4 bash

# inside the container
qmake LivingCity/LivingCity.pro
make -j
cd LivingCity && ./LivingCity
```

If `nvidia-smi` does not work on the host, install the matching NVIDIA driver before running the container.

---

## Manual build (Ubuntu 22.04+)

Tested on Ubuntu 22.04 / 24.04 with CUDA 12.3+, GCC 11+, Qt 5.15. Older versions (CUDA 9, GCC 6.4, Qt 5.9.5) still build but are no longer the supported path.

```bash
# system deps
sudo apt update && sudo apt install -y \
    build-essential git wget pciutils \
    qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools \
    libglew-dev libfontconfig1 mesa-common-dev \
    libboost-all-dev libopencv-dev
```

```bash
# CUDA toolkit on PATH (adjust 12.3 to your installed version)
export PATH=/usr/local/cuda-12.3/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-12.3/lib64:$LD_LIBRARY_PATH
```

Pandana (CH routing backend) needs a shared library:

```bash
git clone https://github.com/UDST/pandana.git ~/pandana
cd ~/pandana/src
# Use the bundled PandanaMakefile from this repo, or write your own:
make -f /path/to/LPSim/PandanaMakefile

export LD_LIBRARY_PATH=$HOME/pandana/src:$LD_LIBRARY_PATH
```

Build LPSim:

```bash
cd ~/LPSim
qmake LivingCity/LivingCity.pro
# If the generated Makefile doesn't pick up Pandana headers/libs, add:
#   INCPATH += -I$(HOME)/pandana/src
#   LIBS    += -L$(HOME)/pandana/src -lchrouting
make -j$(nproc)
cd LivingCity && ./LivingCity
```

---

## Configuration

Edit `LivingCity/command_line_options.ini`:

```ini
[General]
NETWORK_PATH=data/networks/sf_bay_area/
USE_SP_ROUTING=true
USE_PREV_PATHS=true
NUM_PASSES=1
TIME_STEP=0.5
START_HR=5
END_HR=12
```

| Key | Meaning |
|---|---|
| `NETWORK_PATH` | Directory containing `nodes.csv`, `edges.csv`, demand. |
| `USE_SP_ROUTING` | Use the CH (Pandana) shortest-path routing. Keep `true`. |
| `USE_PREV_PATHS` | Reuse cached paths from a prior run. `false` on the first run. |
| `NUM_PASSES` | Number of simulation passes per run. |
| `TIME_STEP` | Simulation time step in seconds. `0.5` is the validated default. |
| `START_HR`, `END_HR` | Simulation window, 24-hour clock. |

`GUI`, `USE_CPU`, `USE_JOHNSON_ROUTING`, `LIMIT_NUM_PEOPLE`, `ADD_RANDOM_PEOPLE` are legacy flags. Leave at defaults.

---

## Data

The repo ships network files (`nodes.csv`, `edges.csv`) for several Bay Area subnetworks under `LivingCity/data/networks/`. The default is `sf_bay_area/` (full nine-county SF Bay Area region).

Demand files (OD matrices) are not redistributable here. Contact [Pavan Yedavalli](mailto:pavyedav@gmail.com) for sample or real demand files.

---

## RL / closed-loop usage

LPSim exposes its simulator as a callable kernel-loop, which is convenient as a high-throughput environment for RL policies operating on city-scale fleets. The Python workflow under `optimizer_*MSA_S3*.py` and `final_analysis_0930.py` is the entry point used by our ICML 2026 work; treat those as reference integrations.

If you build a new RL/optimization pipeline on top of LPSim, please cite the relevant paper(s) below.

---

## Development

```bash
git checkout -b my-feature
# edit, build, test
sh LivingCity/runAllTests.sh
```

### Debugging

```bash
cuda-memcheck ./LivingCity        # OOB / illegal memory
cuda-gdb ./LivingCity             # breakpoints, stepping
```

For `cuda-gdb`, recompile with `-G` and `-O` (instead of `-O3`) in `Makefile`. This makes the program substantially slower; only do it for the kernel(s) under investigation.

### Profiling

```bash
nsys profile -o lpsim ./LivingCity         # timeline
ncu --set full -o lpsim ./LivingCity       # per-kernel metrics
# legacy:
nvprof --print-summary ./LivingCity
```

### Benchmarking

```bash
python3 LivingCity/benchmarking/runBenchmarks.py --name=baseline --runs=5
```

Outputs CSV / xls / plot under `LivingCity/benchmarking/`.

---

## Citing LPSim

If LPSim contributes to your work, please cite the relevant paper:

**Multi-GPU traffic assignment (TR-C 2024)**
Jiang, X., Sengupta, R., Demmel, J., & Williams, S. (2024). *Large scale multi-GPU based parallel traffic simulation for accelerated traffic assignment and propagation.* Transportation Research Part C: Emerging Technologies, 169, 104873. [link](https://www.sciencedirect.com/science/article/pii/S0968090X24003942)

**Deep RL for ride-hailing dispatch (ICML 2026)**
Tang, Y., Cui, K., Park, J. H., Zhao, Y., Jiang, X., He, H., Yu, J., Koutsopoulos, H., & Zhao, J. (2026). *RAST-MoE-RL: A Regime-Aware Spatio-Temporal MoE Framework for Deep Reinforcement Learning in Ride-Hailing.* ICML 2026. [arXiv:2512.13727](https://arxiv.org/abs/2512.13727)

**Urban Air Mobility integration (Journal of Air Transportation 2023)**
Jiang, X., Tang, Y., Cao, J., Bulusu, V., Yang, H., Peng, X., Zheng, Y., Zhao, J., & Sengupta, R. (2023). *Simulating Integration of Urban Air Mobility into Existing Transportation Systems: Survey.* [link](https://arc.aiaa.org/doi/10.2514/1.D0431)

**Vertiport siting (TRR 2024)**
Jiang, X., Cao, S., Mo, B., Cao, J., Yang, H., Tang, Y., Hansen, M., Zhao, J., & Sengupta, R. (2024). *Simulation-Based Optimization for Vertiport Location Selection: A Surrogate Model With Machine Learning Method.* Transportation Research Record. [link](https://journals.sagepub.com/doi/full/10.1177/03611981241277755)

---

## Contributors

LPSim is developed and maintained by Xuan Jiang, Jiaying Li, Yibo Zhao, Chonghe Jiang, Xin Peng, Johan Agerup, Yuhan Tang, Hao Zheng, and Raja Sengupta. The simulator is built on top of Pavan Yedavalli's [microsimulation analysis for network traffic assignment](https://scholar.google.com/citations?view_op=view_citation&hl=en&user=HRLwH5oAAAAJ&citation_for_view=HRLwH5oAAAAJ:2osOgNQ5qMEC).

## License

MIT — see [LICENSE](LICENSE).
