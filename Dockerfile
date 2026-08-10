# LPSim: GPU-accelerated multi-GPU traffic microsimulator.
# CUDA 12.8+ is required for a native Blackwell (sm_100) cubin.
# Build: docker build -t lpsim .
# Run:   docker run --gpus all lpsim

ARG LPSIM_CUDA_IMAGE=nvidia/cuda:12.9.1-devel-ubuntu22.04
FROM yibo123/lpsim:cuda12.4 AS lpsim-legacy-dependencies
FROM ${LPSIM_CUDA_IMAGE}

ARG LPSIM_CUDA_ARCHITECTURES=""

# Pandana/CH routing and Boost 1.59 are not distributed by the CUDA image. Reuse
# the project's existing binary-compatible Ubuntu 22.04 dependency bundle.
COPY --from=lpsim-legacy-dependencies /usr/include/pandana /usr/include/pandana
COPY --from=lpsim-legacy-dependencies /usr/local/boost_1_59_0 /usr/local/boost_1_59_0

ENV LD_LIBRARY_PATH=/usr/include/pandana/src:/usr/local/cuda/lib64:/usr/local/nvidia/lib:/usr/local/nvidia/lib64

RUN apt-get update -qq && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y -qq --no-install-recommends \
        build-essential cmake python3-pip qtbase5-dev libqt5opengl5-dev \
        libglew-dev libfontconfig1 mesa-common-dev && \
    rm -rf /var/lib/apt/lists/* && \
    python3 -m pip install --no-cache-dir --upgrade pip setuptools wheel

WORKDIR /lpsim
COPY . .

RUN python3 -m pip install --no-cache-dir -e .

RUN mkdir -p build && cd build && \
    cmake .. -DLPSIM_CUDA_ARCHITECTURES="${LPSIM_CUDA_ARCHITECTURES}" \
        -DCMAKE_CXX_FLAGS="-w" && \
    make -j$(nproc) && \
    if [ -z "${LPSIM_CUDA_ARCHITECTURES}" ]; then \
        python3 ../tools/check_cuda_artifacts.py lpsim \
            --require-real 80 89 90 100 --require-virtual 80 100; \
    else \
        python3 ../tools/check_cuda_artifacts.py lpsim; \
    fi

CMD ["build/lpsim"]
