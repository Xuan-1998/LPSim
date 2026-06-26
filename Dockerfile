# LPSim: GPU-accelerated multi-GPU traffic microsimulator
# Build: docker build -t lpsim .
# Run:   docker run --gpus all lpsim

FROM yibo123/lpsim:cuda12.4

WORKDIR /lpsim
COPY . .

RUN apt-get update -qq && \
    apt-get install -y -qq cmake python3-pip && \
    rm -rf /var/lib/apt/lists/* && \
    pip3 install --no-cache-dir -e . && \
    mkdir -p build && cd build && \
    cmake .. -DCMAKE_CUDA_ARCHITECTURES="80;89;90" -DCMAKE_CXX_FLAGS="-w" && \
    make -j$(nproc)

CMD ["build/lpsim"]
