
# ========== OpenCV subimage ==========
FROM gcr.io/blissful-jet-303616/opencv:v4 AS opencvbuilder
ARG DEBIAN_FRONTEND=noninteractive
# ========== Pandana subimage ==========

FROM ubuntu:20.04 AS pandanabuilder
ARG DEBIAN_FRONTEND=noninteractive

WORKDIR /usr/include/

RUN apt update && \
    apt install -y \
    dialog \
    apt-utils \
    qtchooser \
    qt5-default \
    libglew-dev \
    build-essential \
    libfontconfig1 \
    mesa-common-dev \
    wget \
    pciutils \
    git 

RUN git clone https://github.com/UDST/pandana

COPY /PandanaMakefile /usr/include/pandana/src/Makefile

WORKDIR /usr/include/pandana/src

RUN make

# ========== MANTA image ==========
FROM nvidia/cuda:12.3.1-devel-ubuntu20.04 as mantabuilder
ARG DEBIAN_FRONTEND=noninteractive
COPY --from=opencvbuilder /usr/include/opencv4/ /usr/include/opencv4/

COPY --from=opencvbuilder /usr/lib/x86_64-linux-gnu/libopencv_core.* /usr/lib/x86_64-linux-gnu/
COPY --from=opencvbuilder /usr/lib/x86_64-linux-gnu/libopencv_imgproc.* /usr/lib/x86_64-linux-gnu/
COPY --from=opencvbuilder /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.* /usr/lib/x86_64-linux-gnu/

COPY --from=pandanabuilder /usr/include/pandana/ /usr/include/pandana/

# libraries
RUN apt update && \
    apt install qtchooser \
    qt5-default \
    libglew-dev \
    build-essential \
    libfontconfig1 \
    mesa-common-dev \
    wget \
    pciutils -y

# boost
RUN wget http://sourceforge.net/projects/boost/files/boost/1.59.0/boost_1_59_0.tar.gz && \
    tar xf boost_1_59_0.tar.gz -C /usr/local

# CUDA paths
ENV PATH="/usr/local/cuda-12.3.1/bin:${PATH}"
ENV LIBRARY_PATH="/usr/local/cuda-12.3.1/lib64:${LIBRARY_PATH}"
ENV LD_LIBRARY_PATH="/usr/local/cuda-12.3.1/lib64:${LD_LIBRARY_PATH}"

# Pandana path
ENV LD_LIBRARY_PATH="/usr/include/pandana/src:${LD_LIBRARY_PATH}"

# Python libraries
RUN apt update && apt install python3-pip -y
RUN apt install gdb cuda-nsight-systems-12-3 cuda-nsight-compute-12-3 -y

ADD . ./

RUN pip3 install -r requirements.txt


# Check if CUDA is properly installed
CMD nvidia-smi
