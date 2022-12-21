# ========== OpenCV subimage ==========
FROM gcr.io/blissful-jet-303616/opencv:v4 AS opencvbuilder

# ========== Pandana subimage ==========
FROM ubuntu:18.04 AS pandanabuilder

WORKDIR /usr/include/

RUN apt update && \
    apt install -y qtchooser \
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
FROM nvidia/cuda:11.2.0-devel-ubuntu18.04 as mantabuilder

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
ENV PATH="/usr/local/cuda-11.2/bin:${PATH}"
ENV LIBRARY_PATH="/usr/local/cuda-11.2/lib64:${LIBRARY_PATH}"
ENV LD_LIBRARY_PATH="/usr/local/cuda-11.2/lib64:${LD_LIBRARY_PATH}"

# Pandana path
ENV LD_LIBRARY_PATH="/usr/include/pandana/src:${LD_LIBRARY_PATH}"

# Python libraries
RUN apt install python3-pip -y

ADD . ./

RUN pip3 install -r requirements.txt


# Check if CUDA is properly installed
CMD nvidia-smi
