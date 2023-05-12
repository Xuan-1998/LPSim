# LPSim (Large (Scale) Parallel (Computing) regional traffic Simulation)

LPSim is a discrete time-driven simulation platform that enables microsimulation analysis for network traffic assignment for both cars and aircrafts. Its architecture incorporates a highly parallelized GPU implementation that provides efficient execution of large-scale simulations on demand and networks with hundreds of thousands of nodes and edges, as well as millions of trips. The computational performance of LPSim is assessed by testing the platform to simulate the entire Bay Area metropolitan region during morning hours, utilizing half-second time steps. The runtime for the nine-county Bay Area simulation, excluding routing and initialization, is just over within minutes depending on how many GPUs are available to be used. 


<img width="569" alt="image" src="https://github.com/Xuan-1998/LPSim/assets/58761221/b5fdaa1c-92f1-4d98-b71a-5d136ff28a1d">

The concept of implementing a multi-GPU simulation can be elucidated as follows: initially, the network will undergo partitioning into distinct GPU units, following which, the simulation of individuals will be executed independently within separate GPUs for multiple time-steps, prior to any communication between these subunits.


![mGPU-roadnetwork](https://github.com/Xuan-1998/LPSim/assets/58761221/37743a60-b394-4498-8eff-bdc8a6ab6614)


## Initial checks

```bash
sudo apt update
sudo apt install qtchooser
sudo apt-get install qt5-default
sudo apt-get install libglew-dev
sudo apt-get install build-essential
sudo apt-get install libfontconfig1
sudo apt-get install mesa-common-dev
sudo apt-get install wget
sudo apt-get install pciutils
sudo apt install git
```

## Dependencies

 - Boost 1.59 
 ```bash 
 wget http://sourceforge.net/projects/boost/files/boost/1.59.0/boost_1_59_0.tar.gz
 sudo tar xf boost_1_59_0.tar.gz -C /usr/local
 ```
 
 - CUDA (used versions: 9.0 in Ubuntu) (If you are using Google Cloud Platform, please follow these [instructions](https://cloud.google.com/compute/docs/gpus/install-drivers-gpu#ubuntu-driver-steps))
 - g++ (used versions: 6.4.0 in Ubuntu)
 - Qt5 (used versions: 5.9.5 in Ubuntu)
 - qmake (used versions: 3.1 in Ubuntu)
 - Python (used versions: 3.6.5 in Ubuntu)
 - pytest (used versions: 6.1.1 in Ubuntu) 
 - pytest-cov (used versions: 2.10.1 in Ubuntu) 
 - pytest-remotedata (used versions: 0.3.2 in Ubuntu) 
 - psutil (used versions: 5.7.2 in Ubuntu) 
 - xlwt (used versions: 1.3.0 in Ubuntu)

## Installation & Compilation

### Manual installation

Once the necessary dependencies are installed, you can use the following lines to make sure the
correct versions of each one are used:
```bash
export PATH=/usr/local/cuda-9.0/bin:$PATH
export LIBRARY_PATH=/usr/local/cuda-9.0/lib64:$LIBRARY_PATH 
export LD_LIBRARY_PATH=/usr/local/cuda-9.0/lib64:$LD_LIBRARY_PATH 
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/{YOUR_USERNAME}/pandana/src
```

You can also add the `export` lines at the end of your user's `~/.bashrc` to
avoid re-entering them in each session.

Clone the repo in your home directory with:
```bash
git clone git@github.com:Xuan-1998/LPSim.git ~/LPSim && cd ~/LPSim
```

Clone the [Pandana repository](https://github.com/UDST/pandana) to your home directory stay on the `main` branch, since MANTA now uses a fast contraction hierarchies framework for shortest path routing. Previously implemented shortest path frameworks include Johnson's all pairs shortest path and a parallelized Dijkstra's priority queue.

Create `Makefile` and compile with:
```bash
sudo qmake LivingCity/LivingCity.pro
```

Importantly, because LPSim uses a shared library from Pandana, a Pandana makefile must be created (to create a shared object file) and the LPSim makefile must be modified.

Pandana `Makefile`:

1. Create Makefile in `pandana/src/` containing the following:

```# Makefile for pandana C++ contraction hierarchy library

CC = gcc  # C compiler
CXX = g++
CPPFLAGS = -DLINUX -DMAC -std=c++0x -c -fPIC -g -O3 -Wall -pedantic -fopenmp  # C flags
LDFLAGS = -shared   # linking flags
RM = rm -f   # rm command
TARGET_LIB = libchrouting.so  # target lib

SRCS =  accessibility.cpp graphalg.cpp contraction_hierarchies/src/libch.cpp

OBJS = $(SRCS:.cpp=.o)

.PHONY: all
all: ${TARGET_LIB}

$(TARGET_LIB): $(OBJS)
        $(CXX) ${LDFLAGS} -o $@ $^

.PHONY: clean
clean:
        -${RM} ${TARGET_LIB} ${OBJS}
```
2. Run `make`.

LPSim `Makefile`:

1. Add `-I/home/{YOUR_USERNAME}/pandana/src` to `INCPATH`.
2. Add `-L/home/{YOUR_USERNAME}/pandana/src -lchrouting` to `LIBS`.
3. Run `sudo make -j`.


### Run with Docker

1.  Make sure that you have Docker, its NVidia container toolkit and the necessary permissions:
```bash
sudo apt install docker.io
sudo groupadd docker
sudo usermod -aG docker {YOUR_USERNAME}
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker
sudo apt-get install -y nvidia-container-toolkit
```

2. You can build it yourself and run it
```bash
docker build -t lpsim:latest .
docker run -it --rm --gpus all -v "$PWD":/lpsim -w /lpsim  lpsim:latest bash
```

3. Once inside the container, compile and run
```bash
qmake LivingCity/LivingCity.pro
make
cd LivingCity
./LivingCity
```

## Data

Before running everything, you need the appropriate data:

1. Network
2. Demand

The networks currently reside in `LPSim/LivingCity/berkeley_2018`, and the default directory is the full SF Bay Area network in `new_full_network/`. This contains the `nodes.csv` and `edges.csv` files to create the network.

The demand is not in `new_full_network/`, but needs to reside there in order to run it. Please contact [Pavan Yedavalli](pavyedav@gmail.com) to procure real or sample demands.

## Running

If you wish to edit the microsimulation configuration, modify `LPSim/LivingCity/command_line_options.ini`, which contains the following:

```[General]
GUI=false
USE_CPU=false
NETWORK_PATH=berkeley_2018/new_full_network/
USE_JOHNSON_ROUTING=false
USE_SP_ROUTING=true
USE_PREV_PATHS=true
LIMIT_NUM_PEOPLE=256000
ADD_RANDOM_PEOPLE=false
NUM_PASSES=1
TIME_STEP=0.5
START_HR=5
END_HR=12
```

Here, you can modify the:

1. `GUI` - deprecated. Do not touch.
2. `USE_CPU` - deprecated. Do not touch.
3. `NETWORK_PATH` - specific path to the network files. Default is `berkeley_2018/new_full_network/`.
4. `USE_JOHNSON_ROUTING` - uses Johnson's all pairs shortest path routing. This should always be set to `false`.
5. `USE_SP_ROUTING` - uses new SP routing framework. This should always be set to `true`.
6. `USE_PREV_PATHS` - uses paths already produced and saved to file. Set to `false` if running for the first time. Set to `true` if the simulation was already run and it was saved to file. 
7. `LIMIT_NUM_PEOPLE` - deprecated. Do not touch.
8. `ADD_RANDOM_PEOPLE` - deprecated. Do not touch.
9. `NUM_PASSES` - the number of times the simulation is run. Set to 1.
10. `TIME_STEP` - timestep. Default is .5 seconds.
11. `START_HR` - start hour of the simulation. Default is 5am.
12. `END_HR` - end hour of the simulation. Default is 12pm.

Run with:
```bash
cd LivingCity
./LivingCity
```

## Development

Should you wish to make any changes, please create a new branch. In addition, once the original Makefile is created, you can simply run `sudo make -j` from the `LPSim/` directory to compile any new changes.

If necessary, you can checkout a different existing branch from main (`edge_speeds_over_time`, for instance):
```bash
git checkout edge_speeds_over_time
```

### Debugging
For debugging we recommend `cuda-memcheck ./LivingCity` for out-of-bounds memory bugs in the CUDA section and `cuda-gdb` for more advanced features such as breakpoints.

In order to use `cuda-gdb`, `LPSim/Makefile` must be modified by adding the flag `-G` to enable debugging and changing `-O3` to `-O` to avoid optimizations that restrict the use of the debugger.

For example, to enable debugging at `LivingCity/traffic/b18CUDA_trafficSimulator.cu`,  its compilation at the line `LPSim/Makefile:1756`:
<pre>
/usr/local/cuda-9.0/bin/nvcc -m64 <b>-O3</b> -arch=sm_50 -c --compiler-options -f
no-strict-aliasing -use_fast_math --ptxas-options=-v -Xcompiler -fopenmp -I/u
sr/include/opencv2/ -I/opt/local/include/ -I/usr/local/boost_1_59_0/ -I/home/
<b>{YOUR_USERNAME}</b>/LPSim/LivingCity/glew/include/ -I/usr/local/cuda-9.0/include  -L/opt/l
ocal/lib -lopencv_imgcodecs -lopencv_core -lopencv_imgproc -lcudart -lcuda -g -lgomp
LivingCity/traffic/b18CUDA_trafficSimulator.cu -o
${OBJECTS_DIR}b18CUDA_trafficSimulator_cuda.o
</pre>

must be modified to:
<pre>
/usr/local/cuda-9.0/bin/nvcc -m64 <b>-O</b> -arch=sm_50 -c --compiler-options -f
no-strict-aliasing -use_fast_math --ptxas-options=-v -Xcompiler -fopenmp -I/u
sr/include/opencv2/ -I/opt/local/include/ -I/usr/local/boost_1_59_0/ -I/home/
<b>{YOUR_USERNAME}</b>/LPSim/LivingCity/glew/include/ -I/usr/local/cuda-9.0/include  -L/opt/l
ocal/lib -lopencv_imgcodecs -lopencv_core -lopencv_imgproc -lcudart -lcuda -g <b>-G</b>
-lgomp LivingCity/traffic/b18CUDA_trafficSimulator.cu -o
${OBJECTS_DIR}b18CUDA_trafficSimulator_cuda.o
</pre>

After this modification, `sudo make clean` and `sudo make -j` must be run.

Please keep in mind that this alteration slows the program down. For more information about `cuda-gdb`, please refer to the official [Website](https://docs.nvidia.com/cuda/cuda-gdb/index.html) and [Documentation](https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=&ved=2ahUKEwiBgbqg9fzrAhUMIrkGHby9Db8QFjADegQIAxAB&url=https%3A%2F%2Fdeveloper.download.nvidia.com%2Fcompute%2FDevZone%2Fdocs%2Fhtml%2FC%2Fdoc%2Fcuda-gdb.pdf&usg=AOvVaw3J9Il2vHkkxtcX83EHC3-z).

### Testing
In order to run all tests you should first move to `LPSim/LivingCity`
```bash
cd LivingCity
```
and then run 
```bash
sh runAllTests.sh
```

### Benchmarking / profiling
In order to obtain a profiling of each component of the simulation, you should run
```bash
python3 LivingCity/benchmarking/runBenchmarks.py
```

If you wish to specify the name of the benchmark outputs and/or the number of iterations, just run:
```bash
python3 LivingCity/benchmarking/runBenchmarks.py --name={name_of_benchmark} --runs={number_of_iterations_to_run}
```
The script will run LivingCity the specified number of times while polling the system resources. For each component, its resource and time consumption will be saved into a `csv` file, a plot and a `xls` file in `LPSim/LivingCity/benchmarking/`. The profiling of each version is encouraged to be stored in [here](https://docs.google.com/spreadsheets/d/14KCUY8vLp9HoLuelYC5DmZwKI7aLsiaNFp7e6Z8bVBU/edit?usp=sharing).




## Record GPU usage
nvidia-smi -l 1 >> logfile.txt

## Docker
docker pull your-dockerhub-username/lpsim:latest

docker run -it your-dockerhub-username/lpsim:latest /bin/bash

## multiple GPUs
/usr/local/cuda-11.2/bin/nvcc -m64 -O3 -arch=sm_50 -c --compiler-options -fno-strict-aliasing -use_fast_math --ptxas-options=-v -Xcompiler -fopenmp --expt-relaxed-constexpr -I/usr/include/opencv4/ -I/opt/local/include/ -I/usr/local/boost_1_59_0/ -I/usr/include -I/usr/include/pandana/src -I/usr/local/cuda-11.2/include  -L/opt/local/lib -lopencv_imgcodecs -lopencv_core -lopencv_imgproc -lm -ldl -L/usr/include/pandana/src -lchrouting -lcudart -lcuda -lgomp LivingCity/traffic/b18CUDA_trafficSimulator.cu -o LivingCity/obj/b18CUDA_trafficSimulator_cuda.o

## Running on gcloud
curl https://raw.githubusercontent.com/GoogleCloudPlatform/compute-gpu-installation/main/linux/install_gpu_driver.py --output install_gpu_driver.py

sudo python3 install_gpu_driver.py

## look at GPU usage:

nvidia-smi --loop-ms=1000 --filename=output_gpuusage.txt

while true; do nvidia-smi >> output_gpu.txt; sleep 10; done



## profiling

run docker with:

docker run -it --rm --privileged --gpus all -v "$PWD":/LPSim -w /LPSim gcr.io/blissful-jet-303616/LPSim:latest  bash

then:

nvprof --print-summary  ./LivingCity >>  profile_g3.txt 2>&1 (summary)

nvprof --print-gpu-trace  ./LivingCity >> output_g3.txt 2>&1 (breakdown)

nvprof --metrics flop_count_sp,flop_count_dp ./LivingCity >> output_g3.txt 2>&1  (flop ratio)





## Acknowledgments

This repository and code have been developed and maintained by Xuan Jiang, Xin Peng, Johan Agerup, Emin Burak Onat, Yuhan Tang, and Raja Sengupta. This work is based from Pavan Yedavalli's [Microsimulation analysis for network traffic assignment project](https://scholar.google.com/citations?view_op=view_citation&hl=en&user=HRLwH5oAAAAJ&citation_for_view=HRLwH5oAAAAJ:2osOgNQ5qMEC).

If this code is used in any shape or form for your project, please cite this paper accordingly:

Jiang, X., Agerup, J. F., & Tang, Y. (2023, May 11). Benchmarking and preparing LPSim for scalability on multiple GPUs. Retrieved from osf.io/ezjrc, Available: [https://osf.io/ezjrc/](https://osf.io/ezjrc/).

and 

Jiang, X., Tang, Y., Tang, Z., Cao, J., Bulusu, V., Poliziani, C., & Sengupta, R. (2023). Simulating the Integration of Urban Air Mobility into Existing Transportation Systems: A Survey. Available:  [arXiv preprint arXiv:2301.12901.](https://arxiv.org/abs/2301.12901)

Thank you!







