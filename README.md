# LPSim (Large (Scale) Parallel (Computing) metropolitan scale traffic Simulation)
## Record GPU usage
nvidia-smi -l 1 >> logfile.txt

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

docker run -it --rm --privileged --gpus all -v "$PWD":/manta -w /manta gcr.io/blissful-jet-303616/manta:latest  bash

then:

nvprof --print-summary  ./LivingCity >>  profile_g3.txt 2>&1 (summary)

nvprof --print-gpu-trace  ./LivingCity >> output_g3.txt 2>&1 (breakdown)

nvprof --metrics flop_count_sp,flop_count_dp ./LivingCity >> output_g3.txt 2>&1  (flop ratio)



## b18CUDA_trafficSimulator.cu
The program appears to include several user-defined header files, such as "b18TrafficPerson.h" and "b18EdgeData.h". It also includes the header file "curand_kernel.h", which is part of the CUDA Random Number Generation (CURAND) library. The CURAND library provides functions for generating high-quality random numbers on the GPU.

The program uses a number of preprocessor directives, such as "#define" and "#ifndef", to define various macros and to conditionally include or exclude code. These directives are used to make the code more portable and easier to maintain.

The constant "MINIMUM_NUMBER_OF_CARS_TO_MEASURE_SPEED" appears to be used to specify the minimum number of cars required in order to measure their speed.

The constant "intersectionClearance" is declared using the constant keyword, which indicates that it is a constant memory variable in CUDA. Constant memory variables are stored in a special memory space on the GPU that is faster to access than global memory, but has a smaller capacity. Constant memory is typically used for data that is read-only and is accessed frequently by the GPU threads.

The function "gpuErrchk" is a macro that checks the error code returned by a CUDA function and prints an error message if the code indicates an error. The macro expands to a call to the "gpuAssert" function, which takes the error code, the name of the source file, and the line number as arguments, and prints an error message including these values if the error code indicates an error.

The function "printMemoryUsage" prints information about the memory usage of the GPU, including the amount of used, free, and total memory. It does this by calling the CUDA function "cudaMemGetInfo", which returns the amount of free and total memory available on the GPU. The function then calculates the used memory by subtracting the free memory from the total memory, and prints the results in megabytes.

trafficPersonVec_d is a vector of traffic persons, indexPathVec_d is a vector of indices for paths, edgesData_d is a vector of edge data, laneMap_d is a lane map, trafficLights_d is a vector of traffic lights, accSpeedPerLinePerTimeInterval_d is a vector of accumulated speeds per line per time interval, and numVehPerLinePerTimeInterval_d is a vector of the number of vehicles per line per time interval.

calculatePollution is a constant boolean that appears to be related to whether pollution should be calculated in the simulation. cellSize is a constant float representing the size of a cell in the simulation.

readFirstMapC, mapToReadShift, mapToWriteShift, and halfLaneMap are variables that seem to be related to reading and writing maps. startTime is a float representing the start time of the simulation.

intersections_d is a vector of intersection data.

The function "b18InitCUDA" is to initialize the CUDA environment for a traffic simulation. It takes a number of input vectors as arguments, including trafficPersonVec, indexPathVec, edgesData, laneMap, trafficLights, and intersections. It also takes a start time, an end time, and a time step size as input.

The function begins by allocating device memory for each of the input vectors and copying the data from the host to the device. It also calculates some other constants and variables that will be used in the simulation, such as numStepsPerSample and numStepsTogether.

Finally, the function allocates device memory for two additional vectors: accSpeedPerLinePerTimeInterval_d and numVehPerLinePerTimeInterval_d, and copies the corresponding data from the host to the device. These vectors will be used to store accumulated speeds and number of vehicles, respectively, per line per time interval during the simulation.

The calculateGapsLC function appears to be responsible for calculating gaps between vehicles on a lane. It takes a number of input arguments, including a map of lanes, a traffic light state, a lane to check, the number of lines on an edge, the position to check in meters, the length of the lane, and some output variables for the 'speed' and gap of the nearest vehicles in front and behind the vehicle of interest.

The calculateLaneCarShouldBe function appears to be responsible for calculating which lane a vehicle should be on based on its current lane, the next edge it will be traveling on, and some other information about the intersection. It takes a number of input arguments, including the current edge lane, the next edge, information about the intersection, the number of lanes on the edge, and some output variables for the initial and ending acceptable lanes. The function appears to be based on a clockwise search through the edges of the intersection, looking for the current edge and an exit edge. It calculates the number of exits to take and the number of exits taken, and uses these values to determine the initial and ending acceptable lanes for the vehicle.

The meters_per_second_to_miles_per_hour function converts a velocity in meters per second to miles per hour. The calculateCOStep function appears to calculate the carbon monoxide emission rate based on the velocity of a person, using a formula from a research paper. The calculateGasConsumption function appears to calculate the gas consumption of a vehicle based on its acceleration and velocity, using a formula from a research paper.

It looks like these functions are intended to be used in a kernel that executes on the CUDA device, as indicated by the __device__ keyword before each function declaration. This indicates that the functions will be executed on the device rather than the host, and that they have been optimized for execution on the device.

kernel_intersectionOneSimulation: This kernel appears to be responsible for updating the state of traffic lights at intersections. It does this by iterating over all intersections in the simulation and checking if it is time to update the traffic light at that intersection. If it is, the kernel sets the traffic lights at the outgoing edge of the intersection to red, and the traffic lights at the incoming edge to green.

kernel_sampleTraffic: This kernel appears to be responsible for collecting statistics about the traffic in the simulation. It does this by iterating over all the people in the simulation and, if they are active (i.e., currently driving), updating a running total of the speed and number of vehicles on the edge of the road they are currently on.

kernel_resetPeople: This kernel appears to be responsible for resetting the state of all people in the simulation. It does this by iterating over all the people in the simulation and setting their active flag to 0.

There is also a function called b18GetSampleTrafficCUDA that appears to be responsible for launching these kernels on the GPU. It takes several arguments, including vectors that hold data about the intersections, traffic lights, people, and paths in the simulation, as well as several scalars that hold various parameters used in the simulation.


# manta

Microsimulation Analysis for Network Traffic Assignment. MANTA employs a highly parallelized GPU implementation that is fast enough to run simulations on large-scale demand and networks within a few minutes - metropolitan and regional scale with hundreds of thousands of nodes and edges and millions of trips. We test our platform to simulate the entire Bay Area metropolitan region over the course of the morning using half-second time steps. The runtime for the nine-county Bay Area simulation is just over four minutes, not including routing and initialization. This computational performance significantly improves state of the art in large-scale traffic microsimulation and offers new capacity for analyzing the detailed travel patterns and travel choices of individuals for infrastructure planning and emergency management.

![](https://github.com/UDST/manta/blob/main/bay_bridge_trips.png)

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
git clone git@github.com:udst/manta.git ~/manta && cd ~/manta
```

Clone the [Pandana repository](https://github.com/UDST/pandana) to your home directory stay on the `main` branch, since MANTA now uses a fast contraction hierarchies framework for shortest path routing. Previously implemented shortest path frameworks include Johnson's all pairs shortest path and a parallelized Dijkstra's priority queue.

Create `Makefile` and compile with:
```bash
sudo qmake LivingCity/LivingCity.pro
```

Importantly, because MANTA uses a shared library from Pandana, a Pandana makefile must be created (to create a shared object file) and the MANTA makefile must be modified.

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

MANTA `Makefile`:

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

2. You can either pull and run our built image
```bash
docker pull gcr.io/blissful-jet-303616/manta:latest
docker run -it --rm --gpus all -v "$PWD":/manta -w /manta gcr.io/blissful-jet-303616/manta:latest  bash
```
Or build it yourself
```bash
docker build -t manta:latest .
docker run -it --rm --gpus all -v "$PWD":/manta -w /manta manta:latest bash
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

The networks currently reside in `manta/LivingCity/berkeley_2018`, and the default directory is the full SF Bay Area network in `new_full_network/`. This contains the `nodes.csv` and `edges.csv` files to create the network.

The demand is not in `new_full_network/`, but needs to reside there in order to run it. Please contact [Pavan Yedavalli](pavyedav@gmail.com) to procure real or sample demands.

## Running

If you wish to edit the microsimulation configuration, modify `manta/LivingCity/command_line_options.ini`, which contains the following:

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

Should you wish to make any changes, please create a new branch. In addition, once the original Makefile is created, you can simply run `sudo make -j` from the `manta/` directory to compile any new changes.

If necessary, you can checkout a different existing branch from main (`edge_speeds_over_time`, for instance):
```bash
git checkout edge_speeds_over_time
```

### Debugging
For debugging we recommend `cuda-memcheck ./LivingCity` for out-of-bounds memory bugs in the CUDA section and `cuda-gdb` for more advanced features such as breakpoints.

In order to use `cuda-gdb`, `manta/Makefile` must be modified by adding the flag `-G` to enable debugging and changing `-O3` to `-O` to avoid optimizations that restrict the use of the debugger.

For example, to enable debugging at `LivingCity/traffic/b18CUDA_trafficSimulator.cu`,  its compilation at the line `manta/Makefile:1756`:
<pre>
/usr/local/cuda-9.0/bin/nvcc -m64 <b>-O3</b> -arch=sm_50 -c --compiler-options -f
no-strict-aliasing -use_fast_math --ptxas-options=-v -Xcompiler -fopenmp -I/u
sr/include/opencv2/ -I/opt/local/include/ -I/usr/local/boost_1_59_0/ -I/home/
<b>{YOUR_USERNAME}</b>/manta/LivingCity/glew/include/ -I/usr/local/cuda-9.0/include  -L/opt/l
ocal/lib -lopencv_imgcodecs -lopencv_core -lopencv_imgproc -lcudart -lcuda -g -lgomp
LivingCity/traffic/b18CUDA_trafficSimulator.cu -o
${OBJECTS_DIR}b18CUDA_trafficSimulator_cuda.o
</pre>

must be modified to:
<pre>
/usr/local/cuda-9.0/bin/nvcc -m64 <b>-O</b> -arch=sm_50 -c --compiler-options -f
no-strict-aliasing -use_fast_math --ptxas-options=-v -Xcompiler -fopenmp -I/u
sr/include/opencv2/ -I/opt/local/include/ -I/usr/local/boost_1_59_0/ -I/home/
<b>{YOUR_USERNAME}</b>/manta/LivingCity/glew/include/ -I/usr/local/cuda-9.0/include  -L/opt/l
ocal/lib -lopencv_imgcodecs -lopencv_core -lopencv_imgproc -lcudart -lcuda -g <b>-G</b>
-lgomp LivingCity/traffic/b18CUDA_trafficSimulator.cu -o
${OBJECTS_DIR}b18CUDA_trafficSimulator_cuda.o
</pre>

After this modification, `sudo make clean` and `sudo make -j` must be run.

Please keep in mind that this alteration slows the program down. For more information about `cuda-gdb`, please refer to the official [Website](https://docs.nvidia.com/cuda/cuda-gdb/index.html) and [Documentation](https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=&ved=2ahUKEwiBgbqg9fzrAhUMIrkGHby9Db8QFjADegQIAxAB&url=https%3A%2F%2Fdeveloper.download.nvidia.com%2Fcompute%2FDevZone%2Fdocs%2Fhtml%2FC%2Fdoc%2Fcuda-gdb.pdf&usg=AOvVaw3J9Il2vHkkxtcX83EHC3-z).

### Testing
In order to run all tests you should first move to `manta/LivingCity`
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
The script will run LivingCity the specified number of times while polling the system resources. For each component, its resource and time consumption will be saved into a `csv` file, a plot and a `xls` file in `manta/LivingCity/benchmarking/`. The profiling of each version is encouraged to be stored in [here](https://docs.google.com/spreadsheets/d/14KCUY8vLp9HoLuelYC5DmZwKI7aLsiaNFp7e6Z8bVBU/edit?usp=sharing).

Versions correspond to [the repository's tags](https://github.com/UDST/manta/tags). In order to create a new tag, just run
```bash
git tag v0.x.0
git push --tags
```


## Acknowledgments

This repository and code have been developed and maintained by Pavan Yedavalli, Ignacio Garcia Dorado, Krishna Kumar, and Paul Waddell. This work heavily derives from Ignacio Garcia Dorado's [Automatic Urban Modeling project](http://www.ignaciogarciadorado.com/p/2014_EG/2014_EG.html).

If this code is used in any shape or form for your project, please cite this paper accordingly:

P. Yedavalli, K. Kumar, and P. Waddell, “Microsimulation Analysis for Network Traffic Assignment (MANTA) at Metropolitan-Scale for Agile Transportation Planning,” arXiv:2007.03614 [physics], Jul. 2020, Available: http://arxiv.org/abs/2007.03614.

Thank you!







