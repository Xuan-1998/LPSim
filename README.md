# LS-PTS (Large Scale - Parallel Traffic Simulator)
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

## b18CommandLineVersion.cpp
The header file includes several types and functions from the b18CommandLineVersion, benchmarker, roadGraphB2018Loader, graph, b18TrafficSP, accessibility, and b18TestSimpleRoadAndOD libraries/modules. It also includes the QString, QCoreApplication, and stdexcept headers from the Qt and C++ standard libraries.

The header file appears to be related to a GUI version of the software, as it has a B18_RUN_WITH_GUI preprocessor directive that includes a b18TestSimpleRoadAndOD header file if it is defined.

a command-line program that simulates traffic on a road network. The program reads several configuration options from an INI file and uses these options to control the behavior of the simulation.

The B18CommandLineVersion class has a runB18Simulation method that is responsible for setting up and running the simulation. The method reads the configuration options from the INI file using a QSettings object and stores them in local variables. It then performs some input validation and sets default values for any options that are not present in the INI file.

Next, the method loads the road network from a file specified in the INI file and sets up a traffic simulation based on the chosen routing algorithm (either shortest path or Johnson's algorithm). It also sets up an origin-destination demand matrix based on a CSV file specified in the INI file.

Finally, the method runs the simulation for a specified number of passes, each time updating the positions of all people in the simulation, collecting statistics about the traffic, and rerouting people as needed. The method also has several other features, such as the ability to run unit tests and show performance benchmarks.

## b18TrafficSimulator.cpp
The header file includes types and functions from the b18TrafficSimulator, benchmarker, global, LC_GLWidget3D, LC_UrbanMain, b18TrafficDijkstra, b18TrafficJohnson, b18TrafficSP, b18CUDA_trafficSimulator, roadGraphB2018Loader, and accessibility libraries/modules. It also includes the thread, unistd, and windows headers from the C++ standard library and the math.h header.

The header file also defines several preprocessor directives, such as DEBUG_TRAFFIC, DEBUG_SIMULATOR, and DEBUG_T_LIGHT, which appear to be used for debugging purposes. There is also a printPercentageMemoryUsed function that appears to be platform-specific and is used to print the percentage of memory being used by the program.

The B18TrafficSimulator class is a class for simulating traffic in a city. It has a member variable deltaTime which represents the time step of the simulation, and simParameters which is a struct containing various parameters for the simulation such as the start and end times of the simulation and the number of people to be simulated. The class also has a member variable b18TrafficOD which is an instance of the B18TrafficOD class, which is responsible for handling the generation of traffic demand and origin-destination (OD) information for the simulation. The class also has a pointer simRoadGraph to a RoadGraph object, which is a representation of the city's road network, and a pointer clientMain to an instance of the LCUrbanMain class, which is a class for the main application. The class has a destructor which deletes the simRoadGraph object.

The function B18TrafficSimulator::createRandomPeople generates random people with a job, that is, they need to travel from a starting location to an ending location within the specified time interval (startTime and endTime). The number of people generated is given by numberPeople. The locations and jobs of these people are stored in the peopleJobInfoLayers object.

The function B18TrafficSimulator::createB2018People loads people with a job from a file, with the specified time interval (startTime and endTime). The number of people loaded is limited by limitNumPeople. If addRandomPeople is true, additional random people are generated.

The function B18TrafficSimulator::createB2018PeopleSP is similar to createB2018People, but it loads people using the shortest path algorithm (SP). It also takes in a graph object graph_ and a vector of departure times dep_times as additional arguments.

The function B18TrafficSimulator::resetPeopleJobANDintersections resets the job and location information for all the people in the simulation, and also resets the intersections and traffic lights.

The function B18TrafficSimulator::createLaneMap creates a lane map for the simulation, using the road graph and additional data such as the edges data and intersections. It also creates mappings between lane map numbers and edge descriptions, and vice versa.

The function B18TrafficSimulator::createLaneMapSP is similar to createLaneMap, but it creates a lane map using the shortest path algorithm (SP). It also takes in a graph object graph_ as an additional argument.

The function B18TrafficSimulator::generateCarPaths generates the paths for all the cars in the simulation, using either the Johnson or Dijkstra routing algorithm. If useJohnsonRouting is true, the Johnson algorithm is used, otherwise the Dijkstra algorithm is used.

The function B18TrafficSimulator::simulateInGPU simulates the traffic in a GPU. It takes in several parameters:

numOfPasses: an integer representing the number of passes to run the simulation.

startTimeH: a float representing the starting time of the simulation in hours.

endTimeH: a float representing the ending time of the simulation in hours.

useJohnsonRouting: a boolean indicating whether to use Johnson routing algorithm or not.

useSP: a boolean indicating whether to use shortest path routing algorithm or not.

graph_: a pointer to the graph used in the simulation.

simParameters: a struct of type parameters containing the simulation parameters.

rerouteIncrementMins: an integer representing the reroute increment in minutes.

all_od_pairs: a vector of pairs of vertexes representing origin-destination pairs.

dep_times: a vector of floats representing the departure times.

networkPathSP: a string representing the path to the network file for shortest path algorithm.

The function first creates the lane map, either using the createLaneMap function for non-shortest path routing, or the createLaneMapSP function for shortest path routing. It then generates routes for the traffic people using either the Johnson routing algorithm or the Dijkstra routing algorithm (or the shortest path algorithm if useSP is true). It then outputs the edges, vertices, and edges indices to files and calls the simulateTrafficGPU function to simulate the traffic in the GPU.

The simulateOnePersonCPU function simulates the movement of a single person in a traffic simulation. It takes in a number of arguments, including the current time and delta time of the simulation, data about the traffic person being simulated, information about the road network and intersections, and simulation parameters.

The function first checks if the person is currently active (i.e., if their departure time has arrived or if they are already in the process of moving). If they are not active, the function returns.

If the person is active, the function gets the current edge and lane that the person is on, as well as the traffic light state at the intersection they are approaching. It then calculates the gaps to the vehicles in front and behind the person on their current lane, as well as the speeds of those vehicles.

Based on these gaps and speeds, as well as the person's own speed and acceleration, the function updates the person's position and velocity. If the person has reached their destination, it sets their active status to false. If the person is approaching an intersection, the function checks if they should change lanes or turn at the intersection. If the person needs to change lanes or turn, it updates their lane and edge information accordingly. Finally, the function updates the lane map to reflect the person's new position.

The simulateOneSTOPIntersectionCPU function simulates the behavior of a traffic intersection where the traffic lights are replaced with STOP signs. It does this by checking if there are any cars stopped at the edge of the intersection (the edge leading into the intersection). If there are, the function sets the traffic light for that edge to a STOP sign (0x0F). If there are no cars stopped at the edge of the intersection, the function moves on to the next edge. The simulateOneIntersectionCPU function simulates a traffic intersection with traffic lights. It does this by setting the traffic light for the current edge (determined by the intersections[i].state variable) to green (0xFF) and setting the traffic light for all other edges to red (0x00). The simulateOneIntersectionCPU function also advances the intersections[i].state variable to the next edge and sets the intersections[i].nextEvent variable to the current time plus a fixed time interval (deltaEvent). This causes the traffic lights to change after a fixed time interval has passed. The simulateOneIntersectionCPU function is called for all intersections in the simulation.

## b18TrafficSP.cpp
It is defining a function make_od_pairs which is used to convert a list of traffic persons (trafficPersonVec) into a list of pairs of vertex indices (od_pairs).

The function takes two arguments:

trafficPersonVec: A vector of B18TrafficPerson structures, which represent the individuals in the traffic simulation. Each B18TrafficPerson has an initial intersection and an end intersection, which are represented as vertex indices in the road graph.
nagents: An integer representing the number of agents (traffic persons) to consider. If this value is std::numeric_limits<int>::max(), then all traffic persons in trafficPersonVec will be considered.
The function first initializes an empty vector of vertex pairs od_pairs. It then loops through each traffic person in trafficPersonVec, and adds the initial and end intersection indices as a pair to od_pairs. If nagents is not equal to std::numeric_limits<int>::max(), the function resizes od_pairs to have only nagents elements. It then sorts od_pairs and returns it.
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







