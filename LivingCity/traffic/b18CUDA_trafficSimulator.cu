


//CUDA CODE
#include <stdio.h>
#include "cuda_runtime.h"
#include "curand_kernel.h"
#include "device_launch_parameters.h"
#include "assert.h"

#include "b18TrafficPerson.h"
#include "b18EdgeData.h"
#include <vector>
#include <iostream>
#include <cstring>

#include "../../src/benchmarker.h"
#include "sp/config.h"

#ifndef ushort
#define ushort uint16_t
#endif
#ifndef uint
#define uint uint32_t
#endif
#ifndef uchar
#define uchar uint8_t
#endif

///////////////////////////////
// CONSTANTS

#define MINIMUM_NUMBER_OF_CARS_TO_MEASURE_SPEED 5
#define ngpus 2
__constant__ float intersectionClearance = 7.8f; //TODO(pavan): WHAT IS THIS?

#define gpuErrchk(ans) { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char *file, int line, bool abort = true) {
  if (code != cudaSuccess) {
    fprintf(stderr, "GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
    if (abort) exit(code);
  }
}
inline void printMemoryUsage() {
  // show memory usage of GPU
  size_t free_byte;
  size_t total_byte;
  cudaError_t cuda_status = cudaMemGetInfo(&free_byte, &total_byte);
  if (cudaSuccess != cuda_status) {
    printf("Error: cudaMemGetInfo fails, %s \n", cudaGetErrorString(cuda_status));
    exit(1);
  }
  double free_db = (double) free_byte;
  double total_db = (double) total_byte;
  double used_db = total_db - free_db;
  printf("GPU memory usage: used = %.0f, free = %.0f MB, total = %.0f MB\n", used_db / 1024.0 / 1024.0, free_db / 1024.0 / 1024.0, total_db / 1024.0 / 1024.0);
}
////////////////////////////////
// VARIABLES on device(s)
// intermediate variable for each gpu?
LC::B18TrafficPerson *trafficPersonVec_d;
// GPU i traffic person vector, i in (0, ..., ngpus)
LC::B18TrafficPerson **trafficPersonVec_d_gpus = new LC::B18TrafficPerson*[ngpus];
int num_people_gpu;
uint **indexPathVec_d = new uint*[ngpus];
uint indexPathVec_d_size;
LC::B18EdgeData **edgesData_d = new LC::B18EdgeData*[ngpus];
uint edgesData_d_size;
uint laneMap_d_size;
uint trafficLights_d_size;
uint accSpeedPerLinePerTimeInterval_d_size;
uint numVehPerLinePerTimeInterval_d_size;
size_t size_gpu_part[ngpus]; 

__constant__ bool calculatePollution = true;
__constant__ float cellSize = 1.0f;

uchar **laneMap_d = new uchar*[ngpus];
bool readFirstMapC=true;
uint mapToReadShift;
uint mapToWriteShift;
uint halfLaneMap;
float startTime;


LC::B18IntersectionData **intersections_d  = new LC::B18IntersectionData*[ngpus];
uchar **trafficLights_d = new uchar*[ngpus];

float* accSpeedPerLinePerTimeInterval_d;
float* numVehPerLinePerTimeInterval_d;

void b18InitCUDA(
  bool firstInitialization,
  std::vector<LC::B18TrafficPerson>& trafficPersonVec, 
  std::vector<uint> &indexPathVec, 
  std::vector<LC::B18EdgeData>& edgesData, 
  std::vector<uchar>& laneMap, 
  std::vector<uchar>& trafficLights, 
  std::vector<LC::B18IntersectionData>& intersections,
  float startTimeH, float endTimeH,
  std::vector<float>& accSpeedPerLinePerTimeInterval,
  std::vector<float>& numVehPerLinePerTimeInterval,
  float deltaTime) {
  cudaStream_t streams[ngpus];
  for(int i = 0; i < ngpus; i++){
      cudaStreamCreate( &streams[i]);
  }
  //printf(">>b18InitCUDA firstInitialization %s\n", (firstInitialization?"INIT":"ALREADY INIT"));
  //printMemoryUsage();
  const uint numStepsPerSample = 30.0f / deltaTime; //each min
  const uint numStepsTogether = 12; //change also in density (10 per hour)
  { // people
    size_t size = trafficPersonVec.size() * sizeof(LC::B18TrafficPerson);
    // if (firstInitialization) gpuErrchk(cudaMalloc((void **) &trafficPersonVec_d, size));   // Allocate array on device

    // gpuErrchk(cudaMemcpy(trafficPersonVec_d, trafficPersonVec.data(), size, cudaMemcpyHostToDevice));
    if (firstInitialization){
      gpuErrchk(cudaMallocManaged(&trafficPersonVec_d, size));
      memcpy(trafficPersonVec_d, trafficPersonVec.data(), size);
    }
    // cudaSetDevice(0);
    // gpuErrchk(cudaMemPrefetchAsync(trafficPersonVec_d, size, 0, streams[0]));
    // cudaSetDevice(1);
    // gpuErrchk(cudaMemPrefetchAsync(trafficPersonVec_d, size, 1, streams[1]));

    // Calculate the size of each half
    num_people_gpu = int(trafficPersonVec.size() / ngpus);
    for(int i = 0; i < ngpus; i++){
        size_gpu_part[i] = num_people_gpu * sizeof(LC::B18TrafficPerson);
    }
    size_gpu_part[ngpus-1] = (trafficPersonVec.size() - num_people_gpu *(ngpus-1)) * sizeof(LC::B18TrafficPerson);

    // Allocate memory for each half on the respective GPU
    //LC::B18TrafficPerson **trafficPersonVec_d_gpus[ngpus];

    // Copy the first half to GPU 0 and the second half to GPU 1
    for(int i = 0; i < ngpus; i++){
      gpuErrchk(cudaSetDevice(i));
      gpuErrchk(cudaMallocManaged(&trafficPersonVec_d_gpus[i], size_gpu_part[i]));
      // trafficPersonVec.data() returns a pointer to the memory of the data of the struct object
      // struct supports plain assignment
      for(int j = 0; j < size_gpu_part[i]/sizeof(LC::B18TrafficPerson); j++){
        trafficPersonVec_d_gpus[i][j] = trafficPersonVec[i * num_people_gpu + j]; 
      }
      gpuErrchk(cudaMemPrefetchAsync(trafficPersonVec_d_gpus[i], size_gpu_part[i], i, streams[i]));
    }
  }
  { 
    for(int i = 0; i < ngpus; i++){
      gpuErrchk(cudaSetDevice(i));
      // indexPathVec
      size_t sizeIn = indexPathVec.size() * sizeof(uint);
      indexPathVec_d_size = indexPathVec.size();
      if (firstInitialization) {
        gpuErrchk(cudaMalloc(&indexPathVec_d[i], sizeIn));   // Allocate array on device
        gpuErrchk(cudaMemcpyAsync(indexPathVec_d[i], indexPathVec.data(), sizeIn, cudaMemcpyHostToDevice, streams[i]));
      }
    }
  }
  {
    for(int i = 0; i < ngpus; i++){
      cudaSetDevice(i);
      //edgeData
      size_t sizeD = edgesData_d_size * sizeof(LC::B18EdgeData);
      edgesData_d_size = edgesData.size();
      if (firstInitialization){
        gpuErrchk(cudaMalloc((void **) &edgesData_d[i], sizeD));   // Allocate array on device
        gpuErrchk(cudaMemcpyAsync(edgesData_d[i], edgesData.data(), sizeD, cudaMemcpyHostToDevice, streams[i]));
      } 
    }
  }
  {//laneMap
    for(int i = 0; i < ngpus; i++){
      cudaSetDevice(i);

      size_t sizeL = laneMap.size() * sizeof(uchar);
      laneMap_d_size = laneMap.size();
      if (firstInitialization) 
      {
        gpuErrchk(cudaMalloc((void **) &laneMap_d[i], sizeL));   // Allocate array on device
        gpuErrchk(cudaMemcpyAsync(laneMap_d[i], laneMap.data(), sizeL, cudaMemcpyHostToDevice, streams[i]));
      }
    }    
  }

  {// intersections
    for(int i = 0; i < ngpus; i++){
      cudaSetDevice(i);
      size_t sizeI = intersections.size() * sizeof(LC::B18IntersectionData);
      if (firstInitialization){
        gpuErrchk(cudaMalloc((void **) &intersections_d[i], sizeI));   // Allocate array on device
        gpuErrchk(cudaMemcpyAsync(intersections_d[i], intersections.data(), sizeI, cudaMemcpyHostToDevice, streams[i]));
      }
      
      size_t sizeT = trafficLights.size() * sizeof(uchar);//total number of lanes
      trafficLights_d_size = trafficLights.size();
      if (firstInitialization) {
        gpuErrchk(cudaMalloc((void **) &trafficLights_d[i], sizeT));   // Allocate array on device
        gpuErrchk(cudaMemcpyAsync(trafficLights_d[i], trafficLights.data(), sizeT, cudaMemcpyHostToDevice, streams[i]));
      }
    }
      // cudaSetDevice(0);
      // gpuErrchk(cudaMemPrefetchAsync(trafficLights_d, sizeT, 0, streams[0]));
      // cudaSetDevice(1);
      // gpuErrchk(cudaMemPrefetchAsync(trafficLights_d, sizeT, 1, streams[1]));

  }
  {
    for(int i = 0; i < ngpus; i++){
      cudaSetDevice(i);

      startTime = startTimeH * 3600.0f;
      uint numSamples = ceil(((endTimeH*3600.0f - startTimeH*3600.0f) / (deltaTime * numStepsPerSample * numStepsTogether))) + 1; //!!!
      accSpeedPerLinePerTimeInterval.clear();
      numVehPerLinePerTimeInterval.clear();
      accSpeedPerLinePerTimeInterval.resize(numSamples * trafficLights.size());
      numVehPerLinePerTimeInterval.resize(numSamples * trafficLights.size());
      size_t sizeAcc = accSpeedPerLinePerTimeInterval.size() * sizeof(float);
      if (firstInitialization)
      {
        gpuErrchk(cudaMalloc((void **) &accSpeedPerLinePerTimeInterval_d, sizeAcc));   // Allocate array on device
      }


      if (firstInitialization) {
        gpuErrchk(cudaMalloc((void **) &numVehPerLinePerTimeInterval_d, sizeAcc));   // Allocate array on device
      }
      
      gpuErrchk(cudaMemset(&accSpeedPerLinePerTimeInterval_d[0], 0, sizeAcc));
      gpuErrchk(cudaMemset(&numVehPerLinePerTimeInterval_d[0], 0, sizeAcc));
      accSpeedPerLinePerTimeInterval_d_size = sizeAcc;
      numVehPerLinePerTimeInterval_d_size = sizeAcc;
    }  
  }
  for(int i = 0; i < ngpus; i++){
    cudaSetDevice(i);
    printMemoryUsage();
  }
}

void b18updateStructuresCUDA(std::vector<LC::B18TrafficPerson>& trafficPersonVec,std::vector<uint> &indexPathVec,std::vector<LC::B18EdgeData>& edgesData){
  std::cout<< ">> b18updateStructuresCUDA" << std::endl;
  //indexPathVec
  cudaStream_t streams[ngpus];
  size_t sizeIn = indexPathVec.size() * sizeof(uint);
  indexPathVec_d_size = indexPathVec.size();
  size_t sizeD = edgesData.size() * sizeof(LC::B18EdgeData);
  size_t size = trafficPersonVec.size() * sizeof(LC::B18TrafficPerson);
  for(int i=0; i < ngpus; i++){
    cudaSetDevice(i);
    cudaStreamCreate( &streams[i] );
    // copy index path vector 
    cudaFree(indexPathVec_d[i]);
    gpuErrchk(cudaMalloc((void **) &indexPathVec_d[i], sizeIn));
    gpuErrchk(cudaMemcpyAsync(indexPathVec_d[i], indexPathVec.data(), sizeIn, cudaMemcpyHostToDevice, streams[i]));
    // copy edge data
    cudaFree(edgesData_d[i]);
    gpuErrchk(cudaMalloc((void **) &edgesData_d[i], sizeD));
    gpuErrchk(cudaMemcpyAsync(edgesData_d[i], edgesData.data(), sizeD, cudaMemcpyHostToDevice, streams[i]));
    // copy traffic person vector
    cudaFree(trafficPersonVec_d_gpus[i]);
    
    gpuErrchk(cudaMallocManaged(&trafficPersonVec_d_gpus[i], size_gpu_part[i]));
    for(int j = 0; j < size_gpu_part[i]/sizeof(LC::B18TrafficPerson); j++){
        trafficPersonVec_d_gpus[i][j] = trafficPersonVec[i * num_people_gpu + j]; 
    }  
    gpuErrchk(cudaMemPrefetchAsync(trafficPersonVec_d_gpus[i], size_gpu_part[i], i, streams[i]));
  }
  printMemoryUsage();
}

void b18FinishCUDA(void){
  cudaFree(trafficPersonVec_d);
  for(int i=0; i < ngpus; i++){
    cudaSetDevice(i);
    cudaFree(trafficPersonVec_d_gpus[i]);
    cudaFree(indexPathVec_d);
    cudaFree(edgesData_d);
    cudaFree(laneMap_d);
    cudaFree(intersections_d);
    cudaFree(trafficLights_d);
    cudaFree(accSpeedPerLinePerTimeInterval_d);
    cudaFree(numVehPerLinePerTimeInterval_d);
  }
}

void b18GetDataCUDA(std::vector<LC::B18TrafficPerson>& trafficPersonVec, std::vector<LC::B18EdgeData> &edgesData){
  // copy back people
  size_t size = trafficPersonVec.size() * sizeof(LC::B18TrafficPerson);
  size_t size_edges = edgesData_d_size * sizeof(LC::B18EdgeData);
  for(int i = 0; i < ngpus; i++){
      for (int j = 0; j < size_gpu_part[i]/sizeof(LC::B18TrafficPerson); j++) {
      trafficPersonVec_d[i * num_people_gpu + j] = trafficPersonVec_d_gpus[i][j];
    }
  }
  cudaMemcpy(trafficPersonVec.data(),trafficPersonVec_d,size,cudaMemcpyDeviceToHost);//cudaMemcpyHostToDevice
  
  /*for(int i = 0; i < ngpus; i++){
    cudaSetDevice(i);
    size_t size = trafficPersonVec.size() * sizeof(LC::B18TrafficPerson);
    size_t size_edges = edgesData_d_size * sizeof(LC::B18EdgeData);
    cudaMemcpy(trafficPersonVec.data(),trafficPersonVec_d,size,cudaMemcpyDeviceToHost);//cudaMemcpyHostToDevice
    cudaMemcpy(edgesData.data(),edgesData_d,size_edges,cudaMemcpyDeviceToHost);//cudaMemcpyHostToDevice
  }*/
}


 __device__ void calculateGapsLC(
   uint mapToReadShift,
   uchar* laneMap,
   uchar trafficLightState,
   uint laneToCheck,
   ushort numLinesEdge,
   float posInMToCheck,
   float length,
   uchar &v_a,
   uchar &v_b,
   float &gap_a,
   float &gap_b,
   uint laneMap_d_size) {

   ushort numOfCells = ceil(length);
   ushort initShift = ceil(posInMToCheck);
   uchar laneChar;
   bool found = false;

   // CHECK FORWARD
   //printf("initShift %u numOfCells %u\n",initShift,numOfCells);
   for (ushort b = initShift - 1; (b < numOfCells) && (!found); b++) { //NOTE -1 to make sure there is none in at the same level
     const uint posToSample = mapToReadShift +
      kMaxMapWidthM * (laneToCheck +
      (((int) (b / kMaxMapWidthM)) * numLinesEdge)) + b % kMaxMapWidthM;
     assert(posToSample < laneMap_d_size);
     laneChar = laneMap[posToSample];

     if (laneChar != 0xFF) {
       gap_a = ((float) b - initShift); //m
       v_a = laneChar; //laneChar is in 3*ms (to save space in array)
       found = true;
       break;
     }
   }

   if (!found) {
     if (trafficLightState == 0x00) { //red
       //found=true;
       gap_a = gap_b = 1000.0f; //force to change to the line without vehicle
       v_a = v_b = 0xFF;
       return;
     }
   }

   if (!found) {
     gap_a = 1000.0f;
   }

   // CHECK BACKWARDS
   found = false;

   //printf("2initShift %u numOfCells %u\n",initShift,numOfCells);
   for (int b = initShift + 1; (b >= 0) && (!found); b--) {  // NOTE +1 to make sure there is none in at the same level
     //laneChar = laneMap[mapToReadShift + maxWidth * (laneToCheck) + b];
     const uint posToSample = mapToReadShift +
      kMaxMapWidthM * (laneToCheck +
      (((int) (b / kMaxMapWidthM)) * numLinesEdge)) + b % kMaxMapWidthM;
     assert(posToSample < laneMap_d_size);
     laneChar = laneMap[posToSample];
     if (laneChar != 0xFF) {
       gap_b = ((float) initShift - b); //m
       v_b = laneChar; //laneChar is in 3*ms (to save space in array)
       found = true;
       break;
     }
   }

   //printf("3initShift %u numOfCells %u\n",initShift,numOfCells);
   if (!found) {
     gap_b = 1000.0f;
   }
  }//

__device__ void calculateLaneCarShouldBe(
  uint curEdgeLane,
  uint nextEdge,
  LC::B18IntersectionData* intersections,
  uint edgeNextInters,
  ushort edgeNumLanes,
  ushort &initOKLanes,
  ushort &endOKLanes) {

  initOKLanes = 0;
  endOKLanes = edgeNumLanes;
  bool currentEdgeFound = false;
  bool exitFound = false;
  ushort numExitToTake = 0;
  ushort numExists = 0;

  for (int eN = intersections[edgeNextInters].totalInOutEdges - 1; eN >= 0; eN--) {  // clockwise
    uint procEdge = intersections[edgeNextInters].edge[eN];

    if ((procEdge & kMaskLaneMap) == curEdgeLane) { //current edge 0xFFFFF
      currentEdgeFound = true;
      if (!exitFound) {
        numExitToTake = 0;
      }
      continue;
    }

    if ((procEdge & kMaskInEdge) == 0x0) { //out edge 0x800000
      numExists++;
      if (currentEdgeFound) {
        numExitToTake++;
      }
      if (!currentEdgeFound && !exitFound) {
        numExitToTake++;
      }
    }
    if ((procEdge & kMaskInEdge) == nextEdge) {
      exitFound = true;
      currentEdgeFound = false;
    }
  }

  if (edgeNumLanes == 0) {
    printf("ERRRROR\n");
  }

  switch (edgeNumLanes) {
    /// ONE LANE
    case 1:
      initOKLanes = 0;
      endOKLanes = 1;
      break;

    /// TWO LANE
    case 2:
      switch (numExists) {
      case 1:
      case 2://all okay
        initOKLanes = 0;
        endOKLanes = 2;
        break;

      case 3:
        if (numExitToTake > 2) { //left
          initOKLanes = 0;
          endOKLanes = 1;
          break;
        }

        initOKLanes = 1;
        endOKLanes = 2;
        break;

      default:
        if (numExitToTake >= numExists - 1) {
          initOKLanes = 0;
          endOKLanes = 1;
          break;
        }

        initOKLanes = 1;
        endOKLanes = 2;
        break;
      }

      break;

      /// THREE LANE
    case 3:
      switch (numExists) {
      case 1:
      case 2://all okay
        initOKLanes = 0;
        endOKLanes = 3;
        break;

      case 3:
        if (numExitToTake > 2) { //left
          initOKLanes = 0;
          endOKLanes = 1;
          break;
        }

        initOKLanes = 1;
        endOKLanes = 3;
        break;

      default:
        if (numExitToTake >= numExists - 1) {
          initOKLanes = 0;
          endOKLanes = 1;
          break;
        }

        initOKLanes = 1;
        endOKLanes = 2;
        break;
      }

      break;

    case 4:
      switch (numExists) {
      case 1:
      case 2://all okay
        initOKLanes = 0;
        endOKLanes = 4;
        break;

      case 3:
        if (numExitToTake == 1) { //right
          initOKLanes = 3;
          endOKLanes = 4;
        }

        if (numExitToTake > 3) { //left
          initOKLanes = 0;
          endOKLanes = 1;
          break;
        }

        initOKLanes = 1;
        endOKLanes = 4;
        break;

      default:
        if (numExitToTake == 1) { //right
          initOKLanes = edgeNumLanes - 1;
          endOKLanes = edgeNumLanes;
        }

        if (numExitToTake >= numExists - 2) {
          initOKLanes = 0;
          endOKLanes = 2;
          break;
        }

        initOKLanes = 1; //also lane 2
        endOKLanes = edgeNumLanes;
      }

      break;

    default:
      switch (numExists) {
      case 1:
      case 2://all okay
        initOKLanes = 0;
        endOKLanes = edgeNumLanes;
        break;

      case 3:
        if (numExitToTake == 1) { //right
          initOKLanes = edgeNumLanes - 1;
          endOKLanes = edgeNumLanes;
        }

        if (numExitToTake > edgeNumLanes - 2) { //left
          initOKLanes = 0;
          endOKLanes = 2;
          break;
        }

        initOKLanes = 1;
        endOKLanes = edgeNumLanes;
        break;

      default:
        if (numExitToTake < 2) { //right
          initOKLanes = edgeNumLanes - 2;
          endOKLanes = edgeNumLanes;
        }

        if (numExitToTake >= numExists - 2) {
          initOKLanes = 0;
          endOKLanes = 2;
          break;
        }

        initOKLanes = 1; //also lane 2
        endOKLanes = edgeNumLanes - 1;
      }

      break;
  }
}

__device__ float meters_per_second_to_miles_per_hour(float meters_per_second) {
  return meters_per_second * 2.2369362920544;
}

__device__ const float calculateCOStep(float personVelocity) {
  // Formula comes from the paper "Designing Large-Scale Interactive Traffic Animations for Urban Modeling"
  // Section 4.4 Traffic Indicators
  const float personVelocityMPH = meters_per_second_to_miles_per_hour(personVelocity);
  return -0.064 + 0.0056 * personVelocityMPH + 0.00026 * (personVelocityMPH - 50.0f) * (personVelocityMPH - 50.0f);
}

__device__ const float calculateGasConsumption(const float a, const float v) {
  // Formula comes from the paper "Architecture for Modular Microsimulation of Real Estate Markets and Transportation"
  // Section 6.3.4 Vehicle energy consumption and pollution estimates formula (9)
  const float Pea = a > 0.0f ? (0.472f*1.680f*a*a*v) : 0.0f;
  return 0.666f + 0.072f*(0.269f*v + 0.000672f*(v*v*v) + 0.0171f*(v*v) + 1.680f*a*v + Pea);
}

 // Kernel that executes on the CUDA device
__global__ void kernel_trafficSimulation(
  int numPeople,
  float currentTime,
  uint mapToReadShift,
  uint mapToWriteShift,
  LC::B18TrafficPerson *trafficPersonVec,
  uint *indexPathVec,
  int indexPathVec_d_size,
  LC::B18EdgeData* edgesData,
  int edgesData_d_size,
  uchar *laneMap,
  int laneMap_d_size,
  LC::B18IntersectionData *intersections,
  uchar *trafficLights,
  uint trafficLights_d_size,
  float deltaTime,
  const parameters simParameters)
  {
  int p = blockIdx.x * blockDim.x + threadIdx.x;
  if (p >= numPeople) return; //CUDA check (inside margins)
  assert( numPeople > p);
  if (trafficPersonVec[p].active == 2) return; // trip finished
  if (trafficPersonVec[p].time_departure > currentTime) return; //1.1 just continue waiting 
  // check that the current path index does not exceed the size of the path index vector
  assert(trafficPersonVec[p].indexPathCurr < indexPathVec_d_size);
  if (indexPathVec[trafficPersonVec[p].indexPathCurr] == END_OF_PATH) {
    trafficPersonVec[p].active = 2; //finished
    return;
  }
  //2.1. check if person should still wait or should start
  if (trafficPersonVec[p].active == 0) {
    //1.2 find first edge
    assert(trafficPersonVec[p].indexPathInit != INIT_EDGE_INDEX_NOT_SET);
    trafficPersonVec[p].indexPathCurr = trafficPersonVec[p].indexPathInit; // reset index.
    int indexFirstEdge = trafficPersonVec[p].indexPathCurr;
    assert(indexFirstEdge < indexPathVec_d_size);
    uint firstEdge = indexPathVec[indexFirstEdge];

    trafficPersonVec[p].last_time_simulated = currentTime;

    if (firstEdge == END_OF_PATH) {
      trafficPersonVec[p].active = 2;
      return;
    }

    if (firstEdge >= edgesData_d_size) {
      printf("firstEdge %d is bigger than edgesData size %d\n", firstEdge, edgesData_d_size);
    }
    assert(firstEdge < edgesData_d_size);
          
    //1.4 try to place it in middle of edge
    ushort numOfCells = ceil(edgesData[firstEdge].length);
    ushort initShift = (ushort) (0.5f * numOfCells); //number of cells it should be placed (half of road)

    uchar laneChar;
    bool placed = false;

    ushort numCellsEmptyToBePlaced = simParameters.s_0;
    ushort countEmptyCells = 0;
    for (ushort b = initShift; (b < numOfCells) && (!placed); b++) {
      ushort lN = edgesData[firstEdge].numLines - 1; //just right lane
      int laneMapPosition = mapToReadShift + kMaxMapWidthM * (firstEdge + lN) + b;
      assert(laneMapPosition < laneMap_d_size);
      laneChar = laneMap[laneMapPosition]; //get byte of edge (proper line)
      if (laneChar != 0xFF) {
        countEmptyCells = 0;
        continue;
      }
      countEmptyCells++;// ensure there is enough room to place the car
      if (countEmptyCells < numCellsEmptyToBePlaced) {
        continue;
      }
      trafficPersonVec[p].numOfLaneInEdge = lN;
      trafficPersonVec[p].posInLaneM = b; //m
      uchar vInMpS = (uchar) (trafficPersonVec[p].v * 3); //speed in m/s *3 (to keep more precision
      int laneMapPosition2 = mapToWriteShift + kMaxMapWidthM * (firstEdge + lN) + b;
      assert(laneMapPosition2 < laneMap_d_size);
      laneMap[laneMapPosition2] = vInMpS;
      placed = true;
      break;
    }

    if (!placed) { //not posible to start now
      return;
    }

    trafficPersonVec[p].v = 0;
    trafficPersonVec[p].LC_stateofLaneChanging = 0;

    //1.5 active car
    trafficPersonVec[p].active = 1;
    trafficPersonVec[p].isInIntersection = 0;
    trafficPersonVec[p].num_steps = 1;
    trafficPersonVec[p].co = 0.0f;
    trafficPersonVec[p].gas = 0.0f;
    
    assert(trafficPersonVec[p].indexPathCurr + 1 < indexPathVec_d_size);
    if (indexPathVec[trafficPersonVec[p].indexPathCurr + 1] != END_OF_PATH) {
      trafficPersonVec[p].LC_initOKLanes = 0xFF;
      trafficPersonVec[p].LC_endOKLanes = 0xFF;
    }
    trafficPersonVec[p].path_length_gpu = 0;

    trafficPersonVec[p].prevEdge = firstEdge;
    return;
  }

  // set up next edge info
  int indexCurrentEdge = trafficPersonVec[p].indexPathCurr;
  assert(indexCurrentEdge < indexPathVec_d_size);
  uint currentEdge = indexPathVec[indexCurrentEdge];
  assert(currentEdge < edgesData_d_size);

  int indexNextEdge = trafficPersonVec[p].indexPathCurr + 1;
  assert(indexNextEdge < indexPathVec_d_size);
  uint nextEdge = indexPathVec[indexNextEdge];
  assert(nextEdge < edgesData_d_size || nextEdge == END_OF_PATH);

  if (nextEdge != END_OF_PATH) {
    trafficPersonVec[p].LC_initOKLanes = 0xFF;
    trafficPersonVec[p].LC_endOKLanes = 0xFF;
  }

  //2. it is moving
  trafficPersonVec[p].num_steps++;
  trafficPersonVec[p].last_time_simulated = fmaxf(currentTime, trafficPersonVec[p].last_time_simulated);

  //2.1 try to move
  float numMToMove;
  bool nextVehicleIsATrafficLight = false;
  

  //when we're on a new edge for the first time
  if (currentEdge == trafficPersonVec[p].nextEdge) {
    trafficPersonVec[p].end_time_on_prev_edge = currentTime - deltaTime;
    float elapsed_s = (trafficPersonVec[p].end_time_on_prev_edge - trafficPersonVec[p].start_time_on_prev_edge); //multiply by delta_time to get seconds elapsed (not half seconds)

    // We filter whenever elapsed_s == 0, which means the time granularity was not enough to measure the speed
    // We also filter whenever 0 > elapsed_s > 5, because it causes manual_v to turn extraordinarily high
    assert(trafficPersonVec[p].prevEdge < edgesData_d_size);
    if (elapsed_s > MINIMUM_NUMBER_OF_CARS_TO_MEASURE_SPEED) {
      trafficPersonVec[p].manual_v = edgesData[trafficPersonVec[p].prevEdge].length / elapsed_s;
      edgesData[trafficPersonVec[p].prevEdge].curr_iter_num_cars += 1;
      edgesData[trafficPersonVec[p].prevEdge].curr_cum_vel += trafficPersonVec[p].manual_v;
    }

    trafficPersonVec[p].start_time_on_prev_edge = currentTime;
    trafficPersonVec[p].prevEdge = currentEdge;
  }
  trafficPersonVec[p].nextEdge = nextEdge;
  

  // www.vwi.tu-dresden.de/~treiber/MicroApplet/IDM.html
  // IDM
  float thirdTerm = 0;
  // 2.1.1 Find front car
  int numCellsCheck = max(30.0f, trafficPersonVec[p].v * deltaTime * 2); //30 or double of the speed*time
  
  // a) SAME LINE (BEFORE SIGNALING)
  bool found = false;
  bool noFirstInLaneBeforeSign = false; //use for stop control (just let 1st to pass) TODO(pavan): I DON'T GET THIS
  bool noFirstInLaneAfterSign = false; //use for stop control (just let 1st to pass)
  float s;
  float delta_v;
  uchar laneChar;
  ushort byteInLine = (ushort) floor(trafficPersonVec[p].posInLaneM);
  ushort numOfCells = ceil((edgesData[currentEdge].length - intersectionClearance)); //intersectionClearance hardcoded to 7.8f - why?

  for (ushort b = byteInLine + 2; (b < numOfCells) && (!found) && (numCellsCheck > 0); b++, numCellsCheck--) {
    // ShiftRead + WIDTH * (width number * # lanes + # laneInEdge) + b  TODO(pavan): WHAT IS THIS?
    //TODO(pavan): double check what mapToReadShift is printing out
    assert(trafficPersonVec[p].indexPathCurr < indexPathVec_d_size);
    const uint posToSample = mapToReadShift +
      kMaxMapWidthM *(currentEdge +
      (((int) (byteInLine / kMaxMapWidthM)) * edgesData[currentEdge].numLines) +
      trafficPersonVec[p].numOfLaneInEdge) + b % kMaxMapWidthM;
    assert(posToSample < laneMap_d_size);
    laneChar = laneMap[posToSample];

    //TODO(pavan): Is this clause for when it is not at the intersection yet but it has found a car in front of it?
    if (laneChar != 0xFF) {
      s = ((float) (b - byteInLine)); //m
      delta_v = trafficPersonVec[p].v - (laneChar / 3.0f); //laneChar is in 3*ms (to save space in array)
      found = true;
      noFirstInLaneBeforeSign = true; 
      break;
    }
  } 

  // NEXT LINE
  // e) MOVING ALONG IN THE NEXT EDGE
  if (!found && numCellsCheck > 0) { //check if in next line
    if ((nextEdge != END_OF_PATH) &&
      (edgesData[currentEdge].nextIntersMapped !=
        trafficPersonVec[p].end_intersection)) { // we haven't arrived to destination (check next line)
      ushort nextEdgeLaneToBe = trafficPersonVec[p].numOfLaneInEdge; //same lane

      //printf("trafficPersonVec[p].numOfLaneInEdge %u\n",trafficPersonVec[p].numOfLaneInEdge);
      assert(nextEdge < edgesData_d_size);
      if (nextEdgeLaneToBe >= edgesData[nextEdge].numLines) {
        nextEdgeLaneToBe = edgesData[nextEdge].numLines - 1; //change line if there are less roads
      }

      //printf("2trafficPersonVec[p].numOfLaneInEdge %u\n",trafficPersonVec[p].numOfLaneInEdge);
      ushort numOfCells = ceil(edgesData[nextEdge].length);

      for (ushort b = 0; (b < numOfCells) && (!found) && (numCellsCheck > 0); b++, numCellsCheck--) {
        const uint posToSample = mapToReadShift + kMaxMapWidthM * (nextEdge + nextEdgeLaneToBe) + b; // b18 not changed since we check first width
        assert(posToSample < laneMap_d_size);
        laneChar = laneMap[posToSample];

        if (laneChar != 0xFF) {
          s = ((float) (b)); //m
          delta_v = trafficPersonVec[p].v - (laneChar / 3.0f);  // laneChar is in 3*ms (to save space in array)
          found = true;
          break;
        }
      }
    }
  }


  float s_star;
  if (found && delta_v > 0) { //car in front and slower than us
    // 2.1.2 calculate dv_dt
    // The following operation is taken from Designing Large-Scale Interactive Traffic Animations for Urban Modeling
    // Section 4.3.1. Car-Following Model formula (2)
    s_star = simParameters.s_0 + max(0.0f,
      (trafficPersonVec[p].v * trafficPersonVec[p].T + (trafficPersonVec[p].v *
      delta_v) / (2 * sqrtf(trafficPersonVec[p].a * trafficPersonVec[p].b))));
    thirdTerm = powf(((s_star) / (s)), 2);
  }

  // The following operation is taken from Designing Large-Scale Interactive Traffic Animations for Urban Modeling
  // Section 4.3.1. Car-Following Model formula (1)
  // And also Architecture for Modular Microsimulation of Real Estate Markets and Transportation
  // Section 6.3.2 Per-vehicle and traffic control simulation formula (7)
  float dv_dt = trafficPersonVec[p].a * (1.0f - std::pow((
    trafficPersonVec[p].v / edgesData[currentEdge].maxSpeedMperSec), 4) - thirdTerm);

  // 2.1.3 update values
  numMToMove = max(0.0f, trafficPersonVec[p].v * deltaTime + 0.5f * (dv_dt) * deltaTime * deltaTime);
  trafficPersonVec[p].v += dv_dt * deltaTime;

  if (trafficPersonVec[p].v < 0) {
    trafficPersonVec[p].v = 0;
    dv_dt = 0.0f;
  }
  trafficPersonVec[p].cum_v += trafficPersonVec[p].v;

  if (calculatePollution && ((float(currentTime) == int(currentTime)))) { // enabled and each second (assuming deltaTime 0.5f)
    const float coStep = calculateCOStep(trafficPersonVec[p].v);
    if (coStep > 0) {
      trafficPersonVec[p].co += coStep;
    }
    trafficPersonVec[p].gas += calculateGasConsumption(dv_dt, trafficPersonVec[p].v);
  }

  if (trafficPersonVec[p].v == 0) { //if not moving not do anything else
    ushort posInLineCells = (ushort) (trafficPersonVec[p].posInLaneM);
    const uint posToSample = mapToWriteShift +
      kMaxMapWidthM * (currentEdge +
      (((int) (posInLineCells / kMaxMapWidthM)) * edgesData[currentEdge].numLines) +
      trafficPersonVec[p].numOfLaneInEdge) +
      posInLineCells % kMaxMapWidthM;
    assert(posToSample < laneMap_d_size);
    laneMap[posToSample] = 0;
    return;
  }

  // COLOR
  trafficPersonVec[p].color = p << 8;

  // STOP (check if it is a stop if it can go through)
  trafficPersonVec[p].posInLaneM = trafficPersonVec[p].posInLaneM + numMToMove;

  //2.2 close to intersection
  //2.2 check if change intersection
  if (trafficPersonVec[p].posInLaneM > edgesData[currentEdge].length) { //reach intersection
    numMToMove = trafficPersonVec[p].posInLaneM - edgesData[currentEdge].length;
    trafficPersonVec[p].posInLaneM = numMToMove;
    trafficPersonVec[p].dist_traveled += edgesData[currentEdge].length;
    trafficPersonVec[p].path_length_gpu++;

    //2.2.1 find next edge
    assert(indexCurrentEdge < indexPathVec_d_size);
    assert(currentEdge < edgesData_d_size);

    trafficPersonVec[p].LC_stateofLaneChanging = 0;

    //2.1 check if end
    if (nextEdge != END_OF_PATH) {
      assert(nextEdge < edgesData_d_size);
      if (trafficPersonVec[p].numOfLaneInEdge >= edgesData[nextEdge].numLines) {
        trafficPersonVec[p].numOfLaneInEdge = edgesData[nextEdge].numLines - 1; //change line if there are less roads
      }

      //TODO: Test if the following line is doing the conversion wrong
      uchar vInMpS = (uchar) (trafficPersonVec[p].v * 3); //speed in m/s to fit in uchar
      ushort posInLineCells = (ushort) (trafficPersonVec[p].posInLaneM);
      const uint posToSample = mapToWriteShift + kMaxMapWidthM *
                              (nextEdge + (((int) (posInLineCells / kMaxMapWidthM)) *
                              edgesData[nextEdge].numLines) + trafficPersonVec[p].numOfLaneInEdge) +
                              posInLineCells % kMaxMapWidthM;  // note the last % should not happen

      assert(posToSample < laneMap_d_size);
      laneMap[posToSample] = vInMpS;

      trafficPersonVec[p].LC_initOKLanes = 0xFF;
      trafficPersonVec[p].LC_endOKLanes = 0xFF;
    } else {
      trafficPersonVec[p].active == 2;
    }
    trafficPersonVec[p].indexPathCurr++;
    trafficPersonVec[p].LC_stateofLaneChanging = 0;

  } else { //does not reach an intersection
    assert(indexCurrentEdge < indexPathVec_d_size);
    assert(indexNextEdge < indexPathVec_d_size);
    assert(currentEdge < edgesData_d_size);
    assert(nextEdge < edgesData_d_size || nextEdge == END_OF_PATH);

    // LANE CHANGING (happens when we are not reached the intersection)
    if (trafficPersonVec[p].v > 3.0f && trafficPersonVec[p].num_steps % 5 == 0) {
      //at least 10km/h to try to change lane
      //just check every (5 steps) 5 seconds

      // next thing is not a traffic light
      // skip if there is one lane (avoid to do this)
      // skip if it is the last edge
      if (!nextVehicleIsATrafficLight &&
        edgesData[currentEdge].numLines > 1 && nextEdge != END_OF_PATH) {
        ////////////////////////////////////////////////////
        // LC 1 update lane changing status
        if (trafficPersonVec[p].LC_stateofLaneChanging == 0) {
          // 2.2-exp((x-1)^2)
          float x = trafficPersonVec[p].posInLaneM / edgesData[currentEdge].length;

          if (x > 0.4f) { //just after 40% of the road
            float probabiltyMandatoryState = 2.2 - exp((x - 1) * (x - 1));

            //if (((float) qrand() / RAND_MAX) < probabiltyMandatoryState) {
            if ((((int) (x * 100) % 100) / 100.0f) < probabiltyMandatoryState) { // pseudo random number
              trafficPersonVec[p].LC_stateofLaneChanging = 1;
            }
          }

        }

        // LC 2 NOT MANDATORY STATE
        if (trafficPersonVec[p].LC_stateofLaneChanging == 0) {
          // discretionary change: v slower than the current road limit and deccelerating and moving
          if ((trafficPersonVec[p].v < (edgesData[currentEdge].maxSpeedMperSec * 0.7f)) &&
            (dv_dt < 0) && trafficPersonVec[p].v > 3.0f) {

            bool leftLane = trafficPersonVec[p].numOfLaneInEdge >
              0; //at least one lane on the left
            bool rightLane = trafficPersonVec[p].numOfLaneInEdge <
              edgesData[currentEdge].numLines - 1; //at least one lane

            if (leftLane && rightLane) {
              if (int(trafficPersonVec[p].v) % 2 == 0) { // pseudo random
                leftLane = false;
              } else {
                rightLane = false;
              }
            }
            ushort laneToCheck;
            if (leftLane) {
              laneToCheck = trafficPersonVec[p].numOfLaneInEdge - 1;
            } else {
              laneToCheck = trafficPersonVec[p].numOfLaneInEdge + 1;
            }

            uchar v_a, v_b;
            float gap_a, gap_b;

            assert(currentEdge + trafficPersonVec[p].numOfLaneInEdge < trafficLights_d_size);
            uchar trafficLightState = trafficLights[currentEdge + trafficPersonVec[p].numOfLaneInEdge];
            calculateGapsLC(mapToReadShift, laneMap, trafficLightState,
              currentEdge + laneToCheck, edgesData[currentEdge].numLines,
              trafficPersonVec[p].posInLaneM,
              edgesData[currentEdge].length, v_a, v_b, gap_a, gap_b, laneMap_d_size);

            if (gap_a == 1000.0f && gap_b == 1000.0f) { //lag and lead car very far
              trafficPersonVec[p].numOfLaneInEdge = laneToCheck; // CHANGE LINE

            } else { // NOT ALONE
              float b1A = 0.05f, b2A = 0.15f;
              float b1B = 0.15f, b2B = 0.40f;
              // simParameters.s_0-> critical lead gap
              float g_na_D, g_bn_D;
              bool acceptLC = true;

              if (gap_a != 1000.0f) {
                g_na_D = max(simParameters.s_0, simParameters.s_0 + b1A * trafficPersonVec[p].v + b2A *
                  (trafficPersonVec[p].v - v_a * 3.0f));

                if (gap_a < g_na_D) { //gap smaller than critical gap
                  acceptLC = false;
                }
              }

              if (acceptLC && gap_b != 1000.0f) {
                g_bn_D = max(simParameters.s_0, simParameters.s_0 + b1B * v_b * 3.0f + b2B * (v_b * 3.0f - trafficPersonVec[p].v));

                if (gap_b < g_bn_D) { //gap smaller than critical gap
                  acceptLC = false;
                }
              }

              if (acceptLC) {
                trafficPersonVec[p].numOfLaneInEdge = laneToCheck; // CHANGE LINE
              }
            }
          }


        }// Discretionary

        // LC 3 *MANDATORY* STATE
        if (trafficPersonVec[p].LC_stateofLaneChanging == 1) {
          // LC 3.1 Calculate the correct lanes
          if (trafficPersonVec[p].LC_endOKLanes == 0xFF) {
            calculateLaneCarShouldBe(currentEdge, nextEdge, intersections,
              edgesData[currentEdge].nextIntersMapped,
              edgesData[currentEdge].numLines,
              trafficPersonVec[p].LC_initOKLanes, trafficPersonVec[p].LC_endOKLanes);

            if (trafficPersonVec[p].LC_initOKLanes == 0 &&
              trafficPersonVec[p].LC_endOKLanes == 0) {
            }
          }

          bool leftLane = false, rightLane = false;

          // LC 3.2 CORRECT LANES--> DICRETIONARY LC WITHIN
          if (trafficPersonVec[p].numOfLaneInEdge >= trafficPersonVec[p].LC_initOKLanes &&
            trafficPersonVec[p].numOfLaneInEdge < trafficPersonVec[p].LC_endOKLanes) {
            // for discretionary it should be under some circustances
            if ((trafficPersonVec[p].v < (edgesData[currentEdge].maxSpeedMperSec * 0.7f)) &&
              (dv_dt < 0) && trafficPersonVec[p].v > 3.0f) {
              leftLane =
                (trafficPersonVec[p].numOfLaneInEdge > 0) && //at least one lane on the left
                (trafficPersonVec[p].numOfLaneInEdge - 1 >= trafficPersonVec[p].LC_initOKLanes)
                &&
                (trafficPersonVec[p].numOfLaneInEdge - 1 < trafficPersonVec[p].LC_endOKLanes);
              rightLane =
                (trafficPersonVec[p].numOfLaneInEdge <
                  edgesData[currentEdge].numLines - 1) &&
                //at least one lane
                (trafficPersonVec[p].numOfLaneInEdge + 1 >= trafficPersonVec[p].LC_initOKLanes)
                &&
                (trafficPersonVec[p].numOfLaneInEdge + 1 < trafficPersonVec[p].LC_endOKLanes);
            }
          } else {
            // LC 3.3 INCORRECT LANES--> MANDATORY LC
            if (trafficPersonVec[p].numOfLaneInEdge < trafficPersonVec[p].LC_initOKLanes) {
              rightLane = true;
            } else {
              leftLane = true;
            }

            if (rightLane &&
              trafficPersonVec[p].numOfLaneInEdge + 1 >= edgesData[currentEdge].numLines) {
              printf("ERROR: RT laneToCheck>=edgeNumLanes\n");
            }

            if (leftLane && trafficPersonVec[p].numOfLaneInEdge == 0) {
              printf("ERROR %u: LT laneToCheck>=edgeNumLanes OK %u-%u NE %u\n",
                p, trafficPersonVec[p].LC_initOKLanes, trafficPersonVec[p].LC_endOKLanes,
                currentEdge);
            }
          }

          if (leftLane || rightLane) {

            // choose lane (if necessary)
            if (leftLane && rightLane) {
              if ((int) (trafficPersonVec[p].posInLaneM) % 2 == 0) { //pseudo random
                leftLane = false;
              } else {
                rightLane = false;
              }
            }
            ushort laneToCheck;
            if (leftLane) {
              laneToCheck = trafficPersonVec[p].numOfLaneInEdge - 1;
            } else {
              laneToCheck = trafficPersonVec[p].numOfLaneInEdge + 1;
            }

            if (laneToCheck >= edgesData[currentEdge].numLines) {
              printf("ERROR: laneToCheck>=edgesData[currentEdge].numLines %u %u\n",
                laneToCheck, edgesData[currentEdge].numLines);
            }

            uchar v_a, v_b;
            float gap_a, gap_b;
            assert(currentEdge + trafficPersonVec[p].numOfLaneInEdge < trafficLights_d_size);
            uchar trafficLightState = trafficLights[currentEdge + trafficPersonVec[p].numOfLaneInEdge];
            calculateGapsLC(mapToReadShift, laneMap, trafficLightState,
              currentEdge + laneToCheck, edgesData[currentEdge].numLines,
              trafficPersonVec[p].posInLaneM,
              edgesData[currentEdge].length, v_a, v_b, gap_a, gap_b, laneMap_d_size);

            if (gap_a == 1000.0f && gap_b == 1000.0f) { //lag and lead car very far
              trafficPersonVec[p].numOfLaneInEdge = laneToCheck; // CHANGE LINE
            } else { // NOT ALONE
              float b1A = 0.05f, b2A = 0.15f;
              float b1B = 0.15f, b2B = 0.40f;
              float gamma = 0.000025;
              // simParameters.s_0-> critical lead gap
              float distEnd = edgesData[currentEdge].length - trafficPersonVec[p].posInLaneM;
              float expTerm = (1 - exp(-gamma * distEnd * distEnd));

              float g_na_M, g_bn_M;
              bool acceptLC = true;

              if (gap_a != 1000.0f) {
                g_na_M = max(simParameters.s_0, simParameters.s_0 + (b1A * trafficPersonVec[p].v + b2A *
                  (trafficPersonVec[p].v - v_a * 3.0f)));

                if (gap_a < g_na_M) { //gap smaller than critical gap
                  acceptLC = false;
                }
              }

              if (acceptLC && gap_b != 1000.0f) {
                g_bn_M = max(simParameters.s_0, simParameters.s_0 + (b1B * v_b * 3.0f + b2B * (v_b * 3.0f -
                  trafficPersonVec[p].v)));

                if (gap_b < g_bn_M) { //gap smaller than critical gap
                  acceptLC = false;
                }
              }

              if (acceptLC) {
                trafficPersonVec[p].numOfLaneInEdge = laneToCheck; // CHANGE LINE
              }
            }
          }
        }// Mandatory
      }//at least two lanes and not stopped by traffic light
    }

    uchar vInMpS = (uchar) (trafficPersonVec[p].v * 3); //speed in m/s to fit in uchar
    ushort posInLineCells = (ushort) (trafficPersonVec[p].posInLaneM);
    const uint posToSample = mapToWriteShift +
      kMaxMapWidthM * (currentEdge + (((int) (posInLineCells / kMaxMapWidthM)) *
      edgesData[currentEdge].numLines) +
      trafficPersonVec[p].numOfLaneInEdge) +
      posInLineCells % kMaxMapWidthM;
    assert(posToSample < laneMap_d_size);
    laneMap[posToSample] = vInMpS;
  }
}

/*
__global__ void kernel_intersectionSTOPSimulation(
     uint numIntersections, 
     float currentTime, 
     LC::B18IntersectionData *intersections, 
     uchar *trafficLights,
     LC::B18EdgeData* edgesData,//for the length
     uchar* laneMap,//to check if there are cars
     uint mapToReadShift) {
     int i = blockIdx.x * blockDim.x + threadIdx.x;
     if (i<numIntersections) {//CUDA check (inside margins)

     const float deltaEvent = 0.0f; 

     //if(i==0)printf("i %d\n",i);
     if (currentTime > intersections[i].nextEvent && intersections[i].totalInOutEdges > 0) {
       uint edgeOT = intersections[i].edge[intersections[i].state];
       uchar numLinesO = edgeOT >> 24;
       uint edgeONum = edgeOT & kMaskLaneMap; // 0xFFFFF

       // red old traffic lights
       for (int nL = 0; nL < numLinesO; nL++) {
         trafficLights[edgeONum + nL] = 0x00; //red old traffic light
       }

       for (int iN = 0; iN <= intersections[i].totalInOutEdges + 1; iN++) { //to give a round
         intersections[i].state = (intersections[i].state + 1) %
           intersections[i].totalInOutEdges;//next light

         if ((intersections[i].edge[intersections[i].state] & kMaskInEdge) == kMaskInEdge) {  // 0x800000
           uint edgeIT = intersections[i].edge[intersections[i].state];
           uint edgeINum = edgeIT & kMaskLaneMap; //get edgeI 0xFFFFF
           uchar numLinesI = edgeIT >> 24;
           /// check if someone in this edge
           int rangeToCheck = 5.0f; //5m
           ushort firstPosToCheck = edgesData[edgeINum].length - intersectionClearance; //last po
           bool atLeastOneStopped = false;

           for (int posCheck = firstPosToCheck; rangeToCheck >= 0 && posCheck >= 0; posCheck--, rangeToCheck--) { //as many cells as the rangeToCheck says
             for (int nL = 0; nL < numLinesI; nL++) {
               //int cellNum = mapToReadShift + maxWidth * (edgeINum + nL) + posCheck;
               const uint posToSample = mapToReadShift + kMaxMapWidthM * (edgeINum + (((int) (posCheck / kMaxMapWidthM)) * numLinesI) + nL) + posCheck % kMaxMapWidthM;


               if (laneMap[posToSample] == 0) { //car stopped
                 trafficLights[edgeINum + nL] = 0x0F; // STOP SIGN 0x0F--> Let pass
                 atLeastOneStopped = true;
               }
             }
           }

           if (atLeastOneStopped == true) {
             intersections[i].nextEvent = currentTime + deltaEvent; //just move forward time if changed (otherwise check in next iteration)
             break;
           }
         }
       }
     }
     ///
   }
   
}//
*/

__global__ void kernel_intersectionOneSimulation(
      uint numIntersections,
      float currentTime,
      LC::B18IntersectionData *intersections,
      uchar *trafficLights) {

  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if(i<numIntersections){//CUDA check (inside margins)
    const float deltaEvent = 20.0f; /// !!!!
    if (currentTime > intersections[i].nextEvent && intersections[i].totalInOutEdges > 0) {

      uint edgeOT = intersections[i].edge[intersections[i].state];
      uchar numLinesO = edgeOT >> 24;
      uint edgeONum = edgeOT & kMaskLaneMap; // 0xFFFFF;

      // red old traffic lights
      if ((edgeOT&kMaskInEdge) == kMaskInEdge) {  // Just do it if we were in in
        for (int nL = 0; nL < numLinesO; nL++) {
          trafficLights[edgeONum + nL] = 0x00; //red old traffic light
        }
      }

      for (int iN = 0; iN <= intersections[i].totalInOutEdges + 1; iN++) { //to give a round
        intersections[i].state = (intersections[i].state + 1) % intersections[i].totalInOutEdges;//next light

        if ((intersections[i].edge[intersections[i].state] & kMaskInEdge) == kMaskInEdge) {  // 0x800000
          // green new traffic lights
          uint edgeIT = intersections[i].edge[intersections[i].state];
          uint edgeINum = edgeIT & kMaskLaneMap; //  0xFFFFF; //get edgeI
          uchar numLinesI = edgeIT >> 24;

          for (int nL = 0; nL < numLinesI; nL++) {
            trafficLights[edgeINum + nL] = 0xFF;
          }

          //trafficLights[edgeINum]=0xFF;
          break;
        }
      }//green new traffic light

      intersections[i].nextEvent = currentTime + deltaEvent;
    }
  }
   
 }//

// Kernel that executes on the CUDA device
__global__ void kernel_sampleTraffic(
  int numPeople,
  LC::B18TrafficPerson *trafficPersonVec,
  uint *indexPathVec,
  int indexPathVec_d_size,
  float *accSpeedPerLinePerTimeInterval,
  uint accSpeedPerLinePerTimeInterval_d_size,
  float *numVehPerLinePerTimeInterval,
  uint numVehPerLinePerTimeInterval_d_size,
  uint offset)
  {
  int p = blockIdx.x * blockDim.x + threadIdx.x;
  if (p >= numPeople) {
    //CUDA check (inside margins)
    return;
  }

  if (trafficPersonVec[p].active == 1 && trafficPersonVec[p].indexPathCurr != END_OF_PATH) {
    assert(trafficPersonVec[p].indexPathCurr < indexPathVec_d_size);
    int edgeNum = indexPathVec[trafficPersonVec[p].indexPathCurr];

    assert(edgeNum + offset < accSpeedPerLinePerTimeInterval_d_size);
    accSpeedPerLinePerTimeInterval[edgeNum + offset] += trafficPersonVec[p].v / 3.0f;

    assert(edgeNum + offset < numVehPerLinePerTimeInterval_d_size);
    numVehPerLinePerTimeInterval[edgeNum + offset]++;
  }
}
__global__ void kernel_resetPeople(
  int numPeople,
  LC::B18TrafficPerson *trafficPersonVec) {
  int p = blockIdx.x * blockDim.x + threadIdx.x;
  if (p < numPeople) {//CUDA check (inside margins)
    trafficPersonVec[p].active = 0;
  }
}

void b18GetSampleTrafficCUDA(std::vector<float>& accSpeedPerLinePerTimeInterval, std::vector<float>& numVehPerLinePerTimeInterval) {
  // copy back people
  size_t size = accSpeedPerLinePerTimeInterval.size() * sizeof(float);
  cudaMemcpy(accSpeedPerLinePerTimeInterval.data(), accSpeedPerLinePerTimeInterval_d, size, cudaMemcpyDeviceToHost);

  size_t sizeI = numVehPerLinePerTimeInterval.size() * sizeof(uchar);
  cudaMemcpy(numVehPerLinePerTimeInterval.data(), numVehPerLinePerTimeInterval_d, sizeI, cudaMemcpyDeviceToHost);
}

void b18ResetPeopleLanesCUDA(uint numPeople) {
  kernel_resetPeople << < ceil(numPeople / 1024.0f), 1024 >> > (numPeople, trafficPersonVec_d);
  cudaMemset(&laneMap_d[0], -1, halfLaneMap*sizeof(unsigned char));
  cudaMemset(&laneMap_d[halfLaneMap], -1, halfLaneMap*sizeof(unsigned char));
}

void b18SimulateTrafficCUDA(float currentTime,
  uint numPeople,
  uint numIntersections,
  float deltaTime,
  const parameters simParameters,
  int numBlocks,
  int threadsPerBlock) {
  intersectionBench.startMeasuring();
  const uint numStepsTogether = 12; //change also in density (10 per hour)
  // 1. CHANGE MAP: set map to use and clean the other
  for(int i = 0; i < ngpus; i++){
    cudaSetDevice(i);
    if (readFirstMapC==true) {
      mapToReadShift=0;
      mapToWriteShift=halfLaneMap;
      gpuErrchk(cudaMemset(&laneMap_d[i][halfLaneMap], -1, halfLaneMap*sizeof(unsigned char)));//clean second half
    } 
    else {
      mapToReadShift=halfLaneMap;
      mapToWriteShift=0;
      gpuErrchk(cudaMemset(&laneMap_d[i][0], -1, halfLaneMap*sizeof(unsigned char)));//clean first half
    }
  }
  readFirstMapC=!readFirstMapC;//next iteration invert use

  // Simulate intersections.
  for(int i = 0; i < ngpus; i++){
    cudaSetDevice(i);
    kernel_intersectionOneSimulation << < ceil(numIntersections / 512.0f), 512 >> > (numIntersections, currentTime, intersections_d[i], trafficLights_d[i]);
    gpuErrchk(cudaPeekAtLastError());
  }
  intersectionBench.stopMeasuring();
  
  peopleBench.startMeasuring();
  // Simulate people.
  // #pragma omp parallel for
        // for(int i = 0; i < 2; i++) {
  
  //printf("Number of people per GPU : %i ", numPeople_gpu);
  for(int i = 0; i < ngpus; i++){
    cudaSetDevice(i);
    int numPeople_gpu = size_gpu_part[i]/sizeof(LC::B18TrafficPerson);
    kernel_trafficSimulation <<< numBlocks, threadsPerBlock>> >
    (numPeople_gpu, currentTime, mapToReadShift,
    mapToWriteShift, trafficPersonVec_d_gpus[i], indexPathVec_d[i], indexPathVec_d_size,
    edgesData_d[i], edgesData_d_size, laneMap_d[i], laneMap_d_size,
    intersections_d[i], trafficLights_d[i], trafficLights_d_size, deltaTime, simParameters);
    gpuErrchk(cudaPeekAtLastError());
    gpuErrchk(cudaDeviceSynchronize());
    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess) {
        printf("CUDA error: %s\n", cudaGetErrorString(err));
    }
  }
  peopleBench.stopMeasuring();
        // }

}


