/************************************************************************************************
*
*		LPSim Development - B18 trafficSimulator
*
*		@author xuan, jiaying, luze
*
************************************************************************************************/


//CUDA CODE
#include <stdio.h>
#include "cuda_runtime.h"
#include "curand_kernel.h"
#include "device_launch_parameters.h"
#include "assert.h"
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/copy.h>
#include "b18TrafficPerson.h"
#include "b18EdgeData.h"
#include <vector>
#include <iostream>
#include <cstring>
#include <map>
#include <algorithm> 
#include "../../src/benchmarker.h"
#include "sp/config.h"
#include <iomanip>
#include <chrono>
#include <ctime>

#ifndef ushort
#define ushort uint16_t
#endif
#include <thread>
#ifndef uint
#define uint uint32_t
#endif
#ifndef uchar
#define uchar uint8_t
#endif

///////////////////////////////
// CONSTANTS

#define MINIMUM_NUMBER_OF_CARS_TO_MEASURE_SPEED 5

__constant__ float intersectionClearance = 0.0f; //TODO(pavan): WHAT IS THIS?

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
LC::B18TrafficVehicle *trafficVehicleVec_d;
// GPU i traffic person vector, i in (0, ..., ngpus)
int ngpus;
thrust::device_vector<LC::B18TrafficVehicle>** vehicles_vec = nullptr;
int num_people_gpu;
LC::B18TrafficVehicle **trafficVehicleVec_d_gpus = nullptr;
uint **indexPathVec_d = nullptr;
uint indexPathVec_d_size;
LC::B18EdgeData **edgesData_d = nullptr;
uint *edgesData_d_size= nullptr;
uint *laneMap_d_size= nullptr;
uint * trafficLights_d_size= nullptr;
uint accSpeedPerLinePerTimeInterval_d_size;
uint numVehPerLinePerTimeInterval_d_size;
size_t *size_gpu_part= nullptr;
__constant__ bool calculatePollution = true;
__constant__ float cellSize = 1.0f;

uchar **laneMap_d= nullptr;
uchar **laneMap_d_gpus = nullptr;
uint** laneIdMapper= nullptr;
uint** laneIdMapper_d = nullptr;
int** vertexIdToPar_d = nullptr;
bool readFirstMapC=true;
uint mapToReadShift;
uint *mapToReadShift_n = nullptr;
uint mapToWriteShift;
uint *mapToWriteShift_n = nullptr;
uint halfLaneMap;
uint *halfLaneMap_n = nullptr;
float startTime;
const int buffer_size=10000; 
const int buffer_lane_size=100000; 
uint **vehicleToCopy_d = nullptr;
uint **copyCursor_d = nullptr;
uint **vehicleToRemove_d = nullptr;
uint **removeCursor_d = nullptr;
uint *copyCursor = nullptr;
uint *removeCursor = nullptr;
uint **ghostLaneBuffer_d = nullptr;
uint **ghostLaneCursor_d = nullptr;
uint *ghostLaneCursor = nullptr;
uint **laneToUpdateIndex_d = nullptr;
uint **laneToUpdateValues_d = nullptr;
LC::B18IntersectionData **intersections_d  = nullptr;
uchar **trafficLights_d  = nullptr;
// std::map<int,std::vector<LC::B18TrafficVehicle> >personToCopy;
// std::map<int,std::vector<int> >personToRemove;//eg: 1->{1,3,5},2->{9},3->{} (gpuIndex->personList)



float* accSpeedPerLinePerTimeInterval_d;
float* numVehPerLinePerTimeInterval_d;
void b18InitCUDA_n(
  int num_gpus,
  bool firstInitialization,
  const std::vector<int>& vertexIdToPar,
  int edges_num,
  std::map<uint, uint>laneIdToLaneIdInGpu[],
  std::vector<LC::B18TrafficVehicle>& trafficVehicleVec, 
  std::vector<uint> indexPathVec_n[], 
  std::vector<LC::B18EdgeData> edgesData_n[], 
  std::vector<uchar> laneMap_n[], 
  std::vector<uchar> trafficLights_n[], 
  std::vector<LC::B18IntersectionData> intersections_n[],
  float startTimeH, float endTimeH,
  std::vector<float>& accSpeedPerLinePerTimeInterval,
  std::vector<float>& numVehPerLinePerTimeInterval,
  float deltaTime) {
  ngpus = num_gpus;
  int maxGpus = 0;
  cudaGetDeviceCount(&maxGpus);
  if(maxGpus<ngpus){
    printf("NUM_GPUS is %d but only %d gpus on device\n",ngpus,maxGpus);
    exit(1);
  }
  assert(maxGpus>=ngpus);
  trafficVehicleVec_d_gpus = new LC::B18TrafficVehicle*[ngpus];
  indexPathVec_d = new uint*[ngpus];
  edgesData_d = new LC::B18EdgeData*[ngpus];
  edgesData_d_size= new uint[ngpus];
  laneMap_d_size= new uint[ngpus];
  trafficLights_d_size= new uint[ngpus];
  size_gpu_part= new size_t[ngpus];
  laneMap_d = new uchar*[ngpus];
  laneMap_d_gpus = new uchar*[ngpus];
  laneIdMapper_d=new uint*[ngpus];
  vertexIdToPar_d= new int*[ngpus];
  mapToReadShift_n= new uint[ngpus];
  mapToWriteShift_n= new uint[ngpus];
  halfLaneMap_n = new uint[ngpus];
  vehicleToCopy_d = new uint*[ngpus];
  copyCursor_d= new uint*[ngpus];
  vehicleToRemove_d = new uint*[ngpus];
  removeCursor_d= new uint*[ngpus];
  copyCursor= new uint[ngpus];
  removeCursor= new uint[ngpus];
  laneIdMapper= new uint*[ngpus];
  ghostLaneBuffer_d = new uint*[ngpus];
  ghostLaneCursor_d= new uint*[ngpus];
  ghostLaneCursor= new uint[ngpus];
  laneToUpdateIndex_d = new uint*[ngpus];
  laneToUpdateValues_d = new uint*[ngpus];
  intersections_d  = new LC::B18IntersectionData*[ngpus];
  trafficLights_d = new uchar*[ngpus];
  vehicles_vec = new thrust::device_vector<LC::B18TrafficVehicle>*[ngpus];

  cudaStream_t *streams = new cudaStream_t[ngpus];
  for(int i = 0; i < ngpus; i++){
      cudaStreamCreate( &streams[i]);
  }
  //printf(">>b18InitCUDA firstInitialization %s\n", (firstInitialization?"INIT":"ALREADY INIT"));
  //printMemoryUsage();
  const uint numStepsPerSample = 30.0f / deltaTime; //each min
  const uint numStepsTogether = 12; //change also in density (10 per hour)
  { // people
    size_t size = trafficVehicleVec.size() * sizeof(LC::B18TrafficVehicle);
    
    if (firstInitialization){
      gpuErrchk(cudaMallocManaged(&trafficVehicleVec_d, size));
      memcpy(trafficVehicleVec_d, trafficVehicleVec.data(), size);
    }

    // Calculate the size of each half
    num_people_gpu = int(trafficVehicleVec.size() / ngpus);
    for(int i = 0; i < ngpus; i++){
        size_gpu_part[i]=0;
    }
    // size_gpu_part[ngpus-1] = (trafficVehicleVec.size() - num_people_gpu *(ngpus-1)) * sizeof(LC::B18TrafficVehicle);

    // Allocate memory for each half on the respective GPU
    //LC::B18TrafficVehicle **trafficVehicleVec_d_gpus[ngpus];

    // Copy the first half to GPU 0 and the second half to GPU 1

    // compute initial size of trafficPerson on each gpu
    for(const LC::B18TrafficVehicle trafficPerson_i : trafficVehicleVec){
      unsigned int init_intersectionId=trafficPerson_i.init_intersection;
      int targetPartition=vertexIdToPar[init_intersectionId];
      size_gpu_part[targetPartition]+= sizeof(LC::B18TrafficVehicle);
    }
    for(int i = 0; i < ngpus; i++){
      trafficVehicleVec_d_gpus[i] = new LC::B18TrafficVehicle[size_gpu_part[i]/sizeof(LC::B18TrafficVehicle)];
    }
    int* personIndex = new int[ngpus]();
    for(const LC::B18TrafficVehicle trafficPerson_i : trafficVehicleVec){
      // for(int j = 0; j < size_gpu_part[i]/sizeof(LC::B18TrafficVehicle); j++){
        unsigned int init_intersectionId=trafficPerson_i.init_intersection;
        int targetPartition=vertexIdToPar[init_intersectionId];
        trafficVehicleVec_d_gpus[targetPartition][personIndex[targetPartition]++] = trafficPerson_i; 
    }
    delete[] personIndex; 
    personIndex = nullptr;
    for(int i = 0; i < ngpus; i++){
      cudaSetDevice(i);
      vehicles_vec[i] = new thrust::device_vector<LC::B18TrafficVehicle>(size_gpu_part[i]/sizeof(LC::B18TrafficVehicle));
      thrust::copy(trafficVehicleVec_d_gpus[i], trafficVehicleVec_d_gpus[i] + size_gpu_part[i]/sizeof(LC::B18TrafficVehicle), vehicles_vec[i]->begin());
    }
    
    
      
  }
  { 
    for(int i = 0; i < ngpus; i++){
      gpuErrchk(cudaSetDevice(i));
      // indexPathVec
      size_t sizeIn = indexPathVec_n[i].size() * sizeof(uint);
      indexPathVec_d_size = indexPathVec_n[i].size();
      if (firstInitialization) {
        gpuErrchk(cudaMalloc((void **) &indexPathVec_d[i], sizeIn));   // Allocate array on device
        gpuErrchk(cudaMemcpyAsync(indexPathVec_d[i], indexPathVec_n[i].data(), sizeIn, cudaMemcpyHostToDevice, streams[i]));
      }
    }
  }
  {
    for(int i = 0; i < ngpus; i++){
      cudaSetDevice(i);
      //edgeData
      size_t sizeD = edgesData_n[i].size() * sizeof(LC::B18EdgeData);
      edgesData_d_size[i] = edgesData_n[i].size();
      if (firstInitialization){
        gpuErrchk(cudaMalloc((void **) &edgesData_d[i], sizeD));   // Allocate array on device
        gpuErrchk(cudaMemcpyAsync(edgesData_d[i], edgesData_n[i].data(), sizeD, cudaMemcpyHostToDevice, streams[i]));
      } 
    }
  }
  {//laneMap
    for(int i = 0; i < ngpus; i++){
      cudaSetDevice(i);

      size_t sizeL = laneMap_n[i].size() * sizeof(uchar);
      laneMap_d_size[i] = laneMap_n[i].size();
      if (firstInitialization) 
      {
        gpuErrchk(cudaMalloc((void **) &laneMap_d[i], sizeL));   // Allocate array on device
        gpuErrchk(cudaMemcpyAsync(laneMap_d[i], laneMap_n[i].data(), sizeL, cudaMemcpyHostToDevice, streams[i]));
      }
      halfLaneMap_n[i] = laneMap_n[i].size() / 2;
    }    
  }

  {// intersections
    for(int i = 0; i < ngpus; i++){
      cudaSetDevice(i);
      size_t sizeI = intersections_n[i].size() * sizeof(LC::B18IntersectionData);
      if (firstInitialization){
        gpuErrchk(cudaMalloc((void **) &intersections_d[i], sizeI));   // Allocate array on device
        gpuErrchk(cudaMemcpyAsync(intersections_d[i], intersections_n[i].data(), sizeI, cudaMemcpyHostToDevice, streams[i]));   

      }
      
      size_t sizeT = trafficLights_n[i].size() * sizeof(uchar);//total number of lanes
      trafficLights_d_size[i] = trafficLights_n[i].size();
      if (firstInitialization) {
        gpuErrchk(cudaMalloc((void **) &trafficLights_d[i], sizeT));   // Allocate array on device
        gpuErrchk(cudaMemcpyAsync(trafficLights_d[i], trafficLights_n[i].data(), sizeT, cudaMemcpyHostToDevice, streams[i]));
      }
    }
      // cudaSetDevice(0);
      // gpuErrchk(cudaMemPrefetchAsync(trafficLights_d, sizeT, 0, streams[0]));
      // cudaSetDevice(1);
      // gpuErrchk(cudaMemPrefetchAsync(trafficLights_d, sizeT, 1, streams[1]));

  }
  {// ghost data structure
      // size_t sizeI = edges_num * sizeof(bool);
      if (firstInitialization){
      for(int i = 0; i < ngpus; i++){
        cudaSetDevice(i);
        
        gpuErrchk(cudaMalloc((void **) &vertexIdToPar_d[i], vertexIdToPar.size()*sizeof(int)));   // Allocate array on device
        // gpuErrchk(cudaMemcpy(vertexIdToPar_d[i], vertexIdToPar.data(), vertexIdToPar.size()*sizeof(int), cudaMemcpyHostToDevice));

        gpuErrchk(cudaMemcpyAsync(vertexIdToPar_d[i], vertexIdToPar.data(), vertexIdToPar.size()*sizeof(int), cudaMemcpyHostToDevice, streams[i]));
        gpuErrchk(cudaMalloc((void **) &vehicleToCopy_d[i], buffer_size*sizeof(uint)*2)); 
        gpuErrchk(cudaMalloc((void **) &vehicleToRemove_d[i], buffer_size*sizeof(uint))); 
        gpuErrchk(cudaMalloc((void **)&removeCursor_d[i], sizeof(uint))); 
        gpuErrchk(cudaMemset(removeCursor_d[i], 0, sizeof(uint)));
        gpuErrchk(cudaMalloc((void **)&copyCursor_d[i], sizeof(uint))); 
        gpuErrchk(cudaMemset(copyCursor_d[i], 0, sizeof(uint)));
        gpuErrchk(cudaMalloc((void **) &ghostLaneBuffer_d[i], buffer_lane_size*sizeof(uint)*4)); 
        gpuErrchk(cudaMalloc((void **)&ghostLaneCursor_d[i], sizeof(uint))); 
        gpuErrchk(cudaMemset(ghostLaneCursor_d[i], 0, sizeof(uint)));   
        gpuErrchk(cudaMalloc((void **) &laneToUpdateIndex_d[i], buffer_lane_size*sizeof(uint)));
        gpuErrchk(cudaMalloc((void **) &laneToUpdateValues_d[i], buffer_lane_size*sizeof(uint)));            
      }
      
      }
  }
  {// laneIdToLaneIdInGpu
    if (firstInitialization){
      
      
      for(int i=0;i<ngpus;i++){
        std::map<uint,uint>laneMapper=laneIdToLaneIdInGpu[i];
        uint largestKey=0;
       if (!laneMapper.empty()){
        auto largestKeyIter = laneMapper.rbegin(); // biggest key
        largestKey = largestKeyIter->first+1;
       } 
        laneIdMapper[i]= new uint[largestKey];
        std::fill_n(laneIdMapper[i], largestKey, -1);
        
        uint laneMapper_size=laneMapper.size();
        int j = 0;
        for (const auto& kv : laneMapper){
          laneIdMapper[i][kv.first]=kv.second;
        }
        cudaSetDevice(i); 
        gpuErrchk(cudaMalloc((void **) &laneIdMapper_d[i], largestKey*sizeof(uint))); 
        gpuErrchk(cudaMemcpyAsync(laneIdMapper_d[i], laneIdMapper[i], largestKey*sizeof(uint), cudaMemcpyHostToDevice, streams[i]));
      }
      

      
      // for (int i = 0; i < ngpus; ++i) {
      //   delete[] laneIdMapper[i];
      // }    
      // delete[] laneIdMapper;
    }
  }
  
  {
    for(int i = 0; i < ngpus; i++){
      cudaSetDevice(i);

      startTime = startTimeH * 3600.0f;
      uint numSamples = ceil(((endTimeH*3600.0f - startTimeH*3600.0f) / (deltaTime * numStepsPerSample * numStepsTogether))) + 1; //!!!
      accSpeedPerLinePerTimeInterval.clear();
      numVehPerLinePerTimeInterval.clear();
      accSpeedPerLinePerTimeInterval.resize(numSamples * trafficLights_n[i].size());
      numVehPerLinePerTimeInterval.resize(numSamples * trafficLights_n[i].size());
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
  // peer to peer
  {
    int canAccessPeer;
    for (int i = 0; i < ngpus; i++) {
      cudaSetDevice(i);
        for (int j = 0; j < ngpus; j++) {
            if (i != j) {
                cudaDeviceCanAccessPeer(&canAccessPeer, i, j);
                if (canAccessPeer) {
                  cudaDeviceEnablePeerAccess(j, 0);
                    printf("Peer2Peer support: %d-%d\n",i,j);
                }
            }
        }
    }   

  }
  for(int i = 0; i < ngpus; i++){
    cudaSetDevice(i);
    gpuErrchk(cudaStreamSynchronize(streams[i]));
    gpuErrchk(cudaStreamDestroy(streams[i]));
    printMemoryUsage();

  }
      
  cudaError_t error = cudaGetLastError();
  printf("CUDA error: %s\n", cudaGetErrorString(error));
  delete[] streams;
}

void b18InitCUDA(
  bool firstInitialization,
  std::vector<LC::B18TrafficVehicle>& trafficVehicleVec, 
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
    size_t size = trafficVehicleVec.size() * sizeof(LC::B18TrafficVehicle);
    // if (firstInitialization) gpuErrchk(cudaMalloc((void **) &trafficVehicleVec_d, size));   // Allocate array on device

    // gpuErrchk(cudaMemcpy(trafficVehicleVec_d, trafficVehicleVec.data(), size, cudaMemcpyHostToDevice));
    if (firstInitialization){
      gpuErrchk(cudaMallocManaged(&trafficVehicleVec_d, size));
      memcpy(trafficVehicleVec_d, trafficVehicleVec.data(), size);
    }
    // cudaSetDevice(0);
    // gpuErrchk(cudaMemPrefetchAsync(trafficVehicleVec_d, size, 0, streams[0]));
    // cudaSetDevice(1);
    // gpuErrchk(cudaMemPrefetchAsync(trafficVehicleVec_d, size, 1, streams[1]));

    // Calculate the size of each half
    num_people_gpu = int(trafficVehicleVec.size() / ngpus);
    for(int i = 0; i < ngpus; i++){
        size_gpu_part[i] = num_people_gpu * sizeof(LC::B18TrafficVehicle);
    }
    size_gpu_part[ngpus-1] = (trafficVehicleVec.size() - num_people_gpu *(ngpus-1)) * sizeof(LC::B18TrafficVehicle);

    // Allocate memory for each half on the respective GPU
    //LC::B18TrafficVehicle **trafficVehicleVec_d_gpus[ngpus];

    // Copy the first half to GPU 0 and the second half to GPU 1
    for(int i = 0; i < ngpus; i++){
      gpuErrchk(cudaSetDevice(i));
      gpuErrchk(cudaMallocManaged(&trafficVehicleVec_d_gpus[i], size_gpu_part[i]));
      // trafficVehicleVec.data() returns a pointer to the memory of the data of the struct object
      // struct supports plain assignment
      for(int j = 0; j < size_gpu_part[i]/sizeof(LC::B18TrafficVehicle); j++){
        trafficVehicleVec_d_gpus[i][j] = trafficVehicleVec[i * num_people_gpu + j]; 
      }
      gpuErrchk(cudaMemPrefetchAsync(trafficVehicleVec_d_gpus[i], size_gpu_part[i], i, streams[i]));
    }
  }
  { 
    for(int i = 0; i < ngpus; i++){
      gpuErrchk(cudaSetDevice(i));
      // indexPathVec
      size_t sizeIn = indexPathVec.size() * sizeof(uint);
      indexPathVec_d_size = indexPathVec.size();
      if (firstInitialization) {
        gpuErrchk(cudaMalloc((void **) &indexPathVec_d[i], sizeIn));   // Allocate array on device
        gpuErrchk(cudaMemcpyAsync(indexPathVec_d[i], indexPathVec.data(), sizeIn, cudaMemcpyHostToDevice, streams[i]));
      }
    }
  }
  {
    for(int i = 0; i < ngpus; i++){
      cudaSetDevice(i);
      //edgeData
      size_t sizeD = edgesData_d_size[i] * sizeof(LC::B18EdgeData);
      edgesData_d_size[i] = edgesData.size();
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
      laneMap_d_size[i] = laneMap.size();
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
      trafficLights_d_size[i] = trafficLights.size();
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

void b18updateStructuresCUDA(std::vector<LC::B18TrafficVehicle>& trafficVehicleVec,std::vector<uint> &indexPathVec,std::vector<LC::B18EdgeData>& edgesData){
  std::cout<< ">> b18updateStructuresCUDA" << std::endl;
  //indexPathVec
  cudaStream_t streams[ngpus];
  size_t sizeIn = indexPathVec.size() * sizeof(uint);
  size_t sizeD = edgesData.size() * sizeof(LC::B18EdgeData);
  size_t size = trafficVehicleVec.size() * sizeof(LC::B18TrafficVehicle);
  for(int i=0; i < ngpus; i++){
    cudaSetDevice(i);
    cudaStreamCreate( &streams[i] );
    // copy index path vector 
    cudaFree(indexPathVec_d[i]);
    indexPathVec_d_size = indexPathVec.size();
    gpuErrchk(cudaMalloc((void **) &indexPathVec_d[i], sizeIn));
    gpuErrchk(cudaMemcpyAsync(indexPathVec_d[i], indexPathVec.data(), sizeIn, cudaMemcpyHostToDevice, streams[i]));
    // copy edge data
    cudaFree(edgesData_d[i]);
    gpuErrchk(cudaMalloc((void **) &edgesData_d[i], sizeD));
    gpuErrchk(cudaMemcpyAsync(edgesData_d[i], edgesData.data(), sizeD, cudaMemcpyHostToDevice, streams[i]));
    // copy traffic person vector
    cudaFree(trafficVehicleVec_d_gpus[i]);
    
    gpuErrchk(cudaMallocManaged(&trafficVehicleVec_d_gpus[i], size_gpu_part[i]));
    for(int j = 0; j < size_gpu_part[i]/sizeof(LC::B18TrafficVehicle); j++){
        trafficVehicleVec_d_gpus[i][j] = trafficVehicleVec[i * num_people_gpu + j]; 
    }  
    gpuErrchk(cudaMemPrefetchAsync(trafficVehicleVec_d_gpus[i], size_gpu_part[i], i, streams[i]));
  }
  printMemoryUsage();
}
void b18updateStructuresCUDA_n(const std::vector<int>& vertexIdToPar,std::vector<LC::B18TrafficVehicle>& trafficVehicleVec,std::vector<uint> &indexPathVec,std::vector<LC::B18EdgeData> edgesData_n[],std::vector<personPath> allPathsInVertexes){
  std::cout<< ">> b18updateStructuresCUDA" << std::endl;
  //indexPathVec
  cudaStream_t *streams = new cudaStream_t[ngpus];
  size_t sizeIn = indexPathVec.size() * sizeof(uint);
  size_t size = trafficVehicleVec.size() * sizeof(LC::B18TrafficVehicle);
  //update size of vehicle on gpu(because of ghost)
  for (const personPath & aPersonPath: allPathsInVertexes){
    if(aPersonPath.pathInVertexes.size()>1){
      int initPar=vertexIdToPar[trafficVehicleVec[aPersonPath.person_id].init_intersection];
      int secondPar=vertexIdToPar[aPersonPath.pathInVertexes[1]];
      if(initPar!=secondPar){
        size_gpu_part[secondPar]+=sizeof(LC::B18TrafficVehicle);
      }
    }
  }
  for(int i=0; i < ngpus; i++){
    cudaSetDevice(i);
    cudaStreamCreate( &streams[i] );
    // copy index path vector 
    cudaFree(indexPathVec_d[i]);
    indexPathVec_d_size = indexPathVec.size();
    gpuErrchk(cudaMalloc((void **) &indexPathVec_d[i], sizeIn));
    gpuErrchk(cudaMemcpyAsync(indexPathVec_d[i], indexPathVec.data(), sizeIn, cudaMemcpyHostToDevice, streams[i]));
    // copy edge data
    cudaFree(edgesData_d[i]);
    size_t sizeD = edgesData_n[i].size() * sizeof(LC::B18EdgeData);
    gpuErrchk(cudaMalloc((void **) &edgesData_d[i], sizeD));
    gpuErrchk(cudaMemcpyAsync(edgesData_d[i], edgesData_n[i].data(), sizeD, cudaMemcpyHostToDevice, streams[i]));
    
    // copy traffic person vector
    delete vehicles_vec[i];
    delete[] trafficVehicleVec_d_gpus[i];
   trafficVehicleVec_d_gpus[i] = new LC::B18TrafficVehicle[size_gpu_part[i]/sizeof(LC::B18TrafficVehicle)];
  }
  
  uint* personIndex = new uint[ngpus]();
  for (const personPath & aPersonPath: allPathsInVertexes){
    int initPar=vertexIdToPar[trafficVehicleVec[aPersonPath.person_id].init_intersection];
    trafficVehicleVec_d_gpus[initPar][personIndex[initPar]++] = trafficVehicleVec[aPersonPath.person_id]; 
    if(aPersonPath.pathInVertexes.size()>1){
      int secondPar=vertexIdToPar[aPersonPath.pathInVertexes[1]];
      if(initPar!=secondPar){
        trafficVehicleVec_d_gpus[secondPar][personIndex[secondPar]++] = trafficVehicleVec[aPersonPath.person_id]; 
      }
    }
    
  }
    delete[] personIndex; 
    personIndex = nullptr;
    for(int i = 0; i < ngpus; i++){
      cudaSetDevice(i);
      std::cout<<"Vehicles on gpu "<<i<<": "<<size_gpu_part[i]/sizeof(LC::B18TrafficVehicle)<<std::endl;
      vehicles_vec[i] = new thrust::device_vector<LC::B18TrafficVehicle>(size_gpu_part[i]/sizeof(LC::B18TrafficVehicle));
      thrust::copy(trafficVehicleVec_d_gpus[i], trafficVehicleVec_d_gpus[i] + size_gpu_part[i]/sizeof(LC::B18TrafficVehicle), vehicles_vec[i]->begin());
    }
    for(int i = 0; i < ngpus; i++){
    cudaSetDevice(i);
    gpuErrchk(cudaStreamSynchronize(streams[i]));
    gpuErrchk(cudaStreamDestroy(streams[i]));
  }
    printMemoryUsage();
    cudaError_t error = cudaGetLastError();
    printf("CUDA error: %s\n", cudaGetErrorString(error));
    delete[] streams;
}

void b18FinishCUDA(void){
  cudaFree(trafficVehicleVec_d);
  for(int i=0; i < ngpus; i++){
    cudaSetDevice(i);
    cudaFree(indexPathVec_d);
    cudaFree(edgesData_d);
    cudaFree(laneMap_d);
    cudaFree(intersections_d);
    cudaFree(trafficLights_d);
    cudaFree(accSpeedPerLinePerTimeInterval_d);
    cudaFree(numVehPerLinePerTimeInterval_d);
  }
}
bool compareById(const LC::B18TrafficVehicle& a, const LC::B18TrafficVehicle& b) {
    return a.id < b.id;
}

void sortTrafficPersonsById(std::vector<LC::B18TrafficVehicle>& trafficVehicleVec) {
    
}
void b18GetDataCUDA(std::vector<LC::B18TrafficVehicle>& trafficVehicleVec, std::vector<LC::B18EdgeData> &edgesData){
  // copy back people
  int vehicle_size=0;
  for(int i=0; i < ngpus; i++){
    cudaSetDevice(i);
    vehicle_size+=vehicles_vec[i]->size();
  }
  trafficVehicleVec.resize(vehicle_size);
  // std::cout<<"size of vehicles_vec: "<<vehicle_size<<std::endl;
  int indexCursor=0;
  for(int i=0; i < ngpus; i++){
    cudaSetDevice(i);
    thrust::copy(vehicles_vec[i]->begin(), vehicles_vec[i]->end(), trafficVehicleVec.begin()+indexCursor);
    indexCursor+=vehicles_vec[i]->size();
  }
  
  // for(int i = 0; i < ngpus; i++){
  //     for (int j = 0; j < size_gpu_part[i]/sizeof(LC::B18TrafficVehicle); j++) {
  //     trafficVehicleVec_d[indexCursor++] = trafficVehicleVec_d_gpus[i][j];
  //   }
  // }
  // trafficVehicleVec.clear();
  // trafficVehicleVec.resize(indexCursor);
  // // cudaMemcpy(trafficVehicleVec.data(),trafficVehicleVec_d,indexCursor*sizeof(LC::B18TrafficVehicle),cudaMemcpyDeviceToHost);//cudaMemcpyHostToDevice
  // memcpy( trafficVehicleVec.data(),trafficVehicleVec_d, indexCursor*sizeof(LC::B18TrafficVehicle));
  std::sort(trafficVehicleVec.begin(), trafficVehicleVec.end(),
        [](const LC::B18TrafficVehicle& a, const LC::B18TrafficVehicle& b) {
            return a.id < b.id;
        }
    );

  // merge replicate
    for (size_t i = 0; i < trafficVehicleVec.size(); ++i) {
        if (i + 1 < trafficVehicleVec.size() && trafficVehicleVec[i].id == trafficVehicleVec[i + 1].id) {
            if (trafficVehicleVec[i] == trafficVehicleVec[i + 1]) {
                // If equal, then merge
                trafficVehicleVec.erase(trafficVehicleVec.begin() + i + 1);
                --i;
            } else {
                throw std::runtime_error("Error: Found different instances with the same id.");
            }
        }
    }


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
__device__ void getLaneIdToLaneIdInGpuValue(int* keys, int* values,int wholeLaneMap_size, int key, int &result) {
    for (int i = 0; i < wholeLaneMap_size; ++i) {
        if (keys[i] == key) {
            result = values[i];
            return;
        }
    }
}

 // Kernel that executes on the CUDA device
__global__ void kernel_trafficSimulation(
  int gpuIndex,
  int numPeople,
  float currentTime,
  uint mapToReadShift,
  uint mapToWriteShift,
  LC::B18TrafficVehicle *trafficVehicleVec,
  uint *indexPathVec,
  uint indexPathVec_d_size,
  LC::B18EdgeData* edgesData,
  uint edgesData_d_size,
  uchar *laneMap,
  uint laneMap_d_size,
  uint *laneMapper,
  LC::B18IntersectionData *intersections,
  uchar *trafficLights,
  uint trafficLights_d_size,
  float deltaTime,
  const parameters simParameters,
  int* vertexIdToPar_d,
  uint* vehicleToCopy,
  uint* vehicleToremove,
  uint* copyCursor,
  uint* removeCursor,
  uint* ghostLaneBuffer,
  uint* ghostLaneCursor
  )
  {
  int p = blockIdx.x * blockDim.x + threadIdx.x;
  if (p >= numPeople) return; //CUDA check (inside margins)
  assert( numPeople > p);
  if (trafficVehicleVec[p].active == 2) return; // trip finished
  if (trafficVehicleVec[p].time_departure > currentTime) return; //1.1 just continue waiting 
  // check that the current path index does not exceed the size of the path index vector
  assert(trafficVehicleVec[p].indexPathCurr < indexPathVec_d_size);
  if (indexPathVec[trafficVehicleVec[p].indexPathCurr] == END_OF_PATH) {
    trafficVehicleVec[p].active = 2; //finished
    return;
  }
  //2.1. check if person should still wait or should start
  if (trafficVehicleVec[p].active == 0) {
    //1.2 find first edge
    assert(trafficVehicleVec[p].indexPathInit != INIT_EDGE_INDEX_NOT_SET);
    trafficVehicleVec[p].indexPathCurr = trafficVehicleVec[p].indexPathInit; // reset index.
    uint indexFirstEdge = trafficVehicleVec[p].indexPathCurr;
    assert(indexFirstEdge < indexPathVec_d_size);
    // firstEdge convert to LaneIndex
    uint firstEdge = indexPathVec[indexFirstEdge];
    uint firstEdge_d = -1;
    if(firstEdge != END_OF_PATH){
      firstEdge_d=laneMapper[firstEdge];
      // getLaneIdToLaneIdInGpuValue(laneIdToLaneIdInGpu_d_keys, laneIdToLaneIdInGpu_d_values, wholeLaneMap_size,firstEdge,firstEdge_d); // turn overall edgeId(laneId) to edge(lane) index in edgesData[i]
      // if(firstEdge_d==-1){
      //   printf("%d %d %f %d: %u\n",p, gpuIndex,currentTime,trafficVehicleVec[p].id,firstEdge);
      // }
      assert(firstEdge_d!=-1);
    }
    
    trafficVehicleVec[p].last_time_simulated = currentTime;
    
    if (firstEdge == END_OF_PATH) {
      trafficVehicleVec[p].active = 2;
      return;
    }

    if (firstEdge_d >= edgesData_d_size) {
      printf("firstEdge %d is bigger than edgesData size %d\n", firstEdge, edgesData_d_size);
    }
    assert(firstEdge_d < edgesData_d_size);

    if(vertexIdToPar_d[edgesData[firstEdge_d].nextInters]!=gpuIndex){
      uint cursor = atomicAdd(removeCursor,1);
      vehicleToremove[cursor]=p;
      return;
    }
    //1.4 try to place it in middle of edge
    ushort numOfCells = ceil(edgesData[firstEdge_d].length);
    ushort initShift = (ushort) (0.5f * numOfCells); //number of cells it should be placed (half of road)

    uchar laneChar;
    bool placed = false;

    ushort numCellsEmptyToBePlaced = simParameters.s_0;
    ushort countEmptyCells = 0;
    for (ushort b = initShift; (b < numOfCells) && (!placed); b++) {
      ushort lN = edgesData[firstEdge_d].numLines - 1; //just right lane
      int laneMapPosition = mapToReadShift + kMaxMapWidthM * (firstEdge_d + lN) + b;
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
      trafficVehicleVec[p].numOfLaneInEdge = lN;
      trafficVehicleVec[p].posInLaneM = b; //m
      uchar vInMpS = (uchar) (trafficVehicleVec[p].v * 3); //speed in m/s *3 (to keep more precision
      int laneMapPosition2 = mapToWriteShift + kMaxMapWidthM * (firstEdge_d + lN) + b;
      assert(laneMapPosition2 < laneMap_d_size);
      if(laneMap[laneMapPosition2]!=vInMpS){
        int initPar=vertexIdToPar_d[trafficVehicleVec[p].init_intersection];
        if(initPar!=gpuIndex){// if on ghost edge, copy lane data to init par
          int cursor = atomicAdd(ghostLaneCursor,4);
          ghostLaneBuffer[cursor]=initPar;// target gpu index
          ghostLaneBuffer[cursor+1]=laneMapPosition2-firstEdge_d*kMaxMapWidthM-mapToWriteShift;// lane position
          ghostLaneBuffer[cursor+2]=firstEdge;// to be mapped to firstEdge_d in another gpu
          ghostLaneBuffer[cursor+3]=vInMpS;// target value
        }
      }
      laneMap[laneMapPosition2] = vInMpS;
      placed = true;
      break;
    }
    if (!placed) { //not posible to start now
      return;
    }
    trafficVehicleVec[p].v = 0;
    trafficVehicleVec[p].LC_stateofLaneChanging = 0;

    //1.5 active car
    trafficVehicleVec[p].active = 1;
    trafficVehicleVec[p].isInIntersection = 0;
    trafficVehicleVec[p].num_steps = 1;
    trafficVehicleVec[p].co = 0.0f;
    trafficVehicleVec[p].gas = 0.0f;
    
    assert(trafficVehicleVec[p].indexPathCurr + 1 < indexPathVec_d_size);
    if (indexPathVec[trafficVehicleVec[p].indexPathCurr + 1] != END_OF_PATH) {
      trafficVehicleVec[p].LC_initOKLanes = 0xFF;
      trafficVehicleVec[p].LC_endOKLanes = 0xFF;
    }
    trafficVehicleVec[p].path_length_gpu = 0;

    trafficVehicleVec[p].prevEdge = firstEdge;
    return;
  }

  bool ifPassIntersection=false;
  // set up next edge info
  uint indexCurrentEdge = trafficVehicleVec[p].indexPathCurr;
  assert(indexCurrentEdge < indexPathVec_d_size);
  uint currentEdge = indexPathVec[indexCurrentEdge];
  trafficVehicleVec[p].currentEdge=currentEdge;
  uint currentEdge_d=-1;
  // return;
  currentEdge_d=laneMapper[currentEdge];
  if(trafficVehicleVec[p].id==410){
  }
  // getLaneIdToLaneIdInGpuValue(laneIdToLaneIdInGpu_d_keys, laneIdToLaneIdInGpu_d_values, wholeLaneMap_size,currentEdge,currentEdge_d); //get edge index in edgesData_d
  if(currentEdge_d==-1){
    printf("gpu %d: %d %u %d %u\n",gpuIndex, trafficVehicleVec[p].id,currentEdge,indexCurrentEdge,trafficVehicleVec[p].prevEdge);
  }
  assert(currentEdge_d!=-1);
  assert(currentEdge_d < edgesData_d_size);
  
  uint indexNextEdge = trafficVehicleVec[p].indexPathCurr + 1;
  assert(indexNextEdge < indexPathVec_d_size);
  uint nextEdge = indexPathVec[indexNextEdge];
  uint nextEdge_d=-1;
  
  
  // if(nextEdge == END_OF_PATH )nextEdge_d=END_OF_PATH;
  if(vertexIdToPar_d[edgesData[currentEdge_d].nextInters]!=gpuIndex){
    uint cursor = atomicAdd(removeCursor,1);
    vehicleToremove[cursor]=p;
    return;
  }
  if(nextEdge != END_OF_PATH ){
    nextEdge_d=laneMapper[nextEdge];
  }
  else {
    nextEdge_d=END_OF_PATH;
  }
  assert(nextEdge_d < edgesData_d_size || nextEdge == END_OF_PATH);

  uint prevEdge = trafficVehicleVec[p].prevEdge;
  uint prevEdge_d=-1;
  prevEdge_d=laneMapper[prevEdge];
  // getLaneIdToLaneIdInGpuValue(laneIdToLaneIdInGpu_d_keys, laneIdToLaneIdInGpu_d_values, wholeLaneMap_size,prevEdge,prevEdge_d); 
  if(prevEdge_d!=-1){
    // printf("@@@@%d %u",prevEdge,trafficVehicleVec[p].init_intersection);
    assert(prevEdge_d < edgesData_d_size);
  }
  

  if (nextEdge != END_OF_PATH) {
    trafficVehicleVec[p].LC_initOKLanes = 0xFF;
    trafficVehicleVec[p].LC_endOKLanes = 0xFF;
  }

  //2. it is moving
  trafficVehicleVec[p].num_steps++;
  trafficVehicleVec[p].last_time_simulated = fmaxf(currentTime, trafficVehicleVec[p].last_time_simulated);

  //2.1 try to move
  float numMToMove;
  bool nextVehicleIsATrafficLight = false;
  

  //when we're on a new edge for the first time
  if (currentEdge == trafficVehicleVec[p].nextEdge) {
    trafficVehicleVec[p].end_time_on_prev_edge = currentTime - deltaTime;
    float elapsed_s = (trafficVehicleVec[p].end_time_on_prev_edge - trafficVehicleVec[p].start_time_on_prev_edge); //multiply by delta_time to get seconds elapsed (not half seconds)

    // We filter whenever elapsed_s == 0, which means the time granularity was not enough to measure the speed
    // We also filter whenever 0 > elapsed_s > 5, because it causes manual_v to turn extraordinarily high
    if(prevEdge_d!=-1){
      assert(prevEdge_d< edgesData_d_size);
      if (elapsed_s > MINIMUM_NUMBER_OF_CARS_TO_MEASURE_SPEED) {
        trafficVehicleVec[p].manual_v = edgesData[prevEdge_d].length / elapsed_s;
        edgesData[prevEdge_d].curr_iter_num_cars += 1;
        edgesData[prevEdge_d].curr_cum_vel += trafficVehicleVec[p].manual_v;
      }
    }
    

    trafficVehicleVec[p].start_time_on_prev_edge = currentTime;
    trafficVehicleVec[p].prevEdge = currentEdge;
  }
  trafficVehicleVec[p].nextEdge = nextEdge;
  
  
  // www.vwi.tu-dresden.de/~treiber/MicroApplet/IDM.html
  // IDM
  float thirdTerm = 0;
  // 2.1.1 Find front car
  int numCellsCheck = max(30.0f, trafficVehicleVec[p].v * deltaTime * 2); //30 or double of the speed*time
  
  // a) SAME LINE (BEFORE SIGNALING)
  bool found = false;
  bool noFirstInLaneBeforeSign = false; //use for stop control (just let 1st to pass) TODO(pavan): I DON'T GET THIS
  bool noFirstInLaneAfterSign = false; //use for stop control (just let 1st to pass)
  float s;
  float delta_v;
  uchar laneChar;
  ushort byteInLine = (ushort) floor(trafficVehicleVec[p].posInLaneM);
  ushort numOfCells = ceil((edgesData[currentEdge_d].length - intersectionClearance)); //intersectionClearance hardcoded to 7.8f - why?

  for (ushort b = byteInLine + 1; (b < numOfCells) && (!found) && (numCellsCheck > 0); b++, numCellsCheck--) {
    // ShiftRead + WIDTH * (width number * # lanes + # laneInEdge) + b  TODO(pavan): WHAT IS THIS?
    //TODO(pavan): double check what mapToReadShift is printing out
    assert(trafficVehicleVec[p].indexPathCurr < indexPathVec_d_size);
    const uint posToSample = mapToReadShift +
      kMaxMapWidthM *(currentEdge_d +
      (((int) (byteInLine / kMaxMapWidthM)) * edgesData[currentEdge_d].numLines) +
      trafficVehicleVec[p].numOfLaneInEdge) + b % kMaxMapWidthM;
    assert(posToSample < laneMap_d_size);
    laneChar = laneMap[posToSample];

    //TODO(pavan): Is this clause for when it is not at the intersection yet but it has found a car in front of it?
    if (laneChar != 0xFF) {
      s = ((float) (b - byteInLine)); //m
      delta_v = trafficVehicleVec[p].v - (laneChar / 3.0f); //laneChar is in 3*ms (to save space in array)
      found = true;
      // if(trafficVehicleVec[p].id==33){
      //       printf("%u %u %f \n",posToSample,laneMap[posToSample],currentTime);
      //     }
      noFirstInLaneBeforeSign = true; 
      break;
    }
  } 

  // NEXT LINE
  // e) MOVING ALONG IN THE NEXT EDGE
  if (!found && numCellsCheck > 0) { //check if in next line
    if ((nextEdge != END_OF_PATH) &&
      (edgesData[currentEdge_d].nextInters !=
        trafficVehicleVec[p].end_intersection)) { // we haven't arrived to destination (check next line)
      ushort nextEdgeLaneToBe = trafficVehicleVec[p].numOfLaneInEdge; //same lane

      //printf("trafficVehicleVec[p].numOfLaneInEdge %u\n",trafficVehicleVec[p].numOfLaneInEdge);
      assert(nextEdge_d < edgesData_d_size);
      if (nextEdgeLaneToBe >= edgesData[nextEdge_d].numLines) {
        nextEdgeLaneToBe = edgesData[nextEdge_d].numLines - 1; //change line if there are less roads
      }

      //printf("2trafficVehicleVec[p].numOfLaneInEdge %u\n",trafficVehicleVec[p].numOfLaneInEdge);
      ushort numOfCells = ceil(edgesData[nextEdge_d].length);

      for (ushort b = 0; (b < numOfCells) && (!found) && (numCellsCheck > 0); b++, numCellsCheck--) {
        const uint posToSample = mapToReadShift + kMaxMapWidthM * (nextEdge_d + nextEdgeLaneToBe) + b; // b18 not changed since we check first width
        assert(posToSample < laneMap_d_size);
        laneChar = laneMap[posToSample];

        if (laneChar != 0xFF) {
          s = ((float) (b)); //m
          delta_v = trafficVehicleVec[p].v - (laneChar / 3.0f);  // laneChar is in 3*ms (to save space in array)
          found = true;
          // if(trafficVehicleVec[p].id==33){
          //   printf("%u %u %f \n",posToSample,laneMap[posToSample],currentTime);
          // }
          break;
        }
      }
    }
    
  }

  
  float s_star;
  if (found && (delta_v > 0 || (delta_v==0 &&trafficVehicleVec[p].v==0))) { //car in front and slower than us
    // 2.1.2 calculate dv_dt
    // The following operation is taken from Designing Large-Scale Interactive Traffic Animations for Urban Modeling
    // Section 4.3.1. Car-Following Model formula (2)
    s_star = simParameters.s_0 + max(0.0f,
      (trafficVehicleVec[p].v * trafficVehicleVec[p].T + (trafficVehicleVec[p].v *
      delta_v) / (2 * sqrtf(trafficVehicleVec[p].a * trafficVehicleVec[p].b))));
    thirdTerm = powf(((s_star) / (s)), 2);
  }

  // The following operation is taken from Designing Large-Scale Interactive Traffic Animations for Urban Modeling
  // Section 4.3.1. Car-Following Model formula (1)
  // And also Architecture for Modular Microsimulation of Real Estate Markets and Transportation
  // Section 6.3.2 Per-vehicle and traffic control simulation formula (7)
  float dv_dt = trafficVehicleVec[p].a * (1.0f - std::pow((
    trafficVehicleVec[p].v / edgesData[currentEdge_d].maxSpeedMperSec), 4) - thirdTerm);

  // 2.1.3 update values
  numMToMove = max(0.0f, trafficVehicleVec[p].v * deltaTime + 0.5f * (dv_dt) * deltaTime * deltaTime);
  trafficVehicleVec[p].v += dv_dt * deltaTime;

  if (trafficVehicleVec[p].v < 0) {
    trafficVehicleVec[p].v = 0;
    dv_dt = 0.0f;
  }
  trafficVehicleVec[p].cum_v += trafficVehicleVec[p].v;

  if (calculatePollution && ((float(currentTime) == int(currentTime)))) { // enabled and each second (assuming deltaTime 0.5f)
    const float coStep = calculateCOStep(trafficVehicleVec[p].v);
    if (coStep > 0) {
      trafficVehicleVec[p].co += coStep;
    }
    trafficVehicleVec[p].gas += calculateGasConsumption(dv_dt, trafficVehicleVec[p].v);
  }

  if (trafficVehicleVec[p].v == 0) { //if not moving not do anything else
    ushort posInLineCells = (ushort) (trafficVehicleVec[p].posInLaneM);
    const uint posToSample = mapToWriteShift +
      kMaxMapWidthM * (currentEdge_d +
      (((int) (posInLineCells / kMaxMapWidthM)) * edgesData[currentEdge_d].numLines) +
      trafficVehicleVec[p].numOfLaneInEdge) +
      posInLineCells % kMaxMapWidthM;
    assert(posToSample < laneMap_d_size);
    if(laneMap[posToSample]!=0){
      // when we get here, the car's next intersection must be in current gpu, because we have access to nextEdge here
        int prePar=vertexIdToPar_d[edgesData[currentEdge_d].prevInters];
        int nextPar=vertexIdToPar_d[edgesData[currentEdge_d].nextInters];
        if(prePar!=nextPar && nextPar==gpuIndex){// if on ghost edge, copy lane data to pre par. Because pre par does not have this car any more
          uint cursor = atomicAdd(ghostLaneCursor,4);
          ghostLaneBuffer[cursor]=prePar;// target gpu index
          ghostLaneBuffer[cursor+1]=posToSample-currentEdge_d*kMaxMapWidthM-mapToWriteShift;// lane position
          ghostLaneBuffer[cursor+2]=currentEdge;// to be mapped to firstEdge_d in another gpu
          ghostLaneBuffer[cursor+3]=0;// target value
        }
    }
    laneMap[posToSample] = 0;
    return;
  }

  // COLOR
  trafficVehicleVec[p].color = p << 8;

  // STOP (check if it is a stop if it can go through)
  trafficVehicleVec[p].posInLaneM = trafficVehicleVec[p].posInLaneM + numMToMove;

  //2.2 close to intersection
  //2.2 check if change intersection
  if (trafficVehicleVec[p].posInLaneM > edgesData[currentEdge_d].length) { //reach intersection
    ifPassIntersection=true;
    numMToMove = trafficVehicleVec[p].posInLaneM - edgesData[currentEdge_d].length;
    trafficVehicleVec[p].posInLaneM = numMToMove;
    trafficVehicleVec[p].dist_traveled += edgesData[currentEdge_d].length;
    trafficVehicleVec[p].path_length_gpu++;

    //2.2.1 find next edge
    assert(indexCurrentEdge < indexPathVec_d_size);
    assert(currentEdge_d < edgesData_d_size);

    trafficVehicleVec[p].LC_stateofLaneChanging = 0;

    //2.1 check if end
    if (nextEdge != END_OF_PATH) {
      assert(nextEdge_d < edgesData_d_size);
      if (trafficVehicleVec[p].numOfLaneInEdge >= edgesData[nextEdge_d].numLines) {
        trafficVehicleVec[p].numOfLaneInEdge = edgesData[nextEdge_d].numLines - 1; //change line if there are less roads
      }

      //TODO: Test if the following line is doing the conversion wrong
      uchar vInMpS = (uchar) (trafficVehicleVec[p].v * 3); //speed in m/s to fit in uchar
      ushort posInLineCells = (ushort) (trafficVehicleVec[p].posInLaneM);
      const uint posToSample = mapToWriteShift + kMaxMapWidthM *
                              (nextEdge_d + (((int) (posInLineCells / kMaxMapWidthM)) *
                              edgesData[nextEdge_d].numLines) + trafficVehicleVec[p].numOfLaneInEdge) +
                              posInLineCells % kMaxMapWidthM;  // note the last % should not happen

      assert(posToSample < laneMap_d_size);
      if(laneMap[posToSample]!=vInMpS){
      // similiar
        int prePar=vertexIdToPar_d[edgesData[nextEdge_d].prevInters];
        int nextPar=vertexIdToPar_d[edgesData[nextEdge_d].nextInters];
        assert(prePar==gpuIndex);
        if(prePar!=nextPar){// if on ghost edge, copy lane data to next par.
          uint cursor = atomicAdd(ghostLaneCursor,4);
          ghostLaneBuffer[cursor]=nextPar;// target gpu index
          ghostLaneBuffer[cursor+1]=posToSample-nextEdge_d*kMaxMapWidthM-mapToWriteShift;// lane position
          ghostLaneBuffer[cursor+2]=nextEdge;// to be mapped to firstEdge_d in another gpu
          ghostLaneBuffer[cursor+3]=vInMpS;// target value
        }
    }
      laneMap[posToSample] = vInMpS;

      trafficVehicleVec[p].LC_initOKLanes = 0xFF;
      trafficVehicleVec[p].LC_endOKLanes = 0xFF;
    } else {
      trafficVehicleVec[p].active == 2;
    }
    trafficVehicleVec[p].indexPathCurr++;
    trafficVehicleVec[p].LC_stateofLaneChanging = 0;
 

  } else { //does not reach an intersection
    assert(indexCurrentEdge < indexPathVec_d_size);
    assert(indexNextEdge < indexPathVec_d_size);
    assert(currentEdge_d < edgesData_d_size);
    assert(nextEdge_d < edgesData_d_size || nextEdge == END_OF_PATH);

    // LANE CHANGING (happens when we are not reached the intersection)
    if (trafficVehicleVec[p].v > 3.0f && trafficVehicleVec[p].num_steps % 5 == 0) {
      //at least 10km/h to try to change lane
      //just check every (5 steps) 5 seconds

      // next thing is not a traffic light
      // skip if there is one lane (avoid to do this)
      // skip if it is the last edge
      if (!nextVehicleIsATrafficLight &&
        edgesData[currentEdge_d].numLines > 1 && nextEdge != END_OF_PATH) {
        ////////////////////////////////////////////////////
        // LC 1 update lane changing status
        if (trafficVehicleVec[p].LC_stateofLaneChanging == 0) {
          // 2.2-exp((x-1)^2)
          float x = trafficVehicleVec[p].posInLaneM / edgesData[currentEdge_d].length;

          if (x > 0.4f) { //just after 40% of the road
            float probabiltyMandatoryState = 2.2 - exp((x - 1) * (x - 1));

            //if (((float) qrand() / RAND_MAX) < probabiltyMandatoryState) {
            if ((((int) (x * 100) % 100) / 100.0f) < probabiltyMandatoryState) { // pseudo random number
              trafficVehicleVec[p].LC_stateofLaneChanging = 1;
            }
          }

        }

        // LC 2 NOT MANDATORY STATE
        if (trafficVehicleVec[p].LC_stateofLaneChanging == 0) {
          // discretionary change: v slower than the current road limit and deccelerating and moving
          if ((trafficVehicleVec[p].v < (edgesData[currentEdge_d].maxSpeedMperSec * 0.7f)) &&
            (dv_dt < 0) && trafficVehicleVec[p].v > 3.0f) {

            bool leftLane = trafficVehicleVec[p].numOfLaneInEdge >
              0; //at least one lane on the left
            bool rightLane = trafficVehicleVec[p].numOfLaneInEdge <
              edgesData[currentEdge_d].numLines - 1; //at least one lane

            if (leftLane && rightLane) {
              if (int(trafficVehicleVec[p].v) % 2 == 0) { // pseudo random
                leftLane = false;
              } else {
                rightLane = false;
              }
            }
            ushort laneToCheck;
            if (leftLane) {
              laneToCheck = trafficVehicleVec[p].numOfLaneInEdge - 1;
            } else {
              laneToCheck = trafficVehicleVec[p].numOfLaneInEdge + 1;
            }

            uchar v_a, v_b;
            float gap_a, gap_b;

            assert(currentEdge_d + trafficVehicleVec[p].numOfLaneInEdge < trafficLights_d_size);
            uchar trafficLightState = trafficLights[currentEdge_d + trafficVehicleVec[p].numOfLaneInEdge];
            calculateGapsLC(mapToReadShift, laneMap, trafficLightState,
              currentEdge_d + laneToCheck, edgesData[currentEdge_d].numLines,
              trafficVehicleVec[p].posInLaneM,
              edgesData[currentEdge_d].length, v_a, v_b, gap_a, gap_b, laneMap_d_size);

            if (gap_a == 1000.0f && gap_b == 1000.0f) { //lag and lead car very far
              trafficVehicleVec[p].numOfLaneInEdge = laneToCheck; // CHANGE LINE

            } else { // NOT ALONE
              float b1A = 0.05f, b2A = 0.15f;
              float b1B = 0.15f, b2B = 0.40f;
              // simParameters.s_0-> critical lead gap
              float g_na_D, g_bn_D;
              bool acceptLC = true;

              if (gap_a != 1000.0f) {
                g_na_D = max(simParameters.s_0, simParameters.s_0 + b1A * trafficVehicleVec[p].v + b2A *
                  (trafficVehicleVec[p].v - v_a * 3.0f));

                if (gap_a < g_na_D) { //gap smaller than critical gap
                  acceptLC = false;
                }
              }

              if (acceptLC && gap_b != 1000.0f) {
                g_bn_D = max(simParameters.s_0, simParameters.s_0 + b1B * v_b * 3.0f + b2B * (v_b * 3.0f - trafficVehicleVec[p].v));

                if (gap_b < g_bn_D) { //gap smaller than critical gap
                  acceptLC = false;
                }
              }

              if (acceptLC) {
                trafficVehicleVec[p].numOfLaneInEdge = laneToCheck; // CHANGE LINE
              }
            }
          }


        }// Discretionary

        // LC 3 *MANDATORY* STATE
        if (trafficVehicleVec[p].LC_stateofLaneChanging == 1) {
          // LC 3.1 Calculate the correct lanes
          if (trafficVehicleVec[p].LC_endOKLanes == 0xFF) {
            calculateLaneCarShouldBe(currentEdge_d, nextEdge_d, intersections,
              edgesData[currentEdge_d].nextIntersMapped,
              edgesData[currentEdge_d].numLines,
              trafficVehicleVec[p].LC_initOKLanes, trafficVehicleVec[p].LC_endOKLanes);

            if (trafficVehicleVec[p].LC_initOKLanes == 0 &&
              trafficVehicleVec[p].LC_endOKLanes == 0) {
            }
          }

          bool leftLane = false, rightLane = false;

          // LC 3.2 CORRECT LANES--> DICRETIONARY LC WITHIN
          if (trafficVehicleVec[p].numOfLaneInEdge >= trafficVehicleVec[p].LC_initOKLanes &&
            trafficVehicleVec[p].numOfLaneInEdge < trafficVehicleVec[p].LC_endOKLanes) {
            // for discretionary it should be under some circustances
            if ((trafficVehicleVec[p].v < (edgesData[currentEdge_d].maxSpeedMperSec * 0.7f)) &&
              (dv_dt < 0) && trafficVehicleVec[p].v > 3.0f) {
              leftLane =
                (trafficVehicleVec[p].numOfLaneInEdge > 0) && //at least one lane on the left
                (trafficVehicleVec[p].numOfLaneInEdge - 1 >= trafficVehicleVec[p].LC_initOKLanes)
                &&
                (trafficVehicleVec[p].numOfLaneInEdge - 1 < trafficVehicleVec[p].LC_endOKLanes);
              rightLane =
                (trafficVehicleVec[p].numOfLaneInEdge <
                  edgesData[currentEdge_d].numLines - 1) &&
                //at least one lane
                (trafficVehicleVec[p].numOfLaneInEdge + 1 >= trafficVehicleVec[p].LC_initOKLanes)
                &&
                (trafficVehicleVec[p].numOfLaneInEdge + 1 < trafficVehicleVec[p].LC_endOKLanes);
            }
          } else {
            // LC 3.3 INCORRECT LANES--> MANDATORY LC
            if (trafficVehicleVec[p].numOfLaneInEdge < trafficVehicleVec[p].LC_initOKLanes) {
              rightLane = true;
            } else {
              leftLane = true;
            }

            if (rightLane &&
              trafficVehicleVec[p].numOfLaneInEdge + 1 >= edgesData[currentEdge_d].numLines) {
              printf("ERROR: RT laneToCheck>=edgeNumLanes\n");
            }

            if (leftLane && trafficVehicleVec[p].numOfLaneInEdge == 0) {
              printf("ERROR %u: LT laneToCheck>=edgeNumLanes OK %u-%u NE %u\n",
                p, trafficVehicleVec[p].LC_initOKLanes, trafficVehicleVec[p].LC_endOKLanes,
                currentEdge_d);
            }
          }

          if (leftLane || rightLane) {

            // choose lane (if necessary)
            if (leftLane && rightLane) {
              if ((int) (trafficVehicleVec[p].posInLaneM) % 2 == 0) { //pseudo random
                leftLane = false;
              } else {
                rightLane = false;
              }
            }
            ushort laneToCheck;
            if (leftLane) {
              laneToCheck = trafficVehicleVec[p].numOfLaneInEdge - 1;
            } else {
              laneToCheck = trafficVehicleVec[p].numOfLaneInEdge + 1;
            }

            if (laneToCheck >= edgesData[currentEdge_d].numLines) {
              printf("ERROR: laneToCheck>=edgesData[currentEdge].numLines %u %u\n",
                laneToCheck, edgesData[currentEdge_d].numLines);
            }

            uchar v_a, v_b;
            float gap_a, gap_b;
            assert(currentEdge_d + trafficVehicleVec[p].numOfLaneInEdge < trafficLights_d_size);
            uchar trafficLightState = trafficLights[currentEdge_d + trafficVehicleVec[p].numOfLaneInEdge];
            calculateGapsLC(mapToReadShift, laneMap, trafficLightState,
              currentEdge_d + laneToCheck, edgesData[currentEdge_d].numLines,
              trafficVehicleVec[p].posInLaneM,
              edgesData[currentEdge_d].length, v_a, v_b, gap_a, gap_b, laneMap_d_size);

            if (gap_a == 1000.0f && gap_b == 1000.0f) { //lag and lead car very far
              trafficVehicleVec[p].numOfLaneInEdge = laneToCheck; // CHANGE LINE
            } else { // NOT ALONE
              float b1A = 0.05f, b2A = 0.15f;
              float b1B = 0.15f, b2B = 0.40f;
              float gamma = 0.000025;
              // simParameters.s_0-> critical lead gap
              float distEnd = edgesData[currentEdge_d].length - trafficVehicleVec[p].posInLaneM;
              float expTerm = (1 - exp(-gamma * distEnd * distEnd));

              float g_na_M, g_bn_M;
              bool acceptLC = true;

              if (gap_a != 1000.0f) {
                g_na_M = max(simParameters.s_0, simParameters.s_0 + (b1A * trafficVehicleVec[p].v + b2A *
                  (trafficVehicleVec[p].v - v_a * 3.0f)));

                if (gap_a < g_na_M) { //gap smaller than critical gap
                  acceptLC = false;
                }
              }

              if (acceptLC && gap_b != 1000.0f) {
                g_bn_M = max(simParameters.s_0, simParameters.s_0 + (b1B * v_b * 3.0f + b2B * (v_b * 3.0f -
                  trafficVehicleVec[p].v)));

                if (gap_b < g_bn_M) { //gap smaller than critical gap
                  acceptLC = false;
                }
              }

              if (acceptLC) {
                trafficVehicleVec[p].numOfLaneInEdge = laneToCheck; // CHANGE LINE
              }
            }
          }
        }// Mandatory
      }//at least two lanes and not stopped by traffic light
    }

    uchar vInMpS = (uchar) (trafficVehicleVec[p].v * 3); //speed in m/s to fit in uchar
    ushort posInLineCells = (ushort) (trafficVehicleVec[p].posInLaneM);
    const uint posToSample = mapToWriteShift +
      kMaxMapWidthM * (currentEdge_d + (((int) (posInLineCells / kMaxMapWidthM)) *
      edgesData[currentEdge_d].numLines) +
      trafficVehicleVec[p].numOfLaneInEdge) +
      posInLineCells % kMaxMapWidthM;
    assert(posToSample < laneMap_d_size);
    if(laneMap[posToSample]!=vInMpS){
      // when we get here, the car's next intersection must be in current gpu, because we have access to nextEdge here
        int prePar=vertexIdToPar_d[edgesData[currentEdge_d].prevInters];
        int nextPar=vertexIdToPar_d[edgesData[currentEdge_d].nextInters];
        assert(nextPar==gpuIndex);
        if(prePar!=nextPar){// if on ghost edge, copy lane data to pre par. Because pre par does not have this car any more
          assert(prePar!=-1);
          uint cursor = atomicAdd(ghostLaneCursor,4);
          ghostLaneBuffer[cursor]=prePar;// target gpu index
          ghostLaneBuffer[cursor+1]=posToSample-currentEdge_d*kMaxMapWidthM-mapToWriteShift;// lane position
          ghostLaneBuffer[cursor+2]=currentEdge;// to be mapped to currentEdge_d in another gpu
          ghostLaneBuffer[cursor+3]=vInMpS;// target value
        }
    }
    laneMap[posToSample] = vInMpS;
    
  }



  if(vertexIdToPar_d[edgesData[currentEdge_d].nextInters]!=gpuIndex){
    uint cursor = atomicAdd(removeCursor,1);
    vehicleToremove[cursor]=p;
    return;
  }
  
  if(ifPassIntersection && nextEdge!=END_OF_PATH){
    //when entering ghost zone, prepare to copy
      int targetGpuIndex=vertexIdToPar_d[edgesData[nextEdge_d].nextInters];
      if(targetGpuIndex!=gpuIndex){
      uint cursor = atomicAdd(copyCursor,2);
      vehicleToCopy[cursor]=p;
      vehicleToCopy[cursor+1]=targetGpuIndex;
    
    }
  }
  
    
}


__global__ void kernel_intersectionOneSimulation(
      uint numIntersections,
      float currentTime,
      LC::B18IntersectionData *intersections,
      uchar *trafficLights) {
  // if(blockIdx.x>218)printf("blockIdx: %d",blockIdx.x);
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
  LC::B18TrafficVehicle *trafficVehicleVec,
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

  if (trafficVehicleVec[p].active == 1 && trafficVehicleVec[p].indexPathCurr != END_OF_PATH) {
    assert(trafficVehicleVec[p].indexPathCurr < indexPathVec_d_size);
    int edgeNum = indexPathVec[trafficVehicleVec[p].indexPathCurr];

    assert(edgeNum + offset < accSpeedPerLinePerTimeInterval_d_size);
    accSpeedPerLinePerTimeInterval[edgeNum + offset] += trafficVehicleVec[p].v / 3.0f;

    assert(edgeNum + offset < numVehPerLinePerTimeInterval_d_size);
    numVehPerLinePerTimeInterval[edgeNum + offset]++;
  }
}
__global__ void kernel_resetPeople(
  int numPeople,
  LC::B18TrafficVehicle *trafficVehicleVec) {
  int p = blockIdx.x * blockDim.x + threadIdx.x;
  if (p < numPeople) {//CUDA check (inside margins)
    trafficVehicleVec[p].active = 0;
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
  kernel_resetPeople << < ceil(numPeople / 1024.0f), 1024 >> > (numPeople, trafficVehicleVec_d);
  
  for(int i = 0; i < ngpus; i++){
    cudaSetDevice(i);
    gpuErrchk(cudaMemset(&laneMap_d[i][0], -1, halfLaneMap_n[i]*sizeof(unsigned char)));
    gpuErrchk(cudaMemset(&laneMap_d[i][halfLaneMap_n[i]], -1, halfLaneMap_n[i]*sizeof(unsigned char)));
  }
}
// check whether the current index is in indices
struct is_in_indices {
    int *indices;
    int size;

    is_in_indices(int *_indices, int _size) : indices(_indices), size(_size) {}

    __device__ bool operator()(const int i) {
        int left = 0;
        int right = size - 1;
        while (left <= right) {
            int mid = left + (right - left) / 2;
            if (indices[mid] == i) {
                return true;
            } else if (indices[mid] < i) {
                left = mid + 1;
            } else {
                right = mid - 1;
            }
        }
        return false;
    }
};
void copy_task(int i, int j, std::vector<int>& indicesToCopy,int targetLoc){
  if(indicesToCopy.size()>0){
  std::sort(indicesToCopy.begin(), indicesToCopy.end());
  gpuErrchk(cudaSetDevice(i));
  thrust::device_vector<int> indicesToCopy_d(indicesToCopy.begin(), indicesToCopy.end());
  thrust::device_vector<LC::B18TrafficVehicle> output(indicesToCopy_d.size());
  // thrust::copy_if(thrust::device, vehicles_vec[i]->begin(), vehicles_vec[i]->end(), thrust::counting_iterator<int>(0), output.begin(), is_in_indices(thrust::raw_pointer_cast(indicesToCopy_d.data()), indicesToCopy_d.size()));
  auto perm_begin = thrust::make_permutation_iterator(vehicles_vec[i]->begin(), indicesToCopy_d.begin());
  auto perm_end = thrust::make_permutation_iterator(vehicles_vec[i]->begin(), indicesToCopy_d.end());
  thrust::copy(perm_begin, perm_end, output.begin());
  
  gpuErrchk(cudaSetDevice(j));
  LC::B18TrafficVehicle* target_ptr = thrust::raw_pointer_cast(vehicles_vec[j]->data()) + targetLoc;
  gpuErrchk(cudaMemcpyPeer(target_ptr, j, thrust::raw_pointer_cast(output.data()), i, output.size() * sizeof(LC::B18TrafficVehicle)));

}
}
void remove_task(int i, std::vector<int>& ToRemove) {
    gpuErrchk(cudaSetDevice(i));
    if(ToRemove.size()>0){
        // Assert that ToRemove has no duplicates
        assert(std::adjacent_find(ToRemove.begin(), ToRemove.end()) == ToRemove.end());
        std::sort(ToRemove.begin(), ToRemove.end());
        // Assert that the indices in ToRemove are within bounds
        assert(ToRemove.back() < vehicles_vec[i]->size());
        thrust::device_vector<int> ToRemove_d = ToRemove;
        std::vector<int> indices_from;
        std::vector<int> indices_to;
        // delete elements by move the last element to absence
        int currentIndOfIndices=0;
        for(int j=vehicles_vec[i]->size()-1;j>=0;j--){
          if(currentIndOfIndices>=ToRemove.size() ||j<ToRemove[currentIndOfIndices])break;
          // if j not in indices
          if(std::find(ToRemove.begin(), ToRemove.end(), j) == ToRemove.end()){
            indices_to.push_back(ToRemove[currentIndOfIndices]);
            indices_from.push_back(j);
              // (*vehicles_vec[i])[ToRemove_d[currentIndOfIndices]]=(*vehicles_vec[i])[j];
              currentIndOfIndices++;
          }
        }
        if(indices_to.size()>0){
        thrust::device_vector<int> indices_from_d(indices_from.begin(),indices_from.end());
        thrust::device_vector<int> indices_to_d(indices_to.begin(),indices_to.end());
        //get last elements to be removed and their target indices
        thrust::device_vector<LC::B18TrafficVehicle> toMove(indices_from_d.size());
        auto perm_begin = thrust::make_permutation_iterator(vehicles_vec[i]->begin(), indices_from_d.begin());
        auto perm_end = thrust::make_permutation_iterator(vehicles_vec[i]->begin(), indices_from_d.end());
        thrust::copy(perm_begin, perm_end, toMove.begin());
        //move
        thrust::scatter(thrust::device,toMove.begin(), toMove.end(), indices_to_d.begin(), vehicles_vec[i]->begin());
        }
        // resize
        vehicles_vec[i]->resize(vehicles_vec[i]->size() - ToRemove_d.size());

      }
}
__global__ void updateLaneMap(uchar *laneMap, uint size, uint laneMap_d_size, uint *updateLaneIndex, uint *updateLaneValues) {
    int index = threadIdx.x + blockIdx.x * blockDim.x;
    if(index<size){
      assert(updateLaneValues[index] >= 0 && updateLaneValues[index] <= 255);
      assert(updateLaneIndex[index]<laneMap_d_size);
      laneMap[updateLaneIndex[index]] = static_cast<uint8_t>(updateLaneValues[index]);

    }
    
}
void b18SimulateTrafficCUDA(float currentTime,
  uint numPeople,
  uint numIntersections_n[],
  float deltaTime,
  const parameters simParameters,
  int numBlocks,
  int threadsPerBlock) {
  intersectionBench.startMeasuring();
  const uint numStepsTogether = 12; //change also in density (10 per hour)
  // 1. CHANGE MAP: set map to use and clean the other
  // cudaStream_t streams[ngpus];
  for(int i = 0; i < ngpus; i++){
    // cudaStreamCreate(&streams[i]);
    cudaSetDevice(i);
    if (readFirstMapC==true) {
      mapToReadShift_n[i]=0;
      mapToWriteShift_n[i]=halfLaneMap_n[i];
      gpuErrchk(cudaMemset(&laneMap_d[i][halfLaneMap_n[i]], -1, halfLaneMap_n[i]*sizeof(unsigned char)));//clean second half
    } 
    else {
      mapToReadShift_n[i]=halfLaneMap_n[i];
      mapToWriteShift_n[i]=0;
      gpuErrchk(cudaMemset(&laneMap_d[i][0], -1, halfLaneMap_n[i]*sizeof(unsigned char)));//clean first half
    }
  }
  readFirstMapC=!readFirstMapC;//next iteration invert use
//  cudaError_t error = cudaGetLastError();
//         printf("CUDA error: %s\n", cudaGetErrorString(error));
  // Simulate intersections.
  for(int i = 0; i < ngpus; i++){
    cudaSetDevice(i);
    if(numIntersections_n[i]>0){
      kernel_intersectionOneSimulation << < ceil(numIntersections_n[i] / 512.0f), 512 >> > (numIntersections_n[i], currentTime, intersections_d[i], trafficLights_d[i]);
      gpuErrchk(cudaPeekAtLastError());
    }
  }
  intersectionBench.stopMeasuring();
  
  peopleBench.startMeasuring();
  // Simulate people.
  // #pragma omp parallel for
        // for(int i = 0; i < 2; i++) {
  
  //printf("Number of people per GPU : %i ", numPeople_gpu);

  std::vector<std::vector<int>> ToCopy(ngpus);
  std::vector<std::vector<int>> ToRemove(ngpus);
  std::vector<std::vector<int>> ghostLaneBuffer(ngpus);
  for(int i = 0; i < ngpus; i++){
    cudaSetDevice(i);
    int numPeople_gpu = vehicles_vec[i]->size();
    LC::B18TrafficVehicle* vehicles_ptr = thrust::raw_pointer_cast((*vehicles_vec[i]).data());
    if(numPeople_gpu>0){
      kernel_trafficSimulation <<<  ceil(numPeople_gpu/ 384.0f), threadsPerBlock>> >
      (i,numPeople_gpu, currentTime, mapToReadShift_n[i],
      mapToWriteShift_n[i],vehicles_ptr, indexPathVec_d[i], indexPathVec_d_size,
      edgesData_d[i], edgesData_d_size[i], laneMap_d[i], laneMap_d_size[i], laneIdMapper_d[i],
      intersections_d[i], trafficLights_d[i], trafficLights_d_size[i], deltaTime, simParameters,
      vertexIdToPar_d[i],vehicleToCopy_d[i],vehicleToRemove_d[i],copyCursor_d[i],removeCursor_d[i],ghostLaneBuffer_d[i],ghostLaneCursor_d[i]);
    }
    gpuErrchk(cudaPeekAtLastError());
    }
    // std::ofstream outFile("gpu_usage_sim_time.txt", std::ios::app);
    
    // auto realTime = std::chrono::system_clock::now();
    // std::time_t t = std::chrono::system_clock::to_time_t(realTime);
    // outFile <<currentTime<<","<< std::put_time(std::localtime(&t), "%Y-%m-%d %H:%M:%S") << std::endl;
    // outFile.close();
    for(int i = 0; i < ngpus; i++){
    cudaSetDevice(i);
    gpuErrchk(cudaDeviceSynchronize());
    }
    std::vector<int> currentLoc(ngpus,0);//current target copy beginning index of vehicles_vec
    int commu_times=0;
    int lane_update_size=0;
    for(int i = 0; i < ngpus; i++){
      cudaSetDevice(i);
      currentLoc[i]=vehicles_vec[i]->size();
      gpuErrchk(cudaMemcpy(&copyCursor[i], copyCursor_d[i], sizeof(int), cudaMemcpyDeviceToHost));
      gpuErrchk(cudaMemcpy(&removeCursor[i], removeCursor_d[i], sizeof(int), cudaMemcpyDeviceToHost));
      gpuErrchk(cudaMemcpy(&ghostLaneCursor[i], ghostLaneCursor_d[i], sizeof(int), cudaMemcpyDeviceToHost));
      ToCopy[i].resize(copyCursor[i]);
      ToRemove[i].resize(removeCursor[i]);
      ghostLaneBuffer[i].resize(ghostLaneCursor[i]);
      gpuErrchk(cudaMemcpy(ToCopy[i].data(), vehicleToCopy_d[i], copyCursor[i] * sizeof(int), cudaMemcpyDeviceToHost));
      gpuErrchk(cudaMemcpy(ToRemove[i].data(), vehicleToRemove_d[i], removeCursor[i] * sizeof(int), cudaMemcpyDeviceToHost));
      gpuErrchk(cudaMemcpy(ghostLaneBuffer[i].data(), ghostLaneBuffer_d[i], ghostLaneCursor[i]*sizeof(int), cudaMemcpyDeviceToHost));
      if(copyCursor[i]>0||removeCursor[i]>0){
        commu_times+=copyCursor[i]/2+removeCursor[i];
      }
      lane_update_size+=ghostLaneCursor[i]/4;
    }
  
    if(lane_update_size>0){
      std::vector<std::vector<int>> laneToUpdateIndex(ngpus);
    std::vector<std::vector<int>> laneToUpdateValues(ngpus);
    for(int i = 0;i < ngpus;i++){
      for(int j = 0; j < ghostLaneCursor[i]; j+=4){
        if(i==j)continue;
      // target position, value
      int targetGpuIndex=ghostLaneBuffer[i][j];
      int targetPosition=ghostLaneBuffer[i][j+1]+laneIdMapper[targetGpuIndex][ghostLaneBuffer[i][j+2]]*kMaxMapWidthM+mapToWriteShift_n[targetGpuIndex];
      laneToUpdateIndex[targetGpuIndex].push_back(targetPosition);
      laneToUpdateValues[targetGpuIndex].push_back(ghostLaneBuffer[i][j+3]);
      }
    }
    for(int i = 0;i < ngpus;i++){
      uint updateSize=laneToUpdateIndex[i].size();
      if(updateSize>0){
        cudaSetDevice(i);
        gpuErrchk(cudaMemcpy(laneToUpdateIndex_d[i], laneToUpdateIndex[i].data(), updateSize*sizeof(int), cudaMemcpyHostToDevice));
        gpuErrchk(cudaMemcpy(laneToUpdateValues_d[i], laneToUpdateValues[i].data(), updateSize*sizeof(int), cudaMemcpyHostToDevice));
        int blockNum = (laneToUpdateIndex[i].size()+ threadsPerBlock - 1) / threadsPerBlock;
        updateLaneMap<<<blockNum, threadsPerBlock>>>(laneMap_d[i], updateSize, laneMap_d_size[i],laneToUpdateIndex_d[i],laneToUpdateValues_d[i]);
        gpuErrchk(cudaPeekAtLastError());
      }
    }
  }
  if(commu_times>0){
      std::ofstream outFile("commu_times.txt", std::ios::app);
      outFile << commu_times << "\n";
      outFile.close();
    // select vehicles to be copied
    std::vector<std::vector<int>> indicesToCopy(ngpus*ngpus);
    std::vector<int> targetLoc(ngpus*ngpus, -1);// target copy beginning index of vehicles_vec, i-j -> i*ngpus+j
    
    // for(int i = 0; i < ngpus; i++){
    // cudaSetDevice(i);
    // gpuErrchk(cudaDeviceSynchronize());
    // }
    for(int i = 0;i < ngpus;i++){
      for(int j = 0; j < ngpus; j++){
        if(i==j)continue;
        // copy from gpu[i] to gpu[j]
        targetLoc[i*ngpus+j]=currentLoc[j];   
        for(int k=0;k<copyCursor[i];k+=2){
            if(ToCopy[i][k+1] == j){
              indicesToCopy[i*ngpus+j].push_back(ToCopy[i][k]);
            }
        }
        currentLoc[j] += indicesToCopy[i*ngpus+j].size();       
      }
    }
    for(int i = 0;i < ngpus;i++){
      cudaSetDevice(i);
      vehicles_vec[i]->resize(currentLoc[i]);   
    }
    std::vector<std::thread> copy_threads;
    for (int i = 0; i < ngpus; ++i)
    for (int j = 0; j < ngpus; ++j) {
      if(i!=j && targetLoc[i*ngpus+j]!=-1&&indicesToCopy[i*ngpus+j].size()>0)
        copy_threads.emplace_back(copy_task, i,j,std::ref(indicesToCopy[i*ngpus+j]),targetLoc[i*ngpus+j]); 
    }
    for (auto& t : copy_threads) {
        t.join();
    }
     std::vector<std::thread> threads;
    for (int i = 0; i < ngpus; ++i) {
      if(ToRemove[i].size()>0)
        threads.emplace_back(remove_task, i,std::ref(ToRemove[i])); 
    }
    for (auto& t : threads) {
        t.join();
    }
    
  }


  for(int i = 0; i < ngpus; i++){
    cudaSetDevice(i); 
    gpuErrchk(cudaMemset(copyCursor_d[i], 0, sizeof(int)));
    gpuErrchk(cudaMemset(removeCursor_d[i], 0, sizeof(int)));
    gpuErrchk(cudaMemset(ghostLaneCursor_d[i], 0, sizeof(int)));
  }

     
  peopleBench.stopMeasuring();

        // }

}


