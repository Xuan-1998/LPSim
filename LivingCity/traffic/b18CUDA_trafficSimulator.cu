


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
#include <map>
#include <algorithm> 
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
#define max_ghost_cars 20000
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
LC::B18TrafficPersonModify ** trafficPersonModify= new LC::B18TrafficPersonModify*[ngpus];
int num_people_gpu;
uint **indexPathVec_d = new uint*[ngpus];
uint indexPathVec_d_size;
LC::B18EdgeData **edgesData_d = new LC::B18EdgeData*[ngpus];
uint *edgesData_d_size= new uint[ngpus];
uint *laneMap_d_size= new uint[ngpus];
uint * trafficLights_d_size= new uint[ngpus];
uint accSpeedPerLinePerTimeInterval_d_size;
uint numVehPerLinePerTimeInterval_d_size;
size_t size_gpu_part[ngpus]; 
__constant__ bool calculatePollution = true;
__constant__ float cellSize = 1.0f;

uchar **laneMap_d = new uchar*[ngpus];
uchar **laneMap_d_gpus = new uchar*[ngpus];
int** laneIdMapper_d=new int*[ngpus];
int** vertexIdToPar_d= new int*[ngpus];
bool readFirstMapC=true;
uint mapToReadShift;
uint *mapToReadShift_n= new uint[ngpus];
uint mapToWriteShift;
uint *mapToWriteShift_n= new uint[ngpus];
uint halfLaneMap;
uint *halfLaneMap_n = new uint[ngpus];
float startTime;
int *ifCommu= new int[ngpus];
int **ifCommu_d = new int*[ngpus];
// std::map<int,std::vector<LC::B18TrafficPerson> >personToCopy;
// std::map<int,std::vector<int> >personToRemove;//eg: 1->{1,3,5},2->{9},3->{} (gpuIndex->personList)

LC::B18IntersectionData **intersections_d  = new LC::B18IntersectionData*[ngpus];
uchar **trafficLights_d = new uchar*[ngpus];

float* accSpeedPerLinePerTimeInterval_d;
float* numVehPerLinePerTimeInterval_d;
void b18InitCUDA_n(
  bool firstInitialization,
  const std::vector<int>& vertexIdToPar,
  int edges_num,
  std::map<int, int>laneIdToLaneIdInGpu[],
  std::vector<LC::B18TrafficPerson>& trafficPersonVec, 
  std::vector<uint> indexPathVec_n[], 
  std::vector<LC::B18EdgeData> edgesData_n[], 
  std::vector<uchar> laneMap_n[], 
  std::vector<uchar> trafficLights_n[], 
  std::vector<LC::B18IntersectionData> intersections_n[],
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
        // size_gpu_part[i] = num_people_gpu * sizeof(LC::B18TrafficPerson);
        size_gpu_part[i]=0;
    }
    // size_gpu_part[ngpus-1] = (trafficPersonVec.size() - num_people_gpu *(ngpus-1)) * sizeof(LC::B18TrafficPerson);

    // Allocate memory for each half on the respective GPU
    //LC::B18TrafficPerson **trafficPersonVec_d_gpus[ngpus];

    // Copy the first half to GPU 0 and the second half to GPU 1

    // compute initial size of trafficPerson on each gpu
    for(const LC::B18TrafficPerson trafficPerson_i : trafficPersonVec){
      unsigned int init_intersectionId=trafficPerson_i.init_intersection;
      int targetPartition=vertexIdToPar[init_intersectionId];
      size_gpu_part[targetPartition]+= sizeof(LC::B18TrafficPerson);
    }
    for(int i = 0; i < ngpus; i++){
      gpuErrchk(cudaSetDevice(i));
      gpuErrchk(cudaMallocManaged(&trafficPersonVec_d_gpus[i], size_gpu_part[i]));
      gpuErrchk(cudaMallocManaged(&trafficPersonModify[i], size_gpu_part[i]/sizeof(LC::B18TrafficPerson)*sizeof(LC::B18TrafficPersonModify)));
      for(int j=0;j<size_gpu_part[i]/sizeof(LC::B18TrafficPerson);j++){
        trafficPersonModify[i][j].ifToCopy = false;
        trafficPersonModify[i][j].ifToRemove = false;
        trafficPersonModify[i][j].gpuIndexToCopy = -1;
      }
      
    }
    int* personIndex = new int[ngpus]();
    for(const LC::B18TrafficPerson trafficPerson_i : trafficPersonVec){
      // for(int j = 0; j < size_gpu_part[i]/sizeof(LC::B18TrafficPerson); j++){
        unsigned int init_intersectionId=trafficPerson_i.init_intersection;
        int targetPartition=vertexIdToPar[init_intersectionId];
        trafficPersonVec_d_gpus[targetPartition][personIndex[targetPartition]++] = trafficPerson_i; 
    }
    delete[] personIndex; 
    personIndex = nullptr;
    for(int i = 0; i < ngpus; i++){
      if(size_gpu_part[i]>0){
      gpuErrchk(cudaMemPrefetchAsync(trafficPersonVec_d_gpus[i], size_gpu_part[i], i, streams[i]));
      gpuErrchk(cudaMemPrefetchAsync(trafficPersonModify[i], size_gpu_part[i]/sizeof(LC::B18TrafficPerson)*sizeof(LC::B18TrafficPersonModify), i, streams[i]));
 
      }
         }
    
    
      
  }
  { 
    for(int i = 0; i < ngpus; i++){
      gpuErrchk(cudaSetDevice(i));
      // indexPathVec
      size_t sizeIn = indexPathVec_n[i].size() * sizeof(uint);
      indexPathVec_d_size = indexPathVec_n[i].size();
      if (firstInitialization) {
        gpuErrchk(cudaMalloc(&indexPathVec_d[i], sizeIn));   // Allocate array on device
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
      for(int i = 0; i < ngpus; i++){
        cudaSetDevice(i);
        if (firstInitialization){
          gpuErrchk(cudaMalloc((void **) &vertexIdToPar_d[i], vertexIdToPar.size()*sizeof(int)));   // Allocate array on device
          gpuErrchk(cudaMemcpyAsync(vertexIdToPar_d[i], vertexIdToPar.data(), vertexIdToPar.size()*sizeof(int), cudaMemcpyHostToDevice, streams[i]));
          gpuErrchk(cudaMalloc((void **)&ifCommu_d[i], sizeof(int)));
          gpuErrchk(cudaMemset(ifCommu_d[i], 0, sizeof(int)));
      }
        // gpuErrchk(cudaMallocManaged(&personToCopy_d, max_ghost_cars * sizeof(uint64_t )));
        // for (size_t i = 0; i < max_ghost_cars; ++i) {
        //     personToCopy_d[i] = UINT64_MAX;
        // }
        // gpuErrchk(cudaMallocManaged(&personToRemove_d, max_ghost_cars * sizeof(uint64_t )));
        // for (size_t i = 0; i < max_ghost_cars; ++i) {
        //     personToRemove_d[i] = UINT64_MAX;
        // }
      }
  }
  {// laneIdToLaneIdInGpu
    if (firstInitialization){
      int** laneIdMapper= new int*[ngpus];
      
      for(int i=0;i<ngpus;i++){
        // 
        laneIdMapper[i]= new int[laneMap_d_size[i]];
        std::fill_n(laneIdMapper[i], laneMap_d_size[i], -1);
        std::map<int,int>laneMapper=laneIdToLaneIdInGpu[i];
        int laneMapper_size=laneMapper.size();
        int j = 0;
        for (const auto& kv : laneMapper){
          laneIdMapper[i][kv.first]=kv.second;
        }
        cudaSetDevice(i); 
        gpuErrchk(cudaMalloc((void **) &laneIdMapper_d[i], laneMap_d_size[i]*sizeof(int))); 
        gpuErrchk(cudaMemcpyAsync(laneIdMapper_d[i], laneIdMapper[i], laneMap_d_size[i]*sizeof(int), cudaMemcpyHostToDevice, streams[i]));
      }

      
      for (int i = 0; i < ngpus; ++i) {
        delete[] laneIdMapper[i];
      }    
      delete[] laneIdMapper;
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
  for(int i = 0; i < ngpus; i++){
    cudaSetDevice(i);
    gpuErrchk(cudaStreamSynchronize(streams[i]));
    printMemoryUsage();
  }
  cudaError_t error = cudaGetLastError();
printf("CUDA error: %s\n", cudaGetErrorString(error));
}

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
void b18updateStructuresCUDA_n(const std::vector<int>& vertexIdToPar,std::vector<LC::B18TrafficPerson>& trafficPersonVec,std::vector<uint> &indexPathVec,std::vector<LC::B18EdgeData> edgesData_n[],std::vector<personPath> allPathsInVertexes){
  std::cout<< ">> b18updateStructuresCUDA" << std::endl;
  //indexPathVec
  cudaStream_t streams[ngpus];
  size_t sizeIn = indexPathVec.size() * sizeof(uint);
  indexPathVec_d_size = indexPathVec.size();
  size_t size = trafficPersonVec.size() * sizeof(LC::B18TrafficPerson);
  //update size of vehicle on gpu(because of ghost)
  for (const personPath & aPersonPath: allPathsInVertexes){
    if(aPersonPath.pathInVertexes.size()>1){
      int initPar=vertexIdToPar[trafficPersonVec[aPersonPath.person_id].init_intersection];
      int secondPar=vertexIdToPar[aPersonPath.pathInVertexes[1]];
      if(initPar!=secondPar){
        size_gpu_part[secondPar]+=sizeof(LC::B18TrafficPerson);
      }
    }
  }
  for(int i=0; i < ngpus; i++){
    cudaSetDevice(i);
    cudaStreamCreate( &streams[i] );
    // copy index path vector 
    cudaFree(indexPathVec_d[i]);
    gpuErrchk(cudaMalloc((void **) &indexPathVec_d[i], sizeIn));
    gpuErrchk(cudaMemcpyAsync(indexPathVec_d[i], indexPathVec.data(), sizeIn, cudaMemcpyHostToDevice, streams[i]));
    // copy edge data
    cudaFree(edgesData_d[i]);
    size_t sizeD = edgesData_n[i].size() * sizeof(LC::B18EdgeData);
    gpuErrchk(cudaMalloc((void **) &edgesData_d[i], sizeD));
    gpuErrchk(cudaMemcpyAsync(edgesData_d[i], edgesData_n[i].data(), sizeD, cudaMemcpyHostToDevice, streams[i]));
    // copy traffic person vector
    cudaFree(trafficPersonVec_d_gpus[i]);
    gpuErrchk(cudaMallocManaged(&trafficPersonVec_d_gpus[i], size_gpu_part[i]));
    // copy trafficPersonModify
    cudaFree(trafficPersonModify[i]);
    if(size_gpu_part[i]>0)gpuErrchk(cudaMallocManaged(&trafficPersonModify[i], size_gpu_part[i]/sizeof(LC::B18TrafficPerson)*sizeof(LC::B18TrafficPersonModify)));
    for(int j=0;j<size_gpu_part[i]/sizeof(LC::B18TrafficPerson);j++){
      trafficPersonModify[i][j].ifToCopy = false;
      trafficPersonModify[i][j].ifToRemove = false;
      trafficPersonModify[i][j].gpuIndexToCopy = -1;
    }
  }
  
  int* personIndex = new int[ngpus]();
  for (const personPath & aPersonPath: allPathsInVertexes){
    int initPar=vertexIdToPar[trafficPersonVec[aPersonPath.person_id].init_intersection];
    trafficPersonVec_d_gpus[initPar][personIndex[initPar]++] = trafficPersonVec[aPersonPath.person_id]; 
    if(aPersonPath.pathInVertexes.size()>1){
      int secondPar=vertexIdToPar[aPersonPath.pathInVertexes[1]];
      if(initPar!=secondPar){
        trafficPersonVec_d_gpus[secondPar][personIndex[secondPar]++] = trafficPersonVec[aPersonPath.person_id]; 
      }
    }
    
  }
    delete[] personIndex; 
    personIndex = nullptr;
    for(int i = 0; i < ngpus; i++){
      if(size_gpu_part[i]>0){
      gpuErrchk(cudaMemPrefetchAsync(trafficPersonVec_d_gpus[i], size_gpu_part[i], i, streams[i]));
      gpuErrchk(cudaMemPrefetchAsync(trafficPersonModify[i], size_gpu_part[i]/sizeof(LC::B18TrafficPerson)*sizeof(LC::B18TrafficPersonModify), i, streams[i]));
    }}
    for(int i = 0; i < ngpus; i++){
    cudaSetDevice(i);
    gpuErrchk(cudaStreamSynchronize(streams[i]));
  }
    printMemoryUsage();
    cudaError_t error = cudaGetLastError();
    printf("CUDA error: %s\n", cudaGetErrorString(error));
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
bool compareById(const LC::B18TrafficPerson& a, const LC::B18TrafficPerson& b) {
    return a.id < b.id;
}

void sortTrafficPersonsById(std::vector<LC::B18TrafficPerson>& trafficPersonVec) {
    
}
void b18GetDataCUDA(std::vector<LC::B18TrafficPerson>& trafficPersonVec, std::vector<LC::B18EdgeData> &edgesData){
  // copy back people
  
  int indexCursor=0;
  for(int i = 0; i < ngpus; i++){
      for (int j = 0; j < size_gpu_part[i]/sizeof(LC::B18TrafficPerson); j++) {
      trafficPersonVec_d[indexCursor++] = trafficPersonVec_d_gpus[i][j];
    }
  }
  trafficPersonVec.clear();
  trafficPersonVec.resize(indexCursor);
  // cudaMemcpy(trafficPersonVec.data(),trafficPersonVec_d,indexCursor*sizeof(LC::B18TrafficPerson),cudaMemcpyDeviceToHost);//cudaMemcpyHostToDevice
  memcpy( trafficPersonVec.data(),trafficPersonVec_d, indexCursor*sizeof(LC::B18TrafficPerson));
  std::sort(trafficPersonVec.begin(), trafficPersonVec.end(),
        [](const LC::B18TrafficPerson& a, const LC::B18TrafficPerson& b) {
            return a.id < b.id;
        }
    );
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
  LC::B18TrafficPerson *trafficPersonVec,
  LC::B18TrafficPersonModify *trafficPersonModify,
  uint *indexPathVec,
  int indexPathVec_d_size,
  LC::B18EdgeData* edgesData,
  int edgesData_d_size,
  uchar *laneMap,
  int laneMap_d_size,
  int *laneMapper,
  LC::B18IntersectionData *intersections,
  uchar *trafficLights,
  uint trafficLights_d_size,
  float deltaTime,
  const parameters simParameters,
  int* vertexIdToPar_d,
  int* ifCommu
  )
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
    // firstEdge convert to LaneIndex
    uint firstEdge = indexPathVec[indexFirstEdge];
    int firstEdge_d = -1;
    if(firstEdge != END_OF_PATH){
      firstEdge_d=laneMapper[firstEdge];
      // getLaneIdToLaneIdInGpuValue(laneIdToLaneIdInGpu_d_keys, laneIdToLaneIdInGpu_d_values, wholeLaneMap_size,firstEdge,firstEdge_d); // turn overall edgeId(laneId) to edge(lane) index in edgesData[i]
      assert(firstEdge_d!=-1);
    }
    
    trafficPersonVec[p].last_time_simulated = currentTime;
    
    if (firstEdge == END_OF_PATH) {
      trafficPersonVec[p].active = 2;
      return;
    }

    if (firstEdge_d >= edgesData_d_size) {
      printf("firstEdge %d is bigger than edgesData size %d\n", firstEdge, edgesData_d_size);
    }
    assert(firstEdge_d < edgesData_d_size);

    if(vertexIdToPar_d[edgesData[firstEdge_d].nextInters]!=gpuIndex){
      trafficPersonModify[p].ifToRemove=true;
      atomicAdd(ifCommu, 1);
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
      trafficPersonVec[p].numOfLaneInEdge = lN;
      trafficPersonVec[p].posInLaneM = b; //m
      uchar vInMpS = (uchar) (trafficPersonVec[p].v * 3); //speed in m/s *3 (to keep more precision
      int laneMapPosition2 = mapToWriteShift + kMaxMapWidthM * (firstEdge_d + lN) + b;
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

  bool ifPassIntersection=false;
  // set up next edge info
  int indexCurrentEdge = trafficPersonVec[p].indexPathCurr;
  assert(indexCurrentEdge < indexPathVec_d_size);
  uint currentEdge = indexPathVec[indexCurrentEdge];
  int currentEdge_d=-1;
  // return;
  currentEdge_d=laneMapper[currentEdge];
  // getLaneIdToLaneIdInGpuValue(laneIdToLaneIdInGpu_d_keys, laneIdToLaneIdInGpu_d_values, wholeLaneMap_size,currentEdge,currentEdge_d); //get edge index in edgesData_d
  
  assert(currentEdge_d!=-1);
  assert(currentEdge_d < edgesData_d_size);
  
  int indexNextEdge = trafficPersonVec[p].indexPathCurr + 1;
  assert(indexNextEdge < indexPathVec_d_size);
  uint nextEdge = indexPathVec[indexNextEdge];
  int nextEdge_d=-1;
  
  
  // if(nextEdge == END_OF_PATH )nextEdge_d=END_OF_PATH;
  if(nextEdge != END_OF_PATH ){
    nextEdge_d=laneMapper[nextEdge];
    // getLaneIdToLaneIdInGpuValue(laneIdToLaneIdInGpu_d_keys, laneIdToLaneIdInGpu_d_values, wholeLaneMap_size,nextEdge,nextEdge_d); 
    if(nextEdge_d==-1){
      trafficPersonModify[p].ifToRemove=true;
      atomicAdd(ifCommu, 1);
      return;
    }
    
  }
  else {
    nextEdge_d=END_OF_PATH;
  }
  assert(nextEdge_d < edgesData_d_size || nextEdge == END_OF_PATH);

  int prevEdge = trafficPersonVec[p].prevEdge;
  int prevEdge_d=-1;
  prevEdge_d=laneMapper[prevEdge];
  // getLaneIdToLaneIdInGpuValue(laneIdToLaneIdInGpu_d_keys, laneIdToLaneIdInGpu_d_values, wholeLaneMap_size,prevEdge,prevEdge_d); 
  if(prevEdge_d!=-1){
    // printf("@@@@%d %u",prevEdge,trafficPersonVec[p].init_intersection);
    assert(prevEdge_d < edgesData_d_size);
  }
  

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
    if(prevEdge_d!=-1){
      assert(prevEdge_d< edgesData_d_size);
      if (elapsed_s > MINIMUM_NUMBER_OF_CARS_TO_MEASURE_SPEED) {
        trafficPersonVec[p].manual_v = edgesData[prevEdge_d].length / elapsed_s;
        edgesData[prevEdge_d].curr_iter_num_cars += 1;
        edgesData[prevEdge_d].curr_cum_vel += trafficPersonVec[p].manual_v;
      }
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
  ushort numOfCells = ceil((edgesData[currentEdge_d].length - intersectionClearance)); //intersectionClearance hardcoded to 7.8f - why?

  for (ushort b = byteInLine + 2; (b < numOfCells) && (!found) && (numCellsCheck > 0); b++, numCellsCheck--) {
    // ShiftRead + WIDTH * (width number * # lanes + # laneInEdge) + b  TODO(pavan): WHAT IS THIS?
    //TODO(pavan): double check what mapToReadShift is printing out
    assert(trafficPersonVec[p].indexPathCurr < indexPathVec_d_size);
    const uint posToSample = mapToReadShift +
      kMaxMapWidthM *(currentEdge_d +
      (((int) (byteInLine / kMaxMapWidthM)) * edgesData[currentEdge_d].numLines) +
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
      (edgesData[currentEdge_d].nextIntersMapped !=
        trafficPersonVec[p].end_intersection)) { // we haven't arrived to destination (check next line)
      ushort nextEdgeLaneToBe = trafficPersonVec[p].numOfLaneInEdge; //same lane

      //printf("trafficPersonVec[p].numOfLaneInEdge %u\n",trafficPersonVec[p].numOfLaneInEdge);
      assert(nextEdge_d < edgesData_d_size);
      if (nextEdgeLaneToBe >= edgesData[nextEdge_d].numLines) {
        nextEdgeLaneToBe = edgesData[nextEdge_d].numLines - 1; //change line if there are less roads
      }

      //printf("2trafficPersonVec[p].numOfLaneInEdge %u\n",trafficPersonVec[p].numOfLaneInEdge);
      ushort numOfCells = ceil(edgesData[nextEdge_d].length);

      for (ushort b = 0; (b < numOfCells) && (!found) && (numCellsCheck > 0); b++, numCellsCheck--) {
        const uint posToSample = mapToReadShift + kMaxMapWidthM * (nextEdge_d + nextEdgeLaneToBe) + b; // b18 not changed since we check first width
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
    trafficPersonVec[p].v / edgesData[currentEdge_d].maxSpeedMperSec), 4) - thirdTerm);

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
      kMaxMapWidthM * (currentEdge_d +
      (((int) (posInLineCells / kMaxMapWidthM)) * edgesData[currentEdge_d].numLines) +
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
  if (trafficPersonVec[p].posInLaneM > edgesData[currentEdge_d].length) { //reach intersection
    ifPassIntersection=true;
    numMToMove = trafficPersonVec[p].posInLaneM - edgesData[currentEdge_d].length;
    trafficPersonVec[p].posInLaneM = numMToMove;
    trafficPersonVec[p].dist_traveled += edgesData[currentEdge_d].length;
    trafficPersonVec[p].path_length_gpu++;

    //2.2.1 find next edge
    assert(indexCurrentEdge < indexPathVec_d_size);
    assert(currentEdge_d < edgesData_d_size);

    trafficPersonVec[p].LC_stateofLaneChanging = 0;

    //2.1 check if end
    if (nextEdge != END_OF_PATH) {
      assert(nextEdge_d < edgesData_d_size);
      if (trafficPersonVec[p].numOfLaneInEdge >= edgesData[nextEdge_d].numLines) {
        trafficPersonVec[p].numOfLaneInEdge = edgesData[nextEdge_d].numLines - 1; //change line if there are less roads
      }

      //TODO: Test if the following line is doing the conversion wrong
      uchar vInMpS = (uchar) (trafficPersonVec[p].v * 3); //speed in m/s to fit in uchar
      ushort posInLineCells = (ushort) (trafficPersonVec[p].posInLaneM);
      const uint posToSample = mapToWriteShift + kMaxMapWidthM *
                              (nextEdge_d + (((int) (posInLineCells / kMaxMapWidthM)) *
                              edgesData[nextEdge_d].numLines) + trafficPersonVec[p].numOfLaneInEdge) +
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
    assert(currentEdge_d < edgesData_d_size);
    assert(nextEdge_d < edgesData_d_size || nextEdge == END_OF_PATH);

    // LANE CHANGING (happens when we are not reached the intersection)
    if (trafficPersonVec[p].v > 3.0f && trafficPersonVec[p].num_steps % 5 == 0) {
      //at least 10km/h to try to change lane
      //just check every (5 steps) 5 seconds

      // next thing is not a traffic light
      // skip if there is one lane (avoid to do this)
      // skip if it is the last edge
      if (!nextVehicleIsATrafficLight &&
        edgesData[currentEdge_d].numLines > 1 && nextEdge != END_OF_PATH) {
        ////////////////////////////////////////////////////
        // LC 1 update lane changing status
        if (trafficPersonVec[p].LC_stateofLaneChanging == 0) {
          // 2.2-exp((x-1)^2)
          float x = trafficPersonVec[p].posInLaneM / edgesData[currentEdge_d].length;

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
          if ((trafficPersonVec[p].v < (edgesData[currentEdge_d].maxSpeedMperSec * 0.7f)) &&
            (dv_dt < 0) && trafficPersonVec[p].v > 3.0f) {

            bool leftLane = trafficPersonVec[p].numOfLaneInEdge >
              0; //at least one lane on the left
            bool rightLane = trafficPersonVec[p].numOfLaneInEdge <
              edgesData[currentEdge_d].numLines - 1; //at least one lane

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

            assert(currentEdge_d + trafficPersonVec[p].numOfLaneInEdge < trafficLights_d_size);
            uchar trafficLightState = trafficLights[currentEdge_d + trafficPersonVec[p].numOfLaneInEdge];
            calculateGapsLC(mapToReadShift, laneMap, trafficLightState,
              currentEdge_d + laneToCheck, edgesData[currentEdge_d].numLines,
              trafficPersonVec[p].posInLaneM,
              edgesData[currentEdge_d].length, v_a, v_b, gap_a, gap_b, laneMap_d_size);

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
            calculateLaneCarShouldBe(currentEdge_d, nextEdge_d, intersections,
              edgesData[currentEdge_d].nextIntersMapped,
              edgesData[currentEdge_d].numLines,
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
            if ((trafficPersonVec[p].v < (edgesData[currentEdge_d].maxSpeedMperSec * 0.7f)) &&
              (dv_dt < 0) && trafficPersonVec[p].v > 3.0f) {
              leftLane =
                (trafficPersonVec[p].numOfLaneInEdge > 0) && //at least one lane on the left
                (trafficPersonVec[p].numOfLaneInEdge - 1 >= trafficPersonVec[p].LC_initOKLanes)
                &&
                (trafficPersonVec[p].numOfLaneInEdge - 1 < trafficPersonVec[p].LC_endOKLanes);
              rightLane =
                (trafficPersonVec[p].numOfLaneInEdge <
                  edgesData[currentEdge_d].numLines - 1) &&
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
              trafficPersonVec[p].numOfLaneInEdge + 1 >= edgesData[currentEdge_d].numLines) {
              printf("ERROR: RT laneToCheck>=edgeNumLanes\n");
            }

            if (leftLane && trafficPersonVec[p].numOfLaneInEdge == 0) {
              printf("ERROR %u: LT laneToCheck>=edgeNumLanes OK %u-%u NE %u\n",
                p, trafficPersonVec[p].LC_initOKLanes, trafficPersonVec[p].LC_endOKLanes,
                currentEdge_d);
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

            if (laneToCheck >= edgesData[currentEdge_d].numLines) {
              printf("ERROR: laneToCheck>=edgesData[currentEdge].numLines %u %u\n",
                laneToCheck, edgesData[currentEdge_d].numLines);
            }

            uchar v_a, v_b;
            float gap_a, gap_b;
            assert(currentEdge_d + trafficPersonVec[p].numOfLaneInEdge < trafficLights_d_size);
            uchar trafficLightState = trafficLights[currentEdge_d + trafficPersonVec[p].numOfLaneInEdge];
            calculateGapsLC(mapToReadShift, laneMap, trafficLightState,
              currentEdge_d + laneToCheck, edgesData[currentEdge_d].numLines,
              trafficPersonVec[p].posInLaneM,
              edgesData[currentEdge_d].length, v_a, v_b, gap_a, gap_b, laneMap_d_size);

            if (gap_a == 1000.0f && gap_b == 1000.0f) { //lag and lead car very far
              trafficPersonVec[p].numOfLaneInEdge = laneToCheck; // CHANGE LINE
            } else { // NOT ALONE
              float b1A = 0.05f, b2A = 0.15f;
              float b1B = 0.15f, b2B = 0.40f;
              float gamma = 0.000025;
              // simParameters.s_0-> critical lead gap
              float distEnd = edgesData[currentEdge_d].length - trafficPersonVec[p].posInLaneM;
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
      kMaxMapWidthM * (currentEdge_d + (((int) (posInLineCells / kMaxMapWidthM)) *
      edgesData[currentEdge_d].numLines) +
      trafficPersonVec[p].numOfLaneInEdge) +
      posInLineCells % kMaxMapWidthM;
    assert(posToSample < laneMap_d_size);
    laneMap[posToSample] = vInMpS;
  }

  if(vertexIdToPar_d[edgesData[currentEdge_d].nextInters]!=gpuIndex){
    trafficPersonModify[p].ifToRemove=true;
    atomicAdd(ifCommu, 1);
    return;
  }
  
  if(ifPassIntersection && nextEdge!=END_OF_PATH){
    //when entering ghost zone, prepare to copy
    
      int targetGpuIndex=vertexIdToPar_d[edgesData[nextEdge_d].nextInters];
      if(targetGpuIndex!=gpuIndex){
      trafficPersonModify[p].ifToCopy=true;
      trafficPersonModify[p].gpuIndexToCopy=targetGpuIndex;
      atomicAdd(ifCommu, 1);
    }
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
  
  for(int i = 0; i < ngpus; i++){
    cudaSetDevice(i);
    gpuErrchk(cudaMemset(&laneMap_d[i][0], -1, halfLaneMap_n[i]*sizeof(unsigned char)));
    gpuErrchk(cudaMemset(&laneMap_d[i][halfLaneMap_n[i]], -1, halfLaneMap_n[i]*sizeof(unsigned char)));
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

    kernel_intersectionOneSimulation << < ceil(numIntersections_n[i] / 512.0f), 512 >> > (numIntersections_n[i], currentTime, intersections_d[i], trafficLights_d[i]);
    
    // cudaError_t err = cudaPeekAtLastError();
    // if (err != cudaSuccess) {
    //   printf("CUDA Error: %s\n", cudaGetErrorString(err));
    // }
    gpuErrchk(cudaPeekAtLastError());

  }
  intersectionBench.stopMeasuring();
  
  peopleBench.startMeasuring();
  // Simulate people.
  // #pragma omp parallel for
        // for(int i = 0; i < 2; i++) {
  
  //printf("Number of people per GPU : %i ", numPeople_gpu);
  LC::B18TrafficPerson **new_trafficPersonVec_d_gpus = new LC::B18TrafficPerson*[ngpus];
  LC::B18TrafficPersonModify ** new_trafficPersonModify= new LC::B18TrafficPersonModify*[ngpus];
  size_t new_size_gpu_part[ngpus]; 
  for (int i = 0; i < ngpus; ++i) {
    new_size_gpu_part[i] = size_gpu_part[i];
  }
  std::map<int,std::vector<std::pair<int,int>> >personToCopy;
 
  for(int i = 0; i < ngpus; i++){
    cudaSetDevice(i);
    int numPeople_gpu = size_gpu_part[i]/sizeof(LC::B18TrafficPerson);

    
    // auto start = std::chrono::high_resolution_clock::now();
    //  cudaEvent_t start, stop;
    // cudaEventCreate(&start);
    // cudaEventCreate(&stop);
    // cudaEventRecord(start, 0);
    kernel_trafficSimulation <<< numBlocks, threadsPerBlock>> >
    (i,numPeople_gpu, currentTime, mapToReadShift_n[i],
    mapToWriteShift_n[i], trafficPersonVec_d_gpus[i],trafficPersonModify[i], indexPathVec_d[i], indexPathVec_d_size,
    edgesData_d[i], edgesData_d_size[i], laneMap_d[i], laneMap_d_size[i], laneIdMapper_d[i],
    intersections_d[i], trafficLights_d[i], trafficLights_d_size[i], deltaTime, simParameters,vertexIdToPar_d[i],ifCommu_d[i]);
    
    // cudaEventRecord(stop, 0);
    // cudaError_t err = cudaGetLastError();
    // if (err != cudaSuccess) {
    //   printf("CUDA error: %s\n", cudaGetErrorString(err));
    // }

    gpuErrchk(cudaPeekAtLastError());
    gpuErrchk(cudaDeviceSynchronize());
    // float elapsedTime;
    // cudaEventElapsedTime(&elapsedTime, start, stop);
    //  std::cout << "Kernel execution time: " << elapsedTime << " milliseconds" << std::endl;

    // cudaEventDestroy(start);
    // cudaEventDestroy(stop);
    // auto end = std::chrono::high_resolution_clock::now();
    
    // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    // printf("Time taken by b18SimulateTrafficCUDA: %lld microseconds\n", duration.count());
    // std::ofstream outFile("times.txt", std::ios::app);
    // outFile << duration.count() << "\n";
    // outFile.close();
    // update trafficPerson_n_gpu
    // new_trafficPersonVec_d_gpus
    
    // gpuErrchk(cudaMemPrefetchAsync(trafficPersonModify[i], size_gpu_part[i]/sizeof(LC::B18TrafficPerson)*sizeof(LC::B18TrafficPersonModify), cudaCpuDeviceId));
    gpuErrchk(cudaMemcpy(&ifCommu[i], ifCommu_d[i], sizeof(int), cudaMemcpyDeviceToHost));

    }
    int commu_times=0;
    bool ifCommunication=false;
    for(int i = 0; i < ngpus; i++){
      commu_times+=ifCommu[i];
      if(ifCommu[i]!=0){
        ifCommunication=true;
        // break;
      }
    }

    if(ifCommunication){
      std::ofstream outFile("commu_times.txt", std::ios::app);
      outFile << commu_times << "\n";
      outFile.close();
    
    for(int i = 0; i < ngpus; i++){
       for (int j=0;j<size_gpu_part[i]/sizeof(LC::B18TrafficPerson);j++){
      LC::B18TrafficPersonModify modifyPerson=trafficPersonModify[i][j];
      if(modifyPerson.ifToCopy){
        new_size_gpu_part[modifyPerson.gpuIndexToCopy] += sizeof(LC::B18TrafficPerson);
        personToCopy[modifyPerson.gpuIndexToCopy].push_back(std::make_pair(i,j));
      }
      if(modifyPerson.ifToRemove){
        new_size_gpu_part[i] -= sizeof(LC::B18TrafficPerson);
      }
    }
  }
    // TODO: use kernel function to copy/delete data
    // cudafree trafficPerson_n_gpu
    // trafficPerson_n_gpu=newtrafficPerson_n_gpu
  }
  // bool ifCommu=false;
  // for (int i = 0; i < ngpus; ++i) {
  //   if(new_size_gpu_part[i] != size_gpu_part[i]){
  //     ifCommu=true;
  //     break;
  //   }
  // }
  if(ifCommunication){
    // auto start = std::chrono::high_resolution_clock::now();
  
  for(int i = 0; i < ngpus; i++){
      gpuErrchk(cudaSetDevice(i));
      // gpuErrchk(cudaMemPrefetchAsync(trafficPersonVec_d_gpus[i], size_gpu_part[i], cudaCpuDeviceId));
      gpuErrchk(cudaMallocManaged(&new_trafficPersonVec_d_gpus[i], new_size_gpu_part[i]));
      gpuErrchk(cudaMallocManaged(&new_trafficPersonModify[i], new_size_gpu_part[i]/sizeof(LC::B18TrafficPerson)*sizeof(LC::B18TrafficPersonModify)));
      //initialize new_trafficPersonModify
      for(int j=0;j<new_size_gpu_part[i]/sizeof(LC::B18TrafficPerson);j++){
        new_trafficPersonModify[i][j].ifToCopy = false;
        new_trafficPersonModify[i][j].ifToRemove = false;
        new_trafficPersonModify[i][j].gpuIndexToCopy = -1;
      }
       
      int new_person_index=0;
      for(int j=0;j<size_gpu_part[i]/sizeof(LC::B18TrafficPerson);j++){
        if(!trafficPersonModify[i][j].ifToRemove){
          // if(trafficPersonVec_d_gpus[i][j].init_intersection==211662&&trafficPersonVec_d_gpus[i][j].init_intersection==88718){
          //   // printf("\n")
          // }
          new_trafficPersonVec_d_gpus[i][new_person_index++]=trafficPersonVec_d_gpus[i][j];
        }
      }
      for (const auto& entry : personToCopy[i]){
        new_trafficPersonVec_d_gpus[i][new_person_index++]=trafficPersonVec_d_gpus[entry.first][entry.second];
        // printf("copy %d %d (id: %d) to %d\n",entry.first,entry.second,trafficPersonVec_d_gpus[entry.first][entry.second].id,i);
      }
  }
  cudaStream_t streams[ngpus];
  for(int i = 0; i < ngpus; i++){
    cudaSetDevice(i); 
    cudaStreamCreate( &streams[i] );
    gpuErrchk(cudaFree(trafficPersonVec_d_gpus[i]));
    gpuErrchk(cudaFree(trafficPersonModify[i]));
    trafficPersonVec_d_gpus[i]=new_trafficPersonVec_d_gpus[i];
    trafficPersonModify[i]=new_trafficPersonModify[i];
    size_gpu_part[i]=new_size_gpu_part[i];
    if(size_gpu_part[i]>0){
      gpuErrchk(cudaMemPrefetchAsync(trafficPersonVec_d_gpus[i], size_gpu_part[i], i, streams[i]));
      gpuErrchk(cudaMemPrefetchAsync(trafficPersonModify[i], size_gpu_part[i]/sizeof(LC::B18TrafficPerson)*sizeof(LC::B18TrafficPersonModify), i, streams[i]));
    }
    
  }
  cudaDeviceSynchronize();
    //   auto end = std::chrono::high_resolution_clock::now();
    
    // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    // std::ofstream outFile("commu_times.txt", std::ios::app);
    // outFile << duration.count() << "\n";
    // outFile.close();
  }
  for(int i = 0; i < ngpus; i++){
    cudaSetDevice(i); 
    gpuErrchk(cudaMemset(ifCommu_d[i], 0, sizeof(int)));
  }

  // else{
  //   for(int i = 0; i < ngpus; i++){
  //   cudaSetDevice(i); 
  //   cudaStreamCreate( &streams[i] );
  //   gpuErrchk(cudaMemPrefetchAsync(trafficPersonModify[i], size_gpu_part[i]/sizeof(LC::B18TrafficPerson)*sizeof(LC::B18TrafficPersonModify), i, streams[i]));
  //   }
  // }
     
  peopleBench.stopMeasuring();

        // }

}


