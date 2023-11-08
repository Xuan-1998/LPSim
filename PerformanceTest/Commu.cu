#include <cuda_runtime.h>
#include <iostream>
#include<vector>
#include <fstream>
int main() {
    
    int ngpus;
    float *d_data_src, *d_data_dst;
    int gpuid1 = 0; 
    int gpuid2 = 1; 
    cudaGetDeviceCount(&ngpus);
    printf("Number of gpus: %d\n",ngpus);
    int canAccessPeer = 0;
    cudaDeviceCanAccessPeer(&canAccessPeer, gpuid1, gpuid2);
    if (canAccessPeer) {
        cudaDeviceEnablePeerAccess(gpuid2, 0);
        printf("Enable PeerAccess\n");
    }
    else{
        printf("Cannot Enable PeerAccess\n");
    }
    // size_t size = (1 << 20)*10;
   
    float milliseconds = 0;
    cudaEvent_t start, stop;
    cudaEventCreate(&start);
    cudaEventCreate(&stop);
    
    
    std::vector<size_t> sizes = {
    1UL << 10, 10UL << 10, 100UL << 10,   // 1KB, 10KB, 100KB
    1UL << 20, 10UL << 20, 100UL << 20,   // 1MB, 10MB, 100MB
    1UL << 30, 2UL << 30   // 1GB, 2GB, 4GB
    };
    for (auto& size : sizes) {
        size /= sizeof(float);
    }
     std::ofstream csvFile("timing_commu.csv");
     csvFile << "Size,Transfer Type,Time (ms)\n";

    if (!csvFile.is_open()) {
        std::cerr << "Unable to open file for writing.\n";
        return -1;
    }
    for (auto size : sizes){
    float *h_data = new float[size];
    for(size_t i = 0; i < size; i++) {
        h_data[i] = 1.0f;
    }
    
   //prepare data
    cudaSetDevice(gpuid1);
    cudaMalloc((void **)&d_data_src, size * sizeof(float));
    cudaEventRecord(start);
    cudaMemcpy(d_data_src, h_data, size * sizeof(float), cudaMemcpyHostToDevice);
    cudaEventRecord(stop);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&milliseconds, start, stop);
    std::cout << "Time cudaMemcpyHostToDevice: " << milliseconds << " ms\n";
    

    cudaSetDevice(gpuid2);
    cudaMalloc((void **)&d_data_dst, size * sizeof(float));
    
    
    //device-host-device
    float* h_data_tmp = new float[size];
    cudaSetDevice(gpuid1);
    cudaEventRecord(start);
    cudaMemcpy(h_data_tmp, d_data_src, size* sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(d_data_src, h_data_tmp, size * sizeof(float), cudaMemcpyHostToDevice);
    cudaEventRecord(stop);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&milliseconds, start, stop);
    std::cout << "Time D-H-D: " << milliseconds << " ms\n";
    csvFile << size * sizeof(float) << ",D-H-D," << milliseconds << "\n";

    //peer to peer
    cudaSetDevice(gpuid1);
    cudaEventRecord(start);
    cudaMemcpyPeer(d_data_dst, gpuid2, d_data_src, gpuid1, size * sizeof(float));
    cudaEventRecord(stop);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&milliseconds, start, stop);
    std::cout << "Time P2P: " << milliseconds << " ms\n";
    csvFile << size * sizeof(float) << ",P2P," << milliseconds << "\n";

    cudaFree(d_data_src);
    cudaFree(d_data_dst);
    delete[] h_data;

    //----cudaMallocManaged-------
    
    float *d_data;
    cudaMallocManaged(&d_data, size * sizeof(float));
    // memcpy(d_data, h_data, size * sizeof(float));
    cudaMemset(d_data, 2.5, size * sizeof(float));

    // to gpu1
    cudaSetDevice(gpuid1);
    cudaMemPrefetchAsync(d_data, size * sizeof(float), gpuid1);
    cudaDeviceSynchronize();

    //to gpu2
    cudaSetDevice(gpuid2);
    cudaEventRecord(start);
    cudaMemPrefetchAsync(d_data, size * sizeof(float), gpuid2);
    cudaDeviceSynchronize();
    cudaEventRecord(stop);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&milliseconds, start, stop);
    std::cout << "Time Unified memory: " << milliseconds << " ms" << std::endl;
    cudaFree(d_data);
}
    cudaEventDestroy(start);
    cudaEventDestroy(stop);
    return 0;
}
