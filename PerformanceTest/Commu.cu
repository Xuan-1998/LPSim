#include <cuda_runtime.h>
#include <iostream>

int main() {
    
    int ngpus;
    cudaGetDeviceCount(&ngpus);
    printf("Number of gpus: %d\n",ngpus);
    size_t size = 1 << 20;
    float *d_data_src, *d_data_dst;
    int gpuid1 = 0; 
    int gpuid2 = 1; 
    float milliseconds = 0;
    cudaEvent_t start, stop;
    cudaEventCreate(&start);
    cudaEventCreate(&stop);
    
    float *h_data = new float[size];
    for(size_t i = 0; i < size; i++) {
        h_data[i] = 1.0f;
    }
    
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
    int canAccessPeer = 0;
    cudaDeviceCanAccessPeer(&canAccessPeer, gpuid1, gpuid2);
    if (canAccessPeer) {
        cudaDeviceEnablePeerAccess(gpuid2, 0);
    }
    else{
        printf("Cannot Enable PeerAccess\n");
    }


    //peer to peer
    cudaSetDevice(gpuid1);
    cudaEventRecord(start);

    cudaMemcpyPeer(d_data_dst, gpuid2, d_data_src, gpuid1, size * sizeof(float));

    cudaEventRecord(stop);
    cudaEventSynchronize(stop);

    cudaEventElapsedTime(&milliseconds, start, stop);
    std::cout << "Time communication: " << milliseconds << " ms\n";

    cudaFree(d_data_src);
    cudaFree(d_data_dst);
    delete[] h_data;
    cudaEventDestroy(start);
    cudaEventDestroy(stop);

    //----cudaMallocManaged-------
    {
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
    cudaEventCreate(&start);
    cudaEventCreate(&stop);
    cudaEventRecord(start, 0);

    cudaMemPrefetchAsync(d_data, size * sizeof(float), gpuid2);
    cudaDeviceSynchronize();

    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);

    float milliseconds = 0;
    cudaEventElapsedTime(&milliseconds, start, stop);
    std::cout << "Time Unified memory: " << milliseconds << " ms" << std::endl;

    cudaFree(d_data);
    cudaEventDestroy(start);
    cudaEventDestroy(stop);
    }
    

    return 0;
}
