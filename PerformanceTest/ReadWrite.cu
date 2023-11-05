#include <cuda_runtime.h>
#include <iostream>

__global__ void readAndWriteKernel(float *data, float *buffer, size_t size) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < size) {
        //simulate read and write
        buffer[idx]=data[idx];
        data[idx] = 2*buffer[idx];
    }
}
__global__ void readKernel(float *data, float *output, size_t size) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < size) {
        output[idx] = data[idx];
    }
}

__global__ void writeKernel(float *data, float value, size_t size) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < size) {
        data[idx] = value;
    }
}
int main() {
    size_t size = 1 << 20; // 1M byte
    float *d_data, *d_output;
    float value = 2.0f;

    cudaMalloc((void **)&d_data, size * sizeof(float));
    cudaMalloc((void **)&d_output, size * sizeof(float));

    float *h_data = new float[size];
    for(size_t i = 0; i < size; i++) {
        h_data[i] = 1.0f; 
    }
    cudaMemcpy(d_data, h_data, size * sizeof(float), cudaMemcpyHostToDevice);

    dim3 blockSize(256);
    dim3 gridSize((size + blockSize.x - 1) / blockSize.x);
    cudaEvent_t start, stop;
    cudaEventCreate(&start);
    cudaEventCreate(&stop);
    float milliseconds = 0;

    // read
    cudaEventRecord(start);
    readKernel<<<gridSize, blockSize>>>(d_data, d_output, size);
    cudaEventRecord(stop);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&milliseconds, start, stop);
    std::cout << "Time read: " << milliseconds << " ms\n";

    

    // write
    cudaEventRecord(start);
    writeKernel<<<gridSize, blockSize>>>(d_data, value, size);
    cudaEventRecord(stop);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&milliseconds, start, stop);
    std::cout << "Time write: " << milliseconds << " ms\n";


    // read and write
    cudaEventRecord(start);
    readAndWriteKernel<<<gridSize, blockSize>>>(d_data,d_output, size);
    cudaEventRecord(stop);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&milliseconds, start, stop);
    std::cout << "Time read and write: " << milliseconds << " ms\n";


    cudaFree(d_data);
    cudaFree(d_output);
    delete[] h_data;

    return 0;
}
