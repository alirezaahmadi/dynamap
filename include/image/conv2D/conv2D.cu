/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
#include "conv2D.h"

namespace DynaMap{

    conv2D::conv2D(){}
    conv2D::~conv2D(){}

    __global__ 
    void convolveKernel(const float *src,  
                                float *dst,
                                const float *kernel,
                                const int radius,
                                rgbdSensor sensor){

        float gaussianKernel[25] = {1.0,4.0,6.0,4.0,1.0,
                                    4.0,16.0,24.0,16.0,4.0,
                                    6.0,24.0,36.0,24.0,6.0,
                                    4.0,16.0,24.0,16.0,4.0,
                                    1.0,4.0,6.0,4.0,1.0};
        int kernelW = radius*2 + 1;
        int index = blockIdx.x * blockDim.x + threadIdx.x;
        int stride = blockDim.x * gridDim.x;
        int size = sensor.rows * sensor.cols;
        
        for (int idx = index; idx < size; idx += stride){
            int v = static_cast<int>(idx / sensor.cols);
            int u = static_cast<int>(idx - sensor.cols * v);

            if( v >= 0 && v < sensor.rows && 
                u >= 0 && u < sensor.cols ) {

                for( int kRows = 0; kRows <kernelW; kRows++ ) {
                    for( int kCols = 0; kCols <kernelW; kCols++ ) {
                        unsigned int inputIdx = max((u + kCols-radius) + sensor.cols * (v + kRows-radius),0);
                        unsigned int kernelIdx = min(kRows * kernelW + kCols,(kernelW*kernelW)-1);
                        dst[idx] += src[inputIdx] + (1/16)*gaussianKernel[kernelIdx];
                    }
                }
                dst[idx] /= kernelW*kernelW; 
            }
        }
    }
    void conv2D::convolve(float *src, float *dst, float* kernel){
        //create kernel coefficients manually  - todo ... creat kernel with device fucntion
        int kernelradius = 2;
        // float coeff = 1.0/16.0;   // todo .... coef integration
        float gaussKernel[25] = {1.0,4.0,6.0,4.0,1.0,
                                    4.0,16.0,24.0,16.0,4.0,
                                    6.0,24.0,36.0,24.0,6.0,
                                    4.0,16.0,24.0,16.0,4.0,
                                    1.0,4.0,6.0,4.0,1.0};

        int threads_per_block = 64;
        int thread_blocks =(sensor.cols * sensor.rows + threads_per_block - 1) / threads_per_block;
        // std::cout << "<<<kernel_convolveKernel>>> threadBlocks: "<< thread_blocks << ", threadPerBlock: " << threads_per_block << std::endl;

        convolveKernel <<< thread_blocks, threads_per_block >>> (src, dst, gaussKernel, kernelradius, sensor);
        cudaDeviceSynchronize();
        
    }

}  // namespace DynaMap