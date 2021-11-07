/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
#include "pyramid.h"

namespace DynaMap{

    pyramid::~pyramid(){}

    // __global__
    // void upSampleKernel(float *src, float *dst){
    // }
    __global__
    void downSampleRGBKernel(uchar3 *src, uchar3 *dst, rgbdSensor sensor,int scale){

        int index = blockIdx.x * blockDim.x + threadIdx.x;
        int stride = blockDim.x * gridDim.x;
        int size = sensor.cols * sensor.rows;
        
        for (int idx = index; idx < size; idx += stride){
            int v = static_cast<int>(idx / sensor.cols);
            int u = static_cast<int>(idx - sensor.cols * v);

            int u_offset = 0;
            int v_offset = 0;
            if(scale == 2){
                u_offset = 160;
                v_offset = 120;
            }else if(scale == 4){
                u_offset = 240;
                v_offset = 180;
            }else if(scale == 8){
                u_offset = 280;
                v_offset = 210;
            }else{
                u_offset = 0;
                v_offset = 0;
            }

            uint py_u = __float2uint_rd(u / scale) + u_offset;
            uint py_v = __float2uint_rd(v / scale) + v_offset;
            dst[py_v * sensor.cols + py_u] = src[v * sensor.cols + u];
        }
    }
    __global__
    void downSampleDepthKernel(float *src, float *dst, rgbdSensor sensor,int scale){

        int index = blockIdx.x * blockDim.x + threadIdx.x;
        int stride = blockDim.x * gridDim.x;
        int size = sensor.cols * sensor.rows;
        
        for (int idx = index; idx < size; idx += stride){
            int v = static_cast<int>(idx / sensor.cols);
            int u = static_cast<int>(idx - sensor.cols * v);

            int u_offset = 0;
            int v_offset = 0;
            if(scale == 2){
                u_offset = 160;
                v_offset = 120;
            }else if(scale == 4){
                u_offset = 240;
                v_offset = 180;
            }else if(scale == 8){
                u_offset = 280;
                v_offset = 210;
            }else{
                u_offset = 0;
                v_offset = 0;
            }

            if( v >= 0 && v < sensor.rows && 
                u >= 0 && u < sensor.cols ) {
                    uint py_u = __float2uint_rd(u / scale) + u_offset;
                    uint py_v = __float2uint_rd(v / scale) + v_offset;
                    dst[py_v * sensor.cols + py_u] = src[v * sensor.cols + u];
            }
        }
    }
    __global__
    void downSampleKernel(pyramid &PyImage, rgbdImage &src, rgbdSensor sensor,int scale){

        int index = blockIdx.x * blockDim.x + threadIdx.x;
        int stride = blockDim.x * gridDim.x;
        int size = sensor.cols * sensor.rows;
        
        for (int idx = index; idx < size; idx += stride){
            int v = static_cast<int>(idx / sensor.cols);
            int u = static_cast<int>(idx - sensor.cols * v);

            int u_offset = 0;
            int v_offset = 0;
            if(scale == 2){
                u_offset = 160;
                v_offset = 120;
            }else if(scale == 4){
                u_offset = 240;
                v_offset = 180;
            }else if(scale == 8){
                u_offset = 280;
                v_offset = 210;
            }else{
                u_offset = 0;
                v_offset = 0;
            }

            if( v >= 0 && v < sensor.rows && 
                u >= 0 && u < sensor.cols ) {
                    uint py_u = __float2uint_rd(u / scale) + u_offset;
                    uint py_v = __float2uint_rd(v / scale) + v_offset;
                    PyImage.depth[py_v * sensor.cols + py_u] = src.depth[v * sensor.cols + u];
                    PyImage.rgb[py_v * sensor.cols + py_u]   = src.rgb[v * sensor.cols + u];
            }
        }
    }
    void pyramid::downPyramidDepth(float *src, int scale){
        if(scale != 1){
            cudaMallocManaged(&dstBlur, sizeof(float) * sensor.rows * sensor.cols);
            cudaDeviceSynchronize();

            float gaussKernel[25] = {1.0,4.0,6.0,4.0,1.0,
                                     4.0,16.0,24.0,16.0,4.0,
                                     6.0,24.0,36.0,24.0,6.0,
                                     4.0,16.0,24.0,16.0,4.0,
                                     1.0,4.0,6.0,4.0,1.0};
            // Convolving guassian kenrnel on image
            convolve(src, dstBlur, gaussKernel);    // applying gaussian blur
            // removing even columns and rows in parallel from the source image
            int threads_per_block = 64;
            int thread_blocks =(sensor.cols * sensor.rows + 
                threads_per_block - 1) / threads_per_block;
            // std::cout << "<<<kernel_downSampleKernel>>> threadBlocks: "<< thread_blocks << ", threadPerBlock: " << threads_per_block << std::endl;
            downSampleDepthKernel<<<thread_blocks, threads_per_block>>>(src, depth, sensor, scale);
            cudaDeviceSynchronize();
        }else{
            cudaMemcpy(depth, src, sizeof(float) * sensor.cols * sensor.rows,
                        cudaMemcpyDeviceToDevice);
            cudaDeviceSynchronize();
        }
    }
    void pyramid::downPyramidRGB(uchar3 *src, int scale){
        if(scale != 1){
            // cudaMallocManaged(&dstBlur, sizeof(uchar3) * sensor.rows * sensor.cols);
            // cudaDeviceSynchronize();

            // float gaussKernel[25] = {1.0,4.0,6.0,4.0,1.0,
            //                          4.0,16.0,24.0,16.0,4.0,
            //                          6.0,24.0,36.0,24.0,6.0,
            //                          4.0,16.0,24.0,16.0,4.0,
            //                          1.0,4.0,6.0,4.0,1.0};
            // // Convolving guassian kenrnel on image
            // convolve(src, dstBlur, gaussKernel);    // applying gaussian blur
            // removing even columns and rows in parallel from the source image
            int threads_per_block = 64;
            int thread_blocks =(sensor.cols * sensor.rows + 
                threads_per_block - 1) / threads_per_block;
            // std::cout << "<<<kernel_downSampleKernel>>> threadBlocks: "<< thread_blocks << ", threadPerBlock: " << threads_per_block << std::endl;
            downSampleRGBKernel<<<thread_blocks, threads_per_block>>>(src, rgb, sensor, scale);
            cudaDeviceSynchronize();
        }else{
            cudaMemcpy(rgb, src, sizeof(uchar3) * sensor.cols * sensor.rows,
                        cudaMemcpyDeviceToDevice);
            cudaDeviceSynchronize();
        }
    }
    void pyramid::downPyramid(rgbdImage src, int scale){
        if(scale != 1){
            cudaMallocManaged(&dstBlur, sizeof(float) * sensor.rows * sensor.cols);
            cudaDeviceSynchronize();

            float gaussKernel[25] = {1.0,4.0,6.0,4.0,1.0,
                                     4.0,16.0,24.0,16.0,4.0,
                                     6.0,24.0,36.0,24.0,6.0,
                                     4.0,16.0,24.0,16.0,4.0,
                                     1.0,4.0,6.0,4.0,1.0};
            // Convolving guassian kenrnel on image
            convolve(src.depth, dstBlur, gaussKernel);    // applying gaussian blur
            // removing even columns and rows in parallel from the source image
            int threads_per_block = 64;
            int thread_blocks =(sensor.cols * sensor.rows + 
                threads_per_block - 1) / threads_per_block;
            // std::cout << "<<<kernel_downSampleKernel>>> threadBlocks: "<< thread_blocks << ", threadPerBlock: " << threads_per_block << std::endl;
            downSampleKernel<<<thread_blocks, threads_per_block>>>(*this, src, sensor, scale);
            cudaDeviceSynchronize();
        }else{
            cudaMemcpy(depth, src.depth, sizeof(float) * sensor.cols * sensor.rows,
                        cudaMemcpyDeviceToDevice);
            cudaMemcpy(rgb, src.rgb, sizeof(uchar3) * sensor.cols * sensor.rows,
                        cudaMemcpyDeviceToDevice);
            cudaDeviceSynchronize();
        }
    }
    __global__   // todo ...
    void replicationPaddingKernel(){
    }

    __host__ __device__
    float pyramid::vertexDistance(int x, int y, int nx, int ny) {
        
        float res = distance(make_float3(x,y,0), make_float3(nx, ny,0));
        // printf("x: %d, y: %d , nx: %d, ny: %d dist: %f \n", x, y, nx, ny, res);
        return res;
    }
    __host__ __device__ 
    double pyramid::gaussian(float x, double sigma) {
        return exp(-(pow(x, 2))/(2 * pow(sigma, 2))) / (2 * M_PI * pow(sigma, 2));
    
    }
    __host__ __device__
    void pyramid::applyBilateralFilter(float* dst, float* src, int x, int y, int diameter, float sigmaI, float sigmaS, rgbdSensor sensor ) {
        float iFiltered = 0;
        float wP = 0;
        int neighbor_x = 0;
        int neighbor_y = 0;
        int half = diameter / 2;
        int pixelID = x * sensor.cols + y;
        for(int i = 0; i < diameter; i++) {
            for(int j = 0; j < diameter; j++) {
                neighbor_x = x - (half - i);
                neighbor_y = y - (half - j);
                int nID = neighbor_x * sensor.cols + neighbor_y;
                float gi = gaussian(src[nID] - src[pixelID], sigmaI);
                float gs = gaussian(vertexDistance(x, y, neighbor_x, neighbor_y), sigmaS);
                float w = gi * gs;
                iFiltered += src[nID] * w;
                wP += w;
            }
        }
        dst[pixelID] = iFiltered / wP;
    }

    __global__ 
    void bilateralFilterKernel( pyramid &filter,
                                float* dst, 
                                float* src, 
                                const int r,  
                                const float e_f, 
                                const float e_d,
                                rgbdSensor sensor) {

        int index = blockIdx.x * blockDim.x + threadIdx.x;
        int stride = blockDim.x * gridDim.x;
        int size = sensor.rows * sensor.cols;
        for (int idx = index; idx < size; idx += stride){
            int u = static_cast<int>(idx / sensor.cols);
            int v = static_cast<int>(idx - sensor.cols * u);
            filter.applyBilateralFilter(dst, src, u, v, 2*r, e_f, e_d,sensor);
        }
    }
    void pyramid::bilateralFilter(float* dst, const gaussianKernal& kernel){
        int threads_per_block = 64;
        int thread_blocks =(sensor.cols * sensor.rows + threads_per_block - 1) / threads_per_block;
        // std::cout << "<<<kernel_bilateralFilter>>> threadBlocks: "<< thread_blocks 
        //           << ", threadPerBlock: " << threads_per_block 
        //           << ", Size: " << sensor.cols * sensor.rows 
        //           << std::endl;
        bilateralFilterKernel <<< thread_blocks, threads_per_block >>>(*this,
                                                                        dst, this->depth, 
                                                                        kernel.diameter,
                                                                        kernel.sigmaI, 
                                                                        kernel.sigmaS, 
                                                                        sensor);
        cudaDeviceSynchronize();
    }
    void pyramid::bilateralFilter(float* dst, float* src, const gaussianKernal& kernel){
        int threads_per_block = 64;
        int thread_blocks =(sensor.cols * sensor.rows + threads_per_block - 1) / threads_per_block;
        // std::cout << "<<<kernel_bilateralFilter>>> threadBlocks: "<< thread_blocks 
        //           << ", threadPerBlock: " << threads_per_block 
        //           << ", Size: " << sensor.cols * sensor.rows 
        //           << std::endl;
        bilateralFilterKernel <<< thread_blocks, threads_per_block >>>(*this,
                                                                        dst, src, 
                                                                        kernel.diameter,
                                                                        kernel.sigmaI, 
                                                                        kernel.sigmaS, 
                                                                        sensor);
        cudaDeviceSynchronize();    
    }


}  // namespace DynaMap



// for (int idx = 0; idx < 100; idx++){
//     int v = static_cast<int>(idx / 10);
//     int u = static_cast<int>(idx - 10 * v);
    
//     if( v >= 0 && v < 10 && 
//         u >= 0 && u < 10 ) {
//             std:: cout << "heree .." << std::endl;
//         if( v % 2 != 0 && u % 2 != 0){
//             uint py_u = floor(u / 2);
//             uint py_v = floor(v / 2);
//             std:: cout << "pyidx: "<< py_u * 5 + py_v  << ", origidx: " << u * 10 + v<< std::endl;
//         }
//     }
// }
