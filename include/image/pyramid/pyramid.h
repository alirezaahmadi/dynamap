/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
#pragma once

#include <cuda.h>
#include <cuda_runtime.h>

#include "conv2D/conv2D.h"
#include "utils.h"

#define DEPTH_MAX 2.0f
#define DEPTH_MIN 0.15f

namespace DynaMap{

class pyramid : public conv2D , public virtual rgbdImage {
    public:

    ~pyramid();
    
    void downPyramid(rgbdImage src, int scale);
    void downPyramidDepth(float *src, int scale);
    void downPyramidRGB(uchar3 *src, int scale);

    __host__ __device__
    float vertexDistance(int x, int y, int nx, int ny);
    __host__ __device__ 
    double gaussian(float x, double sigma);
    __host__ __device__
    void applyBilateralFilter(float* dst, float* src, int x, int y, int diameter, float sigmaI, float sigmaS, rgbdSensor sensor);
    void bilateralFilter(float* dst, const gaussianKernal& kernel);
    void bilateralFilter(float* dst, float* src, const gaussianKernal& kernel);
    
    float* dstBlur;
};
}  // namespace DynaMap