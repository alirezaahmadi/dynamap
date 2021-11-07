/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
#pragma once
#include <cuda.h>
#include <cuda_runtime.h>

namespace DynaMap{

struct rgbdSensor{
    unsigned int rows;
    unsigned int cols;
    float2 f;
    float2 c;
    float depthScaler;
};

}  //namespace DynaMap