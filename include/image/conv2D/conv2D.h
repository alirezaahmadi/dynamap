/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
#pragma once

#include <cuda.h>
#include <cuda_runtime.h>

#include "kernel2D/kernel2D.h"
#include "utils.h"

namespace DynaMap{

class conv2D : public kernel2D , public virtual rgbdImage {
    public:

    conv2D();
    ~conv2D();

    void convolve(float *src, float *dst, float* kernel);
};

}  // namespace DynaMap