/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
#pragma once

#include <cuda.h>
#include <cuda_runtime.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "utils.h"

namespace DynaMap{

    struct gaussianKernal{
        int diameter;
        float sigmaI;
        float sigmaS;
        float* kernel;

        void print(void){
            std::cout << "diameter: "<< diameter
                      << "sigmaI: "<< sigmaI
                      << "sigmaS: "<< sigmaS 
                      << std::endl;
            for(int i=0; i< diameter+1; i++){
                std::cout << ", " << kernel[i];
            }
            std::cout<< std::endl;
        }
        void fprint(void){
            printf( "diameter: %d, sigmaI: %f, sigmaS: %f \n", diameter,sigmaI,sigmaS );
            for(int i=0; i<diameter; i++){
                printf( "%f ", kernel[i]);
            }
            printf( "\n");
        }
    };

class kernel2D {
    public:

    kernel2D();
    ~kernel2D();

    void generateFilterkernel1D(gaussianKernal& kernel);
    void generateFilterkernel2D(gaussianKernal& kernel);
    cv::Mat testgenerateGaussianKernel1D(gaussianKernal& kernel);
    cv::Mat testgenerateGaussianKernel2D(gaussianKernal& kernel);
};

}  // namespace DynaMap