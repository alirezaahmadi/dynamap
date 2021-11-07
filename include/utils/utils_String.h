/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
#pragma once
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <string>

#include <iostream>
#include <fstream>

#include <cuda.h>
#include <cuda_runtime.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include"geom_Types.h"

#define IDX2C(i,j,ld) (((j)*(ld))+(i)) 

namespace DynaMap{
namespace utils{

    int splitLine(const std::string& refLine, std::vector<std::string>& words);
    std::vector<double> splitLinetoNumbers(const std::string& refLine);

    void writeNormalsToTxt(const std::string& fileName, 
                            geometry::PointCloudXYZ& Points,  
                            int size);
                
    void writePoseToTxt(const std::string& fileName, 
                        const Eigen::Matrix4f& Pose,
                           int count);
    
    // template <class T>
    void writeMatrixToTxt(const char* fileName, 
                          float *matrix,
                          int rows, int cols,
                          int mode);
        
} // end of namespace utils

}