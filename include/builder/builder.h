/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
#pragma once

#include <cuda.h>
#include <cuda_runtime.h>

#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <string>

#include <ctime>
#include <fstream>
#include <utility>
#include <iostream>
#include <algorithm>

#include "DynaMap.h"
#include "gnSolver.h"
#include "tsdf/tsdf.h"
#include "mesh/mesh.h"
#include "geom_Types.h"
#include "sensor/rgbdImage.h"

#include "utils_Dataset.h"
#include "utils/utils_Timer.h"

#include "math/Quaternion.h"
#include "pyramid/pyramid.h"
#include "kernel2D/kernel2D.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>

namespace DynaMap{

    typedef Eigen::Matrix<double, 6, 1> Vector6d;

class builder {
    public:
    builder(void);
    void init(const tsdf::tsdfProperties& tsdfProp,
            const solver::Properties _OptimizerProp,
            const rgbdSensor &_sensor,
            const gaussianKernal &_kernel);
    virtual ~builder();
    // ******** todo to improve spped read image file not in cv type  *********
    void buildModelLS(utils::dataSet& dataset, int Mode=0);
    void buildModelSVD(const cv::Mat &rgb, cv::Mat &depth, int Mode);
    void buildModelGT(utils::dataSet& dataset);
    void SVDMatcher(const cv::Mat &rgb, const cv::Mat &depth);
    

    geometry::Mesh ExtractMesh(const float3 &lower_corner,
                           const float3 &upper_corner);
    
    uchar3* GenerateRgb(int width, int height);
    float* GenerateDepth(int width, int height);

    cv::Mat getCanonicalRgbCV(void);
    cv::Mat getCanonicalDepthCV(void);
    Eigen::Matrix4f getCurrentPose(void);

    void setInitialPose(std::vector<double>& vec);
    Eigen::Matrix4f v2tRef(std::vector<double>& vec);
    void writePoseToTxt(const char* fileName,
                                 std::vector<double>& vec, 
                                 const Eigen::Matrix4f& Pose,
                                 int count);

    solver::gnSolver GNSolver;

    pyramid targetImage;
    pyramid sourceImage;

    pyramid visImage;

    rgbdSensor sensor;

    tsdf::tsdfVolume *volume_;

    
    Eigen::Matrix4f pose_last;
    Eigen::Matrix4f init_pose;
    Vector6d prev_increment_;

    bool FirstFrame;

    gaussianKernal GKernel;

    geometry::PointCloudXYZ *targetPCL;
    geometry::PointCloudXYZ *sourcePCL;

    cv::Mat virtual_depthCV;
	cv::Mat virtual_rgbCV;

    int frameNum;

    std::vector<float> old_tran;
    int cntt;
    #ifdef EIGEN_SVD
    MapRecon::MapRecon eigenSVD;
    #endif

    private:
    Eigen::Matrix4f currentPose;
};

}  // namespace DynaMap

