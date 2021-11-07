/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
#pragma once

#include <cuda.h>
#include <cuda_runtime.h>


#include "sensor/rgbdSensor.h"
#include "geom_Types.h"
#include "utils.h"

// #include "pyramid/pyramid.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define SIZE_OF(x) sizeof(x)/sizeof(x[0])

namespace DynaMap{

    // struct gaussianKernal{
    //     int kernelRadius;
    //     float kernelDelta;
    //     float filterDelta;
    //     float* kernel;
    // };

class rgbdImage{
    public:
    
    ~rgbdImage();

    void init(const rgbdSensor& _sensor);

    __device__
    float3 depthtoNormal(float* depth, rgbdSensor sensor, int idx);
    __device__
    float3 vertextoNormal(geometry::PointCloudXYZ& cloud, rgbdSensor sensor, int idx);

    geometry::PointXYZ getPointXYZ(int u, int v) const;
    geometry::PointXYZRGB getPointXYZRGB(int u, int v) const;
    void getPointCloudXYZ(geometry::PointCloudXYZ& cloud, int scale = 1);

    void getNormalsfromVertices(geometry::PointCloudXYZ& cloud);
    void getNormalsfromDepthImage(geometry::PointCloudXYZ& cloud);

    cv::Mat testNormalsfromDepthImage(cv::Mat& depth, float depthScalar);
    cv::Mat testNormalsfromVertices(cv::Mat& depth, float depthScalar);
    cv::Mat testNormalsfromDepthImageCV(cv::Mat& depth, float depthScalar);

    cv::Mat getCVImagefromCudaDepth(void);
    void getCudafromCVDepth(cv::Mat cvDepth);
    
    float* depth;
    uchar3* rgb;
    rgbdSensor sensor;
};

}  // namespace DynaMap
