/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
#pragma once

#include <vector>
#include <fstream>
#include <string>
#include <sstream>
#include <iostream>
#include <algorithm>    // std::sort

#include "sensor/rgbdSensor.h"
#include "utils_String.h"

//Eigen
#include <eigen3/Eigen/Dense>
#include <boost/filesystem.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace DynaMap{

namespace utils{

struct frame{
    cv::Mat rgb;
    cv::Mat depth;
    float3 t;
    float3 r;
    float4 q;
};

class dataSet{
  public:
    bool PreSufixCheck;
    std::string rgbFolderAddress;
    std::string depthFolderAddress;
    std::vector<std::string> depthFileNames;
    std::vector<std::string> rgbFileNames;
    std::size_t firstFrameID;
    std::size_t lastFrameID;
    std::size_t skipFrameNum;
    std::size_t curID;
    std::string depthPrefix;
    std::string rgbPrefix;
    std::string rgbSuffix;
    std::string depthSuffix;
    rgbdSensor sensor;
    int offsetLineGroundTruth;
    std::string groundTruthFileAddress;
    std::string associationFileAddress;
    std::vector<std::string> refLines;
    std::vector<double> groundTruthLine;

    bool groundTruthExist;
    bool assciateFileExist;

    frame image;
    
    dataSet(std::string mainDir);
    virtual ~dataSet();

    bool loadData(void);
    Eigen::Matrix4f getCurrentPose(void);
    void setCurrentPose(Eigen::Matrix4f Pose);
    std::vector<std::string> getFileContent(std::string fileName);
    void getAssociateContent(std::string fileName);
    void invokeNextGroundTruthLine();
    void invokeNextFrame();
    void invokeNextImage();
    std::vector<std::string> invokeFileNames(const std::string _folderAddress, 
                                             std::string _suffix, 
                                             std::string _prefix);
    
  private:

    static bool sortFunc(const std::string &s1, const std::string &s2);

    
};
} // end of namespace utils

}  // namespace DynaMap