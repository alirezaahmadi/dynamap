/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DynaMap: A non-Rigid SLAM implementation 					%
% by: Alireza Ahmadi                                     	%
% University of Bonn- MSc Robotics & Geodetic Engineering	%
% Alireza.Ahmadi@uni-bonn.de                             	%
% AlirezaAhmadi.xyz                                      	%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

#include <iostream>
#include <thread>
#include "builder/builder.h"
#include "utils_String.h"

#include "mesh/meshSTD.h"
#include "mesh/mesh_IO.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace DynaMap;

int cnt;
rgbdSensor Xtion;
rgbdImage deviceImage;
utils::dataSet Dataset;
pyramid pyDown;
pyramid filterImage;
gaussianKernal gKernel;

int main(int argc, char **argv) {

    srand(time(NULL));

    // Check that we have at least one CUDA device
    cudaError_t err0;
    int nb_devices;
    err0 = cudaGetDeviceCount(&nb_devices);
    if (err0 != cudaSuccess || nb_devices == 0) {
        printf("ERROR: No CUDA device found\n");
        return false;
    }
    // Select the first CUDA device as default
    err0 = cudaSetDevice(0);
    if (err0 != cudaSuccess) {
        printf("ERROR: Cannot set the chosen CUDA device\n");
        return false;
    }

    if (argc < 3) {
        std::cout << "enter number of frames and dataset address depth and rgb" << std::endl;
        return 0;
    }
    // Initializing GaussianKernel for Bilateral Filter **************************
    gKernel.diameter = 2;
    gKernel.sigmaI = 50.0f;
    gKernel.sigmaS = 3.0f;
    // Initializing RGBD-Sensor **************************************************
    Xtion.c.x = 319.5f;
    Xtion.c.y = 239.5f;
    Xtion.f.x = 525.0f;
    Xtion.f.y = 525.0f;
    // Xtion.f.x = 481.20;
    // Xtion.f.y = 480.00;
    Xtion.depthScaler = 5000;
    Xtion.rows = 480;
    Xtion.cols = 640;
    // Initializing Input Image **************************************************
    deviceImage.init(Xtion);
    // Load Dataset **************************************************************
    std::string folderAdd(argv[2]);
    std::string rgbFolder = "rgb/";
    std::string depthFolder = "depth/";
    std::string groundTruthFileName = "groundtruth.txt";
    Dataset.groundTruthFileAddress = folderAdd + groundTruthFileName;

    Dataset.PreSufixCheck = true;
    Dataset.rgbFolderAddress = folderAdd + rgbFolder;
    Dataset.depthFolderAddress = folderAdd + depthFolder;

    // Dataset.depthPrefix = "frame";
    // Dataset.rgbPrefix = "frame";
    // Dataset.rgbSuffix = ".png";
    // Dataset.depthSuffix = ".png";

    Dataset.depthPrefix = "";
    Dataset.rgbPrefix = "";
    Dataset.rgbSuffix = ".png";
    Dataset.depthSuffix = ".png";

    Dataset.curID = 0;
    Dataset.sensor = Xtion;
    Dataset.loadData();
    // ICP NN SVD ***************************************************************
    pyDown.init(Xtion);
    filterImage.init(Xtion);
    std::cout << "Main Loop: " << Dataset.rgbFileNames.size() << std::endl;
    for (std::vector<std::string>::iterator it = Dataset.rgbFileNames.begin();
        it != Dataset.rgbFileNames.begin() + atoi(argv[1]); ++it) {
        utils::timerStart();
        // Load Dataset **************************************************************
        Dataset.invokeNextFrame();
        // Dataset.invokeNextGroundTruthLine();
        Dataset.curID++;

        std::cout << "BilateralFilter Test" << std::endl;
        for (uint i = 0; i < Xtion.rows; i++) {
            for (uint j = 0; j < Xtion.cols; j++) {
                filterImage.rgb[i * Xtion.cols + j] = make_uchar3(Dataset.image.rgb.at<cv::Vec3b>(i, j)(0), 
                                                                Dataset.image.rgb.at<cv::Vec3b>(i, j)(1), 
                                                                Dataset.image.rgb.at<cv::Vec3b>(i, j)(2));
                filterImage.depth[i * Xtion.cols + j] = Dataset.image.depth.at<float>(i, j);
            }
        }

        filterImage.bilateralFilter(pyDown.depth, gKernel);

        cv::Mat filtered_IMG(Xtion.rows, Xtion.cols, CV_32FC1);
        for (int i = 0; i < Xtion.rows; i++) {
            for (int j = 0; j < Xtion.cols; j++) {
                filtered_IMG.at<float>(i, j) = pyDown.depth[i * Xtion.cols + j];
            }
        }
        cv::imshow("filtered_IMG", filtered_IMG);

        cv::waitKey(0);
        //**************************************************************
        utils::timerStop();
        std::cout << "*********** cnt: " << cnt++ << ", Time: " << utils::tval() << " ***********" << std::endl;
    }

    return 0;
}