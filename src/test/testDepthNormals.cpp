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
    std::cout << "Main Loop: " << Dataset.rgbFileNames.size() << std::endl;
    for (std::vector<std::string>::iterator it = Dataset.rgbFileNames.begin();
        it != Dataset.rgbFileNames.begin() + atoi(argv[1]); ++it) {
        utils::timerStart();
        // Load Dataset **************************************************************
        Dataset.invokeNextFrame();
        // Dataset.invokeNextGroundTruthLine();
        Dataset.curID++;

        for (uint i = 0; i < Xtion.rows; i++) {
            for (uint j = 0; j < Xtion.cols; j++) {
                deviceImage.rgb[i * Xtion.cols + j] =
                    make_uchar3(Dataset.image.rgb.at<cv::Vec3b>(i, j)(2), Dataset.image.rgb.at<cv::Vec3b>(i, j)(1),
                                Dataset.image.rgb.at<cv::Vec3b>(i, j)(0));
                deviceImage.depth[i * Xtion.cols + j] = Dataset.image.depth.at<float>(i, j);
            }
        }

        geometry::PointCloudXYZ *Cloud;
        cudaMallocManaged(&Cloud, sizeof(geometry::PointCloudXYZ));
        cudaMallocManaged(&Cloud->points, sizeof(geometry::PointXYZ) * Xtion.rows * Xtion.cols);
        cudaMallocManaged(&Cloud->normals, sizeof(geometry::NormalXYZ) * Xtion.rows * Xtion.cols);
        cudaDeviceSynchronize();

        pyDown.downPyramidDepth(deviceImage.depth, 1);
        pyDown.getNormalsfromDepthImage(*Cloud);

        cv::Mat NormalsDepth(Xtion.rows, Xtion.cols, CV_32FC3);
        for (int i = 0; i < Xtion.rows; i++) {
        for (int j = 0; j < Xtion.cols; j++) {
            NormalsDepth.at<cv::Vec3f>(i, j)[0] = Cloud->normals[i * Xtion.cols + j].x;
            NormalsDepth.at<cv::Vec3f>(i, j)[1] = Cloud->normals[i * Xtion.cols + j].y;
            NormalsDepth.at<cv::Vec3f>(i, j)[2] = Cloud->normals[i * Xtion.cols + j].z;
            // std::cout << NormalImaged.at<cv::Vec3f>(i, j)[0] - NormalImage.at<cv::Vec3f>(i, j)[0] << std::endl;
        }
        }
        cv::imshow("Depth normals", NormalsDepth);
        cv::moveWindow("depth normals", 800, 300);

        pyDown.getPointCloudXYZ(*Cloud, 1);
        pyDown.getNormalsfromVertices(*Cloud);

        cv::Mat NormalsPCL(Xtion.rows, Xtion.cols, CV_32FC3);
        for (int i = 0; i < Xtion.rows; i++) {
        for (int j = 0; j < Xtion.cols; j++) {
            NormalsPCL.at<cv::Vec3f>(i, j)[0] = Cloud->normals[i * Xtion.cols + j].x;
            NormalsPCL.at<cv::Vec3f>(i, j)[1] = Cloud->normals[i * Xtion.cols + j].y;
            NormalsPCL.at<cv::Vec3f>(i, j)[2] = Cloud->normals[i * Xtion.cols + j].z;
            // std::cout << NormalImaged.at<cv::Vec3f>(i, j)[0] - NormalImage.at<cv::Vec3f>(i, j)[0] << std::endl;
        }
        }

        cv::imshow("PCL normals", NormalsPCL);

        cudaDeviceSynchronize();
        cudaFree(Cloud);
        cudaFree(Cloud->points);
        cudaFree(Cloud->normals);

        cv::waitKey(0);
        //**************************************************************
        utils::timerStop();
        std::cout << "*********** cnt: " << cnt++ << ", Time: " << utils::tval() << " ***********" << std::endl;
    }

    return 0;
}