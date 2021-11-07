/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
// Example to Run : 
// ./testDynaMap 1 /home/ipb/Datasets/rgbd_dataset_freiburg3_teddy/ ../Results/rigid/teddy.obj 

#include "defGraph/defGraph.h"
#include "dqBlender/dqBlender.h"
#include "mesh/mesh_IO.h"
#include "nonRigidICP.h"

#include "utils_Dataset.h"
#include "utils_String.h"
#include "utils_Timer.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace DynaMap;

int main(int argc, char **argv) {

  // Initializing RGBD-Sensor **************************************************
  rgbdSensor Xtion;
  Xtion.c.x = 319.5f;
  Xtion.c.y = 239.5f;
  Xtion.f.x = 525.0;
  Xtion.f.y = 525.0;
  Xtion.depthScaler = 5000;
  Xtion.rows = 480;
  Xtion.cols = 640;
  // Initializing GaussNewton LS Properties ************************************
  solver::Properties OptimizerProp;
  OptimizerProp.maxIteration = 100;
  OptimizerProp.lvlIterationNum[0] = 8;
  OptimizerProp.lvlIterationNum[1] = 4;
  OptimizerProp.lvlIterationNum[2] = 100;
  OptimizerProp.lvlScale[0] = 4;
  OptimizerProp.lvlScale[1] = 2;
  OptimizerProp.lvlScale[2] = 1;
  OptimizerProp.minIncrement = 1e-10f;
  OptimizerProp.regularization = 0.1f;
  // Load Dataset **************************************************************
  utils::dataSet Dataset;
  std::string folderAdd(argv[2]);
  std::string rgbFolder = "rgb/";
  std::string depthFolder = "depth/";
  std::string groundTruthFileName = "groundtruth.txt";
  std::string associationFileAddress = "associated.txt";

  if(argc >= 6){
    Dataset.assciateFileExist = atoi(argv[5]); 
    std::cout << "Associ: " << Dataset.assciateFileExist << std::endl; 
  }
  Dataset.groundTruthFileAddress = folderAdd + groundTruthFileName;
  Dataset.associationFileAddress = folderAdd + associationFileAddress;
  Dataset.PreSufixCheck = true;
  Dataset.rgbFolderAddress = folderAdd + rgbFolder;
  Dataset.depthFolderAddress = folderAdd + depthFolder;

  Dataset.depthPrefix = "";
  Dataset.rgbPrefix = "";
  Dataset.rgbSuffix = ".png";
  Dataset.depthSuffix = ".png";

  Dataset.curID = 0;
  Dataset.sensor = Xtion;
  Dataset.loadData();

  // Setup inpMesh **************************************************************
  geometry::Mesh_IO inpMesh(argv[3]);
  if (argc >= 5 && atoi(argv[4]) == 1) {
    std::cout << "Converting Mesh to STDMesh ..." << std::endl;
    inpMesh.Mesh2MeshSTDCompressed();
  }

  // Setup Blender **************************************************************
  blender::dqBlender *blender;
  cudaMallocManaged(&blender, sizeof(blender::dqBlender));
  cudaDeviceSynchronize();

  // Setup Src Mesh *************************************************************
  geometry::MeshSTD *srcMesh;
  cudaMallocManaged(&srcMesh, sizeof(geometry::MeshSTD));
  srcMesh->Init(inpMesh.verticesNum, inpMesh.trianglesNum);
  cudaDeviceSynchronize();

  cudaMemcpy(srcMesh->vertices, inpMesh.vertices, sizeof(geometry::Vertex) * srcMesh->verticesNum,
             cudaMemcpyHostToDevice);
  cudaMemcpy(srcMesh->triangles, inpMesh.triangles, sizeof(geometry::Polygon) * srcMesh->trianglesNum,
             cudaMemcpyHostToDevice);
  cudaDeviceSynchronize();

  // Setup Def Grph ************************************************************
  geometry::defGraph *graph;
  cudaMallocManaged(&graph, sizeof(geometry::defGraph));
  cudaDeviceSynchronize();

  graph->KDTREE = false;
  utils::timerStart();
  graph->init(*srcMesh, Xtion.rows * Xtion.cols, 1);
  utils::timerStop();
  std::cout << "init graph... " << utils::tval() << std::endl;

  utils::timerStart();
  graph->defGraphUpdateNodes();
  utils::timerStop();
  std::cout << "update graph nodes... " << utils::tval() << std::endl;

  // Setup Input image objects *************************************************
  pyramid liveFrame;
  liveFrame.init(Xtion);

  pyramid PredictedImage;
  PredictedImage.init(Xtion);

  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

  // float sin_t = std::sin(0.05);
  // float cos_t = std::cos(0.05);
  // Eigen::Matrix4f m;
  // m << 1, 0, 0, 0, 0, cos_t, -sin_t, 0, 0, sin_t, cos_t, 0, 0, 0, 0, 1;
  // pose = m * pose;

  solver::nonRigidICP *Solver;
  cudaMallocManaged(&Solver, sizeof(solver::nonRigidICP));
  cudaDeviceSynchronize();
  
  utils::timerStart();
  Solver->init(*graph, OptimizerProp, Xtion.rows * Xtion.cols);
  utils::timerStop();
  std::cout << "Solver Init... " << utils::tval() << std::endl; 

  std::cout << "Main Loop: " << Dataset.rgbFileNames.size() << std::endl;

  for (std::vector<std::string>::iterator it = Dataset.rgbFileNames.begin();
       it != Dataset.rgbFileNames.begin() + atoi(argv[1]); ++it) {
  	// Load Dataset **************************************************************
    Dataset.invokeNextFrame();
    // loading image *****************
    for (uint i = 0; i < Xtion.rows; i++) {
        for (uint j = 0; j < Xtion.cols; j++) {
            liveFrame.rgb[i * Xtion.cols + j] = make_uchar3(Dataset.image.rgb.at<cv::Vec3b>(i, j)(2), 
                                                            Dataset.image.rgb.at<cv::Vec3b>(i, j)(1),
                                                            Dataset.image.rgb.at<cv::Vec3b>(i, j)(0));
            liveFrame.depth[i * Xtion.cols + j] = Dataset.image.depth.at<float>(i, j);
        }
    }
    // datsaet RGB and Depth images view
    if (false) {
      cv::imshow("Dataset depth", Dataset.image.depth);
      cv::imshow("Dataset rgb", Dataset.image.rgb);
      cv::waitKey(0);
    }
    // Load Dataset **************************************************************
    Dataset.invokeNextFrame();
    // loading image *****************
    for (uint i = 0; i < Xtion.rows; i++) {
        for (uint j = 0; j < Xtion.cols; j++) {
            PredictedImage.rgb[i * Xtion.cols + j] = make_uchar3(Dataset.image.rgb.at<cv::Vec3b>(i, j)(2), 
                                                                 Dataset.image.rgb.at<cv::Vec3b>(i, j)(1),
                                                                 Dataset.image.rgb.at<cv::Vec3b>(i, j)(0));
            PredictedImage.depth[i * Xtion.cols + j] = Dataset.image.depth.at<float>(i, j);
        }
    }
    // ***************************************************************************
    // datsaet RGB and Depth images view
    if (false) {
      cv::imshow("Dataset depth", Dataset.image.depth);
      cv::imshow("Dataset rgb", Dataset.image.rgb);
      cv::waitKey(0);
    }
    
    std::cout << "Solver Start... " << std::endl; 
    utils::timerStart();
    Solver->solve(*graph,
                  *blender,
                  liveFrame,
                  PredictedImage,
                  pose,
                  Xtion);
    utils::timerStop();
    std::cout << "Solver Done... " << utils::tval() << std::endl; 
  }

  std::cout << "dst mesh init ..." << std::endl;
  geometry::MeshSTD *dstMesh;
  cudaMallocManaged(&dstMesh, sizeof(geometry::MeshSTD));
  dstMesh->Init(inpMesh.verticesNum, inpMesh.trianglesNum);
  cudaDeviceSynchronize();

  cudaMemcpy(dstMesh->vertices, srcMesh->vertices, sizeof(geometry::Vertex) * srcMesh->verticesNum,
             cudaMemcpyDeviceToDevice);
  cudaMemcpy(dstMesh->triangles, srcMesh->triangles, sizeof(geometry::Polygon) * srcMesh->trianglesNum,
             cudaMemcpyDeviceToDevice);
  cudaDeviceSynchronize();

  // graph->writeDefGraphToFile("../logs/DefGraph.txt");

  std::cout << "Blend ..." << std::endl;
  blender->init(*graph);
  blender->blendMesh(*dstMesh, *graph);

  cudaMemcpy(inpMesh.vertices, dstMesh->vertices, sizeof(geometry::Vertex) * srcMesh->verticesNum,
             cudaMemcpyDeviceToHost);
  cudaDeviceSynchronize();

  std::cout << "Save Mesh" << std::endl;
  inpMesh.saveOBJ("../meshes/warppedModel.obj", 1);

  std::cout << "Free Cuda" << std::endl;
  cudaDeviceSynchronize();
  cudaFree(blender);
  cudaFree(Solver);
  cudaFree(graph);
  cudaFree(srcMesh);
  cudaFree(dstMesh);

  return 0;
}
