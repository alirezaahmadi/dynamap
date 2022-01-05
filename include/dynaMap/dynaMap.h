/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
#pragma once
// Rigid Registration Solver 
// #define GPU_SOLVER

// SOLVER_TYPE 0: Eigen Solver, 1: cuSparceSolver, 2: cuSolver 
#define SOLVER_TYPE 1

// #define DEBUG  // writing matrixes in files in /log
// #define DEBUG_DefGraph // saving graph structure in file /log
// #define LOG_EN  // stream out steps in termnial
#define LOG_MAX 100

// KDtree dimension
#define KNN 		4
#define KDTREE_MAX_DIM 	3

// Inex convertors 
#define IDX2C(i,j,ld) (((j)*(ld))+(i))
#define IDX2F(i,j,ld) ((((j)-1)*(ld))+((i)-1))

// size of variabels
#define SIZE_OF(x) sizeof(x)/sizeof(x[0])


/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DynaMap: A non-Rigid SLAM implementation 					%
% by: Alireza Ahmadi                                     	%
% University of Bonn- MSc Robotics & Geodetic Engineering	%
% Alireza.Ahmadi@uni-bonn.de                             	%
% AlirezaAhmadi.xyz                                      	%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
// #include <ros/ros.h>
// #include <ctime>

#include <iostream>
#include <thread>
#include "builder/builder.h"
#include "utils_String.h"
#include <numeric>

#include "mesh/meshSTD.h"
#include "mesh/mesh_IO.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "gui/openGL/openGL.h"

// #include "glow/glbase.h"
// #include "glow/glutil.h"
// #include "glow/util/X11OffscreenContext.h"

// #include "DepthProjector.h"
// #include "ObjReader.h"

using namespace DynaMap;
// using namespace glow;

#define POSE_LOG_EN false

int argc;
char **argv;
bool creatMesh;
int cnt;
float DownSamplingScale;
int PointsNum;
int idxNum;
rgbdSensor Xtion;
gaussianKernal gKernel;
solver::Properties OptimizerProp;
float4x4 initialPose;
rgbdImage deviceImage;
utils::dataSet Dataset("");
tsdf::tsdfProperties tsdfProp;
builder mapBuilder;
pyramid pyDown;
pyramid filterImage;
vector<double> runtimes;


void runSLAM(void);
void runGUI(void);

void slam_callback(void) {
  runSLAM();
}
void gui_callback(void) {
  // runGUI();
}

// int main(int _argc, char **_argv) {
//   argc = _argc;
//   argv = _argv;


//   // dataset Dir
//   datasetDir = argv[2]

//   srand(time(NULL));

//   // Check that we have at least one CUDA device
//   cudaError_t err0;
//   int nb_devices;
//   err0 = cudaGetDeviceCount(&nb_devices);
//   if (err0 != cudaSuccess || nb_devices == 0) {
//     printf("ERROR: No CUDA device found\n");
//     return false;
//   }
//   // Select the first CUDA device as default
//   err0 = cudaSetDevice(0);
//   if (err0 != cudaSuccess) {
//     printf("ERROR: Cannot set the chosen CUDA device\n");
//     return false;
//   }

//   if (argc < 3) {
//     std::cout << "enter number of frames and dataset address depth and rgb" << std::endl;
//     return 0;
//   }

//   // if(POSE_LOG_EN){
//   //   writePoseToTxt("../logs/LSPoses.txt", dataset.groundTruthLine, currentPose, frameNum++);
//   // }

//   // Launch a thread
//   std::thread slamThread(slam_callback);
//   std::thread guiThread(gui_callback);
//   // Join the thread with the main thread
//   slamThread.join();
//   guiThread.join();

//   return 0;
// }

// void runSLAM(void) {
//   // frame Counter
//   cnt = 0;
//   // Initializing Input Image **************************************************
//   deviceImage.init(Xtion);

//   std::cout << "TSDF voxelNum: " << tsdfProp.numBlocks * tsdfProp.blockSize << std::endl;
//   // ICP NN SVD ***************************************************************
//   mapBuilder.init(tsdfProp, OptimizerProp, Xtion, gKernel);
//   pyDown.init(Xtion);
//   filterImage.init(Xtion);
//   initialPose.setIdentity();
//   std::cout << "Main Loop: " << Dataset.rgbFileNames.size() << std::endl;
//   int framesToIntegrate = (Dataset.rgbFileNames.size() > atoi(argv[1])) 
//                         ? atoi(argv[1]) : Dataset.rgbFileNames.size();
//   for (std::vector<std::string>::iterator it = Dataset.rgbFileNames.begin();
//        it != Dataset.rgbFileNames.begin() + framesToIntegrate; ++it) {
//     utils::timerStart();
//     // Load Dataset **************************************************************
//     Dataset.invokeNextFrame();
//     // TSDF **************************************************************>
//     // mapBuilder.buildModelGT(Dataset);
//     mapBuilder.buildModelLS(Dataset, 0);
//     //**************************************************************
//     // mesh test
//     if (false) {
//       float3 low_limits = make_float3(-1, -1, 0);
//       float3 high_limits = make_float3(1, 1, 2);

//       geometry::Mesh *mesh;
//       cudaMallocManaged(&mesh, sizeof(geometry::Mesh));

//       geometry::MeshSTD *meshSTD;
//       cudaMallocManaged(&meshSTD, sizeof(geometry::MeshSTD));
//       *mesh = mapBuilder.ExtractMesh(low_limits, high_limits);
//       meshSTD->Init(mesh->num_triangles_ * 3, mesh->num_triangles_);
//       meshSTD->Mesh2MeshSTD(*mesh, mesh->num_triangles_);
//       if (false) {
//         // std::cout << "meshSTD: "<< meshSTD->trianglesNum << ", " << meshSTD->verticesNum << std::endl;
//         geometry::Mesh_IO inpMesh(meshSTD->verticesNum, meshSTD->trianglesNum);
//         cudaMemcpy(inpMesh.vertices, meshSTD->vertices, sizeof(geometry::Vertex) * meshSTD->verticesNum,
//                    cudaMemcpyDeviceToHost);
//         cudaMemcpy(inpMesh.triangles, meshSTD->triangles, sizeof(geometry::Polygon) * meshSTD->trianglesNum,
//                    cudaMemcpyDeviceToHost);
//         cudaDeviceSynchronize();
//         // std::cout << "InpMesh: "<<inpMesh.trianglesNum << ", " << inpMesh.verticesNum << std::endl;
//         inpMesh.saveOBJ("stdMesh.obj");
//       }
//       cudaFree(mesh);
//       mesh->Free();
//       cudaFree(meshSTD);
//       meshSTD->Free();
//     }
//     //**************************************************************
//     // datsaet RGB and Depth images view
//     if (false) {
//       // cv::imshow("Dataset depth", Dataset.image.depth);
//       cv::imshow("Dataset rgb", Dataset.image.rgb);
//     }
//     // virtual RGB and Depth images view
//     if (true) {
//       cv::imshow("virtual rgb", mapBuilder.getCanonicalRgbCV());
//       // cv::moveWindow("virtual rgb", 300, 1000);
//       // cv::imshow("virtual depth",mapBuilder.getCanonicalDepthCV());
//     }
//     // show builder Normals
//     if (true) {
//       cv::Mat NormalImage(Xtion.rows, Xtion.cols, CV_32FC3);
//       for (int i = 0; i < Xtion.rows; i++) {
//         for (int j = 0; j < Xtion.cols; j++) {
//           NormalImage.at<cv::Vec3f>(i, j)[0] = mapBuilder.targetPCL->normals[i * Xtion.cols + j].x;
//           NormalImage.at<cv::Vec3f>(i, j)[1] = mapBuilder.targetPCL->normals[i * Xtion.cols + j].y;
//           NormalImage.at<cv::Vec3f>(i, j)[2] = mapBuilder.targetPCL->normals[i * Xtion.cols + j].z;
//         }
//       }
//       cv::imshow("BuilderNormnals", NormalImage);
//       cv::moveWindow("vertexNormnals", 1200, 300);
//     }
//     cv::waitKey(1);
//     //**************************************************************
//     // loading cameraPose into gui instance marker
//     // PredictedAxis.updatePose(mapBuilder.getCurrentPose());
//     // GroundTruthCamera.updatePose(Dataset.getCurrentPose());

//     // printing frame num and process time for current frame.
//     #ifdef LOG_EN
      
//       std::cout << mapBuilder.getCurrentPose() << std::endl;
//       std::cout << "***********" << std::endl;
//       std::cout << Dataset.getCurrentPose() << std::endl;
//     #endif 

//     utils::timerStop();
//     runtimes.push_back(utils::tval());
//     double avgRuntime = std::accumulate(runtimes.begin(),  runtimes.end(), 0.0) /  runtimes.size();
//     double fps = 1 /avgRuntime;
//     std::cout << "*********** cnt: " << cnt++ << ", AVG-FPS: " << fps  << ", Time: " << utils::tval() << " ***********" << std::endl;
//     // std::cout << "*********** cnt: " << cnt++ << ", Time: " << utils::tval() << std::endl;
//   }
//   // Crearting mesh from current model
//   if (false) {
//     utils::timerStart();
//     std::stringstream filepath_out;
//     filepath_out << "/result.txt";
//     std::ofstream result(filepath_out.str());
//     std::cout << "Creating mesh..." << std::endl;
//     float3 low_limits = make_float3(-3, -3, 0);
//     float3 high_limits = make_float3(3, 3, 4);
//     geometry::Mesh *mesh;
//     cudaMallocManaged(&mesh, sizeof(geometry::Mesh));
//     *mesh = mapBuilder.ExtractMesh(low_limits, high_limits);
//     filepath_out.str("");
//     filepath_out.clear();
//     filepath_out << "../meshes/currModel.obj";
//     mesh->SaveToFile(filepath_out.str());
//     std::cout << "End..." << std::endl;
//     if (false) {
//       // X11OffscreenContext ctx;
//       // inititializeGLEW();
//       // uint32_t width = 640, height = 480;

//       // Eigen::Matrix4f P = glow::glPerspective(glow::radians(58.0f), 4.0 / 3.0, 0.03, 2.0f);  // World -> Image -> NDC
//       // Eigen::Matrix4f Tranform = glRotateX(M_PI);
//       // Eigen::Matrix4f view = mapBuilder.currentPose;
//       // // view(2,3) += -2.0f;
//       // view = Tranform * view;
//       // DepthProjector proj(width, height, P);

//       // std::vector<float3> vertices;
//       // for (size_t i = 0; i < mesh->num_triangles_; i++) {
//       //   vertices.push_back(mesh->triangles_[i].v0.position);
//       //   vertices.push_back(mesh->triangles_[i].v1.position);
//       //   vertices.push_back(mesh->triangles_[i].v2.position);
//       // }
//       // std::cout << "vertices loaded..." << std::endl;

//       // proj.render(vertices, view);
//       // std::vector<glow::vec4> data(width * height);
//       // uint32_t count = 0;
//       // std::cout << "non-zeros = " << count << std::endl;
//       // proj.texture().download(data);
//       // for (auto d : data) {
//       //   count += (d.x > 0);
//       // }
//       // std::cout << "non-zeros = " << count << std::endl;
//       // proj.texture().save("../renders/test.ppm");
//     }
//     utils::timerStop();
//     std::cout << "***********MarchingCubes:  " << ", Time: " << utils::tval()  << " ***********" << std::endl;
//   }
  
//   return;
// }
// void runGUI(void) {
//   // Print usage
//   // printf(
//   //     "This is an implementation of the marching cubes algorithm in CUDA.\n"
//   //     "\nAlgorithm options:\n"
//   //     "\t'm'         stops/starts the algorithm.\n"
//   //     "\t'f'         changes the function of the surface to render.\n"
//   //     "\t'1','2','3' changes the dimension of the algorithm.\n"
//   //     "\t'+','-'     changes the number of points in the algorithm.\n"
//   //     "\nRendering options:\n"
//   //     "\t'g' shows/hides the grid.\n"
//   //     "\t'p' shows/hides the points.\n"
//   //     "\t's' shows/hides the resulting surface.\n"
//   //     "\nViewing options:\n"
//   //     "\t'ESC' and 'q' quits the program.\n"
//   //     "\t'Left click' rotates the figure.\n"
//   //     "\t'Right click' zooms in/out.\n\n");

//   // Initialize GLUT
//   glutInit(&argc, argv);

//   // Display mode
//   glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH | GLUT_STENCIL);

//   // Initialize the window settings.
//   glutInitWindowSize(400,400);
//   glutInitWindowPosition(800, 200);
//   glutCreateWindow("Display");

//   // Initialize the scene.
//   initGL();
  
//   // Initialize the data.
//   createVBOs(vbo);
  
//   // init cameras
//   MainCamera.z = -10.0f;
//   PredictedAxis.init();
//   GroundTruthAxis.init(5.0f);

//   // cudaMemcpy(glPoints, mapBuilder.targetPCL->points, sizeof(float3) * Xtion.rows * Xtion.cols, cudaMemcpyDeviceToHost);

//   // Set up GLUT call-backs.
//   glutDisplayFunc(display);
//   glutTimerFunc(33, timer, 33);  // redraw only every given millisec
//   glutReshapeFunc(reshape);
//   glutKeyboardFunc(keyboard);
//   glutMouseFunc(mouse);
//   glutMotionFunc(motion);
  
//   // // Start the GLUT main loop
//   glutMainLoop();
// }


