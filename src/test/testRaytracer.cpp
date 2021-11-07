/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

#include "dqBlender/dqBlender.h"
#include "image/pyramid/pyramid.h"
#include "mesh/meshSTD.h"
#include "mesh/mesh_IO.h"

#include "rayTracing/rayTracing.h"

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

  geometry::Mesh_IO inpMesh(argv[1]);
  if (argc >= 3 && atoi(argv[2]) == 1) inpMesh.Mesh2MeshSTD();

  geometry::MeshSTD *srcMesh;
  cudaMallocManaged(&srcMesh, sizeof(geometry::MeshSTD));
  srcMesh->Init(inpMesh.verticesNum, inpMesh.trianglesNum);

  geometry::MeshSTD *dstMesh;
  cudaMallocManaged(&dstMesh, sizeof(geometry::MeshSTD));
  dstMesh->Init(inpMesh.verticesNum, inpMesh.trianglesNum);
  cudaDeviceSynchronize();

  cudaMemcpy(srcMesh->vertices, inpMesh.vertices, sizeof(geometry::Vertex) * srcMesh->verticesNum,
             cudaMemcpyHostToDevice);
  cudaMemcpy(srcMesh->triangles, inpMesh.triangles, sizeof(geometry::Polygon) * srcMesh->trianglesNum,
             cudaMemcpyHostToDevice);
  cudaDeviceSynchronize();

  pyramid targetImage;
  targetImage.init(Xtion);

  geometry::rayTracing *tracer;
  cudaMallocManaged(&tracer, sizeof(geometry::rayTracing));
  tracer->init(*srcMesh, Xtion);

  float4x4 cameraPose;
  cameraPose.setIdentity();

  tracer->GenerateDepth(cameraPose, targetImage.depth);

  cv::Mat cv_virtual_depth(Xtion.rows, Xtion.cols, CV_32FC1);
  for (int i = 0; i < Xtion.rows; i++) {
    for (int j = 0; j < Xtion.cols; j++) {
      cv_virtual_depth.at<float>(i, j) = static_cast<float>(targetImage.depth[i * Xtion.cols + j]);
    }
  }
  cv_virtual_depth.convertTo(cv_virtual_depth, CV_32FC1, 1);
  cv::imshow("rendered Depth", cv_virtual_depth);
  cv::waitKey(0);

  cudaDeviceSynchronize();
  cudaFree(srcMesh);
  cudaFree(dstMesh);
  cudaFree(tracer);

  return 0;
}
