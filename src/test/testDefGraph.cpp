/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

#include "defGraph/defGraph.h"
#include "dqBlender/dqBlender.h"
#include "mesh/mesh_IO.h"
#include "nonRigidICP.h"

#include "utils_Dataset.h"
#include "utils_String.h"
#include "utils_Timer.h"

using namespace DynaMap;

int main(int argc, char **argv) {

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
  OptimizerProp.regularization = 5.0f;

  geometry::Mesh_IO inpMesh(argv[1]);
  geometry::Mesh_IO targMesh(argv[2]);
  inpMesh.updateVerticesNormals();
  targMesh.updateVerticesNormals();
  if (argc >= 4 && atoi(argv[3]) == 1) {
    std::cout << "Converting Mesh to STDMesh ..." << std::endl;
    inpMesh.Mesh2MeshSTDCompressed();
  }
  // inpMesh.saveOBJ("../meshes/loadedMesh.obj", 1);
  blender::dqBlender *blender;
  cudaMallocManaged(&blender, sizeof(blender::dqBlender));
  cudaDeviceSynchronize();

  geometry::MeshSTD *srcMesh;
  cudaMallocManaged(&srcMesh, sizeof(geometry::MeshSTD));
  srcMesh->Init(inpMesh.verticesNum, inpMesh.trianglesNum);
  cudaDeviceSynchronize();

  cudaMemcpy(srcMesh->vertices, inpMesh.vertices, sizeof(geometry::Vertex) * srcMesh->verticesNum,
             cudaMemcpyHostToDevice);
  cudaMemcpy(srcMesh->triangles, inpMesh.triangles, sizeof(geometry::Polygon) * srcMesh->trianglesNum,
             cudaMemcpyHostToDevice);
  cudaDeviceSynchronize();

  geometry::MeshSTD *targetMesh;
  cudaMallocManaged(&targetMesh, sizeof(geometry::MeshSTD));
  targetMesh->Init(targMesh.verticesNum, targMesh.trianglesNum);
  cudaDeviceSynchronize();

  cudaMemcpy(targetMesh->vertices, targMesh.vertices, sizeof(geometry::Vertex) * targetMesh->verticesNum,
             cudaMemcpyHostToDevice);
  cudaMemcpy(targetMesh->triangles, targMesh.triangles, sizeof(geometry::Polygon) * targetMesh->trianglesNum,
             cudaMemcpyHostToDevice);
  cudaDeviceSynchronize();

  geometry::defGraph *graph;
  cudaMallocManaged(&graph, sizeof(geometry::defGraph));
  cudaDeviceSynchronize();

  graph->KDTREE = false;
  utils::timerStart();
  graph->init(*srcMesh, srcMesh->verticesNum, 1);  // mode:0 random initialization, 1: exact match
  utils::timerStop();
  std::cout << "init graph... " << utils::tval() << std::endl;

  utils::timerStart();
  graph->defGraphUpdateNodes();
  utils::timerStop();
  std::cout << "updatenodes... " << utils::tval() << std::endl;

  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

  solver::nonRigidICP *Solver;
  cudaMallocManaged(&Solver, sizeof(solver::nonRigidICP));
  cudaDeviceSynchronize();

  blender->init(*graph);
  utils::timerStart();
  Solver->init(*graph, OptimizerProp, targetMesh->verticesNum);
  utils::timerStop();
  std::cout << "Solver Init... " << utils::tval() << std::endl; 

  utils::timerStart();
  Solver->solve(*graph, *blender, *targetMesh, pose);
  utils::timerStop();
  std::cout << "Solve... " << utils::tval() << std::endl; 
  
  // graph->writeDefGraphToFile("../logs/DefGraph.txt");

  geometry::MeshSTD *dstMesh;
  cudaMallocManaged(&dstMesh, sizeof(geometry::MeshSTD));
  dstMesh->Init(inpMesh.verticesNum, inpMesh.trianglesNum);
  cudaDeviceSynchronize();

  cudaMemcpy(dstMesh->vertices, srcMesh->vertices, sizeof(geometry::Vertex) * srcMesh->verticesNum,
             cudaMemcpyDeviceToDevice);
  cudaMemcpy(dstMesh->triangles, srcMesh->triangles, sizeof(geometry::Polygon) * srcMesh->trianglesNum,
             cudaMemcpyDeviceToDevice);
  cudaDeviceSynchronize();

  cudaMemcpy(inpMesh.vertices, dstMesh->vertices, sizeof(geometry::Vertex) * srcMesh->verticesNum,
             cudaMemcpyDeviceToHost);
  cudaDeviceSynchronize();

  inpMesh.updateNormalIndices();
  inpMesh.saveOBJ("../meshes/warppedModel.obj", 1);

  cudaDeviceSynchronize();
  cudaFree(blender);
  cudaFree(Solver);
  cudaFree(graph);
  cudaFree(srcMesh);
  cudaFree(dstMesh);
  cudaFree(targetMesh);

  return 0;
}
