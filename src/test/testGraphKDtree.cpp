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

#include "dataStructure/kdtree/kdtree.h"

using namespace DynaMap;

int main(int argc, char **argv) {

  // Initializing GaussNewton LS Properties ************************************
  solver::Properties OptimizerProp;
  OptimizerProp.maxIteration = 100;
  OptimizerProp.lvlIterationNum[0] = 8;
  OptimizerProp.lvlIterationNum[1] = 4;
  OptimizerProp.lvlIterationNum[2] = 10;
  OptimizerProp.lvlScale[0] = 4;
  OptimizerProp.lvlScale[1] = 2;
  OptimizerProp.lvlScale[2] = 1;
  OptimizerProp.minIncrement = 1e-2f;
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
  graph->KDTREE = true;
  std::cout << "init graph Start... " << utils::tval() << std::endl;
  utils::timerStart();
  graph->init(*srcMesh, targetMesh->verticesNum, 1);
  utils::timerStop();
  std::cout << "init graph... " << utils::tval() << std::endl;

  utils::timerStart();
  graph->defGraphUpdateNodes();
  utils::timerStop();
  std::cout << "updatenodes... " << utils::tval() << std::endl;
  graph->writeDefGraphToFile("../logs/DefGraph.txt",0);
  std::cout << "write done... " << utils::tval() << std::endl;
  geometry::defGraph *host_graph;
  cudaMemcpy(host_graph, graph, sizeof(geometry::defGraph) * graph->nodeNum, cudaMemcpyDeviceToHost);
  cudaDeviceSynchronize();


  struct kdNode testNode = {0, {-0.0474, 0.123187, -0.208831}};

  utils::timerStart();
  graph->graphKDTree->findKNN(testNode);
  utils::timerStop();
  std::cout << "find KNN done... " << utils::tval() << std::endl;

  printf("searching for (%g, %g, %g)\n"
          "found (%g, %g, %g) dist %g\n ID: %d, seen %d nodes\n",
          testNode.x[0], testNode.x[1], testNode.x[2],
          graph->graphKDTree->kdFound->x[0], graph->graphKDTree->kdFound->x[1], graph->graphKDTree->kdFound->x[2],
          sqrt(graph->graphKDTree->kdDistnaces[0]), 
          graph->graphKDTree->kdFound->id,
          graph->graphKDTree->visited);

  // Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

  // solver::nonRigidICP *Solver;
  // cudaMallocManaged(&Solver, sizeof(solver::nonRigidICP));
  // cudaDeviceSynchronize();

  // blender->init(*graph);

  // Solver->init(*graph, OptimizerProp, targetMesh->verticesNum);
  // Solver->solve(*graph, *blender, *targetMesh, pose);

  // geometry::MeshSTD *dstMesh;
  // cudaMallocManaged(&dstMesh, sizeof(geometry::MeshSTD));
  // dstMesh->Init(inpMesh.verticesNum, inpMesh.trianglesNum);
  // cudaDeviceSynchronize();

  // cudaMemcpy(dstMesh->vertices, srcMesh->vertices, sizeof(geometry::Vertex) * srcMesh->verticesNum,
  //            cudaMemcpyDeviceToDevice);
  // cudaMemcpy(dstMesh->triangles, srcMesh->triangles, sizeof(geometry::Polygon) * srcMesh->trianglesNum,
  //            cudaMemcpyDeviceToDevice);
  // cudaDeviceSynchronize();

  // cudaMemcpy(inpMesh.vertices, dstMesh->vertices, sizeof(geometry::Vertex) * srcMesh->verticesNum,
  //            cudaMemcpyDeviceToHost);
  // cudaDeviceSynchronize();

  // inpMesh.updateNormalIndices();
  // inpMesh.saveOBJ("../meshes/warppedModel.obj", 1);

  cudaDeviceSynchronize();
  cudaFree(blender);
  // cudaFree(Solver);
  cudaFree(graph);
  cudaFree(srcMesh);
  // cudaFree(dstMesh);
  cudaFree(targetMesh);

  return 0;
}
