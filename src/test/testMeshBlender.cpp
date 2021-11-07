/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

#include "dqBlender/dqBlender.h"
#include "mesh/meshSTD.h"
#include "mesh/mesh_IO.h"

using namespace DynaMap;

int main(int argc, char **argv) {
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

  blender::dqBlender *blender;
  cudaMallocManaged(&blender, sizeof(blender::dqBlender));
  cudaDeviceSynchronize();

  math::dualQuat *DQ;
  cudaMallocManaged(&DQ, sizeof(math::dualQuat) * srcMesh->verticesNum);

  geometry::defGraph *graph;
  cudaMallocManaged(&graph, sizeof(geometry::defGraph));
  cudaDeviceSynchronize();

  if(false){

    for (size_t i = 0; i < inpMesh.verticesNum; i++) {
      DQ[i].setIdentity();
      DQ[i].addTranslation(make_float3(0.0, 0.5, 0.0));
    }
    // DQ[0].addTranslation(make_float3(0.01, 0.01, 0.0));
    // DQ[6].addTranslation(make_float3(0.01, 0.01, 0.0));
    // DQ[20].addTranslation(make_float3(0.01, 0.01, 0.0));

    blender->init(srcMesh);
    blender->blendMesh(*dstMesh, DQ);

  }else{

    graph->KDTREE = false;
    // mode:0 random initialization, 1: exact match
    graph->init(*srcMesh, srcMesh->verticesNum, 1);
    graph->defGraphUpdateNodes();

    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    float4x4 cuPose = float4x4(pose.data()).getTranspose();

    graph->updateActiveNeighbourNodes(*srcMesh, cuPose);

    // graph->nodes[0].dq.addTranslation(make_float3(0.0, -0.5, 0.0));
    // graph->nodes[6].dq.addTranslation(make_float3(0.2, 0.0, 0.0));
    
    for (size_t i = 0; i < (int)NODE_NUM/2; i++) {
      graph->nodes[i].dq.setIdentity();
      graph->nodes[i].dq.addTranslation(make_float3(0.0, -0.5, 0.5));
      // std::cout << i << ": "<< graph->nodes[i].dq << std::endl;
    }

    blender->blendMesh(*dstMesh, *graph);
  }

  cudaMemcpy(inpMesh.vertices, dstMesh->vertices, sizeof(geometry::Vertex) * dstMesh->verticesNum,
             cudaMemcpyDeviceToHost);
  cudaDeviceSynchronize();

  inpMesh.saveOBJ("../meshes/blendedModel.obj");

  cudaDeviceSynchronize();
  cudaFree(DQ);
  cudaFree(graph);
  cudaFree(srcMesh);
  cudaFree(dstMesh);
  cudaFree(blender);

  return 0;
}
