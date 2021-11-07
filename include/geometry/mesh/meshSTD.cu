/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
#include "meshSTD.h"

namespace DynaMap {

namespace geometry {

  MeshSTD::MeshSTD(void){}
  MeshSTD::~MeshSTD(void){
    Free();
  }

  void MeshSTD::Init(unsigned int verticesNum_, unsigned int trianglesNum_) {
    trianglesNum = trianglesNum_;
    verticesNum = verticesNum_;
    triangleCnt = 0;
    vertexCnt = 0;
    cudaMallocManaged(&vertices,  sizeof(geometry::Vertex) * verticesNum_);
    cudaMallocManaged(&triangles, sizeof(geometry::Polygon) * trianglesNum_);

    cudaDeviceSynchronize();
  }
  void MeshSTD::Free() {
    cudaDeviceSynchronize();
    cudaFree(vertices);
    cudaFree(triangles);
  }
  __device__ 
  void MeshSTD::appendVertices(Vertex v0, Vertex v1, Vertex v2, int idx){
      maxTrianglesNum = 500000;
      if (triangleCnt <= maxTrianglesNum){
          float lambda = 0.0f;
          unsigned int idv1, idv2, idv3;
          Polygon tmpPoly;
          float error;
          int v_size = 0;
          bool idv1NExists = true; 
          bool idv2NExists = true;
          bool idv3NExists = true;
          if(triangleCnt == 0){
            // printf("first: %d, t: %d, v: %d \n", idx, triangleCnt, vertexCnt);
              tmpPoly.vertexIndex.x = vertexCnt + 1;
              tmpPoly.vertexIndex.y = vertexCnt + 2;
              tmpPoly.vertexIndex.z = vertexCnt + 3;
              triangles[triangleCnt] = tmpPoly;
              atomicAdd(&triangleCnt, 1);
              
              vertices[vertexCnt] = v0; atomicAdd(&vertexCnt, 1);
              vertices[vertexCnt] = v1; atomicAdd(&vertexCnt, 1);
              vertices[vertexCnt] = v2; atomicAdd(&vertexCnt, 1);
          }else{
              // printf("idx: %d, T: %d, V: %d \n", idx, triangleCnt, vertexCnt);
              v_size = vertexCnt; 
              for (size_t i = 0; i < v_size ; i++){
                  float3 v_pose = vertices[i].position;

                  error = distance(vertices[triangles[triangleCnt].vertexIndex.x].position, v_pose);
                  if((error <= lambda) && idv1NExists){
                      idv1 = i;
                      idv1NExists = false;
                  }
                  error = distance(vertices[triangles[triangleCnt].vertexIndex.y].position, v_pose);
                  if((error <= lambda) && idv2NExists){
                      idv2 = i;
                      idv2NExists = false;
                  }
                  error = distance(vertices[triangles[triangleCnt].vertexIndex.z].position, v_pose);
                  if((error <= lambda) && idv3NExists){
                      idv3 = i;
                      idv3NExists = false;
                  }
              }

              if(idv1NExists){
                  idv1 = vertexCnt;
                  vertices[vertexCnt] = v0;
                  atomicAdd(&vertexCnt, 1);
              }
              if(idv2NExists){
                  idv2 = vertexCnt;
                  vertices[vertexCnt] = v1;
                  atomicAdd(&vertexCnt, 1);
              }
              if(idv3NExists){
                  idv3 = vertexCnt;
                  vertices[vertexCnt] = v2;
                  atomicAdd(&vertexCnt, 1);
              }

              if(idv1NExists || idv2NExists || idv3NExists){
                  tmpPoly.vertexIndex.x = idv1;
                  tmpPoly.vertexIndex.y = idv2;
                  tmpPoly.vertexIndex.z = idv3;
                  triangles[triangleCnt] = tmpPoly;
                  atomicAdd(&triangleCnt, 1);
              }

              idv1NExists = true; 
              idv2NExists = true;
              idv3NExists = true;
          }
      } 
  }
  __global__
  void Mesh2MeshSTDKernel(MeshSTD& meshSTD, Mesh& mesh, int triangleNum){
    int index = blockIdx.x * blockDim.x + threadIdx.x;
        int stride = blockDim.x * gridDim.x;
        int size = triangleNum;
        for (int idx = index; idx < size; idx += stride){
          meshSTD.appendVertices(mesh.triangles_[idx].v0, mesh.triangles_[idx].v1,mesh.triangles_[idx].v2, idx);
        } 
  }
  void MeshSTD::Mesh2MeshSTD(Mesh& mesh, int triangleNum){
    int threads_per_block = 64;
    int thread_blocks =
    (triangleNum + threads_per_block - 1) / threads_per_block;
    Mesh2MeshSTDKernel <<<thread_blocks, threads_per_block>>>(*this, mesh, triangleNum);
    cudaDeviceSynchronize();
  }


}  // namespace geometry

}  // namespace DynaMap


