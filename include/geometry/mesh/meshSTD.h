/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
#pragma once
#include <cuda_runtime.h>
#include "geometry/geom_Types.h"
#include "utils/utils.h"
#include "mesh.h"

namespace DynaMap {

namespace geometry {

class MeshSTD {
 public:
  MeshSTD(void);
  virtual ~MeshSTD(void);
  
  void Init(unsigned int verticesNum_, unsigned int trianglesNum_);
  void Free();
  __device__ void appendVertices(Vertex v0, Vertex v1, Vertex v2, int idx);

  void Mesh2MeshSTD(Mesh& mesh, int triangleNum);

  Polygon *triangles;              
  unsigned int trianglesNum;
  unsigned int maxTrianglesNum;

  unsigned int triangleCnt;

  Vertex *vertices;
  unsigned int verticesNum;
  unsigned int maxVerticesNum;

  unsigned int vertexCnt;

  unsigned int textureNum;
  unsigned int normalNum;
  
};

}  // namespace tsdfvh

}  // namespace refusion
