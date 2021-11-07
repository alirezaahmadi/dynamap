/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
#pragma once
#include <cuda_runtime.h>

#include "utils/utils.h"
#include "math/dualQuat.h"
#include "geometry/mesh/meshSTD.h"
#include "geometry/defGraph/defGraph.h"

namespace DynaMap{

namespace blender{

class dqBlender{
    public:

    dqBlender(void);
    virtual ~dqBlender(void);

    void init(geometry::MeshSTD *srcMesh);
    void init(geometry::defGraph& graph);
    void Free();

    __device__
    float3 blendVertexPose(math::dualQuat* subWarpField,
                           float* subWarpFiledWeights,
                           int* subWarpFiledIDs,
                           float3 vertexPose,
                           int vertexID);
    __device__
    float3 blendVertexPose(const geometry::defGraph& graph,
                           float3 vertexPose,
                           unsigned int vertexID);
    __device__
    void updateWeights(geometry::defGraph& graph,
                       unsigned int vertexID);
    __device__
    void sortNeighbour(geometry::defGraph& graph,
                            unsigned int vertexID);
    __device__ 
    void updateDistnaces(geometry::defGraph& graph,
                         unsigned int vertexID);
    __device__
    void updateVertexNeighbours(geometry::MeshSTD& srcMesh,
                                unsigned int vertexID,
                                float *nWeights,
                                int *nIds);
    
    void blendMesh(geometry::MeshSTD& dstMesh, 
                   geometry::defGraph& DefGraph);
            
    __device__
    void blendVertex(geometry::MeshSTD& dstMesh,
                    math::dualQuat *DualQuat,
                    unsigned int vertexID);
    __device__
    void blendVertexNormal(geometry::Vertex srctNromal,
                           geometry::Vertex dstNromal,
                           geometry::defGraph& graph,
                           unsigned int vertexID);
    __device__
    void blendVertexPose(geometry::Vertex srcVertex,
                         geometry::Vertex dstVertex,
                         geometry::defGraph& graph,
                         unsigned int vertexID);
    __device__
    void blendVertex(geometry::MeshSTD& dstMesh,
                     geometry::defGraph& DefGraph,
                     unsigned int vertexID);

    void blendMesh(geometry::MeshSTD& dstMesh, 
                   math::dualQuat *DualQuat);

    geometry::MeshSTD *sourceMesh;
    float *nWeights;
    int *nIds;
    float *nDistances;
}; 

}   // namespace blender
}   // namespace DynaMap

