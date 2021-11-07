/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
#pragma once

#include <cuda.h>
#include <cuda_runtime.h>
#include <curand.h>

#include "mesh/mesh.h"
#include "math/math_types.h"
#include "geom_Types.h"
#include "rgbdSensor.h"
#include "math/dualQuat.h"
#include "math/utils_matrix.h"
#include "geometry/mesh/meshSTD.h"

__device__ __host__
inline float deg2rad(const float &deg){ 
    return deg * M_PI / 180; 
}
__device__ __host__
inline float clamp(const float &lo, const float &hi, const float &v){ 
    return std::max(lo, std::min(hi, v)); 
}

#define CUDA_CALL(x) do { if((x) != cudaSuccess) { \
    printf("Error at %s:%d\n",__FILE__,__LINE__);     \
    return EXIT_FAILURE;}} while(0)
#define CURAND_CALL(x) do { if((x) != CURAND_STATUS_SUCCESS) { \
    printf("Error at %s:%d\n",__FILE__,__LINE__);            \
    return EXIT_FAILURE;}} while(0)


namespace DynaMap{
namespace geometry{

    class rayTracing{
        public:
            rayTracing(void);
            virtual ~rayTracing(void);
            
            void init(MeshSTD& mesh, rgbdSensor& sensor);
            void Free(void);
            __device__
            bool rayTriangleIntersect(const float3& cameraOrigin, 
                                      const float3& rayDirection,
                                      const float3& v0,
                                      const float3& v1,
                                      const float3& v2,
                                      float& t, 
                                      float& u, 
                                      float& v);
            __device__             
            bool intersec(const float3& cameraOrigin,
                          const float3& rayDirection,
                          const MeshSTD& mesh,
                          float& nearestTriDistance, 
                          size_t& triangleIndex, 
                          float2& uv);
            __device__
            void getSurfaceProperties(const Vertex& hitPoint,
                                      const float3& rayDirection,
                                      const MeshSTD& mesh,
                                      const size_t& triangleindex,
                                      const float2 &uv);              
            __device__
            float rayCast(const float3& cameraOrigin, 
                          const float3& rayDirection,
                          MeshSTD& mesh,
                          size_t rayID);

            void GenerateDepth(float4x4& cameraPose, 
                               float* virtual_depth);


            MeshSTD rayCastMesh;
            rgbdSensor rayCastingCamera;
    }; 

}   // namespace Geometry
}   // namespace DynaMap

