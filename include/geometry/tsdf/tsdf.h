// original implementation Refusion: https://github.com/PRBonn/refusion
#pragma once

#include <stdio.h>
#include <iostream>
#include <cuda.h>
#include <cuda_runtime.h>
#include "geom_Types.h"
#include "sensor/rgbdImage.h"
#include "utils.h"
#include "utils_matrix.h"
#include "mesh/mesh.h"
#include "hashTable/hashTable.h"

namespace DynaMap{

namespace tsdf{

struct tsdfProperties{
  float voxelSize; // in meters
  
  int numBuckets;
  int bucketSize;
  int numBlocks;
  int blockSize;
  
  int entriesNum;
  int voxelNumPerAxes;

  int maxSDFWeight;
  float truncationDistance;

  float maxSensorDepth;
  float minSensorDepth;
};

class tsdfVolume : public HashTable{
  public:

    tsdfProperties Properties;

    tsdfVolume(void);
    virtual ~tsdfVolume(void);

    void init(const tsdfProperties& _Properties);
    void Free();

    __device__ 
    geometry::Voxel GetVoxel(float3 position);
    __device__ 
    bool SetVoxel(float3 position, const geometry::Voxel& voxel);
    __device__ 
    bool UpdateVoxel(float3 position, const geometry::Voxel& voxel);
    
    void IntegrateScan(const rgbdImage &image, float4x4 camera_pose);

    geometry::Mesh ExtractMesh(const float3 &lower_corner, const float3 &upper_corner);
    __host__ __device__
    tsdfProperties GetProperties();
    __device__
    float3 voxelArrayIndexToWorld(int index);
    __device__
    float3 GlobalVoxelToArrayIndex(int3 position);
    __device__ 
    geometry::Voxel GetInterpolatedVoxel(float3 position);

    float* GenerateDepth(float4x4 camera_pose, rgbdSensor sensor);

    uchar3* GenerateRgb(float4x4 camera_pose, rgbdSensor sensor);
    __device__ 
    int WorldToGlobalVoxel(float3 position);
  protected:
    __device__ 
    float3 GlobalVoxelToWorld(int3 position);
    __device__
    int3 WorldToGlobalVoxelHashing(float3 position);
    __device__ 
    int3 WorldToBlock(float3 position);
    __device__ 
    int3 WorldToLocalVoxel(float3 position);
};
    
}  // namespace tsdf

}  // namespace DynaMap