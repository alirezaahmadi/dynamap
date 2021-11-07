/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
#pragma once

#include <stdio.h>
#include <iostream>
#include <cuda.h>
#include <cuda_runtime.h>

namespace DynaMap{

namespace geometry{

typedef float3 PointXYZ;
typedef float3 NormalXYZ;

struct PointXYZRGB{
  PointXYZ point;
  uchar3 rgb;
};

struct PointCloudXYZ{
  PointXYZ *points;
  NormalXYZ *normals;
};

struct PointCloudXYZRGB{
  PointXYZ *points;
  NormalXYZ *normals;
  uchar3 *colors;
};

/**
 * @brief      Structure representing a 3D We furthermore use trilinear interpolation on the SDF and intensity of neighboring voxels to alleviate discretization effects of the voxel gridvertex
 */
struct Vertex {
  /** The 3D position of the vertex */
  float3 position;
  /* normal at vertex position */
  float3 normal;
  /** The color of the vertex */
  float3 color;
};

/**
 * @brief      Structure representing a triangle
 */
struct Triangle {
  /** The first vertex of the triangle */
  Vertex v0;

  /** The second vertex of the triangle */
  Vertex v1;

  /** The third vertex of the triangle */
  Vertex v2;
};

struct Polygon{
  // indexes of vertices forming each tirangle 
  int3 vertexIndex;
  // indices of normals 
  int3 normalIndex;
  // face normal of each triangle
  float3 normal;
};

class Voxel {
  public:
  uchar3 color; // Voxel color
  unsigned char weight; // Voxel wieght
  float sdf; //Signed distance function

  __host__ __device__ 
  void mergeVoxel(const Voxel& voxel, int max_weight) {
      mergeVoxelSDF(voxel);
      mergeVoxelColor(voxel, voxel.color.x, color.x);
      mergeVoxelColor(voxel, voxel.color.y, color.y);
      mergeVoxelColor(voxel, voxel.color.z, color.z);
      mergeVoxelWeight(voxel, max_weight);
  }
  __host__ __device__ 
  void mergeVoxelColor(const Voxel& voxel, const unsigned char& voxelProperty, unsigned char& oldvoxelProperty){
      oldvoxelProperty =  static_cast<unsigned char>((static_cast<float>(oldvoxelProperty) * static_cast<float>(weight) +
                          static_cast<float>(voxelProperty) * static_cast<float>(voxel.weight)) /
                          (static_cast<float>(weight) + static_cast<float>(voxel.weight)));
  }
  __host__ __device__ 
  void mergeVoxelSDF(const Voxel& voxel){
      sdf =  static_cast<float>((sdf * static_cast<float>(weight) +
              static_cast<float>(voxel.sdf) * static_cast<float>(voxel.weight)) /
              (static_cast<float>(weight) + static_cast<float>(voxel.weight)));
  }
  __host__ __device__ 
  void mergeVoxelWeight(const Voxel& voxel, int max_weight){
      weight = weight + voxel.weight;
      if (weight > max_weight) weight = max_weight;
  }
};

class VoxelBlock{
  public:

  Voxel* voxels; // address of fisrt voxel in voxel array
  int blockSize; // nmuber of voxels in one edge of each block 

  void init(Voxel* _voxels, int _blockSize){
      voxels = _voxels;
      blockSize = _blockSize;
  }
  __host__ __device__ 
  Voxel &at(int3 position) {
      return voxels[position.x * blockSize * blockSize +
                      position.y * blockSize + position.z];
  }
  __host__ __device__ 
  Voxel &at(int idx) {
      return voxels[idx];
  }
};
    
}  // namespace DynaMap

}  // namespace geometry
