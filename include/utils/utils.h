// Copyright 2019 Emanuele Palazzolo (emanuele.palazzolo@uni-bonn.de), Cyrill Stachniss, University of Bonn
// Modified and developed with Alireza Ahmadi 
#pragma once

#include <cuda_runtime.h>
#include <cmath>
#include "math/cuda_math.h"
#include "sensor/rgbdImage.h"
#include "sensor/rgbdSensor.h"
namespace DynaMap{
// dot product

__host__ __device__
inline unsigned int iDivUp( const unsigned int &a, const unsigned int &b ) 
{ 
      return ( a%b != 0 ) ? (a/b+1):(a/b); 
}
__host__ __device__
inline  float dot(float3 a, float3 b)
{ 
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
__host__ __device__
inline  int max(int a, int b)
{
  return a > b ? a : b;
}

__host__ __device__ 
inline int min(int a, int b)
{
  return a < b ? a : b;
}

__host__ __device__
inline  float3 element(float3 a, float3 b){ 
    return make_float3(a.x*b.x, a.y*b.y, a.z*b.z); 
}

__host__ __device__
inline  float rowSum3(float3 a){ 
    return (a.x + a.y + a.z); 
}

__host__ __device__
inline  float3 repmat3x1(float a){ 
    return make_float3(a, a, a); 
}

// __host__ __device__
// inline  float3 cross(float3 a, float3 b){ 
//     return make_float3(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x); 
// }

// __host__ __device__ inline float norm(const float3 &vec) {
//   return sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
// }

// __host__ __device__ inline float3 normalize(const float3 &vec) {
//   float vec_norm = norm(vec);
//   return make_float3(vec.x / vec_norm, vec.y / vec_norm, vec.z / vec_norm);
// }

// __host__ __device__ inline float3 operator+(const float3 &a, const float3 &b) {
//   return make_float3(a.x + b.x, a.y + b.y, a.z + b.z);
// }

// __host__ __device__ inline int3 operator+(const int3 &a, const int3 &b) {
//   return make_int3(a.x + b.x, a.y + b.y, a.z + b.z);
// }

// __host__ __device__ inline float3 operator-(const float3 &a, const float3 &b) {
//   return make_float3(a.x - b.x, a.y - b.y, a.z - b.z);
// }

// __host__ __device__ inline float3 operator*(const float3 &a, float b) {
//   return make_float3(a.x * b, a.y * b, a.z * b);
// }

// __host__ __device__ inline float3 operator*(float b, const float3 &a) {
//   return make_float3(a.x * b, a.y * b, a.z * b);
// }

// __host__ __device__ inline float3 operator/(const float3 &a, float b) {
//   return make_float3(a.x / b, a.y / b, a.z / b);
// }

__host__ __device__ inline float distance(const float3 &a, const float3 &b) {
  return norm(a - b);
}

__host__ __device__ inline int sign(float n) { return (n > 0) - (n < 0); }

__host__ __device__ inline float signf(float value) {
  return (value > 0) - (value < 0);
}

__host__ __device__ inline float3 ColorToFloat(uchar3 c) {
  return make_float3(static_cast<float>(c.x)/255,
                            static_cast<float>(c.y)/255,
                            static_cast<float>(c.z)/255);
}
 /// todo ... scaling point cloud doesn't work !!!
__host__ __device__ inline float3 getPoint3d(int i, float depth, rgbdSensor sensor, int scale = 1) {
  int v = i / sensor.cols;
  int u = i - sensor.cols * v;    
  float3 point;
  point.z = depth;
  point.x = (static_cast<float>(u) - sensor.c.x) * point.z / (sensor.f.x / scale);
  point.y = (static_cast<float>(v) - sensor.c.y) * point.z / (sensor.f.y / scale);
  return point;
}

__host__ __device__ inline int2 Project(float3 point3d, rgbdSensor sensor, int scale = 1) {
  float2 point2df;
  point2df.x = ((sensor.f.x / scale) * point3d.x) / point3d.z + sensor.c.x;
  point2df.y = ((sensor.f.y / scale) * point3d.y) / point3d.z + sensor.c.y;
  return make_int2(round(point2df.x), round(point2df.y));
}
}  // namespace DynaMap