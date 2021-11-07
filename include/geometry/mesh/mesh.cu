// Copyright 2019 Emanuele Palazzolo (emanuele.palazzolo@uni-bonn.de), Cyrill Stachniss, University of Bonn
#include "mesh.h"
#include <algorithm>

namespace DynaMap {
namespace geometry {

void Mesh::Init(unsigned int max_triangles) {
  num_triangles_ = 0;
  max_triangles_ = max_triangles;
  cudaMallocManaged(&triangles_, sizeof(Triangle) * max_triangles_);
  cudaDeviceSynchronize();
}

void Mesh::Free() {
  cudaDeviceSynchronize();
  cudaFree(triangles_);
}

__device__ void Mesh::AppendTriangle(Triangle t) {
  unsigned int idx = atomicAdd(&num_triangles_, 1);
  if (num_triangles_ <= max_triangles_) triangles_[idx] = t;
}

__device__ Vertex Mesh::getVertices(unsigned int id){
  Vertex result;
  switch(id%3){
    case 0:{
      result = triangles_[id/3].v0;
    }case 1:{
      result = triangles_[id/3].v1;
    }case 2:{
      result = triangles_[id/3].v2;
    }
  }
  return result;
}

int Mesh::SaveToFile(const std::string &filename) {
  std::ofstream fout(filename.c_str(), std::ofstream::out);
  int n = std::min(num_triangles_, max_triangles_);
  if (n == max_triangles_) {
    std::cout << "Triangles limit reached!" << std::endl;
  }
  for (unsigned int i = 0; i < n; i++) {

    // std::cout << "v " << triangles_[i].v0.position.x << " "
    // << triangles_[i].v0.position.y << " " << triangles_[i].v0.position.z
    // << " " << triangles_[i].v0.color.x << " " << triangles_[i].v0.color.y
    // << " " << triangles_[i].v0.color.z << std::endl;

    fout << "v " << triangles_[i].v0.position.x << " "
         << triangles_[i].v0.position.y << " " << triangles_[i].v0.position.z
         << " " << triangles_[i].v0.color.x << " " << triangles_[i].v0.color.y
         << " " << triangles_[i].v0.color.z << std::endl;
    fout << "v " << triangles_[i].v1.position.x << " "
         << triangles_[i].v1.position.y << " " << triangles_[i].v1.position.z
         << " " << triangles_[i].v1.color.x << " " << triangles_[i].v1.color.y
         << " " << triangles_[i].v1.color.z << std::endl;
    fout << "v " << triangles_[i].v2.position.x << " "
         << triangles_[i].v2.position.y << " " << triangles_[i].v2.position.z
         << " " << triangles_[i].v2.color.x << " " << triangles_[i].v2.color.y
         << " " << triangles_[i].v2.color.z << std::endl;
  }
  for (unsigned int i = 1; i <= n * 3; i += 3) {
    fout << "f " << i << " " << i + 1 << " " << i + 2 << std::endl;
  }
  fout.close();

  return num_triangles_;
}

}  // namespace geometry

}  // namespace DynaMap
