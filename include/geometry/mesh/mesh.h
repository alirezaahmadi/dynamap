// Copyright 2019 Emanuele Palazzolo (emanuele.palazzolo@uni-bonn.de), Cyrill Stachniss, University of Bonn
#pragma once
#include <cuda_runtime.h>

#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include "geom_Types.h"
#include "rgbdSensor.h"
#include "utils_matrix.h"

#include "utils/utils_String.h"

//Eigen
#include <eigen3/Eigen/Dense>

namespace DynaMap {

namespace geometry {

/**
 * @brief      Class representing a mesh.
 */
class Mesh {
 public:
  /**
   * @brief      Initializes the class.
   *
   * @param[in]  max_triangles  The maximum number of triangles in the mesh
   */
  void Init(unsigned int max_triangles);

  /**
   * @brief      Frees the memory allocated for the class.
   */
  void Free();

  /**
   * @brief      Appends a triangle to the mesh.
   *
   * @param[in]  t     The triangle
   */
  __device__ void AppendTriangle(Triangle t);

  __device__ Vertex getVertices(unsigned int id);
  /**
   * @brief      Saves the mesh to an obj file.
   *
   * @param[in]  filename  The filename
   */
  int SaveToFile(const std::string &filename);

  /** The triangles that compose the mesh */
  Triangle *triangles_;

  /** The number of triangles that compose the mesh */
  unsigned int num_triangles_;

  /** The maximum number of triangles that the mesh can have */
  unsigned int max_triangles_;

  unsigned int verticesNum;

  unsigned int maxVerticesNum;
};

}  // namespace tsdfvh

}  // namespace refusion
