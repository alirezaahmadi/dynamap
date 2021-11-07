/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

#pragma once

#include <cuda.h>
#include <cublas.h>
#include <cuda_runtime.h>

#include <algorithm>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

#include "utils/utils.h"
#include "geometry/defGraph/defGraph.h"

#define BLOCK_DIM 16

namespace DynaMap{

    class knn{
        public:

        virtual ~knn();

        void init(geometry::DefGraph& graph);
        void Free(void);
        
        bool cuBlasKNN(float *       knn_dist,
                       int *         knn_index);


        // Parameters
        unsigned int refVerticesNum;
        unsigned int queryVerticesNum;
        unsigned int dim;
        unsigned int k;

        geometry::DefGraph& defGraph;

        geometry::PointXYZ* refVertices;
        geometry::PointXYZ* queryVertices;
        float* distances;
        unsigned int* indices;
        geometry::PointXYZ* refNorms;
        geometry::PointXYZ* queryNorms;
        

    }; 
}   //end namespace DynaMap