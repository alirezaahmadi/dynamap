/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
#pragma once

#include <cuda.h>
#include <cuda_runtime.h>

#include "DynaMap.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

namespace DynaMap{

	struct kdNode{
		int id;
		float x[KDTREE_MAX_DIM];
		struct kdNode *left, *right;
		float distance;
	};

	class kdTree{
	  public:
		__device__ __host__
		kdTree();
		__device__ __host__
		virtual ~kdTree();
		void init(int querySize);
		void Free(void);
		__device__ __host__
		inline float dist(struct kdNode *a, struct kdNode *b, int dim);
		inline void swap(struct kdNode *x, struct kdNode *y);
		struct kdNode* findMedian(struct kdNode *start, struct kdNode *end, int idx);
		struct kdNode* buildTree(struct kdNode *t, int len, int i, int dim);
		__device__ __host__
		void findNearest(struct kdNode *root, 
						 struct kdNode *nd, 
						 int i, 
						 int dim,
						 struct kdNode **best, 
						 float *best_dist);
		__device__ 	__host__
		void findKNearest(struct kdNode *root, 
						  struct kdNode *nd, 
						  int i, 
						  int dim,
						  struct kdNode **best, 
						  float *best_dist,
						  struct kdNode *VisitedNodes);
		__device__ __host__
		void findKNN(struct kdNode &targetNode);
		__device__ __host__
		inline void sortNodes(int visitedNum);

		int visited;
		float *kdDistnaces;
		struct kdNode *kdRoot, *kdQuery, *kdFound, *VisitedNodes;
	};		
}   // namespace DynaMap















