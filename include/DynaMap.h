/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
#pragma once
// Rigid Registration Solver 
// #define GPU_SOLVER

// SOLVER_TYPE 0: Eigen Solver, 1: cuSparceSolver, 2: cuSolver 
#define SOLVER_TYPE 1

// #define DEBUG  // writing matrixes in files in /log
// #define DEBUG_DefGraph // saving graph structure in file /log
// #define LOG_EN  // stream out steps in termnial
#define LOG_MAX 100

// KDtree dimension
#define KNN 		4
#define KDTREE_MAX_DIM 	3

#define expWeight 	true
#define NODE_NUM    273
#define dgw 		0.015f
#define EPSILLON  	1e-6
#define LAMBDA    	0.0f   // Regularization term wight
// 8 3 true 273 0.015f 1e-5 0.01f
// test 300, 16, 0.5, LAMBDA 0.2 

#define IDX2C(i,j,ld) (((j)*(ld))+(i))
#define IDX2F(i,j,ld) ((((j)-1)*(ld))+((i)-1))

#define SIZE_OF(x) sizeof(x)/sizeof(x[0])