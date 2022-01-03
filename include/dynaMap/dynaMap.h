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

// Inex convertors 
#define IDX2C(i,j,ld) (((j)*(ld))+(i))
#define IDX2F(i,j,ld) ((((j)-1)*(ld))+((i)-1))

// size of variabels
#define SIZE_OF(x) sizeof(x)/sizeof(x[0])


