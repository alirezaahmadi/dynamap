/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
#pragma once

// 0.1 50  0.75 bigplane + random node initialization

// Eigen still uses the __CUDACC_VER__ macro,
// which is deprecated in CUDA 9.
// #if __CUDACC_VER_MAJOR__ >= 9
// #undef __CUDACC_VER__
// #define __CUDACC_VER__ \
//   ((__CUDACC_VER_MAJOR__ * 10000) + (__CUDACC_VER_MINOR__ * 100))
// #endif

#include <cuda.h>
#include <cuda_runtime.h>

// #include "cuSparseUtils.h"

#include <curand.h>
#include <cusparse.h>
#include <cublas_v2.h>
#include <cusolverDn.h>
#include <cusolverSp.h>

#include <stdlib.h>
#include <assert.h>
#include <time.h>
#include <stdio.h>
#include <sstream>
#include <cmath>
#include <iomanip>
#include <string>
#include <iostream>
#include <fstream>
#include <ctime>

#include "DynaMap.h"
#include "gnSolver.h"
#include "defGraph.h"
#include "math_types.h"
#include "mesh/meshSTD.h"
#include "utils/utils_String.h"
#include "blender/dqBlender/dqBlender.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Cholesky>

typedef float Jacobian;

namespace DynaMap{
namespace solver{

    class nonRigidICP : public gnSolver {
        public:

        nonRigidICP(void);
        virtual ~nonRigidICP(void);
        
        void init(geometry::defGraph &warpField, 
                  Properties &_Prop,
                  int problemSize);
        void Free();

        __device__
        math::dualQuat getPDQGradient(math::dualQuat& dq, int paramID);
        __device__
        math::dualQuat getNDQGradient(math::dualQuat& dq, int paramID);

        __device__
        float getDQGradient(math::dualQuat *subWarpField,
                            float* subWarpFiledWeights,
                            int* subWarpFiledIDs,
                            blender::dqBlender &blender,
                            int nodeIndex,
                            int ObsID,
                            int paramID,
                            float3 normalAtSrc,
                            float3 vertexPose,
                            float residual);
        __device__
        void validate(float& variable);
        __device__
        void updateDataJacobianBlock(Jacobian *_Jacob, 
                                     math::dualQuat *subWarpField,
                                     float* subWarpFiledWeights,
                                     int* subWarpFiledIDs,
                                     blender::dqBlender &blender,
                                     int ObsNum,
                                     int ObsID,
                                     int nodeID,
                                     int nodeIndex,
                                     float3 srcVertexPose,
                                     float3 srcVertexNormal,
                                     float3 dstVertexPose,
                                     float residual);
                                     
        __device__             
        void updateRegJacobianBlock(Jacobian *_Jacob, 
                                    geometry::defGraph &warpField,
                                    int nodeID);
        
        __device__
        float3 getRegResiduals(geometry::defGraph &warpField,                             
                               int nodeID);

        void solve(geometry::defGraph &warpField,
                   blender::dqBlender &blender,
                   pyramid &targetImage,
                   pyramid &sourceImage,
                   Eigen::Matrix4f pose, 
                   rgbdSensor sensor);

        void solve(geometry::defGraph &warpField,
                   blender::dqBlender &blender,
                   geometry::MeshSTD &targetMesh,
                   Eigen::Matrix4f pose);

        bool evaluateError(geometry::defGraph &warpField,
                           blender::dqBlender &blender);
        
        void buildLinearSystem(geometry::defGraph &warpField);
        
        void solveLinearSystem(geometry::defGraph &warpField);
        void cuSolveLinearSystem(geometry::defGraph &warpField);
        void solveSparceLinearSystem(geometry::defGraph &warpField);
        
        void cuBlasMatrixMul(const float *A, const float *B, float *C, const int m, const int k, const int n);
        void cuBlasMatrixMulTrans(const float *A, const float *B, float *C, const int m, const int k, const int n);
        void cuBlasMatrixSum(const float alf, const float *A, const float bet, const float *B, float *C, const int m);
        void cuBlasMatrixSumTrans(const float alf, const float *A, const float bet, const float *B, float *C, const int m);
        void cuBlasVectorSum(const float alf, const float *A, const float bet, const float *B, float *C, const int m);
        void cuBlasMatrixTrans(const float *A, float *C, const int m, const int n);

        void writeMatrixToTxt(const char* fileName, 
                          float *matrix,
                          size_t size,
                          int rows, int cols,
                          int mode,
                          int from,
                          int to);
        
        Properties  Prop;
        int iterationNum;

        float alpha;
        float beta;
        int n,m,k;

        size_t Mdata;

        cublasHandle_t cuBlasHandle;
        cublasStatus_t cuBlasStatus;

        cusolverDnHandle_t cuSolverHandle;
        cusolverStatus_t cuSolverStatus;

        Jacobian *JTJ;

        geometry::PointCloudXYZ *cloud;

        float* rData;
        float* rReg;

        float* result;
        float* b;
        float* bData;
        float* bReg;

        float prevError, currError;
        
        Jacobian *JdTJd;
        Jacobian *dataJacob;

        Jacobian *JrTJr;
        Jacobian *regJacob;

    }; 
}   //end namespace solver
}   //end namespace DynaMap