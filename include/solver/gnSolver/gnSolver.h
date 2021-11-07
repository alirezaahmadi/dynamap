/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
#pragma once

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

#include "DynaMap.h"
#include "tsdf/tsdf.h"
#include "utils/utils.h"
#include "pyramid/pyramid.h"
#include "math/math_types.h"
#include "math/utils_matrix.h"
#include "sensor/rgbdSensor.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Cholesky>
#include <eigen3/unsupported/Eigen/MatrixFunctions>

namespace DynaMap{

namespace solver{

    typedef Eigen::Matrix<double, 6, 1> Vector6d;

    struct Properties {
        int lvlIterationNum[3];
        int lvlScale[3];
        int maxIteration;
        float minIncrement;
        float maxIncrement;
        float regularization;
        float huberConstant;
    };

    class gnSolver{
        public:

        void init(Properties  _OptProp, 
                  const rgbdSensor &sensor);
        void Free();
        __host__ __device__
        float getHuberWeight(float error, float thershold);
        __host__ __device__
        float getTukeyWeight(float error, float thershold);
        __host__ __device__
        float getL1Weight(float error);

        Vector6d solveSparceLinearSystem(Eigen::Matrix<double, 6, 6>& _A, Vector6d& _b);

        void solve(Eigen::Matrix4f& pose, Vector6d& prev_increment_, 
                                rgbdImage &targetImage,
                                rgbdImage &sourceImage,
                                geometry::PointCloudXYZ &targetCloud,
                                geometry::PointCloudXYZ &sourceCloud);

        Eigen::Matrix<double, 6, 1> SolveLdlt(const Eigen::Matrix<double, 6, 6> &H,
                                                    const Eigen::Matrix<double, 6, 1> &b);

        Eigen::Matrix4d Exp(const Eigen::Matrix4d &mat);

        Eigen::Matrix4d v2t(const Vector6d &xi);
        void printSolverPros(void);

        __host__ __device__ 
        float ColorDifference(uchar3 c1, uchar3 c2);
        __host__ __device__ 
        float Intensity(float3 color);

        void cuBlasVectorSum(const float alf, const float *A, const float bet, const float *B, float *C, const int m);

        Properties  OptProp;
        int iterationNum;

        pyramid tragetPyImage;
        pyramid sourcePyImage;

        cudaError_t cudaStatus;
    }; 
}   //end namespace Solver
}   //end namespace DynaMap