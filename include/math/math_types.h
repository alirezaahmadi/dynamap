/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
#pragma once

#include <cuda.h>
#include <cuda_runtime.h>

#include <stdio.h>
#include <iostream>
#include "math/utils_matrix.h"


namespace DynaMap{
namespace math{

struct EulerAngles{
    float roll, pitch, yaw;
    __host__ __device__
    EulerAngles(){roll = 0.0f; pitch = 0.0f; yaw = 0.0f;}
    __host__ __device__
    EulerAngles(float _roll, float _pitch, float _yaw): 
                roll(_roll), pitch(_pitch), yaw(_yaw){}
    __host__ __device__
    EulerAngles(float3 euAngles):roll(euAngles.x), pitch(euAngles.y), yaw(euAngles.z){}

    __host__ __device__
    EulerAngles transformationToEulerAngles(float4x4 T){
        EulerAngles res;
        res.roll = atan2(T.m32, T.m33);
        res.pitch = atan2(-T.m31, (sqrt(pow(T.m32,2) + pow(T.m33,2))));
        res.yaw = atan2(T.m21, T.m11);
        return res;
    }
    __host__ __device__
    float3 toFloat3(void){
        return make_float3(roll, pitch, yaw);
    }

    friend std::ostream& operator<<(std::ostream& os, EulerAngles& q){
        os << "roll: "<< q.roll << ", pitch: "<< q.pitch << ", yaw: " << q.yaw;
        return os;
    }
};

struct AxisAngle{
    float theta;
    float3 n;

    friend std::ostream& operator<<(std::ostream& os, AxisAngle& q){
        os << "theta: "<< q.theta << ", x: "<< q.n.x << ", y: " << q.n.y << ", z: " << q.n.z;
        return os;
    }
};

}
}