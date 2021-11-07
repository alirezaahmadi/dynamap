/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
#pragma once

#include <cmath>
#include <cuda.h>
#include <cuda_runtime.h>
#include <iostream>
#include "math/Quaternion.h"

namespace DynaMap{
namespace math{

class dualQuat {
    public:
    Quaternion real;
    Quaternion dual;

    __host__ __device__
    inline dualQuat(void){
        real = Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
        dual = Quaternion(0.0f, 0.0f, 0.0f, 0.0f);
    }
    __host__ __device__
    inline dualQuat(Quaternion r, Quaternion d): real(r), dual(d){}
    
    __host__ __device__
    inline dualQuat(Quaternion r, float3 t){   // todo ... check the equation
        r = r.normalize();
        float w  = -0.5f*( t.x * r.x + t.y * r.y + t.z * r.z);
        float xi =  0.5f*( t.x * r.w + t.y * r.z - t.z * r.y);
        float yj =  0.5f*(-t.x * r.z + t.y * r.w + t.z * r.x);
        float zk =  0.5f*( t.x * r.y - t.y * r.x + t.z * r.w);
        // dualQuat(r, Quaternion(w, xi, yj, zk));
        real = r;
        dual = Quaternion(w, xi, yj, zk);
        //  ( new Quaternion( 0, t ) * real ) * 0.5f;
    }
    __host__ __device__
    inline dualQuat(float4x4& t){
        dualQuat dq(Quaternion(t.getFloat3x3()), make_float3(t.m14, t.m24, t.m34));
        real = dq.real;
        dual = dq.dual;
    }
    __host__ __device__
    inline dualQuat(EulerAngles euAngles, float3 t){
        Quaternion Quat(euAngles);
        real = Quat;
        dual = Quaternion(0, t.x, t.y, t.z);
    }
    __host__ __device__
    inline static dualQuat identity(void){
        return dualQuat(Quaternion(1.0f, 0.0f, 0.0f, 0.0f), Quaternion(0.0f, 0.0f, 0.0f, 0.0f));
    }
    __host__ __device__
    inline void setIdentity(void){
        real = Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
        dual = Quaternion(0.0f, 0.0f, 0.0f, 0.0f);
    }
    /****************************************************************/
    /**************************Products******************************/
    /****************************************************************/
    __host__ __device__
    inline static dualQuat normalize(dualQuat q){
        float norm = q.real.norm();
        dualQuat dq;
        dq.real = q.real / norm;
        dq.dual = q.dual / norm;
        return dq;
    }
    __host__ __device__
    inline dualQuat normalize(void){
        float norm = real.norm();
        real /= norm;
        dual /= norm;
        return *this;
    }
    __host__ __device__
    inline static float dot(dualQuat a, dualQuat b){
        return Quaternion::dot(a.real, b.real);
    }
    __host__ __device__
    inline float dot(dualQuat b){
        return Quaternion::dot(real, b.real);
    }
    __host__ __device__ 
    inline static dualQuat mul(dualQuat a, dualQuat b){
        return dualQuat(a.real * b.real, (a.real* b.dual) + (b.real * a.dual));
    }
    __host__ __device__ 
    inline dualQuat mul(dualQuat b){
        return dualQuat(real * b.real, (real* b.dual) + (b.real * dual));
    }
    __host__ __device__ 
    inline static dualQuat conjugate(dualQuat q){
        return dualQuat(Quaternion::conjugate(q.real), Quaternion::conjugate(q.dual));
    }
    __host__ __device__ 
    inline dualQuat conjugate(void){
        return dualQuat(Quaternion::conjugate(real), Quaternion::conjugate(dual));
    }
    __host__ __device__ 
    inline static dualQuat inverse(dualQuat q){
        assert(q.real != 0.0f);
        return dualQuat(q.real.inverse() ,  (q.real.inverse() - (q.real.inverse() * q.dual * q.real.inverse()))); 
    }
    __host__ __device__ 
    inline dualQuat inverse(void){
        assert(real != 0.0f);
        return dualQuat(real.inverse() , (real.inverse() - (real.inverse() * dual * real.inverse())));
    } 
    /****************************************************************/
    /**************************Operators*****************************/
    /****************************************************************/
    __host__ __device__
    inline dualQuat operator / (dualQuat q){
        Quaternion denom = (q.real * q.real);
        Quaternion _real = (real * q.real)/denom;
        Quaternion _dual = ((q.real * dual)-(real * q.dual))/denom;
        return dualQuat(_real, _dual);
    }
    __host__ __device__
    inline void operator /= (dualQuat q){
        Quaternion denom = (q.real * q.real);
        Quaternion _real = (real * q.real)/denom;
        Quaternion _dual = ((q.real * dual)-(real * q.dual))/denom;
        *this = dualQuat(_real, _dual);
    }
    __host__ __device__
    inline dualQuat operator * (float scale){
        return dualQuat(real * scale, dual * scale);
    }
    inline void operator *= (float scale){
        *this = dualQuat(real * scale, dual * scale);
    }
    __host__ __device__ 
    inline dualQuat operator * (dualQuat q){
        return dualQuat(real * q.real, (real* q.dual) + (q.real * dual));
    }
    __host__ __device__ 
    inline void operator *= (dualQuat q){
        dualQuat dq(real * q.real, (real* q.dual) + (q.real * dual));
        *this = dq;
    }
    __host__ __device__ 
    inline  dualQuat operator + (dualQuat q){
        return dualQuat(real + q.real, dual + q.dual);
    }
    __host__ __device__ 
    inline  void operator += (dualQuat q){
        *this = dualQuat(real + q.real, dual + q.dual);
    }
    __host__ __device__ 
    inline  dualQuat operator = (const dualQuat q){
        this->real = q.real; 
        this->dual = q.dual;
        return *this;
    }
    __host__ __device__ 
    inline  bool operator == (dualQuat q){
        bool res = (real == q.real && dual == q.dual) ? true : false;
        return res;
    }
    __host__ __device__ 
    inline  bool operator != (dualQuat q){
        bool res = (real != q.real || dual != q.dual) ? true : false;
        return res;
    }
    friend std::ostream& operator<<(std::ostream& os, dualQuat& q){
        os <<  "Real: " << q.real << std::endl;
        os <<  "Dual: " << q.dual ;
        return os;
    }
    
    /****************************************************************/
    /**************************Applications**************************/
    /****************************************************************/

    inline float3 rotate(float3 v){
        Quaternion q = real;
        q.normalize();
        return q.rotate(v);
    }
    __host__ __device__ 
    inline static Quaternion getRotation(dualQuat q){
        return q.real;
    }
    __host__ __device__ 
    inline Quaternion getRotation(void){
        return real;
    }
    __host__ __device__
    inline static float3 getTranslation(dualQuat q){
        Quaternion translation =  (q.dual * q.real.conjugate()) * 2.0f;
        return translation.getVectorPart();
    }
    __host__ __device__
    inline float3 getTranslation(void){
        Quaternion translation = (dual * real.conjugate()) * 2.0f;
        return translation.getVectorPart();
    }
    __host__ __device__
    inline static dualQuat addTranslation(dualQuat q, float3 t){
        return dualQuat(q.dual, t + q.getTranslation());
    }
    __host__ __device__
    inline void addTranslation(float3 t){
        dualQuat dq(real, t + this->getTranslation());
        *this = dq;
    }
    __host__ __device__
    inline static dualQuat addRotation(dualQuat q, math::EulerAngles euAngles){
        Quaternion q_ = Quaternion(euAngles);
        q_.normalize();
        return dualQuat(q.real + q_, q.dual);
    }
    __host__ __device__
    inline void addRotation(math::EulerAngles euAngles){
       Quaternion q = Quaternion(euAngles);
        q.normalize();
        real += q;
    }
    __host__ __device__ 
    inline float4x4 getTransformation(void){
        dualQuat q = *this;
        q.normalize();
        float3x4 M;
        float w = q.real.w;
        float x = q.real.x;
        float y = q.real.y;
        float z = q.real.z; 
        // Extract rotational information
        M.m11 = powf(w, 2.0f) + powf(x, 2.0f) - powf(y, 2.0f) - powf(z, 2.0f);
        M.m12 = 2.0f * x * y + 2.0f * w * z;
        M.m13 = 2.0f * x * z - 2.0f * w * y;

        M.m21 = 2.0f * x * y - 2.0f * w * z;
        M.m22 = powf(w, 2.0f) + powf(y, 2.0f) - powf(x, 2.0f) - powf(z, 2.0f);
        M.m23 = 2.0f * y * z + 2.0f * w * x;

        M.m31 = 2.0f * x * z + 2.0f * w * y;
        M.m32 = 2.0f * y * z - 2.0f * w * x;
        M.m33 = powf(w, 2.0f) + powf(z, 2.0f) - powf(x, 2.0f) - powf(y, 2.0f);
        
        // Extract translation information
        Quaternion t = (q.dual * 2.0f) * Quaternion::conjugate( q.real);
        M.setTranslation(t.getVectorPart());
        return M;
    }
     __host__ __device__ 
    inline static float4x4 getTransformation(dualQuat q){
        q.normalize();
        float3x4 M;
        float w = q.real.w;
        float x = q.real.x;
        float y = q.real.y;
        float z = q.real.z; 
        // Extract rotational information
        M.m11 = powf(w, 2.0f) + powf(x, 2.0f) - powf(y, 2.0f) - powf(z, 2.0f);
        M.m12 = 2.0f * x * y + 2.0f * w * z;
        M.m13 = 2.0f * x * z - 2.0f * w * y;

        M.m21 = 2.0f * x * y - 2.0f * w * z;
        M.m22 = powf(w, 2.0f) + powf(y, 2.0f) - powf(x, 2.0f) - powf(z, 2.0f);
        M.m23 = 2.0f * y * z + 2.0f * w * x;

        M.m31 = 2.0f * x * z + 2.0f * w * y;
        M.m32 = 2.0f * y * z - 2.0f * w * x;
        M.m33 = powf(w, 2.0f) + powf(z, 2.0f) - powf(x, 2.0f) - powf(y, 2.0f);
        
        // Extract translation information
        Quaternion t = (q.dual * 2.0f) * Quaternion::conjugate( q.real);
        M.setTranslation(t.getVectorPart());
        return M;
    }
    __host__ __device__ 
    inline static float3 transformPosition(float3 point, dualQuat q ){
        float norm = q.real.norm();
        Quaternion blendReal = q.real / norm;
        Quaternion blendDual = q.dual / norm;

        float3 vecReal = Quaternion::getVectorPart(blendReal);
        float3 vecDual = Quaternion::getVectorPart(blendDual);
        float3 tranlation = ((vecDual * blendReal.w - vecReal * blendDual.w) + cross(vecReal, vecDual)) * 2.0f;
        return blendReal.rotate(point) + tranlation;

    }
    __host__ __device__ 
    inline float3 transformPosition(float3 point){
        float norm = real.norm();
        Quaternion blendReal(real / norm);
        Quaternion blendDual(dual / norm);
        float3 vecReal = Quaternion::getVectorPart(blendReal);
        float3 vecDual = Quaternion::getVectorPart(blendDual);
        float3 tranlation = ((vecDual * blendReal.w - vecReal * blendDual.w) + cross(vecReal, vecDual)) * 2.0f;
        return blendReal.rotate(point) + tranlation;

    }
    __host__ __device__ 
    inline static float3 transformNormal( float3 normal, dualQuat q ){
        float norm = q.real.norm();
        Quaternion blendReal = q.real / norm;
        Quaternion blendDual = q.dual / norm;

        float3 vecReal = Quaternion::getVectorPart(blendReal);
        float3 vecDual = Quaternion::getVectorPart(blendDual);
        float3 tranlation = ((vecDual * blendReal.w - vecReal * blendDual.w) + cross(vecReal, vecDual)) * 2.0f;
        return (blendReal.rotate(normal)) + tranlation;
    }
    __host__ __device__ 
    inline float3 transformNormal(float3 normal){
        float norm = real.norm();
        Quaternion blendReal = real / norm;
        Quaternion blendDual = dual / norm;

        float3 vecReal = Quaternion::getVectorPart(blendReal);
        float3 vecDual = Quaternion::getVectorPart(blendDual);
        float3 tranlation = ((vecDual * blendReal.w - vecReal * blendDual.w) + cross(vecReal, vecDual)) * 2.0f;
        return (blendReal.rotate(normal)) + tranlation;
    }
     // get euler angles from quaternion
    __host__ __device__
    inline math::EulerAngles getEulerAngles(void){
        math::EulerAngles angles;

        // roll (x-axis rotation)
        float sinr_cosp = +2.0f * (real.w * real.x + real.y * real.z);
        float cosr_cosp = +1.0f - 2.0f * (real.x * real.x + real.y * real.y);
        angles.roll = atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        float sinp = +2.0f * (real.w * real.y - real.z * real.x);
        if (fabs(sinp) >= 1.0f)
            angles.pitch = copysign(M_PI / 2, sinp); // use 90 degrees if res of range
        else
            angles.pitch = asin(sinp);

        // yaw (z-axis rotation)
        float siny_cosp = +2.0f * (real.w * real.z + real.x * real.y);
        float cosy_cosp = +1.0f - 2.0f * (real.y * real.y + real.z * real.z);  
        angles.yaw = atan2(siny_cosp, cosy_cosp);

        return angles;
    }
     // get euler angles from quaternion
    __host__ __device__
    inline static EulerAngles getEulerAngles(const dualQuat q){
        EulerAngles angles;

        // roll (x-axis rotation)
        float sinr_cosp = +2.0f * (q.real.w * q.real.x + q.real.y * q.real.z);
        float cosr_cosp = +1.0f - 2.0f * (q.real.x * q.real.x + q.real.y * q.real.y);
        angles.roll = atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        float sinp = +2.0f * (q.real.w * q.real.y - q.real.z * q.real.x);
        if (fabs(sinp) >= 1.0f)
            angles.pitch = copysign(M_PI / 2.0f, sinp); // use 90 degrees if res of range
        else
            angles.pitch = asin(sinp);

        // yaw (z-axis rotation)
        float siny_cosp = +2.0f * (q.real.w * q.real.z + q.real.x * q.real.y);
        float cosy_cosp = +1.0f - 2.0f * (q.real.y * q.real.y + q.real.z * q.real.z);  
        angles.yaw = atan2(siny_cosp, cosy_cosp);

        return angles;
    }
    __host__ __device__
    inline AxisAngle getAxisAngle(const dualQuat q){
        this->normalize();
        AxisAngle aa;
        aa.theta = 2.0f * acos(q.real.w);
        float _sin = sin(aa.theta/2.0f);
        aa.n = make_float3(q.real.x/_sin, q.real.y/_sin, q.real.z/_sin);
        return aa;
    }
};

}   // namespace math
}   // namespace DynaMap