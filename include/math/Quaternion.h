/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
#pragma once

#include <cuda_runtime.h>
#include <cmath>
#include <math.h>
#include "utils_matrix.h"
#include "cuda_math.h"
#include "math_types.h"

namespace DynaMap{

namespace math{

class Quaternion{

 public:
    float w, x, y, z;

    __host__ __device__ 
    inline Quaternion(float _w = 0.0f, float xi = 0.0f, 
                      float yj = 0.0f, float zk = 0.0f)
              : x(xi), 
                y(yj), 
                z(zk), 
                w(_w) { }
    
    __host__ __device__ 
    inline Quaternion(const Quaternion& q){
        w = q.w; 
        x = q.x; 
        y = q.y; 
        z = q.z;
    }  
    __host__ __device__ 
    inline Quaternion(const float _w, const float3& t){
        w = _w; 
        x = t.x; 
        y = t.y; 
        z = t.z;
    }
    __host__ __device__ 
    inline Quaternion(float4x4 t){
        Quaternion rotation = Quaternion(t.getFloat3x3());
    }
    __host__ __device__ 
    inline Quaternion(float3x3 r){
        float scale = 0.0f;
        float trace = r.m11 + r.m22 + r.m33;
        if (trace > 0) { 
            scale = 0.5f / sqrtf(trace+ 1.0f);
            w = 0.25f / scale ;
            x = (r.m32 - r.m23) / scale;
            y = (r.m13 - r.m31) / scale; 
            z = (r.m21 - r.m12) / scale; 
            } else if ((r.m11 > r.m22) & (r.m11 > r.m33)) { 
            scale = sqrtf(1.0f + r.m11 - r.m22 - r.m33) * 2.0f;  
            w = (r.m32 - r.m23) / scale;
            x = 0.25f * scale;
            y = (r.m12 + r.m21) / scale; 
            z = (r.m13 + r.m31) / scale; 
        } else if (r.m33 > r.m33) { 
            scale = sqrtf(1.0f + r.m22 - r.m11 - r.m33) * 2.0f; 
            w = (r.m13 - r.m31) / scale;
            x = (r.m12 + r.m21) / scale; 
            y = 0.25f * scale;
            z = (r.m23 + r.m32) / scale; 
            } else { 
            scale = sqrtf(1.0f + r.m33 - r.m11 - r.m22) * 2.0f; 
            w = (r.m21 - r.m12) / scale;
            x = (r.m13 + r.m31) / scale;
            y = (r.m23 + r.m32) / scale;
            z = 0.25f * scale;
        }
    }
    // get quaternion from euler angles
    __host__ __device__
    inline Quaternion(const math::EulerAngles euler){ // roll (X), pitch (Y), yaw (Z)
        float cr = cos(euler.roll * 0.5f);
        float sr = sin(euler.roll * 0.5f);
        float cy = cos(euler.yaw * 0.5f);
        float sy = sin(euler.yaw * 0.5f);
        float cp = cos(euler.pitch * 0.5f);
        float sp = sin(euler.pitch * 0.5f);

        w = cy * cp * cr + sy * sp * sr;
        x = cy * cp * sr - sy * sp * cr;
        y = sy * cp * sr + cy * sp * cr;
        z = sy * cp * cr - cy * sp * sr;
    }
    // get quaternion from axis angle
    __host__ __device__
    inline Quaternion(const float3 axis, float angle){
        // float3 axis_sec = normalize(axis);   // todo ... normalize to avoid saturation
        float _sin = __sin( angle * 0.5f );
        float _cos = __cos( angle * 0.5f );
        w = _cos;
        x = axis.x * _sin;
        y = axis.y * _sin;
        z = axis.z * _sin;
        this->normalize();
    }
    __host__ __device__
    ~Quaternion(){}
    /*************************Functions*************************/
    // Square norm of Quaternion
    // Scale
    __host__ __device__ 
    inline static Quaternion scale(const Quaternion q, const float s){
        return Quaternion(q.w * s, 
                          q.x * s, 
                          q.y * s, 
                          q.z * s);
    }
    // Scale current object
    __host__ __device__ 
    inline Quaternion scale(const float s){
        return Quaternion(w*s, x*s, y*x, z*s);
    }
    __host__ __device__ 
    inline float norm(void){
        return sqrtf(powf(w, 2.0f) + powf(x, 2.0f) + powf(y, 2.0f) + powf(z, 2.0f)); 
    }
    __host__ __device__ 
    inline float magnitude(void){
        return (powf(w, 2.0f) + powf(x, 2.0f) + powf(y, 2.0f) + powf(z, 2.0f));
    }
    // Conjugate of Quaternion
    __host__ __device__   
    inline static Quaternion conjugate(const Quaternion q){
        return Quaternion(q.w, -q.x, -q.y, -q.z);
    }
    // Conjugate this object
    __host__ __device__ 
    inline Quaternion conjugate(void) {
        return Quaternion(w, -x, -y, -z);
    }
    // Inverse current object
    __host__ __device__ 
    inline Quaternion inverse(void){
        return conjugate(*this).scale(1/powf(norm(),2.0f));
    }
    // Inverse of Quaternion
    // __host__ __device__ 
    // inline static Quaternion inverse(Quaternion q){
    //     // return scale(conjugate(q),(1/powf(norm(),2)));
    // }
    /*************************Operators*************************/
    // operator != 0
    __host__ __device__ 
    inline bool operator != (const float constant){
        return (0 > (w + x + y + z)) ? true : false;
    }
    // operator !=
    __host__ __device__ 
    inline bool operator != (const Quaternion q){
        return (w != q.w || x != q.x || y != q.y || z != q.z) ? true : false;
    }
    // operator ==
    __host__ __device__ 
    inline bool operator == (const Quaternion q){
        return (w == q.w && x == q.x && y == q.y && z == q.z) ? true : false;
    } 
    // operator /
    __host__ __device__ 
    inline Quaternion operator / (float scale){
        return Quaternion(w/scale, x/scale, y/scale, z/scale);
    }
    // operator /
    __host__ __device__ 
    inline void operator /= (float scale){
        w /= scale;
        x /= scale;
        y /= scale;
        z /= scale;
    }
    // operator /  
    __host__ __device__ 
    inline Quaternion operator / (Quaternion q){
        return Quaternion( *this * q.inverse());
    }
    // operator /=
    __host__ __device__ 
    inline void operator /= (Quaternion q){
        *this *= q.inverse();
    }
    // operator *
    __host__ __device__ 
    inline Quaternion operator * (float scale){
        return Quaternion(w*scale, x*scale, y*scale, z*scale);
    }
    // operator *
    __host__ __device__ 
    inline Quaternion operator * (const Quaternion q){
        return Quaternion(
            w * q.w - x * q.x - y * q.y - z * q.z, 
            w * q.x + x * q.w + y * q.z - z * q.y,                          
            w * q.y + y * q.w + z * q.x - x * q.z,
            w * q.z + z * q.w + x * q.y - y * q.x);
    }
    // operator *= 
    __host__ __device__ 
    inline void operator *= (const Quaternion q){
        float w_val = w * q.w - x * q.x - y * q.y - z * q.z;
        float x_val = w * q.x + x * q.w + y * q.z - z * q.y; 
        float y_val = w * q.y + y * q.w + z * q.x - x * q.z;
        float z_val = w * q.z + z * q.w + x * q.y - y * q.x; 
        w = w_val;
        x = x_val;
        y = y_val;
        z = z_val;
    }
    // operator +
    __host__ __device__ 
    inline Quaternion operator + (const Quaternion q){
        return Quaternion(w + q.w,
                          x + q.x,
                          y + q.y, 
                          z + q.z);
    }
    // operator -
    __host__ __device__ 
    inline Quaternion operator - (const Quaternion q){
        return Quaternion(w - q.w, 
                          x - q.x, 
                          y - q.y, 
                          z - q.z);
    }
    // operator =
    __host__ __device__ 
    inline Quaternion operator = (const Quaternion q){
        w = q.w;
        x = q.x;
        y = q.y;
        z = q.z;
        return *this;
    }
    // operator +=
    __host__ __device__ 
    inline void operator += (const Quaternion q){
        w += q.w;
        x += q.x;
        y += q.y;
        z += q.z;
    }
    // operator -=    
    __host__ __device__ 
    inline void operator -= (const Quaternion q){
        w -= q.w;
        x -= q.x;
        y -= q.y;
        z -= q.z;
    }
    friend std::ostream& operator<<(std::ostream& os, Quaternion& q){
        os << "w: "<<q.w << ", xi: "<< q.x << ", yj: " << q.y << ", zk: " <<  q.z;
        return os;
    }
    /**************************Applications**************************/
    // cross product
    __host__ __device__
    inline float3 cross(float3 a, float3 b){ 
        return make_float3(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x); 
    }
    // rotate Vector
    __host__ __device__ 
    inline static float3 rotate(Quaternion q, float3 v){
        Quaternion quatVec(0, v.x, v.y, v.z);
        Quaternion tmp_quat = q * quatVec * q.inverse();  // conjugate()
        return make_float3(tmp_quat.x,tmp_quat.y,tmp_quat.z);
    } 
    // rotate  Vector
    __host__ __device__ 
    inline float3 rotate(float3 v){
        float3 qVec = this->getVectorPart();
        return v + cross((qVec*2.f), (cross(qVec, v) + (v * w)) );
    }
    // unit Quaternion
    __device__ __host__
    inline static Quaternion getUnit(const Quaternion q){
        Quaternion res = normalize(q);
        res.w = 0.0f;
        return res;
    }
    __device__ __host__
    inline Quaternion getUnit(void){
        Quaternion res = normalize(*this);
        res.w = 0.0f;
        return res;
    }
    // normalize
    __host__ __device__ 
    inline static Quaternion normalize(Quaternion q){
        return  Quaternion(q * sqrtf(dot(q, q)));
    }
    // normalize currecnt object
    __host__ __device__ 
    inline Quaternion normalize(void){
        return  Quaternion(*this * sqrtf(dot(*this, *this)));
    }
    // cross
    __host__ __device__ 
    inline static Quaternion cross(const Quaternion a, const Quaternion b) {
        return Quaternion(0,
                        a.y * b.z - a.z * b.y,    // todo ... check it
                        a.z * b.x - a.x * b.z,
                        a.x * b.y - a.y * b.x);
    }
    // cross
    __host__ __device__ 
    inline Quaternion cross(Quaternion b) {
        return Quaternion(0,
                        y * b.z - z * b.y,    // todo ... check it
                        z * b.x - x * b.z,
                        x * b.y - y * b.x);
    }
    // dot product
    __host__ __device__ 
    inline static float dot(Quaternion a, Quaternion b){ 
        return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    }
    // dot product
    __host__ __device__ 
    inline float dot(Quaternion b){ 
        return x * b.x + y * b.y + z * b.z + w * b.w;
    }
    // multiplicaton
    __host__ __device__ 
    inline static Quaternion mul(const Quaternion a, const Quaternion b) {
        return Quaternion(-a.x * b.x - a.y * b.y - a.z * b.z + a.w * b.w,
                           a.x * b.w + a.y * b.z - a.z * b.y + a.w * b.x,
                          -a.x * b.z + a.y * b.w + a.z * b.x + a.w * b.y,
                           a.x * b.y - a.y * b.x + a.z * b.w + a.w * b.z);
    }
    __host__ __device__ 
    inline Quaternion mul(const Quaternion b) {
        return Quaternion(-x * b.x - y * b.y - z * b.z + w * b.w,
                           x * b.w + y * b.z - z * b.y + w * b.x,
                          -x * b.z + y * b.w + z * b.x + w * b.y,
                           x * b.y - y * b.x + z * b.w + w * b.z);
    }
    // get Rotation Matrix 
    __host__ __device__ 
    inline static float3x3 getRotationMatrix(const Quaternion q){
        Quaternion v = Quaternion(0.0f, q.x * q.x, q.y * q.y, q.z * q.z);
        float3x3 res;
        res.m11 = 1.0f - 2.0f * v.y - 2.0f * v.z;
        res.m21 = 2.0f * q.x * q.y -2.0f * q.w * q.z;
        res.m31 = 2.0f * q.x * q.z +2.0f * q.w * q.y; 
  
        res.m12 = 2.0f *  q.x * q.y + 2.0f * q.w * q.z;
        res.m22 = 1.0f -  2.0f * v.x - 2.0f * v.z;
        res.m32 = 2.0f *  q.y * q.z - 2.0f * q.w * q.x; 
  
        res.m13 = 2.0f * q.x * q.z- 2.0f * q.w* q.y;
        res.m23 = 2.0f * q.y * q.z+ 2.0f * q.w* q.x;
        res.m33 = 1.0f - 2.0f * v.x- 2.0f * v.y;
        return res;
    }
    __host__ __device__ 
    inline float3x3 getRotationMatrix(void){
        Quaternion v = Quaternion(0.0f, x * x, y * y, z * z);
        float3x3 res;
        res.m11 = 1.0f - 2.0f * v.y - 2.0f * v.z;
        res.m21 = 2.0f * x * y -2.0f * w * z;
        res.m31 = 2.0f * x * z +2.0f * w * y; 
  
        res.m12 = 2.0f *  x * y + 2.0f * w * z;
        res.m22 = 1.0f - 2.0f * v.x - 2.0f * v.z;
        res.m32 = 2.0f * y * z - 2.0f * w * x; 
  
        res.m13 = 2.0f * x * z - 2.0f * w * y;
        res.m23 = 2.0f * y * z + 2.0f * w * x;
        res.m33 = 1.0f - 2.0f * v.x- 2.0f * v.y;
        return res;
    }
    // get euler angles from quaternion
    __host__ __device__
    inline math::EulerAngles getEulerAngles(const Quaternion q){
        math::EulerAngles angles;

        // roll (x-axis rotation)
        float sinr_cosp = +2.0f * (q.w * q.x + q.y * q.z);
        float cosr_cosp = +1.0f - 2.0f * (q.x * q.x + q.y * q.y);
        angles.roll = atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        float sinp = +2.0f * (q.w * q.y - q.z * q.x);
        if (fabs(sinp) >= 1.0f)
            angles.pitch = copysign(M_PI / 2.0f, sinp); // use 90 degrees if res of range
        else
            angles.pitch = asin(sinp);

        // yaw (z-axis rotation)
        float siny_cosp = +2.0f * (q.w * q.z + q.x * q.y);
        float cosy_cosp = +1.0f - 2.0f * (q.y * q.y + q.z * q.z);  
        angles.yaw = atan2(siny_cosp, cosy_cosp);

        return angles;
    }
    __host__ __device__
    inline AxisAngle getAxisAngle(const Quaternion q){
        this->normalize();
        AxisAngle aa;
        aa.theta = 2.0f * acos(q.w);
        float _sin = sin(aa.theta/2.0f);
        aa.n = make_float3(q.x/_sin, q.y/_sin, q.z/_sin);
        return aa;
    }
    // ge float x, y, z  -> used in dual quaternion (to get translation with w = 0)
    __host__ __device__
    inline static float3 getVectorPart(const Quaternion q){
        return make_float3(q.x, q.y, q.z);
    }
    // ge float x, y, z  -> used in dual quaternion (to get translation with w = 0)
    __host__ __device__
    inline float3 getVectorPart(void){
        return make_float3(this->x, this->y, this->z);
    }
};

}   // namespace math
}   // namespace DynaMap

