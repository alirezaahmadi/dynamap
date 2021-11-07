/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

#include <cuda.h>
#include <cuda_runtime.h>
#include <cmath>
#include <math.h>
#include "utils_matrix.h"
#include "cuda_math.h"
#include "math_types.h"
#include <iostream>
#include "math/Quaternion.h"
#include "math/dualQuat.h"
#define EPSILLON 0.1f;
using namespace DynaMap;

int main(int argc, char **argv) {
  float3 testPoint = make_float3(1, 1, 1);
  float3 resPoint = make_float3(0, 0, 0);
  
  math::Quaternion q(1, 0, 0, 0);
  math::Quaternion t(0, 0, 0, 0.1);

  resPoint = math::Quaternion::rotate(q, testPoint);
  std::cout << q << std::endl;
  std::cout << "q: " << resPoint.x << ", " << resPoint.y << ", " << resPoint.z << std::endl;
  
  

  float4x4 T;
  T.setIdentity();
  T.m14 = 10;
  T.m24 = 10;
  T.m34 = 10;
  // math::dualQuat dq(q,t);
  // math::dualQuat dq(q, make_float3(10,10,5));
  math::dualQuat dq(T);
  std::cout << dq << std::endl;
  resPoint = dq.transformPosition(testPoint);
  std::cout << "dq: " << resPoint.x << ", " << resPoint.y << ", " << resPoint.z << std::endl;

  T.setIdentity();
  T.m14 = 10;
  T.m24 = 10;
  T.m34 = 10;
  math::dualQuat dq1(T);
  math::dualQuat dq2(T);
  math::dualQuat blend;
  blend = dq1 * 0.1 + dq2 * 0.1;
  // dq.setIdentity();
  // dq.addTranslation(make_float3(1,1,1));
  resPoint = blend.transformPosition(testPoint);
  std::cout << "blend: " << resPoint.x << ", " << resPoint.y << ", " << resPoint.z << std::endl;

  math::EulerAngles eu;
  eu.roll = 0.1f;
  eu.pitch = 0.0f;
  eu.yaw = 0.35f;
  std::cout << "eu: " << eu << std::endl;

  math::Quaternion q1(eu);
  std::cout << "Qaut: \n" << q1 << std::endl;

  math::EulerAngles quat_euAngles = q1.getEulerAngles(q1);
  std::cout << "quat_euAngles: \n" << quat_euAngles << std::endl;

  math::dualQuat dq3;
  dq3.setIdentity();
  dq3 = math::dualQuat(eu, make_float3(0,0,0));
  dq3.addTranslation(make_float3(10,0,-2));
  std::cout << "dual-Quat: \n" << dq3 << std::endl;

  math::EulerAngles euAngles = dq3.getEulerAngles();
  float3 trans = dq3.getTranslation();
  std::cout << "dual-Quat_euAngles: \n" << euAngles << std::endl;
  std::cout << "dual-Quat_Trans: \n" << trans.x << ", " << trans.y << ", " << trans.z << std::endl;  

  std::cout << "********************************************************" << std::endl;
  math::dualQuat backup;
  for (size_t paramID = 0; paramID < 6; paramID++){

    // get Euler Angles from dq
    math::EulerAngles euAngles;
    // get translation vector from dq
    float3 trans = make_float3(0.0f, 0.0f, 0.0f);

    switch(paramID){
        case 0:{
            // roll -> x
            euAngles.roll -= 2* EPSILLON; 
            break;
        }case 1:{
            // pitch -> y
            euAngles.pitch -= 2* EPSILLON; 
            break;
        }case 2:{
            // yaw -> z
            euAngles.yaw -= 2* EPSILLON; 
            break;
        }case 3:{
            // trans -> x
            trans.x -= 2* EPSILLON;  
            break;
        }case 4:{
            // trans -> y
            trans.y -= 2* EPSILLON;  
            break;
        }case 5:{
            // trans -> z
            trans.z -= 2* EPSILLON;  
            break;
        }
    }
    // get dq form again
    math::dualQuat pDQ(euAngles, trans);
    // pDQ = math::dualQuat::identity();
    // blend vertex with k neighbour 
    backup = pDQ;
    std::cout << "pID: " << paramID << ", backup : \n" <<  backup << std::endl;
  }

  return 0;
}
