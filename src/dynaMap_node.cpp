/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DynaMap: A non-Rigid SLAM implementation 					%
% by: Alireza Ahmadi                                     	%
% University of Bonn- MSc Robotics & Geodetic Engineering	%
% Alireza.Ahmadi@uni-bonn.de                             	%
% AlirezaAhmadi.xyz                                      	%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

#include <ros/ros.h>
#include "nodeHandler.h"
#include "dynaMap.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "DynaMap_node");
  ros::NodeHandle nodeHandle("~");

  DynaMap::DynaMapNodeHandler DynaMapNodeHandler(nodeHandle);
  
  ros::spin();
  return 0;

}
