#include "axis.h"

namespace DynaMap{
namespace gui{

  void axis::init(float width){
	this->x = 0.0f;
	this->y = 0.0f;
	this->z = 0.0f;
	this->roll = 0.0f;
	this->pitch = 0.0f;
	this->yaw = 0.0f;
	this->axisLength = 0.3f;
	this->axisWidth = width;
  }

  void axis::draw(){
	glLineWidth(this->axisWidth);
	// draw some lines

	// glColor3f(1.0,1.0,0.0); // red x
	// glBegin(GL_POINTS);
	// glVertex3f(0.0f, 0.0f, 0.0f);
	// glVertex3f(x, y, z);
	// glEnd();

	// x aix
	glColor3f(1.0f,0.0f,0.0f); // red x
	glBegin(GL_LINES);
	glVertex3f(0.0f,0.0f,0.0f);
	glVertex3f(0.0f, this->axisLength, 0.0f);
	glEnd();

	// y 
	glColor3f(0.0f,1.0f,0.0f); // green y
	glBegin(GL_LINES);
	glVertex3f(0.0f,0.0f,0.0f);
	glVertex3f(0.0f, 0.0f, -this->axisLength);
	glEnd();

	// z 
	glColor3f(0.0f, 0.0f, 1.0f); // blue z
	glBegin(GL_LINES);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(-this->axisLength, 0.0f , 0.0f );
	glEnd();

	glLineWidth(1.0);
  }

  void axis::updatePose(Eigen::Matrix4f Pose){  
    this->x = Pose(0,3);
    this->y = Pose(1,3);
    this->z = Pose(2,3);

    Eigen::Matrix3f R = Pose.block<3,3>(0,0);
    Eigen::Vector3f euAngles = R.eulerAngles(2, 1, 0); 

    this->roll  = euAngles(0) * 180/M_PI;
    this->pitch = euAngles(1) * 180/M_PI;
    this->yaw   = euAngles(2) * 180/M_PI;
  }

}
} // namespace DynaMap
