#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <cassert>
#include <fstream>

#ifdef __APPLE__
#  include <GLUT/glut.h>
#else
#  include <GL/glut.h>
#endif

namespace DynaMap{
 
class model{
  public: 		
    void loadFile(std::string fileName);
	void saveModel();
	void resetPose();
	void getVertex();
	void getboundary();
	void transform();
	float* getArray(void);
	
	//vectors
	std::vector<float> glModel;
	std::vector<float> vertexList;
	std::vector<std::string> facesStringList;
	std::vector<std::string> normalsStringList;

	float meanX;
	float meanY;
	float meanZ;

	float maxX;
	float minX;
	float maxY;
	float minY;
	float maxZ;
	float minZ;

	float maxScale;

	float x;
	float y;
	float z;

	float roll;
	float pitch;
	float yaw;
};

} // namespace DynaMap
