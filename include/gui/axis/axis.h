#pragma once

#include <vector>
#include <string>
#include <fstream>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "cuda.h"
#include "cuda_runtime.h"

#ifdef _WIN32
#define WINDOWS_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#endif

#if defined(__APPLE__) || defined(MACOSX)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

namespace DynaMap{
namespace gui{

	class axis {
	public: 	
	
		void updatePose(Eigen::Matrix4f Pose);
		void init(float width=3.0);
		void draw();
		
		float axisLength;
		float axisWidth;

		float x;
		float y;
		float z;

		float roll;
		float pitch;
		float yaw;

	private:
		
	};
}
} // namespace DynaMap
