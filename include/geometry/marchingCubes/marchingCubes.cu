#include "marchingCubes.h"
#include "helper_math.h"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>

#include "utils_Cuda.h"
#include "geometry/marchingCubes/lookupTables.h"
#include "gui/model/model.h"
#include "gui/camera/camera.h"


#define BLOCK_SIZE 512
#define PI 3.141592654f

#define gpuErrchk(ans) { gpuAssert((ans), (char*)__FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, char* file, int line,
		bool abort = true) {
	if (code != cudaSuccess) {
		fprintf(stderr, "GPUassert: %s %s %d\n", cudaGetErrorString(code), file,
				line);
		if (abort)
			exit(code);
	}
}

// Rendering variables
float xmax = 10.0f;
float xmin = -10.0f;
int numPoints = 3;
int dim = 3;
int func = 0;

// Flag to toggle CUDA usage
int cuda = 1;

DynaMap::model model_object;		// an instance of Model
DynaMap::gui::camera camera_object;	// an instance of Camera

// Surface to be rendered in 3D
__device__ __host__
int function3D(float4& point, int func)
{
	float fun; int flag;
	switch (func) {

	case 0:
		fun = point.x * point.x + point.y * point.y + point.z * point.z;
		flag = (fun < 9);
		break;

	case 1:
		fun = point.x * point.x / 5.0 + point.y * point.y / 3.0
			- point.z * point.z / 7.0;
		flag = (fun < 5);
		break;

	case 2:
		fun = point.x * point.x / 10.0 - point.y * point.y / 3.0
			- point.z / 2.0;
		flag = (fun < 0);
		break;
	}

	return flag;
}


///////////////////////////////////////////////////////////////////////////////
// CUDA kernels																 //
///////////////////////////////////////////////////////////////////////////////

// This kernel checks whether each point lies within the desired surface.
__global__
void points_kernel(float4* points, int size, int dim, int func)
{
	// Get unique thread id
	unsigned int globalID = blockIdx.x * blockDim.x + threadIdx.x;

	// Check whether the point lies within.
	// Fourth coordinate represent containment.
	for (int k = globalID; k < size * size * size;
			k += gridDim.x * blockDim.x) {
		float4 pt = points[k];
		points[k].w = function3D(pt, func);
	}

}

// This kernel classifies each cube in the grid.
__global__
void kernel3D(float4* points, float4* geom, int size)
{
	// Get unique thread ID, this is the point ID
	unsigned int globalID = blockIdx.x * blockDim.x + threadIdx.x;

	for (int id = globalID; id < size * size * size; id += gridDim.x * blockDim.x) {

		// Transform point ID to cube ID
		int j = (int) ( (int) floor((double) (id / size)) % size );
		int k = (int) floor((double) (id / (size*size)));
		int idx = id - j + k - 2 * k * size;

		if (idx < (size - 1) * (size - 1) * (size - 1)) {

			// Get the vertices of this cube
			float4 verts[8];
			verts[0] = points[id];
			verts[1] = points[id + 1];
			verts[2] = points[id + size + 1];
			verts[3] = points[id + size];

			verts[4] = points[id + size*size];
			verts[5] = points[id + size*size + 1];
			verts[6] = points[id + size*size + size + 1];
			verts[7] = points[id + size*size + size];

			// Obtain the type of this cube
			int type = 0;
			for (int l = 0; l < 8; l++) {
				type += verts[l].w * pow((double)2,(double)l);
			}

			// Get the configuration for this type of cube from the table
			// and generate the triangles accordingly
			int* config = triangleTable[type];
			int e, e0, e1;
			for (int l = 0; l < 15; l++) {
				e = config[l];
				e0 = cube_edgeToVerts[e][0]; e1 = cube_edgeToVerts[e][1];
				if (e != -1) {
					geom[15*idx + l] = ( verts[e0] + verts[e1] ) * (0.5f);
					geom[15*idx + l].w = 1.0f;
				} else { break; }
			}

		}
	}
}


///////////////////////////////////////////////////////////////////////////////
// Run the CUDA part of the computation										 //
///////////////////////////////////////////////////////////////////////////////
void runCuda(GLuint *vbo)
{
	// Map OpenGL buffer object for writing from CUDA
	float4* dev_points;
	float4* dev_geometry;

	// Map OpenGL buffers to CUDA
	cudaGLMapBufferObject((void**) &dev_points, vbo[1]);
	cudaGLMapBufferObject((void**) &dev_geometry, vbo[2]);

	// Choose a block size and a grid size
	const unsigned int threadsPerBlock = BLOCK_SIZE;
	const unsigned int maxBlocks = 50;
	unsigned int blocks;


	blocks = min(maxBlocks,
			(int) ceil(
					numPoints * numPoints * numPoints
							/ (float) threadsPerBlock));

	// Check for containment of vertices
	points_kernel<<<blocks, threadsPerBlock>>>
			(dev_points, numPoints, dim, func);

	// Obtain the triangles from the data table
	kernel3D<<<blocks, threadsPerBlock>>>
			(dev_points, dev_geometry, numPoints);


	// Unmap buffer objects from CUDA
	cudaGLUnmapBufferObject(vbo[1]);
	cudaGLUnmapBufferObject(vbo[2]);
}


///////////////////////////////////////////////////////////////////////////////
// Vertex Buffer Objects													 //
///////////////////////////////////////////////////////////////////////////////

// Initialize 3D data
void createData3D(float4* points, float4* grid, float4* geom)
{
	// Initialize points data.
	float delta = (xmax - xmin) / (numPoints - 1);
	for (int i = 0; i < numPoints; i++) {
		for (int j = 0; j < numPoints; j++) {
			for (int k = 0; k < numPoints; k++) {

				int idx = i + j * numPoints + k * numPoints * numPoints;

				// Set initial position data
				points[idx].x = xmin + delta * i;
				points[idx].y = xmax - delta * j;
				points[idx].z = xmin + delta * k;
				points[idx].w = 1.0f;
			}
		}
	}

	// Initialize grid data.
	for (int i = 0; i < (numPoints - 1); i++) {
		for (int j = 0; j < (numPoints - 1); j++) {
			for (int k = 0; k < (numPoints - 1); k++) {

				int idx_pt = i + j * numPoints + k * numPoints * numPoints;
				int idx_sq = idx_pt - j + k - 2 * k * numPoints;

				// Set initial position data
				grid[16 * idx_sq + 0] = points[idx_pt];
				grid[16 * idx_sq + 1] = points[idx_pt+1];
				grid[16 * idx_sq + 2] = points[idx_pt+numPoints+1];
				grid[16 * idx_sq + 3] = points[idx_pt+numPoints];

				grid[16 * idx_sq + 4] = points[idx_pt+numPoints*numPoints];
				grid[16 * idx_sq + 5] = points[idx_pt+numPoints*numPoints+1];
				grid[16 * idx_sq + 6] = points[idx_pt+numPoints*numPoints+numPoints+1];
				grid[16 * idx_sq + 7] = points[idx_pt+numPoints*numPoints+numPoints];

				grid[16 * idx_sq + 8] = points[idx_pt];
				grid[16 * idx_sq + 9] = points[idx_pt+1];
				grid[16 * idx_sq + 10] = points[idx_pt+numPoints*numPoints+1];
				grid[16 * idx_sq + 11] = points[idx_pt+numPoints*numPoints];

				grid[16 * idx_sq + 12] = points[idx_pt+numPoints];
				grid[16 * idx_sq + 13] = points[idx_pt+numPoints+1];
				grid[16 * idx_sq + 14] = points[idx_pt+numPoints*numPoints+numPoints+1];
				grid[16 * idx_sq + 15] = points[idx_pt+numPoints*numPoints+numPoints];

			}
		}
	}

	// Initialize geometry data.
	float4 zero = make_float4(0.0f, 0.0f, 0.0f, 0.0f);
	for (int k = 0; k < (numPoints - 1) * (numPoints - 1) * (numPoints - 1) * 15; k++) {
		geom[k] = zero;
	}
}

// Create VBOs
void createVBOs(GLuint* vbo)
{
	// Create VBOs.
	glGenBuffers(3, vbo);

	// Initialize points and grid
	unsigned int points_size;
	float4* points;
	unsigned int grid_size;
	float4* grid;
	unsigned int geom_size;
	float4* geom;


	// Allocate memory
	points_size = numPoints * numPoints * numPoints * sizeof(float4);
	points = (float4*) malloc(points_size);
	grid_size = (numPoints - 1) * (numPoints - 1) * (numPoints - 1) * 16
			* sizeof(float4);
	grid = (float4*) malloc(grid_size);
	geom_size = (numPoints - 1) * (numPoints - 1) * (numPoints - 1) * 15
			* sizeof(float4);
	geom = (float4*) malloc(geom_size);
	// Initialize data
	// createData3D(points, grid, geom);

	// Activate VBO id to use.
	glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);

	// Upload data to video card.
	glBufferData(GL_ARRAY_BUFFER, grid_size, grid, GL_DYNAMIC_DRAW);

	// Activate VBO id to use.
	glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);

	// Upload data to video card.
	glBufferData(GL_ARRAY_BUFFER, points_size, points, GL_DYNAMIC_DRAW);

	// Register buffer objects with CUDA
	gpuErrchk(cudaGLRegisterBufferObject(vbo[1]));

	std::string add = "bunny.obj";

	glPushMatrix();
	model_object.loadFile(add);
	glPopMatrix();

	// Activate VBO id to use.
	glBindBuffer(GL_ARRAY_BUFFER, vbo[2]);

	// Upload data to video card.
	glBufferData(GL_ARRAY_BUFFER, model_object.glModel.size() * sizeof(float), &model_object.glModel[0], GL_DYNAMIC_DRAW);
	// glBufferData(GL_ARRAY_BUFFER, geom_size, geom, GL_DYNAMIC_DRAW);
	// Register buffer objects with CUDA
	gpuErrchk(cudaGLRegisterBufferObject(vbo[2]));

	// Free temporary data
	free(points); free(grid); free(geom);

	// Release VBOs with ID 0 after use.
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	// Execute the algorithm, if asked
	// if (cuda) { runCuda(vbo); }

}

// Delete VBOs
void deleteVBOs(GLuint* vbo)
{
	// Delete VBOs
	glBindBuffer(1, vbo[0]);
	glDeleteBuffers(1, &vbo[0]);
	glBindBuffer(1, vbo[1]);
	glDeleteBuffers(1, &vbo[1]);
	glBindBuffer(1, vbo[2]);
	glDeleteBuffers(1, &vbo[2]);

	// Unregister buffer objects with CUDA
	gpuErrchk(cudaGLUnregisterBufferObject(vbo[1]));
	gpuErrchk(cudaGLUnregisterBufferObject(vbo[2]));

	// Free VBOs
	*vbo = 0;
}


///////////////////////////////////////////////////////////////////////////////
// Gets/sets the number of vertices											 //
///////////////////////////////////////////////////////////////////////////////
int getNumPoints()
{
	return numPoints;
}
void setNumPoints(int n)
{
	numPoints = n;
}

///////////////////////////////////////////////////////////////////////////////
// Gets/sets the dimension													 //
///////////////////////////////////////////////////////////////////////////////
int getDimension()
{
	return dim;
}
void setDimension(int n)
{
	dim = n;
}

///////////////////////////////////////////////////////////////////////////////
// Sets the GPU usage														 //
///////////////////////////////////////////////////////////////////////////////
void setCUDA()
{
	cuda = 1 - cuda;
}

///////////////////////////////////////////////////////////////////////////////
// Changes the function of the surface to render							 //
///////////////////////////////////////////////////////////////////////////////
void changeFunction()
{
	func = (func+1)%3;
}

//***************************** Vertex Buffer Objects *************************
void writeMatrixToTxt(const char* fileName, 
	float4 *matrix,
	size_t size){


	float4 *h_matrix = new float4[size];
	memset(h_matrix, 0, sizeof(float4) * size );
	cudaMemcpy(h_matrix, matrix, 
	sizeof(float4) * size, 
	cudaMemcpyDeviceToHost);
	cudaDeviceSynchronize();

	std::ofstream file(fileName,std::ios::ate);
	if (file.is_open()){
	file << fileName <<", size: " << size << std::endl;
	file << "{ " << std::endl;
	for (size_t i = 0; i < size; i++){
	file << std::fixed << std::setprecision(6)
	<< i << " -> x: "<< h_matrix[i].x 
	<< ", y: " << h_matrix[i].y
	<< ", z: " << h_matrix[i].z 
	<< ", w: " << h_matrix[i].w  
	<< std::endl;
	}
	file << " }"<< std::endl;
	file.close();
	}
	else std::cout << "Unable to open file";

	delete[] h_matrix;
} 

