/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
#pragma once

#include <cuda.h>
#include <cuda_runtime.h>
#include <curand.h>
#include <ctime>

#include "pyramid.h"
#include "mesh/mesh.h"
#include "geom_Types.h"
#include "math/dualQuat.h"
#include "math/math_types.h"
#include "math/utils_matrix.h"
#include "geometry/mesh/meshSTD.h"
#include "dataStructure/kdtree/kdtree.h"


#define CUDA_CALL(x) do { if((x) != cudaSuccess) { \
    printf("Error at %s:%d\n",__FILE__,__LINE__);     \
    return EXIT_FAILURE;}} while(0)
#define CURAND_CALL(x) do { if((x) != CURAND_STATUS_SUCCESS) { \
    printf("Error at %s:%d\n",__FILE__,__LINE__);            \
    return EXIT_FAILURE;}} while(0)


namespace DynaMap{
namespace geometry{

struct defGraphNode{
    int id;             // ID in geraph structure
    int vertexID;       // vertex ID in mesh structure
    int *nIds;          // sorted neighbour nodeIDs in graph structure
    Vertex vertex;      // Vertex
    math::dualQuat dq;
    float* nWeights;    // sorted neighbour weights in graph structure
    float* nDistances;  // sorted neighbour distances in graph structure
};

class defGraph{
    public:
        defGraph(void);
        virtual ~defGraph(void);
        void initGraphNodes(MeshSTD &srcMesh, int mode = 0);
        void init(MeshSTD &srcMesh, int ObservationNum, int mode = 0);
        void Free(void);

        void updateActiveNeighbourNodes(pyramid &targetImage, rgbdSensor sensor, float4x4 cuPose);
        void updateActiveNeighbourNodes(MeshSTD &targetMesh, float4x4 cuPose);
        void updateNodeWeightsKDTree(kdTree &kdtree);
        void defGraphUpdate(void);

        void updateActiveNodesWeightsKDTree(kdTree &kdtree, MeshSTD &targetMesh, float4x4 cuPose);
        void updateActiveNodesWeightsKDTree(kdTree &kdtree, pyramid &targetImage, rgbdSensor sensor, float4x4 cuPose);
    
        void randomNum(int randNums[], int elements, int range);
        void sampleDownMeshDevice(void);
        void sampleDownMeshHost(int obsNum, int mode = 0);

        void defGraphUpdateNodes(void);
        void writeDefGraphToFile(const char* fileName, int obsNum = NODE_NUM, int mode = 0);

        void updateNodesDQ(float* dqVector);
        
        geometry::MeshSTD defGraphMesh;
        defGraphNode *nodes;  
        
        double MaxDistNeighbour;  // todo... needs to be integrated...
        int nNum;         // effection neighbour number  4, 5 ..
        int nodeNum;        // total number of nodes in graph 100, 200
        int activeNodesNum;  //contains number of nodes active on current model
        int visibleNodesNum;  // number of nodes visible in the scene 90, 199

        int   *visibleNodeIds;   // stores nodeIds for each pixel (sorted based on distance)
        float *visibleNWeights;
        float *visibleNDistances;

        float *identityMat;

        bool KDTREE;
        kdTree *graphKDTree;
}; 

}   // namespace Geometry
}   // namespace DynaMap

