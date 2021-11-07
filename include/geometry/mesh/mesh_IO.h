/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
#pragma once
#include <cuda_runtime.h> 

// #include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <fstream>
#include <vector>
#include <boost/filesystem.hpp>

//Eigen
#include <eigen3/Eigen/Dense>

#include "utils/utils.h"
#include "geometry/geom_Types.h"


namespace DynaMap{
namespace geometry{

    struct BoundingBox{
        Eigen::Vector3d _min;
        Eigen::Vector3d _max;
    };

    class Mesh_IO{
        public:
            
            unsigned int verticesNum;
            unsigned int normalNum;
            unsigned int textureNum;
            unsigned int trianglesNum;

            // Triangle *triangles;
            Vertex *vertices;
            Polygon *triangles;              // all triangles in a mesh structure

            unsigned int maxTriangleNum;
            unsigned int maxVertexNum;

            BoundingBox BBox;
            Mesh_IO(unsigned int& vertexNum, unsigned int& triangleNum);
            Mesh_IO(const char* filename);
            virtual ~Mesh_IO();
            void Free(void);

            void meshInfo()const;

            void push_backVertex(float3 vertexPose, unsigned int id);
            void push_backNormal(float3 vertexNormal, unsigned int id);
            void push_backTexture(float3 vertexTexture, unsigned int id);
            void Mesh2MeshSTD(void);
            void Mesh2MeshSTDCompressed(void);
            void readOBJ(const char* filename);
            void saveOBJ(const char* filename, int mode =0)const;
            void saveOBJVector(const char* filename, std::vector<Polygon>& Triangles, std::vector<Vertex>& Vetices)const;
            // void readPLY(const char* filename);
            // void saveOBJ(const char* filename)const;
            // void savePLY(const char* filename)const;

            void updateVerticesNormals();
            void updateFacesNormals();
            void updateNormalIndices(void);
            // void downSample(size_t sampledMesh_idx, double maxDist);

            // void pointCloud2PolyMesh(pcl::PolygonMesh& triangles, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
            // void polyMesh2Mesh(Mesh& mesh);
            // void mesh2PointCloud(pcl::PointCloud<pcl::PointNormal>::Ptr& cloud);
    }; 
}  // namespace Geometry
}  // namespace DynaMap




          // void render(DisplayMode mode);
            // void getBoundingBox(Eigen::Vector3d& Min, Eigen::Vector3d& Max);
            // static void drawBoundingBox(Eigen::Vector3d& _min, Eigen::Vector3d& _max);

            // Geometry::Mesh PolyMesh2Mesh(pcl::PolygonMesh triangles);
            // Geometry::Mesh TriMesh2Mesh(pcl::PolygonMesh triangles);

            // pcl::PolygonMesh pointCloud2PolyMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
            // void _save_pcd2ply(std::vector<Eigen::Vector3d> node_pos, 
            // std::vector<Eigen::Vector3d> node_norm, const char* filename);
            // void _save_pcd2obj(std::vector<Eigen::Vector3d> node_pos, 
            // std::vector<Eigen::Vector3d> node_norm, const char* filename);
