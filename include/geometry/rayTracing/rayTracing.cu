#include "rayTracing.h"

namespace DynaMap {
namespace geometry {
    rayTracing::rayTracing(void){}
    rayTracing::~rayTracing(void){
        Free();
    }
    void rayTracing::init(MeshSTD& mesh, rgbdSensor& sensor) {
        rayCastMesh = mesh;
        rayCastingCamera = sensor;
    }
    void rayTracing::Free() {
    }

    __device__
    void rayTracing::getSurfaceProperties(const Vertex& hitPoint,
                                          const float3& rayDirection,
                                          const MeshSTD& mesh,
                                          const size_t& triangleindex,
                                          const float2& uv){
        // face normal
        // **Attention*** -1 if because the vertex ids in mesh face begins from 1 
        const float3& v0 = mesh.vertices[mesh.triangles[triangleindex].vertexIndex.x - 1].position; 
        const float3& v1 = mesh.vertices[mesh.triangles[triangleindex].vertexIndex.y - 1].position;
        const float3& v2 = mesh.vertices[mesh.triangles[triangleindex].vertexIndex.z - 1].position;
        mesh.vertices[mesh.triangles[triangleindex].vertexIndex.x].normal = cross((v1 - v0), (v2 - v0));
        normalize(mesh.vertices[mesh.triangles[triangleindex].vertexIndex.x].normal);

        // todo... complete it ...
        // texture coordinates
        // const Vec2f &st0 = texCoordinates[triIndex * 3];
        // const Vec2f &st1 = texCoordinates[triIndex * 3 + 1];
        // const Vec2f &st2 = texCoordinates[triIndex * 3 + 2];
        // hitTextureCoordinates = (1 - uv.x - uv.y) * st0 + uv.x * st1 + uv.y * st2;
    
        // vertex normal
        /*
        const Vec3f &n0 = N[triIndex * 3];
        const Vec3f &n1 = N[triIndex * 3 + 1];
        const Vec3f &n2 = N[triIndex * 3 + 2];
        hitNormal = (1 - uv.x - uv.y) * n0 + uv.x * n1 + uv.y * n2;
        */
    }
    __device__
    bool rayTracing::rayTriangleIntersect(const float3& cameraOrigin, 
                                          const float3& rayDirection,
                                          const float3& v0,
                                          const float3& v1,
                                          const float3& v2,
                                          float& t, 
                                          float& u, 
                                          float& v){
        float3 v0v1 = v1 - v0;
        float3 v0v2 = v2 - v0;
        float3 pvec = cross(rayDirection, v0v2);
        float dotProduct = DynaMap::dot(v0v1, pvec);
    
        // ray and triangle are parallel if dotProduct is close to 0
        if (fabs(dotProduct) < 1e-8) {
            // printf("reject, dotProduct: %f\n",dotProduct);
            return false;
        }
    
        float invdotProduct = 1 / dotProduct;
    
        float3 tvec = cameraOrigin - v0;
        u = DynaMap::dot(tvec, pvec) * invdotProduct;
        if (u < 0 || u > 1) return false;
    
        float3 qvec = cross(tvec, v0v1);
        v = DynaMap::dot(rayDirection, qvec) * invdotProduct;
        if (v < 0 || u + v > 1) return false;
        
        t = DynaMap::dot(v0v2, qvec) * invdotProduct;
        
        return true;
    }
    // Test if the ray interesests this triangle mesh
    __device__
    bool rayTracing::intersec(const float3& cameraOrigin,
                              const float3& rayDirection,
                              const MeshSTD& mesh,
                              float& nearestTriDistance, 
                              size_t& triangleIndex, 
                              float2& uv){
        bool isIntersecting = false;
        for (size_t i = 0; i < mesh.trianglesNum; i++) {
            float t, u, v;
            // todo ... check for tNear can be added..
            // **Attention*** -1 is because the vertex ids in mesh face begins from 1 
            if (rayTriangleIntersect(cameraOrigin,    
                                     rayDirection, 
                                     mesh.vertices[mesh.triangles[i].vertexIndex.x - 1].position, 
                                     mesh.vertices[mesh.triangles[i].vertexIndex.y - 1].position, 
                                     mesh.vertices[mesh.triangles[i].vertexIndex.z - 1].position, 
                                     t, u, v))
                    if(t < nearestTriDistance) { 
                // if the ray is intersecting any trianle in mesh we stack laying point, 
                // distance to triangle and triangle index    
                nearestTriDistance = t;
                uv.x = u;
                uv.y = v;
                triangleIndex = i;
                isIntersecting = true;
            } 
        }
        return isIntersecting;
    }
    __device__
    float rayTracing::rayCast(const float3& cameraOrigin, 
                              const float3& rayDirection,
                              MeshSTD& mesh,
                              size_t rayID){
                                
        float closestTriDistance = 10e4;
        float2 uv;
        size_t triangleIndex = 0;
        // if point lies in on of the triangles --> then compute interpolated attribites
        if (intersec(cameraOrigin, rayDirection, mesh, closestTriDistance, triangleIndex, uv)) {
            Vertex hitVertex;
            getSurfaceProperties(hitVertex, rayDirection, mesh, triangleIndex, uv);
            // float NdotView = std::max(0.f, hitNormal.dotProduct(-dir));
            // prepare depth value 
        }else{
            // printf("rejected Rays: %d\n",rayID);
        }
        return closestTriDistance;
    }
    __global__ 
    void GenerateDepthKernel(rayTracing& rayTracing,
                             const MeshSTD& mesh,
                             rgbdSensor& sensor,
                             float4x4 cameraPose,
                             float* virtual_depth) {

        int index = blockIdx.x * blockDim.x + threadIdx.x;
        int stride = blockDim.x * gridDim.x;
        int size = sensor.rows * sensor.cols;
        
        float3 cameraOrigin = make_float3(cameraPose.m14, cameraPose.m24, cameraPose.m34);
        // field of view in degrees 58'
        float scale = tan(deg2rad(58.0f/2));
        // getting image aspect retion columns/rows
        float imageAspectRatio = sensor.cols / (float)sensor.rows; 
        for (int idx = index; idx < size; idx += stride) {
            // extracting v, u image position of idx 
            int v = static_cast<int>(idx / sensor.cols);
            int u = static_cast<int>(idx - sensor.cols * v);
            // generating primary ray direction
            float pixelNDCx = (u + 1) / (float)sensor.cols;
            float pixelNDCy = (v + 1) / (float)sensor.rows;
            float pixelCamerax = (2 * pixelNDCx - 1) * imageAspectRatio * scale; 
            float pixelCameray = (1 - 2 * pixelNDCy) * scale; 
            // picking a 1.0 meter far point in frustrum
            float4 point3D = make_float4(pixelCamerax, pixelCameray, -1.0f,0.0f);
            // tranforming 3D point to camera coordinate
            point3D = cameraPose * point3D;  
            // computing noraml direction toward picked point (normalized)           
            float3 rayDirection = make_float3(normalize(point3D));
            // getting depth value of pixel[idx]
            virtual_depth[idx] = rayTracing.rayCast(cameraOrigin, 
                                                    rayDirection, 
                                                    rayTracing.rayCastMesh, 
                                                    idx); 
            if(virtual_depth[idx] == 0)virtual_depth[idx] = 0.2;                                  
        }
    }
    void rayTracing::GenerateDepth(float4x4& cameraPose, float* virtual_depth){
        // todo ... can be one thread per tirangle too (then should be more efficent)
        int threads_per_block = 512;
        int thread_blocks = (rayCastingCamera.rows * rayCastingCamera.cols + threads_per_block - 1) / threads_per_block; 
        #ifdef LOG_EN
            std::cout << "<<< GenerateDepthKernel >>> threadBlocks: "<< thread_blocks << 
                ", threadPerBlock: " << threads_per_block << 
                ", triNum: " << rayCastMesh.trianglesNum << 
                ", vertNum: " << rayCastMesh.verticesNum << 
                std::endl;
        #endif
        GenerateDepthKernel<<<thread_blocks, threads_per_block>>>(*this, rayCastMesh, rayCastingCamera, cameraPose, virtual_depth);
        cudaDeviceSynchronize();
    }

}  // namespace geometry
}  // namespace DynaMap