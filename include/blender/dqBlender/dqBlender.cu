/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
#include "dqBlender.h"
#define effectConst 0.5f
#define knn 4

namespace DynaMap{

namespace blender{

    dqBlender::dqBlender(void){}
    dqBlender::~dqBlender(void){
        Free();
    }
    void dqBlender::init(geometry::MeshSTD *srcMesh){
        sourceMesh = srcMesh;
        cudaMallocManaged(&nWeights,    sizeof(float) * srcMesh->verticesNum * srcMesh->verticesNum);
        cudaMallocManaged(&nDistances,  sizeof(float) * srcMesh->verticesNum * srcMesh->verticesNum);
        cudaMallocManaged(&nIds,  sizeof(int) * srcMesh->verticesNum * srcMesh->verticesNum);
        cudaDeviceSynchronize();
    }
    void dqBlender::init(geometry::defGraph& graph){
        sourceMesh = NULL;
    }
    void dqBlender::Free(void){
        if(sourceMesh != NULL){
            cudaFree(nDistances);
            cudaFree(nWeights);
            cudaFree(nIds);
        }
    }
    //************************** blend mesh based on defGraph ******************************
    __device__
    float3 dqBlender::blendVertexPose(math::dualQuat* subWarpField,
                                      float* subWarpFiledWeights,
                                      int* subWarpFiledIDs,
                                      float3 vertexPose,
                                      int vertexID){
        math::dualQuat dqblend;
        dqblend = math::dualQuat::identity();

        for(int n = 0; n < KNN; n++){
                dqblend +=  subWarpField[n] * subWarpFiledWeights[n];
        }
        dqblend.normalize(); 

        for(int n = 0; n < KNN; n++){
            if(subWarpFiledIDs[n] == vertexID){
                dqblend = subWarpField[n];
            }
        }

        float3 blendedPose;
        if(dqblend != math::dualQuat::identity()){
            // get blended vertex position
            blendedPose = dqblend.transformPosition(vertexPose);
        }else{
            blendedPose = vertexPose;
        } 
        return blendedPose;
    }
    __device__
    float3 dqBlender::blendVertexPose(const geometry::defGraph& graph,
                                      float3 vertexPose,
                                      unsigned int vertexID){
        math::dualQuat dqblend;
        dqblend = math::dualQuat::identity();
        for(int n = 0; n < graph.nNum; n++){
            int nIdx = 0;
            if(graph.KDTREE){  
                nIdx = vertexID * graph.nNum + n;
            }else{
                nIdx = vertexID * graph.nodeNum + n;
            }
            dqblend +=  graph.nodes[graph.visibleNodeIds[nIdx]].dq * graph.visibleNWeights[nIdx];
        }
        dqblend.normalize();

        for(int n = 0; n < graph.nodeNum; n++){
            if(graph.nodes[n].id == vertexID){
                dqblend = graph.nodes[n].dq;
            }
        }
        
        float3 blendedPose;
        if(dqblend != math::dualQuat::identity()){
            // get blended vertex position
            blendedPose = dqblend.transformPosition(vertexPose);
        }else{
            blendedPose = vertexPose;
        } 
        return blendedPose;
    }
    __device__
    void dqBlender::blendVertexPose(geometry::Vertex srcVertex,
                                    geometry::Vertex dstVertex,
                                    geometry::defGraph& graph,
                                    unsigned int vertexID){
        math::dualQuat dqblend;
        dqblend = math::dualQuat::identity();
        for(int n = 0; n < graph.nNum; n++){
            int nIdx = 0;
            if(graph.KDTREE){  
                nIdx = vertexID * graph.nNum + n;
            }else{
                nIdx = vertexID * graph.nodeNum + n;
            }
            dqblend +=  graph.nodes[nIds[nIdx]].dq * graph.nodes[nIds[nIdx]].nWeights[nIdx];
        }
        if(graph.nodes[vertexID].dq != math::dualQuat::identity()){
            dqblend += graph.nodes[vertexID].dq;
        }
        dqblend.normalize(); 

        for(int n = 0; n < graph.nodeNum; n++){
            if(graph.nodes[n].id == vertexID){
                dqblend = graph.nodes[n].dq;
            }
        }

        if(dqblend != math::dualQuat::identity()){
            // get blended vertex position
            dstVertex.position = dqblend.transformPosition(srcVertex.position);
        }else{
            dstVertex.position = srcVertex.position;
        } 
    }
    __device__
    void dqBlender::blendVertexNormal(geometry::Vertex srctNromal,
                                      geometry::Vertex dstNromal,
                                      geometry::defGraph& graph,
                                      unsigned int vertexID){
        math::dualQuat dqblend;
        dqblend = math::dualQuat::identity();
        for(int n = 0; n < graph.nNum; n++){
            int nIdx = vertexID * graph.nodeNum + n;                
            dqblend +=  graph.nodes[nIds[nIdx]].dq * graph.nodes[nIds[nIdx]].nWeights[nIdx];
        }
        if(graph.nodes[vertexID].dq != math::dualQuat::identity()){
            dqblend += graph.nodes[vertexID].dq;
        }
        dqblend.normalize(); 

        for(int n = 0; n < graph.nodeNum; n++){
            if(graph.nodes[n].id == vertexID){
                dqblend = graph.nodes[n].dq;
            }
        }

        if(dqblend != math::dualQuat::identity()){
            // get blended vertex position
            dstNromal.normal = dqblend.transformNormal(srctNromal.normal);
        }else{
            dstNromal.normal = srctNromal.normal;
        } 
    }
    __device__
    void dqBlender::blendVertex(geometry::MeshSTD& dstMesh,
                                geometry::defGraph& graph,
                                unsigned int vertexID){

        math::dualQuat dqblend;
        dqblend.setIdentity();
        for(int n = 0; n < graph.nNum; n++){
            int nIdx = 0;
            if(graph.KDTREE){
                nIdx = vertexID * graph.nNum + n;
            }else{
                nIdx = vertexID * graph.nodeNum + n; 
            }
            dqblend +=  graph.nodes[graph.visibleNodeIds[nIdx]].dq * graph.visibleNWeights[nIdx];
            dqblend.normalize();
        }
        for(int n = 0; n < graph.nodeNum; n++){
            if(graph.nodes[n].id == vertexID){
                dqblend = graph.nodes[n].dq;
            }
        }
         
        dstMesh.vertices[vertexID].position = dqblend.transformPosition(graph.defGraphMesh.vertices[vertexID].position);
        dstMesh.vertices[vertexID].normal = dqblend.transformNormal(graph.defGraphMesh.vertices[vertexID].normal);
    }
    __global__
    void blendMeshbyDefGraphKernel(dqBlender& blender,
                                   geometry::defGraph& graph,
                                   geometry::MeshSTD& dstMesh){

        int index = blockIdx.x * blockDim.x + threadIdx.x;
        int stride = blockDim.x * gridDim.x;
        int size = graph.defGraphMesh.verticesNum;
        for (int idx = index; idx < size; idx += stride){
            // blend mesh vertices based on nearby nodes 
            blender.blendVertex(dstMesh, graph, idx);
        }
    }
    void dqBlender::blendMesh(geometry::MeshSTD& dstMesh, 
                              geometry::defGraph& graph){
                
        int threads_per_block = 512;
        int thread_blocks =(graph.defGraphMesh.verticesNum + threads_per_block - 1) / threads_per_block;
        // std::cout << "<<< blendMeshbyDefGraph >>> threadBlocks: "<< thread_blocks << 
        //              ", threadPerBlock: " << threads_per_block << 
        //              ", VertexNum: " << graph.defGraphMesh.verticesNum <<
        //              ", nodeNum: " << graph.nodeNum <<
        //               std::endl;
        blendMeshbyDefGraphKernel<<<thread_blocks, threads_per_block>>>(*this,
                                                                        graph,
                                                                        dstMesh);
        cudaDeviceSynchronize();

        // for(int cnt=0; cnt<graph.nodeNum; cnt++)
        // std::cout <<cnt << ": "<< graph.nodes[cnt].dq << std::endl;
    }
    //************************** blend mesh based on DQ arrays ******************************
    __device__
    void dqBlender::blendVertex(geometry::MeshSTD& dstMesh,
                                math::dualQuat *DualQuat,
                                unsigned int vertexID){
        math::dualQuat dqblend;
        dqblend.setIdentity();
        for(int n = 0; n < knn; n++){
            int nIdx = vertexID * sourceMesh->verticesNum + n;
            dqblend += DualQuat[nIds[nIdx]] * nWeights[nIdx];
        }
        if(DualQuat[vertexID] != math::dualQuat::identity()){
            dqblend += DualQuat[vertexID];
        }
        dqblend.normalize();
        // get blended vertex position
        dstMesh.vertices[vertexID].position = dqblend.transformPosition(sourceMesh->vertices[vertexID].position);
        dstMesh.vertices[vertexID].normal = dqblend.transformNormal(sourceMesh->vertices[vertexID].normal);
    }
    __global__
    void blendMeshKernel(dqBlender& blender,
                         geometry::MeshSTD& dstMesh,
                         math::dualQuat *DualQuat){

        int index = blockIdx.x * blockDim.x + threadIdx.x;
        int stride = blockDim.x * gridDim.x;
        int size = blender.sourceMesh->verticesNum;

        for (int idx = index; idx < size; idx += stride){
            blender.blendVertex(dstMesh,   
                                DualQuat,
                                idx);
        }
    }
    __global__
    void updateNodeWeightsKernel(dqBlender& blender){
        int index = blockIdx.x * blockDim.x + threadIdx.x;
        int stride = blockDim.x * gridDim.x;
        int size = blender.sourceMesh->verticesNum;
        for (int idx = index; idx < size; idx += stride){
            // In case of using radius reach for the neighborhood, this parameter will show number of close nodes
            for(int w = 0; w < blender.sourceMesh->verticesNum; w++){
                int w_index = idx * blender.sourceMesh->verticesNum + w;
                // float ref = effectConst * blender.nDistances[idx * size];
                float ref = effectConst ;
                if(expWeight){
                    // supposed distance[0] contains leasts distance after sorting
                    blender.nWeights[w_index] = exp(-pow(blender.nDistances[idx * size + w],2) / pow(ref,2));  
                }else{
                    blender.nWeights[w_index] = blender.nDistances[idx * size] * effectConst / blender.nDistances[idx * size + w];   
                }
            }
            // if(idx == 10)
            // for(int cnt=0; cnt< graph.nodeNum; cnt++){
            //     printf("dist: %f, ids: %d, W: %f\n", graph.nodes[idx].nDistances[cnt], graph.nodes[idx].nIds[cnt],graph.nodes[idx].nWeights[cnt]);
            // }
        }  
    }
    __global__
    void updateNodeDistnacesKernel(dqBlender& blender){
        int index = blockIdx.x * blockDim.x + threadIdx.x;
        int stride = blockDim.x * gridDim.x;
        int size = blender.sourceMesh->verticesNum;

        for (int idx = index; idx < size; idx += stride){
            // invoking target node vertex position from degGraph
            geometry::Vertex vi = blender.sourceMesh->vertices[idx];
            int nIdx = idx * size;
            for(int n = 0; n < size; n++){
                // shouldn't add node itself as a neighbour in neighbour list
                if(n == idx) continue;
                // invoking neighbour node j vertex position from degGraph
                geometry::Vertex vj = blender.sourceMesh->vertices[n];
                // computing distance between target node vi and i-th neighbour vertex position 
                float tmp_dist = distance(vi.position, vj.position);
                // excluding absolute 0.0 to avoid nan and inf products 
                if(tmp_dist < 10e-5) tmp_dist = 10e-5;
                // storing distance and id of the neighbour in target node struct
                blender.nDistances[nIdx] = tmp_dist;
                blender.nIds[nIdx] = n;
                nIdx++; 
            }   
        }  
    }
    __global__ 
    void sortNeighbourNodesKernel(dqBlender& blender){
        int index = blockIdx.x * blockDim.x + threadIdx.x;
        int stride = blockDim.x * gridDim.x;
        int size = blender.sourceMesh->verticesNum;
        for (int idx = index; idx < size; idx += stride){
            // Go through all neighbour points 
            for (int i = idx * size; i < idx * size + size - 1; i++) {
                // Store current distance and associated index
                float currDist  = blender.nDistances[i];
                int   currIndex = blender.nIds[i];
                // Shift values (and indexes) higher that the current distance to the right
                int j = i;
                float tmp_dist = 0;
                int tmp_index = 0;
                while (j > idx * size && blender.nDistances[j-1] > currDist) {

                    tmp_dist  = blender.nDistances[j-1];
                    tmp_index = blender.nIds[j-1];

                    blender.nDistances[j-1] = currDist;
                    blender.nIds[j-1]       = currIndex;

                    blender.nDistances[j] = tmp_dist;
                    blender.nIds[j]       = tmp_index;

                    --j;
                }
            }
        }  
    }
    void dqBlender::blendMesh(geometry::MeshSTD& dstMesh, 
                              math::dualQuat *DualQuat){

        // build KDtree for input mesh "*defGraphMesh"

        // find KNN for each vertex in mesh
        // update Euclidian distnaces between vertices and nodes
        int threads_per_block = 1024;
        int thread_blocks =(sourceMesh->verticesNum + threads_per_block - 1) / threads_per_block;
        // std::cout << "<<< updateNodeDistnacesKernel >>> threadBlocks: "<< thread_blocks << 
        //     ", threadPerBlock: " << threads_per_block << 
        //     ", sourceMesh->verticesNum: " << sourceMesh->verticesNum << 
        //     std::endl;
        updateNodeDistnacesKernel<<<thread_blocks, threads_per_block>>>(*this);
        cudaDeviceSynchronize();

        // for(int cnt=0; cnt< 8; cnt++){
        //     int idx = sourceMesh->verticesNum * 201 + cnt;
        //     printf("cnt: %d, dist: %f, ID: %d \n", cnt, nDistances[idx], nIds[idx]);
        // }

        // Sort vertices based on their distances
        // std::cout << "<<< sortNeighbourNodesKernel >>> threadBlocks: "<< thread_blocks << 
        //     ", threadPerBlock: " << threads_per_block << 
        //     std::endl;
        sortNeighbourNodesKernel<<<thread_blocks, threads_per_block>>>(*this);
        cudaDeviceSynchronize();
        
        // for(int cnt=0; cnt<8; cnt++){
        //     int idx = sourceMesh->verticesNum * 201 + cnt;
        //     printf("cnt: %d, dist: %f, ID: %d \n", cnt, nDistances[idx], nIds[idx]);
        // }

        // std::cout << "<<< updateNodeWeightsKernel >>> threadBlocks: "<< thread_blocks << 
        //     ", threadPerBlock: " << threads_per_block << 
        //     std::endl;
        updateNodeWeightsKernel<<<thread_blocks, threads_per_block>>>(*this);
        cudaDeviceSynchronize();

        // for(int cnt=0; cnt< 8; cnt++){
        //     int idx = sourceMesh->verticesNum * 201 + cnt;
        //     printf("cnt: %d, dist: %f, weight: %f, ID: %d \n", cnt, nDistances[idx], nWeights[idx], nIds[idx]);
        // }

        
        // Blend Mesh
        // std::cout << "<<< blendMesh >>> threadBlocks: " << thread_blocks << 
                    //  ", threadPerBlock: " << threads_per_block <<
                    //  ", VertexNum: " << sourceMesh->verticesNum <<
                    //   std::endl;
        blendMeshKernel<<<thread_blocks, threads_per_block>>>(*this, dstMesh, DualQuat);
        cudaDeviceSynchronize();
    }

} // namespace blender

} // namespace DynaMap
