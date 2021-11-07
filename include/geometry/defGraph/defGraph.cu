/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
#include "defGraph.h"

namespace DynaMap{
namespace geometry{
    defGraph::defGraph(void){}
    defGraph::~defGraph(void){
        Free();
    }
    void defGraph::init(MeshSTD &srcMesh, int ObservationNum, int mode){

        nodeNum = NODE_NUM;
        activeNodesNum = (nodeNum > ObservationNum) ? ObservationNum : nodeNum;
        defGraphMesh = srcMesh;
        nNum = KNN;
        visibleNodesNum = activeNodesNum;

        initGraphNodes(srcMesh, mode);
        
        if(!KDTREE){
            cudaMallocManaged(&visibleNodeIds,    sizeof(int)   * ObservationNum * nodeNum);
            cudaMallocManaged(&visibleNWeights,   sizeof(float) * ObservationNum * nodeNum);
            cudaMallocManaged(&visibleNDistances, sizeof(float) * ObservationNum * nodeNum);
            cudaDeviceSynchronize();
        }else{
            cudaMallocManaged(&graphKDTree, sizeof(kdTree));
            cudaDeviceSynchronize();
            graphKDTree->init(nodeNum);

            // allocating Query nodes on Host memory
            struct kdNode *kdQuery_h;
            kdQuery_h      =(struct kdNode*) calloc(nodeNum, sizeof(struct kdNode));
            // loading KDtree Query nodes from Graph Nodes
            for(int n = 0; n < nodeNum; n++){
                kdQuery_h[n].id = n;
                kdQuery_h[n].x[0] = nodes[n].vertex.position.x;
                kdQuery_h[n].x[1] = nodes[n].vertex.position.y;
                kdQuery_h[n].x[2] = nodes[n].vertex.position.z;
                // std::cout << kdQuery[n].id << ", "<<kdQuery[n].x[0] << ", " << kdQuery[n].x[1] << ", " <<kdQuery[n].x[2] << std::endl;
            }
            // copy Query KDtree to Device memory
            cudaMemcpy(graphKDTree->kdQuery, kdQuery_h, sizeof(struct kdNode) * nodeNum, cudaMemcpyHostToDevice);
            cudaDeviceSynchronize();

            // allocating Root of KDTree on Host memory
            struct kdNode *kdRoot_h;
            kdRoot_h = (struct kdNode*) calloc(nodeNum, sizeof(struct kdNode));

            // build DKTree on Host memory
            kdRoot_h = graphKDTree->buildTree(kdQuery_h, nodeNum, 0, 3);

            // copy KDtree to Device memory
            cudaMemcpy(graphKDTree->kdRoot, kdRoot_h, sizeof(struct kdNode) * nodeNum, cudaMemcpyHostToDevice);
            cudaDeviceSynchronize();

            cudaMallocManaged(&visibleNodeIds,    sizeof(int)   * ObservationNum * nNum);
            cudaMallocManaged(&visibleNWeights,   sizeof(float) * ObservationNum * nNum);
            cudaMallocManaged(&visibleNDistances, sizeof(float) * ObservationNum * nNum);

            cudaDeviceSynchronize();
        }

        std::cout <<  "Graph nodeNum: " << nodeNum << ", KDtree: " << KDTREE << std::endl;
    }
    void Free(void){}
    // *******************************************************************
    void defGraph::initGraphNodes(MeshSTD &srcMesh, int mode){

        cudaMallocManaged(&nodes, sizeof(defGraphNode) * nodeNum);
        if(KDTREE == false){ 
            for(int cnt=0; cnt < nodeNum; cnt++){
                cudaMallocManaged(&nodes[cnt].nIds, sizeof(int) * nodeNum);
                cudaMallocManaged(&nodes[cnt].nWeights, sizeof(float) * nodeNum);
                cudaMallocManaged(&nodes[cnt].nDistances, sizeof(float) * nodeNum);
            }
        }else{ // todo..... KDTree...!!!
            /// issue.............
            for(int cnt=0; cnt < nodeNum; cnt++){
                cudaMallocManaged(&nodes[cnt].nIds, sizeof(int) * nodeNum);
                cudaMallocManaged(&nodes[cnt].nWeights, sizeof(float) * nodeNum);
                cudaMallocManaged(&nodes[cnt].nDistances, sizeof(float) * nodeNum);
            }
        }
        cudaDeviceSynchronize();
        //  mesh vertices to initialize graph nodes
        sampleDownMeshHost(srcMesh.verticesNum, mode);
        // initialize nodes dual-quaternions to identity
        for (size_t i = 0; i < nodeNum; i++){
            nodes[i].dq = math::dualQuat::identity();
        }
    }
    void defGraph::Free(void){
        cudaDeviceSynchronize();
        if(KDTREE){
            cudaFree(graphKDTree);   
        }

        for(int cnt=0; cnt < NODE_NUM; cnt++){
            cudaFree(nodes[cnt].nIds);
            cudaFree(nodes[cnt].nWeights);
            cudaFree(nodes[cnt].nDistances);
        }
        cudaFree(nodes);

        cudaFree(visibleNodeIds);
        cudaFree(visibleNWeights);
        cudaFree(visibleNDistances);
    }
    /*********************Sort and Wight nodes************************/
    __global__
    void updateActiveNodesWeightsKernel(defGraph &graph,
                                        int verticesNum){
        int index = blockIdx.x * blockDim.x + threadIdx.x;
        int stride = blockDim.x * gridDim.x;
        int size = verticesNum;

        for (int idx = index; idx < size; idx += stride){

            for(int w = 0; w < KNN; w++){
                if(graph.visibleNodeIds[idx] == -1)continue;
                int nidx = idx * graph.visibleNodesNum + w;
                float ref = dgw;
                // float ref = dgw * graph.visibleNDistances[idx * graph.visibleNodesNum];
                // supposed distance[0] contains leasts distance after sorting
                if(expWeight){
                    graph.visibleNWeights[nidx] = exp(-pow(graph.visibleNDistances[nidx],2) / pow(ref,2)); 
                }else{
                    graph.visibleNWeights[nidx] = graph.visibleNDistances[idx * graph.nodeNum] * dgw / graph.visibleNDistances[nidx]; 
                }
            }
            // if(idx == 1)
            // for(int cnt=0; cnt< graph.visibleNodesNum; cnt++){
            //     int nId =  idx * graph.visibleNodesNum + cnt; 
            //     printf("nIds: %d, dist: %f, weight: %f \n", graph.visibleNodeIds[nId], graph.visibleNDistances[nId], graph.visibleNWeights[nId]);
            // }
        }
    }
    __global__
    void sortActiveNodesKernel(defGraph &graph,
                               int verticesNum){
        int index = blockIdx.x * blockDim.x + threadIdx.x;
        int stride = blockDim.x * gridDim.x;
        int size = verticesNum;

        for (int idx = index; idx < size; idx += stride){
            // Go through all neighbour points 
            for (int n = 0; n < graph.visibleNodesNum -1; n++) {
                if(graph.visibleNodeIds[n] == -1)break;
                int nIdx = idx * graph.visibleNodesNum + n;
                // Store current distance and associated nIdx
                float currDist  = graph.visibleNDistances[nIdx];
                int   currIndex = graph.visibleNodeIds[nIdx];
                // Shift values (and indexes) higher
                int j = nIdx;
                float tmp_dist = 0;
                int tmp_index = 0;
                while (j > idx * graph.visibleNodesNum && graph.visibleNDistances[j-1] > currDist) {

                    tmp_dist  = graph.visibleNDistances[j-1];
                    tmp_index = graph.visibleNodeIds[j-1];

                    graph.visibleNDistances[j-1] = currDist;
                    graph.visibleNodeIds[j-1]    = currIndex;

                    graph.visibleNDistances[j] = tmp_dist;
                    graph.visibleNodeIds[j]    = tmp_index;

                    --j;
                }
            }
        }
    }
    /*********************Graph to target Mesh************************/
    __global__ 
    void updateActiveNodesDistnacesKernel(defGraph &graph,
                                          MeshSTD &targetMesh,
                                          float4x4 cuPose){
        int index = blockIdx.x * blockDim.x + threadIdx.x;
        int stride = blockDim.x * gridDim.x;
        int size = targetMesh.verticesNum;

        for (int idx = index; idx < size; idx += stride){
            // invoking target vertex 3D position from target mesh
            geometry::PointXYZ vi = targetMesh.vertices[idx].position;  // todo... does it need transformation ?
            int nIdx = idx * graph.visibleNodesNum;
            for(int n = 0; n < graph.visibleNodesNum; n++){
                // non-visible node indices are filled with -1
                if(graph.visibleNodeIds[n] == -1 || n == idx)continue;
                // invoking neighbour node j vertex position from degGraph
                geometry::PointXYZ vj = graph.nodes[n].vertex.position;
                // computing distance between target vertex vi and j-th neighbour node(joint) position 
                float tmp_dist = distance(vi, vj);
                // excluding absolute 0.0 to avoids nan and inf products 
                if(tmp_dist == 0.0) tmp_dist = 1e-5;
                // storing distance and id of the neighbour in target node struct
                graph.visibleNDistances[nIdx] = tmp_dist;
                graph.visibleNodeIds[nIdx] = n;
                // index of targeted vertex in the array
                nIdx++;
            }  
        }
    }
    __global__
    void updateNodeWeightsKDTreeKernel(defGraph& graph, kdTree &kdtree){
        int index = blockIdx.x * blockDim.x + threadIdx.x;
        int stride = blockDim.x * gridDim.x;
        int size = graph.nodeNum;

        for (int idx = index; idx < size; idx += stride){
            // make a copy of KDtree in thread registers...
            kdTree tmpkdtree = kdtree;
            // graph node to KDtree node type
            struct kdNode currNode = {idx, {graph.nodes[idx].vertex.position.x,
                                            graph.nodes[idx].vertex.position.y,
                                            graph.nodes[idx].vertex.position.z}};
            // // finding closest neighbors in KDtree structure
            tmpkdtree.findKNN(currNode);

            printf("searching for (%g, %g, %g)\n"
                    "found (%g, %g, %g) dist %g\n ID: %d, seen %d nodes\n",
                    currNode.x[0], currNode.x[1], currNode.x[2],
                    tmpkdtree.kdFound->x[0], tmpkdtree.kdFound->x[1], tmpkdtree.kdFound->x[2],
                    sqrt(tmpkdtree.kdDistnaces[0]), 
                    tmpkdtree.kdFound->id,
                    tmpkdtree.visited);
            
            // for(int w = 0; w < graph.nNum; w++){
            //     float ref = dgw * tmpkdtree.VisitedNodes[0].distance;
            //     // supposed distance[0] contains leasts distance after sorting
            //     graph.nodes[idx].nWeights[w] = exp(-pow(tmpkdtree.VisitedNodes[w].distance,2) / pow(ref,2));  
            // }
            // if(idx == 10)
            // for(int cnt=0; cnt< graph.nodeNum; cnt++){
            //     printf("dist: %f, ids: %d, W: %f\n", graph.nodes[idx].nDistances[cnt], graph.nodes[idx].nIds[cnt],graph.nodes[idx].nWeights[cnt]);
            // }
        }  
    }
    void defGraph::updateActiveNeighbourNodes(MeshSTD &targetMesh, float4x4 cuPose){
        //load active nodes
        visibleNodesNum = nodeNum;
        if(!KDTREE){
            // update Euclidian distnaces between vertices and nodes
            int threads_per_block = 1024;
            int thread_blocks =(targetMesh.verticesNum + threads_per_block - 1) / threads_per_block;
            // std::cout << "<<< updateActiveNodesDistnaces >>> threadBlocks: "<< thread_blocks << 
            //     ", threadPerBlock: " << threads_per_block << 
            //     ", visibleNodesNum: " << visibleNodesNum <<
            //     std::endl;
            updateActiveNodesDistnacesKernel<<<thread_blocks, threads_per_block>>>(*this, targetMesh, cuPose);
            cudaDeviceSynchronize();

            // for(int cnt=0; cnt< KNN; cnt++){
            //     printf("cnt: %d, dist: %f, ID: %d \n", cnt, visibleNDistances[cnt], visibleNodeIds[cnt]);
            // }
            
            // std::cout << "<<< sortActiveNodes >>> threadBlocks: "<< thread_blocks << 
            //     ", threadPerBlock: " << threads_per_block <<
            //     std::endl;
            sortActiveNodesKernel<<<thread_blocks, threads_per_block>>>(*this, targetMesh.verticesNum);
            cudaDeviceSynchronize();  

            // for(int cnt=0; cnt< KNN; cnt++){
            //     printf("cnt: %d, dist: %f, ID: %d \n", cnt, visibleNDistances[cnt], visibleNodeIds[cnt]);
            // }

            // std::cout << "<<< updateActiveNodesWeights >>> threadBlocks: "<< thread_blocks << 
            //     ", threadPerBlock: " << threads_per_block << 
            //     std::endl;
            updateActiveNodesWeightsKernel<<<thread_blocks, threads_per_block>>>(*this, targetMesh.verticesNum);
            cudaDeviceSynchronize();

            // for(int cnt=0; cnt< KNN; cnt++){
            //     printf("cnt: %d, dist: %f, weight: %f, ID: %d \n", cnt, visibleNDistances[cnt], visibleNWeights[cnt], visibleNodeIds[cnt]);
            // }
        }else{

            // build KDtree for input mesh "*defGraphMesh" -> is done init Function
            // int threads_per_block = 512;
            // int thread_blocks =(this->nodeNum + threads_per_block - 1) / threads_per_block;
            // std::cout << "<<< updateNodeWeightsKDTreeKernel >>> threadBlocks: "<< thread_blocks << 
            //     ", threadPerBlock: " << threads_per_block << 
            //     std::endl;
            // updateNodeWeightsKDTreeKernel <<< thread_blocks , threads_per_block >>>(*this, *graphKDTree);

            kdTree *kdtree_h = new kdTree;
            cudaMemcpy(kdtree_h, graphKDTree, sizeof(kdTree), cudaMemcpyDeviceToHost);
            cudaDeviceSynchronize();
            updateActiveNodesWeightsKDTree(*kdtree_h, targetMesh, cuPose);

            // for(int cnt = 0; cnt < targetMesh.verticesNum; cnt ++){
            //     for(int j =0; j<nNum; j++){
            //         int nidx = cnt * nNum + j;
            //         printf("id: %d, j:%d,  %d, %f, %f \n", cnt, j , visibleNodeIds[nidx], visibleNDistances[nidx], visibleNWeights[nidx]);
            //     }
            // }
        }
    }
    /*********************Graph to depth Image************************/
    __global__ 
    void updateActiveNodesDistnacesKernel(defGraph& graph,
                                          float* targetdepth,
                                          rgbdSensor sensor,
                                          float4x4 cuPose){
        int index = blockIdx.x * blockDim.x + threadIdx.x;
        int stride = blockDim.x * gridDim.x;
        int size = sensor.rows * sensor.cols;

        for (int idx = index; idx < size; idx += stride){
            // check for valid data in depth and normal from source image
            if (targetdepth[idx] < DEPTH_MIN || targetdepth[idx] > DEPTH_MAX) continue;   // todo...  hard-coded limit for depth!!!
            // invoking target pixel 3D position from depth image
            geometry::PointXYZ vi = getPoint3d(idx, targetdepth[idx], sensor);  // todo... does it need transformation???
            for(int n = 0; n < graph.visibleNodesNum; n++){
                // non-visible node indices are filled with -1
                if(graph.visibleNodeIds[n] == -1)break;
                // invoking neighbour node j vertex position from degGraph
                geometry::PointXYZ vj = graph.nodes[n].vertex.position;
                // computing distance between target vertex vi and j-th neighbour node(joint) position 
                float tmp_dist = distance(vi, vj);
                // excluding absolute 0.0 to avoids nan and inf products 
                if(tmp_dist == 0.0) tmp_dist = 1e-5;
                // // index of targeted vertex in the array
                int nIdx = idx * graph.visibleNodesNum + n;
                // storing distance and id of the neighbour in target node struct
                graph.visibleNDistances[nIdx] = tmp_dist;
                graph.visibleNodeIds[nIdx] = n;
            }  
            // if(idx == 0) {
            //     for(int cnt=0; cnt< graph.nodeNum; cnt++){
            //         printf("idx: %d, cnt: %d, dist: %f\n",idx, cnt, graph.visibleNDistances[idx * graph.nodeNum + cnt]);
            //     } 
            // }
        }
    }
    void defGraph::updateActiveNeighbourNodes(pyramid &targetImage, rgbdSensor sensor, float4x4 cuPose){
        //load active nodes
        visibleNodesNum = nodeNum;

        if(!KDTREE){
            // update Euclidian distnaces between vertices and nodes
            int threads_per_block = 1024;
            int thread_blocks =(targetImage.sensor.rows * targetImage.sensor.cols + threads_per_block - 1) / threads_per_block;
            // std::cout << "<<< updateActiveNodesDistnaces >>> threadBlocks: "<< thread_blocks << 
            //     ", threadPerBlock: " << threads_per_block << 
            //     ", visibleNodesNum: " << visibleNodesNum <<
            //     ", rows: " << targetImage.sensor.rows << 
            //     ", cols: " << targetImage.sensor.cols <<  
            //     std::endl;
            updateActiveNodesDistnacesKernel<<<thread_blocks, threads_per_block>>>(*this, targetImage.depth, targetImage.sensor, cuPose);
            cudaDeviceSynchronize();
            
            // std::cout << "<<< sortActiveNodes >>> threadBlocks: "<< thread_blocks << 
            //     ", threadPerBlock: " << threads_per_block <<
            //     ", rows: " << targetImage.sensor.rows << 
            //     ", cols: " << targetImage.sensor.cols << 
            //     std::endl;
            sortActiveNodesKernel<<<thread_blocks, threads_per_block>>>(*this, targetImage.sensor.rows * targetImage.sensor.cols);
            cudaDeviceSynchronize();  

            // std::cout << "<<< updateActiveNodesWeights >>> threadBlocks: "<< thread_blocks << 
            //     ", threadPerBlock: " << threads_per_block << 
            //     ", rows: " << targetImage.sensor.rows << 
            //     ", cols: " << targetImage.sensor.cols << 
            //     std::endl;
            updateActiveNodesWeightsKernel<<<thread_blocks, threads_per_block>>>(*this, targetImage.sensor.rows * targetImage.sensor.cols);
            cudaDeviceSynchronize();
        }else{

            // build KDtree for input mesh "*defGraphMesh" -> is done init Function
            // int threads_per_block = 512;
            // int thread_blocks =(this->nodeNum + threads_per_block - 1) / threads_per_block;
            // std::cout << "<<< updateNodeWeightsKDTreeKernel >>> threadBlocks: "<< thread_blocks << 
            //     ", threadPerBlock: " << threads_per_block << 
            //     std::endl;
            // updateNodeWeightsKDTreeKernel <<< thread_blocks , threads_per_block >>>(*this, *graphKDTree);

            kdTree *kdtree_h = new kdTree;
            cudaMemcpy(kdtree_h, graphKDTree, sizeof(kdTree), cudaMemcpyDeviceToHost);
            cudaDeviceSynchronize();
            updateActiveNodesWeightsKDTree(*kdtree_h, targetImage, sensor, cuPose);

            // int size = sensor.rows * sensor.cols;
            // for(int cnt = 0; cnt < size; cnt ++){
            //     for(int w =0; w < nNum; w++){
            //         int nidx = cnt * nNum + w;
            //         printf("id: %d, jw:%d,  %d, %f, %f \n", cnt, w , visibleNodeIds[nidx], visibleNDistances[nidx], visibleNWeights[nidx]);
            //     }
            // }
        }
    }
    /*******************In graph Connections**************************/
    __global__
    void updateNodeWeightsKernel(defGraph& graph){
        int index = blockIdx.x * blockDim.x + threadIdx.x;
        int stride = blockDim.x * gridDim.x;
        int size = graph.nodeNum;
        for (int idx = index; idx < size; idx += stride){
            // In case of using radius reach for the neighborhood, this parameter will show number of close nodes
            for(int w = 0; w < graph.nNum; w++){
                float ref = dgw;
                // float ref = dgw * graph.nodes[idx].nDistances[0];
                if(graph.nodes[idx].nDistances[w] == 0.0f)continue;
                // supposed distance[0] contains leasts distance after sorting
                if(expWeight){
                    graph.nodes[idx].nWeights[w] = exp(-pow(graph.nodes[idx].nDistances[w],2) / pow(ref,2));  
                }else{
                    graph.nodes[idx].nWeights[w] =  graph.nodes[idx].nDistances[0] * dgw / graph.nodes[idx].nDistances[w]; 
                }
            }
            // if(idx == 10)
            // for(int cnt=0; cnt< graph.nodeNum; cnt++){
            //     printf("dist: %f, ids: %d, W: %f\n", graph.nodes[idx].nDistances[cnt], graph.nodes[idx].nIds[cnt],graph.nodes[idx].nWeights[cnt]);
            // }
        }  
    }
    __global__
    void updateNodeDistnacesKernel(defGraph& graph){
        int index = blockIdx.x * blockDim.x + threadIdx.x;
        int stride = blockDim.x * gridDim.x;
        int size = graph.nodeNum;

        for (int idx = index; idx < size; idx += stride){
            // invoking target node vertex position from degGraph
            geometry::Vertex vi = graph.nodes[idx].vertex;
            int nIdx = 0;
            for(int n = 0; n < graph.nodeNum; n++){
                // shouldn't add node itself as a neighbour in neighbour list
                if(n == idx) continue;
                // invoking neighbour node j vertex position from degGraph
                geometry::Vertex vj = graph.nodes[n].vertex;
                // computing distance between target node vi and i-th neighbour vertex position 
                float tmp_dist = distance(vi.position, vj.position);
                // excluding absolute 0.0 to avoid nan and inf products 
                if(tmp_dist < 10e-5) tmp_dist = 10e-5;
                // storing distance and id of the neighbour in target node struct
                graph.nodes[idx].nDistances[nIdx] = tmp_dist;
                graph.nodes[idx].nIds[nIdx] = n;
                nIdx++;
            }   
        }  
    }
    __global__ 
    void sortNeighbourNodesKernel(defGraph& graph){
        int index = blockIdx.x * blockDim.x + threadIdx.x;
        int stride = blockDim.x * gridDim.x;
        int size = graph.nodeNum;
        for (int idx = index; idx < size; idx += stride){
            // Go through all neighbour points 
            for (int i = 1; i < graph.nodeNum -1; i++) {
                // Store current distance and associated index
                float currDist  = graph.nodes[idx].nDistances[i];
                int   currIndex = graph.nodes[idx].nIds[i];

                // Shift values (and indexes) higher that the current distance to the right
                int j = i;
                float tmp_dist = 0;
                int tmp_index = 0;
                while (j > 0 && graph.nodes[idx].nDistances[j-1] > currDist) {

                    tmp_dist  = graph.nodes[idx].nDistances[j-1];
                    tmp_index = graph.nodes[idx].nIds[j-1];

                    graph.nodes[idx].nDistances[j-1] = currDist;
                    graph.nodes[idx].nIds[j-1]       = currIndex;

                    graph.nodes[idx].nDistances[j] = tmp_dist;
                    graph.nodes[idx].nIds[j]       = tmp_index;

                    --j;
                }
            }
            // if(idx == 10)
            // for(int cnt=0; cnt< graph.nodeNum; cnt++){
            //     printf("dist: %f, ids: %d\n", graph.nodes[idx].nDistances[cnt], graph.nodes[idx].nIds[cnt]);
            // }
        }  
    }
    void defGraph::defGraphUpdateNodes(void){
        if(!KDTREE){
            // build KDtree for input mesh "*defGraphMesh"

            // find KNN for each vertex in mesh
            // update Euclidian distnaces between vertices and nodes
            int threads_per_block = 1024;
            int thread_blocks =(this->nodeNum + threads_per_block - 1) / threads_per_block;
            // std::cout << "<<< updateNodeDistnacesKernel >>> threadBlocks: "<< thread_blocks << 
            //     ", threadPerBlock: " << threads_per_block << 
            //     ", nodeNum: " << nodeNum << 
            //     std::endl;
            updateNodeDistnacesKernel<<<thread_blocks, threads_per_block>>>(*this);
            cudaDeviceSynchronize();

            // for(int cnt=0; cnt< KNN; cnt++){
            //     printf("cnt: %d, dist: %f, ID: %d \n", cnt, nodes[0].nDistances[cnt], nodes[0].nIds[cnt]);
            // }

            // Sort vertices based on their distances
            threads_per_block = 512;
            thread_blocks =(this->nodeNum + threads_per_block - 1) / threads_per_block;
            // std::cout << "<<< sortNeighbourNodesKernel >>> threadBlocks: "<< thread_blocks << 
            //     ", threadPerBlock: " << threads_per_block << 
            //     std::endl;
            sortNeighbourNodesKernel<<<thread_blocks, threads_per_block>>>(*this);
            cudaDeviceSynchronize();
            
            // for(int cnt=0; cnt< KNN; cnt++){
            //     printf("cnt: %d, dist: %f, ID: %d \n", cnt, nodes[0].nDistances[cnt], nodes[0].nIds[cnt]);
            // }

            threads_per_block = 512;
            thread_blocks =(this->nodeNum + threads_per_block - 1) / threads_per_block;
            // std::cout << "<<< updateNodeWeightsKernel >>> threadBlocks: "<< thread_blocks << 
            //     ", threadPerBlock: " << threads_per_block << 
            //     std::endl;
            updateNodeWeightsKernel<<<thread_blocks, threads_per_block>>>(*this);
            cudaDeviceSynchronize();

            // for(int cnt=0; cnt< KNN; cnt++){
            //     printf("cnt: %d, dist: %f, weight: %f, ID: %d \n", cnt, nodes[0].nDistances[cnt], nodes[0].nWeights[cnt], nodes[0].nIds[cnt]);
            // }
        }else{
            // Update GraphStructure with KDtree - beforehand call GraphKDtreeInit ...
            // build KDtree for input mesh "*defGraphMesh" -> is done init Function
            // int threads_per_block = 512;
            // int thread_blocks =(this->nodeNum + threads_per_block - 1) / threads_per_block;
            // std::cout << "<<< updateNodeWeightsKDTreeKernel >>> threadBlocks: "<< thread_blocks << 
            //     ", threadPerBlock: " << threads_per_block << 
            //     std::endl;
            // updateNodeWeightsKDTreeKernel <<< thread_blocks , threads_per_block >>>(*this, *graphKDTree);

            kdTree *kdtree_h = new kdTree;
            cudaMemcpy(kdtree_h, graphKDTree, sizeof(kdTree), cudaMemcpyDeviceToHost);
            cudaDeviceSynchronize();
            updateNodeWeightsKDTree(*kdtree_h);

            // for(int cnt=0; cnt< KNN; cnt++){
            //     printf("cnt: %d, Dist: %f, weight:%f, ID: %d \n", cnt, nodes[0].nDistances[cnt], nodes[0].nWeights[cnt], nodes[0].nIds[cnt]);
            // }
        }
    }
    /*****************************************************************/
    void defGraph::updateNodeWeightsKDTree(kdTree &kdtree){
        
        defGraphNode *tmpnodes =  new defGraphNode[NODE_NUM];
        // copy Query KDtree to Device memory
        cudaMemcpy(tmpnodes, this->nodes, sizeof(defGraphNode) * NODE_NUM, cudaMemcpyDeviceToHost);
        cudaDeviceSynchronize();
        for (int idx = 0; idx < nodeNum; idx ++){
            // graph node to KDtree node type
            struct kdNode currNode = {idx, {tmpnodes[idx].vertex.position.x,
                                            tmpnodes[idx].vertex.position.y,
                                            tmpnodes[idx].vertex.position.z}};
            // finding closest neighbors in KDtree structure
            kdtree.findKNN(currNode);
            if(kdtree.VisitedNodes[0].distance < 1e-5)kdtree.VisitedNodes[0].distance = 1e-5;

            for(int w = 0; w < nNum; w++){
                float ref = dgw;
                // float ref = NODE_NUM/dgw * kdtree.VisitedNodes[0].distance;
                // supposed distance[0] contains leasts distance after sorting
                if(kdtree.VisitedNodes[w].distance == 0.0f)continue;
                if(expWeight){
                    nodes[idx].nWeights[w] = exp(-pow(kdtree.VisitedNodes[w].distance, 2) / pow(ref,2));
                }else{
                    nodes[idx].nWeights[w] = kdtree.VisitedNodes[0].distance * dgw / kdtree.VisitedNodes[w].distance; 
                }
                nodes[idx].nDistances[w] =  kdtree.VisitedNodes[w].distance;
                nodes[idx].nIds[w] = kdtree.VisitedNodes[w].id;
            }
        }  
    }
    void defGraph::updateActiveNodesWeightsKDTree(kdTree &kdtree, MeshSTD &targetMesh, float4x4 cuPose){

        for (int idx = 0; idx < targetMesh.verticesNum; idx ++){
            // graph node to KDtree node type
            struct kdNode currNode = {idx, {targetMesh.vertices[idx].position.x,
                                            targetMesh.vertices[idx].position.y,
                                            targetMesh.vertices[idx].position.z}};
            // finding closest neighbors in KDtree structure
            kdtree.findKNN(currNode);
            
            for(int w = 0; w < nNum; w++){
                int nidx = idx * nNum + w;
                float ref = dgw;
                // float ref = dgw * kdtree.VisitedNodes[0].distance;
                if(expWeight){
                    visibleNWeights[nidx] = exp(-pow(kdtree.VisitedNodes[w].distance,2) / pow(ref,2));
                }else{
                    visibleNWeights[nidx] = kdtree.VisitedNodes[0].distance * dgw / kdtree.VisitedNodes[w].distance; 
                }                 
                visibleNDistances[nidx] = kdtree.VisitedNodes[w].distance;
                visibleNodeIds[nidx] = kdtree.VisitedNodes[w].id;
                // printf("id: %d, w:%d,  %d, %f, %f \n", idx, w , visibleNodeIds[nidx], visibleNDistances[nidx], visibleNWeights[nidx]);
            }
        }
    }
    void defGraph::updateActiveNodesWeightsKDTree(kdTree &kdtree, pyramid &targetImage, rgbdSensor sensor, float4x4 cuPose){
        int size = sensor.rows * sensor.cols;
        for (int idx = 0; idx < size; idx ++){
            // graph node to KDtree node type   // todo... check the range again !!!!
            if(targetImage.depth[idx] < DEPTH_MAX && targetImage.depth[idx] > DEPTH_MIN){
                geometry::PointXYZ vi = getPoint3d(idx, targetImage.depth[idx], sensor);
                struct kdNode currNode = {idx, {vi.x,
                                                vi.y,
                                                vi.z}};
                // finding closest neighbors in KDtree structure
                kdtree.findKNN(currNode);
                
                for(int w = 0; w < nNum; w++){
                    int nidx = idx * nNum + w;
                    float ref = dgw;
                    // float ref = dgw * kdtree.VisitedNodes[0].distance;
                    if(expWeight){
                        visibleNWeights[nidx] = exp(-pow(kdtree.VisitedNodes[w].distance,2) / pow(ref,2));
                    }else{
                        visibleNWeights[nidx] = kdtree.VisitedNodes[0].distance * dgw / kdtree.VisitedNodes[w].distance; 
                    }                     
                    visibleNDistances[nidx] = kdtree.VisitedNodes[w].distance;
                    visibleNodeIds[nidx] = kdtree.VisitedNodes[w].id;
                    // printf("id: %d, w:%d,  %d, %f, %f \n", idx, w , visibleNodeIds[nidx], visibleNDistances[nidx], visibleNWeights[nidx]);
                }
            }   
        }
    }
    /*****************************************************************/
    void  defGraph::randomNum(int randNums[], int elements, int range){
        for (int i = 0; i < elements; i++){
            bool same;
            do{
                same = false;
                randNums[i] = rand() % range;
                // Check if the newly generated number is a duplicate:
                for (int check = 0; check < i; check++){
                    if (randNums[i] == randNums[check]){
                        same = true;
                        break;
                    }
                }
            } while (same);
        }
    }
    void defGraph::sampleDownMeshDevice(void){
        float *randDevData,*hostData;
        int n = NODE_NUM;
        cudaMalloc((void **)&randDevData, n * sizeof(float));
        hostData = (float *)calloc(n, sizeof(float));
        /* initialize random seed: */
        curandGenerator_t gen;
        /* Create pseudo-random number generator */
        curandCreateGenerator(&gen, CURAND_RNG_PSEUDO_DEFAULT);
        /* Set seed */
        curandSetPseudoRandomGeneratorSeed(gen, 1234ULL);
        /* Generate n floats on device */
        curandGenerateUniform(gen, randDevData, n);
        /* Copy device memory to host */
        cudaMemcpy(hostData, randDevData, n * sizeof(float),cudaMemcpyDeviceToHost);
        // update node ids querry (subsampling origianl mesh to the max number of nodes "NODE_NUM")
        // take fisrt n nodes as deformation graph nodes
        for(int cnt=0; cnt < NODE_NUM; cnt++){
            int num = static_cast<int>(hostData[cnt] * this->defGraphMesh.verticesNum); 
            this->nodes[cnt].id = cnt;
            this->nodes[cnt].vertex = this->defGraphMesh.vertices[num];
            this->nodes[cnt].vertexID = num;
            // printf("%d: %d \n",cnt, num);
        }
        curandDestroyGenerator(gen);
    }
    void defGraph::sampleDownMeshHost(int obsNum, int mode){
        // update node ids querry (subsampling origianl mesh to the max number of nodes "NODE_NUM")
        // take fisrt n nodes as deformation graph nodes
        // image or random nodes assignment
        std::cout << "activeNodesNum: "<< activeNodesNum << ", mode: " << mode << std::endl; 
        if(mode == 0){
            //declare array
            int *nodeIds_rand;
            //random number generator
            srand(static_cast<int>(time(0)));

            nodeIds_rand = new int[activeNodesNum];
            randomNum(nodeIds_rand, activeNodesNum, obsNum);

            for(int cnt = 0; cnt < activeNodesNum; cnt++){
                this->nodes[cnt].id = cnt;
                this->nodes[cnt].vertex = this->defGraphMesh.vertices[nodeIds_rand[cnt]];
                this->nodes[cnt].vertexID = nodeIds_rand[cnt];
                // std::cout << "Random -> ID: " << this->nodes[cnt].id << ", VertexNum: " << this->nodes[cnt].vertexID << std::endl;
            }
        }else{
            // identical nodes as mesh vertices in order of mesh 
            for(int cnt=0; cnt < activeNodesNum; cnt++){
                this->nodes[cnt].id = cnt;
                this->nodes[cnt].vertex = this->defGraphMesh.vertices[cnt];
                this->nodes[cnt].vertexID = cnt;
                // std::cout << "Exact -> ID: " << this->nodes[cnt].id << ", VertexNum: " << this->nodes[cnt].vertexID << std::endl;
            } 
        }
    }
    /*****************************************************************/
    __global__
    void updateNodesDQKernel(defGraph& graph, float* dqVector){
        int index = blockIdx.x * blockDim.x + threadIdx.x;
        int stride = blockDim.x * gridDim.x;
        int size = graph.nodeNum;
        math::EulerAngles EA;
        for (int idx = index; idx < size; idx += stride){
            EA.roll  = dqVector[idx * 6 + 0];
            EA.pitch = dqVector[idx * 6 + 1];
            EA.yaw   = dqVector[idx * 6 + 2];
            graph.nodes[idx].dq = math::dualQuat(math::Quaternion(EA), make_float3(dqVector[idx * 6 + 3],
                                                                                   dqVector[idx * 6 + 4],
                                                                                   dqVector[idx * 6 + 5]));
            // printf("%f, %f, %f, %f, %f, %f \n", EA.roll, EA.pitch, EA.yaw ,dqVector[idx * 6 + 3], dqVector[idx * 6 + 4],dqVector[idx * 6 + 5]);
            
        } 
    }
    void defGraph::updateNodesDQ(float* dqVector){
        // update Euclidian distnaces between nodes
        int threads_per_block = 512;
        int thread_blocks =(this->nodeNum + threads_per_block - 1) / threads_per_block;
        // std::cout << "<<< updateNodesDQ >>> threadBlocks: "<< thread_blocks << 
        //    ", threadPerBlock: " << threads_per_block << 
        //    std::endl;
        updateNodesDQKernel<<<thread_blocks, threads_per_block>>>(*this, dqVector);
        cudaDeviceSynchronize(); 
    }
    void defGraph::writeDefGraphToFile(const char* fileName, int obsNum, int mode){
        defGraphNode *h_nodes = new defGraphNode[NODE_NUM];
        cudaMemcpy(h_nodes, this->nodes, 
                    sizeof(defGraphNode) * NODE_NUM, 
                    cudaMemcpyDeviceToHost);
        cudaDeviceSynchronize();
           
        std::ofstream file(fileName,std::ios::ate);
        if (file.is_open()){
            if(mode == 0){
                file << fileName <<", NODE_NUM: " << NODE_NUM << std::endl;
                for (size_t i = 0; i < nodeNum; i++){
                    file << "[ " << std::endl;
                    file << "id: " << h_nodes[i].id << std::endl;
                    file << "nNum: " << nNum << std::endl;
                    file << "nodeIds: " ;
                    for(int j = 0; j < nNum; j++){
                        file << h_nodes[i].nIds[j] << " ";
                    }
                    file << std::endl;
                    file << "VertexPose: " << h_nodes[i].vertex.position.x << " "
                                        << h_nodes[i].vertex.position.y << " "
                                        << h_nodes[i].vertex.position.z << " " << std::endl;
                    file << "VertexNormal: " << h_nodes[i].vertex.normal.x << " "
                                            << h_nodes[i].vertex.normal.y << " "
                                            << h_nodes[i].vertex.normal.z << " " << std::endl;
                    file << "VertexColor: " << h_nodes[i].vertex.color.x << " "
                                            << h_nodes[i].vertex.color.y << " "
                                            << h_nodes[i].vertex.color.z << " " << std::endl;
                    file << "dq: " << h_nodes[i].dq << std::endl;
                    file << "nDistances: " ;
                    for(int j = 0; j < nNum; j++){
                        file << h_nodes[i].nDistances[j] << " ";
                    }
                    file << std::endl;
                    file << "nWeights: " ;
                    for(int j = 0; j < nNum; j++){
                        file << h_nodes[i].nWeights[j] << " ";
                    }
                    file << std::endl;
                    
                    file << "]," << std::endl;
                }
            }else if(mode == 1){
                // for(int cnt = 0; cnt < obsNum; cnt ++){
                //     for(int j =0; j<nNum; j++){
                //         int nidx = cnt * nNum + j;
                //         printf("xcid: %d, j:%d,  %d, %f, %f \n", cnt, j , visibleNodeIds[nidx], visibleNDistances[nidx], visibleNWeights[nidx]);
                //     }
                // }
                file << fileName <<", ACTIVE_NODE_NUM: " << visibleNodesNum << ", ObsNum: " << obsNum<< std::endl;
                int i = 0;
                while(true){
                    int index = i * KNN;
                    file << "[ " << std::endl;
                    file << "pixelID/vertexID: " << i << std::endl;
                    file << "visibleNodeIds: " ;
                    for(int j = 0; j < nNum; j++){
                        file << this->visibleNodeIds[index + j] << " ";
                    }
                    file << std::endl;
                    file << "visibleNDistances: " ;
                    for(int j = 0; j < nNum; j++){
                        file << this->visibleNDistances[index + j] << " ";
                    }
                    file << std::endl;
                    file << "visibleNWeights: " ;
                    for(int j = 0; j < nNum; j++){
                        file << this->visibleNWeights[index + j] << " ";
                    }
                    file << std::endl;
                    file << "]," << std::endl;
                    i++;
                    if(i >= obsNum || i >= 500)break;
                }
            }
            file.close();
        }
        else std::cout << "Unable to open file";

        delete[] h_nodes;
    } 
} // namespace geometry
} // namespace DynaMap
