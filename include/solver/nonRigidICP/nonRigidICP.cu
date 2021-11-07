/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
//  optimizations...
// 1. jacobianT matrices can be generate in kernel at the smae time 
#include "nonRigidICP.h"

namespace DynaMap{
namespace solver{

    nonRigidICP::nonRigidICP(void){}
    nonRigidICP::~nonRigidICP(void){
        Free();
    }
    void nonRigidICP::init(geometry::defGraph &warpField, 
                           Properties &_Prop,
                           int problemSize){
                            
        alpha = 0;
        beta = 0;

        n = 0;
        m = 0;
        k = 0;   
        
        prevError = 10;
        Mdata = problemSize;
        Prop = _Prop;
        alpha = 0;
        beta = 0;
        n = 0;
        m = 0;
        k = 0; 

        // std::cout << "Solver init.... " << std::endl;
        
    }
    void nonRigidICP::Free(void){   

        cudaDeviceSynchronize();
        cudaFree(cloud->points);
        cudaFree(cloud->normals);
        cudaFree(cloud);
    }
    /*************************************************************************************************/
    __device__
    math::dualQuat nonRigidICP::getPDQGradient(math::dualQuat& dq, int paramID){
        // get dq of neighbour with ID of neighbourNum and extract its SE3 from
        // get Euler Angles from dq
        math::EulerAngles euAngles = dq.getEulerAngles();
        // get translation vector from dq
        float3 trans = dq.getTranslation();
        // change i-th (paramID) in +EPSILLON value
        switch(paramID){
            case 0:{
                // roll -> x
                euAngles.roll += EPSILLON; 
                break;
            }case 1:{
                // pitch -> y
                euAngles.pitch += EPSILLON; 
                break;
            }case 2:{
                // yaw -> z
                euAngles.yaw += EPSILLON; 
                break;
            }case 3:{
                // trans -> x
                trans.x += EPSILLON;  
                break;
            }case 4:{
                // trans -> y
                trans.y += EPSILLON;  
                break;
            }case 5:{
                // trans -> z
                trans.z += EPSILLON;  
                break;
            }
        }
        // get dq form again
        math::dualQuat pDQ(euAngles, trans);
        return pDQ;
    }
    __device__
    math::dualQuat nonRigidICP::getNDQGradient(math::dualQuat& dq, int paramID){
        // get Euler Angles from dq
        math::EulerAngles euAngles = dq.getEulerAngles();
        // get translation vector from dq
        float3 trans = dq.getTranslation();
        // change i-th (paramID) in +EPSILLON value
        switch(paramID){
            case 0:{
                // roll -> x
                euAngles.roll -= EPSILLON; 
                break;
            }case 1:{
                // pitch -> y
                euAngles.pitch -= EPSILLON; 
                break;
            }case 2:{
                // yaw -> z
                euAngles.yaw -= EPSILLON; 
                break;
            }case 3:{
                // trans -> x
                trans.x -= EPSILLON;  
                break;
            }case 4:{
                // trans -> y
                trans.y -= EPSILLON;  
                break;
            }case 5:{
                // trans -> z
                trans.z -= EPSILLON;  
                break;
            }
        }
        // get dq form again
        math::dualQuat nDQ(euAngles, trans);
        return nDQ;
    }
    __device__
    void nonRigidICP::validate(float& variable){
        if(isnan(abs(variable)) || isinf(abs(variable)) || abs(variable) > 1e2) variable = 0.0f;
    }
    __device__
    float nonRigidICP::getDQGradient(math::dualQuat *subWarpField,
                                     float* subWarpFiledWeights,
                                     int* subWarpFiledIDs,
                                     blender::dqBlender &blender,
                                     int nodeIndex,
                                     int ObsID,
                                     int paramID,
                                     float3 normalAtSrc,
                                     float3 vertexPose,
                                     float residual){
        // for(int j=0; j< KNN; j++){
        //     printf("id: %d, %f \n", j, subWarpFiledWeights[j]);
        // }           
        float nGradient = 0.0f;                    
        math::dualQuat backupDQ = subWarpField[nodeIndex];
        // get dq of neighbour with ID of neighbourNum and extract its SE3 from
        math::EulerAngles euAngles = backupDQ.getEulerAngles();
        // get translation vector from dq
        float3 trans = backupDQ.getTranslation();
        // change i-th (paramID) in -EPSILLON value
        switch(paramID){
            case 0:{
                // roll -> x
                euAngles.roll -= EPSILLON; 
                validate(euAngles.roll);
                break;
            }case 1:{
                // pitch -> y
                euAngles.pitch -= EPSILLON; 
                validate(euAngles.pitch);
                break;
            }case 2:{
                // yaw -> z
                euAngles.yaw -= EPSILLON; 
                validate(euAngles.yaw);
                break;
            }case 3:{
                // trans -> x
                // trans.x -= EPSILLON; 
                validate(trans.x); 
                break;
            }case 4:{
                // trans -> y
                trans.y -= EPSILLON;  
                validate(trans.y);
                break;
            }case 5:{
                // trans -> z
                trans.z -= EPSILLON;
                validate(trans.z);
                break;
            }
        }

        // printf("%f, %f , %f, %f, %f, %f \n", euAngles.roll, euAngles.pitch, euAngles.yaw, trans.x, trans.y, trans.z);
        // get dq form again
        subWarpField[nodeIndex] = math::dualQuat(euAngles, trans);
        // blend vertex with k neighbour 
        float3 blendedNDQ = blender.blendVertexPose(subWarpField, subWarpFiledWeights, subWarpFiledIDs, vertexPose, ObsID);
        // change i-th (paramID) in +EPSILLON value
        switch(paramID){
            case 0:{
                // roll -> x
                euAngles.roll += 2* EPSILLON; 
                validate(euAngles.roll);
                break;
            }case 1:{
                // pitch -> y
                euAngles.pitch += 2* EPSILLON;
                validate(euAngles.pitch); 
                break;
            }case 2:{
                // yaw -> z
                euAngles.yaw += 2* EPSILLON; 
                validate(euAngles.yaw);
                break;
            }case 3:{
                // trans -> x todo...
                // trans.x += 2* EPSILLON; 
                validate(trans.x); 
                break;
            }case 4:{
                // trans -> y
                trans.y += 2* EPSILLON;
                validate(trans.y);  
                break;
            }case 5:{
                // trans -> z
                trans.z += 2* EPSILLON; 
                validate(trans.z); 
                break;
            }
        }
        // get dq form again
        subWarpField[nodeIndex] = math::dualQuat(euAngles, trans);
        // blend vertex with k neighbour 
        float3 blendedPDQ = blender.blendVertexPose(subWarpField, subWarpFiledWeights, subWarpFiledIDs, vertexPose, ObsID);
        // get numerical derivative w.r.t the changed parameter
        nGradient = distance(blendedPDQ , blendedNDQ)/(2 * EPSILLON);
        // reload back the original value of warpfiled DQ
        subWarpField[nodeIndex] = backupDQ; 
        // if(nGradient != 0.0f)printf("%d, %f \n",pixelID, nGradient);
        // rerun the gradient w.r.t  i-th paramete (paramID) 
        // printf("%f, %f , %f, %f, %f, %f \n", euAngles.roll, euAngles.pitch, euAngles.yaw, trans.x, trans.y, trans.z);
        return nGradient;                       
    }
    __device__
    void nonRigidICP::updateDataJacobianBlock(Jacobian *_Jacob, 
                                              math::dualQuat *subWarpField,
                                              float* subWarpFiledWeights,
                                              int* subWarpFiledIDs,
                                              blender::dqBlender &blender,
                                              int ObsNum,
                                              int ObsID,
                                              int nodeID,
                                              int nodeIndex,
                                              float3 srcVertexPose,
                                              float3 srcVertexNormal,
                                              float3 dstVertexPose,
                                              float residual){
        float tmpGradient = 0;                                       
        int id = (nodeID * ObsNum * 6) + ObsID;
        // Filling Jacobian Blocks 
        for(int paramID = 0; paramID < 6; paramID++){
            int jacobIndex = paramID * ObsNum + id;
            tmpGradient = getDQGradient(subWarpField, 
                                        subWarpFiledWeights,
                                        subWarpFiledIDs,
                                        blender, 
                                        nodeIndex, 
                                        ObsID,
                                        paramID, 
                                        srcVertexNormal, 
                                        srcVertexPose,
                                        residual);
            if(isnan(tmpGradient) || isinf(tmpGradient) || fabs(residual) <= EPSILLON ) tmpGradient = 0.0f;
                _Jacob[jacobIndex] = tmpGradient;
        }
    }
    // builds Data term Jacobian on depth image 
    __global__ 
    void buildDataJacbianKernel(nonRigidICP &nonRigidSolver,
                                geometry::defGraph &warpField,
                                blender::dqBlender &blender,
                                float *targetdepth,
                                float *sourcedepth,
                                geometry::PointCloudXYZ &cloud,
                                rgbdSensor sensor, 
                                float4x4 cuPose,
                                Jacobian *Jacob,
                                Jacobian *residuals) {

        int index = blockIdx.x * blockDim.x + threadIdx.x;
        int size = sensor.rows * sensor.cols;
        int stride = blockDim.x * gridDim.x;

        // for each pixel in predicted image  pixelID = idx  
        // node Id is different that vertex ID in mesh
        for (int idx = index; idx < size; idx += stride) {
            // printf("%f, %f, %f, %i \n", sourcedepth[idx], targetdepth[idx], DEPTH_MIN, idx);
            if(sourcedepth[idx] > DEPTH_MAX || sourcedepth[idx] < DEPTH_MIN ||
               targetdepth[idx] > DEPTH_MAX || targetdepth[idx] < DEPTH_MIN )continue; 

            // project predicted image pixels to 3D-space
            float3 vc = getPoint3d(idx, targetdepth[idx], sensor); // todo... make sure transform is correct 
            float3 nc = cloud.normals[idx];    // todo ....update normals....
            // get the corresponding vertex in live-frame depth image 
            // project back 3D point from source pixel to target image 
            int2 vc2D = Project(vc, sensor);
            // get projection pixel index in target image 
            int index = vc2D.y * sensor.cols + vc2D.x;
            float3 vl = cuPose * getPoint3d(index, sourcedepth[index], sensor); // todo... make sure transform is correct 
            // update residuals
            residuals[idx] = DynaMap::dot(nc, (vc - vl));
            // printf("%f, %i \n", residuals[idx], idx);
            // fill jacobian blocks w.r.t K nodes affecting spceific pixels on the depth map
            // make a subWarp field for the pixel idx with  its near graph nodes
            math::dualQuat subWarpField[KNN];
            float subWarpFiledWeights[KNN] = {0.0f};
            int subWarpFiledIDs[KNN]={0};
            for(int nodeIndex = 0; nodeIndex < KNN; nodeIndex++){
                // form J (update Ji block in each thread)
                int nodeID = 0;
                if(warpField.KDTREE){  
                    nodeID = idx * warpField.nNum + nodeIndex;
                }else{
                    nodeID = idx * warpField.nodeNum + nodeIndex;
                }
                subWarpField[nodeIndex] = warpField.nodes[nodeID].dq;
                subWarpFiledWeights[nodeIndex] = warpField.visibleNWeights[nodeID];
                subWarpFiledIDs[nodeIndex] = warpField.nodes[nodeID].id;
                // for(int j=0; j< KNN; j++){
                //     printf("KNN: %d, id: %d, w: %f \n", KNN, j, subWarpFiledWeights[j]);
                // }     
            }
            // for each neighbor node to projection of pixel idx-th estimate numerical gradient and fill Jacobian blocks
            for(int nodeIndex = 0; nodeIndex < KNN; nodeIndex++){
                int tmpID = 0;
                if(warpField.KDTREE){  
                    tmpID = idx * warpField.nNum + nodeIndex;
                }else{
                    tmpID = idx * warpField.nodeNum + nodeIndex;
                }
                int nodeID = warpField.visibleNodeIds[tmpID];
                // form J (update Ji block in each thread)
                nonRigidSolver.updateDataJacobianBlock(Jacob,                       // output Jaboian (in each call one block gets updated)
                                                       subWarpField,                // sub-WarpField (Deformation Graph)
                                                       subWarpFiledWeights,         // subWarp field weights
                                                       subWarpFiledIDs,             // subWarp field ids
                                                       blender,                     // current blending status
                                                       size,                        // Observations Num
                                                       idx,                         // pixel ID
                                                       nodeID,                      // target node
                                                       nodeIndex,                   // near graphNode ID (in query)
                                                       vc,                          // vertex of current pixel
                                                       nc,                          // normal at current vertex position
                                                       vl,                          // correspondig vertex in input image
                                                       residuals[idx]);             // corresponding vertex in live-frame
            }
        }
    }
    // builds Data term Jacobian on target mesh
    __global__ 
    void buildDataJacbianKernel(nonRigidICP &nonRigidSolver,
                                geometry::defGraph &warpField,
                                blender::dqBlender &blender,
                                geometry::MeshSTD &targetMesh, 
                                float4x4 cuPose,
                                Jacobian *Jacob,
                                Jacobian *residuals) {

        int index = blockIdx.x * blockDim.x + threadIdx.x;
        int stride = blockDim.x * gridDim.x;
        int size = targetMesh.verticesNum;

        // for each vertex target mesh
        // node Id is different that vertex ID in mesh
        for (int idx = index; idx < size; idx += stride) {
            
            // invoke 3D position and normal of vertex in source mesh
            float3 vc = warpField.defGraphMesh.vertices[idx].position;   // todo ....camera pose update ?....
            float3 nc = warpField.defGraphMesh.vertices[idx].normal;     // todo ....update normals ?....
            // get the corresponding vertex in target mesh
            // todo... need correspondance 
            // (for now as we now they are same) also ActiveNodesDistances and Ids can be used
            float3 vl = targetMesh.vertices[idx].position; 
            // update residuals
            residuals[idx] = DynaMap::dot(nc, (vc - vl));
            // pick K nearest neighbour dual-quaternions and weights from main graph
            math::dualQuat subWarpField[KNN];
            float subWarpFiledWeights[KNN] = {0.0f};
            int subWarpFiledIDs[KNN]={0};
            for(int nodeIndex = 0; nodeIndex < KNN; nodeIndex++){
                // form J (update Ji block in each thread)
                int nodeID = 0;
                if(warpField.KDTREE){  
                    nodeID = idx * warpField.nNum + nodeIndex;
                }else{
                    nodeID = idx * warpField.nodeNum + nodeIndex;
                }
                subWarpField[nodeIndex] = warpField.nodes[nodeID].dq;
                subWarpFiledWeights[nodeIndex] = warpField.visibleNWeights[nodeID];
                subWarpFiledIDs[nodeIndex] = warpField.nodes[nodeID].id;
            }
            // fill jacobian blocks w.r.t K nodes affecting spceific pixels on the depth map
            for(int nodeIndex = 0; nodeIndex < KNN; nodeIndex++){
                int tmpID = 0;
                if(warpField.KDTREE){  
                    tmpID = idx * warpField.nNum + nodeIndex;
                }else{
                    tmpID = idx * warpField.nodeNum + nodeIndex;
                }
                int nodeID = warpField.visibleNodeIds[tmpID];
                // form J (update Ji block in each thread)
                nonRigidSolver.updateDataJacobianBlock(Jacob,                   // output Jaboian (in each call one block gets updated)
                                                       subWarpField,            // sub-WarpField (Deformation Graph)
                                                       subWarpFiledWeights,
                                                       subWarpFiledIDs,
                                                       blender,                 // current blending status
                                                       targetMesh.verticesNum,  // Observations Num
                                                       idx,                     // Observation ID -> pixelID or vertexID
                                                       nodeID,
                                                       nodeIndex,               // near graphNode ID (in query)
                                                       vc,                      // vertex of current pixel
                                                       nc,                      // normal at current vertex position
                                                       vl,                      // corresponding vertex in live-frame
                                                       residuals[idx]);                     
            }
        }
    }
    /*************************************************************************************************/
    __device__
    void nonRigidICP::updateRegJacobianBlock(Jacobian *_Jacob, 
                                             geometry::defGraph &warpField,
                                             int nodeID){
        // gets position of current neighbor node 
        float3 di = warpField.nodes[nodeID].vertex.position;
        // get number of entier nodes in deformation graph
        int n = warpField.nodeNum;
        int id = (6 * 3 * n * nodeID) + 3 * nodeID;

        // Jacobian Blocks for main node 
        // First row
        _Jacob[0 * 3 * warpField.nodeNum + id] =  0.0f;
        _Jacob[1 * 3 * warpField.nodeNum + id] =  di.z;
        _Jacob[2 * 3 * warpField.nodeNum + id] = -di.y;
        _Jacob[3 * 3 * warpField.nodeNum + id] =  1.0f;
        _Jacob[4 * 3 * warpField.nodeNum + id] =  0.0f;
        _Jacob[5 * 3 * warpField.nodeNum + id] =  0.0f;
        
        // Second row
        _Jacob[0 * 3 * warpField.nodeNum + id + 1] = -di.z;
        _Jacob[1 * 3 * warpField.nodeNum + id + 1] =  0.0f;
        _Jacob[2 * 3 * warpField.nodeNum + id + 1] =  di.x;
        _Jacob[3 * 3 * warpField.nodeNum + id + 1] =  0.0f;
        _Jacob[4 * 3 * warpField.nodeNum + id + 1] =  1.0f; 
        _Jacob[5 * 3 * warpField.nodeNum + id + 1] =  0.0f;

        // Third row
        _Jacob[0 * 3 * warpField.nodeNum + id + 2] =  di.y;
        _Jacob[1 * 3 * warpField.nodeNum + id + 2] =  -di.x;
        _Jacob[2 * 3 * warpField.nodeNum + id + 2] =  0.0f;
        _Jacob[3 * 3 * warpField.nodeNum + id + 2] =  0.0f;
        _Jacob[4 * 3 * warpField.nodeNum + id + 2] =  0.0f;
        _Jacob[5 * 3 * warpField.nodeNum + id + 2] =  1.0f;

        for(int n = 0; n < KNN; n++){
            // gets the ID of the neighbor list for current node
            int neighborID = warpField.nodes[nodeID].nIds[n];
            // if(nodeID == 0)printf("NodeID: %d, nID: %d \n", nodeID, neighborID);
            // gets position of selected neighbor node of currect node
            float3 dj = warpField.nodes[neighborID].vertex.position;  // id of neighbour  ???!!!
            // printf("%d, %d, %d, %d, %d\n",(id + 0 * n + 0 + nodeID), (id + 0 * n + 0 + neighborID), neighborID, id, NodeNeighbor);
            int id = (6 * 3 * warpField.nodeNum * neighborID) + nodeID * 3 ;
            //********************************************************//
            // Jacobian Blocks for each node neighbour
            // First row
            _Jacob[0 * 3 * warpField.nodeNum + id] =  0;
            _Jacob[1 * 3 * warpField.nodeNum + id] =  dj.z;
            _Jacob[2 * 3 * warpField.nodeNum + id] = -dj.y;
            _Jacob[3 * 3 * warpField.nodeNum + id] =  1;
            _Jacob[4 * 3 * warpField.nodeNum + id] =  0;
            _Jacob[5 * 3 * warpField.nodeNum + id] =  0;
            
            // Second row
            _Jacob[0 * 3 * warpField.nodeNum + id + 1] = -dj.z;
            _Jacob[1 * 3 * warpField.nodeNum + id + 1] =  0;
            _Jacob[2 * 3 * warpField.nodeNum + id + 1] =  dj.x;
            _Jacob[3 * 3 * warpField.nodeNum + id + 1] =  0;
            _Jacob[4 * 3 * warpField.nodeNum + id + 1] =  1; 
            _Jacob[5 * 3 * warpField.nodeNum + id + 1] =  0;

            // Third row
            _Jacob[0 * 3 * warpField.nodeNum + id + 2] =  dj.y;
            _Jacob[1 * 3 * warpField.nodeNum + id + 2] =  -dj.x;
            _Jacob[2 * 3 * warpField.nodeNum + id + 2] =  0;
            _Jacob[3 * 3 * warpField.nodeNum + id + 2] =  0;
            _Jacob[4 * 3 * warpField.nodeNum + id + 2] =  0;
            _Jacob[5 * 3 * warpField.nodeNum + id + 2] =  1;
        }
    }
    // computes and returns Regularization residuals for each Node of derGraph specified with nodeID
    __device__
    float3 nonRigidICP::getRegResiduals(geometry::defGraph &warpField, int nodeID){  

        float3 result;   
        // vertex of neigbhour node j
        // float3 vi = warpField.nodes[nodeID].vertex.position;    
        // Transformation of target node i
        float4x4 Ti = warpField.nodes[nodeID].dq.getTransformation();
        for(int cnt = 0; cnt < KNN; cnt++){
            // gets the neigbhour id j of target node i
            int neighborID = warpField.nodes[nodeID].nIds[cnt];
            // vertex of neigbhour node j
            float3 vj = warpField.nodes[neighborID].vertex.position;
            // Transformation of neigbhour node j
            float4x4 Tj = warpField.nodes[neighborID].dq.getTransformation();
            // weight of neigbhour node j
            float wij = fmax(warpField.nodes[nodeID].nWeights[cnt], warpField.nodes[neighborID].nWeights[cnt]);
            // todo... Huber Penalty should be add too ...
            result += wij * make_float3((Ti * make_float4(vj, 1.0f)) -  (Tj * make_float4(vj, 1.0f)));
            // if(nodeID == 0)printf("%f, %f, %f \n", result.x, result.y, result.z);
        }
        return result;
    }
    // builds Regularization term Jacobian
    __global__ 
    void buildRegJacbianKernel(nonRigidICP &nonRigidSolver,
                               geometry::defGraph &warpField,
                               Jacobian *Jacob,
                               Jacobian *residuals) {

        int index = blockIdx.x * blockDim.x + threadIdx.x;
        int stride = blockDim.x * gridDim.x;
        int size = warpField.nodeNum;

        // for each node   nodeID = idx  
        // node Id is different that vertex ID in mesh
        for (int idx = index; idx < size; idx += stride) {
            nonRigidSolver.updateRegJacobianBlock(Jacob, warpField, idx);   // todo... needs to be checked
            float3 residual = nonRigidSolver.getRegResiduals(warpField, idx);
            residuals[3 * idx    ] = residual.x;
            residuals[3 * idx + 1] = residual.y;
            residuals[3 * idx + 2] = residual.z;
            // printf("Residuals ->> %d: res: %f\n",idx, residuals[idx]);
        }
    }
    __global__ 
    void initIdentityGPU(float *matrix, int numR, int numC, float scalar) {
        int index = blockIdx.x * blockDim.x + threadIdx.x;
        int stride = blockDim.x * gridDim.x;
        int size = numR * numC;

        for (int idx = index; idx < size; idx += stride) {
            int x = static_cast<float>(idx / numC);
            int y = static_cast<float>(idx -  numC * x);
            if(y < numR && x < numC) {
                if(x == y)
                    matrix[idx] = scalar;
                else
                    matrix[idx] = 0;
            }
        }
    } 
    /*************************************************************************************************/
    // build Jabocians and Non-Linear system and Solve LU factorization for graph to live-frame image
    void nonRigidICP::solve(geometry::defGraph &warpField, 
                            blender::dqBlender &blender,
                            pyramid &targetImage,
                            pyramid &sourceImage,
                            Eigen::Matrix4f pose, 
                            rgbdSensor sensor){
             
        float4x4 cuPose = float4x4(pose.data()).getTranspose();  
        /***************** 2D neighbour update ********************/
        #ifdef DEBUG_DefGraph
            warpField.writeDefGraphToFile("../logs/DefGraph.txt");
        #endif

        // todo ... update active graph nodes beforehand in gl_render
        // finds K nearest Active defGraph nodes on the visible scene and computes associated weights
        warpField.updateActiveNeighbourNodes(targetImage, sensor, cuPose);

        #ifdef DEBUG_DefGraph                        
            warpField.writeDefGraphToFile("../logs/DefGraph_activeNodes.txt", Mdata, 1); 
        #endif

        for(int pylvl = 2; pylvl < 3; pylvl++) {
            // int scale = Prop.lvlScale[pylvl];
            // initializing kernel papameters 
            for (iterationNum = 1 ; iterationNum < Prop.lvlIterationNum[pylvl]; iterationNum++) {

                float4x4 cuPose = float4x4(pose.data()).getTranspose();

                // sourceImage.getNormalsfromDepthImage(normals);
                cudaMallocManaged(&cloud, sizeof(geometry::PointCloudXYZ));
                cudaMallocManaged(&cloud->points, sizeof(geometry::PointXYZ) * Mdata);
                cudaMallocManaged(&cloud->normals, sizeof(geometry::NormalXYZ) * Mdata);

                targetImage.getPointCloudXYZ(*cloud, 1);
                targetImage.getNormalsfromVertices(*cloud);

                /******************** JD^T * JD ***************************/
                /************** rData -> data Residuals *******************/
                cudaMallocManaged(&dataJacob,   sizeof(Jacobian)     * warpField.nodeNum * 6 * Mdata);
                cudaMallocManaged(&rData,       sizeof(Jacobian)     * Mdata);
                cudaDeviceSynchronize();
                // Reseting Jacobin values
                cudaMemset(rData,      0, sizeof(Jacobian)     * Mdata);
                cudaMemset(dataJacob,  0, sizeof(Jacobian)     * warpField.nodeNum * 6 * Mdata);
                cudaDeviceSynchronize();

                #ifdef LOG_EN
                    std::cout << "* dataJ" << std::endl;
                    std::cout << "* rData" << std::endl;
                #endif

                int threads_per_block = 64;
                int thread_blocks =(Mdata + threads_per_block - 1) / threads_per_block;
                buildDataJacbianKernel<<<thread_blocks, threads_per_block>>>(*this,
                                                                             warpField,
                                                                             blender,
                                                                             targetImage.depth,
                                                                             sourceImage.depth,
                                                                             *cloud,
                                                                             targetImage.sensor, 
                                                                             cuPose,
                                                                             dataJacob,
                                                                             rData);
                cudaDeviceSynchronize();

                // write dataJacob into file for debug
                #ifndef DEBUG
                    m = Mdata;
                    n = 6 * warpField.nodeNum;
                    writeMatrixToTxt("../logs/dataJacob.txt", dataJacob, sizeof(Jacobian), m, n, 1, 0, LOG_MAX);
                #endif
                // write rData into file for debug
                #ifndef DEBUG
                    m = Mdata;
                    n = 1;
                    writeMatrixToTxt("../logs/rData.txt", rData, sizeof(Jacobian), m, n, 1, 0, LOG_MAX);
                #endif
                
                /******************** JR^T * JR ***************************/
                /************* rReg -> data Residuals *********************/

                cudaMallocManaged(&regJacob,    sizeof(Jacobian) * 3 * warpField.nodeNum * 6 * warpField.nodeNum);
                cudaMallocManaged(&rReg,        sizeof(Jacobian) * 3 * warpField.nodeNum);
                cudaDeviceSynchronize();
                // Reseting Jacobin values
                cudaMemset(rReg,      0, sizeof(Jacobian) * 3 * warpField.nodeNum);
                cudaMemset(regJacob,  0, sizeof(Jacobian) * 3 * warpField.nodeNum * 6 * warpField.nodeNum); 
                cudaDeviceSynchronize();

                #ifdef LOG_EN
                    std::cout << "* regJ" << std::endl;
                    std::cout << "* rReg" << std::endl;
                #endif

                threads_per_block = 64;
                thread_blocks =(warpField.nodeNum + threads_per_block - 1) / threads_per_block;
                // std::cout << "<<< buildRegJacbianKernel >>> threadBlocks: "<< thread_blocks << 
                //              ", threadPerBlock: " << threads_per_block << 
                //              ", NODE_NUM: " << warpField.nodeNum <<
                //              ", visibleNodesNum: " << warpField.visibleNodesNum <<
                //               std::endl; 
                // build Jacobian fro Regularization term in Error Function
                buildRegJacbianKernel<<<thread_blocks, threads_per_block>>>(*this,
                                                                            warpField,
                                                                            regJacob,
                                                                            rReg);
                cudaDeviceSynchronize();
                    
                // write regJacob into file for debug
                #ifdef DEBUG
                    m = 3 * warpField.nodeNum;
                    n = 6 * warpField.nodeNum;
                    writeMatrixToTxt("../logs/regJacob.txt", regJacob, sizeof(Jacobian), m, n, 1, 0, m);
                #endif
                // write rReg into file for debug
                #ifdef DEBUG
                    m = 3 * warpField.nodeNum;
                    n = 1;
                    writeMatrixToTxt("../logs/rReg.txt", rReg, sizeof(Jacobian), m, n, 1, 0, m);
                #endif

                #ifdef LOG_EN
                    std::cout << "build Linear System... " << std::endl;
                #endif
                buildLinearSystem(warpField);

                // updating Warpfiled 
                #ifdef LOG_EN
                    std::cout << "Update DG nodes ... " << std::endl;
                #endif
                warpField.updateNodesDQ(b);

                #ifdef LOG_EN
                    std::cout << "Error evaluation ... " << std::endl;
                #endif
                if(evaluateError(warpField, blender))break;

                // write result into file for debug
                #ifdef DEBUG
                    writeMatrixToTxt("../logs/result.txt", result, sizeof(Jacobian), 6 * warpField.nodeNum, 6 * warpField.nodeNum, 1, 0, 6 * warpField.nodeNum);
                #endif 

                cudaFree(JTJ);
                cudaFree(b);
                cudaFree(result);
            }
        }
    }
    // build Jabocians and Non-Linear system and Solve LU factorization for graph to live-frame Mesh
    void nonRigidICP::solve(geometry::defGraph &warpField,
                            blender::dqBlender &blender,
                            geometry::MeshSTD &targetMesh,
                            Eigen::Matrix4f pose){

        
        float4x4 cuPose = float4x4(pose.data()).getTranspose();
        /***************** 2D neighbour update ********************/
        #ifdef DEBUG_DefGraph
            warpField.writeDefGraphToFile("../logs/DefGraph.txt");
        #endif
        // todo ... update active graph nodes beforehand in gl_render
        // finds K nearest Active defGraph nodes on the visible scene and computes associated weights
        warpField.updateActiveNeighbourNodes(targetMesh, cuPose);
        
        #ifdef DEBUG_DefGraph
            warpField.writeDefGraphToFile("../logs/DefGraph_activeNodes.txt",targetMesh.verticesNum,1);
        #endif

        // maybe pyramid can be used too... 
        for(int pylvl = 2; pylvl < 3; pylvl++) {
            //int scale = Prop.lvlScale[pylvl];
            // place to add hierarcical levels of mesh (generating on place)
            // initializing kernel papameters 
            for (iterationNum = 1 ; iterationNum < Prop.lvlIterationNum[pylvl]; iterationNum++) {

                /******************** JD^T * JD ***************************/
                /************** rData -> data Residuals *******************/
                cudaMallocManaged(&rData,   sizeof(Jacobian)     * Mdata);
                cudaMallocManaged(&dataJacob,   sizeof(Jacobian)     * warpField.nodeNum * 6 * Mdata);
                cudaDeviceSynchronize();
                // Reseting Jacobin values
                cudaMemset(rData,      0, sizeof(Jacobian)     * Mdata);
                cudaMemset(dataJacob,  0, sizeof(Jacobian)     * warpField.nodeNum * 6 * Mdata);
                cudaDeviceSynchronize();

                #ifdef LOG_EN
                    std::cout << "* dataJ" << std::endl;
                    std::cout << "* rData" << std::endl;
                #endif

                int threads_per_block = 64;
                int thread_blocks =(Mdata + threads_per_block - 1) / threads_per_block;

                #ifdef LOG_EN
                    std::cout << "<<< buildDataJacbianKernel >>> threadBlocks: "<< thread_blocks << 
                                ", threadPerBlock: " << threads_per_block << 
                                ", NODE_NUM: " << warpField.nodeNum <<
                                ", visibleNodesNum: " << warpField.visibleNodesNum <<
                                std::endl; 
                #endif

                // build Jacobian for data term in Error Fucntion
                buildDataJacbianKernel<<<thread_blocks, threads_per_block>>>(*this,
                                                                             warpField,
                                                                             blender,
                                                                             targetMesh, 
                                                                             cuPose,
                                                                             dataJacob,
                                                                             rData);
                cudaDeviceSynchronize();
                
                // write dataJacob into file for debug
                #ifdef DEBUG
                    m = Mdata;
                    n = 6 * warpField.nodeNum;
                    writeMatrixToTxt("../logs/dataJacob.txt", dataJacob, sizeof(Jacobian), m, n, 1, 0, m);
                #endif
                // write rData into file for debug
                #ifdef DEBUG
                    m = Mdata;
                    n = 1;
                    writeMatrixToTxt("../logs/rData.txt", rData, sizeof(Jacobian), m, n, 1, 0, m);
                #endif
                /******************** JR^T * JR ***************************/
                /************* rReg -> data Residuals *********************/
                cudaMallocManaged(&regJacob,    sizeof(Jacobian) * 3 * warpField.nodeNum * 6 * warpField.nodeNum);
                cudaMallocManaged(&rReg,    sizeof(Jacobian) * 3 * warpField.nodeNum);
                cudaDeviceSynchronize(); 
                // Reseting Jacobin values
                cudaMemset(rReg,      0, sizeof(Jacobian) * 3 * warpField.nodeNum);
                cudaMemset(regJacob,  0, sizeof(Jacobian) * 3 * warpField.nodeNum * 6 * warpField.nodeNum); 
                cudaDeviceSynchronize();

                #ifdef LOG_EN
                    std::cout << "* regJ" << std::endl;
                    std::cout << "* rReg" << std::endl;
                #endif

                threads_per_block = 64;
                thread_blocks =(warpField.nodeNum + threads_per_block - 1) / threads_per_block;

                #ifdef LOG_EN
                    std::cout << "<<< buildRegJacbianKernel >>> threadBlocks: "<< thread_blocks << 
                                ", threadPerBlock: " << threads_per_block << 
                                ", NODE_NUM: " << warpField.nodeNum <<
                                ", visibleNodesNum: " << warpField.visibleNodesNum <<
                                std::endl; 
                #endif

                // build Jacobian fro Regularization term in Error Function
                buildRegJacbianKernel<<<thread_blocks, threads_per_block>>>(*this,
                                                                            warpField,
                                                                            regJacob,
                                                                            rReg);
                cudaDeviceSynchronize();

                // write regJacob into file for debug
                #ifdef DEBUG
                    m = 3 * warpField.nodeNum;
                    n = 6 * warpField.nodeNum;
                    writeMatrixToTxt("../logs/regJacob.txt", regJacob, sizeof(Jacobian), m, n, 1, 0, m);
                #endif
                // write rReg into file for debug
                #ifdef DEBUG
                    m = 3 * warpField.nodeNum;
                    n = 1;
                    writeMatrixToTxt("../logs/rReg.txt", rReg, sizeof(Jacobian), m, n, 1, 0, m);
                #endif

                #ifdef LOG_EN
                    std::cout << "build Linear System... " << std::endl;
                #endif
                buildLinearSystem(warpField);

                // updating Warpfiled 
                #ifdef LOG_EN
                    std::cout << "Update DG nodes ... " << std::endl;
                #endif
                warpField.updateNodesDQ(b);

                #ifdef LOG_EN
                    std::cout << "Error evaluation ... " << std::endl;
                #endif
                if(evaluateError(warpField, blender))break;

                // write result into file for debug
                #ifdef DEBUG
                    writeMatrixToTxt("../logs/result.txt", result, sizeof(Jacobian), 6 * warpField.nodeNum, 6 * warpField.nodeNum, 1, 0, 6 * warpField.nodeNum);
                #endif 

                cudaFree(JTJ);
                cudaFree(b);
                cudaFree(result);
            }
        }   
    }
    bool nonRigidICP::evaluateError(geometry::defGraph &warpField,
                                    blender::dqBlender &blender){
        bool loop_break = false;
        currError = 0.0f;
        for(int cnt = 0; cnt < warpField.nodeNum; cnt++){
            if(isnan(rData[cnt]) || isinf(rData[cnt]))continue;
            currError += powf(rData[cnt], 2);
        }
        if(LAMBDA > 1e-5){
            for(int cnt = 0; cnt < warpField.nodeNum * 3; cnt++){
                if(isnan(rReg[cnt]) || isinf(rData[cnt]))continue;
                currError += LAMBDA * powf(rReg[cnt], 2);
            }
        }
        
        float change = prevError - currError;
        std::cout << "iteration: " << iterationNum << ", Error: " << currError << ", change: " << change << std::endl;
        if(currError == 0.0f){
            std::cout << " ****** Break-out !!!! ****** "<< std::endl;
            loop_break = true;
        }else{
            if(change >= Prop.minIncrement){
                prevError = currError;
                blender.blendMesh(warpField.defGraphMesh, warpField);
                loop_break = false;
            }else if(change <= 0.0f){
                std::cout << " ****** Doesn't Converge !!!! ****** "<< std::endl;
                loop_break = true;
            }else{
                std::cout << " ICP done ..."<< std::endl;
                loop_break = true;
            }
        }
        return loop_break;
    }
    // build Jabocians and Non-Linear system and Solve LU factorization Common steps
    void nonRigidICP::buildLinearSystem(geometry::defGraph &warpField){

        /******************** JD^T * JD ***********************/

        cudaMallocManaged(&JdTJd,       sizeof(Jacobian) * 6 * warpField.nodeNum * 6 * warpField.nodeNum);
        cudaDeviceSynchronize();
        cudaMemset(JdTJd,      0, sizeof(Jacobian) * 6 * warpField.nodeNum * 6 * warpField.nodeNum);
        cudaDeviceSynchronize();

        #ifdef LOG_EN
            std::cout << "* JD^T * JD" << std::endl;
        #endif
        m = 6 * warpField.nodeNum;
        n = Mdata;
        cuBlasMatrixMulTrans(dataJacob, dataJacob, JdTJd, m, n, m);
        // write JdTJd into file for debug
        #ifdef DEBUG
            writeMatrixToTxt("../logs/JdTJd.txt", JdTJd, sizeof(Jacobian), m, m, 1, 0, LOG_MAX);
        #endif

        /******************** bData ***************************/
        // from b = J^T * r
        cudaMallocManaged(&bData,   sizeof(Jacobian) * 6 * warpField.nodeNum);
        cudaDeviceSynchronize();
        cudaMemset(bData, 0, sizeof(Jacobian) * 6 * warpField.nodeNum);
        cudaDeviceSynchronize();

        #ifdef LOG_EN
            std::cout << "* bData" << std::endl;
        #endif
        
        m = 6 * warpField.nodeNum;
        k = Mdata;
        n = 1;
        cuBlasMatrixMulTrans(dataJacob, rData, bData, m, k, n);
        // write bData into file for debug
        #ifdef DEBUG
            writeMatrixToTxt("../logs/bData.txt", bData, sizeof(Jacobian), m, n, 1, 0, LOG_MAX);
        #endif

        cudaFree(dataJacob);

        /********************  JR^T * JR ***********************/

        cudaMallocManaged(&JrTJr,       sizeof(Jacobian) * 6 * warpField.nodeNum * 6 * warpField.nodeNum);
        cudaDeviceSynchronize();
        cudaMemset(JrTJr,     0, sizeof(Jacobian) * 6 * warpField.nodeNum * 6 * warpField.nodeNum); 
        cudaDeviceSynchronize();

        #ifdef LOG_EN
            std::cout << "* JR^T * JR" << std::endl;
        #endif
        alpha = 1.0;
        beta = 0.0;
        m = 6 * warpField.nodeNum;
        n = 3 * warpField.nodeNum;
        cuBlasMatrixMulTrans(regJacob, regJacob, JrTJr, m, n, m);

        // write JrTJr into file for debug
        #ifdef DEBUG
            writeMatrixToTxt("../logs/JrTJr.txt", JrTJr, sizeof(Jacobian), m, m, 1, 0, LOG_MAX);
        #endif

        /******************** bReg ***************************/
        // from b = J^T * r

        cudaMallocManaged(&bReg,    sizeof(Jacobian) * 6 * warpField.nodeNum);
        cudaDeviceSynchronize();
        cudaMemset(bReg, 0, sizeof(Jacobian) * 6 * warpField.nodeNum);
        cudaDeviceSynchronize();

        #ifdef LOG_EN
            std::cout << "* bReg" << std::endl;
        #endif
        
        m = 6 * warpField.nodeNum;
        k = 3 * warpField.nodeNum;
        n = 1;
        cuBlasMatrixMulTrans(regJacob, rReg, bReg, m, k, n);

        // write bReg into file for debug
        #ifdef DEBUG
            writeMatrixToTxt("../logs/bReg.txt", bReg, sizeof(Jacobian), m, n, 1, 0, LOG_MAX);
        #endif

        cudaFree(regJacob);

        /******************** A ***************************/
        // A = JT * J
        #ifdef LOG_EN
            std::cout << "* JT * J" << std::endl;
        #endif

        cudaMallocManaged(&JTJ, sizeof(Jacobian) * 6 * warpField.nodeNum * 6 * warpField.nodeNum);
        cudaDeviceSynchronize();
        cudaMemset(JTJ, 0, sizeof(Jacobian) * 6 * warpField.nodeNum * 6 * warpField.nodeNum);
        cudaDeviceSynchronize();
       
        // J^T * J = JD^T * JD + LAMBDA * JReg^T * JReg
        alpha = 1.0f;
        beta = LAMBDA;
        m = 6 * warpField.nodeNum;
        cuBlasMatrixSum(alpha, JdTJd, beta, JrTJr, JTJ, m);

        // write JTJ into file for debug
        #ifdef DEBUG
            writeMatrixToTxt("../logs/JTJ.txt", JTJ, sizeof(Jacobian), m, m, 1, 0, LOG_MAX);
        #endif

        cudaFree(JdTJd);
        cudaFree(JrTJr);

        /******************** b ***************************/
        // b = bData + LAMBDA * bReg
        #ifdef LOG_EN
            std::cout << "* b" << std::endl;
        #endif

        cudaMallocManaged(&b,       sizeof(Jacobian) * 6 * warpField.nodeNum);
        cudaDeviceSynchronize();
        cudaMemset(b, 0, sizeof(Jacobian) * 6 * warpField.nodeNum);
        cudaDeviceSynchronize();

        alpha = -1.0f;
        beta = LAMBDA;
        m = 6 * warpField.nodeNum;
        n = 1;
        cuBlasVectorSum(alpha, bData, beta, bReg, b, m);
       
        // write b into file for debug
        #ifdef DEBUG
            writeMatrixToTxt("../logs/b.txt", b, sizeof(Jacobian), m, n, 1, 0, LOG_MAX);
        #endif

        cudaFree(bData);
        cudaFree(bReg);

        /****************Solve Non-Linear System***********/
        #ifdef LOG_EN
            std::cout << "Solve Linear system ... " << std::endl;
        #endif

        cudaMallocManaged(&result, sizeof(Jacobian) * 6 * warpField.nodeNum);
        cudaDeviceSynchronize();
        cudaMemset(result,  0, sizeof(Jacobian) * 6 * warpField.nodeNum); // todo... correct size
        cudaDeviceSynchronize();

        if(SOLVER_TYPE == 0){  
            std::cout << "Eigen Solver ..." << std::endl;      
            solveLinearSystem(warpField);
        }else if(SOLVER_TYPE == 1){
            std::cout << "cuSparce Solver ..." << std::endl; 
            solveSparceLinearSystem(warpField); 
        }else if(SOLVER_TYPE == 2){
            std::cout << "cuSolver Solver ..." << std::endl; 
            cuSolveLinearSystem(warpField);
        }
    }
    void nonRigidICP::cuSolveLinearSystem(geometry::defGraph &warpField){
        /******************** cuSolver  **************************/
        // create Solver handler        
        cusolverDnCreate(&cuSolverHandle);

        m = 6 * warpField.nodeNum;
        n = 6 * warpField.nodeNum;

        int Lwork = 0;
        cusolverDnSgetrf_bufferSize(cuSolverHandle, m, n, JTJ, m, &Lwork);
        std::cout << "* bufferSize ... , Lwork: " << Lwork << std::endl;
        // set up workspace
        float* d_workspace;
        int* d_ipiv, *d_info;
        int h_info = 0;
        int lda = m;
        cudaMalloc(&d_workspace, Lwork * sizeof(float));
        cudaMalloc(&d_ipiv, min(m,n) * sizeof(int));
        cudaMalloc(&d_info, sizeof(int));
        cudaDeviceSynchronize();
        // decomposition
        std::cout << "* LU Decomposition of A ... "<< std::endl;
        cuSolverStatus = cusolverDnSgetrf(cuSolverHandle, m, n, JTJ, lda, d_workspace, d_ipiv, d_info);

        cudaMemcpy(&h_info, d_info, sizeof(int), cudaMemcpyDeviceToHost);
        if(cuSolverStatus != CUSOLVER_STATUS_SUCCESS) {
            std::cerr<<"failed to LU, info = "<<h_info<<std::endl;
        } else {
            std::cerr<<"done LU, info = "<<h_info<<std::endl;
        }
        
        // solve
        int ldb = n;
        std::cout << "* Solving op(A)x = b " << std::endl;
        cuSolverStatus = cusolverDnSgetrs(cuSolverHandle, CUBLAS_OP_N, n, 1, JTJ, n, d_ipiv, b, ldb, d_info);

        cudaMemcpy(&h_info, d_info, sizeof(int), cudaMemcpyDeviceToHost);
        cudaDeviceSynchronize();
        if(cuSolverStatus != CUSOLVER_STATUS_SUCCESS) {
            std::cerr<<"failed to solve, info = "<<h_info<<std::endl;
        } else {
            std::cerr<<"solved, info = "<<h_info<<std::endl;
        }

        if (d_info  ) { cudaFree(d_info); }
        if (d_workspace) { cudaFree(d_workspace); }
        if (d_ipiv  ) { cudaFree(d_ipiv);}
        
        
        const float _alfa = 1.0f;
        const float _beta = 1.0f;
        m = 6 * warpField.nodeNum;
        n = 1;
        cudaMemset(result,  0, sizeof(Jacobian) * 6 * warpField.nodeNum);   // todo.... result value is not correct!!!
        cudaDeviceSynchronize();
        cuBlasVectorSum(_alfa, result, _beta, b, result, m);

    }
    void nonRigidICP::solveLinearSystem(geometry::defGraph &warpField){
        /******************** Eigen Chol. Decomposition**************************/
       
        m = 6 * warpField.nodeNum;
        n = 6 * warpField.nodeNum;

        Jacobian *h_JTJ = new Jacobian[m * n];
        Jacobian *h_JTr = new Jacobian[m * 1];

        memset(h_JTJ, 0, sizeof(Jacobian) * m * n);
        memset(h_JTr, 0, sizeof(Jacobian) * m * 1);
        
        cudaMemcpy(h_JTJ, JTJ, sizeof(Jacobian) * m * n, cudaMemcpyDeviceToHost);
        cudaMemcpy(h_JTr, b, sizeof(Jacobian) * m * 1, cudaMemcpyDeviceToHost);
        cudaDeviceSynchronize();

        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> A;
        A = (Eigen::Map<Eigen::Matrix<float, 6*NODE_NUM, 6*NODE_NUM> >(h_JTJ)).cast <double> ();

        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> _b;
        _b = (Eigen::Map<Eigen::Matrix<float, 6*NODE_NUM, 1> >(h_JTr)).cast <double> ();

        // double scaling = 1 / A.maxCoeff();
        // _b *= scaling;
        // A *= scaling;
        // float alfa = Prop.regularization *  A.maxCoeff();
        // A = A + alfa * Eigen::MatrixXd::Identity(6*NODE_NUM, 6*NODE_NUM) * 1/iterationNum;
        // std::cout << "Eigen Solve LTVL" << std::endl;
        Eigen::Matrix<double, 6*NODE_NUM, 1> increment = A.ldlt().solve(_b);

        Jacobian *h_increment = new Jacobian[m * 1];
        memset(h_increment, 0, sizeof(Jacobian) * m * 1);

        for(int cnt=0; cnt<m; cnt++){
            h_increment[cnt] = increment[cnt];
        }
        
        const float _alfa = 1.0f;
        const float _beta = 1.0f;
        m = 6 * warpField.nodeNum;
        n = 1;
        cudaMemset(result,  0, sizeof(Jacobian) * m * n);   // todo.... result value is not correct!!!
        cudaMemcpy(result, h_increment, sizeof(Jacobian) * m * n, cudaMemcpyHostToDevice);
        cudaDeviceSynchronize();
        // std::cout << "apply increments ... " << std::endl;
        cuBlasVectorSum(_alfa, result, _beta, b, result, m);
   
    }
    void nonRigidICP::solveSparceLinearSystem(geometry::defGraph &warpField){
        // // --- Initialize cuSPARSE
        cusparseHandle_t handle;    cusparseCreate(&handle);

        const int Nrows = 6 * warpField.nodeNum;                        // --- Number of rows
        const int Ncols = 6 * warpField.nodeNum;                        // --- Number of columns
        const int N = Nrows;

        float *identityMat;
        cudaMalloc(&identityMat, Nrows * Ncols * sizeof(float));
        cudaMemset(identityMat, 0, Nrows * Ncols * sizeof(float));
        cudaDeviceSynchronize();

        int threads_per_block = 64;
        int thread_blocks =(Nrows * Ncols + threads_per_block - 1) / threads_per_block;
        initIdentityGPU <<<thread_blocks, threads_per_block>>>(identityMat, Nrows, Ncols, Prop.regularization/iterationNum);
        cudaDeviceSynchronize();

        alpha = 1.0f;
        beta = 1.0f;
        cuBlasVectorSum(alpha, JTJ, beta, identityMat, JTJ, Ncols);
        cudaDeviceSynchronize();

        // --- Descriptor for sparse matrix A
        cusparseMatDescr_t descrA;      cusparseCreateMatDescr(&descrA);
        cusparseSetMatType(descrA, CUSPARSE_MATRIX_TYPE_GENERAL);
        cusparseSetMatIndexBase(descrA, CUSPARSE_INDEX_BASE_ZERO);

        int nnz = 0;                                // --- Number of nonzero elements in dense matrix
        const int lda = Nrows;                      // --- Leading dimension of dense matrix
        // --- Device side number of nonzero elements per row
        int *d_nnzPerVector;    
        cudaMalloc(&d_nnzPerVector, Nrows * sizeof(*d_nnzPerVector));
        cudaMemset(d_nnzPerVector, 0, Nrows * sizeof(*d_nnzPerVector));
        cusparseSnnz(handle, CUSPARSE_DIRECTION_ROW, Nrows, Ncols, descrA, JTJ, lda, d_nnzPerVector, &nnz);
        // --- Host side number of nonzero elements per row
        int *h_nnzPerVector = (int *)malloc(Nrows * sizeof(*h_nnzPerVector));
        cudaMemcpy(h_nnzPerVector, d_nnzPerVector, Nrows * sizeof(*h_nnzPerVector), cudaMemcpyDeviceToHost);

        // printf("Number of nonzero elements in dense matrix = %i\n\n", nnz);
        // for (int i = 0; i < Nrows; ++i) printf("Number of nonzero elements in row %i = %i \n", i, h_nnzPerVector[i]);
        // printf("\n");

        // // --- Device side dense matrix
        float *d_A;             cudaMalloc(&d_A, nnz * sizeof(*d_A));
        int *d_A_RowIndices;    cudaMalloc(&d_A_RowIndices, (Nrows + 1) * sizeof(*d_A_RowIndices));
        int *d_A_ColIndices;    cudaMalloc(&d_A_ColIndices, nnz * sizeof(*d_A_ColIndices));

        cusparseSdense2csr(handle, Nrows, Ncols, descrA, JTJ, lda, d_nnzPerVector, d_A, d_A_RowIndices, d_A_ColIndices);

        // --- Host side dense matrix
        float *h_A = (float *)malloc(nnz * sizeof(*h_A));
        int *h_A_RowIndices = (int *)malloc((Nrows + 1) * sizeof(*h_A_RowIndices));
        int *h_A_ColIndices = (int *)malloc(nnz * sizeof(*h_A_ColIndices));
        cudaMemcpy(h_A, d_A, nnz*sizeof(*h_A), cudaMemcpyDeviceToHost);
        cudaMemcpy(h_A_RowIndices, d_A_RowIndices, (Nrows + 1) * sizeof(*h_A_RowIndices), cudaMemcpyDeviceToHost);
        cudaMemcpy(h_A_ColIndices, d_A_ColIndices, nnz * sizeof(*h_A_ColIndices), cudaMemcpyDeviceToHost);

        // for (int i = 0; i < nnz; ++i) printf("A[%i] = %.0f ", i, h_A[i]); printf("\n");

        // for (int i = 0; i < (Nrows + 1); ++i) printf("h_A_RowIndices[%i] = %i \n", i, h_A_RowIndices[i]); printf("\n");

        // for (int i = 0; i < nnz; ++i) printf("h_A_ColIndices[%i] = %i \n", i, h_A_ColIndices[i]);

        // --- Allocating and defining dense host and device data vectors
        float *h_b = (float *)malloc(Nrows * sizeof(float));
        cudaMemcpy(h_b, b, Nrows * sizeof(float), cudaMemcpyDeviceToHost);

        // --- Allocating the host and device side result vector
        float *h_x = (float *)malloc(Ncols * sizeof(float));
        float *d_x;        cudaMalloc(&d_x, Ncols * sizeof(float));

        // --- CUDA solver initialization
        cusolverSpHandle_t solver_handle;
        cusolverSpCreate(&solver_handle);

        // --- Using Cholesky factorization
        int singularity;
        cusolverSpScsrlsvcholHost(solver_handle, N, nnz, descrA, h_A, h_A_RowIndices, h_A_ColIndices, h_b, 0.000001, 0, h_x, &singularity);
        cudaMemcpy(b, h_b, Nrows * sizeof(float), cudaMemcpyHostToDevice);
        // pr r (int i = 0; i < N; i++) printf("%f\n", h_x[i]);

        Jacobian *h_increment = new Jacobian[m * 1];
        memset(h_increment, 0, sizeof(Jacobian) * m * 1);

        const float _alfa = 1.0f;
        const float _beta = 1.0f;

        cudaMemset(result,  0, sizeof(Jacobian) * 6 * warpField.nodeNum);   // todo.... result value is not correct!!!
        cudaDeviceSynchronize();
        cuBlasVectorSum(_alfa, result, _beta, b, result, N);

        cudaFree(d_nnzPerVector);
        cudaFree(d_A);
        cudaFree(d_A_RowIndices);
        cudaFree(d_A_ColIndices);
        cudaFree(d_x);
        cudaFree(identityMat);

        delete[] h_nnzPerVector;
        delete[] h_A;
        delete[] h_A_RowIndices;
        delete[] h_A_ColIndices;
        delete[] h_b;
        delete[] h_x;
    }
    /*************************************************************************************************/
    // Multiply the arrays A and B on GPU and save the result in C
    // Calculate: C = α op ( A ) * op ( B ) + β C
    // (m X k) * (k X n) = (m X n)
    void nonRigidICP::cuBlasMatrixMul(const float *A, const float *B, float *C, const int m, const int k, const int n) {
        int lda = m, ldb = k, ldc = m;
        const float alf = 1;
        const float bet = 0;
        const float *alpha = &alf;
        const float *beta = &bet;

        // Create a handle for CUBLAS
        cublasHandle_t handle;
        cublasCreate(&handle);

        // Do the actual multiplication
        cublasSgemm(handle, CUBLAS_OP_N, CUBLAS_OP_N, m, n, k, alpha, A, lda, B, ldb, beta, C, ldc);

        // Destroy the handle
        cublasDestroy(handle);
    }
    // Multiply the arrays A and B on GPU and save the result in C
    // Calculate: C = α op ( A )T * op ( B ) + β C
    // (m X k) * (k X n) = (m X n)
    void nonRigidICP::cuBlasMatrixMulTrans(const float *A, const float *B, float *C, const int m, const int k, const int n) {
        int lda = k, ldb = k, ldc = m;
        const float alf = 1;
        const float bet = 0;
        const float *alpha = &alf;
        const float *beta = &bet;

        // Create a handle for CUBLAS
        cublasHandle_t handle;
        cublasCreate(&handle);

        // Do the actual multiplication
        cublasSgemm(handle, CUBLAS_OP_T, CUBLAS_OP_N, m, n, k, alpha, A, lda, B, ldb, beta, C, ldc);

        // Destroy the handle
        cublasDestroy(handle);
    }
    // Multiply the arrays A and B on GPU and save the result in C
    // Calculate: C = α op ( A ) + β op ( B )
    // (m X k) * (k X n) = (m X n)
    void nonRigidICP::cuBlasMatrixSum(const float alf, const float *A, const float bet, const float *B, float *C, const int m){
        int lda=m,ldb=m,ldc=m;
        int n = m;
        const float *alpha = &alf;
        const float *beta = &bet;

        // Create a handle for CUBLAS
        cublasHandle_t handle;
        cublasCreate(&handle);

        // Do the Transpose
        cublasSgeam(handle, CUBLAS_OP_N, CUBLAS_OP_N, m, n, alpha, A, lda, beta, B, ldb, C, ldc);
        
        // Destroy the handle
        cublasDestroy(handle);
    }
    // Multiply the arrays A and B on GPU and save the result in C
    // Calculate: C = α op ( A ) + β op ( B )
    // (m X k) * (k X n) = (m X n)
    void nonRigidICP::cuBlasMatrixSumTrans(const float alf, const float *A, const float bet, const float *B, float *C, const int m){
        int lda=m,ldb=m,ldc=m;
        int n = m;
        const float *alpha = &alf;
        const float *beta = &bet;

        // Create a handle for CUBLAS
        cublasHandle_t handle;
        cublasCreate(&handle);

        // Do the Transpose
        cublasSgeam(handle, CUBLAS_OP_T, CUBLAS_OP_T, m, n, alpha, A, lda, beta, B, ldb, C, ldc);
        
        // Destroy the handle
        cublasDestroy(handle);
    }
    // Sum Vector A and B on GPU and save the result in C
    // Calculate: C = α op ( A ) + β op ( B )
    // (m X k) * (k X n) = (m X n)
    void nonRigidICP::cuBlasVectorSum(const float alf, const float *A, const float bet, const float *B, float *C, const int m){
        int n =1;
        int lda=m,ldb=m,ldc=m;

        const float *alpha = &alf;
        const float *beta = &bet;

        // Create a handle for CUBLAS
        cublasHandle_t handle;
        cublasCreate(&handle);

        // Do the Transpose
        cublasSgeam(handle, CUBLAS_OP_N, CUBLAS_OP_N, m, n, alpha, A, lda, beta, B, ldb, C, ldc);
        
        // Destroy the handle
        cublasDestroy(handle);
    }
    // Returns the Transpose of A -> C , α =1, β =0, op(A)= AT
    // Calculate: C = α op ( A ) + β op ( B )
    // A(m * n) 
    void nonRigidICP::cuBlasMatrixTrans(const float *A, float *C, const int m, const int n){
        int lda=n,ldb=m,ldc=m;
        const float alf = 1;
        const float bet = 0;
        const float *alpha = &alf;
        const float *beta = &bet;

        // Create a handle for CUBLAS
        cublasHandle_t handle;
        cublasCreate(&handle);

        // Do the Transpose
        cublasSgeam(handle, CUBLAS_OP_T, CUBLAS_OP_N, m, n, alpha, A, lda, beta, A, ldb, C, ldc);

        // Destroy the handle
        cublasDestroy(handle);
    }
    /*************************************************************************************************/
    void nonRigidICP::writeMatrixToTxt(const char* fileName, 
                                        float *matrix,
                                        size_t size,
                                        int rows, int cols,
                                        int mode,
                                        int from,
                                        int to){

            
        float *h_matrix = new float[rows * cols];
        memset(h_matrix, 0, size * rows * cols);
        cudaMemcpy(h_matrix, matrix, 
                    size * rows * cols, 
                    cudaMemcpyDeviceToHost);
        cudaDeviceSynchronize();
           
        std::ofstream file(fileName,std::ios::ate);
        if (file.is_open()){
            file << fileName <<", rows: " << rows << ", cols: " << cols << std::endl;
            file << "{ " << std::endl;
            for (size_t i = from; i < to; i++){
                file << "[ " << i ;
                for (size_t j = 0; j < cols; j++){
                    if(mode == 0){        // row major
                        file << std::fixed << std::setprecision(6)
                        << " " << matrix[i * cols + j];
                    }else if(mode ==1){   // column major
                        file << std::fixed << std::setprecision(6)
                        << " " << matrix[i + rows * j];
                    }
                }
                file << " ]"<< std::endl;
            }
            file << " }"<< std::endl;
            file.close();
        }
        else std::cout << "Unable to open file";

        delete[] h_matrix;
    }
    /*************************************************************************************************/
}   // end namespace solver
}   // end namespace DynaMap