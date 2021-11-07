/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
#include <knn.h>

namespace DynaMap{

    knn::~knn(){
        Free();
    }

    void knn::init(geometry::DefGraph& graph){

        defGraph = graph;
        // Display
        printf("KNN Config:\n");
        printf("- Number reference vertics : %d\n",   refVerticesNum);
        printf("- Number query vertics     : %d\n",   queryVerticesNum);
        printf("- Dimension of vertics     : %d\n",   dim);
        printf("- Number of neighbors      : %d\n",  k);

        // Sanity check
        if (refVerticesNum < k) {
            printf("Error: k value is larger that the number of reference points\n");
            return;
        }

        // allocation of reference and Query Vertices
        cudaMallocManaged(&refVertices, sizeof(geometry::PointXYZ) * refVerticesNum);
        cudaMallocManaged(&refNorms, sizeof(float) * refVerticesNum);

        cudaMallocManaged(&queryVertices, sizeof(geometry::PointXYZ) * queryVerticesNum);
        cudaMallocManaged(&queryNorms, sizeof(float) * queryVerticesNum);

        cudaMallocManaged(&distances, sizeof(float) * queryVerticesNum * refVerticesNum);
        cudaMallocManaged(&indices, sizeof(int) * queryVerticesNum * k);
    }

    void Free(){
        cudaDeviceSynchronize();
        cudaFree(refVertices);
        cudaFree(refNorms);
        cudaFree(queryVertices);
        cudaFree(queryNorms);
        cudaFree(distances);
        cudaFree(indices);
    }

    __global__
    void computeDistnacesKernel(const geometry::DefGraph& graph, float* ditances){
        int index = blockIdx.x * blockDim.x + threadIdx.x;
        int stride = blockDim.x * gridDim.x;
        int size = graph->nodeNum;
        for (int idx = index; idx < size; idx += stride){
            geometry::PointXYZ v = graph.nodes[idx].vertex.position;
            int nNum = graph.nodes[idx].nNum;
            for(int n = 0; n < graph.nodeNum; n++){
                int nId = graph.nodes[idx].nIds[n];
                geometry::PointXYZ vj = graph.nodes[nId].vertex.position;
                ditances[idx * nNum + n] = distance(vi, vj);
            }
        }  
    }

    __global__ 
    void sortPointsKernel(float * dist,
                                            int     dist_pitch,
                                            int *   index,
                                            int     index_pitch,
                                            int     width,
                                            int     height,
                                            int     k){

        // Column position
        unsigned int xIndex = blockIdx.x * blockDim.x + threadIdx.x;

        // Do nothing if we are out of bounds
        if (xIndex < width) {

            // Pointer shift
            float * p_dist  = dist  + xIndex;
            int *   p_index = index + xIndex;

            // Initialise the first index
            p_index[0] = 0;

            // Go through all points
            for (int i=1; i<height; ++i) {

                // Store current distance and associated index
                float curr_dist = p_dist[i*dist_pitch];
                int   curr_index  = i;

                // Skip the current value if its index is >= k and if it's higher the k-th slready sorted mallest value
                if (i >= k && curr_dist >= p_dist[(k-1)*dist_pitch]) {
                    continue;
                }

                // Shift values (and indexes) higher that the current distance to the right
                int j = min(i, k-1);
                while (j > 0 && p_dist[(j-1)*dist_pitch] > curr_dist) {
                    p_dist[j*dist_pitch]   = p_dist[(j-1)*dist_pitch];
                    p_index[j*index_pitch] = p_index[(j-1)*index_pitch];
                    --j;
                }

                // Write the current distance and index at their position
                p_dist[j*dist_pitch]   = curr_dist;
                p_index[j*index_pitch] = curr_index; 
            }
        }
    }

    __global__ 
    void computeSquaredNormKernel(geometry::PointXYZ* vertices, unsigned int verticesNum, float* norm){
        int index = blockIdx.x * blockDim.x + threadIdx.x;
        int stride = blockDim.x * gridDim.x;
        int size = verticesNum;
        for (int idx = index; idx < size; idx += stride){
            norm[idx] = powf(vertices[idx]->position.x + vertices[idx]->position.y + vertices[idx]->position.z, 2);
        }   
    }

    __global__ 
    void addReferencePointsNormKernel(float * array, int width, int pitch, int height, float * norm){
        unsigned int tx = threadIdx.x;
        unsigned int ty = threadIdx.y;
        unsigned int xIndex = blockIdx.x * blockDim.x + tx;
        unsigned int yIndex = blockIdx.y * blockDim.y + ty;
        __shared__ float shared_vec[16];
        if (tx==0 && yIndex<height)
            shared_vec[ty] = norm[yIndex];
        __syncthreads();
        if (xIndex<width && yIndex<height)
            array[yIndex*pitch+xIndex] += shared_vec[ty];
    }

    __global__ 
    void addQueryPointsNormKernel(float * array, int width, int pitch, int k, float * norm){
        unsigned int xIndex = blockIdx.x * blockDim.x + threadIdx.x;
        unsigned int yIndex = blockIdx.y * blockDim.y + threadIdx.y;
        if (xIndex<width && yIndex<k)
            array[yIndex*pitch + xIndex] = sqrt(array[yIndex*pitch + xIndex] + norm[xIndex]);
    }

    void knn::cuBlasKNN(float *       knn_dist,
                        int *         knn_index) {
        
        int threads_per_block = 512;
        // Computation of Euclidian distances
        int thread_blocks =(queryVerticesNum * refVerticesNum + threads_per_block - 1) / threads_per_block;
        std::cout << "<<< sortPointsKernel >>> threadBlocks: "<< thread_blocks << 
                     ", threadPerBlock: " << threads_per_block << 
                      std::endl;
        computeDistnacesKernel<<<thread_blocks, threads_per_block>>>(distances, indices, queryVerticesNum, refVerticesNum, k);
        cudaDeviceSynchronize();

        // Sort vertices based on their distances
        int thread_blocks =(queryVerticesNum + threads_per_block - 1) / threads_per_block;
        std::cout << "<<< sortPointsKernel >>> threadBlocks: "<< thread_blocks << 
                     ", threadPerBlock: " << threads_per_block << 
                      std::endl;
        sortPointsKernel<<<thread_blocks, threads_per_block>>>(distances, indices, queryVerticesNum, refVerticesNum, k);
        cudaDeviceSynchronize();
    }
}   //end namespace DynaMap






    // knn::cuBlasVectorKNN(void){
    //     int threads_per_block = 512;
    //     // Compute the squared norm of the reference points
    //     int thread_blocks =(refVerticesNum + threads_per_block - 1) / threads_per_block;
    //     std::cout << "<<< computeSquaredNormKernel f >>> threadBlocks: "<< thread_blocks << 
    //                  ", threadPerBlock: " << threads_per_block << 
    //                   std::endl;
    //     computeSquaredNormKernel<<<thread_blocks, threads_per_block>>>(refVertices, refVerticesNum, refNorms);
    //     cudaDeviceSynchronize();

    //     // Compute the squared norm of the query points
    //     int thread_blocks =(queryVerticesNum + threads_per_block - 1) / threads_per_block;
    //     std::cout << "<<< computeSquaredNormKernel q >>> threadBlocks: "<< thread_blocks << 
    //                  ", threadPerBlock: " << threads_per_block << 
    //                   std::endl;
    //     computeSquaredNormKernel<<<thread_blocks, threads_per_block>>>(queryVertices, queryVerticesNum, queryNorms);
    //     cudaDeviceSynchronize();

    //     // Computation of query*transpose(reference)
    //     // Multiply the arrays A and B on GPU and save the result in C
    //     // Calculate: C = α op ( A ) * op ( B ) + β C
    //     // (m X k) * (k X n) = (m X n)
    //     int m = ??? 
    //     int n = ???
    //     int k = ???
    //     int lda=m,ldb=k,ldc=m;
    //     const float alf = -2.0;
    //     const float bet = 0.0;
    //     const float *alpha = &alf;
    //     const float *beta = &bet;
    //     // Create a handle for CUBLAS
    //     cublasHandle_t handle;
    //     cublasCreate(&handle);
    //     // Do the actual multiplication
    //     cublasSgemm(handle, CUBLAS_OP_N, CUBLAS_OP_T, m, n, k, alpha, queryVertices, lda, refVertices, ldb, beta, C, ldc);
    //     // Destroy the handle
    //     cublasDestroy(handle);

    //     // Add reference points norm
    //     int thread_blocks =(refVerticesNum + threads_per_block - 1) / threads_per_block;
    //     std::cout << "<<< addReferencePointsNormKernel >>> threadBlocks: "<< thread_blocks << 
    //                  ", threadPerBlock: " << threads_per_block << 
    //                   std::endl;
    //     addReferencePointsNormKernel<<<thread_blocks, threads_per_block>>>(distances, queryVerticesNum, refVerticesNum, refNorms);

    //     // Sort each column
    //     int thread_blocks =(queryVerticesNum + threads_per_block - 1) / threads_per_block;
    //     std::cout << "<<< sortPointsKernel >>> threadBlocks: "<< thread_blocks << 
    //                  ", threadPerBlock: " << threads_per_block << 
    //                   std::endl;
    //     sortPointsKernel<<<thread_blocks, threads_per_block>>>(distances, dist_pitch, indices, index_pitch, queryVerticesNum, refVerticesNum, k);
    //     cudaDeviceSynchronize();

    //     // Add query norm and compute the square root of the of the k first elements
    //     int thread_blocks =(queryVerticesNum + threads_per_block - 1) / threads_per_block;
    //     std::cout << "<<< addQueryPointsNormKernel >>> threadBlocks: "<< thread_blocks << 
    //                  ", threadPerBlock: " << threads_per_block << 
    //                   std::endl;
    //     addQueryPointsNormKernel<<<thread_blocks, threads_per_block>>>(distances, queryVerticesNum, dist_pitch, k, queryNorms);
    //     cudaDeviceSynchronize();
    //     return true;
    // }