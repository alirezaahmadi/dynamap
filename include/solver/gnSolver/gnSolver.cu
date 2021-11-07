/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
#include "gnSolver.h"

namespace DynaMap{
namespace solver{

    void gnSolver::init(Properties  _OptProp, 
                         const rgbdSensor &sensor){
        OptProp = _OptProp;
        iterationNum = 1;

        tragetPyImage.init(sensor);
        sourcePyImage.init(sensor);

        cudaStatus = cudaSetDevice(0);
    }
    void gnSolver::Free(){}

    // builds up Non-linear system structure
    __global__ 
    void buildNLSKernel(gnSolver& solver,
                        uchar3 *target_rgb, float *target_depth,
                        uchar3 *source_rgb, float *source_depth,
                        rgbdSensor sensor, float4x4 last_transform,
                        mat6x6 *acc_A, mat6x1 *acc_b, 
                        geometry::PointCloudXYZ &targetCloud,
                        geometry::PointCloudXYZ &sourceCloud,
                        float scale,
                        float huberConstant) {
        
        int index = blockIdx.x * blockDim.x + threadIdx.x;
        int stride = blockDim.x * gridDim.x;
        int size = sensor.rows * sensor.cols;

        int u_offset = 0;
        int v_offset = 0;
        if(scale == 2){
            u_offset = 160;
            v_offset = 120;
        }else if(scale == 4){
            u_offset = 240;
            v_offset = 180;
        }else if(scale == 8){
            u_offset = 280;
            v_offset = 210;
        }else{
            u_offset = 0;
            v_offset = 0;
        }                  
        for (int idx = index; idx < size; idx += stride) {
            
            int v = static_cast<int>(idx / sensor.cols);
            int u = static_cast<int>(idx - sensor.cols * v);

            if( v >= 0 && u >= 0) {
                if(scale == 2) {
                    if(v > (v_offset + 240) || u > (u_offset + 360) || v < v_offset || u < u_offset)
                        continue;
                }else if(scale == 4){
                    if(v > (v_offset + 120) || u > (u_offset + 180) || v < v_offset || u < u_offset)
                        continue;
                }else if(scale == 8){
                    if(v > (v_offset + 60) || u > (u_offset + 80) || v < v_offset || u < u_offset)
                        continue;
                }
            }

            mat6x6 _A;
            mat6x1 _b;

            _A.setZero();
            _b.setZero();
            
            // get normal at source depth image
            float3 s_n = make_float3(sourceCloud.normals[idx].x, sourceCloud.normals[idx].y, sourceCloud.normals[idx].z);

            // check for valid data in depth and normal from source image
            if (source_depth[idx] < DEPTH_MIN || source_depth[idx] > DEPTH_MAX || 
                isnan(s_n.x) || isnan(s_n.y) || isnan(s_n.z) || s_n == make_float3(0.0f, 0.0f, 0.0f)) continue;   // todo...  hard-coded limit for depth!!!

            // project source pixel to 3D space and transform with latest camera pose
            float3 p = last_transform * getPoint3d(idx, source_depth[idx], sensor);  // make sure transform is correct 
            
            // project back 3D point from source pixel to target image 
            int2 p2D = Project(p, sensor);
            
            // get projection pixel index in target image 
            int index = p2D.y * sensor.cols + p2D.x;
            
            // get corresponding normal 
            float3 n = make_float3(targetCloud.normals[index].x, targetCloud.normals[index].y, targetCloud.normals[index].z);

            // check for valid data in depth and normal from target image
            if (target_depth[index] < DEPTH_MIN || target_depth[index] > DEPTH_MAX || isnan(n.x) || isnan(n.y) || isnan(n.z) || n == make_float3(0.0f, 0.0f, 0.0f)) continue;

            // dot product of normals on source and target image to get difference --> to reject
            float cosTheta = DynaMap::dot(n, s_n);
            
            // if back projection of source pixel into target image is outside of the boundaries or normal of correspondaces are so different -> reject
            // get correspondance point in target depth map
            float3 q = getPoint3d(index, target_depth[index], sensor);  // make sure transform is correct
            // printf("%d, %f, %f \n", idx, cosTheta, cos(M_PI/10));
            if (abs(cosTheta) > cos(M_PI/10)){

                // testing intersection point in source image and normal of corresponding point on target
                float3 c = cross(p,n);

                math::EulerAngles euAngles;
                // euAngles.transformationToEulerAngles(last_transform);
                float residual = DynaMap::dot((p - q),n) +  DynaMap::dot(last_transform.getTranslation(), n) +  DynaMap::dot(euAngles.toFloat3(), c);
                // float weight = solver.getHuberWeight(residual, huberConstant);
                // float weight = solver.getTukeyWeight(residual, huberConstant);
                // float weight = solver.getL1Weight(residual);
                float weight = 1.0f;

                mat6x1 cn;
                cn(0,0) = c.x; 
                cn(1,0) = c.y; 
                cn(2,0) = c.z;
                cn(3,0) = n.x; 
                cn(4,0) = n.y; 
                cn(5,0) = n.z;
                _A = cn * weight *(cn.getTranspose());
                
                _b(0,0) = weight * rowSum3(element((p - q),element(repmat3x1(cn(0,0)), n)));
                _b(1,0) = weight * rowSum3(element((p - q),element(repmat3x1(cn(1,0)), n)));
                _b(2,0) = weight * rowSum3(element((p - q),element(repmat3x1(cn(2,0)), n)));
                _b(3,0) = weight * rowSum3(element((p - q),element(repmat3x1(cn(3,0)), n)));
                _b(4,0) = weight * rowSum3(element((p - q),element(repmat3x1(cn(4,0)), n)));
                _b(5,0) = weight * rowSum3(element((p - q),element(repmat3x1(cn(5,0)), n)));

                float3 targetColor = make_float3(static_cast<float>(target_rgb[index].x)/255,
                                                static_cast<float>(target_rgb[index].y)/255,
                                                static_cast<float>(target_rgb[index].z)/255);
                float3 sourceColor = make_float3(static_cast<float>(source_rgb[idx].x)/255,
                                                static_cast<float>(source_rgb[idx].y)/255,
                                                static_cast<float>(source_rgb[idx].z)/255);

                mat3x6 d_position;
                d_position(0, 0) = 0;
                d_position(0, 1) = p.z;
                d_position(0, 2) = -p.y;
                d_position(0, 3) = 1;
                d_position(0, 4) = 0;
                d_position(0, 5) = 0;
                d_position(1, 0) = -p.z;
                d_position(1, 1) = 0;
                d_position(1, 2) = p.x;
                d_position(1, 3) = 0;
                d_position(1, 4) = 1;
                d_position(1, 5) = 0;
                d_position(2, 0) = p.y;
                d_position(2, 1) = -p.x;
                d_position(2, 2) = 0;
                d_position(2, 3) = 0;
                d_position(2, 4) = 0;
                d_position(2, 5) = 1;

                // mat2x6 jacobColor;
                // jacobColor(0, 0) = sensor.f.x / p.z ;
                // jacobColor(0, 1) = 0;
                // jacobColor(0, 2) = sensor.f.x * p.x / pow(p.z, 2);
                // jacobColor(0, 3) = sensor.f.x * p.x * p.y / pow(p.z, 2);
                // jacobColor(0, 4) = sensor.f.x * pow(p.x, 2) * pow(p.z, 2) / pow(p.z, 2);
                // jacobColor(0, 5) = sensor.f.x * p.y / p.z;
                // jacobColor(1, 0) = 0;
                // jacobColor(1, 1) = sensor.f.y / p.z;
                // jacobColor(1, 2) = sensor.f.y * p.y / pow(p.z, 2);
                // jacobColor(1, 3) = sensor.f.y * pow(p.y, 2) * pow(p.z, 2) / pow(p.z, 2);
                // jacobColor(1, 4) = sensor.f.y * p.x * p.y / pow(p.z, 2);
                // jacobColor(1, 5) = sensor.f.y * p.x / p.z;

                mat1x6 jacobColor;
                mat1x3 gradient_color;
                float color_weight = 0.0f;
                int step = 1;
                int up_index = p2D.y * sensor.cols + (p2D.x - step);
                gradient_color(0) = solver.ColorDifference(target_rgb[index], target_rgb[up_index]) / 2;
                int left_index = (p2D.y - step) * sensor.cols + p2D.x;
                gradient_color(1) = solver.ColorDifference(target_rgb[index], target_rgb[left_index]) / 2;
                gradient_color(2) = 0;

                jacobColor = gradient_color * d_position;
                _A = (1.0f - color_weight) * _A + color_weight * jacobColor.getTranspose() * jacobColor;
                _b = (1.0f - color_weight) * _b + color_weight * jacobColor.getTranspose() * (solver.Intensity(targetColor) - solver.Intensity(sourceColor));

                for (int j = 0; j < 36; j++) atomicAdd(&((*acc_A)(j)), _A(j));
                for (int j = 0; j < 6; j++)  atomicAdd(&((*acc_b)(j)), -_b(j));
            }
        }
    }
    Vector6d gnSolver::solveSparceLinearSystem(Eigen::Matrix<double, 6, 6>& _A, Vector6d& _b){

        // // --- Initialize cuSPARSE
        cusparseHandle_t handle;    cusparseCreate(&handle);

        const int Nrows = 6;                        // --- Number of rows
        const int Ncols = 6;                        // --- Number of columns
        const int N = Nrows;

        float *A; cudaMallocManaged(&A, Nrows * Ncols * sizeof(*A));
        float *b; cudaMallocManaged(&b, Nrows * sizeof(*b));

        Vector6d resultVec;
        resultVec << 0, 0, 0, 0, 0, 0;

        for (int r = 0; r < 6; r++) {
            for (int c = 0; c < 6; c++) {

                int index = r * 6 + c;
                // std::cout << "index: " << index << ", " << r << ", " << c << std::endl;
                A[index] = static_cast<float>(_A(r, c));

                if(index < 6){
                    b[index] = static_cast<float>(_b(index));
                }
            }
        }

        // --- Descriptor for sparse matrix A
        cusparseMatDescr_t descrA;      cusparseCreateMatDescr(&descrA);
        cusparseSetMatType(descrA, CUSPARSE_MATRIX_TYPE_GENERAL);
        cusparseSetMatIndexBase(descrA, CUSPARSE_INDEX_BASE_ZERO);

        int nnz = 0;                                // --- Number of nonzero elements in dense matrix
        const int lda = Nrows;                      // --- Leading dimension of dense matrix
        // --- Device side number of nonzero elements per row
        int *d_nnzPerVector;    cudaMalloc(&d_nnzPerVector, Nrows * sizeof(*d_nnzPerVector));
        cusparseSnnz(handle, CUSPARSE_DIRECTION_ROW, Nrows, Ncols, descrA, A, lda, d_nnzPerVector, &nnz);
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

        cusparseSdense2csr(handle, Nrows, Ncols, descrA, A, lda, d_nnzPerVector, d_A, d_A_RowIndices, d_A_ColIndices);
        // --- Host side dense matrix
        float *h_A = (float *)malloc(nnz * sizeof(*h_A));
        int *h_A_RowIndices = (int *)malloc((Nrows + 1) * sizeof(*h_A_RowIndices));
        int *h_A_ColIndices = (int *)malloc(nnz * sizeof(*h_A_ColIndices));
        cudaMemcpy(h_A, d_A, nnz*sizeof(*h_A), cudaMemcpyDeviceToHost);
        cudaMemcpy(h_A_RowIndices, d_A_RowIndices, (Nrows + 1) * sizeof(*h_A_RowIndices), cudaMemcpyDeviceToHost);
        cudaMemcpy(h_A_ColIndices, d_A_ColIndices, nnz * sizeof(*h_A_ColIndices), cudaMemcpyDeviceToHost);

        // for (int i = 0; i < nnz; ++i) printf("A[%i] = %.0f \n", i, h_A[i]); 

        // for (int i = 0; i < (Nrows + 1); ++i) printf("h_A_RowIndices[%i] = %i \n", i, h_A_RowIndices[i]); printf("\n");

        // for (int i = 0; i < nnz; ++i) printf("h_A_ColIndices[%i] = %i \n", i, h_A_ColIndices[i]);

        // --- Allocating and defining dense host and device data vectors
        float *h_b = (float *)malloc(Nrows * sizeof(float));
        cudaMemcpy(h_b, b, Nrows * sizeof(float), cudaMemcpyDeviceToHost);
        // --- Allocating the host and device side result vector
        float *h_x = (float *)malloc(Ncols * sizeof(float));
        float *d_x; cudaMalloc(&d_x, Ncols * sizeof(float));

        // --- CUDA solver initialization
        cusolverSpHandle_t solver_handle;
        cusolverSpCreate(&solver_handle);

        // --- Using Cholesky factorization
        int singularity;
        cusolverSpScsrlsvcholHost(solver_handle, N, nnz, descrA, h_A, h_A_RowIndices, h_A_ColIndices, h_b, 0.000001, 0, h_x, &singularity);
        cudaMemcpy(b, h_b, Nrows * sizeof(float), cudaMemcpyHostToDevice);
        
        // printf("Showing the results...\n");
        // for (int i = 0; i < N; i++) printf("%f\n", h_x[i]);

        cudaFree(d_nnzPerVector);
        cudaFree(d_A);
        cudaFree(d_A_RowIndices);
        cudaFree(d_A_ColIndices);
        cudaFree(d_x);

        for (int k = 0; k < 6; k++) {
            resultVec(k) = h_b[k];
        }

        delete[] h_nnzPerVector;
        delete[] h_A;
        delete[] h_A_RowIndices;
        delete[] h_A_ColIndices;
        delete[] h_b;
        delete[] h_x;

        return resultVec;
    }
    __host__ __device__
    float gnSolver::getHuberWeight(float error, float thershold){
        float smaller = 1;
        float greater = thershold/fabs(error);
        return fabs(error) > thershold ? greater : smaller;
    }
    __host__ __device__
    float gnSolver::getTukeyWeight(float error, float thershold){
        float smaller = pow((1 - pow(error/thershold,2)), 2);
        float greater = 0;
        return fabs(error) > thershold ? greater : smaller;
    }
    __host__ __device__
    float gnSolver::getL1Weight(float error){
        return 1/fabs(error);
    }
    void gnSolver::solve(Eigen::Matrix4f& pose, 
                         Vector6d& prev_increment_, 
                         rgbdImage &targetImage,
                         rgbdImage &sourceImage,
                         geometry::PointCloudXYZ &targetCloud,
                         geometry::PointCloudXYZ &sourceCloud){
        
        #ifdef LOG_EN 
            std::cout << "Rigid Solver Start... " << std::endl;
        #endif

        mat6x6 *acc_A;
        cudaStatus = cudaMallocManaged(&acc_A, sizeof(mat6x6));
        if (cudaStatus != cudaSuccess) {
            fprintf(stderr, "cudaMallocManaged failed!  acc_A -> gnSolver?");
            return;
        }
        cudaMemset(acc_A,  0, sizeof(mat6x6));
        cudaDeviceSynchronize();
        mat6x1 *acc_b;
        cudaStatus = cudaMallocManaged(&acc_b, sizeof(mat6x1));
        if (cudaStatus != cudaSuccess) {
            fprintf(stderr, "cudaMallocManaged failed!  acc_b -> gnSolver?");
            return;
        }
        cudaMemset(acc_b,  0, sizeof(mat6x1));
        cudaDeviceSynchronize();
                
        Vector6d increment, prev_increment;
        increment << 0, 0, 0, 0, 0, 0;
        prev_increment = increment;

        for(int pylvl = 0; pylvl < 3; pylvl++) {

            int scale = OptProp.lvlScale[pylvl];

            tragetPyImage.downPyramidDepth(targetImage.depth, scale);
            sourcePyImage.downPyramidDepth(sourceImage.depth, scale);
            
            // tragetPyImage.downPyramid(targetImage, scale);
            // sourcePyImage.downPyramid(sourceImage, scale);

            tragetPyImage.getPointCloudXYZ(targetCloud, scale);
            tragetPyImage.getNormalsfromVertices(targetCloud);

            sourceImage.getPointCloudXYZ(sourceCloud, scale);
            sourceImage.getNormalsfromVertices(sourceCloud);
            // tragetPyImage.getNormalsfromDepthImage(normals);

            int threads_per_block = 64;
            int thread_blocks =(sourceImage.sensor.rows * sourceImage.sensor.cols + 
                threads_per_block - 1) / threads_per_block;

            for (iterationNum = 1 ; iterationNum < OptProp.lvlIterationNum[pylvl]; iterationNum++) {
                
                Eigen::Matrix4d camToWorld = Exp(v2t(increment));
                Eigen::Matrix4f camToWorldf = (camToWorld.cast<float>());
                float4x4 transformCuda = float4x4(camToWorldf.data()).getTranspose();
                
                acc_A->setZero();
                acc_b->setZero();

                buildNLSKernel<<<thread_blocks, threads_per_block>>>(*this,
                                                                     targetImage.rgb, tragetPyImage.depth,
                                                                     sourceImage.rgb, sourcePyImage.depth,
                                                                     sourceImage.sensor, transformCuda,
                                                                     acc_A, acc_b, 
                                                                     targetCloud,
                                                                     sourceCloud, 
                                                                     scale, OptProp.huberConstant);
                cudaDeviceSynchronize();

                Eigen::Matrix<double, 6, 6> A;
                for (int r = 0; r < 6; r++) {
                    for (int c = 0; c < 6; c++) {
                        A(r, c) = static_cast<double>((*acc_A)(r, c));
                    }
                }
                Vector6d b;
                for (int k = 0; k < 6; k++) {
                    b(k) = static_cast<double>((*acc_b)(k));
                }
   
                double scaling = 1 / A.maxCoeff();
                b *= scaling;
                A *= scaling;

                double alpha = OptProp.regularization * A.maxCoeff();
                A = A + (alpha / iterationNum) * Eigen::MatrixXd::Identity(6, 6);

                #ifdef GPU_SOLVER
                    increment = increment + solveSparceLinearSystem(A, b);
                #else
                    increment = increment + SolveLdlt(A, b);
                #endif

                Vector6d change = increment - prev_increment;
                if (fabs(change.norm()) <= OptProp.minIncrement) break;
                prev_increment = increment;
            }

            #ifdef LOG_EN 
                std::cout << "pylvl: " << pylvl << 
                ", maxIt: " << OptProp.lvlIterationNum[pylvl] << 
                                ", iterationNum: " << iterationNum << 
                                ", scale: " << scale << std::endl;
            #endif
        }

        if (std::isnan(increment.sum()) || std::isinf(increment.sum())) increment << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        // std::cout << increment << " ----> " << increment.sum() << std::endl;
        if (increment.sum() >= OptProp.maxIncrement) increment << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        pose = (Exp(v2t(increment)).cast<float>()) * pose;
        prev_increment_ = increment;

        cudaFree(acc_A);
        cudaFree(acc_b);

        #ifdef LOG_EN 
            std::cout << "Rigid Solver Done... " << std::endl;
        #endif
    }
    Eigen::Matrix<double, 6, 1> gnSolver::SolveLdlt(const Eigen::Matrix<double, 6, 6> &A,
        const Eigen::Matrix<double, 6, 1> &b) {
        return A.ldlt().solve(b);
    }
    Eigen::Matrix4d gnSolver::Exp(const Eigen::Matrix4d &mat) {
        return mat.exp();
    }
    Eigen::Matrix4d gnSolver::v2t(const Vector6d &xi) {
        Eigen::Matrix4d M;
      
        M << 0.0  , -xi(2),  xi(1), xi(3),
             xi(2), 0.0   , -xi(0), xi(4),
            -xi(1), xi(0) , 0.0   , xi(5),
             0.0,   0.0   , 0.0   ,   0.0;
      
        return M;
    }
    __host__ __device__ 
    float gnSolver::Intensity(float3 color) {
        return 0.2126 * color.x + 0.7152 * color.y + 0.0722 * color.z;
    }
    __host__ __device__ 
    float gnSolver::ColorDifference(uchar3 c1, uchar3 c2) {
        float3 c1_float = ColorToFloat(c1);
        float3 c2_float = ColorToFloat(c2);
        return Intensity(c1_float)-Intensity(c2_float);
    }
    void gnSolver::printSolverPros(void){
        printf("%f \n", OptProp.huberConstant);
    } 
    void gnSolver::cuBlasVectorSum(const float alf, const float *A, const float bet, const float *B, float *C, const int m){
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

} // end of namespace Solver
}   //end namespace DynaMap