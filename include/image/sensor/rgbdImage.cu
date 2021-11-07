/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
#include "sensor/rgbdImage.h"
#define NORMAL_STEP 2
namespace DynaMap{

    rgbdImage::~rgbdImage(){
        cudaDeviceSynchronize();
        cudaFree(rgb);
        cudaFree(depth);
    }
    void rgbdImage::init(const rgbdSensor& _sensor){
        sensor = _sensor;
        cudaMallocManaged(&rgb, sizeof(uchar3) * sensor.rows * sensor.cols);
        cudaMallocManaged(&depth, sizeof(float) * sensor.rows * sensor.cols);
        cudaDeviceSynchronize();
    }
    __global__ 
    void creatPointCloudKernel(float* depth, 
                                geometry::PointCloudXYZ& cloud,
                                rgbdSensor sensor, 
                                int scale) {
        int index = blockIdx.x * blockDim.x + threadIdx.x;
        int stride = blockDim.x * gridDim.x;
        int size = sensor.rows * sensor.cols;
        geometry::PointXYZ point;
        for (int idx = index; idx < size; idx += stride){
            if(depth[idx] <= 0.0) continue;
            point = getPoint3d(idx, depth[idx], sensor, scale);
            cloud.points[idx].x = point.x;
            cloud.points[idx].y = point.y;
            cloud.points[idx].z = point.z;
        }
    }
    void rgbdImage::getPointCloudXYZ(geometry::PointCloudXYZ& cloud, int scale){
        int threads_per_block = 256;
        int thread_blocks =(sensor.cols * sensor.rows + threads_per_block - 1) / threads_per_block;
        //std::cout << "<<<kernel_creatPointCloud>>> threadBlocks: "<< thread_blocks << ", threadPerBlock: " << threads_per_block << std::endl;
        creatPointCloudKernel <<< thread_blocks, threads_per_block >>> (this->depth, 
                                                                        cloud,
                                                                        this->sensor, 
                                                                        scale);
        cudaDeviceSynchronize();
        
    }
    __device__
    float3 rgbdImage::depthtoNormal(float* depth, rgbdSensor sensor, int idx){
        int size =  sensor.rows * sensor.cols;                        
        int step = NORMAL_STEP;
        int index = idx + step;
        int right = (index >= size ) ? idx : index;
        index = idx - step;
        int left = (index <= 0 ) ? idx : index;
        
        index = idx + sensor.cols * step;
        int down = (index >= size ) ? idx : index;
        index = idx - sensor.cols * step;
        int up = (index <= 0 ) ? idx : index;

        float dzdx = ((depth[right] - depth[left])/2) *sensor.depthScaler;
        float dzdy = ((depth[down]  - depth[up])/2) *sensor.depthScaler;
        float3 tanX = make_float3(1.0f ,0.0f, dzdx);

        float3 tanY = make_float3(0.0f, 1.0f, dzdy);
        return normalize(cross(tanX, tanY));
        
    }
    __global__
    void depthToNormalKernel(rgbdImage &image, 
                             float* depth, 
                             rgbdSensor sensor, 
                             geometry::PointCloudXYZ& cloud){
        int index = blockIdx.x * blockDim.x + threadIdx.x;
        int stride = blockDim.x * gridDim.x;
        int size = sensor.rows * sensor.cols;

        for (int idx = index; idx < size; idx += stride) {
            // int v = static_cast<int>(idx / sensor.cols);
            // int u = static_cast<int>(idx - sensor.cols * v);
            cloud.normals[idx]  = image.depthtoNormal(depth, sensor, idx);
        }
    }
    void rgbdImage::getNormalsfromDepthImage(geometry::PointCloudXYZ& cloud){
        int threads_per_block = 256;
        int thread_blocks =(sensor.cols * sensor.rows + threads_per_block - 1) / threads_per_block;
        // std::cout << "<<< depthToNormal >>> threadBlocks: "<< thread_blocks << ", threadPerBlock: " << threads_per_block << std::endl;
        depthToNormalKernel <<< thread_blocks, threads_per_block >>> (*this,
                                                                       depth, 
                                                                       sensor,
                                                                       cloud);
        cudaDeviceSynchronize();
    }
    // cv::Mat rgbdImage::testNormalsfromDepthImage(cv::Mat& depth, float depthScalar){
    //     geometry::NormalsXYZ Normals;
    //     depth.convertTo(depth, CV_32FC1, depthScalar);
    //     cv::Mat NormalImage(sensor.rows, sensor.cols, CV_32FC3);
    //     cudaMallocManaged(&Normals.x, sizeof(float) * sensor.rows * sensor.cols);
    //     cudaMallocManaged(&Normals.y, sizeof(float) * sensor.rows * sensor.cols);
    //     cudaMallocManaged(&Normals.z, sizeof(float) * sensor.rows * sensor.cols);

    //     for (uint i = 0; i < sensor.rows; i++) {
    //         for (uint j = 0; j < sensor.cols; j++) {
    //                 this->depth[i * sensor.cols + j] = depth.at<float>(i, j);
    //         }
    //     }

    //     this->getNormalsfromDepthImage(Normals);

    //     for (int i = 0; i < sensor.rows; i++) {
    //         for (int j = 0; j < sensor.cols; j++) {
    //             NormalImage.at<cv::Vec3f>(i, j)[0] = Normals.x[i * sensor.cols + j];
	// 			NormalImage.at<cv::Vec3f>(i, j)[1] = Normals.y[i * sensor.cols + j];
	// 			NormalImage.at<cv::Vec3f>(i, j)[2] = Normals.z[i * sensor.cols + j];
    //         }
    //     }
	// 	cudaFree(Normals.x);
    //     cudaFree(Normals.y);
    //     cudaFree(Normals.z);
    //     return NormalImage;
    // }
    __device__
    float3 rgbdImage::vertextoNormal(geometry::PointCloudXYZ& cloud, rgbdSensor sensor, int idx){
        int step = NORMAL_STEP;
            int size = sensor.cols * sensor.rows;
            int index = idx - step;
            int left_idx = (index < 0 ) ? idx : index;
            const float3 left = cloud.points[left_idx];
            index = idx + step;
            int right_idx = (index >= size ) ? idx : index;
            const float3 right = cloud.points[right_idx];
            index = idx - sensor.cols;
            int up_idx = (index <= 0 ) ? idx : index;
            const float3 up = cloud.points[up_idx];
            index = idx + sensor.cols;
            int down_idx = (index >= size ) ? idx : index;
            const float3 down = cloud.points[down_idx];

            const float3 dzdx = right - left;
            const float3 dzdy = down - up;
            return normalize(cross(dzdx, dzdy)); // switched dx and dy to get factor -1
    }
    __global__ 
    void vertexToNormalKernel(rgbdImage& image, 
                              rgbdSensor sensor, 
                              geometry::PointCloudXYZ& cloud){
        int index = blockIdx.x * blockDim.x + threadIdx.x;
        int stride = blockDim.x * gridDim.x;
        int size = sensor.cols * sensor.rows;

        for (int idx = index; idx < size; idx += stride) {
            cloud.normals[idx] = image.vertextoNormal(cloud, sensor, idx);  
        }
    }
    void rgbdImage::getNormalsfromVertices(geometry::PointCloudXYZ& cloud){
        int threads_per_block = 256;
        int thread_blocks =(sensor.cols * sensor.rows + threads_per_block - 1) / threads_per_block;
        // std::cout << "<<< vertexToNormal >>> threadBlocks: "<< thread_blocks << ", threadPerBlock: " << threads_per_block << std::endl;
        vertexToNormalKernel <<< thread_blocks, threads_per_block >>> (*this, 
                                                                        sensor, 
                                                                        cloud);
        cudaDeviceSynchronize();
        
    }
    // cv::Mat rgbdImage::testNormalsfromVertices(cv::Mat& depth, float depthScalar){
    //     geometry::NormalsXYZ Normals;
    //     geometry::PointCloudXYZ testPCL;
    //     depth.convertTo(depth, CV_32FC1, depthScalar);
    //     cv::Mat NormalImage(sensor.rows, sensor.cols, CV_32FC3);
    //     cudaMallocManaged(&Normals.x, sizeof(float) * sensor.rows * sensor.cols);
    //     cudaMallocManaged(&Normals.y, sizeof(float) * sensor.rows * sensor.cols);
    //     cudaMallocManaged(&Normals.z, sizeof(float) * sensor.rows * sensor.cols);

    //     cudaMallocManaged(&testPCL.x, sizeof(float) * sensor.rows * sensor.cols);
    //     cudaMallocManaged(&testPCL.y, sizeof(float) * sensor.rows * sensor.cols);
    //     cudaMallocManaged(&testPCL.z, sizeof(float) * sensor.rows * sensor.cols);

    //     for (uint i = 0; i < sensor.rows; i++) {
    //         for (uint j = 0; j < sensor.cols; j++) {
    //                 this->depth[i * sensor.cols + j] = depth.at<float>(i, j);
    //         }
    //     }

    //     this->getPointCloudXYZ(testPCL);
    //     this->getNormalsfromVertices(Normals, testPCL);
    //     // for (size_t i = 1000; i < 1050 ; i++){
    //     //     std::cout << Normals.x[i] << ", " << Normals.y[i] << ", " << Normals.z[i] << std::endl;
    //     // }
    //     for (int i = 0; i < sensor.rows; i++) {
    //         for (int j = 0; j < sensor.cols; j++) {
    //             NormalImage.at<cv::Vec3f>(i, j)[0] = static_cast<float>(Normals.x[i * sensor.cols + j]);
	// 			NormalImage.at<cv::Vec3f>(i, j)[1] = static_cast<float>(Normals.y[i * sensor.cols + j]);
	// 			NormalImage.at<cv::Vec3f>(i, j)[2] = static_cast<float>(Normals.z[i * sensor.cols + j]);
    //         }
    //     }
	// 	cudaFree(Normals.x);
    //     cudaFree(Normals.y);
    //     cudaFree(Normals.z);

    //     cudaFree(testPCL.x);
    //     cudaFree(testPCL.y);
    //     cudaFree(testPCL.z);
    //     return NormalImage;
    // }
    // cv::Mat rgbdImage::testNormalsfromDepthImageCV(cv::Mat& depth, float depthScalar){
    //     depth.convertTo(depth, CV_32FC1, depthScalar);  // check depth scalar
    //     cv::Mat CVnormals(depth.size(), CV_32FC3);
	// 	for(int x = 0; x < depth.rows; ++x){
	// 		for(int y = 0; y < depth.cols; ++y){
	// 			float dzdx = (depth.at<float>(x+1, y) - depth.at<float>(x-1, y)) / 2.0;
	// 			float dzdy = (depth.at<float>(x, y+1) - depth.at<float>(x, y-1)) / 2.0;

	// 			cv::Vec3f d(-dzdx, -dzdy, 1.0f);

	// 			cv::Vec3f n = cv::normalize(d);
	// 			CVnormals.at<cv::Vec3f>(x, y) = n;
	// 		}
	// 	}
	// 	return CVnormals;
    // }
    cv::Mat rgbdImage::getCVImagefromCudaDepth(void){
        cv::Mat _depth(sensor.rows, sensor.cols, CV_32FC1);
        for (int i = 0; i < sensor.rows; i++) {
            for (int j = 0; j < sensor.cols; j++) {
                _depth.at<float>(i, j) = static_cast<float>(this->depth[i * sensor.cols + j]);
            }
        }
        return _depth;
    }
    void rgbdImage::getCudafromCVDepth(cv::Mat cvDepth){
        for (uint i = 0; i < sensor.rows; i++) {
            for (uint j = 0; j < sensor.cols; j++) {
                this->depth[i * sensor.cols + j] = cvDepth.at<float>(i, j);
            }
        }
    }
}  // namespace DynaMap




