/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
#include <builder.h>

namespace DynaMap{
    builder::builder(void){}
    void builder::init(const tsdf::tsdfProperties& tsdfProp,
                     const solver::Properties _OptimizerProp,
                     const rgbdSensor &_sensor,
                     const gaussianKernal &_kernel) {

        sensor = _sensor;
        cntt = 0;
        old_tran.resize(7,0);
        sourceImage.init(sensor);
        targetImage.init(sensor);

        visImage.init(sensor);

        GNSolver.init(_OptimizerProp, sensor);
        
        init_pose = Eigen::Matrix4f::Identity();   
        currentPose = Eigen::Matrix4f::Identity();  
        
        GKernel = _kernel;
        
        FirstFrame = true;
        frameNum = 0;
        
        cudaMallocManaged(&volume_, sizeof(tsdf::tsdfVolume));
        cudaMallocManaged(&GKernel.kernel, sizeof(float) * GKernel.diameter * GKernel.sigmaI); 

        cudaMallocManaged(&targetPCL, sizeof(geometry::PointCloudXYZ));
        cudaMallocManaged(&targetPCL->points, sizeof(geometry::PointXYZ) * sensor.rows * sensor.cols); 
        cudaMallocManaged(&targetPCL->normals, sizeof(geometry::NormalXYZ) * sensor.rows * sensor.cols);  

        cudaMallocManaged(&sourcePCL, sizeof(geometry::PointCloudXYZ) * sensor.rows * sensor.cols);
        cudaMallocManaged(&sourcePCL->points, sizeof(geometry::PointXYZ) * sensor.rows * sensor.cols); 
        cudaMallocManaged(&sourcePCL->normals, sizeof(geometry::NormalXYZ) * sensor.rows * sensor.cols);

        cudaDeviceSynchronize();
        
        volume_->init(tsdfProp);
        
    }
    builder::~builder() {
        cudaDeviceSynchronize();
        volume_->Free();
        cudaFree(volume_);
        cudaFree(GKernel.kernel);

        cudaFree(targetPCL);
        cudaFree(targetPCL->points);
        cudaFree(targetPCL->normals);

        cudaFree(sourcePCL);
        cudaFree(sourcePCL->points);
        cudaFree(sourcePCL->normals);

        cudaDeviceSynchronize();
    }
    geometry::Mesh builder::ExtractMesh(const float3 &lower_corner,
                                        const float3 &upper_corner) {
        return volume_->ExtractMesh(lower_corner, upper_corner);
    }
    void builder::buildModelLS(utils::dataSet& dataset, int Mode){ 
        #ifdef LOG_EN
            std::cout << "Rigis Registration start ..." << std::endl;
        #endif

        cudaMemset(targetPCL->points, 0, sizeof(geometry::PointCloudXYZ) * sensor.rows * sensor.cols);
        cudaMemset(targetPCL->normals, 0, sizeof(geometry::PointCloudXYZ) * sensor.rows * sensor.cols);

        cudaMemset(sourcePCL->points, 0, sizeof(geometry::PointCloudXYZ) * sensor.rows * sensor.cols);
        cudaMemset(sourcePCL->normals, 0, sizeof(geometry::PointCloudXYZ) * sensor.rows * sensor.cols);
        cudaDeviceSynchronize();

        #ifdef LOG_EN
            std::cout << "Memory Reset done ..." << std::endl;
        #endif

        if(FirstFrame){
            // seting initial pose
            // setInitialPose(dataset.groundTruthLine);
            FirstFrame = false;
        }else{
            // sourceImage.bilateralFilter(sourceImage.depth, GKernel);
            // targetImage.bilateralFilter(targetImage.depth, GKernel);
            // std::cout << "Solver Pose: " << std::endl <<  currentPose << std::endl;
            GNSolver.solve(currentPose, 
                           prev_increment_, 
                           targetImage,
                           sourceImage,
                           *targetPCL,
                           *sourcePCL); 
        }

        float4x4 pose_cuda = float4x4(currentPose.data()).getTranspose();
        // writePoseToTxt("../logs/LSPoses.txt", dataset.groundTruthLine, currentPose, frameNum++);

        for (uint i = 0; i < sensor.rows; i++) {
            for (uint j = 0; j < sensor.cols; j++) {
                sourceImage.rgb[i * sensor.cols + j] =
                    make_uchar3(dataset.image.rgb.at<cv::Vec3b>(i, j)(2), 
                                dataset.image.rgb.at<cv::Vec3b>(i, j)(1),
                                dataset.image.rgb.at<cv::Vec3b>(i, j)(0));
                sourceImage.depth[i * sensor.cols + j] = dataset.image.depth.at<float>(i, j);
            }
        }
        #ifdef LOG_EN
            std::cout << "Intergrate Frames ..." << std::endl;
        #endif
        volume_->IntegrateScan(sourceImage, pose_cuda);
        #ifdef LOG_EN
            std::cout << "Intergrate Frames done..." << std::endl;
        #endif
        switch(Mode){
            case 0:{
                // clone image source to target
                targetImage = sourceImage;
                visImage.rgb = GenerateRgb(640, 480);
            break;}
            case 1:{  
                // clone virtual images taken from TSDF to target
                visImage.rgb = GenerateRgb(640, 480);
                visImage.depth = GenerateDepth(640, 480);
                // Pyramid downsampling is needed 
                targetImage = visImage;
            break;}
        }
        #ifdef LOG_EN
            if(!FirstFrame)
            std::cout << "Rigid Registration done..." << std::endl;
        #endif
    }
    void builder::buildModelGT(utils::dataSet& dataset){ 
       
        for (uint i = 0; i < sensor.rows; i++) {
            for (uint j = 0; j < sensor.cols; j++) {
                sourceImage.rgb[i * sensor.cols + j] =
                    make_uchar3(dataset.image.rgb.at<cv::Vec3b>(i, j)(2), dataset.image.rgb.at<cv::Vec3b>(i, j)(1),
                    dataset.image.rgb.at<cv::Vec3b>(i, j)(0));
                sourceImage.depth[i * sensor.cols + j] = dataset.image.depth.at<float>(i, j);
            }
        }

        currentPose = v2tRef(dataset.groundTruthLine);
        // writePoseToTxt("../logs/RMPoses.txt", dataset.groundTruthLine, currentPose, frameNum++);
        // currentPose  =  init_pose.inverse() * currentPose;
        float4x4 pose_cuda = float4x4(currentPose.data()).getTranspose();

        volume_->IntegrateScan(sourceImage, pose_cuda);

        visImage.rgb = GenerateRgb(640, 480);
        // visImage.depth = GenerateDepth(640, 480);
    }
    uchar3* builder::GenerateRgb(int width, int height) {
        Eigen::Matrix4f posef = currentPose.cast<float>();
        float4x4 pose_cuda = float4x4(posef.data()).getTranspose();
        rgbdSensor virtual_sensor;
        virtual_sensor.rows = height;
        virtual_sensor.cols = width;
        virtual_sensor.depthScaler = sensor.depthScaler;
        float factor_x = static_cast<float>(virtual_sensor.cols) /
                        static_cast<float>(sensor.cols);
        float factor_y = static_cast<float>(virtual_sensor.rows) /
                        static_cast<float>(sensor.rows);
        virtual_sensor.f.x = factor_x * sensor.f.x;
        virtual_sensor.f.y = factor_y * sensor.f.y;
        virtual_sensor.c.x = factor_x * sensor.c.x;
        virtual_sensor.c.y = factor_y * sensor.c.y;
        uchar3 *virtual_rgb = volume_->GenerateRgb(pose_cuda, virtual_sensor);
        return virtual_rgb;
    }
    float* builder::GenerateDepth(int width, int height) {
        Eigen::Matrix4f posef = currentPose.cast<float>();
        float4x4 pose_cuda = float4x4(posef.data()).getTranspose();
        rgbdSensor virtual_sensor;
        virtual_sensor.rows = height;
        virtual_sensor.cols = width;
        virtual_sensor.depthScaler = sensor.depthScaler;
        float factor_x = static_cast<float>(virtual_sensor.cols) /
                        static_cast<float>(sensor.cols);
        float factor_y = static_cast<float>(virtual_sensor.rows) /
                        static_cast<float>(sensor.rows);
        virtual_sensor.f.x = factor_x * sensor.f.x;
        virtual_sensor.f.y = factor_y * sensor.f.y;
        virtual_sensor.c.x = factor_x * sensor.c.x;
        virtual_sensor.c.y = factor_y * sensor.c.y;
        float *virtual_depth = volume_->GenerateDepth(pose_cuda, virtual_sensor);
        return virtual_depth;
    }
    cv::Mat builder::getCanonicalRgbCV(void) {
        cv::Mat cv_virtual_rgb(sensor.rows, sensor.cols, CV_8UC3);
        for (int i = 0; i < sensor.rows; i++) {
        for (int j = 0; j < sensor.cols; j++) {
            cv_virtual_rgb.at<cv::Vec3b>(i, j)[2] =
                visImage.rgb[i * sensor.cols + j].x;
            cv_virtual_rgb.at<cv::Vec3b>(i, j)[1] =
                visImage.rgb[i * sensor.cols + j].y;
            cv_virtual_rgb.at<cv::Vec3b>(i, j)[0] =
                visImage.rgb[i * sensor.cols + j].z;
        }
        }
        return cv_virtual_rgb;
    }
    cv::Mat builder::getCanonicalDepthCV(void) {
        
        cv::Mat cv_virtual_depth(sensor.rows, sensor.cols, CV_32FC1);
        for (int i = 0; i < sensor.rows; i++) {
            for (int j = 0; j < sensor.cols; j++) {
                cv_virtual_depth.at<float>(i, j) = static_cast<float>(visImage.depth[i * sensor.cols + j]);
            }
        }
        //cv_virtual_depth.convertTo(cv_virtual_depth, CV_32FC1, 1/5000);  // check depth scalar
        return cv_virtual_depth;
    }
    Eigen::Matrix4f builder::v2tRef(std::vector<double>& vec){
        // TUM x: left,right, y: up,down, z: forward,backward
        // IPB x: right,left, y: down,up, z: forward,backward
        Eigen::Matrix4f result;
        Eigen::Vector3d T = Eigen::Vector3d::Zero();
        T << vec[1], 
             vec[2], 
             vec[3];

        Eigen::Quaterniond q;
        q.x() = vec[4];
        q.y() = vec[5];
        q.z() = vec[6];
        q.w() = vec[7]; 
        Eigen::Matrix3f R = q.normalized().toRotationMatrix().cast<float>();
        result(0,0) = R(0,0); result(0,1) = R(0,1); result(0,2) = R(0,2); result(0,3) = T[0];
        result(1,0) = R(1,0); result(1,1) = R(1,1); result(1,2) = R(1,2); result(1,3) = T[1];
        result(2,0) = R(2,0); result(2,1) = R(2,1); result(2,2) = R(2,2); result(2,3) = T[2];
        result(3,0) = 0.0f;   result(3,1) = 0.0f;   result(3,2) = 0.0f;   result(3,3) = 1.0f;
        return result;
    }
    void builder::writePoseToTxt(const char* fileName,
                                 std::vector<double>& vec, 
                                 const Eigen::Matrix4f& Pose,
                                 int count){

        std::ofstream file(fileName, std::ios::ate | std::ios::app);
        Eigen::Matrix3f R;
        R << Pose(0,0), Pose(0,1), Pose(0,2), 
             Pose(1,0), Pose(1,1), Pose(1,2), 
             Pose(2,0), Pose(2,1), Pose(2,2);
        Eigen::Quaterniond q(R.cast<double>());
        if (file.is_open()){
                file << std::fixed << std::setprecision(4)
                     << vec[0]
                     << " " << Pose.block<3, 1>(0, 3).transpose() << " "
                     << q.vec().transpose() << " " << q.w() << std::endl;
            file.close();
        }
        else std::cout << "Unable to open file";
    } 
    void builder::setInitialPose(std::vector<double>& vec){
            Eigen::Vector3d T = Eigen::Vector3d::Zero();
            T << vec[1], 
                 vec[2], 
                 vec[3];

            Eigen::Quaterniond q;
            q.x() = vec[4];
            q.y() = vec[5];
            q.z() = vec[6];
            q.w() = vec[7]; 
            Eigen::Matrix3f R = q.normalized().toRotationMatrix().cast<float>();

            init_pose(0,0) = R(0,0); init_pose(0,1) = R(0,1); init_pose(0,2) = R(0,2); init_pose(0,3) = T[0];
            init_pose(1,0) = R(1,0); init_pose(1,1) = R(1,1); init_pose(1,2) = R(1,2); init_pose(1,3) = T[1];
            init_pose(2,0) = R(2,0); init_pose(2,1) = R(2,1); init_pose(2,2) = R(2,2); init_pose(2,3) = T[2];
            init_pose(3,0) = 0;      init_pose(3,1) = 0;      init_pose(3,2) = 0;      init_pose(3,3) = 1;

            currentPose = init_pose;
            std::cout << "Initial Pose from Ground-truth..." << std::endl;
    }  
    Eigen::Matrix4f builder::getCurrentPose(void){
        return currentPose;
    }
    
}  // namespace DynaMap