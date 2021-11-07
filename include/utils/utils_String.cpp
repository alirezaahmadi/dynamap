/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
#include "utils_String.h"

namespace DynaMap{
namespace utils{

    /************************************/
    /* SAVE REAL ARRAY FROM CPU TO FILE */
    /************************************/
    template <class T>
    void saveCPUrealtxt(const T * h_in, const char *filename, const int M) {

        std::ofstream outfile;
        outfile.open(filename);
        for (int i = 0; i < M; i++) outfile << std::setprecision(4) << h_in[i] << "\n";
        outfile.close();

    }

    std::vector<double> splitLinetoNumbers(const std::string& refLine){
        std::vector<double> numbers;
       
        std::string::size_type pos = 0;
        double Num;
        std::stringstream stream(refLine);
        while(stream >> Num){
            numbers.push_back(Num);
        }
        return numbers;
    }  
    void writeNormalsToTxt(const std::string& fileName, 
                           geometry::PointCloudXYZ& cloud, 
                           int size){
        std::ofstream file(fileName);
        if (file.is_open()){
            for(int count = 0; count < size; count ++){
                file << count << " " 
                << cloud.points[count].x << " " << cloud.points[count].y << " " << cloud.points[count].z << " "
                << cloud.normals[count].x << " " << cloud.normals[count].y << " " << cloud.normals[count].z << std::endl ;
            }
            file.close();
        }
        else std::cout << "Unable to open file";
    } 
    void writePoseToTxt(const std::string& fileName, 
                        const Eigen::Matrix4f& Pose,
                           int count){

        std::ofstream file(fileName,std::ios::app);
        Eigen::Matrix3f R;
        R << Pose(0,0), Pose(0,1), Pose(0,2), 
             Pose(1,0), Pose(1,1), Pose(1,2), 
             Pose(2,0), Pose(2,1), Pose(2,2);
        Eigen::Quaterniond q(R.cast<double>());
        if (file.is_open()){
                file << std::fixed << std::setprecision(6) << count
                     << " " << Pose.block<3, 1>(0, 3).transpose() << " "
                     << q.vec().transpose() << " " << q.w() << std::endl;
            file.close();
        }
        else std::cout << "Unable to open file";
    }  
    // template <class T>
    void writeMatrixToTxt(const char* fileName, 
                          float *matrix,
                          int rows, int cols,
                          int mode){
        std::ofstream file(fileName,std::ios::app);
        if (file.is_open()){
            std::cout << "[ "<< std::endl;
            for (size_t i = 0; i < rows; i++){
                for (size_t j = 0; j < cols; j++){
                    if(mode == 0){        // row major
                        file << std::fixed << std::setprecision(6)
                        << " " << matrix[i * cols + j];
                    }else if(mode ==1){   // column major
                        file << std::fixed << std::setprecision(6)
                        << " " << matrix[i + cols * j];
                    }
                }
                std::cout << " ]"<< std::endl;
            }
            file.close();
        }
        else std::cout << "Unable to open file";
    }  
} // end of namespace utils
} // end of namespace DynaMap


// /************************************/
// /* SAVE REAL ARRAY FROM CPU TO FILE */
// /************************************/
// template <class T>
// void saveCPUrealtxt(const T * h_in, const char *filename, const int M) {

//     std::ofstream outfile;
//     outfile.open(filename);
//     for (int i = 0; i < M; i++) outfile << std::setprecision(prec_save) << h_in[i] << "\n";
//     outfile.close();

// }

// /************************************/
// /* SAVE REAL ARRAY FROM GPU TO FILE */
// /************************************/
// template <class T>
// void saveGPUrealtxt(const T * d_in, const char *filename, const int M) {

//     T *h_in = (T *)malloc(M * sizeof(T));

//     gpuErrchk(cudaMemcpy(h_in, d_in, M * sizeof(T), cudaMemcpyDeviceToHost));

//     std::ofstream outfile;
//     outfile.open(filename);
//     for (int i = 0; i < M; i++) outfile << std::setprecision(prec_save) << h_in[i] << "\n";
//     outfile.close();

// }