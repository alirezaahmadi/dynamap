/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
#include "utils_Dataset.h"

namespace DynaMap{

namespace utils{

    dataSet::dataSet(std::string mainDir){

        curID = 0;
        groundTruthExist = false;
        assciateFileExist = false;

        std::string rgbFolder = "/rgb/";
        std::string depthFolder = "/depth/";
        std::string groundTruthFileName = "/groundtruth.txt";
        std::string associationFileName = "/association.txt";

        //  check for assocation file
        // this->assciateFileExist =  
        // std::cout << "Assocition: " << Dataset.assciateFileExist << std::endl; 

        this->groundTruthFileAddress = mainDir + groundTruthFileName;
        this->associationFileAddress = mainDir + associationFileName;
        this->PreSufixCheck = true;
        this->rgbFolderAddress = mainDir + rgbFolder;
        this->depthFolderAddress = mainDir + depthFolder;

        // this->depthPrefix = "frame";
        // this->rgbPrefix = "frame";
        // this->rgbSuffix = ".png";
        // this->depthSuffix = ".png";

        // TUM
        this->depthPrefix = "";
        this->rgbPrefix = "";
        this->rgbSuffix = ".png";
        this->depthSuffix = ".png";

        // this->depthPrefix = "depth_";
        // this->rgbPrefix = "color_";
        // this->rgbSuffix = ".png";
        // this->depthSuffix = ".png";

        this->curID = 0;
        // this->sensor = sensor;
        this->loadData();
  
    }

    dataSet::~dataSet(){}

    bool dataSet::loadData(void){
        if(assciateFileExist){
            getAssociateContent(associationFileAddress);
        }else{
            depthFileNames =  dataSet::invokeFileNames(depthFolderAddress, depthSuffix, depthPrefix);
            rgbFileNames =  dataSet::invokeFileNames(rgbFolderAddress, rgbSuffix, rgbPrefix);
        }
        refLines = getFileContent(groundTruthFileAddress);
        // for(int i=0; i< depthFileNames.size(); i++)std::cout << depthFileNames[i] << ", " << rgbFileNames[i] << std::endl;
        return true;
    }
    Eigen::Matrix4f dataSet::getCurrentPose(void){
        Eigen::Matrix4f result;
        if(groundTruthExist){
            Eigen::Vector3d T = Eigen::Vector3d::Zero();
            T << groundTruthLine[1], 
                 groundTruthLine[2], 
                 groundTruthLine[3];
            Eigen::Quaterniond q;
            q.x() = groundTruthLine[4];
            q.y() = groundTruthLine[5];
            q.z() = groundTruthLine[6];
            q.w() = groundTruthLine[7]; 
            Eigen::Matrix3f R = q.normalized().toRotationMatrix().cast<float>();
            result(0,0) = R(0,0); result(0,1) = R(0,1); result(0,2) = R(0,2); result(0,3) = T[0];
            result(1,0) = R(1,0); result(1,1) = R(1,1); result(1,2) = R(1,2); result(1,3) = T[1];
            result(2,0) = R(2,0); result(2,1) = R(2,1); result(2,2) = R(2,2); result(2,3) = T[2];
            result(3,0) = 0.0f;   result(3,1) = 0.0f;   result(3,2) = 0.0f;   result(3,3) = 1.0f;
        }
        return result;
    }
    void dataSet::setCurrentPose(Eigen::Matrix4f Pose){
        Eigen::Matrix4f res;
        if(groundTruthExist){
            Eigen::Vector3d T = Eigen::Vector3d::Zero();
            groundTruthLine[1] = Pose(0, 3); 
            groundTruthLine[2] = Pose(1, 3); 
            groundTruthLine[3] = Pose(2, 3);
        }
        //     Eigen::Quaterniond q = 
        //     groundTruthLine[4] = ;
        //     groundTruthLine[5];
        //     groundTruthLine[6];
        //     groundTruthLine[7]; 
        //     Eigen::Matrix3f R = q.normalized().toRotationMatrix().cast<float>();

        //     Eigen::Matrix4f res;
        //     res(0,0) = R(0,0); res(0,1) = R(0,1); res(0,2) = R(0,2); res(0,3) = T[0];
        //     res(1,0) = R(1,0); res(1,1) = R(1,1); res(1,2) = R(1,2); res(1,3) = T[1];
        //     res(2,0) = R(2,0); res(2,1) = R(2,1); res(2,2) = R(2,2); res(2,3) = T[2];
        //     res(3,0) = 0;      res(3,1) = 0;      res(3,2) = 0;      res(3,3) = 1;
        // }
    }
    std::vector<std::string> dataSet::invokeFileNames(const std::string _folderAddress, std::string _suffix, std::string _prefix) {
        std::vector<std::string> ImageNames;
        // represent path in the filesystem (std::filesystem::path)
        boost::filesystem::path p(_folderAddress);
        // std::cout << "Database from:  " << _folderAddress << std::endl;
        for (auto i = boost::filesystem::directory_iterator(p); i != boost::filesystem::directory_iterator(); i++) {
            std::string name_ = i->path().filename().string();
            // std::cout << "check data ..."  << std::endl;
            // check if not a directory and if the file is a png-image !boost::filesystem::is_directory(i->path())
            if (true) {
                if(!PreSufixCheck && name_.std::string::compare(name_.size() - _suffix.size(), name_.size(), _suffix) == 0){
                    std::cout<< "PreSufixCheck ... fault !! " << std::endl; 
                    continue;
                }
                // std::cout << i->path().filename().string() << std::endl;
                ImageNames.push_back(i->path().filename().string());
            } else {
                std::cout << "cant read data ..." << std::endl;
                continue;
            }
            
        }
        for (size_t i = 0; i < ImageNames.size(); i++){
            ImageNames[i].erase(ImageNames[i].size() - _suffix.size());
            if(ImageNames[i].std::string::compare(0, _prefix.size(), rgbPrefix) == 0){
                ImageNames[i].erase(0, rgbPrefix.size());
            }else{
                ImageNames[i].erase(0, depthPrefix.size());
            }
            
            // std::cout << ImageNames[i] << std::endl;  
        } 
        
        std::sort(ImageNames.begin(), ImageNames.end(), sortFunc);

        std::cout << "Database Loaded from: " << _folderAddress << std::endl;
        return ImageNames;
    }
    void dataSet::getAssociateContent(std::string fileName){
        std::string _depthFileName;
        std::string _rgbFileNames;
        std::ifstream in(fileName.c_str());
        // Check if object is valid
        if(!in){
            std::cerr << "Cannot open the File : "<<fileName<<std::endl;
        }else{
            std::string str;
            // Read the next line from File untill it reaches the end.
            while (std::getline(in, str)){
                // 
                if(str[0] == '#'){
                    std::cout <<"Ignored lines: " << str << std::endl;
                    continue;
                }
                // Line contains string of length > 0 then save it in vector
                if(str.size() > 0){
                    std::string word;                 // Have a buffer string
                    std::stringstream ss(str);       // Insert the string into a stream
                    std::vector<std::string> words; // Create vector to hold our words

                    while (ss >> word)
                        words.push_back(word);

                    depthFileNames.push_back(words[0]);
                    rgbFileNames.push_back(words[2]);
                }
            }
            //Close The File
            in.close();
        }
    }
    std::vector<std::string> dataSet::getFileContent(std::string fileName){
        std::vector<std::string> _refLines;
        std::ifstream in(fileName.c_str());
        // Check if object is valid
        if(!in){
            std::cerr << "Cannot open the File : "<<fileName<<std::endl;
            groundTruthExist = false;
        }else{
            std::string str;
            // Read the next line from File untill it reaches the end.
            while (std::getline(in, str)){
                // 
                if(str[0] == '#'){
                    std::cout <<"Ignored lines: " << str << std::endl;
                    continue;
                }
                // Line contains string of length > 0 then save it in vector
                if(str.size() > 0){
                    // std::cout << str << std::endl;
                    _refLines.push_back(str);
                }
            }
            //Close The File
            in.close();
            groundTruthExist = true;
        }
        return _refLines;
    }
    void dataSet::invokeNextGroundTruthLine(void){
        if(groundTruthExist){
            int index = curID + offsetLineGroundTruth;
            if(refLines.size() != 0){
                groundTruthLine = splitLinetoNumbers(refLines[index]);
                // groundTruthLine.erase (groundTruthLine.begin());
                // std::cout << refLines[index] << std::endl;
                // for(int i=0; i< 8; i++)
                // std::cout << std::fixed << std::setprecision(6) << groundTruthLine[i] << " "
                // << std::endl;
            }
        }
    }
    void dataSet::invokeNextImage(){
        if(rgbFileNames.size() != 0){
            rgbFileNames[curID] = rgbFolderAddress + rgbPrefix + rgbFileNames[curID] + rgbSuffix;
            // std:: cout << rgbFileNames[curID] << std::endl;
            image.rgb = cv::imread(rgbFileNames[curID], 
                                        CV_LOAD_IMAGE_COLOR);
        }
        if(depthFileNames.size() != 0){
            depthFileNames[curID] = depthFolderAddress + depthPrefix + depthFileNames[curID] + depthSuffix;
            // std:: cout << depthFileNames[curID] << std::endl;
            image.depth = cv::imread(depthFileNames[curID], 
                                          CV_LOAD_IMAGE_ANYDEPTH);
            image.depth.convertTo(image.depth, CV_32FC1, 1.0f / sensor.depthScaler);
        }
    }
    void dataSet::invokeNextFrame(){
        invokeNextGroundTruthLine();
        invokeNextImage();
        curID++;
    }
    bool dataSet::sortFunc(const std::string &s1, const std::string &s2){
        if(std::stod(s1) < std::stod(s2))
            return true;
        return false;
    }
} // end of namespace utils

}  // namespace DynaMap