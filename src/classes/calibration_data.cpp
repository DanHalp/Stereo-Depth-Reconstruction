#include <iostream>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#define CALIBRATION_DATA_IMPLIMENTATION
#include "calibration_data.h"



/**
 * @brief find all the location of a substring in a string
 * @param main_str : main string
 * @param substr : substring to find in the main string
 * 
 * @return location : vector containing all the location of substring in the main string
*/
std::vector<size_t> FindSubstringLocation (const std::string &main_str, const std::string &substr){
    std::vector<size_t> location;
    size_t pos = main_str.find(substr);
    while(pos != std::string::npos){
        location.push_back(pos);
        pos = main_str.find(substr, pos + substr.size());
    }
    return location;
}


/**
 * @brief get single camera intrinsics
 * @param cam_intrinsic : string including intrinsics data
 * 
 * @return cam : single camera intrinsics stored as Eigen::Matrix3d type
*/
Eigen::Matrix3d GetSingleIntrinsics(const std::string &cam_intrinsic){
    Eigen::Matrix3d cam;
    std::vector<size_t> loc = FindSubstringLocation (cam_intrinsic, " ");
    cam(0,0) = std::stod(cam_intrinsic.substr(0, loc[0]));
    cam(0,1) = std::stod(cam_intrinsic.substr(loc[0], loc[1]));
    cam(0,2) = std::stod(cam_intrinsic.substr(loc[1], loc[2]-1));

    cam(1,0) = std::stod(cam_intrinsic.substr(loc[2], loc[3]));
    cam(1,1) = std::stod(cam_intrinsic.substr(loc[3], loc[4]));
    cam(1,2) = std::stod(cam_intrinsic.substr(loc[4], loc[5]-1));

    cam(2,0) = std::stod(cam_intrinsic.substr(loc[5], loc[6]));
    cam(2,1) = std::stod(cam_intrinsic.substr(loc[6], loc[7]));
    cam(2,2) = std::stod(cam_intrinsic.substr(loc[7], loc[8]-1));
    return cam;
}



CalibrationData::CalibrationData(const std::string &filelocation) : _path(filelocation)
{
    std::ifstream inFile(filelocation);
    std::string str;
    std::string file_contents;
    if(!inFile){
        std::cerr << "cannot open file" << std::endl;
        exit(1);
    }
    else{
        while (std::getline(inFile, str)){
            file_contents += str;
            file_contents.push_back('\n');
        }
    }
    double baseline;
    int doffs, width, height, ndisp, vmin, vmax;
    
    std::vector<size_t> vec = FindSubstringLocation (file_contents, "\n");
    
    std::string cam0sub = file_contents.substr(6, vec[0]-2);
    std::string cam1sub = file_contents.substr(vec[0]+7, vec[1]-2);

    Eigen::Matrix3d cam0Eigen = GetSingleIntrinsics(cam0sub);
    Eigen::Matrix3d cam1Eigen = GetSingleIntrinsics(cam1sub);
    cv::eigen2cv(cam0Eigen, CalibrationData::cam0);
    cv::eigen2cv(cam1Eigen, CalibrationData::cam1);
    
    CalibrationData::doffs = std::stoi(file_contents.substr(vec[1]+7, vec[2]-(vec[1]+7)));
    CalibrationData::baseline = std::stod(file_contents.substr(vec[2]+10, vec[3]-(vec[1]+10)));
    CalibrationData::width = std::stoi(file_contents.substr(vec[3]+7, vec[4]-(vec[3]+7)));
    CalibrationData::height = std::stoi(file_contents.substr(vec[4]+8, vec[5]-(vec[4]+8)));
    CalibrationData::ndisp = std::stoi(file_contents.substr(vec[5]+7, vec[6]-(vec[5]+7)));
    CalibrationData::vmin = std::stoi(file_contents.substr(vec[6]+6, vec[7]-(vec[6]+6)));
    CalibrationData::vmax = std::stoi(file_contents.substr(vec[7]+6, vec[8]-(vec[7]+6)));
}

/**
 * @brief print all elements in a the class CalibrationData
 *        just for testing and convinence
*/
const void CalibrationData::PrintCalData(){

    std::cout << "cam0 " << std::endl;
    std::cout << cam0 << std::endl;

    std::cout << "cam1 " << std::endl;
    std::cout << cam1<< std::endl;
    
    std::cout << "doffs " << doffs << std::endl;
    std::cout << "baseline " << baseline << std::endl;
    std::cout << "width " << width << std::endl;
    std::cout << "height " << height << std::endl;
    std::cout << "ndisp " << ndisp << std::endl;
    std::cout << "vmin " << vmin << std::endl;
    std::cout << "vmax " << vmax << std::endl;
}