/**
 * @file Images.h
 * @brief Images class
 * 
 */
#include <opencv2/opencv.hpp>

#ifndef IMAGES_H
#define IMAGES_H


//Keypoint Matching Methods
enum Mode{
    NCC_MODE,
    SSD_MODE,
    FLANN_MODE
};

const double INIT_VALUE = -2.0; // A negative value that is out of the possilbe range of scores for matching algorithms. Also, a starting index value for scoring system.
const int EPIP_THRESHOLD = 5;   // The two images are not exactly parallel, but we could assume that they are almost, so we allow a small offest from the epipolar line.

/**
 * @brief handles all image related functions
 * 
 */
class Images
{
private:
    //variables
    cv::Mat img_left;
    cv::Mat img_right;
    cv::Mat img_left_grey;
    cv::Mat img_right_grey;
    cv::Mat depth_map;
    cv::Mat disparity;
    std::vector<cv::KeyPoint> keypoints_left;   
    std::vector<cv::KeyPoint> keypoints_right;   
    std::vector<cv::KeyPoint> matched_left;   
    std::vector<cv::KeyPoint> matched_right;       
    cv::Mat L_descriptor;
    cv::Mat R_descriptor;
    //matching functions
    double NCC(const cv::Mat& l_window, const cv::Mat& r_window);
    double SSD(const cv::Mat& l_window, const cv::Mat& r_window);
    void FLANN(std::vector<cv::KeyPoint> keypoints_left, std::vector<cv::KeyPoint> keypoints_right);
    
    
public:
    //constructor
    Images(const std::string &, const std::string &, const std::string &);
    ~Images();
    //getter
    cv::Mat GetLeftImg();
    cv::Mat GetRightImg();
    cv::Mat GetLeftImgGrey();
    cv::Mat GetRightImgGrey();
    cv::Mat getDepthMap();
    cv::Mat getGroundTruthDepth();
    std::vector<cv::KeyPoint> GetLeftKeypoints();
    std::vector<cv::KeyPoint> GetRightKeypoints();
    std::vector<cv::KeyPoint> GetLeftMatched();
    std::vector<cv::KeyPoint> GetRightMatched();
    cv::Mat GetLeftDescriptor();
    cv::Mat GetRightDescriptor();
    cv::Mat getDisparity();
    //setter
    void SetPoints(std::vector<cv::KeyPoint> &keypoints_left, std::vector<cv::KeyPoint> &keypoints_right);
    void SetMatchedPoints(std::vector<cv::KeyPoint> &keypoints_left, std::vector<cv::KeyPoint> &keypoints_right);
    //functions
    void DetectKeypoints(const int &num_features); //detect keypoints using SIFT
    void Match(Mode mode);
    cv::Mat FindEssentialMatrix(const cv::Mat &intrinsics);
    void CalculateDepth(cv::Mat, CalibrationData);
    
};

#endif //IMAGES_H