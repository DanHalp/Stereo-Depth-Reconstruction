/**
 * @file images.cpp
 * @brief images.h implementation
 * 
 */

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/core.hpp>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include "calibration_data.h"
#include "../auxillaries.cpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>

// ###################

#define IMAGES_IMPLEMENTATION
#include "images.h"


//Constructor
Images::Images(const std::string &imgLfileloc, const std::string &imgRfileloc, const std::string &gt_disparity):
    img_left(ReadImage(imgLfileloc)), img_right(ReadImage(imgRfileloc))
{
    Images::img_left_grey = rgbToGrey(img_left);
    Images::img_right_grey = rgbToGrey(img_right);
}


//Destructor
Images::~Images() = default;

//Public
//Getter functions
cv::Mat Images::GetLeftImg(){
    return Images::img_left;
}
cv::Mat Images::GetRightImg(){
    return Images::img_right;
}
cv::Mat Images::GetLeftImgGrey(){
    return Images::img_left_grey;
}
cv::Mat Images::GetRightImgGrey(){
    return Images::img_right_grey;
}
cv::Mat Images::getDepthMap(){
    return depth_map;
}
std::vector<cv::KeyPoint> Images::GetLeftKeypoints(){
    return Images::keypoints_left;
}
std::vector<cv::KeyPoint> Images::GetRightKeypoints(){
    return Images::keypoints_right;
}
std::vector<cv::KeyPoint> Images::GetLeftMatched(){
    return Images::matched_left;
}
std::vector<cv::KeyPoint> Images::GetRightMatched(){
    return Images::matched_right;
}
cv::Mat Images::GetLeftDescriptor(){
    return Images::L_descriptor;
}
cv::Mat Images::GetRightDescriptor(){
    return Images::R_descriptor;
}
cv::Mat Images::getDisparity(){
    return Images::disparity;
}


//Setter functions
void Images::SetPoints(std::vector<cv::KeyPoint> &keypoints_left, std::vector<cv::KeyPoint> &keypoints_right){
    Images::keypoints_left = keypoints_left;
    Images::keypoints_right = keypoints_right;
}
void Images::SetMatchedPoints(std::vector<cv::KeyPoint> &keypoints_left, std::vector<cv::KeyPoint> &keypoints_right){
    Images::matched_left = keypoints_left;
    Images::matched_right = keypoints_right;
}

//Functions
/**
 * @brief Detect keypoints in two stereo images using SIFT
 * 
 * @param num_features Number of keypoints to detect
 */
void Images::DetectKeypoints (const int &num_features){
    cv::Ptr<cv::SIFT> siftPtr = cv::SIFT::create(num_features);
    siftPtr->detectAndCompute(img_left_grey, cv::noArray(), Images::keypoints_left, Images::L_descriptor);
    siftPtr->detectAndCompute(img_right_grey, cv::noArray(), Images::keypoints_right, Images::R_descriptor);
}
    

/**
 * @brief Apply the point matching method Of choice
 * 
 * @param mode The selected point matching method (NCC, SSD, FLANN)
 */
void Images::Match(Mode mode){ 

    // Create also arrays for matched points, so the original ones could be visualized and used again.
    matched_right.clear();
    matched_left.clear();

    std::vector<cv::KeyPoint> keypoints_left = Images::keypoints_left;
    std::vector<cv::KeyPoint> keypoints_right = Images::keypoints_right;

    if(mode == FLANN_MODE){
        FLANN(keypoints_left, keypoints_right);
        std::cout << "matched :" << matched_left.size() << std::endl;
        return;
    }

    // Work on grayscale images.
    cv::Mat image_left = Images::img_left_grey;
    cv::Mat image_right = Images::img_right_grey;

    // Variables. l_scores and r_scores are meant to keep the best scores and corresponding indices of the matched points from the other image, for both images.
    std::vector<std::pair<int, double>> r_scores(keypoints_right.size(), std::make_pair((int)INIT_VALUE, INIT_VALUE));  // Negative values to show that no matching point was found yet.
    std::vector<std::pair<int, double>> l_scores(keypoints_left.size(), std::make_pair((int)INIT_VALUE, INIT_VALUE));  // Negative values to show that no matching point was found yet.
    int l_rows = image_left.rows, l_cols = image_left.cols, r_rows = image_right.rows, r_cols = image_right.cols;
    int l_h = 0, l_w = 0, l_h_offset = 0, l_w_offset = 0, r_h = 0, r_w = 0, r_h_offset = 0, r_w_offset = 0;
    double c;

    // Each mode requires different hyperparameters.
    int windows_radius = 30;
    int windows_size = windows_radius * 2 + 1;
    double threshold = 0.98;

    if (mode == NCC_MODE) {
        windows_radius = 30;
        windows_size = windows_radius * 2 + 1;
        threshold = 0.75;
    }
    else if (mode == SSD_MODE) {
        windows_radius = 20;
        windows_size = windows_radius * 2 + 1;
        threshold = 0.999;
    }

    // First decide if a point's corresponding window is within the image.
    cv::Point2f curr_l = keypoints_left[0].pt, curr_r = keypoints_right[0].pt;
    cv::Mat l_window, r_window;
    for(int i = 0; i < keypoints_left.size(); ++i) {
        curr_l = keypoints_left[i].pt;
        l_h = curr_l.y - windows_radius; l_w = curr_l.x - windows_radius;
        l_h_offset = l_h + windows_size; l_w_offset = l_w + windows_size;

        // Is the window within the current image?
        if (l_h >= 0 && l_w >= 0 && l_h_offset < l_rows && l_w_offset < l_cols){

            for(int j = 0; j < keypoints_right.size(); ++j) {

                curr_r = keypoints_right[j].pt;
                r_h = curr_r.y - windows_radius; r_w = curr_r.x - windows_radius;
                r_h_offset = r_h + windows_size; r_w_offset = r_w + windows_size;

                // Is the window within the current image? NOTE: Since in this project we assume that there were nelegible rotation and translation, we could assume that
                // corresponding points are on the same epipolar lines (In approximation...). Thus, we could check a much smaller number of correspondences, to boost performance.
                if (r_h >= 0 && r_w >= 0 && r_h_offset < r_rows && r_w_offset < r_cols && abs(curr_r.y - curr_l.y) < EPIP_THRESHOLD) {
                    
                    l_window = image_left(cv::Range(l_h, l_h_offset), cv::Range(l_w, l_w_offset)); 
                    r_window = image_right(cv::Range(r_h, r_h_offset), cv::Range(r_w, r_w_offset));
                    l_window.convertTo(l_window, CV_64F); r_window.convertTo(r_window, CV_64F);  // Otherwise, values are rounded and clipped to the range of [0, 255].


                    if (mode == NCC_MODE) {
                        c = NCC(l_window, r_window);
                    }
                    
                    else if (mode == SSD_MODE) {
                        c = SSD(l_window, r_window);
                    }

                    // If "c" is better than the current best_score for a match for the current points from the left and right images.
                    // If the point from the right images was already coupled with some point from the left one, remove correspondece from the left point.
                    if (r_scores[j].second < c && l_scores[i].second < c) {
                        if (r_scores[j].first != INIT_VALUE) {
                            l_scores[r_scores[j].first].second = INIT_VALUE;
                        }
                        l_scores[i] = std::make_pair(j, c);
                        r_scores[j] = std::make_pair(i, c);
                    }
                }
            }
            
        }
    }
   
    // Return the matches from left and right images <point in left image, point in right image>
    for (unsigned k = 0; k < r_scores.size(); ++k) {

        if (r_scores[k].second >= threshold && k == l_scores[r_scores[k].first].first) { 
            matched_left.push_back(keypoints_left[r_scores[k].first]);
            matched_right.push_back(keypoints_right[k]);
            
        }
            
    }
    std::cout << "matched : "<<matched_left.size() << std::endl;
    
}

double Images::NCC(const cv::Mat& l_window, const cv::Mat& r_window){
    cv::Scalar l_mean = cv::mean(l_window), r_mean = cv::mean(r_window);
    cv::Mat l = l_window, r = r_window;
    l -= l_mean; r -= r_mean;
    cv::Scalar t = (cv::sum(l.mul(r))) / (cv::norm(l, cv::NORM_L2) * cv::norm(r, cv::NORM_L2));
    return t[0];
}

double Images::SSD(const cv::Mat& l_window, const cv::Mat& r_window){
    cv::Mat l = normalize(l_window), r = normalize(r_window);
    cv::Scalar l_mean = cv::mean(l), r_mean = cv::mean(r);
    int m = l_window.cols;
    cv::Scalar t = cv::norm(l - r + r_mean - l_mean, cv::NORM_L2);
    return 1.0 - (t[0] / (m*m));
}  

/**
 * @brief FLANN keypoint matching algorithm
 * 
 * @param keypoints_left Keypoints of the left image
 * @param keypoints_right Keypoints of the right image
 * adapted from https://docs.opencv.org/3.4/d5/d6f/tutorial_feature_flann_matcher.html
 */
void Images::FLANN(std::vector<cv::KeyPoint> keypoints_left, std::vector<cv::KeyPoint> keypoints_right){

    std::vector<cv::KeyPoint> matchedLeft; 
    std::vector<cv::KeyPoint> matchedRight;

    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    std::vector< std::vector<cv::DMatch> > knn_matches;
    matcher->knnMatch( Images::L_descriptor, Images::R_descriptor, knn_matches, 2 );

    // Filter matches using the Lowe's ratio test
    const float ratio_thresh = 0.7f;
    std::vector<cv::DMatch> good_matches;
    for (size_t i = 0; i < knn_matches.size(); i++)
    {
        if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
        {
            good_matches.push_back(knn_matches[i][0]);
        }
    }

    //convert back to points
    for(auto i = 0; i < good_matches.size(); i++){
        matchedLeft.push_back(keypoints_left[good_matches[i].queryIdx]);
        matchedRight.push_back(keypoints_right[good_matches[i].trainIdx]);
    }
    
    // return matched point;
    Images::SetMatchedPoints(matchedLeft, matchedRight);
}

/**
 * @brief get Essential Matrix
 * 
 * @param intrinsics Camera intrinsics 
 * @return cv::Mat Essential Matrix
 */
cv::Mat Images::FindEssentialMatrix(const cv::Mat &intrinsics){
    std::vector<cv::Point2f> keypoints_left_vec(matched_left.size());
    std::vector<cv::Point2f> keypoints_right_vec(matched_right.size());

    cv::KeyPoint::convert(matched_left, keypoints_left_vec);
    cv::KeyPoint::convert(matched_right, keypoints_right_vec);

    return cv::findEssentialMat(keypoints_left_vec, keypoints_right_vec, intrinsics, cv::LMEDS);
}

/**
 * @brief Calculate the depth map
 * 
 * @param essential_mat_cv Essential matrix 
 * @param cal_data calibration data of the camera
 */
void Images::CalculateDepth(cv::Mat essential_mat_cv, CalibrationData cal_data){

    //output Mat
    cv::Mat R1, R2, P1, P2, Q;
    //input
    cv::Size img_size = Images::img_left.size(); 
    cv::Mat dist_coeffs = (cv::Mat1d(1,4) << 0,0,0,0);

    cv::Mat R, t;
    std::vector<cv::Point2f> keypoints_left_vec(matched_left.size());
    std::vector<cv::Point2f> keypoints_right_vec(matched_right.size());

    cv::KeyPoint::convert(matched_left, keypoints_left_vec);
    cv::KeyPoint::convert(matched_right, keypoints_right_vec);

    //get R, t
    cv::recoverPose(essential_mat_cv, keypoints_left_vec, keypoints_right_vec, cal_data.cam0, R, t); 

    //calculate Q
    cv::stereoRectify(cal_data.cam0, dist_coeffs, cal_data.cam1, dist_coeffs, img_size, R, t, R1, R2, P1, P2, Q); //, cv::CALIB_ZERO_DISPARITY, -1);

    //calculate disparity
    cv::Ptr<cv::StereoSGBM> matcher = cv::StereoSGBM::create(cal_data.vmin, cal_data.ndisp, 5, 0, 0, 0, 1, 2, 50, 2, cv::StereoSGBM::MODE_HH);

    //rectify images
    cv::Mat new_cam_mat0 = cv::getOptimalNewCameraMatrix(cal_data.cam0, dist_coeffs, img_size, 0);
    cv::Mat new_cam_mat1 = cv::getOptimalNewCameraMatrix(cal_data.cam1, dist_coeffs, img_size, 0);

    cv::Mat out_left_map1, out_left_map2, out_right_map1, out_right_map2;
    cv::initUndistortRectifyMap(new_cam_mat0, dist_coeffs, R1, P1, img_size, CV_32FC1, out_left_map1, out_left_map2);
    cv::initUndistortRectifyMap(new_cam_mat1, dist_coeffs, R2, P2, img_size, CV_32FC1, out_right_map1, out_right_map2);

    cv::Mat rectified_left, rectified_right;

    int interpolation = cv::INTER_LINEAR;
    int border = cv::BORDER_TRANSPARENT;
    cv::remap(img_left_grey, rectified_left, out_left_map1, out_left_map2, interpolation, border);
    cv::remap(img_right_grey, rectified_right, out_right_map1, out_right_map2, interpolation, border);

    // compute disparity
    cv::Mat disparity; 
    matcher->compute(rectified_left, rectified_right, disparity);

    disparity.convertTo(disparity, CV_32FC1);
    Images::disparity = disparity;

    //create non all-white/black visualization of the disparity map
    //cv::imwrite("Disparity.png", disparity/128);

    //reproject disparity
    cv::Mat depthimg;
    cv::reprojectImageTo3D(disparity, depthimg, Q);
    depth_map = depthimg;

    //cv::imwrite("Depthmap.png", depth_map);

    //checkRectification(matched_left, matched_right, essential_mat_cv, cal_data.cam0, cal_data.cam1, img_left_grey, img_right_grey, R1, R2, P1, P2, rectified_left, rectified_right);
    return;
}

