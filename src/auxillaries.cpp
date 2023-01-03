/**
 * @file auxillaries.cpp
 * @brief Proviudes various auxillary functions
 * 
 * print()
 * computeMean()
 * estimateRotation()
 * computeTranslation()
 * distancePointLine()
 * drawEpipolarLines()
 * VisualizeMatching()
 * ReadImage()
 * rgbToGrey()
 * vec2mat()
 * mat2vec()
 * normalize()
 * checkRectification()
 * PrintVec()
 * 
 * @version 1.0
 * @date 2022-07-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <opencv2/opencv.hpp>
#include "classes/images.h"

/**
 * @brief print out log sentence
 */
void log(std::string s){
    std::cout << s << std::endl;
}


/**
 * @brief print any text
 * 
 * @tparam T 
 * @param msg message to print
 */
template<typename T>
void print(T const& msg)
{  
    std::cout << "   \n" << std::endl;
    std::cout << msg << std::endl;
    std::cout << " \n  " << std::endl;
}

/**
 * @brief Print two texts
 * 
 * @tparam T 
 * @param msg1 messaage 1 to print
 * @param msg2 messaage 2 to print
 */
template<typename T>
void print(T const& msg1, T const& msg2)
{
    std::cout << "   \n" << std::endl;
    std::cout << msg1 << "           " << msg2<< std::endl;
    std::cout << "   \n" << std::endl;

}

void computeMean(const cv::Mat &x, cv::Mat& row_mean) {
        cv::reduce(x, row_mean, 0, cv::REDUCE_AVG);
}

cv::Mat estimateRotation(const cv::Mat& sourcePoints, const cv::Mat& sourceMean, const cv::Mat& targetPoints, const cv::Mat& targetMean) {
    // TODO: Estimate the rotation from source to target points, following the Procrustes algorithm.
    // To compute the singular value decomposition you can use JacobiSVD() from Eigen.
    // Hint: You can initialize an Eigen matrix with "MatrixXf m(num_rows,num_cols);" and access/modify parts of it using the .block() method (see above).
    cv::Mat X;
    for (int i = 0; i < sourcePoints.rows; ++i) {
        X.push_back(sourcePoints.row(i) - sourceMean);
    }
    cv::Mat Y;
    
    for (int i = 0; i < targetPoints.rows; ++i) {
        Y.push_back(targetPoints.row(i) - targetMean);
    }
    cv::Mat m = (Y.t() * X);
    cv::Mat w, u, vt;
    
    cv::SVD::compute(m, w, u, vt);
    // Eigen::Matrix3d m_eigen;
    // cv::cv2eigen(m, m_eigen);
    // Eigen::JacobiSVD<Eigen::MatrixXf> svd(m_eigen, Eigen::ComputeFullU | Eigen::ComputeFullV);
    cv::Mat S = cv::Mat::eye(3, 3, CV_64F);  
   
    if (abs(cv::determinant(u) * cv::determinant(vt) - 1) > 0.01) {
        S.at<int>(2, 2) = -1;
    }
    cv::Mat rotation = (u * (S * vt));
    return rotation;
}

 cv::Mat computeTranslation(const cv::Mat& sourceMean, const  cv::Mat& targetMean, const cv::Mat& rotation) {

    cv::Mat translation = targetMean - (sourceMean * rotation.t());
    return translation.t();
}

template <typename T>
static float distancePointLine(const cv::Point_<T> point, const cv::Vec<T,3>& line)
{
  //Line is given as a*x + b*y + c = 0
  return std::fabs(line(0)*point.x + line(1)*point.y + line(2))
      / std::sqrt(line(0)*line(0)+line(1)*line(1));
}

/**
 * \brief Compute and draw the epipolar lines in two images
 *      associated to each other by a fundamental matrix
 *
 * \param title     Title of the window to display
 * \param F         Fundamental matrix
 * \param img1      First image
 * \param img2      Second image
 * \param points1   Set of points in the first image
 * \param points2   Set of points in the second image matching to the first set
 * \param inlierDistance      Points with a high distance to the epipolar lines are
 *                not displayed. If it is negative, all points are displayed
 **/
template <typename T2>
static void drawEpipolarLines(const std::string& s, const cv::Mat& F,
                const cv::Mat& img1, const cv::Mat& img2,
                const std::vector<cv::Point_<T2>> points1,
                const std::vector<cv::Point_<T2>> points2,
                int num_points=10,
                const int inlierDistance = -1)
{

    std::string title = s;
    // cv::Matx<T1,3,3> F((double*)Fundemental.ptr());

    CV_Assert(img1.size() == img2.size() && img1.type() == img2.type());
    cv::Mat outImg(img1.rows, img1.cols*2, CV_8UC3);
    cv::Rect rect1(0,0, img1.cols, img1.rows);
    cv::Rect rect2(img1.cols, 0, img1.cols, img1.rows);
    /*
    * Allow color drawing
    */
    if (img1.type() == CV_8U)
    {
    cv::cvtColor(img1, outImg(rect1), cv::COLOR_GRAY2BGR);
    cv::cvtColor(img2, outImg(rect2), cv::COLOR_GRAY2BGR);
    }
    else
    {
    img1.copyTo(outImg(rect1));
    img2.copyTo(outImg(rect2));
    }

    std::vector<cv::Vec<T2,3>> epilines1, epilines2;
    cv::computeCorrespondEpilines(points1, 1, F, epilines1); //Index starts with 1
    cv::computeCorrespondEpilines(points2, 2, F, epilines2);
    CV_Assert(points1.size() == points2.size() &&
        points2.size() == epilines1.size() &&
        epilines1.size() == epilines2.size());


    cv::RNG rng(0);
    int thickness = 2;
    int radius = 9;
    if (num_points > (int)points1.size()) {num_points = (int)points1.size();}
    for(int i=0; i < num_points; i++)
    {
    if(inlierDistance > 0)
    {
        if(distancePointLine(points1[i], epilines2[i]) > inlierDistance ||
        distancePointLine(points2[i], epilines1[i]) > inlierDistance)
        {
        //The point match is no inlier
        continue;
        }
    }
    /*
        * Epipolar lines of the 1st point set are drawn in the 2nd image and vice-versa
        */
    cv::Scalar color(rng(256),rng(256),rng(256));
    // print(epilines1[i][0] / epilines1[i][1]);
    cv::line(outImg(rect2),
        cv::Point(0,-epilines1[i][2]/epilines1[i][1]),
    //   cv::Point(img1.cols,-epilines1[i][2]/epilines1[i][1]),
        cv::Point(img1.cols,-(epilines1[i][2]+epilines1[i][0]*img1.cols)/epilines1[i][1]),
        color, thickness);
    cv::circle(outImg(rect1), points1[i], radius, color, cv::FILLED, cv::LINE_8);


    cv::line(outImg(rect1),
        cv::Point(0,-epilines2[i][2]/epilines2[i][1]),
        cv::Point(img2.cols,-(epilines2[i][2]+epilines2[i][0]*img2.cols)/epilines2[i][1]),
    //   cv::Point(img2.cols,-epilines2[i][2]/epilines2[i][1]),
        color, thickness);
    cv::circle(outImg(rect2), points2[i], radius, color, cv::FILLED, cv::LINE_8);
    }
    cv::imwrite(title, outImg);
}
 
/**
 * @brief visualize point matching
 * 
 * @param img Images object containing the images and points
 */
void VisualizeMatching(Images img) {
    int width = 1920, height = 1080;
    double thickness = 2;

    cv::Mat image(height, width * 2, CV_8UC3);
    cv::hconcat(img.GetLeftImg(), img.GetRightImg(), image);

    auto orig_right = img.GetRightKeypoints();
    auto orig_left = img.GetLeftKeypoints();
    for (size_t i = 0; i < orig_right.size(); i++)
    {   
        orig_right[i].pt.x += width;
        cv::circle(image, orig_right[i].pt, 5, cv::Scalar(0, 255, 255), cv::FILLED, cv::LINE_8);
        cv::circle(image, orig_left[i].pt, 5, cv::Scalar(0, 255, 255), cv::FILLED, cv::LINE_8);
    }
    
    
    std::vector<cv::KeyPoint> points_left = img.GetLeftMatched();
    std::vector<cv::KeyPoint> points_right = img.GetRightMatched();
    for (size_t i = 0; i < img.GetRightMatched().size(); i++)
    {   
        cv::KeyPoint p = points_right[i];
        p.pt.x += width;
        points_left.push_back(p);
        cv::line(image, points_left[i].pt, points_left[i + points_right.size()].pt, cv::Scalar(0, 0, 255), thickness, cv::LINE_8);
        cv::circle(image, points_left[i].pt, 5, cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_8);
        cv::circle(image, points_left[i + points_right.size()].pt, 5, cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_8);
 

    }
    cv::imwrite("output.png", image);
    print("########### Saved visualziation of matching points.");
}


cv::Mat ReadImage(const std::string &filename, const cv::ImreadModes imreadmode= cv::IMREAD_COLOR){
    return cv::imread(filename, imreadmode);
}


cv::Mat rgbToGrey(const cv::Mat& img){
    cv::Mat grey;
    cv::cvtColor(img, grey, cv::COLOR_BGR2GRAY);
    return grey;
}

cv::Mat vec2mat(std::vector<cv::Point2f>& v, int type=CV_64FC1) {
    cv::Mat ret(v);
    ret.convertTo(ret, type); 
    ret = ret.reshape(1, v.size());
    return ret;
}

std::vector<cv::Point2f> mat2vec(cv::Mat& m, int size) {
    std::vector<cv::Point2f> v;
    for (int i = 0; i < size; i++)
    {
        v.push_back(cv::Point2f(m.at<double>(i, 0), m.at<double>(i, 1)));
    }
    return v;
}

cv:: Mat normalize(const cv::Mat &X, double currMin=0., double currMax=255., double newMin=0., double newMax=1.) {
    return ((X - cv::Scalar(currMin)) / cv::Scalar(currMax - currMin)) * (newMax - newMin) + newMin;

}

/**
 * @brief check, if the rectification was successful based on the produced output.png image
 * 
 * @param matched_left left matched keypoints
 * @param matched_right right matched keypoints
 * @param essential_mat_cv essential Matrix
 * @param Cam0cv left camera intrinsics
 * @param Cam1cv right camera intrinsics
 * @param img_left_grey lefr greyscaled image
 * @param img_right_grey right greyscaled image
 * @param R1 R1 Matrix computed by stereoRectify
 * @param R2 R2 Matrix computed by stereoRectify
 * @param P1 P1 Matrix computed by stereoRectify
 * @param P2 P2 Matrix computed by stereoRectify
 * @param map1 rectified left image
 * @param map2 rectified right image
 */
void checkRectification(std::vector<cv::KeyPoint> matched_left, std::vector<cv::KeyPoint> matched_right, cv::Mat essential_mat_cv, 
        cv::Mat Cam0cv, cv::Mat Cam1cv, cv::Mat img_left_grey, cv::Mat img_right_grey, cv::Mat R1, cv::Mat R2, cv::Mat P1, cv::Mat P2,
        cv::Mat map1, cv::Mat map2){

    cv::Mat dist_coeffs = (cv::Mat1d(1,4) << 0,0,0,0);;
    std::vector<cv::Point2f> mr, ml;
    cv::KeyPoint::convert(matched_left, ml);
    cv::KeyPoint::convert(matched_right, mr);

    // Show the epipolar lines before the rectification.
    std::string s = "Orig_Epipolar_Lines.png";
    auto F = Cam0cv.t().inv() * essential_mat_cv * Cam1cv.inv();
    drawEpipolarLines(s, F, img_left_grey, img_right_grey, ml, mr, 15, -1);

    // Rotate the points to match the corresponding rectified images.
    cv::Mat mat_ml, mat_mr, temp, temp2;
    cv::undistortPoints(ml, mat_ml, Cam0cv, dist_coeffs, R1, P1);
    cv::undistortPoints(mr, mat_mr, Cam1cv, dist_coeffs, R2, P2);
    mat_ml.convertTo(mat_ml, CV_64FC1); mat_mr.convertTo(mat_mr, CV_64FC1);  // To avoid overflow... I hate this language. Can't even describe how much.
    
    // Calculate the new Essential matrix
    std::vector<cv::Point2f> ml2, mr2;
    ml2 = mat2vec(mat_ml, mat_ml.rows);
    mr2 = mat2vec(mat_mr, mat_mr.rows);
    cv::Mat new_e = cv::findEssentialMat(ml2, mr2, P1(cv::Range(0, 3), cv::Range(0, 3)), cv::FM_8POINT);

    s ="Rectified_Images.png";
    F = P1(cv::Range(0, 3), cv::Range(0, 3)).t().inv() * new_e * P2(cv::Range(0, 3), cv::Range(0, 3)).inv();
    //drawEpipolarLines(s, F, disparity_left, disparity_right, ml2, mr2, ml2.size(), -1);
    drawEpipolarLines(s, F, map1, map2, ml2, mr2, ml2.size(), -1);
}

/**
 * @brief print all the elements in a vector
 *        just for testing and convinence
 * @param vec : vector to print
*/
template<typename T>
void PrintVec(const std::vector<T> &vec){
    for (auto &elem: vec){
        std::cout << elem << std::endl;
    }
}