/**
 * @file 3d_reconstruction.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-07-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include "classes/images.h"
#include <opencv2/viz/vizcore.hpp>
#include <opencv2/imgproc.hpp>

/**
 * @brief 
 * 
 * @param img Image object containing image information
 * @param filename Filename for the output image
 */
void depthMapToPointCloud(Images img, std::string filename){

    //TODO apply filtering here
    cv::Mat depth_map = img.getDepthMap();

    //cv::threshold(depth_map, depth_map, 1, 0, cv::THRESH_TOZERO_INV);

    cv::viz::writeCloud(filename, img.getDepthMap(), img.GetLeftImg());
    //cv::viz::writeCloud("ground_truth.ply", img.getGroundTruthDepth(), img.GetLeftImg());

    return;
}