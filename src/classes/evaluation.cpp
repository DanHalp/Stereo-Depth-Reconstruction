/**
 * @file evaluation.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-08-01
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <iostream>
#include <pcl/impl/point_types.hpp>
#include <pcl/pcl_base.h>
#include "../tools/compute_hausdorff.cpp"
#include "evaluation.h"
#include <opencv2/viz/vizcore.hpp>
#include <opencv2/core/cvstd.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/io/ply_io.h>

Cloud::Cloud(){}
Cloud::~Cloud() = default;
std::string Cloud::getName(){
    return Cloud::_name;
}
void Cloud::setName(std::string name){
    Cloud::_name = name;
}
pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud::getCloud(){
    return Cloud::_cloud;
}
void Cloud::setCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    Cloud::_cloud = cloud;
}

Evaluation::Evaluation(){
    //
}

Evaluation::~Evaluation() = default;

/**
 * @brief load one point cloud from a ply file
 * 
 * @param filename name of the file
 * @param name custome name of the cloud 
 * 
 */
void Evaluation::loadCloud(std::string filename, std::string name){
    Cloud c;
    pcl::PointCloud<pcl::PointXYZ> new_cloud;
    cv::String cv_s(filename);

    cv::Mat inp = cv::viz::readCloud(cv_s);
    pcl::io::loadPLYFile<pcl::PointXYZ>(filename, new_cloud);
    //std::cout << inp << std::endl;
    
    //convert Mat to point cloud
    //TODO problem is probably the cloud size of 2.000.000 points
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    // for(int i=0;i<inp.cols;i=+3)
    // {
    //     pcl::PointXYZ point;
    //     point.x = inp.at<float>(i);
    //     point.y = inp.at<float>(i+1);
    //     point.z = inp.at<float>(i+2);
    //     point_cloud_ptr -> points.push_back(point);
    // }
    //point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
    //point_cloud_ptr->height = 1;

    // add cloud to vec of clouds 
    *point_cloud_ptr = new_cloud;
    c.setCloud(point_cloud_ptr);
    c.setName(name);
    Evaluation::_clouds.push_back(c);
}

/**
 * @brief evaluate all point clouds based on the Hausdorff distance 
 * 
 */
void Evaluation::evaluate(){
    //TODO not tested
    //copy clouds vector and remove first element
    std::vector<Cloud> clouds_to_compare = _clouds;
    clouds_to_compare.erase(clouds_to_compare.begin());

    // iterate over all possible pairs of point clouds
    for(int i = 0; i < _clouds.size()-1; i++){
        for(int k = 0; k < clouds_to_compare.size(); k++ ){
            // store results
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_a; 
            cloud_a = _clouds.at(i).getCloud();            
            std::cout << _clouds.at(i).getName() << std::endl;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_b; 
            cloud_b = clouds_to_compare.at(k).getCloud();
            std::cout  << clouds_to_compare.at(k).getName() << std::endl;
            std::vector<std::string> res;
            res.push_back(_clouds.at(i).getName());
            res.push_back(" ");
            res.push_back(clouds_to_compare.at(k).getName());
            res.push_back(" : ");
            std::vector<std::string> res_calc = compute(cloud_a, cloud_b);
            for (const auto &elem : res_calc){
                res.push_back(elem);
                res.push_back(" ");
            }
            _results.push_back(res);
        }
        clouds_to_compare.erase(clouds_to_compare.begin());
    }
}

std::vector<std::vector<std::string>> Evaluation::getResults(){
    return _results;
}

/**
 * @brief print results of the evaluation
 * 
 */
void Evaluation::printResult() {
    std::cout << "Method 1 || Method 2 || 1 -> 2 || 2 -> 1 || Hausdorff Distance" << std::endl;
    for(int i = 0; i < _results.size(); i++){
        for(int k = 0; k < _results.at(i).size(); k++){
            std::cout << _results.at(i).at(k);
        }
        std::cout << std::endl;
    }
}

