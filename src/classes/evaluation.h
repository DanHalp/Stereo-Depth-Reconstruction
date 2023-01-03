/**
 * @file evaluation.hpp
 * @brief evaluate distacne between point cloud
 * 
 */

#include <iostream>
#include <pcl/impl/point_types.hpp>
#include <pcl/pcl_base.h>

#ifndef EVALUATION_HPP
#define EVALUATION_HPP

class Cloud
{
    private:
        std::string _name;
        pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud;
    public:
        Cloud();
        ~Cloud();
        std::string getName();
        void setName(std::string);
        pcl::PointCloud<pcl::PointXYZ>::Ptr getCloud();
        void setCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr);
};

class Evaluation
{
    private:
        std::vector<Cloud> _clouds;
        std::vector<std::vector<std::string>> _results;
    public:
        Evaluation();
        ~Evaluation();
        void loadCloud(std::string filename, std::string name);
        void evaluate();
        std::vector<std::vector<std::string>> getResults();
        void printResult();

};


#endif // EVALUATION_HPP