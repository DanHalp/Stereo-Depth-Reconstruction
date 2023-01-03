#include <opencv2/opencv.hpp>
#include <pcl/console/parse.h>
#include <fstream>
#include <string>
#include "classes/calibration_data.cpp"
#include <iostream>
#include <vector>
#include "classes/images.cpp"
#include "3d_reconstruction.cpp"
#include "classes/evaluation.cpp"
#include <pcl/io/ply_io.h>

//run the pipeline
int main(int argc, char* argv[]){

    //set vars
    int num_features;
    std::string im0,im1,calib,gt; 
    double cx,cy,f;
    //react co command line input
    // usage: ./stereo [dataset chess || artroom] [Nr. Keypoints]
    if(argc != 3){
        log("Invalid Arguments");
        log("Usage: ./stereo [dataset] [nr. keypoints]");
        log("dataset: chess || artroom");
        return 1;
    }

    std::string dataset = argv[1];
    if(dataset.compare("chess") == 0){
        //load chess data
        im0 = "../data/chess2/im0.png";
        im1 = "../data/chess2/im1.png";
        calib = "../data/chess2/calib.txt";
        gt = "../data/chess2/disp0.pfm";
        cx = -872.36;
        cy = -552.32;
        f = 1758.23;
    } else if (dataset.compare("artroom") == 0){
        //load artroom data
        im0 = "../data/artroom1/im0.png";
        im1 = "../data/artroom1/im1.png";
        calib = "../data/artroom1/calib.txt";
        gt = "../data/artroom1/disp0.pfm";
        cx = -792.27;
        cy = -541.89;
        f = 1733.74;
    } else {
        log("Invalid Dataset: choose between 'chess' or 'artroom'");
        return 1;
    }

    CalibrationData caldata = CalibrationData(calib);
    Images img = Images(im0, im1, gt);
    double dataQ[16] = { 1., 0., 0., cx, 0., 1., 0., cy, 0., 0., 0., f, 0., 0., 1.,0.};

    int num_points = std::atoi(argv[2]);
    if(num_points <= 0 || num_points > 10000){
        log("Number of keypoints out of range! Choose a number between  1 and 10.000");
        return 1;
    } else {
        num_features = num_points;
    }

    //setup
    log("Setup Pipeline");
    img.DetectKeypoints(num_features);
    log("Setup finished");

    //Create FLANN based point cloud
    log("### FLANN ###");
    img.Match(FLANN_MODE);
    cv::Mat essential_matrix = img.FindEssentialMatrix(caldata.cam0);
    img.CalculateDepth(essential_matrix, caldata);
    depthMapToPointCloud(img, "reconstruction_flann.ply");
    log("Flann finished");

    //Create NCC based point cloud
    log("### NCC ###");
    img.Match(NCC_MODE);
    essential_matrix = img.FindEssentialMatrix(caldata.cam0);
    img.CalculateDepth(essential_matrix, caldata);
    depthMapToPointCloud(img, "reconstruction_ncc.ply");
    log("NCC finished");

    //Create SSD based point cloud
    log("### SSD ###");
    img.Match(SSD_MODE);
    essential_matrix = img.FindEssentialMatrix(caldata.cam0);
    img.CalculateDepth(essential_matrix, caldata);
    depthMapToPointCloud(img, "reconstruction_ssd.ply");
    log("SSD finished");
    
    //Create Ground Truth based point cloud
    log("### Ground Truth ###");
    cv::Ptr<cv::StereoSGBM> matcher = cv::StereoSGBM::create(caldata.vmin, caldata.ndisp, 5, 0, 0, 0, 1, 2, 50, 2, cv::StereoSGBM::MODE_HH);
    cv::Mat disp_gt;
    matcher->compute(img.GetLeftImgGrey(), img.GetRightImgGrey(), disp_gt);
    // Q matrix
    cv::Mat Q_gt = cv::Mat(4, 4, CV_64F, dataQ);
    cv::Mat depthimg;
    cv::reprojectImageTo3D(disp_gt, depthimg, Q_gt);
    cv::viz::writeCloud("reconstruction_ground_truth.ply", depthimg, img.GetLeftImg());
    log("Ground Truth finished");

    //evaluate
    Evaluation eval;
    eval.loadCloud("reconstruction_ncc.ply", "NCC");
    eval.loadCloud("reconstruction_flann.ply", "FLANN");
    eval.loadCloud("reconstruction_ssd.ply", "SSD");
    eval.loadCloud("reconstruction_ground_truth.ply", "GroundTruth");

    eval.evaluate();

    eval.printResult();

    //debug printout
    //VisualizeMatching(img);

    return 0;
}

