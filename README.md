
# 3DSMC-project

Stereo reconstruction project for the 3D Scanning and Motion Capture Lecture at TUM in the summer semester 2022.
This program creates a 3D reconstruction from two images of a scene take from a different point of view. It compares keypoint matching algorithm - FLANN, NCC, NSSD - by comparing the Hausdorff Distance between two reconsturcted point clouds. 

## Libraries

Following libraries are required to run the program:
Eigen - https://eigen.tuxfamily.org/index.php?title=Main_Page  
OpenCV - https://opencv.org/  
PCL - https://pointclouds.org/  
 
## Dataset

We are using the Chess images from the middlebury dataset (https://vision.middlebury.edu/stereo/data/). From here, two datasets are used *artroom1, chess2*

## Usage
To run the program, two inputs are needed

- a choice between dataset **chess** and **artroom**
- number of keypoints

an example of how to run the program is shown in the section *Compilation*

The program outputs 4 point clouds obtained using different keypoint matching method such as FLANN, NCC, SSD
- reconstruction_flann.ply   : keypoints matched using FLANN
- reconstruction_ncc.ply : keypoints matched using NCC
- reconstruction_ssd.ply : keypoints matched using SSD
- reconstruction_ground_truth.ply

Additionally, it gives a comparison between the keypoint matching method by returning the Hausdorff Distance between two reconsturcted point clouds


## Compilation example
~~~bash
mkdir build
cd build
cmake ..
make
./stereo chess 1000
~~~

## Sample output

The output describes the distance between two point clouds and the Hausdorff distance

Method 1 || Method 2 || 1 -> 2 || 2 -> 1 || Hausdorff Distance
NCC FLANN : 0.122338 0.134659 0.134659 
NCC SSD : 0.140634 0.134371 0.140634 
NCC GroundTruth : 0.183123 0.139710 0.183123 
FLANN SSD : 0.115069 0.140643 0.140643 
FLANN GroundTruth : 0.150245 0.115853 0.150245 
SSD GroundTruth : 0.143259 0.116161 0.143259 

## Roadmap
1. Week1 (June 18 - June 24)  
    • Set up coding environment  
    • Research related papers/works to define our method (see results for more details)  
2. Week2 (June 25 - July 1)  
    • Implementation of the 8-point-algorithm to calculate the extrinsic parameters  
3. Week3 (July 2 - July 8)  
    • Implementation of multiple matching algorithms used for comparison.
4. Week4 (July 9 - July 15)  
    • Implementation of the triangulation  
    • Computing the disparity map  
    • Computing the depth map  
5. Week5 (July 16 - July 22)  
    • Comparison of the matching algorithms by comparison of our depth map with the given ground truth depth map  
    • Creation of a 3D scene model based on the depth map  
6. Week6 (July 23 - July 29)  
    • additional week to finish unsolved problems  
    • Preparation of the project video/poster  

## Authors and acknowledgment
Dan Halperin, Hayoung Kim, Sebastian Steinmüller