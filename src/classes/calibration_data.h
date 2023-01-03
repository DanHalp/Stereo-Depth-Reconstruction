/**
 * @file calibration_data.h
 * @brief calibration_data class
 * 
*/

#ifndef CALIBRATION_DATA_H
#define CALIBRATION_DATA_H


class CalibrationData
{
private:
    const std::string _path;
public:
    cv::Mat cam0;   // camera 0 intrinsics 
                        // [f 0 cx; 0 f cy; 0 0 1], where
                        // f: focal length in pixels, cx, cy: principal point
    cv::Mat cam1;   // camera 1 intrinsics
    int doffs;
    double baseline;
    int width;
    int height;
    int ndisp;
    int vmin;
    int vmax;

    CalibrationData(const std::string &filelocation);

    const void PrintCalData();

};

#endif // CALIBRATION_DATA_H
