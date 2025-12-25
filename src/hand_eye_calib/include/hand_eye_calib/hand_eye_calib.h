#ifndef HAND_EYE_CALIB_H
#define HAND_EYE_CALIB_H

#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <iterator> 
#include <algorithm>  

using namespace cv;
using namespace std;

bool hand_eye_calib_calc(int num, 
					Mat& CalPose,
					Mat& ToolPose,
					cv::Mat& Homo_cam2gripper);

#endif /* HAND_EYE_CALIB_H */