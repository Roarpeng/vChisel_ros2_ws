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

// 标定质量评估结果结构
struct CalibrationQuality {
	double reprojection_error;  // 重投影误差 (mm)
	double consistency_score;   // 一致性评分 (0-100)
	double rotation_error;      // 旋转误差 (度)
	double translation_error;   // 平移误差 (mm)
	int quality_grade;          // 质量等级: 0=优秀, 1=良好, 2=一般, 3=较差
	std::string quality_message;
};

bool hand_eye_calib_calc(int num,
					Mat& CalPose,
					Mat& ToolPose,
					cv::Mat& Homo_cam2gripper);

// 带质量评估的手眼标定函数
bool hand_eye_calib_calc_with_quality(int num,
					Mat& CalPose,
					Mat& ToolPose,
					cv::Mat& Homo_cam2gripper,
					CalibrationQuality& quality);

// 单独计算标定质量
CalibrationQuality evaluate_calibration_quality(
	const std::vector<cv::Mat>& Homo_gripper2base,
	const std::vector<cv::Mat>& Homo_target2cam,
	const cv::Mat& Homo_cam2gripper);

// 计算质量评分 (0-100)
int calculate_quality_score(const CalibrationQuality& quality);

#endif /* HAND_EYE_CALIB_H */