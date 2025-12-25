#ifndef HOLE_DETECTOR_H
#define HOLE_DETECTOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <sensor_msgs/CameraInfo.h> // Remove ROS1 header
#include <sensor_msgs/msg/camera_info.hpp> // Add ROS2 header
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

bool holeDetector(cv::Mat src, cv::Mat depthInfo, sensor_msgs::msg::CameraInfo camInfo, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_holes);

#endif