#ifndef NORM_CALC_H
#define NORM_CALC_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// 声明处理函数
bool processPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,
                       pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_out,
                       float x_min, float x_max, float y_min, float y_max,
                       float z_min, float z_max, float search_radius,
                       int search_num_th);

#endif