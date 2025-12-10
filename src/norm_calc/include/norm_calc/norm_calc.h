#ifndef NORM_CALC_H
#define NORM_CALC_H

#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "rclcpp/rclcpp.hpp"
#include "norm_calc/chisel_box.h"
#include "norm_calc/edge_grid.h"

// #include <pcl/point_cloud.h>
// #include <pcl/console/parse.h>
// #include <pcl/common/transforms.h>
// #define EDGE_CHISEL_MODE         // 边缘凿击模式
#define EDGE_ROW 20
#define EDGE_COL 10

bool normCalc(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_holes,
              pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
              pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downSampled,
              pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered,
              pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_smoothed,
              pcl::PointCloud<pcl::Normal>::Ptr cloud_normals,
              pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals,
              pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_shrink,
              pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_tarPoint,
              pcl::PolygonMesh::Ptr triangles,
              chisel_box::ChiselNormPoint *tarPointList,
              chisel_box::ChiselParam chiselParam,
              edge_grid::GridParam gridParam);

#endif /* NORM_CALC_H */ 
