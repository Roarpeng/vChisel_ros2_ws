#include "norm_calc/norm_calc.h"
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

// 纯函数：输入原始RGB点云 -> 输出带法向的平滑点云
bool processPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,
                       pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_out,
                       float x_min, float x_max, float y_min, float y_max,
                       float z_min, float z_max, float search_radius,
                       int search_num_th) {
  if (cloud_in->empty())
    return false;

  // 1. 直通滤波 (ROI裁剪)
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pass(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(cloud_in);

  pass.setFilterFieldName("z");
  pass.setFilterLimits(z_min, z_max);
  pass.filter(*cloud_pass);

  pass.setInputCloud(cloud_pass);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(x_min, x_max);
  pass.filter(*cloud_pass);

  pass.setInputCloud(cloud_pass);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(y_min, y_max);
  pass.filter(*cloud_pass);

  if (cloud_pass->empty())
    return false;

  // 2. 体素下采样 (降采样，提高速度)
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_voxel(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud(cloud_pass);
  sor.setLeafSize(0.003f, 0.003f, 0.003f); // 3mm 精度
  sor.filter(*cloud_voxel);

  // 3. 统计滤波 (去除飞点/噪点)
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> stat;
  stat.setInputCloud(cloud_voxel);
  stat.setMeanK(50);
  stat.setStddevMulThresh(1.0);
  stat.filter(*cloud_filtered);

  if (cloud_filtered->empty())
    return false;

  // 4. 法向估计
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZRGB>());
  ne.setInputCloud(cloud_filtered);
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(search_radius);
  ne.compute(*normals);

  // 5. 合并 XYZRGB + Normal
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_temp(
      new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::concatenateFields(*cloud_filtered, *normals, *cloud_temp);

  // 6. 边缘点剔除 (可选，防止选到边缘)
  pcl::RadiusOutlierRemoval<pcl::PointXYZRGBNormal> ror;
  ror.setInputCloud(cloud_temp);
  ror.setRadiusSearch(search_radius);
  ror.setMinNeighborsInRadius(search_num_th);
  ror.filter(*cloud_out);

  return !cloud_out->empty();
}