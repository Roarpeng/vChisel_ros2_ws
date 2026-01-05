#include "norm_calc/norm_calc.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_msgs/msg/polygon_mesh.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
// [修复] 添加缺失的可视化头文件
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

// Using ROS2 message filter types
namespace sync_policies = message_filters::sync_policies;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    cloud_smoothed(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr
    cloud_tarPoint(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr
    cloud_allNorm(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
pcl::PointCloud<pcl::PointXYZ>::Ptr
    cloud_holes(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);

// 使用智能指针延迟初始化
std::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

bool dataRefresh = false;

void cloudCallBack(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr inputSmoothedData,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr inputTarPointData,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr inputAllNormData,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr inputHolesData,
    const pcl_msgs::msg::PolygonMesh::ConstSharedPtr inputTrianglesData) {
  pcl::fromROSMsg(*inputSmoothedData, *cloud_smoothed);
  pcl::fromROSMsg(*inputTarPointData, *cloud_tarPoint);
  pcl::fromROSMsg(*inputAllNormData, *cloud_allNorm);
  pcl::fromROSMsg(*inputHolesData, *cloud_holes);
  pcl_conversions::toPCL(*inputTrianglesData, *triangles);
  dataRefresh = true;
  RCLCPP_INFO(rclcpp::get_logger("norm_viewer"), "data refresh!!!");
}

void cloudViewer() {
  // 首次收到数据时才初始化 viewer
  if (!viewer && dataRefresh) {
    try {
      viewer.reset(new pcl::visualization::PCLVisualizer("PCL Viewer"));
      viewer->setBackgroundColor(0.0, 0.0, 0.3);
      viewer->setCameraPosition(0, 0, -1, 0, 0, 1, 0, -1, 0);
      viewer->setPosition(1000, 500);
      viewer->setSize(640, 480);
      RCLCPP_INFO(rclcpp::get_logger("norm_viewer"),
                  "PCL Visualizer initialized on first data");
    } catch (const std::exception &e) {
      RCLCPP_ERROR(rclcpp::get_logger("norm_viewer"),
                   "Failed to initialize PCL Visualizer: %s", e.what());
      dataRefresh = false;
      return;
    }
  }

  if (viewer && dataRefresh) {
    viewer->removeAllShapes();
    viewer->removeAllPointClouds();
    viewer->setBackgroundColor(0.0, 0.0, 0.3);

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(
        cloud_smoothed);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud_smoothed, rgb,
                                            "cloud_smoothed");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_smoothed");

    viewer->addPointCloudNormals<pcl::PointXYZRGBNormal>(cloud_tarPoint, 1, 0.1,
                                                         "cloud_tarPoint");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "cloud_tarPoint");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud_tarPoint");

    viewer->addPointCloud<pcl::PointXYZ>(cloud_holes, "cloud_holes");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "cloud_holes");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud_holes");

    std::string id = std::to_string(0);
    viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(-0.165, -0.115, 0.46),
                                   pcl::PointXYZ(0.165, -0.115, 0.46), 255, 0,
                                   0, id);
    for (int i = 1; i < 4; i++) {
      id = std::to_string(i);
      viewer->addLine<pcl::PointXYZ>(
          pcl::PointXYZ(-0.165, 0.05 * i - 0.1, 0.46),
          pcl::PointXYZ(0.165, 0.05 * i - 0.1, 0.46), 255, 0, 0, id);
    }
    id = std::to_string(4);
    viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(-0.165, 0.115, 0.46),
                                   pcl::PointXYZ(0.165, 0.115, 0.46), 255, 0, 0,
                                   id);

    id = std::to_string(5);
    viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(-0.165, -0.115, 0.46),
                                   pcl::PointXYZ(-0.165, 0.115, 0.46), 255, 0,
                                   0, id);
    for (int j = 1; j < 6; j++) {
      id = std::to_string(5 + j);
      viewer->addLine<pcl::PointXYZ>(
          pcl::PointXYZ(0.05 * j - 0.15, -0.125, 0.46),
          pcl::PointXYZ(0.05 * j - 0.15, 0.115, 0.46), 255, 0, 0, id);
    }
    id = std::to_string(11);
    viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(0.165, -0.115, 0.46),
                                   pcl::PointXYZ(0.165, 0.115, 0.46), 255, 0, 0,
                                   id);

    viewer->addCoordinateSystem(0.2);
    viewer->setCameraPosition(0, 0, -1, 0, 0, 1, 0, -1, 0);
    // viewer->setRepresentationToSurfaceForAllActors(); //
    // 注释掉，防止某些PCL版本不兼容

    dataRefresh = false;
  }
  if (viewer) {
    viewer->spinOnce();
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("norm_viewer");

  RCLCPP_INFO(rclcpp::get_logger("norm_viewer"),
              "norm_viewer node started, waiting for data...");

  // Using message_filters for time synchronization in ROS2
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> subSmoothedData(
      node, "smoothedData");
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> subTarPointData(
      node, "tarPointData");
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> subAllNormtData(
      node, "allNormData");
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> subHolesData(
      node, "holesData");
  message_filters::Subscriber<pcl_msgs::msg::PolygonMesh> subTrianglesData(
      node, "trianglesData");

  typedef sync_policies::ApproximateTime<
      sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2,
      sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2,
      pcl_msgs::msg::PolygonMesh>
      MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(
      MySyncPolicy(10), subSmoothedData, subTarPointData, subAllNormtData,
      subHolesData, subTrianglesData);
  sync.registerCallback(std::bind(
      &cloudCallBack, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3, std::placeholders::_4, std::placeholders::_5));

  rclcpp::Rate loop_rate(10);
  while (rclcpp::ok()) {
    if (viewer && viewer->wasStopped()) {
      RCLCPP_INFO(rclcpp::get_logger("norm_viewer"),
                  "Viewer stopped, shutting down...");
      break;
    }

    rclcpp::spin_some(node);
    cloudViewer();
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}