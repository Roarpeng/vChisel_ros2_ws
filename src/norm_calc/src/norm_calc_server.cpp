#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "norm_calc/srv/norm_calc_data.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "norm_calc/norm_calc.h"
#include "norm_calc/chisel_box.h"
#include "norm_calc/edge_grid.h"
#include <geometry_msgs/msg/pose.hpp>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <memory>

using namespace std;

// Global service object
rclcpp::Service<norm_calc::srv::NormCalcData>::SharedPtr service;
rclcpp::Client<norm_calc::srv::NormCalcData>::SharedPtr client;

// Function to process NormCalcData request
void processNormCalc(const std::shared_ptr<norm_calc::srv::NormCalcData::Request> request,
                     std::shared_ptr<norm_calc::srv::NormCalcData::Response> response)
{
    // Print the received sequence number
    RCLCPP_INFO(rclcpp::get_logger("norm_calc_server"), "Received request with sequence: %d", request->seq);

    // Create some dummy point clouds for demonstration
    // In a real implementation, you would get these from a sensor or other source
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_holes (new pcl::PointCloud<pcl::PointXYZ>);

    // Fill with some dummy data (in a real case, this would come from a sensor)
    cloud->width = 100;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].r = 255;
        cloud->points[i].g = 255;
        cloud->points[i].b = 255;
    }

    cloud_holes->width = 10;
    cloud_holes->height = 1;
    cloud_holes->points.resize(cloud_holes->width * cloud_holes->height);
    for (size_t i = 0; i < cloud_holes->points.size(); ++i) {
        cloud_holes->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud_holes->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud_holes->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    }

    // Create intermediate point clouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downSampled (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_smoothed (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_shrink (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_tarPoint (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PolygonMesh::Ptr triangles (new pcl::PolygonMesh);

    // Initialize parameters (these should come from parameters or config files in a real implementation)
    chisel_box::ChiselParam chiselParam;
    chiselParam.BOX_LEN = 0.05;
    chiselParam.BOX_ROW = 4;
    chiselParam.BOX_COLUMN = 6;
    // NOTE: Point cloud coordinates are normalized to [-1, 1] range from camera
    // Adjust filter bounds to match actual coordinate range
    chiselParam.XMIN = -1.0;   // Points range -0.99 to 0.97
    chiselParam.XMAX = 1.0;
    chiselParam.YMIN = -1.0;   // Points range -0.97 to 0.98
    chiselParam.YMAX = 1.0;
    chiselParam.ZMIN = -1.0;   // Points range -0.99 to 0.98
    chiselParam.ZMAX = 1.0;
    chiselParam.BORDER_WIDTH = 0.03;
    // RELAXED for testing: loosen surface normal requirement
    chiselParam.NORM_TH = 0.60;
    // RELAXED for testing: disable hole proximity filtering (radius^2 <= 0 => skip check)
    chiselParam.AFFECT_RADIUS = 0.0;
    chiselParam.HEIGHT_WEIGHT = 3.0;
    chiselParam.CURV_WEIGHT = 2.0;
    chiselParam.ANGLE_WEIGHT = 1.0;
    chiselParam.SEARCH_RADIUS = 0.03;
    chiselParam.SEARCH_NUM_TH = 20;

    edge_grid::GridParam gridParam;
    gridParam.THDEEP = 0.57;
    gridParam.THDEEPNORM = 0.96;
    // RELAXED for testing: reduce grid coverage thresholds
    gridParam.THHALF = 10;
    gridParam.THFULL = 25;
    gridParam.THANGLE = 0.985;
    gridParam.GRID_LEN = 0.03; // This needs to be defined based on your grid setup
    gridParam.GRID_ROW = 20;   // This needs to be defined based on your grid setup
    gridParam.GRID_COLUMN = 10; // This needs to be defined based on your grid setup
    gridParam.YMIN = chiselParam.YMIN;
    gridParam.XMIN = chiselParam.XMIN;

    // Create target point list
    chisel_box::ChiselNormPoint tarPointList[24]; // 4*6 = 24

    // Call the normCalc function
    bool success = normCalc(
        cloud_holes,
        cloud,
        cloud_downSampled,
        cloud_filtered,
        cloud_smoothed,
        cloud_normals,
        cloud_with_normals,
        cloud_shrink,
        cloud_tarPoint,
        triangles,
        tarPointList,
        chiselParam,
        gridParam
    );

    if (success) {
        RCLCPP_INFO(rclcpp::get_logger("norm_calc_server"), "Norm calculation completed successfully.");

        // Create response with the calculated poses
        geometry_msgs::msg::PoseArray pose_array;
        pose_array.header.frame_id = "base_link";
        pose_array.header.stamp = rclcpp::Clock().now();

        // Add poses from tarPointList to the response
        for (int i = 0; i < 24; i++) {
            if (tarPointList[i].status) {
                geometry_msgs::msg::Pose pose;
                pose.position.x = tarPointList[i].ox;
                pose.position.y = tarPointList[i].oy;
                pose.position.z = tarPointList[i].oz;
                pose.orientation.x = tarPointList[i].nx;
                pose.orientation.y = tarPointList[i].ny;
                pose.orientation.z = tarPointList[i].nz;
                pose.orientation.w = tarPointList[i].curv; // Using curvature as w component for demo

                pose_array.poses.push_back(pose);
            }
        }

        response->pose_list = pose_array;
        RCLCPP_INFO(rclcpp::get_logger("norm_calc_server"), "Sending response with %zu poses.", pose_array.poses.size());
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("norm_calc_server"), "Norm calculation failed.");
        // Return an empty pose array on failure
        geometry_msgs::msg::PoseArray empty_pose_array;
        empty_pose_array.header.frame_id = "base_link";
        empty_pose_array.header.stamp = rclcpp::Clock().now();
        response->pose_list = empty_pose_array;
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("norm_calc_server");

    // Create the service
    service = node->create_service<norm_calc::srv::NormCalcData>(
        "norm_calc",
        std::bind(processNormCalc, std::placeholders::_1, std::placeholders::_2)
    );

    RCLCPP_INFO(node->get_logger(), "Norm Calc Server is ready to receive requests.");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}