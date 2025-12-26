#include "norm_calc/norm_calc.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_msgs/msg/polygon_mesh.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include <functional>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

// Using ROS2 message filter types
namespace sync_policies = message_filters::sync_policies;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_smoothed (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_tarPoint (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_allNorm (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_holes (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PolygonMesh::Ptr triangles (new pcl::PolygonMesh);

pcl::visualization::PCLVisualizer viewer("PCL Viewer");

bool dataRefresh = false;

void cloudCallBack(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr inputSmoothedData,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr inputTarPointData,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr inputAllNormData,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr inputHolesData,
    const pcl_msgs::msg::PolygonMesh::ConstSharedPtr inputTrianglesData
)
{
    pcl::fromROSMsg(*inputSmoothedData, *cloud_smoothed);
    pcl::fromROSMsg(*inputTarPointData, *cloud_tarPoint);
    pcl::fromROSMsg(*inputAllNormData,  *cloud_allNorm);
    pcl::fromROSMsg(*inputHolesData,    *cloud_holes);
    pcl_conversions::toPCL(*inputTrianglesData, *triangles);
    dataRefresh = true;
    RCLCPP_INFO(rclcpp::get_logger("norm_viewer"), "data refresh!!!");
}

void cloudViewer()
{
    if (dataRefresh)
    {
        viewer.removeAllShapes();
        viewer.removeAllPointClouds();
        viewer.setBackgroundColor (0.0, 0.0, 0.3);

        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_smoothed);
        viewer.addPointCloud<pcl::PointXYZRGB>(cloud_smoothed,rgb,"cloud_smoothed");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"cloud_smoothed");

        viewer.addPointCloudNormals<pcl::PointXYZRGBNormal>(cloud_tarPoint,1,0.1,"cloud_tarPoint");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,5,"cloud_tarPoint");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,0,0,"cloud_tarPoint");

        // viewer.addPointCloudNormals<pcl::PointXYZRGBNormal>(cloud_allNorm,1,0.1,"cloud_allNorm");
        // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,1,"cloud_allNorm");
        // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,1,0,"cloud_allNorm");

        viewer.addPointCloud<pcl::PointXYZ>(cloud_holes,"cloud_holes");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,10,"cloud_holes");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,1,0,"cloud_holes");

        // viewer.addPolygonMesh(*triangles,"triangles");


        std::string id = std::to_string(0);
        viewer.addLine<pcl::PointXYZ>(pcl::PointXYZ(-0.165, -0.115, 0.46),pcl::PointXYZ(0.165, -0.115, 0.46),255,0,0,id);
        for (int i = 1; i < 4; i++)
        {
            id = std::to_string(i);
            viewer.addLine<pcl::PointXYZ>(pcl::PointXYZ(-0.165, 0.05*i - 0.1, 0.46),pcl::PointXYZ(0.165, 0.05*i - 0.1, 0.46),255,0,0,id);
        }
        id = std::to_string(4);
        viewer.addLine<pcl::PointXYZ>(pcl::PointXYZ(-0.165, 0.115, 0.46),pcl::PointXYZ(0.165, 0.115, 0.46),255,0,0,id);

        id = std::to_string(5);
        viewer.addLine<pcl::PointXYZ>(pcl::PointXYZ(-0.165, -0.115, 0.46),pcl::PointXYZ(-0.165, 0.115, 0.46),255,0,0,id);
        for (int j = 1; j < 6; j++)
        {
            id = std::to_string(5+j);
            viewer.addLine<pcl::PointXYZ>(pcl::PointXYZ(0.05*j-0.15, -0.125, 0.46),pcl::PointXYZ(0.05*j-0.15, 0.115, 0.46),255,0,0,id);
        }
        id = std::to_string(11);
        viewer.addLine<pcl::PointXYZ>(pcl::PointXYZ(0.165, -0.115, 0.46),pcl::PointXYZ(0.165, 0.115, 0.46),255,0,0,id);


        viewer.addCoordinateSystem (0.2);
        viewer.setCameraPosition(0,0,-1,0,0,1,0,-1,0);
        viewer.setRepresentationToSurfaceForAllActors(); //网格模型以面片形式显示

        dataRefresh = false;
    }
    viewer.spinOnce ();
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("norm_viewer");

    // Using message_filters for time synchronization in ROS2
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> subSmoothedData(node, "smoothedData");
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> subTarPointData(node, "tarPointData");
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> subAllNormtData(node, "allNormData");
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> subHolesData(node, "holesData");
    message_filters::Subscriber<pcl_msgs::msg::PolygonMesh> subTrianglesData(node, "trianglesData");

    typedef sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2,
                                           sensor_msgs::msg::PointCloud2,
                                           sensor_msgs::msg::PointCloud2,
                                           sensor_msgs::msg::PointCloud2,
                                           pcl_msgs::msg::PolygonMesh> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),
        subSmoothedData, subTarPointData, subAllNormtData, subHolesData, subTrianglesData);
    sync.registerCallback(std::bind(&cloudCallBack, std::placeholders::_1, std::placeholders::_2,
                                   std::placeholders::_3, std::placeholders::_4, std::placeholders::_5));

    viewer.setPosition(1000,500);

    rclcpp::Rate loop_rate(10);
    while(rclcpp::ok() && !viewer.wasStopped())
    {
        rclcpp::spin_some(node);
        cloudViewer();
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}