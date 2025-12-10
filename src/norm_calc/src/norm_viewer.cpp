#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::placeholders::_4;
using std::placeholders::_5;

class NormViewerNode : public rclcpp::Node
{
public:
    NormViewerNode() : Node("norm_viewer"), data_refresh_(false)
    {
        using namespace message_filters;

        sub_smoothed_.subscribe(this, "smoothedData");
        sub_tarpoint_.subscribe(this, "tarPointData");
        sub_allnorm_.subscribe(this, "allNormData");
        sub_holes_.subscribe(this, "holesData");
        sub_triangles_.subscribe(this, "trianglesData");

        sync_ = std::make_shared<
            Synchronizer<MySyncPolicy>>(MySyncPolicy(10),
                                        sub_smoothed_,
                                        sub_tarpoint_,
                                        sub_allnorm_,
                                        sub_holes_,
                                        sub_triangles_);

        sync_->registerCallback(std::bind(&NormViewerNode::cloudCallback, this, _1, _2, _3, _4, _5));

        viewer_ = std::make_shared<pcl::visualization::PCLVisualizer>("PCL Viewer");
        viewer_->setPosition(1000, 500);
        viewer_->setBackgroundColor(0.0, 0.0, 0.3);
        RCLCPP_INFO(this->get_logger(), "norm_viewer node initialized.");
    }

    void spinViewer()
    {
        rclcpp::Rate rate(10);
        while (rclcpp::ok() && !viewer_->wasStopped())
        {
            rclcpp::spin_some(shared_from_this());
            updateViewer();
            rate.sleep();
        }
    }

private:
    // 同步回调函数
    void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &inputSmoothedData,
                       const sensor_msgs::msg::PointCloud2::ConstSharedPtr &inputTarPointData,
                       const sensor_msgs::msg::PointCloud2::ConstSharedPtr &inputAllNormData,
                       const sensor_msgs::msg::PointCloud2::ConstSharedPtr &inputHolesData,
                       const pcl_msgs::msg::PolygonMesh::ConstSharedPtr &inputTrianglesData)
    {
        pcl::fromROSMsg(*inputSmoothedData, *cloud_smoothed_);
        pcl::fromROSMsg(*inputTarPointData, *cloud_tarpoint_);
        pcl::fromROSMsg(*inputAllNormData, *cloud_allnorm_);
        pcl::fromROSMsg(*inputHolesData, *cloud_holes_);
        pcl_conversions::toPCL(*inputTrianglesData, *triangles_);
        data_refresh_ = true;
        RCLCPP_INFO(this->get_logger(), "Data refreshed!");
    }

    void updateViewer()
    {
        if (data_refresh_)
        {
            viewer_->removeAllShapes();
            viewer_->removeAllPointClouds();

            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_smoothed_);
            viewer_->addPointCloud<pcl::PointXYZRGB>(cloud_smoothed_, rgb, "cloud_smoothed");
            viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_smoothed");

            viewer_->addPointCloudNormals<pcl::PointXYZRGBNormal>(cloud_tarpoint_, 1, 0.1, "cloud_tarPoint");
            viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "cloud_tarPoint");
            viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud_tarPoint");

            viewer_->addPointCloud<pcl::PointXYZ>(cloud_holes_, "cloud_holes");
            viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "cloud_holes");
            viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud_holes");

            // 加一些示意线
            std::string id;
            for (int i = 0; i <= 11; i++)
            {
                id = std::to_string(i);
                viewer_->addLine<pcl::PointXYZ>(pcl::PointXYZ(-0.165 + 0.05 * i, -0.115, 0.46),
                                                pcl::PointXYZ(-0.165 + 0.05 * i, 0.115, 0.46), 255, 0, 0, id);
            }

            viewer_->addCoordinateSystem(0.2);
            viewer_->setCameraPosition(0, 0, -1, 0, 0, 1, 0, -1, 0);
            viewer_->setRepresentationToSurfaceForAllActors();
            data_refresh_ = false;
        }
        viewer_->spinOnce();
    }

    // 成员对象
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2,
        sensor_msgs::msg::PointCloud2,
        sensor_msgs::msg::PointCloud2,
        sensor_msgs::msg::PointCloud2,
        pcl_msgs::msg::PolygonMesh>
        MySyncPolicy;

    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_smoothed_, sub_tarpoint_, sub_allnorm_, sub_holes_;
    message_filters::Subscriber<pcl_msgs::msg::PolygonMesh> sub_triangles_;
    std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_smoothed_{new pcl::PointCloud<pcl::PointXYZRGB>};
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_tarpoint_{new pcl::PointCloud<pcl::PointXYZRGBNormal>};
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_allnorm_{new pcl::PointCloud<pcl::PointXYZRGBNormal>};
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_holes_{new pcl::PointCloud<pcl::PointXYZ>};
    pcl::PolygonMesh::Ptr triangles_{new pcl::PolygonMesh};

    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
    bool data_refresh_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NormViewerNode>();
    node->spinViewer();
    rclcpp::shutdown();
    return 0;
}
