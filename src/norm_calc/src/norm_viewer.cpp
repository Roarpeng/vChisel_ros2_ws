#include "norm_calc/norm_calc.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_msgs/PolygonMesh.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_smoothed (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_tarPoint (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_allNorm (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_holes (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PolygonMesh::Ptr triangles (new pcl::PolygonMesh);

pcl::visualization::PCLVisualizer viewer("PCL Viewer");

bool dataRefresh = false;

void cloudCallBack( const sensor_msgs::PointCloud2ConstPtr& inputSmoothedData,
                    const sensor_msgs::PointCloud2ConstPtr& inputTarPointData,
                    const sensor_msgs::PointCloud2ConstPtr& inputAllNormData,
                    const sensor_msgs::PointCloud2ConstPtr& inputHolesData,
                    const pcl_msgs::PolygonMeshConstPtr& inputTrianglesData
)  
{
    pcl::fromROSMsg(*inputSmoothedData, *cloud_smoothed);
    pcl::fromROSMsg(*inputTarPointData, *cloud_tarPoint);
    pcl::fromROSMsg(*inputAllNormData,  *cloud_allNorm);
    pcl::fromROSMsg(*inputHolesData,    *cloud_holes);
    pcl_conversions::toPCL(*inputTrianglesData,*triangles);
    dataRefresh = true;
    ROS_INFO("data refressh!!!");
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
    ros::init(argc, argv, "norm_viewer");

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::PointCloud2> subSmoothedData(nh,"smoothedData",1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> subTarPointData(nh,"tarPointData",1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> subAllNormtData(nh,"allNormData",1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> subHolesData(nh,"holesData",1);
    message_filters::Subscriber<pcl_msgs::PolygonMesh> subTrianglesData(nh,"trianglesData",1);
    message_filters::TimeSynchronizer<sensor_msgs::PointCloud2,sensor_msgs::PointCloud2,sensor_msgs::PointCloud2,sensor_msgs::PointCloud2,pcl_msgs::PolygonMesh> sync(subSmoothedData,subTarPointData,subAllNormtData,subHolesData, subTrianglesData,10);
    sync.registerCallback(boost::bind(&cloudCallBack,_1,_2,_3,_4,_5));
    // ros::Subscriber subSmoothedData = nh.subscribe("smoothedData","tarPointData","trianglesData",5,cloudCallBack);
    // ros::Subscriber subTarPointData = nh.subscribe("tarPointData",1,cloudCallBack);
    // ros::Subscriber subTrianglesData = nh.subscribe("trianglesData",1,cloudCallBack);
    viewer.setPosition(1000,500);
    ros::Rate loop_rate(10);
    while(ros::ok && !viewer.wasStopped())
    {
        ros::spinOnce();
        cloudViewer();
        loop_rate.sleep();
    }
    return 0;
}
