#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_msgs/PolygonMesh.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "norm_calc/norm_calc.h"
#include "norm_calc/NormCalcData.h"
#include <geometry_msgs/Pose.h>
// Add for coordinate transfer 
#include <iostream>
#include <Eigen/Dense>
//#include<Eigen/Eigen>
//#include<Eigen/Core>
#include <math.h>

#include "norm_calc/hole_detector.h"
#include <cv_bridge/cv_bridge.h>


#define MULTI_FRAME_NUM 3        // 融合帧数
#define Cam2Tool_TF              // 坐标转换
#define Tool2World               // 工具转世界坐标
#define FIRST_ROBOT
#define SECOND_ROBOT

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_holes (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downSampled (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_smoothed (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_shrink (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_tarPoint (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
pcl::PolygonMesh::Ptr triangles (new pcl::PolygonMesh); 
chisel_box::ChiselNormPoint tarPointList[24];

geometry_msgs::Pose tempPose;
uint32_t frameNum = 0, depthNum = 0, camInfoNum = 0, imgNum = 0;
cv::Mat imgColor;
cv::Mat imgDepth = cv::Mat::zeros(480, 848, CV_16UC1);  
sensor_msgs::CameraInfo camInfo;

sensor_msgs::PointCloud2 outSmoothedCloud;
sensor_msgs::PointCloud2 outTarPointCloud;
sensor_msgs::PointCloud2 outAllNormCloud;
sensor_msgs::PointCloud2 outHolesCloud;
pcl_msgs::PolygonMesh outTriangles;

chisel_box::ChiselParam inChiselParam;
edge_grid::GridParam inGridParam;

// Using Quaternion represent the rotation
bool Coordinate_TF(chisel_box::ChiselNormPoint& Inputdata, float Translate_x, float Translate_y, float Translate_z, float x, float y, float z, float w) {
	Eigen::Quaterniond q(w, x, y, z);
	q.normalize();
	Eigen::Matrix3d rotation_matrix;
	rotation_matrix = q.matrix();
	Eigen::Vector3d Translate(Translate_x, Translate_y, Translate_z);

	Eigen::Vector3d PCam_Translate( Inputdata.ox, Inputdata.oy, Inputdata.oz);
	Eigen::Vector3d PTool_Tranlate;
	PTool_Tranlate = rotation_matrix * PCam_Translate + Translate;

	Eigen::Vector3d PCam_Normal(Inputdata.nx, Inputdata.ny, Inputdata.nz);
	Eigen::Vector3d PTool_Normal;
	PTool_Normal = rotation_matrix * PCam_Normal;
	
	Inputdata.ox = PTool_Tranlate(0);
	Inputdata.oy = PTool_Tranlate(1);
	Inputdata.oz = PTool_Tranlate(2);

	Inputdata.nx = PTool_Normal(0);
	Inputdata.ny = PTool_Normal(1);
	Inputdata.nz = PTool_Normal(2);
    return 1;
}

// Using Euler Angle represent the rotation 
bool Coordinate_TF(chisel_box::ChiselNormPoint& Inputdata, float Translate_x, float Translate_y, float Translate_z, float alpha, float beta, float gama) {
    Eigen::AngleAxisd R_Angle(Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitX()));
	Eigen::AngleAxisd P_Angle(Eigen::AngleAxisd(beta, Eigen::Vector3d::UnitY()));
	Eigen::AngleAxisd Y_Angle(Eigen::AngleAxisd(gama, Eigen::Vector3d::UnitZ()));
    //  RPY 
	Eigen::Matrix3d rotation_matrix;
	rotation_matrix = Y_Angle * P_Angle * R_Angle;
	Eigen::Vector3d Translate(Translate_x, Translate_y, Translate_z);

	Eigen::Vector3d PCam_Translate( Inputdata.ox, Inputdata.oy, Inputdata.oz);
	Eigen::Vector3d PTool_Tranlate;
	PTool_Tranlate = rotation_matrix * PCam_Translate + Translate;

	Eigen::Vector3d PCam_Normal(Inputdata.nx, Inputdata.ny, Inputdata.nz);
	Eigen::Vector3d PTool_Normal;
	PTool_Normal = rotation_matrix * PCam_Normal;

	Inputdata.ox = PTool_Tranlate(0);
	Inputdata.oy = PTool_Tranlate(1);
	Inputdata.oz = PTool_Tranlate(2);

	Inputdata.nx = PTool_Normal(0);
	Inputdata.ny = PTool_Normal(1);
	Inputdata.nz = PTool_Normal(2);
    return 1;
}

// Transform Matrix known
bool Coordinate_TF(chisel_box::ChiselNormPoint& Inputdata, Eigen::Matrix4d Cam2Tool_Matrix){
    Eigen::Vector4d PCam(Inputdata.ox, Inputdata.oy, Inputdata.oz, 1);
    Eigen::Vector4d PCam_N(Inputdata.nx, Inputdata.ny, Inputdata.nz, 0);
    Eigen::Vector4d P_1(0, 0, 0, 1);

    Eigen::Vector4d PTool = Cam2Tool_Matrix * PCam;
    Eigen::Vector4d PTool_N = Cam2Tool_Matrix * PCam_N;
    Eigen::Vector4d P_translate = Cam2Tool_Matrix * P_1;

    Inputdata.ox = PTool(0);
	Inputdata.oy = PTool(1);
	Inputdata.oz = PTool(2);

	Inputdata.nx = PTool_N(0);
	Inputdata.ny = PTool_N(1);
	Inputdata.nz = PTool_N(2);

    return 1;
}

bool Vector2Euler(chisel_box::ChiselNormPoint& Inputdata){
    Eigen::Vector3f N_V(-1 * Inputdata.nx, -1 * Inputdata.ny, -1 * Inputdata.nz);
    N_V = N_V / N_V.norm();
    // Depends on the actual condition, we dont need rotate the tool in Roll Axis
    float Yaw, Pitch;
    Yaw = atan2(N_V(1), N_V(0));
    // std::cout<< "Input_X: " << Inputdata.nx << std::endl;
    Pitch = -asin(N_V(2) / 1.0f);

    Inputdata.nx = Yaw;
	Inputdata.ny = Pitch;
	Inputdata.nz = 0;
    
    return 1;
}


// subscribe pointcloud
// void cloudCallBack(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
// { 
//     pcl::fromROSMsg(*cloud_msg, *cloud_in);
//     if (cloud_in->size() > 0)
//     {
//         *cloud += *cloud_in;
//         frameNum++;
//     }
//     ROS_INFO("receive cloud: [%d]", cloud_in->size());
// }

// subscribe color image
void imgCallBack(const sensor_msgs::ImageConstPtr& img_msg)
{ 
    cv_bridge::CvImagePtr cv_ptr;

    if (!imgNum)
    {
        try
        {
            cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
            imgColor = cv_ptr->image;
            imgNum++;
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_INFO("cv_bridge error: %s", e.what());
            return;
        }
    }
    
}

// subscribe depth image
void depthCallBack(const sensor_msgs::ImageConstPtr& depth_msg)
{ 
    cv_bridge::CvImagePtr cv_ptr;

    if (!depthNum)
    {
        try
        {
            cv_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
            imgDepth = cv_ptr->image;
            depthNum++;
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_INFO("cv_bridge error: %s", e.what());
            return;
        }
    }
    
}

// subscribe camera info
void camInfoCallBack(const sensor_msgs::CameraInfo& cam_msg)
{ 
    if (!camInfoNum)
    {
        camInfo = cam_msg;
        camInfoNum++;
    }
}

void imageToPointCloud(cv::Mat colorImg, cv::Mat depthImg, sensor_msgs::CameraInfo camInfo, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    pcl::PointXYZRGB p;
    
    cv::Mat grayImg, blurImg;
    cv::cvtColor(colorImg, grayImg, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(grayImg, blurImg, cv::Size(17, 9), 5, 0);
    // cv::imshow("gray", grayImg);
    // cv::imshow("blur", blurImg);
    cv::waitKey(0);
    for(int row = 0; row < depthImg.rows; row++)
    {
        for(int col = 0; col < depthImg.cols; col++)
        {
            p.z = 0.001f * depthImg.at<u_int16_t>(row, col);
            p.x = (col - camInfo.K.at(2))/camInfo.K.at(0) * p.z;
            p.y = (row - camInfo.K.at(5))/camInfo.K.at(4) * p.z;
            p.b = colorImg.at<u_int8_t>(row,col*3);
            p.g = colorImg.at<u_int8_t>(row,col*3 + 1);
            p.r = colorImg.at<u_int8_t>(row,col*3 + 2);
            p.a = blurImg.at<u_int8_t>(row,col);
            cloud->points.push_back(p);
        }
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);
}

bool getTheNorm(norm_calc::NormCalcData::Request   &req,
    norm_calc::NormCalcData::Response   &res)
{
    ros::NodeHandle nh2;
    // ros::Subscriber cloudSub = nh2.subscribe("camera/depth_registered/points", 1, cloudCallBack);
    ros::Publisher pubSmoothedData  = nh2.advertise<sensor_msgs::PointCloud2>("smoothedData",1);
    ros::Publisher pubTarPointData  = nh2.advertise<sensor_msgs::PointCloud2>("tarPointData",1);
    ros::Publisher pubAllNormData   = nh2.advertise<sensor_msgs::PointCloud2>("allNormData",1);
    ros::Publisher pubHolesData     = nh2.advertise<sensor_msgs::PointCloud2>("holesData",1);
    ros::Publisher pubTrianglesData = nh2.advertise<pcl_msgs::PolygonMesh>("trianglesData",1);
    ros::Rate loop_rate(10);
    memset(tarPointList,0,sizeof(tarPointList));
    frameNum = 0;

    // while ( frameNum < MULTI_FRAME_NUM )
    // {
    //     ros::spinOnce();
    // }
    // cloudSub.shutdown();

    ros::Subscriber imgSub = nh2.subscribe("camera/color/image_raw", 1, imgCallBack);
    ros::Subscriber depthSub = nh2.subscribe("camera/aligned_depth_to_color/image_raw", 1, depthCallBack);
    ros::Subscriber camInfoSub = nh2.subscribe("camera/aligned_depth_to_color/camera_info", 1, camInfoCallBack);

    while (imgNum < 1 || depthNum < 1 || camInfoNum < 1)
    {
        ros::spinOnce();
    }

    imgSub.shutdown();
    depthSub.shutdown();
    camInfoSub.shutdown();

    ROS_INFO("hole detect start!");
    holeDetector(imgColor, imgDepth, camInfo, cloud_holes);

    imageToPointCloud(imgColor, imgDepth, camInfo, cloud);

    normCalc(cloud_holes, cloud, cloud_downSampled, cloud_filtered, 
            cloud_smoothed, cloud_normals, cloud_with_normals,
            cloud_shrink, cloud_tarPoint, triangles, tarPointList, inChiselParam, inGridParam);

    ROS_INFO("norm calc done");

    #ifdef Cam2Tool_TF
    float translate_x = 0 ;
    float translate_y = 0 ;
    float translate_z = 0 ;

    float Q_x = 1 ;
    float Q_y = 2 ;
    float Q_z = 3 ;
    float Q_w = 4 ;

    // Transform Matrix Cam2Tool 
    Eigen::Matrix4d Cam2Tool_Matrix = Eigen::Matrix4d::Identity(); 


     // 1st 20241017
    Cam2Tool_Matrix( 0, 0) =  0.004826963675269136;
	Cam2Tool_Matrix( 0, 1) = -0.01345517196028745;
	Cam2Tool_Matrix( 0, 2) = -0.9998978241646477;
	Cam2Tool_Matrix( 0, 3) =  0.3171330727142787;
	Cam2Tool_Matrix( 1, 0) = -0.9998245418956727;
	Cam2Tool_Matrix( 1, 1) = -0.01816284281943825;
	Cam2Tool_Matrix( 1, 2) = -0.004582200762140387;
	Cam2Tool_Matrix( 1, 3) =  0.03597278476443532;
	Cam2Tool_Matrix( 2, 0) = -0.01809933271658953;
	Cam2Tool_Matrix( 2, 1) =  0.9997445021045304;
	Cam2Tool_Matrix( 2, 2) = -0.01354048252385098;
	Cam2Tool_Matrix( 2, 3) = -0.2736846123222878;
	Cam2Tool_Matrix( 3, 0) =  0;
	Cam2Tool_Matrix( 3, 1) =  0;
	Cam2Tool_Matrix( 3, 2) =  0;
	Cam2Tool_Matrix( 3, 3) =  1;

    // 1st 20240925
    // Cam2Tool_Matrix( 0, 0) = -0.002315326536401319;
	// Cam2Tool_Matrix( 0, 1) =  0.01897224535252429;
	// Cam2Tool_Matrix( 0, 2) = -0.999817329900474;
	// Cam2Tool_Matrix( 0, 3) =  0.3331884230794901;
	// Cam2Tool_Matrix( 1, 0) = -0.9999534940812815;
	// Cam2Tool_Matrix( 1, 1) = -0.009404366273868314;
	// Cam2Tool_Matrix( 1, 2) =  0.002137187315900346;
	// Cam2Tool_Matrix( 1, 3) =  0.03924394005705515;
	// Cam2Tool_Matrix( 2, 0) = -0.009362101135223566;
	// Cam2Tool_Matrix( 2, 1) =  0.9997757807635022;
	// Cam2Tool_Matrix( 2, 2) =  0.01899313720961593;
	// Cam2Tool_Matrix( 2, 3) = -0.2749748792503904;
	// Cam2Tool_Matrix( 3, 0) =  0;
	// Cam2Tool_Matrix( 3, 1) =  0;
	// Cam2Tool_Matrix( 3, 2) =  0;
	// Cam2Tool_Matrix( 3, 3) =  1;

    //2nd 20240225
    // Cam2Tool_Matrix( 0, 0) =  0.1411879593799479;
	// Cam2Tool_Matrix( 0, 1) =  0.1224912764377885;
	// Cam2Tool_Matrix( 0, 2) = -0.9823756141735031;
	// Cam2Tool_Matrix( 0, 3) =  0.156975765359138;
	// Cam2Tool_Matrix( 1, 0) = -0.8193675770357438;
	// Cam2Tool_Matrix( 1, 1) =  0.5713782055537362;
	// Cam2Tool_Matrix( 1, 2) = -0.04651580291435042;
	// Cam2Tool_Matrix( 1, 3) =  0.06241973791752749;
	// Cam2Tool_Matrix( 2, 0) =  0.5556102355326983;
	// Cam2Tool_Matrix( 2, 1) =  0.8114941980167409;
	// Cam2Tool_Matrix( 2, 2) =  0.1810371032591547;
	// Cam2Tool_Matrix( 2, 3) = -0.02311450049952559;
	// Cam2Tool_Matrix( 3, 0) =  0;
	// Cam2Tool_Matrix( 3, 1) =  0;
	// Cam2Tool_Matrix( 3, 2) =  0;
	// Cam2Tool_Matrix( 3, 3) =  1;

    //2nd 20240226 Tool5 A 180deg
    // Cam2Tool_Matrix( 0, 0) =  0.002136129093849748;
	// Cam2Tool_Matrix( 0, 1) =  0.0131617748027531;
	// Cam2Tool_Matrix( 0, 2) =  0.9999110983665177;
	// Cam2Tool_Matrix( 0, 3) = -0.3592755296727744;
	// Cam2Tool_Matrix( 1, 0) =  0.9998109504292265;
	// Cam2Tool_Matrix( 1, 1) = -0.01935263737141502;
	// Cam2Tool_Matrix( 1, 2) = -0.001881177444387261;
	// Cam2Tool_Matrix( 1, 3) = -0.03444265988510906;
	// Cam2Tool_Matrix( 2, 0) =  0.01932615725645348;
	// Cam2Tool_Matrix( 2, 1) =  0.9997260840404296;
	// Cam2Tool_Matrix( 2, 2) = -0.01320062630660779;
	// Cam2Tool_Matrix( 2, 3) = -0.2227647594718163;
	// Cam2Tool_Matrix( 3, 0) =  0;
	// Cam2Tool_Matrix( 3, 1) =  0;
	// Cam2Tool_Matrix( 3, 2) =  0;
	// Cam2Tool_Matrix( 3, 3) =  1;

    // Transform Matrix Tool2World, fill this matrix with the transform matrix when the robot set on the shoot position
    Eigen::Matrix4d Tool2World_Matrix = Eigen::Matrix4d::Identity(); 
	Tool2World_Matrix( 0, 0) =  1;
	Tool2World_Matrix( 0, 1) =  0;
	Tool2World_Matrix( 0, 2) =  0;
	Tool2World_Matrix( 0, 3) =  0;
	Tool2World_Matrix( 1, 0) =  0;
	Tool2World_Matrix( 1, 1) =  1;
	Tool2World_Matrix( 1, 2) =  0;
	Tool2World_Matrix( 1, 3) =  0;
	Tool2World_Matrix( 2, 0) =  0;
	Tool2World_Matrix( 2, 1) =  0;
	Tool2World_Matrix( 2, 2) =  1;
	Tool2World_Matrix( 2, 3) =  0;
	Tool2World_Matrix( 3, 0) =  0;
	Tool2World_Matrix( 3, 1) =  0;
	Tool2World_Matrix( 3, 2) =  0;
	Tool2World_Matrix( 3, 3) =  1;
    // Matrix 
    Eigen::Matrix4d TF_Matrix = Cam2Tool_Matrix;
    #ifdef Tool2World
    TF_Matrix = Tool2World_Matrix * TF_Matrix; 
    #endif


    #ifdef EDGE_CHISEL_MODE
    size_t maxTarNum = inGridParam.GRID_ROW;
    #else
    size_t maxTarNum = inChiselParam.BOX_ROW * inChiselParam.BOX_COLUMN;
    #endif

    for (int i = 0; i < maxTarNum; i++)
    {
        if (tarPointList[i].status)
        {
            if (Coordinate_TF(tarPointList[i], TF_Matrix)){
                // ROS_INFO("Coordinate Transform Done");
            } 
            if (Vector2Euler(tarPointList[i])){
                // ROS_INFO("Normal Coordinate Transform Done");
                tarPointList[i].nz = (rand()/double(RAND_MAX) - 0.5f) *30.0f / 180.0f * 3.14159f;
                ROS_INFO("x: %f, y: %f, z: %f, A: %f, B: %f, C %f",tarPointList[i].ox, tarPointList[i].oy ,tarPointList[i].oz, tarPointList[i].nx, tarPointList[i].ny,tarPointList[i].nz);
            }
        }      
    }
    #endif 

    for (int i = 0; i < maxTarNum; i++)
    {
        if (tarPointList[i].status)
        {
            tempPose.position.x = tarPointList[i].ox;
            tempPose.position.y = tarPointList[i].oy;
            tempPose.position.z = tarPointList[i].oz;
            tempPose.orientation.x = tarPointList[i].nx;
            tempPose.orientation.y = tarPointList[i].ny;
            tempPose.orientation.z = tarPointList[i].nz;
            tempPose.orientation.w = tarPointList[i].curv;
            res.pose_list.poses.push_back(tempPose);
        }      
    }

    ros::Time now = ros::Time::now();

    res.pose_list.header.seq = req.seq;
    res.pose_list.header.frame_id = cloud->header.frame_id;
    res.pose_list.header.stamp = now;
    ROS_INFO("POSE LIST SIZE: %d",res.pose_list.poses.size());
    ROS_INFO("result trans done");

    #ifdef USING_SMOOTH
    pcl::toROSMsg(*cloud_smoothed,outSmoothedCloud);
    #else
    pcl::toROSMsg(*cloud_downSampled,outSmoothedCloud);
    #endif
    pcl::toROSMsg(*cloud_tarPoint,outTarPointCloud);
    pcl::toROSMsg(*cloud_with_normals,outAllNormCloud);
    pcl::toROSMsg(*cloud_holes, outHolesCloud);
    pcl_conversions::fromPCL(*triangles,outTriangles);
    
    outSmoothedCloud.header.frame_id    = res.pose_list.header.frame_id;
    outTarPointCloud.header.frame_id    = res.pose_list.header.frame_id;
    outAllNormCloud.header.frame_id        = res.pose_list.header.frame_id;
    outHolesCloud.header.frame_id        = res.pose_list.header.frame_id;
    outTriangles.header.frame_id        = res.pose_list.header.frame_id;

    outSmoothedCloud.header.seq         = res.pose_list.header.seq;
    outTarPointCloud.header.seq         = res.pose_list.header.seq;
    outAllNormCloud.header.seq             = res.pose_list.header.seq;
    outHolesCloud.header.seq             = res.pose_list.header.seq;
    outTriangles.header.seq             = res.pose_list.header.seq;

    outSmoothedCloud.header.stamp       = res.pose_list.header.stamp;
    outTarPointCloud.header.stamp       = res.pose_list.header.stamp;
    outAllNormCloud.header.stamp           = res.pose_list.header.stamp;
    outHolesCloud.header.stamp           = res.pose_list.header.stamp;
    outTriangles.header.stamp           = res.pose_list.header.stamp;
    
    int pubNum = 0;
    while (pubNum < 2)
    {
        loop_rate.sleep(); 
        pubSmoothedData.publish(outSmoothedCloud);
        pubTarPointData.publish(outTarPointCloud);
        pubAllNormData.publish(outAllNormCloud);
        pubHolesData.publish(outHolesCloud);
        pubTrianglesData.publish(outTriangles);
        pubNum++;
    }   

    return true;
}

int main(int argc, char** argv)
{     
    ros::init(argc, argv, "norm_calc_server");
    ros::NodeHandle nh;

    srand((unsigned)time(NULL));
    float randX, randY;
    randX = (rand()/double(RAND_MAX) - 0.5f) * inChiselParam.BOX_LEN;
    randY = (rand()/double(RAND_MAX) - 0.5f) * inChiselParam.BOX_LEN;
    cout << "randnum: " << randX << "," << randY << endl;

    inChiselParam.BOX_LEN       = nh.param("/norm_calc_server/BOX_LEN",0.05);
    inChiselParam.BOX_ROW       = nh.param("/norm_calc_server/BOX_ROW",4);
    inChiselParam.BOX_COLUMN    = nh.param("/norm_calc_server/BOX_COLUMN",6);
    inChiselParam.XMIN          = nh.param("/norm_calc_server/XMIN",-0.15) + randX;
    inChiselParam.XMAX          = nh.param("/norm_calc_server/XMAX",0.15) + randX;
    inChiselParam.YMIN          = nh.param("/norm_calc_server/YMIN",-0.1) + randY;
    inChiselParam.YMAX          = nh.param("/norm_calc_server/YMAX",0.1) + randY;
    inChiselParam.ZMIN          = nh.param("/norm_calc_server/ZMIN",0.2);
    inChiselParam.ZMAX          = nh.param("/norm_calc_server/ZMAX",0.6);
    inChiselParam.BORDER_WIDTH  = nh.param("/norm_calc_server/BORDER_WIDTH",0.03);
    inChiselParam.NORM_TH       = nh.param("/norm_calc_server/NORM_TH",0.95);
    inChiselParam.AFFECT_RADIUS = nh.param("/norm_calc_server/AFFECT_RADIUS",0.0009);
    inChiselParam.HEIGHT_WEIGHT = nh.param("/norm_calc_server/HEIGHT_WEIGHT",3.0);
    inChiselParam.CURV_WEIGHT   = nh.param("/norm_calc_server/CURV_WEIGHT",2.0);
    inChiselParam.ANGLE_WEIGHT  = nh.param("/norm_calc_server/ANGLE_WEIGHT",1.0);
    inChiselParam.SEARCH_RADIUS = nh.param("/norm_calc_server/SEARCH_RADIUS",0.03);
    inChiselParam.SEARCH_NUM_TH = nh.param("/norm_calc_server/SEARCH_NUM_TH",100);

    inGridParam.GRID_LEN    = inChiselParam.BOX_LEN;
    inGridParam.GRID_ROW    = inChiselParam.BOX_ROW;
    inGridParam.GRID_COLUMN = inChiselParam.BOX_COLUMN;
    inGridParam.XMIN        = inChiselParam.XMIN;
    inGridParam.XMAX        = inChiselParam.XMAX;
    inGridParam.YMIN        = inChiselParam.YMIN;
    inGridParam.YMAX        = inChiselParam.YMAX;
    inGridParam.ZMIN        = inChiselParam.ZMIN;
    inGridParam.ZMAX        = inChiselParam.ZMAX;
    inGridParam.THDEEP      = nh.param("/norm_calc_server/TH_DEEP",0.4);
    inGridParam.THDEEPNORM  = nh.param("/norm_calc_server/TH_DEEPNORM",0.8);
    inGridParam.THHALF      = nh.param("/norm_calc_server/TH_HALF",50);
    inGridParam.THFULL      = nh.param("/norm_calc_server/TH_FULL",100);
    inGridParam.THANGLE     = nh.param("/norm_calc_server/TH_ANGLE",0.99);

    ROS_INFO("randX = %f",randX);
    ROS_INFO("randY = %f",randY);
    ROS_INFO("Ready to calc the norm.");
    ros::ServiceServer service = nh.advertiseService("norm_calc",getTheNorm);

    ros::spin();
    return 0;
}
