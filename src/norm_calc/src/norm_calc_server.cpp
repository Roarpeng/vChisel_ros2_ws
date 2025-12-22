#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "pcl_msgs/msg/polygon_mesh.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include <chrono>

#include "norm_calc/srv/norm_calc_data.hpp"
#include "norm_calc/norm_calc.h"
#include "norm_calc/hole_detector.h"
#include "norm_calc/chisel_box.h"
#include "norm_calc/edge_grid.h"

#define MULTI_FRAME_NUM 5
#define Cam2Tool_TF
#define Tool2World

using std::placeholders::_1;
using std::placeholders::_2;

class NormCalcServer : public rclcpp::Node
{
public:
    NormCalcServer() : Node("norm_calc_server")
    {
        RCLCPP_INFO(this->get_logger(), "Starting NormCalc ROS2 server...");
        this->declare_parameter<double>("BOX_LEN", 0.05);
        this->declare_parameter<int>("BOX_ROW", 4);
        this->declare_parameter<int>("BOX_COLUMN", 6);
        this->declare_parameter<double>("XMIN", -0.15);
        this->declare_parameter<double>("XMAX", 0.15);
        this->declare_parameter<double>("YMIN", -0.1);
        this->declare_parameter<double>("YMAX", 0.1);
        this->declare_parameter<double>("ZMIN", 0.2);
        this->declare_parameter<double>("ZMAX", 0.6);
        this->declare_parameter<double>("BORDER_WIDTH", 0.03);
        this->declare_parameter<double>("NORM_TH", 0.95);
        this->declare_parameter<double>("AFFECT_RADIUS", 0.0009);
        this->declare_parameter<double>("HEIGHT_WEIGHT", 3.0);
        this->declare_parameter<double>("CURV_WEIGHT", 2.0);
        this->declare_parameter<double>("ANGLE_WEIGHT", 1.0);
        this->declare_parameter<double>("SEARCH_RADIUS", 0.03);
        this->declare_parameter<int>("SEARCH_NUM_TH", 100);

        this->declare_parameter<double>("TH_DEEP", 0.4);
        this->declare_parameter<double>("TH_DEEPNORM", 0.8);
        this->declare_parameter<int>("TH_HALF", 50);
        this->declare_parameter<int>("TH_FULL", 100);
        this->declare_parameter<double>("TH_ANGLE", 0.99);

        // initialize params into structs
        srand((unsigned)time(NULL));
        inChiselParam.BOX_LEN = this->get_parameter("BOX_LEN").as_double();
        double randX = ((double)rand() / RAND_MAX - 0.5) * inChiselParam.BOX_LEN;
        double randY = ((double)rand() / RAND_MAX - 0.5) * inChiselParam.BOX_LEN;
        inChiselParam.BOX_ROW = this->get_parameter("BOX_ROW").as_int();
        inChiselParam.BOX_COLUMN = this->get_parameter("BOX_COLUMN").as_int();
        inChiselParam.XMIN = this->get_parameter("XMIN").as_double() + randX;
        inChiselParam.XMAX = this->get_parameter("XMAX").as_double() + randX;
        inChiselParam.YMIN = this->get_parameter("YMIN").as_double() + randY;
        inChiselParam.YMAX = this->get_parameter("YMAX").as_double() + randY;
        inChiselParam.ZMIN = this->get_parameter("ZMIN").as_double();
        inChiselParam.ZMAX = this->get_parameter("ZMAX").as_double();
        inChiselParam.BORDER_WIDTH = this->get_parameter("BORDER_WIDTH").as_double();
        inChiselParam.NORM_TH = this->get_parameter("NORM_TH").as_double();
        inChiselParam.AFFECT_RADIUS = this->get_parameter("AFFECT_RADIUS").as_double();
        inChiselParam.HEIGHT_WEIGHT = this->get_parameter("HEIGHT_WEIGHT").as_double();
        inChiselParam.CURV_WEIGHT = this->get_parameter("CURV_WEIGHT").as_double();
        inChiselParam.ANGLE_WEIGHT = this->get_parameter("ANGLE_WEIGHT").as_double();
        inChiselParam.SEARCH_RADIUS = this->get_parameter("SEARCH_RADIUS").as_double();
        inChiselParam.SEARCH_NUM_TH = this->get_parameter("SEARCH_NUM_TH").as_int();

        inGridParam.GRID_LEN = inChiselParam.BOX_LEN;
        inGridParam.GRID_ROW = inChiselParam.BOX_ROW;
        inGridParam.GRID_COLUMN = inChiselParam.BOX_COLUMN;
        inGridParam.XMIN = inChiselParam.XMIN;
        inGridParam.XMAX = inChiselParam.XMAX;
        inGridParam.YMIN = inChiselParam.YMIN;
        inGridParam.YMAX = inChiselParam.YMAX;
        inGridParam.ZMIN = inChiselParam.ZMIN;
        inGridParam.ZMAX = inChiselParam.ZMAX;
        inGridParam.THDEEP = this->get_parameter("TH_DEEP").as_double();
        inGridParam.THDEEPNORM = this->get_parameter("TH_DEEPNORM").as_double();
        inGridParam.THHALF = this->get_parameter("TH_HALF").as_int();
        inGridParam.THFULL = this->get_parameter("TH_FULL").as_int();
        inGridParam.THANGLE = this->get_parameter("TH_ANGLE").as_double();

        RCLCPP_INFO(this->get_logger(), "randX = %6f", randX);
        RCLCPP_INFO(this->get_logger(), "BOX_LEN = %6f", inChiselParam.BOX_LEN);
        RCLCPP_INFO(this->get_logger(), "randY = %6f", randY);

        pubSmoothedData = this->create_publisher<sensor_msgs::msg::PointCloud2>("smoothedData", 1);
        pubTarPointData = this->create_publisher<sensor_msgs::msg::PointCloud2>("tarPointData", 1);
        pubAllNormData = this->create_publisher<sensor_msgs::msg::PointCloud2>("allNormData", 1);
        pubHolesData = this->create_publisher<sensor_msgs::msg::PointCloud2>("holesData", 1);
        pubTrianglesData = this->create_publisher<pcl_msgs::msg::PolygonMesh>("trianglesData", 1);
        pubNormCalcResult = this->create_publisher<geometry_msgs::msg::PoseArray>("norm_calc_result", 1);

        callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        
        options.callback_group = callback_group;
        // 下面三個對應ros2 launch norm_calc norm_calc_launch.py
        // imgSub = this->create_subscription<sensor_msgs::msg::Image>("camera/realsense2_camera/color/image_raw", 1, std::bind(&NormCalcServer::imgCallback, this, _1),options);
        // depthSub = this->create_subscription<sensor_msgs::msg::Image>("camera/realsense2_camera/depth/image_rect_raw", 1, std::bind(&NormCalcServer::depthCallback, this, _1),options);
        // camInfoSub = this->create_subscription<sensor_msgs::msg::CameraInfo>("camera/realsense2_camera/depth/camera_info", 1, std::bind(&NormCalcServer::camInfoCallback, this, _1),options);
        
        imgSub = this->create_subscription<sensor_msgs::msg::Image>("camera/camera/color/image_raw", 1, std::bind(&NormCalcServer::imgCallback, this, _1),options);
        depthSub = this->create_subscription<sensor_msgs::msg::Image>("camera/camera/depth/image_rect_raw", 1, std::bind(&NormCalcServer::depthCallback, this, _1),options);
        camInfoSub = this->create_subscription<sensor_msgs::msg::CameraInfo>("camera/camera/depth/camera_info", 1, std::bind(&NormCalcServer::camInfoCallback, this, _1),options);
        
        RCLCPP_INFO(this->get_logger(), "发布者订阅者创建成功.");


        service_ = this->create_service<norm_calc::srv::NormCalcData>(
            "norm_calc",
            std::bind(&NormCalcServer::getTheNorm, this, _1, _2));
        // node-level subscriptions are created inside service callback when needed
    }

private:
    // ======================= 成员定义 ==========================
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imgSub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depthSub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camInfoSub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSmoothedData;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubTarPointData;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubAllNormData;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubHolesData;
    rclcpp::Publisher<pcl_msgs::msg::PolygonMesh>::SharedPtr pubTrianglesData;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pubNormCalcResult;

    rclcpp::Service<norm_calc::srv::NormCalcData>::SharedPtr service_;
    rclcpp::CallbackGroup::SharedPtr callback_group;
    rclcpp::SubscriptionOptions options;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_holes_{new pcl::PointCloud<pcl::PointXYZ>};
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_{new pcl::PointCloud<pcl::PointXYZRGB>};
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_{new pcl::PointCloud<pcl::PointXYZRGB>};
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downSampled_{new pcl::PointCloud<pcl::PointXYZRGB>};
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_{new pcl::PointCloud<pcl::PointXYZRGB>};
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_smoothed_{new pcl::PointCloud<pcl::PointXYZRGB>};
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_{new pcl::PointCloud<pcl::Normal>};
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals_{new pcl::PointCloud<pcl::PointXYZRGBNormal>};
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_shrink_{new pcl::PointCloud<pcl::PointXYZRGBNormal>};
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_tarPoint_{new pcl::PointCloud<pcl::PointXYZRGBNormal>};
    pcl::PolygonMesh::Ptr triangles_{new pcl::PolygonMesh};
    chisel_box::ChiselNormPoint tarPointList_[24];

    geometry_msgs::msg::Pose tempPose_;
    uint32_t frameNum_ = 0, depthNum_ = 0, camInfoNum_ = 0, imgNum_ = 0;
    cv::Mat imgColor_;
    cv::Mat imgDepth_ = cv::Mat::zeros(480, 848, CV_16UC1);
    sensor_msgs::msg::CameraInfo camInfo_;
    
    // 添加时间戳和数据验证相关变量
    rclcpp::Time last_img_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_depth_time_{0, 0, RCL_ROS_TIME};
    cv::Mat last_imgColor_;
    cv::Mat last_imgDepth_;
    double image_diff_threshold_ = 500.0; // 降低图像差异阈值从1000到500
    double relative_diff_threshold_ = 0.05; // 相对差异阈值5%

    sensor_msgs::msg::PointCloud2 outSmoothedCloud_;
    sensor_msgs::msg::PointCloud2 outTarPointCloud_;
    sensor_msgs::msg::PointCloud2 outAllNormCloud_;
    sensor_msgs::msg::PointCloud2 outHolesCloud_;
    pcl_msgs::msg::PolygonMesh outTriangles_;

    chisel_box::ChiselParam inChiselParam;
    edge_grid::GridParam inGridParam;

    bool img_ShotFlag = false, depth_ShotFlag = false, info_ShotFlag = false;
    bool img_ready = false, depth_ready = false, info_ready = false;

    // 点云质量监控相关
    struct CloudQualityMetrics {
        float point_density = 0.0f;
        float depth_variance = 0.0f;
        float coverage_ratio = 0.0f;
        float overall_score = 0.0f;
        size_t total_points = 0;
    };
    
    CloudQualityMetrics evaluateCloudQuality(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    float calculateDepthVariance(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    float calculateCoverageRatio(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);


    // Using Quaternion represent the rotation
    bool Coordinate_TF(chisel_box::ChiselNormPoint &Inputdata, float Translate_x, float Translate_y, float Translate_z, float x, float y, float z, float w)
    {
        Eigen::Quaterniond q(w, x, y, z);
        q.normalize();
        Eigen::Matrix3d rotation_matrix = q.matrix();
        Eigen::Vector3d Translate(Translate_x, Translate_y, Translate_z);

        Eigen::Vector3d PCam_Translate(Inputdata.ox, Inputdata.oy, Inputdata.oz);
        Eigen::Vector3d PTool_Tranlate = rotation_matrix * PCam_Translate + Translate;

        Eigen::Vector3d PCam_Normal(Inputdata.nx, Inputdata.ny, Inputdata.nz);
        Eigen::Vector3d PTool_Normal = rotation_matrix * PCam_Normal;

        Inputdata.ox = PTool_Tranlate(0);
        Inputdata.oy = PTool_Tranlate(1);
        Inputdata.oz = PTool_Tranlate(2);

        Inputdata.nx = PTool_Normal(0);
        Inputdata.ny = PTool_Normal(1);
        Inputdata.nz = PTool_Normal(2);
        return true;
    }

    // Using Euler Angle represent the rotation
    bool Coordinate_TF(chisel_box::ChiselNormPoint &Inputdata, float Translate_x, float Translate_y, float Translate_z, float alpha, float beta, float gama)
    {
        Eigen::AngleAxisd R_Angle(Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitX()));
        Eigen::AngleAxisd P_Angle(Eigen::AngleAxisd(beta, Eigen::Vector3d::UnitY()));
        Eigen::AngleAxisd Y_Angle(Eigen::AngleAxisd(gama, Eigen::Vector3d::UnitZ()));
        // convert angle-axis to rotation matrices and multiply
        Eigen::Matrix3d rotation_matrix = Y_Angle.toRotationMatrix() * P_Angle.toRotationMatrix() * R_Angle.toRotationMatrix();
        Eigen::Vector3d Translate(Translate_x, Translate_y, Translate_z);

        Eigen::Vector3d PCam_Translate(Inputdata.ox, Inputdata.oy, Inputdata.oz);
        Eigen::Vector3d PTool_Tranlate = rotation_matrix * PCam_Translate + Translate;

        Eigen::Vector3d PCam_Normal(Inputdata.nx, Inputdata.ny, Inputdata.nz);
        Eigen::Vector3d PTool_Normal = rotation_matrix * PCam_Normal;

        Inputdata.ox = PTool_Tranlate(0);
        Inputdata.oy = PTool_Tranlate(1);
        Inputdata.oz = PTool_Tranlate(2);

        Inputdata.nx = PTool_Normal(0);
        Inputdata.ny = PTool_Normal(1);
        Inputdata.nz = PTool_Normal(2);
        return true;
    }

    // Transform Matrix known
    bool Coordinate_TF(chisel_box::ChiselNormPoint &Inputdata, Eigen::Matrix4d Cam2Tool_Matrix)
    {
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

        return true;
    }

    bool Vector2Euler(chisel_box::ChiselNormPoint &Inputdata)
    {
        Eigen::Vector3f N_V(-1 * Inputdata.nx, -1 * Inputdata.ny, -1 * Inputdata.nz);
        N_V = N_V / N_V.norm();
        float Yaw = atan2(N_V(1), N_V(0));
        float Pitch = -asin(N_V(2) / 1.0f);

        Inputdata.nx = Yaw;
        Inputdata.ny = Pitch;
        Inputdata.nz = 0;
        return true;
    }

    // subscribe color image
    void imgCallback(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg)
    {
        if (img_ShotFlag)
        {
            try
            {
                // 时间戳验证 - 检查图像是否为新数据
                auto current_time = this->get_clock()->now();
                rclcpp::Time image_time(img_msg->header.stamp);
                auto time_diff = (current_time - image_time).seconds();
                
                if (last_img_time_.nanoseconds() > 0 && image_time <= last_img_time_) {
                    RCLCPP_WARN(this->get_logger(), "Received old or duplicate color image timestamp");
                    return;
                }
                
                if (time_diff > 2.0) { // 如果图像时间戳超过200ms，则认为可能过期
                    RCLCPP_WARN(this->get_logger(), "Color image may be stale: %f seconds ago", time_diff);
                }

                // RealSense publishes color as RGB8; use RGB8 and handle channels accordingly
                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::RGB8);
                
                // 数据唯一性检查 - 与上一帧比较（使用绝对和相对差异）
                if (!last_imgColor_.empty()) {
                    double abs_diff = cv::norm(cv_ptr->image, last_imgColor_, cv::NORM_L2);
                    double mean_intensity = cv::mean(last_imgColor_)[0];
                    double rel_diff = mean_intensity > 0 ? abs_diff / (mean_intensity * last_imgColor_.total()) : 0.0;
                    
                    if (abs_diff < image_diff_threshold_ && rel_diff < relative_diff_threshold_) {
                        RCLCPP_DEBUG(this->get_logger(), "Color image appears to be duplicate (abs: %f, rel: %f)", abs_diff, rel_diff);
                        return;
                    }
                }
                
                imgColor_ = cv_ptr->image.clone();
                last_imgColor_ = imgColor_.clone();
                last_img_time_ = image_time;
                img_ready = true;
                
                RCLCPP_INFO(this->get_logger(), "New valid color image received, timestamp diff: %f s", time_diff);
            }
            catch (cv_bridge::Exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
            }
        }
    }

    // subscribe depth image
    void depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg)
    {
        if (depth_ShotFlag)
        {
            try
            {
                // 时间戳验证 - 检查深度图像是否为新数据
                auto current_time = this->get_clock()->now();
                rclcpp::Time depth_time(depth_msg->header.stamp);
                auto time_diff = (current_time - depth_time).seconds();
                
                if (last_depth_time_.nanoseconds() > 0 && depth_time <= last_depth_time_) {
                    RCLCPP_WARN(this->get_logger(), "Received old or duplicate depth image timestamp");
                    return;
                }
                
                if (time_diff > 0.2) { // 如果深度图像时间戳超过200ms，则认为可能过期
                    RCLCPP_WARN(this->get_logger(), "Depth image may be stale: %f seconds ago", time_diff);
                }

                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
                
                // 数据唯一性检查 - 与上一帧比较（使用绝对和相对差异）
                if (!last_imgDepth_.empty()) {
                    double abs_diff = cv::norm(cv_ptr->image, last_imgDepth_, cv::NORM_L2);
                    double mean_intensity = cv::mean(last_imgDepth_)[0];
                    double rel_diff = mean_intensity > 0 ? abs_diff / (mean_intensity * last_imgDepth_.total()) : 0.0;
                    
                    if (abs_diff < image_diff_threshold_ && rel_diff < relative_diff_threshold_) {
                        RCLCPP_DEBUG(this->get_logger(), "Depth image appears to be duplicate (abs: %f, rel: %f)", abs_diff, rel_diff);
                        return;
                    }
                }
                
                imgDepth_ = cv_ptr->image.clone();
                last_imgDepth_ = imgDepth_.clone();
                last_depth_time_ = depth_time;
                depth_ready = true;
                
                RCLCPP_INFO(this->get_logger(), "New valid depth image received, timestamp diff: %f s", time_diff);
            }
            catch (cv_bridge::Exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
            }
        }
    }

    // subscribe camera info
    void camInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &cam_msg)
    {
        if (info_ShotFlag)
        {
            // RCLCPP_INFO(this->get_logger(), "camInfo回调成功!!");
            camInfo_ = *cam_msg;
            info_ready = true;
        }
    }

    void imageToPointCloud(cv::Mat colorImg, cv::Mat depthImg, const sensor_msgs::msg::CameraInfo &camInfo, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
    {
        cloud->points.clear();
        
        // 检查图像是否为空
        if (colorImg.empty() || depthImg.empty()) {
            RCLCPP_ERROR(rclcpp::get_logger("norm_calc"), "Empty color or depth image in imageToPointCloud");
            return;
        }
        
        // 检查图像尺寸是否匹配
        if (colorImg.rows != depthImg.rows || colorImg.cols != depthImg.cols) {
            RCLCPP_INFO(rclcpp::get_logger("norm_calc"), 
                       "Color and depth image dimensions do not match: color(%dx%d) vs depth(%dx%d)", 
                       colorImg.cols, colorImg.rows, depthImg.cols, depthImg.rows);
            
            // 将彩色图像缩放到深度图像尺寸以确保匹配
            cv::Mat resized_color;
            cv::resize(colorImg, resized_color, cv::Size(depthImg.cols, depthImg.rows), 0, 0, cv::INTER_LINEAR);
            
            int rows = depthImg.rows;
            int cols = depthImg.cols;
            
            // 确保相机内参矩阵有足够的元素
            if (camInfo.k.size() < 9) {
                RCLCPP_ERROR(rclcpp::get_logger("norm_calc"), "Insufficient camera intrinsic parameters");
                return;
            }
            
            pcl::PointXYZRGB p;
            cv::Mat grayImg, blurImg;
            cv::cvtColor(resized_color, grayImg, cv::COLOR_BGR2GRAY);
            cv::GaussianBlur(grayImg, blurImg, cv::Size(17, 9), 5, 0);
            
            for (int row = 0; row < rows; row++)
            {
                for (int col = 0; col < cols; col++)
                {
                    p.z = 0.001f * depthImg.at<uint16_t>(row, col);
                    if (p.z <= 0.0f) continue; // 跳过无效深度
                    
                    p.x = (col - camInfo.k[2]) / camInfo.k[0] * p.z;
                    p.y = (row - camInfo.k[5]) / camInfo.k[4] * p.z;
                    
                    // 使用调整后的彩色图像
                    if (resized_color.channels() >= 3) {
                        p.r = resized_color.at<cv::Vec3b>(row, col)[2];  // BGR format, R is 3rd channel
                        p.g = resized_color.at<cv::Vec3b>(row, col)[1];  // G is 2nd channel
                        p.b = resized_color.at<cv::Vec3b>(row, col)[0];  // B is 1st channel
                    } else {
                        p.r = p.g = p.b = 128; // 默认灰色
                    }
                    
                    p.a = blurImg.at<uint8_t>(row, col);
                    cloud->points.push_back(p);
                }
            }
        } else {
            // 图像尺寸匹配的情况
            if (camInfo.k.size() < 9) {
                RCLCPP_ERROR(rclcpp::get_logger("norm_calc"), "Insufficient camera intrinsic parameters");
                return;
            }
            
            pcl::PointXYZRGB p;
            cv::Mat grayImg, blurImg;
            cv::cvtColor(colorImg, grayImg, cv::COLOR_BGR2GRAY);
            cv::GaussianBlur(grayImg, blurImg, cv::Size(17, 9), 5, 0);
            
            for (int row = 0; row < depthImg.rows; row++)
            {
                for (int col = 0; col < depthImg.cols; col++)
                {
                    p.z = 0.001f * depthImg.at<uint16_t>(row, col);
                    p.x = (col - camInfo.k[2]) / camInfo.k[0] * p.z;
                    p.y = (row - camInfo.k[5]) / camInfo.k[4] * p.z;
                    
                    // 使用更安全的颜色访问方式
                    if (colorImg.channels() >= 3) {
                        p.r = colorImg.at<cv::Vec3b>(row, col)[2];  // BGR format, R is the 3rd channel
                        p.g = colorImg.at<cv::Vec3b>(row, col)[1];  // G is the 2nd channel
                        p.b = colorImg.at<cv::Vec3b>(row, col)[0];  // B is the 1st channel
                    } else {
                        p.r = p.g = p.b = 128; // 默认灰色
                    }
                    
                    p.a = blurImg.at<uint8_t>(row, col);
                    cloud->points.push_back(p);
                }
            }
        }
        
        cloud->width = cloud->points.size();
        cloud->height = 1;
        cloud->is_dense = false;
        cloud->points.resize(cloud->width * cloud->height);
    }

    // Service 回调
    void getTheNorm(const std::shared_ptr<norm_calc::srv::NormCalcData::Request> req, std::shared_ptr<norm_calc::srv::NormCalcData::Response> res)
    {
        RCLCPP_INFO(this->get_logger(), "NormCalc service called.");
        rclcpp::Rate rate(10);
        memset(tarPointList_, 0, sizeof(tarPointList_));
        frameNum_ = 0;

        img_ShotFlag = depth_ShotFlag = info_ShotFlag = true;//回调信号置1,三个sub线程执行回调，接受相机数据
        
        // 调整等待时间，适应提高帧率到15fps后的情况
        int wait_count = 0;
        const int max_wait_count = 45; // 45 * 100ms = 4.5秒，给更高帧率更多时间
        while (rclcpp::ok() && (!img_ready || !depth_ready || !info_ready)){
            if (wait_count >= max_wait_count) {
                RCLCPP_ERROR(this->get_logger(), "Camera data timeout after 4.5 seconds. img_ready:%d, depth_ready:%d, info_ready:%d", 
                            img_ready, depth_ready, info_ready);
                img_ShotFlag = depth_ShotFlag = info_ShotFlag = false;//回调信号置0
                break; // 继续执行，使用可能不完整的数据
            }
            
            if (wait_count % 10 == 0) { // 每秒打印一次状态
                RCLCPP_WARN(this->get_logger(), "NOT ready yet.img_ready:%d,depth_ready:%d,info_ready:%d (waited %d.%ds)", 
                           img_ready, depth_ready, info_ready, wait_count/10, wait_count%10);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            wait_count++;
        }
        
        img_ShotFlag = depth_ShotFlag = info_ShotFlag = false;//回调信号置0
        RCLCPP_INFO(this->get_logger(), "Ready.img_ready:%d,depth_ready:%d,info_ready:%d", img_ready, depth_ready, info_ready);
        
        // 验证获取的数据是否有效，但降低严格性以适应降低帧率的情况
        if (imgColor_.empty() || imgDepth_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Invalid camera data received - image or depth is empty");
            res->pose_list.header.frame_id = "error";
            res->pose_list.header.stamp = this->get_clock()->now();
            return;
        }
        
        // 验证相机内参是否有效，但降低严格性
        bool valid_camera_info = true;
        for (size_t i = 0; i < camInfo_.k.size(); ++i) {
            if (std::isnan(camInfo_.k[i]) || std::isinf(camInfo_.k[i])) {
                valid_camera_info = false;
                break;
            }
        }
        if (!valid_camera_info) {
            RCLCPP_ERROR(this->get_logger(), "Invalid camera info received - intrinsic parameters contain NaN or Inf");
            res->pose_list.header.frame_id = "error";
            res->pose_list.header.stamp = this->get_clock()->now();
            return;
        }

        img_ready = depth_ready = info_ready = false;

        // 验证数据有效性 - 确保使用的是新数据
        bool data_is_fresh = true;
        auto current_time = this->get_clock()->now();
        auto img_age = (current_time - last_img_time_).seconds();
        auto depth_age = (current_time - last_depth_time_).seconds();
        
        if (img_age > 0.5 || depth_age > 0.5) {
            RCLCPP_WARN(this->get_logger(), "Using potentially stale data - img age: %f s, depth age: %f s", img_age, depth_age);
            data_is_fresh = false;
        }
        
        if (!data_is_fresh) {
            RCLCPP_WARN(this->get_logger(), "Data may not be fresh, but proceeding with calculation");
        }

// 简化的单帧处理 - 保留图像尺寸匹配修复
        RCLCPP_INFO(this->get_logger(), "Processing camera data...");
        
        // 执行孔洞检测和点云转换
        holeDetector(imgColor_, imgDepth_, camInfo_, cloud_holes_);
        imageToPointCloud(imgColor_, imgDepth_, camInfo_, cloud_);
        
        RCLCPP_INFO(this->get_logger(), "hole detect done, holes size=%zu", cloud_holes_->size());
        RCLCPP_INFO(this->get_logger(), "image->pointcloud done, cloud size=%zu", cloud_->points.size());
        RCLCPP_INFO(this->get_logger(), "norm calc start!");
        normCalc(cloud_holes_, cloud_, cloud_downSampled_, cloud_filtered_, cloud_smoothed_, cloud_normals_, cloud_with_normals_, cloud_shrink_, cloud_tarPoint_, triangles_, tarPointList_, inChiselParam, inGridParam);
        RCLCPP_INFO(this->get_logger(), "norm calc done, tarPoint size maybe=%zu", cloud_tarPoint_->points.size());
        
        

#ifdef Cam2Tool_TF
        Eigen::Matrix4d Cam2Tool_Matrix = Eigen::Matrix4d::Identity();
        Cam2Tool_Matrix(0, 0) = 0.002136129093849748;
        Cam2Tool_Matrix(0, 1) = 0.0131617748027531;
        Cam2Tool_Matrix(0, 2) = 0.9999110983665177;
        Cam2Tool_Matrix(0, 3) = -0.3592755296727744;
        Cam2Tool_Matrix(1, 0) = 0.9998109504292265;
        Cam2Tool_Matrix(1, 1) = -0.01935263737141502;
        Cam2Tool_Matrix(1, 2) = -0.001881177444387261;
        Cam2Tool_Matrix(1, 3) = -0.03444265988510906;
        Cam2Tool_Matrix(2, 0) = 0.01932615725645348;
        Cam2Tool_Matrix(2, 1) = 0.9997260840404296;
        Cam2Tool_Matrix(2, 2) = -0.01320062630660779;
        Cam2Tool_Matrix(2, 3) = -0.2227647594718163;
        Cam2Tool_Matrix(3, 0) = 0;
        Cam2Tool_Matrix(3, 1) = 0;
        Cam2Tool_Matrix(3, 2) = 0;
        Cam2Tool_Matrix(3, 3) = 1;

        Eigen::Matrix4d Tool2World_Matrix = Eigen::Matrix4d::Identity();
        Tool2World_Matrix(0, 0) = 1;
        Tool2World_Matrix(0, 1) = 0;
        Tool2World_Matrix(0, 2) = 0;
        Tool2World_Matrix(0, 3) = 0;
        Tool2World_Matrix(1, 0) = 0;
        Tool2World_Matrix(1, 1) = 1;
        Tool2World_Matrix(1, 2) = 0;
        Tool2World_Matrix(1, 3) = 0;
        Tool2World_Matrix(2, 0) = 0;
        Tool2World_Matrix(2, 1) = 0;
        Tool2World_Matrix(2, 2) = 1;
        Tool2World_Matrix(2, 3) = 0;
        Tool2World_Matrix(3, 0) = 0;
        Tool2World_Matrix(3, 1) = 0;
        Tool2World_Matrix(3, 2) = 0;
        Tool2World_Matrix(3, 3) = 1;
        Eigen::Matrix4d TF_Matrix = Cam2Tool_Matrix;
#ifdef Tool2World
        TF_Matrix = Tool2World_Matrix * TF_Matrix;
#endif

#ifdef EDGE_CHISEL_MODE
        size_t maxTarNum = inGridParam.GRID_ROW;
#else
        size_t maxTarNum = inChiselParam.BOX_ROW * inChiselParam.BOX_COLUMN;
#endif

        for (int i = 0; i < maxTarNum; ++i)
        {
            if (tarPointList_[i].status)
            {
                if (Coordinate_TF(tarPointList_[i], TF_Matrix))
                {
                    // RCLCPP_INFO(this->get_logger(), "Coordinate Transform Done");
                }
                if (Vector2Euler(tarPointList_[i]))
                {
                    // RCLCPP_INFO(this->get_logger(), "Normal Coordinate Transform Done");
                    tarPointList_[i].nz = (rand() / double(RAND_MAX) - 0.5) * 30.0 / 180.0 * 3.14159;
                    // RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, z: %f, A: %f, B: %f, C %f",tarPointList_[i].ox, tarPointList_[i].oy ,tarPointList_[i].oz, tarPointList_[i].nx, tarPointList_[i].ny,tarPointList_[i].nz);
                }
            }
        }
        RCLCPP_INFO(this->get_logger(), "Normal Coordinate Transform Done");
#endif

        // fill response
        for (int i = 0; i < maxTarNum; ++i)
        {
            if (tarPointList_[i].status)
            {
                geometry_msgs::msg::Pose pose;
                pose.position.x = tarPointList_[i].ox;
                pose.position.y = tarPointList_[i].oy;
                pose.position.z = tarPointList_[i].oz;
                pose.orientation.x = tarPointList_[i].nx;
                pose.orientation.y = tarPointList_[i].ny;
                pose.orientation.z = tarPointList_[i].nz;
                pose.orientation.w = tarPointList_[i].curv;
                res->pose_list.poses.push_back(pose);
            }
        }

        // header
        auto now = this->get_clock()->now();

        res->pose_list.header.stamp = now;
        res->pose_list.header.frame_id = cloud_->header.frame_id;

        RCLCPP_INFO(this->get_logger(), "POSE LIST SIZE: %zu", res->pose_list.poses.size());

#ifdef USING_SMOOTH
        pcl::toROSMsg(*cloud_smoothed_, outSmoothedCloud_);
#else
        pcl::toROSMsg(*cloud_downSampled_, outSmoothedCloud_);
#endif
        pcl::toROSMsg(*cloud_tarPoint_, outTarPointCloud_);
        pcl::toROSMsg(*cloud_with_normals_, outAllNormCloud_);
        pcl::toROSMsg(*cloud_holes_, outHolesCloud_);
        pcl_conversions::fromPCL(*triangles_, outTriangles_);

        outSmoothedCloud_.header.frame_id = res->pose_list.header.frame_id;
        outTarPointCloud_.header.frame_id = res->pose_list.header.frame_id;
        outAllNormCloud_.header.frame_id = res->pose_list.header.frame_id;
        outHolesCloud_.header.frame_id = res->pose_list.header.frame_id;
        outTriangles_.header.frame_id = res->pose_list.header.frame_id;

        outSmoothedCloud_.header.stamp = now;
        outTarPointCloud_.header.stamp = now;
        outAllNormCloud_.header.stamp = now;
        outHolesCloud_.header.stamp = now;
        outTriangles_.header.stamp = now;

        // 设置header信息
        outSmoothedCloud_.header.frame_id = res->pose_list.header.frame_id;
        outTarPointCloud_.header.frame_id = res->pose_list.header.frame_id;
        outAllNormCloud_.header.frame_id = res->pose_list.header.frame_id;
        outHolesCloud_.header.frame_id = res->pose_list.header.frame_id;
        outTriangles_.header.frame_id = res->pose_list.header.frame_id;

        outSmoothedCloud_.header.stamp = now;
        outTarPointCloud_.header.stamp = now;
        outAllNormCloud_.header.stamp = now;
        outHolesCloud_.header.stamp = now;
        outTriangles_.header.stamp = now;

        // Conditionally publish based on separate data transmission mode
        int pubNum = 0;
        while (pubNum < 2 && rclcpp::ok())
        {
            // Publish all data independently - allows for separate point cloud and RGB data transmission
            pubSmoothedData->publish(outSmoothedCloud_);
            pubTarPointData->publish(outTarPointCloud_);
            pubAllNormData->publish(outAllNormCloud_);
            pubHolesData->publish(outHolesCloud_);
            pubTrianglesData->publish(outTriangles_);
            pubNormCalcResult->publish(res->pose_list);  // Publish the norm calculation results
            pubNum++;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            rate.sleep();
        }
        
        // 内存清理：清理临时数据结构
        // 清理输出点云数据 - 在发布后进行清理
        outSmoothedCloud_.data.clear();
        outSmoothedCloud_.data.shrink_to_fit();
        outTarPointCloud_.data.clear();
        outTarPointCloud_.data.shrink_to_fit();
        outAllNormCloud_.data.clear();
        outAllNormCloud_.data.shrink_to_fit();
        outHolesCloud_.data.clear();
        outHolesCloud_.data.shrink_to_fit();
        outTriangles_.polygons.clear();
        outTriangles_.polygons.shrink_to_fit();
        
        // 清理内部点云数据
        cloud_holes_->clear();
        cloud_in_->clear();
        cloud_->clear();
        cloud_downSampled_->clear();
        cloud_filtered_->clear();
        cloud_smoothed_->clear();
        cloud_normals_->clear();
        cloud_with_normals_->clear();
        cloud_shrink_->clear();
        cloud_tarPoint_->clear();
        
        // 重置结果列表
        memset(tarPointList_, 0, sizeof(tarPointList_));
    }
};

// 点云质量监控实现
NormCalcServer::CloudQualityMetrics NormCalcServer::evaluateCloudQuality(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    CloudQualityMetrics metrics;
    
    if (cloud->points.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("norm_calc"), "Empty cloud for quality evaluation");
        return metrics;
    }
    
    metrics.total_points = cloud->points.size();
    
    // 1. 计算点密度（点数/工作区域体积）
    float working_area_volume = (inChiselParam.XMAX - inChiselParam.XMIN) * 
                               (inChiselParam.YMAX - inChiselParam.YMIN) * 
                               (inChiselParam.ZMAX - inChiselParam.ZMIN);
    metrics.point_density = working_area_volume > 0 ? metrics.total_points / working_area_volume : 0.0f;
    
    // 2. 计算深度方差
    metrics.depth_variance = calculateDepthVariance(cloud);
    
    // 3. 计算覆盖率（在有效工作区域内的点比例）
    metrics.coverage_ratio = calculateCoverageRatio(cloud);
    
    // 4. 综合质量评分
    float density_score = std::min(metrics.point_density / 1000.0f, 1.0f) * 0.4f;
    float variance_score = std::min(metrics.depth_variance / 0.01f, 1.0f) * 0.3f;
    float coverage_score = metrics.coverage_ratio * 0.3f;
    
    metrics.overall_score = density_score + variance_score + coverage_score;
    
    return metrics;
}

float NormCalcServer::calculateDepthVariance(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    if (cloud->points.size() < 2) return 0.0f;
    
    // 计算Z值的均值和方差
    double sum_z = 0.0;
    size_t valid_points = 0;
    
    for (const auto& point : cloud->points) {
        if (std::isfinite(point.z)) {
            sum_z += point.z;
            valid_points++;
        }
    }
    
    if (valid_points == 0) return 0.0f;
    
    double mean_z = sum_z / valid_points;
    double sum_variance = 0.0;
    
    for (const auto& point : cloud->points) {
        if (std::isfinite(point.z)) {
            double diff = point.z - mean_z;
            sum_variance += diff * diff;
        }
    }
    
    return valid_points > 1 ? static_cast<float>(sum_variance / (valid_points - 1)) : 0.0f;
}

float NormCalcServer::calculateCoverageRatio(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    if (cloud->points.empty()) return 0.0f;
    
    // 将工作区域划分为网格，计算有数据的网格比例
    const int grid_size = 20; // 20x20网格
    const float x_step = (inChiselParam.XMAX - inChiselParam.XMIN) / grid_size;
    const float y_step = (inChiselParam.YMAX - inChiselParam.YMIN) / grid_size;
    
    std::vector<std::vector<bool>> grid(grid_size, std::vector<bool>(grid_size, false));
    
    for (const auto& point : cloud->points) {
        if (std::isfinite(point.x) && std::isfinite(point.y)) {
            int x_idx = static_cast<int>((point.x - inChiselParam.XMIN) / x_step);
            int y_idx = static_cast<int>((point.y - inChiselParam.YMIN) / y_step);
            
            if (x_idx >= 0 && x_idx < grid_size && y_idx >= 0 && y_idx < grid_size) {
                grid[x_idx][y_idx] = true;
            }
        }
    }
    
    // 计算被覆盖的网格数量
    size_t covered_cells = 0;
    for (int i = 0; i < grid_size; i++) {
        for (int j = 0; j < grid_size; j++) {
            if (grid[i][j]) covered_cells++;
        }
    }
    
    return static_cast<float>(covered_cells) / (grid_size * grid_size);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NormCalcServer>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin(); // 会并行调度回调
    rclcpp::shutdown();
    return 0;
}

