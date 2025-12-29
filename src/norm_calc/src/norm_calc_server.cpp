#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "pcl_msgs/msg/polygon_mesh.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <Eigen/Dense>
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <thread>

#include "norm_calc/chisel_box.h"
#include "norm_calc/edge_grid.h"
#include "norm_calc/hole_detector.h"
#include "norm_calc/norm_calc.h"
#include "norm_calc/srv/norm_calc_data.hpp"

#define MULTI_FRAME_NUM 3
#define Cam2Tool_TF
#define Tool2World

using std::placeholders::_1;
using std::placeholders::_2;

class NormCalcServer : public rclcpp::Node {
public:
  NormCalcServer() : Node("norm_calc_server") {
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
    this->declare_parameter<double>("MIN_DISTANCE_THRESHOLD", 0.03);

    // initialize params into structs
    srand((unsigned)time(NULL));
    inChiselParam.BOX_LEN = this->get_parameter("BOX_LEN").as_double();
    inChiselParam.BOX_ROW = this->get_parameter("BOX_ROW").as_int();
    inChiselParam.BOX_COLUMN = this->get_parameter("BOX_COLUMN").as_int();
    inChiselParam.XMIN = this->get_parameter("XMIN").as_double();
    inChiselParam.XMAX = this->get_parameter("XMAX").as_double();
    inChiselParam.YMIN = this->get_parameter("YMIN").as_double();
    inChiselParam.YMAX = this->get_parameter("YMAX").as_double();
    inChiselParam.ZMIN = this->get_parameter("ZMIN").as_double();
    inChiselParam.ZMAX = this->get_parameter("ZMAX").as_double();
    inChiselParam.BORDER_WIDTH =
        this->get_parameter("BORDER_WIDTH").as_double();
    inChiselParam.NORM_TH = this->get_parameter("NORM_TH").as_double();
    inChiselParam.AFFECT_RADIUS =
        this->get_parameter("AFFECT_RADIUS").as_double();
    inChiselParam.HEIGHT_WEIGHT =
        this->get_parameter("HEIGHT_WEIGHT").as_double();
    inChiselParam.CURV_WEIGHT = this->get_parameter("CURV_WEIGHT").as_double();
    inChiselParam.ANGLE_WEIGHT =
        this->get_parameter("ANGLE_WEIGHT").as_double();
    inChiselParam.SEARCH_RADIUS =
        this->get_parameter("SEARCH_RADIUS").as_double();
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
    inChiselParam.MIN_DISTANCE_THRESHOLD =
        this->get_parameter("MIN_DISTANCE_THRESHOLD").as_double();
    min_spacing_threshold_ = inChiselParam.MIN_DISTANCE_THRESHOLD;

    RCLCPP_INFO(this->get_logger(), "Grid initialized at nominal bounds:");
    RCLCPP_INFO(this->get_logger(), "BOX_LEN = %6f", inChiselParam.BOX_LEN);
    RCLCPP_INFO(this->get_logger(), "XMIN = %6f, YMIN = %6f",
                inChiselParam.XMIN, inChiselParam.YMIN);

    pubSmoothedData = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "smoothedData", 1);
    pubTarPointData = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "tarPointData", 1);
    pubAllNormData =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("allNormData", 1);
    pubHolesData =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("holesData", 1);
    pubTrianglesData =
        this->create_publisher<pcl_msgs::msg::PolygonMesh>("trianglesData", 1);
    pubNormCalcResult = this->create_publisher<geometry_msgs::msg::PoseArray>(
        "norm_calc_result", 1);
    pubVisualNormResult = this->create_publisher<geometry_msgs::msg::PoseArray>(
        "visual_norm_result", 1);
    pubCapturedImage =
        this->create_publisher<sensor_msgs::msg::Image>("captured_image", 1);

    callback_group = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    options.callback_group = callback_group;
    // 下面三個對應ros2 launch norm_calc norm_calc_launch.py
    // imgSub =
    // this->create_subscription<sensor_msgs::msg::Image>("camera/realsense2_camera/color/image_raw",
    // 1, std::bind(&NormCalcServer::imgCallback, this, _1),options); depthSub =
    // this->create_subscription<sensor_msgs::msg::Image>("camera/realsense2_camera/depth/image_rect_raw",
    // 1, std::bind(&NormCalcServer::depthCallback, this, _1),options);
    // camInfoSub =
    // this->create_subscription<sensor_msgs::msg::CameraInfo>("camera/realsense2_camera/depth/camera_info",
    // 1, std::bind(&NormCalcServer::camInfoCallback, this, _1),options);

    imgSub = this->create_subscription<sensor_msgs::msg::Image>(
        "camera/camera/color/image_raw", 1,
        std::bind(&NormCalcServer::imgCallback, this, _1), options);
    depthSub = this->create_subscription<sensor_msgs::msg::Image>(
        "camera/camera/aligned_depth_to_color/image_raw", 1,
        std::bind(&NormCalcServer::depthCallback, this, _1), options);
    camInfoSub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera/camera/aligned_depth_to_color/camera_info", 1,
        std::bind(&NormCalcServer::camInfoCallback, this, _1), options);

    RCLCPP_INFO(this->get_logger(), "发布者订阅者创建成功.");

    service_ = this->create_service<norm_calc::srv::NormCalcData>(
        "norm_calc", std::bind(&NormCalcServer::getTheNorm, this, _1, _2));
    // node-level subscriptions are created inside service callback when needed
  }

private:
  // ======================= 成员定义 ==========================
  // 区域5帧融合功能
  bool enhanceRegionWithMultiFrame(
      int targetRow, int targetColumn,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr enhanced_cloud,
      pcl::PointCloud<pcl::PointXYZ>::Ptr enhanced_holes,
      chisel_box::ChiselParam chiselParam);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imgSub;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depthSub;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camInfoSub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSmoothedData;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubTarPointData;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubAllNormData;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubHolesData;
  rclcpp::Publisher<pcl_msgs::msg::PolygonMesh>::SharedPtr pubTrianglesData;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pubNormCalcResult;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr
      pubVisualNormResult;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pubCapturedImage;

  rclcpp::Service<norm_calc::srv::NormCalcData>::SharedPtr service_;
  rclcpp::CallbackGroup::SharedPtr callback_group;
  rclcpp::SubscriptionOptions options;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_holes_{
      new pcl::PointCloud<pcl::PointXYZ>};
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_{
      new pcl::PointCloud<pcl::PointXYZRGB>};
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_{
      new pcl::PointCloud<pcl::PointXYZRGB>};
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downSampled_{
      new pcl::PointCloud<pcl::PointXYZRGB>};
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_{
      new pcl::PointCloud<pcl::PointXYZRGB>};
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_smoothed_{
      new pcl::PointCloud<pcl::PointXYZRGB>};
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_{
      new pcl::PointCloud<pcl::Normal>};
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals_{
      new pcl::PointCloud<pcl::PointXYZRGBNormal>};
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_shrink_{
      new pcl::PointCloud<pcl::PointXYZRGBNormal>};
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_tarPoint_{
      new pcl::PointCloud<pcl::PointXYZRGBNormal>};
  pcl::PolygonMesh::Ptr triangles_{new pcl::PolygonMesh};
  chisel_box::ChiselNormPoint tarPointList_[24];
  std::vector<chisel_box::ChiselNormPoint> history_points_;
  double min_spacing_threshold_ = 0.03;

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
  double image_diff_threshold_ = 1000.0; // 图像差异阈值

  sensor_msgs::msg::PointCloud2 outSmoothedCloud_;
  sensor_msgs::msg::PointCloud2 outTarPointCloud_;
  sensor_msgs::msg::PointCloud2 outAllNormCloud_;
  sensor_msgs::msg::PointCloud2 outHolesCloud_;
  pcl_msgs::msg::PolygonMesh outTriangles_;

  chisel_box::ChiselParam inChiselParam;
  edge_grid::GridParam inGridParam;

  bool img_ShotFlag = false, depth_ShotFlag = false, info_ShotFlag = false;
  bool img_ready = false, depth_ready = false, info_ready = false;

  // Using Quaternion represent the rotation
  bool Coordinate_TF(chisel_box::ChiselNormPoint &Inputdata, float Translate_x,
                     float Translate_y, float Translate_z, float x, float y,
                     float z, float w) {
    Eigen::Quaterniond q(w, x, y, z);
    q.normalize();
    Eigen::Matrix3d rotation_matrix = q.matrix();
    Eigen::Vector3d Translate(Translate_x, Translate_y, Translate_z);

    Eigen::Vector3d PCam_Translate(Inputdata.ox, Inputdata.oy, Inputdata.oz);
    Eigen::Vector3d PTool_Tranlate =
        rotation_matrix * PCam_Translate + Translate;

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
  bool Coordinate_TF(chisel_box::ChiselNormPoint &Inputdata, float Translate_x,
                     float Translate_y, float Translate_z, float alpha,
                     float beta, float gama) {
    Eigen::AngleAxisd R_Angle(
        Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd P_Angle(
        Eigen::AngleAxisd(beta, Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd Y_Angle(
        Eigen::AngleAxisd(gama, Eigen::Vector3d::UnitZ()));
    // convert angle-axis to rotation matrices and multiply
    Eigen::Matrix3d rotation_matrix = Y_Angle.toRotationMatrix() *
                                      P_Angle.toRotationMatrix() *
                                      R_Angle.toRotationMatrix();
    Eigen::Vector3d Translate(Translate_x, Translate_y, Translate_z);

    Eigen::Vector3d PCam_Translate(Inputdata.ox, Inputdata.oy, Inputdata.oz);
    Eigen::Vector3d PTool_Tranlate =
        rotation_matrix * PCam_Translate + Translate;

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
  bool Coordinate_TF(chisel_box::ChiselNormPoint &Inputdata,
                     Eigen::Matrix4d Cam2Tool_Matrix) {
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

  bool Vector2Euler(chisel_box::ChiselNormPoint &Inputdata) {
    Eigen::Vector3f N_V(-1 * Inputdata.nx, -1 * Inputdata.ny,
                        -1 * Inputdata.nz);
    N_V = N_V / N_V.norm();
    float Yaw = atan2(N_V(1), N_V(0));
    float Pitch = -asin(N_V(2) / 1.0f);

    Inputdata.nx = Yaw;
    Inputdata.ny = Pitch;
    Inputdata.nz = 0;
    return true;
  }

  // subscribe color image
  void imgCallback(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg) {
    if (img_ShotFlag) {
      try {
        // 时间戳验证 - 检查图像是否为新数据
        auto current_time = this->get_clock()->now();
        rclcpp::Time image_time(img_msg->header.stamp);
        auto time_diff = (current_time - image_time).seconds();

        if (last_img_time_.nanoseconds() > 0 && image_time <= last_img_time_) {
          RCLCPP_WARN(this->get_logger(),
                      "Received old or duplicate color image timestamp");
          return;
        }

        if (time_diff > 2.0) { // 如果图像时间戳超过200ms，则认为可能过期
          RCLCPP_WARN(this->get_logger(),
                      "Color image may be stale: %f seconds ago", time_diff);
        }

        // RealSense publishes color as RGB8; use RGB8 and handle channels
        // accordingly
        cv_bridge::CvImagePtr cv_ptr =
            cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::RGB8);

        // 数据唯一性检查 - 与上一帧比较
        if (!last_imgColor_.empty()) {
          double diff = cv::norm(cv_ptr->image, last_imgColor_, cv::NORM_L2);
          if (diff < image_diff_threshold_) {
            RCLCPP_WARN(
                this->get_logger(),
                "Color image appears to be duplicate or unchanged (diff: %f)",
                diff);
            return;
          }
        }

        imgColor_ = cv_ptr->image.clone();
        last_imgColor_ = imgColor_.clone();
        last_img_time_ = image_time;
        img_ready = true;

        RCLCPP_INFO(this->get_logger(),
                    "New valid color image received, timestamp diff: %f s",
                    time_diff);
      } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
      }
    }
  }

  // subscribe depth image
  void depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg) {
    if (depth_ShotFlag) {
      try {
        // 时间戳验证 - 检查深度图像是否为新数据
        auto current_time = this->get_clock()->now();
        rclcpp::Time depth_time(depth_msg->header.stamp);
        auto time_diff = (current_time - depth_time).seconds();

        if (last_depth_time_.nanoseconds() > 0 &&
            depth_time <= last_depth_time_) {
          RCLCPP_WARN(this->get_logger(),
                      "Received old or duplicate depth image timestamp");
          return;
        }

        if (time_diff > 0.2) { // 如果深度图像时间戳超过200ms，则认为可能过期
          RCLCPP_WARN(this->get_logger(),
                      "Depth image may be stale: %f seconds ago", time_diff);
        }

        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(
            depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);

        // 数据唯一性检查 - 与上一帧比较
        if (!last_imgDepth_.empty()) {
          double diff = cv::norm(cv_ptr->image, last_imgDepth_, cv::NORM_L2);
          if (diff < image_diff_threshold_) {
            RCLCPP_WARN(
                this->get_logger(),
                "Depth image appears to be duplicate or unchanged (diff: %f)",
                diff);
            return;
          }
        }

        imgDepth_ = cv_ptr->image.clone();
        last_imgDepth_ = imgDepth_.clone();
        last_depth_time_ = depth_time;
        depth_ready = true;

        RCLCPP_INFO(this->get_logger(),
                    "New valid depth image received, timestamp diff: %f s",
                    time_diff);
      } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
      }
    }
  }

  // subscribe camera info
  void
  camInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &cam_msg) {
    if (info_ShotFlag) {
      // RCLCPP_INFO(this->get_logger(), "camInfo回调成功!!");
      camInfo_ = *cam_msg;
      info_ready = true;
    }
  }

  void imageToPointCloud(cv::Mat colorImg, cv::Mat depthImg,
                         const sensor_msgs::msg::CameraInfo &camInfo,
                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
    cloud->points.clear();

    // 检查图像是否为空
    if (colorImg.empty() || depthImg.empty()) {
      RCLCPP_ERROR(rclcpp::get_logger("norm_calc"),
                   "Empty color or depth image in imageToPointCloud");
      return;
    }

    // 检查图像尺寸是否匹配
    if (colorImg.rows != depthImg.rows || colorImg.cols != depthImg.cols) {
      RCLCPP_WARN(rclcpp::get_logger("norm_calc"),
                  "Color and depth image dimensions do not match: color(%dx%d) "
                  "vs depth(%dx%d)",
                  colorImg.cols, colorImg.rows, depthImg.cols, depthImg.rows);
      // 尝试使用较小的尺寸
      int rows = std::min(colorImg.rows, depthImg.rows);
      int cols = std::min(colorImg.cols, depthImg.cols);

      // 确保相机内参矩阵有足够的元素
      if (camInfo.k.size() < 9) {
        RCLCPP_ERROR(rclcpp::get_logger("norm_calc"),
                     "Insufficient camera intrinsic parameters");
        return;
      }

      pcl::PointXYZRGB p;
      cv::Mat grayImg, blurImg;
      cv::cvtColor(colorImg(cv::Rect(0, 0, cols, rows)), grayImg,
                   cv::COLOR_BGR2GRAY);
      cv::GaussianBlur(grayImg, blurImg, cv::Size(17, 9), 5, 0);

      for (int row = 0; row < rows; row++) {
        for (int col = 0; col < cols; col++) {
          // 检查深度图像边界
          if (row >= depthImg.rows || col >= depthImg.cols) {
            continue;
          }

          p.z = 0.001f * depthImg.at<uint16_t>(row, col);

          // 检查相机内参有效性
          if (camInfo.k[0] == 0 || camInfo.k[4] == 0) {
            continue;
          }

          p.x = (col - camInfo.k[2]) / camInfo.k[0] * p.z;
          p.y = (row - camInfo.k[5]) / camInfo.k[4] * p.z;

          // 确保颜色图像有足够的通道数且边界检查
          if (colorImg.channels() >= 3 && row < colorImg.rows &&
              col < colorImg.cols) {
            p.r = colorImg.at<cv::Vec3b>(row, col)[0]; // BGR format
            p.g = colorImg.at<cv::Vec3b>(row, col)[1];
            p.b = colorImg.at<cv::Vec3b>(row, col)[2];
          } else {
            p.r = p.g = p.b = 128; // 默认灰色
          }

          // 确保blurImg尺寸匹配
          if (row < blurImg.rows && col < blurImg.cols) {
            p.a = blurImg.at<uint8_t>(row, col);
          } else {
            p.a = 128; // 默认值
          }

          // 只添加有效点
          if (p.z > 0 && !std::isnan(p.x) && !std::isnan(p.y) &&
              !std::isnan(p.z)) {
            cloud->points.push_back(p);
          }
        }
      }
    } else {
      // 图像尺寸匹配的情况
      if (camInfo.k.size() < 9) {
        RCLCPP_ERROR(rclcpp::get_logger("norm_calc"),
                     "Insufficient camera intrinsic parameters");
        return;
      }

      pcl::PointXYZRGB p;
      cv::Mat grayImg, blurImg;
      cv::cvtColor(colorImg, grayImg, cv::COLOR_BGR2GRAY);
      cv::GaussianBlur(grayImg, blurImg, cv::Size(17, 9), 5, 0);

      for (int row = 0; row < depthImg.rows; row++) {
        for (int col = 0; col < depthImg.cols; col++) {
          // 检查深度图像边界
          if (row >= depthImg.rows || col >= depthImg.cols) {
            continue;
          }

          p.z = 0.001f * depthImg.at<uint16_t>(row, col);

          // 检查相机内参有效性
          if (camInfo.k[0] == 0 || camInfo.k[4] == 0) {
            continue;
          }

          p.x = (col - camInfo.k[2]) / camInfo.k[0] * p.z;
          p.y = (row - camInfo.k[5]) / camInfo.k[4] * p.z;

          // 使用更安全的颜色访问方式，并添加边界检查
          if (colorImg.channels() >= 3 && row < colorImg.rows &&
              col < colorImg.cols) {
            p.r = colorImg.at<cv::Vec3b>(
                row, col)[2]; // BGR format, R is the 3rd channel
            p.g = colorImg.at<cv::Vec3b>(row, col)[1]; // G is the 2nd channel
            p.b = colorImg.at<cv::Vec3b>(row, col)[0]; // B is the 1st channel
          } else {
            p.r = p.g = p.b = 128; // 默认灰色
          }

          // 确保blurImg尺寸匹配
          if (row < blurImg.rows && col < blurImg.cols) {
            p.a = blurImg.at<uint8_t>(row, col);
          } else {
            p.a = 128; // 默认值
          }

          // 只添加有效点
          if (p.z > 0 && !std::isnan(p.x) && !std::isnan(p.y) &&
              !std::isnan(p.z)) {
            cloud->points.push_back(p);
          }
        }
      }
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);
  }

  // Service 回调
  void
  getTheNorm(const std::shared_ptr<norm_calc::srv::NormCalcData::Request> req,
             std::shared_ptr<norm_calc::srv::NormCalcData::Response> res) {
    RCLCPP_INFO(this->get_logger(), "NormCalc service called.");
    rclcpp::Rate rate(10);
    memset(tarPointList_, 0, sizeof(tarPointList_));
    frameNum_ = 0;

    img_ShotFlag = depth_ShotFlag = info_ShotFlag =
        true; // 回调信号置1,三个sub线程执行回调，接受相机数据

    // 调整等待时间，适应提高帧率到15fps后的情况
    int wait_count = 0;
    const int max_wait_count = 45; // 45 * 100ms = 4.5秒，给更高帧率更多时间
    while (rclcpp::ok() && (!img_ready || !depth_ready || !info_ready)) {
      if (wait_count >= max_wait_count) {
        RCLCPP_ERROR(this->get_logger(),
                     "Camera data timeout after 4.5 seconds. img_ready:%d, "
                     "depth_ready:%d, info_ready:%d",
                     img_ready, depth_ready, info_ready);
        img_ShotFlag = depth_ShotFlag = info_ShotFlag = false; // 回调信号置0
        break; // 继续执行，使用可能不完整的数据
      }

      if (wait_count % 10 == 0) { // 每秒打印一次状态
        RCLCPP_WARN(this->get_logger(),
                    "NOT ready yet.img_ready:%d,depth_ready:%d,info_ready:%d "
                    "(waited %d.%ds)",
                    img_ready, depth_ready, info_ready, wait_count / 10,
                    wait_count % 10);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      wait_count++;
    }

    img_ShotFlag = depth_ShotFlag = info_ShotFlag = false; // 回调信号置0
    RCLCPP_INFO(this->get_logger(),
                "Ready.img_ready:%d,depth_ready:%d,info_ready:%d", img_ready,
                depth_ready, info_ready);

    // 验证获取的数据是否有效，但降低严格性以适应降低帧率的情况
    if (imgColor_.empty() || imgDepth_.empty()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Invalid camera data received - image or depth is empty");
      res->pose_list.header.frame_id = "error";
      res->pose_list.header.stamp = this->get_clock()->now();
      return;
    }

    // 检查图像尺寸是否有效
    if (imgColor_.rows <= 0 || imgColor_.cols <= 0 || imgDepth_.rows <= 0 ||
        imgDepth_.cols <= 0) {
      RCLCPP_ERROR(this->get_logger(),
                   "Invalid image dimensions: color(%dx%d), depth(%dx%d)",
                   imgColor_.cols, imgColor_.rows, imgDepth_.cols,
                   imgDepth_.rows);
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
      RCLCPP_ERROR(this->get_logger(),
                   "Invalid camera info received - intrinsic parameters "
                   "contain NaN or Inf");
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
      RCLCPP_WARN(
          this->get_logger(),
          "Using potentially stale data - img age: %f s, depth age: %f s",
          img_age, depth_age);
      data_is_fresh = false;
    }

    if (!data_is_fresh) {
      RCLCPP_WARN(this->get_logger(),
                  "Data may not be fresh, but proceeding with calculation");
    }

    // 多帧融合处理 - 增加点云密度
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr accumulated_cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_holes(
        new pcl::PointCloud<pcl::PointXYZ>);

    RCLCPP_INFO(this->get_logger(),
                "Starting multi-frame accumulation (%d frames)...",
                MULTI_FRAME_NUM);

    for (int frame = 0; frame < MULTI_FRAME_NUM && rclcpp::ok(); frame++) {
      // 等待新帧数据
      img_ShotFlag = depth_ShotFlag = info_ShotFlag = true;
      img_ready = depth_ready = info_ready = false;

      // 等待数据就绪
      int wait_count = 0;
      const int max_wait = 30; // 3秒超时
      while ((!img_ready || !depth_ready || !info_ready) &&
             wait_count < max_wait && rclcpp::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        wait_count++;
      }

      img_ShotFlag = depth_ShotFlag = info_ShotFlag = false;

      if (img_ready && depth_ready && info_ready) {
        // 处理当前帧
        pcl::PointCloud<pcl::PointXYZ>::Ptr frame_holes(
            new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr frame_cloud(
            new pcl::PointCloud<pcl::PointXYZRGB>);

        // 目标对齐分辨率：使用深度相机的原生分辨率（例如848x480）以保证最佳法向精度
        cv::Size targetSize(848, 480);
        cv::Mat resizedColor, resizedDepth;

        // 如果原始图不是848x480（驱动对齐后的可能是1280x720），则统一缩放到480p进行计算
        if (imgColor_.size() != targetSize) {
          cv::resize(imgColor_, resizedColor, targetSize);
        } else {
          resizedColor = imgColor_;
        }

        if (imgDepth_.size() != targetSize) {
          cv::resize(imgDepth_, resizedDepth, targetSize, 0, 0,
                     cv::INTER_NEAREST);
        } else {
          resizedDepth = imgDepth_;
        }

        // 缩放内参以匹配 848x480 分辨率
        sensor_msgs::msg::CameraInfo scaled_info = camInfo_;
        if (scaled_info.width != (uint32_t)targetSize.width ||
            scaled_info.height != (uint32_t)targetSize.height) {
          double sw = (double)targetSize.width / scaled_info.width;
          double sh = (double)targetSize.height / scaled_info.height;
          scaled_info.k[0] *= sw;
          scaled_info.k[2] *= sw;
          scaled_info.k[4] *= sh;
          scaled_info.k[5] *= sh;
          scaled_info.width = targetSize.width;
          scaled_info.height = targetSize.height;
        }

        holeDetector(resizedColor, resizedDepth, scaled_info, frame_holes);
        imageToPointCloud(resizedColor, resizedDepth, scaled_info, frame_cloud);

        // 累积数据
        if (frame_cloud->points.size() > 1000) { // 确保有足够的点
          *accumulated_cloud += *frame_cloud;
          if (frame_holes->points.size() > 0) {
            *accumulated_holes += *frame_holes;
          }
          RCLCPP_INFO(this->get_logger(),
                      "Frame %d accumulated: %zu points, %zu holes", frame + 1,
                      frame_cloud->points.size(), frame_holes->points.size());
        } else {
          RCLCPP_WARN(this->get_logger(),
                      "Frame %d skipped: insufficient points (%zu)", frame + 1,
                      frame_cloud->points.size());
        }

        img_ready = depth_ready = info_ready = false;

        // 帧间间隔，确保获取不同时刻的数据
        if (frame < MULTI_FRAME_NUM - 1) {
          std::this_thread::sleep_for(
              std::chrono::milliseconds(150)); // 150ms间隔
        }
      } else {
        RCLCPP_WARN(this->get_logger(),
                    "Frame %d timeout, using available data", frame + 1);
      }
    }

    // 使用累积的数据
    if (accumulated_cloud->points.size() > 5000) { // 确保有足够的数据
      *cloud_ = *accumulated_cloud;
      *cloud_holes_ = *accumulated_holes;

      // 为保证对齐，确保最后的 imgColor_ 也是对齐到深度尺寸的（供后面发布使用）
      if (!imgColor_.empty() && imgColor_.size() != imgDepth_.size()) {
        cv::resize(imgColor_, imgColor_, imgDepth_.size());
      }

      RCLCPP_INFO(this->get_logger(),
                  "Multi-frame accumulation complete: %zu points, %zu holes",
                  cloud_->points.size(), cloud_holes_->size());
      RCLCPP_INFO(this->get_logger(), "norm calc start!");

      if (normCalc(cloud_holes_, accumulated_cloud, cloud_downSampled_,
                   cloud_filtered_, cloud_smoothed_, cloud_normals_,
                   cloud_with_normals_, cloud_shrink_, cloud_tarPoint_,
                   triangles_, tarPointList_, inChiselParam, inGridParam,
                   history_points_)) {
        RCLCPP_INFO(this->get_logger(), "norm calc done, tarPoint size=%zu",
                    cloud_tarPoint_->points.size());

        // Update history for next shot
        history_points_.clear();
        for (int i = 0; i < inChiselParam.BOX_ROW * inChiselParam.BOX_COLUMN;
             ++i) {
          if (tarPointList_[i].status) {
            history_points_.push_back(tarPointList_[i]);
          }
        }
      } else {
        RCLCPP_ERROR(this->get_logger(), "norm calc failed!");
      }
    } else {
      RCLCPP_ERROR(this->get_logger(),
                   "Insufficient accumulated data (%zu points), falling back "
                   "to single frame",
                   accumulated_cloud->points.size());

      // 同样处理单帧回退逻辑，确保输出图像尺寸一致
      cv::Size targetSize(848, 480);
      if (!imgColor_.empty() && imgColor_.size() != targetSize) {
        cv::resize(imgColor_, imgColor_, targetSize);
      }
      if (!imgDepth_.empty() && imgDepth_.size() != targetSize) {
        cv::resize(imgDepth_, imgDepth_, targetSize, 0, 0, cv::INTER_NEAREST);
      }

      // 更新camInfo以匹配缩放后的图像（如果需要）
      // 假设订阅的是depth_camera_info且depth是848x480，而imgDepth_也是848x480，则无需缩放。
      // 但为了鲁棒性，我们根据imgDepth的实际尺寸缩放相参。
      sensor_msgs::msg::CameraInfo scaled_info = camInfo_;
      if (scaled_info.width != (uint32_t)imgDepth_.cols ||
          scaled_info.height != (uint32_t)imgDepth_.rows) {
        double sw = (double)imgDepth_.cols / scaled_info.width;
        double sh = (double)imgDepth_.rows / scaled_info.height;
        scaled_info.k[0] *= sw;
        scaled_info.k[2] *= sw;
        scaled_info.k[4] *= sh;
        scaled_info.k[5] *= sh;
        scaled_info.width = imgDepth_.cols;
        scaled_info.height = imgDepth_.rows;
      }

      // 探测孔洞
      holeDetector(imgColor_, imgDepth_, scaled_info, cloud_holes_);
      // 生成点云
      imageToPointCloud(imgColor_, imgDepth_, scaled_info, cloud_);
      if (normCalc(cloud_holes_, cloud_, cloud_downSampled_, cloud_filtered_,
                   cloud_smoothed_, cloud_normals_, cloud_with_normals_,
                   cloud_shrink_, cloud_tarPoint_, triangles_, tarPointList_,
                   inChiselParam, inGridParam, history_points_)) {

        // Update history for next shot
        history_points_.clear();
        for (int i = 0; i < inChiselParam.BOX_ROW * inChiselParam.BOX_COLUMN;
             ++i) {
          if (tarPointList_[i].status) {
            history_points_.push_back(tarPointList_[i]);
          }
        }
      }
    }

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

    // Publish visual norms (camera frame, original vectors)
    geometry_msgs::msg::PoseArray visual_poses;
    visual_poses.header.stamp = this->get_clock()->now();
    visual_poses.header.frame_id = cloud_->header.frame_id;
    for (size_t i = 0; i < maxTarNum && i < 24; ++i) {
      if (tarPointList_[i].status) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = tarPointList_[i].ox;
        pose.position.y = tarPointList_[i].oy;
        pose.position.z = tarPointList_[i].oz;
        pose.orientation.x = tarPointList_[i].nx;
        pose.orientation.y = tarPointList_[i].ny;
        pose.orientation.z = tarPointList_[i].nz;
        pose.orientation.w = 1.0;
        visual_poses.poses.push_back(pose);
      }
    }
    // Data prepared for visual results, will be published in the loop below

    for (int i = 0; i < maxTarNum; ++i) {
      // 确保数组访问在边界内（24个元素，索引0-23）
      if (i >= 24) {
        RCLCPP_ERROR(this->get_logger(), "Array index out of bounds: %d >= 24",
                     i);
        break;
      }

      if (tarPointList_[i].status) {
        if (Coordinate_TF(tarPointList_[i], TF_Matrix)) {
          // RCLCPP_INFO(this->get_logger(), "Coordinate Transform Done");
        }
        if (Vector2Euler(tarPointList_[i])) {
          // RCLCPP_INFO(this->get_logger(), "Normal Coordinate Transform
          // Done");
          tarPointList_[i].nz =
              (rand() / double(RAND_MAX) - 0.5) * 30.0 / 180.0 * 3.14159;
          // RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, z: %f, A: %f, B: %f,
          // C %f",tarPointList_[i].ox, tarPointList_[i].oy
          // ,tarPointList_[i].oz, tarPointList_[i].nx,
          // tarPointList_[i].ny,tarPointList_[i].nz);
        }
      }
    }
    RCLCPP_INFO(this->get_logger(), "Normal Coordinate Transform Done");
#endif

    // fill response
    for (int i = 0; i < maxTarNum; ++i) {
      // 确保数组访问在边界内（24个元素，索引0-23）
      if (i >= 24) {
        RCLCPP_ERROR(this->get_logger(),
                     "Array index out of bounds in response: %d >= 24", i);
        break;
      }

      if (tarPointList_[i].status) {
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

    RCLCPP_INFO(this->get_logger(), "POSE LIST SIZE: %zu",
                res->pose_list.poses.size());

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
    while (pubNum < 2 && rclcpp::ok()) {
      // Publish all data independently - allows for separate point cloud and
      // RGB data transmission
      pubSmoothedData->publish(outSmoothedCloud_);
      pubTarPointData->publish(outTarPointCloud_);
      pubAllNormData->publish(outAllNormCloud_);
      pubHolesData->publish(outHolesCloud_);
      pubTrianglesData->publish(outTriangles_);
      pubNormCalcResult->publish(
          res->pose_list); // Publish the norm calculation results

      pubVisualNormResult->publish(visual_poses);
      RCLCPP_INFO(this->get_logger(), "Published visual result with %zu points",
                  visual_poses.poses.size());

      // Publish the captured image for the viewer
      if (!imgColor_.empty()) {
        std::string encoding = (imgColor_.channels() == 3) ? "bgr8" : "mono8";
        auto captured_img_msg =
            cv_bridge::CvImage(res->pose_list.header, encoding, imgColor_)
                .toImageMsg();
        pubCapturedImage->publish(*captured_img_msg);
      }
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

// 区域5帧融合实现
bool NormCalcServer::enhanceRegionWithMultiFrame(
    int targetRow, int targetColumn,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr enhanced_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr enhanced_holes,
    chisel_box::ChiselParam chiselParam) {
  RCLCPP_INFO(this->get_logger(), "Starting 5-frame enhancement for Box[%d,%d]",
              targetRow, targetColumn);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr accumulated_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_holes(
      new pcl::PointCloud<pcl::PointXYZ>);

  // 计算目标区域的坐标范围
  float boxSize = chiselParam.BOX_LEN;
  float regionXMin = chiselParam.XMIN + targetColumn * boxSize;
  float regionXMax = regionXMin + boxSize;
  float regionYMin = chiselParam.YMIN + targetRow * boxSize;
  float regionYMax = regionYMin + boxSize;

  RCLCPP_INFO(this->get_logger(), "Target region: X[%.3f,%.3f], Y[%.3f,%.3f]",
              regionXMin, regionXMax, regionYMin, regionYMax);

  // 采集5帧数据
  for (int frame = 0; frame < 5; frame++) {
    // 等待新帧数据
    img_ShotFlag = depth_ShotFlag = info_ShotFlag = true;
    img_ready = depth_ready = info_ready = false;

    int wait_count = 0;
    while ((!img_ready || !depth_ready || !info_ready) && wait_count < 30 &&
           rclcpp::ok()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      wait_count++;
    }

    img_ShotFlag = depth_ShotFlag = info_ShotFlag = false;

    if (img_ready && depth_ready && info_ready) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr frame_holes(
          new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr frame_cloud(
          new pcl::PointCloud<pcl::PointXYZRGB>);

      // 为保证对齐，确保RGB尺寸匹配深度
      cv::Mat resizedColor;
      if (imgColor_.size() != imgDepth_.size()) {
        cv::resize(imgColor_, resizedColor, imgDepth_.size());
      } else {
        resizedColor = imgColor_;
      }

      // 缩放内参以匹配深度图像分辨率
      sensor_msgs::msg::CameraInfo scaled_info = camInfo_;
      if (scaled_info.width != (uint32_t)imgDepth_.cols ||
          scaled_info.height != (uint32_t)imgDepth_.rows) {
        double sw = (double)imgDepth_.cols / scaled_info.width;
        double sh = (double)imgDepth_.rows / scaled_info.height;
        scaled_info.k[0] *= sw;
        scaled_info.k[2] *= sw;
        scaled_info.k[4] *= sh;
        scaled_info.k[5] *= sh;
        scaled_info.width = imgDepth_.cols;
        scaled_info.height = imgDepth_.rows;
      }

      holeDetector(resizedColor, imgDepth_, scaled_info, frame_holes);
      imageToPointCloud(resizedColor, imgDepth_, scaled_info, frame_cloud);

      // 过滤出目标区域的点云
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr region_cloud(
          new pcl::PointCloud<pcl::PointXYZRGB>);
      for (const auto &point : frame_cloud->points) {
        if (point.x >= regionXMin && point.x <= regionXMax &&
            point.y >= regionYMin && point.y <= regionYMax &&
            point.z >= chiselParam.ZMIN && point.z <= chiselParam.ZMAX) {
          region_cloud->points.push_back(point);
        }
      }

      // 同样过滤孔洞数据
      pcl::PointCloud<pcl::PointXYZ>::Ptr region_holes(
          new pcl::PointCloud<pcl::PointXYZ>);
      for (const auto &hole : frame_holes->points) {
        if (hole.x >= regionXMin && hole.x <= regionXMax &&
            hole.y >= regionYMin && hole.y <= regionYMax &&
            hole.z >= chiselParam.ZMIN && hole.z <= chiselParam.ZMAX) {
          region_holes->points.push_back(hole);
        }
      }

      // 累积数据
      if (region_cloud->points.size() > 100) {
        *accumulated_cloud += *region_cloud;
        if (region_holes->points.size() > 0) {
          *accumulated_holes += *region_holes;
        }
        RCLCPP_INFO(this->get_logger(),
                    "Frame %d: %zu region points, %zu region holes", frame + 1,
                    region_cloud->points.size(), region_holes->points.size());
      }

      img_ready = depth_ready = info_ready = false;

      if (frame < 4) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
      }
    }
  }

  // 检查增强效果
  if (accumulated_cloud->points.size() > 1000) {
    *enhanced_cloud = *accumulated_cloud;
    *enhanced_holes = *accumulated_holes;
    RCLCPP_INFO(this->get_logger(),
                "Region enhancement successful: %zu points, %zu holes",
                enhanced_cloud->points.size(), enhanced_holes->points.size());
    return true;
  } else {
    RCLCPP_WARN(this->get_logger(),
                "Region enhancement failed: insufficient points (%zu)",
                accumulated_cloud->points.size());
    return false;
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NormCalcServer>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin(); // 会并行调度回调
  rclcpp::shutdown();
  return 0;
}
