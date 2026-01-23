#include "geometry_msgs/msg/pose_array.hpp"
#include "norm_calc/chisel_box.h"
#include "norm_calc/hole_detector.h"
#include "norm_calc/norm_calc.h"
#include "norm_calc/srv/norm_calc_data.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <Eigen/Dense>
#include <cv_bridge/cv_bridge.h>
#include <mutex>
#include <pcl_conversions/pcl_conversions.h>
#include <thread>
#include <pcl/io/pcd_io.h>
#include <filesystem>

using std::placeholders::_1;
using std::placeholders::_2;

class NormCalcServer : public rclcpp::Node {
public:
  NormCalcServer() : Node("norm_calc_server") {
    RCLCPP_INFO(this->get_logger(), "=== NormCalc Server Starting ===");

    readParameters();
    initGrids();

    // [新增] 初始化点云保存计数器：从pcd_data文件夹获取最后一个文件编号
    std::string pcd_dir = "/home/bosch/vChisel_ros2_ws/pcd_data";
    if (std::filesystem::exists(pcd_dir) && std::filesystem::is_directory(pcd_dir)) {
      int max_file_num = 0;
      try {
        for (const auto& entry : std::filesystem::directory_iterator(pcd_dir)) {
          if (entry.is_regular_file() && entry.path().extension() == ".pcd") {
            std::string filename = entry.path().stem().string();
            try {
              int file_num = std::stoi(filename);
              if (file_num > max_file_num) {
                max_file_num = file_num;
              }
            } catch (const std::exception& e) {
              // 忽略非数字文件名的文件
            }
          }
        }
        pcd_save_count_ = max_file_num;
        RCLCPP_INFO(this->get_logger(), "Initialized PCD save count from existing files: %d", pcd_save_count_);
      } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Failed to scan pcd_data directory: %s", e.what());
      }
    } else {
      // 创建pcd_data目录（如果不存在）
      try {
        std::filesystem::create_directories(pcd_dir);
        RCLCPP_INFO(this->get_logger(), "Created pcd_data directory: %s", pcd_dir.c_str());
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create pcd_data directory: %s", e.what());
        enable_pcd_save_ = false;
      }
    }

    // 1. 发布者 (调试点云 + 可视化图像 + 可视化结果)
    pub_debug_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "debug_processed_cloud", 1);
    pub_captured_image_ =
        this->create_publisher<sensor_msgs::msg::Image>("captured_image", 1);
    pub_visual_norm_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
        "visual_norm_result", 1);

    // 2. 回调组 (Reentrant 防止死锁)
    callback_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;

    // 3. QoS (Reliable + Volatile) 匹配 RealSense
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(rclcpp::ReliabilityPolicy::Reliable);
    qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);

    // 4. 订阅器
    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "camera/camera/color/image_raw", qos_profile,
        std::bind(&NormCalcServer::imgCallback, this, _1), options);

    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "camera/camera/aligned_depth_to_color/image_raw", qos_profile,
        std::bind(&NormCalcServer::depthCallback, this, _1), options);

    info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera/camera/aligned_depth_to_color/camera_info", qos_profile,
        std::bind(&NormCalcServer::infoCallback, this, _1), options);

    // 5. 服务
    srv_ = this->create_service<norm_calc::srv::NormCalcData>(
        "norm_calc", std::bind(&NormCalcServer::handleService, this, _1, _2),
        rmw_qos_profile_services_default, callback_group_);

    RCLCPP_INFO(this->get_logger(),
                "=== NormCalc Server Ready (Waiting for Service Call) ===");
  }

private:
  chisel_box::ChiselParam param_;
  float z_min_, z_max_, border_w_;
  float search_r_;
  int search_n_;

  std::vector<std::shared_ptr<chisel_box::ChiselBox>> grids_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr global_obstacles_{
      new pcl::PointCloud<pcl::PointXYZ>};
  pcl::PointCloud<pcl::PointXYZ>::Ptr last_success_points_{
      new pcl::PointCloud<pcl::PointXYZ>};  // 存储上一次成功的点位，用于避让

  std::mutex data_mutex_;
  cv::Mat img_color_, img_depth_;
  sensor_msgs::msg::CameraInfo cam_info_;
  bool img_ready_ = false, depth_ready_ = false, info_ready_ = false;

  // [新增] 点云保存计数器
  int pcd_save_count_ = 0;
  bool enable_pcd_save_ = true;  // 是否启用点云保存功能

  // 发布者句柄
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_debug_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_captured_image_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_visual_norm_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_, depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Service<norm_calc::srv::NormCalcData>::SharedPtr srv_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  // 手眼标定矩阵（相机坐标系到工具坐标系）
  Eigen::Matrix4d T_cam_tool_;

  void readParameters() {
    param_.BOX_LEN = this->declare_parameter("BOX_LEN", 0.05);
    param_.BOX_ROW = this->declare_parameter("BOX_ROW", 4);
    param_.BOX_COLUMN = this->declare_parameter("BOX_COLUMN", 6);
    param_.XMIN = this->declare_parameter("XMIN", -0.15);
    param_.XMAX = this->declare_parameter("XMAX", 0.15);
    param_.YMIN = this->declare_parameter("YMIN", -0.1);
    param_.YMAX = this->declare_parameter("YMAX", 0.1);

    z_min_ = this->declare_parameter("ZMIN", 0.2);
    z_max_ = this->declare_parameter("ZMAX", 0.8);
    border_w_ = this->declare_parameter("BORDER_WIDTH", 0.03);

    search_r_ = this->declare_parameter("SEARCH_RADIUS", 0.02);
    search_n_ = this->declare_parameter("SEARCH_NUM_TH", 30);

    param_.STRICT_NORM_TH = this->declare_parameter("STRICT_NORM_TH", 0.96);
    param_.STRICT_HOLE_DIST = this->declare_parameter("STRICT_HOLE_DIST", 0.05);
    param_.STRICT_CURV_TH = this->declare_parameter("STRICT_CURV_TH", 0.04);

    param_.RELAXED_NORM_TH = this->declare_parameter("RELAXED_NORM_TH", 0.86);
    param_.RELAXED_HOLE_DIST =
        this->declare_parameter("RELAXED_HOLE_DIST", 0.035);
    param_.RELAXED_CURV_TH = this->declare_parameter("RELAXED_CURV_TH", 0.10);

    param_.HEIGHT_WEIGHT = this->declare_parameter("HEIGHT_WEIGHT", 3.0);
    param_.CURV_WEIGHT = this->declare_parameter("CURV_WEIGHT", 2.0);
    param_.ANGLE_WEIGHT = this->declare_parameter("ANGLE_WEIGHT", 1.0);
    param_.CENTER_WEIGHT = this->declare_parameter("CENTER_WEIGHT", 5.0);

    // [新增] 读取凸起判定参数
    param_.PROTRUSION_CURV_TH = this->declare_parameter("PROTRUSION_CURV_TH", 0.04);
    param_.PROTRUSION_TH = this->declare_parameter("PROTRUSION_TH", 0.02);
    param_.TIP_CROP_RATIO = this->declare_parameter("TIP_CROP_RATIO", 0.25);
    param_.BASE_CROP_RATIO = this->declare_parameter("BASE_CROP_RATIO", 0.10);

    // [新增] 读取随机模式参数
    param_.RANDOM_OFFSET_RANGE = this->declare_parameter("RANDOM_OFFSET_RANGE", 0.02);
    param_.RANDOM_ANGLE_RANGE = this->declare_parameter("RANDOM_ANGLE_RANGE", 0.35);

    // [新增] 读取平面面积阈值参数
    param_.PLANE_AREA_HIGH = this->declare_parameter("PLANE_AREA_HIGH", 0.00035);
    param_.PLANE_AREA_LOW = this->declare_parameter("PLANE_AREA_LOW", 0.00025);

    // [新增] 读取Z-range阈值参数
    param_.Z_RANGE_HYBRID_TH = this->declare_parameter("Z_RANGE_HYBRID_TH", 0.03);
    param_.Z_RANGE_PROTRUSION_TH = this->declare_parameter("Z_RANGE_PROTRUSION_TH", 0.05);

    param_.HYBRID_NORM_TH = this->declare_parameter("HYBRID_NORM_TH", 0.885);
    param_.HYBRID_HOLE_DIST = this->declare_parameter("HYBRID_HOLE_DIST", 0.0425);
    param_.HYBRID_CURV_TH = this->declare_parameter("HYBRID_CURV_TH", 0.085);

    // [新增] 读取平面判定参数（基于曲率阈值内的面积比例）
    param_.PLANE_FLAT_RATIO_HIGH = this->declare_parameter("PLANE_FLAT_RATIO_HIGH", 0.70);
    param_.PLANE_FLAT_RATIO_LOW = this->declare_parameter("PLANE_FLAT_RATIO_LOW", 0.50);
    param_.PLANE_HOLE_DIST = this->declare_parameter("PLANE_HOLE_DIST", 0.015);
    param_.PLANE_ANGLE_WEIGHT_MULT = this->declare_parameter("PLANE_ANGLE_WEIGHT_MULT", 3.0);
    param_.PLANE_CURV_WEIGHT_MULT = this->declare_parameter("PLANE_CURV_WEIGHT_MULT", 5.0);
    param_.PLANE_CENTER_WEIGHT_MULT = this->declare_parameter("PLANE_CENTER_WEIGHT_MULT", 0.2);
    param_.PLANE_AVOID_CONCAVE_DIST = this->declare_parameter("PLANE_AVOID_CONCAVE_DIST", 0.015);

    // [新增] 读取凸起策略参数优化
    param_.PROTRUSION_ANGLE_WEIGHT_MULT = this->declare_parameter("PROTRUSION_ANGLE_WEIGHT_MULT", 2.0);
    param_.PROTRUSION_CURV_WEIGHT_MULT = this->declare_parameter("PROTRUSION_CURV_WEIGHT_MULT", 2.0);
    param_.PROTRUSION_HEIGHT_WEIGHT_MULT = this->declare_parameter("PROTRUSION_HEIGHT_WEIGHT_MULT", 0.5);
    param_.PROTRUSION_CENTER_WEIGHT_MULT = this->declare_parameter("PROTRUSION_CENTER_WEIGHT_MULT", 1.0);

    // 读取手眼标定矩阵参数
    std::vector<double> row1 = this->declare_parameter(
      "hand_eye_calibration.row1",
      std::vector<double>{1.0, -1.2734571168246914e-17, 1.3885605319037612e-17, -0.8175393051548504});
    std::vector<double> row2 = this->declare_parameter(
      "hand_eye_calibration.row2",
      std::vector<double>{1.2734571168246914e-17, 1.0, 8.725997245916549e-18, 0.10332821476190851});
    std::vector<double> row3 = this->declare_parameter(
      "hand_eye_calibration.row3",
      std::vector<double>{-1.3885605319037612e-17, -8.725997245916549e-18, 1.0, 0.10346660541091252});
    std::vector<double> row4 = this->declare_parameter(
      "hand_eye_calibration.row4",
      std::vector<double>{0.0, 0.0, 0.0, 1.0});

    // 构建手眼标定矩阵
    T_cam_tool_ = Eigen::Matrix4d::Identity();
    T_cam_tool_ << row1[0], row1[1], row1[2], row1[3],
                   row2[0], row2[1], row2[2], row2[3],
                   row3[0], row3[1], row3[2], row3[3],
                   row4[0], row4[1], row4[2], row4[3];

    // 输出标定矩阵
    RCLCPP_INFO(this->get_logger(), "=== 手眼标定矩阵已加载 ===");
    RCLCPP_INFO(this->get_logger(), "[%.6f, %.6f, %.6f, %.6f]",
      T_cam_tool_(0,0), T_cam_tool_(0,1), T_cam_tool_(0,2), T_cam_tool_(0,3));
    RCLCPP_INFO(this->get_logger(), "[%.6f, %.6f, %.6f, %.6f]",
      T_cam_tool_(1,0), T_cam_tool_(1,1), T_cam_tool_(1,2), T_cam_tool_(1,3));
    RCLCPP_INFO(this->get_logger(), "[%.6f, %.6f, %.6f, %.6f]",
      T_cam_tool_(2,0), T_cam_tool_(2,1), T_cam_tool_(2,2), T_cam_tool_(2,3));
    RCLCPP_INFO(this->get_logger(), "[%.6f, %.6f, %.6f, %.6f]",
      T_cam_tool_(3,0), T_cam_tool_(3,1), T_cam_tool_(3,2), T_cam_tool_(3,3));
    RCLCPP_INFO(this->get_logger(), "===========================");
  }

  void initGrids() {
    grids_.clear();
    for (int r = 0; r < param_.BOX_ROW; r++) {
      for (int c = 0; c < param_.BOX_COLUMN; c++) {
        grids_.push_back(std::make_shared<chisel_box::ChiselBox>(r, c, param_));
      }
    }
  }

  void imgCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    try {
      img_color_ = cv_bridge::toCvCopy(msg, "rgb8")->image;
      img_ready_ = true;
      static int count = 0;
      if (++count % 60 == 0)
        RCLCPP_INFO(this->get_logger(), "[Data Flow] RGB Image Received");
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Img Callback Error: %s", e.what());
    }
  }
  void depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    try {
      img_depth_ =
          cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1)
              ->image;
      depth_ready_ = true;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Depth Callback Error: %s", e.what());
    }
  }
  void infoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    cam_info_ = *msg;
    info_ready_ = true;
  }

  void generateRawPointCloud(const cv::Mat &color, const cv::Mat &depth,
                             const sensor_msgs::msg::CameraInfo &info,
                             pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
    if (depth.empty() || color.empty())
      return;
    if (info.k[0] == 0 || info.k[4] == 0)
      return;

    int rows = depth.rows;
    int cols = depth.cols;

    if (color.rows != rows || color.cols != cols) {
      RCLCPP_WARN(this->get_logger(),
                  "Size mismatch: Depth(%dx%d) vs Color(%dx%d)", cols, rows,
                  color.cols, color.rows);
      return;
    }

    cloud->reserve(rows * cols);

    for (int v = 0; v < rows; ++v) {
      for (int u = 0; u < cols; ++u) {
        uint16_t d = depth.at<uint16_t>(v, u);
        if (d == 0)
          continue;

        float z = d * 0.001f;
        float x = (u - info.k[2]) * z / info.k[0];
        float y = (v - info.k[5]) * z / info.k[4];

        if (z > 3.0 || z < 0.1)
          continue;

        pcl::PointXYZRGB pt;
        pt.x = x;
        pt.y = y;
        pt.z = z;

        auto &c = color.at<cv::Vec3b>(v, u);
        pt.r = c[0];
        pt.g = c[1];
        pt.b = c[2];

        cloud->push_back(pt);
      }
    }
  }

  void extractGridROI(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr full,
                      std::shared_ptr<chisel_box::ChiselBox> grid,
                      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr roi) {
    float x_min = param_.XMIN + grid->getCol() * param_.BOX_LEN;
    float x_max = x_min + param_.BOX_LEN;
    float y_min = param_.YMIN + grid->getRow() * param_.BOX_LEN;
    float y_max = y_min + param_.BOX_LEN;

    float margin = 0.01;

    for (const auto &pt : full->points) {
      if (pt.x >= x_min - margin && pt.x <= x_max + margin &&
          pt.y >= y_min - margin && pt.y <= y_max + margin) {
        roi->push_back(pt);
      }
    }
  }

  // 四元数转欧拉角（ZYX顺序，即绕Z-Y-X轴旋转）
  Eigen::Vector3d quaternionToEulerZYX(const Eigen::Quaterniond &q) {
    Eigen::Matrix3d R = q.toRotationMatrix();

    double sy = sqrt(R(0,0) * R(0,0) + R(1,0) * R(1,0));

    bool singular = sy < 1e-6;

    double x, y, z;
    if (!singular) {
      x = atan2(R(2,1), R(2,2));
      y = atan2(-R(2,0), sy);
      z = atan2(R(1,0), R(0,0));
    } else {
      x = atan2(-R(1,2), R(1,1));
      y = atan2(-R(2,0), sy);
      z = 0;
    }

    return Eigen::Vector3d(z, y, x); // 返回 ZYX 欧拉角（弧度）
  }

  // 欧拉角转四元数（ZYX顺序）
  Eigen::Quaterniond eulerZYXToQuaternion(double z, double y, double x) {
    Eigen::AngleAxisd rollAngle(x, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(y, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(z, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    return q;
  }

  // 限制角度在±30度范围内
  Eigen::Vector3d clampEulerAngles(const Eigen::Vector3d &euler_rad) {
    const double max_angle = M_PI / 6.0; // 30度 = π/6 弧度

    Eigen::Vector3d clamped;
    clamped(0) = std::max(-max_angle, std::min(max_angle, euler_rad(0))); // Z轴旋转
    clamped(1) = std::max(-max_angle, std::min(max_angle, euler_rad(1))); // Y轴旋转
    clamped(2) = std::max(-max_angle, std::min(max_angle, euler_rad(2))); // X轴旋转

    return clamped;
  }

  void transformPose(const pcl::PointXYZRGBNormal &target,
                     geometry_msgs::msg::Pose &pose) {
    // 使用配置文件中的手眼标定矩阵
    Eigen::Matrix4d T_cam_tool = T_cam_tool_;

    Eigen::Vector4d p_cam(target.x, target.y, target.z, 1.0);
    Eigen::Vector4d p_tool = T_cam_tool * p_cam;

    pose.position.x = p_tool(0);
    pose.position.y = p_tool(1);
    pose.position.z = p_tool(2);

    Eigen::Matrix3d R_cam_tool = T_cam_tool.block<3, 3>(0, 0);
    Eigen::Vector3d n_cam(target.normal_x, target.normal_y, target.normal_z);
    Eigen::Vector3d n_tool = R_cam_tool * n_cam;
    n_tool.normalize();

    Eigen::Vector3d tool_z_axis(0, 0, 1);
    Eigen::Quaterniond q =
        Eigen::Quaterniond::FromTwoVectors(tool_z_axis, -n_tool);

    // 将四元数转换为欧拉角（ZYX顺序）
    Eigen::Vector3d euler = quaternionToEulerZYX(q);

    // 限制欧拉角在±30度范围内
    Eigen::Vector3d clamped_euler = clampEulerAngles(euler);

    // 将限制后的欧拉角转换回四元数
    Eigen::Quaterniond clamped_q = eulerZYXToQuaternion(clamped_euler(0), clamped_euler(1), clamped_euler(2));

    pose.orientation.x = clamped_q.x();
    pose.orientation.y = clamped_q.y();
    pose.orientation.z = clamped_q.z();
    pose.orientation.w = clamped_q.w();
  }

  void handleService(
      const std::shared_ptr<norm_calc::srv::NormCalcData::Request> req,
      std::shared_ptr<norm_calc::srv::NormCalcData::Response> res) {
    RCLCPP_INFO(this->get_logger(), ">>> Processing Request Seq: %d", req->seq);

    // 重置所有网格状态，每次拍照计算都是从头开始
    for (auto &grid : grids_) {
      grid->reset();
    }
    RCLCPP_INFO(this->get_logger(), "State Reset: All grids reset for new calculation.");

    // 1. 等待数据
    {
      int wait_cnt = 0;
      RCLCPP_INFO(this->get_logger(), "Waiting for camera data...");
      while (wait_cnt < 30) {
        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          if (img_ready_ && depth_ready_ && info_ready_)
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        wait_cnt++;
      }
    }

    // 2. 快照
    cv::Mat color_snap, depth_snap;
    sensor_msgs::msg::CameraInfo info_snap;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!img_ready_ || !depth_ready_ || !info_ready_) {
        RCLCPP_ERROR(this->get_logger(), "TIMEOUT: Camera data not ready!");
        return;
      }
      color_snap = img_color_.clone();
      depth_snap = img_depth_.clone();
      info_snap = cam_info_;
      img_ready_ = depth_ready_ = false;
    }
    RCLCPP_INFO(this->get_logger(), "Data Acquired. Generating Cloud...");

    // 3. 【恢复发布】发布原始图像给 Viewer
    {
      cv_bridge::CvImage cv_img;
      cv_img.header = info_snap.header;
      cv_img.header.stamp = this->get_clock()->now();  // 更新时间戳以同步可视化
      cv_img.encoding = "rgb8";
      cv_img.image = color_snap;
      pub_captured_image_->publish(*cv_img.toImageMsg());
    }

    // 4. 生成点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    generateRawPointCloud(color_snap, depth_snap, info_snap, raw_cloud);

    if (raw_cloud->empty()) {
      RCLCPP_WARN(
          this->get_logger(),
          "Generated raw cloud is empty. Check Z range or depth image.");
      return;
    }

    // [新增] 保存点云到pcd文件
    if (enable_pcd_save_) {
      pcd_save_count_++;
      std::string pcd_dir = "/home/bosch/vChisel_ros2_ws/pcd_data";

      // 保存点云文件
      std::string pcd_filename = pcd_dir + "/" + std::to_string(pcd_save_count_) + ".pcd";
      try {
        pcl::io::savePCDFileBinary(pcd_filename, *raw_cloud);
        RCLCPP_INFO(this->get_logger(), "Saved point cloud to: %s (%zu points)",
                    pcd_filename.c_str(), raw_cloud->size());
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to save PCD file: %s", e.what());
      }
    }

    // 5. 检测空洞
    pcl::PointCloud<pcl::PointXYZ>::Ptr vision_holes(
        new pcl::PointCloud<pcl::PointXYZ>);
    holeDetector(color_snap, depth_snap, info_snap, vision_holes);
    RCLCPP_INFO(this->get_logger(), "[DEBUG] Detected %zu holes (obstacles)", vision_holes->size());

    // 清空障碍物列表，添加检测到的空洞和上一次成功的点位
    global_obstacles_->clear();
    *global_obstacles_ += *vision_holes;
    *global_obstacles_ += *last_success_points_;  // 添加上一次成功的点位用于避让
    RCLCPP_INFO(this->get_logger(), "[DEBUG] Added %zu previous success points as obstacles", last_success_points_->size());

    // 6. 预处理
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr processed_cloud(
        new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    bool ok = processPointCloud(
        raw_cloud, processed_cloud, param_.XMIN - border_w_,
        param_.XMAX + border_w_, param_.YMIN - border_w_,
        param_.YMAX + border_w_, z_min_, z_max_, search_r_, search_n_);

    if (!ok) {
      RCLCPP_WARN(this->get_logger(),
                  "Processed cloud empty (filtered out). Check ROI params.");
      return;
    }

    // 7. 发布调试点云
    sensor_msgs::msg::PointCloud2 debug_msg;
    pcl::toROSMsg(*processed_cloud, debug_msg);
    debug_msg.header.frame_id = info_snap.header.frame_id;
    pub_debug_cloud_->publish(debug_msg);

    // 8. 网格决策
    int plan_count = 0;
    // 用于可视化的 PoseArray (Camera Frame)
    geometry_msgs::msg::PoseArray visual_poses;
    visual_poses.header = info_snap.header;

    for (auto &grid : grids_) {
      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr roi_cloud(
          new pcl::PointCloud<pcl::PointXYZRGBNormal>);
      extractGridROI(processed_cloud, grid, roi_cloud);

      pcl::PointXYZRGBNormal target;
      if (grid->findBestPoint(roi_cloud, global_obstacles_, target)) {

        // 1. 存入结果 (Tool Frame)
        geometry_msgs::msg::Pose tool_pose;
        transformPose(target, tool_pose);
        tool_pose.orientation.w = target.curvature;
        res->pose_list.poses.push_back(tool_pose);

        // 2. 存入可视化 (Camera Frame)
        geometry_msgs::msg::Pose cam_pose;
        cam_pose.position.x = target.x;
        cam_pose.position.y = target.y;
        cam_pose.position.z = target.z;
        // 简单的法向可视化：向量
        cam_pose.orientation.x = target.normal_x;
        cam_pose.orientation.y = target.normal_y;
        cam_pose.orientation.z = target.normal_z;
        cam_pose.orientation.w = 0; // 标记
        visual_poses.poses.push_back(cam_pose);

        // 3. 立即将当前成功的点位添加到 global_obstacles_，用于后续网格避让
        pcl::PointXYZ pt;
        pt.x = target.x;
        pt.y = target.y;
        pt.z = target.z;
        global_obstacles_->push_back(pt);

        plan_count++;
      }
    }

    // 保存当前成功的点位到 last_success_points_，供下一轮使用
    // global_obstacles_ 在计算过程中被添加了新的点位，需要提取出来
    size_t initial_obstacle_count = vision_holes->size() + last_success_points_->size();
    last_success_points_->clear();
    for (size_t i = initial_obstacle_count; i < global_obstacles_->size(); ++i) {
        last_success_points_->push_back(global_obstacles_->points[i]);
    }
    RCLCPP_INFO(this->get_logger(), "[DEBUG] Saved %zu current success points for next round", last_success_points_->size());

    // 9. 检查点位数量，动态调整权重和触发宽松模式
    int min_points_threshold = (int)(grids_.size() * 0.5);
    int critical_threshold = 8;  // 关键阈值：8个点

    if (plan_count < min_points_threshold) {
      // 动态调整权重因子
      float weight_factor = 1.0f;
      if (plan_count < critical_threshold) {
        // 点数少于8，大幅降低权重要求
        weight_factor = 0.5f;
        RCLCPP_ERROR(this->get_logger(),
                     "CRITICAL: Only %d points found (< %d), reducing weights by 50%%",
                     plan_count, critical_threshold);
      } else {
        // 点数在8-12之间，适度降低权重
        weight_factor = 0.7f;
        RCLCPP_WARN(this->get_logger(),
                    "Strict mode only found %d points (< %d), reducing weights by 30%%",
                    plan_count, min_points_threshold);
      }

      // 临时调整参数
      float original_angle_weight = param_.ANGLE_WEIGHT;
      float original_center_weight = param_.CENTER_WEIGHT;
      param_.ANGLE_WEIGHT *= weight_factor;
      param_.CENTER_WEIGHT *= weight_factor;

      RCLCPP_WARN(this->get_logger(),
                  "Strict mode only found %d points (< %d), triggering relaxed mode for skipped grids...",
                  plan_count, min_points_threshold);

      int relaxed_count = 0;
      for (auto &grid : grids_) {
        if (grid->getState() == chisel_box::STATE_SKIPPED_ONCE) {
          // 强制进入宽松模式
          pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr roi_cloud(
              new pcl::PointCloud<pcl::PointXYZRGBNormal>);
          extractGridROI(processed_cloud, grid, roi_cloud);

          pcl::PointXYZRGBNormal target;
          if (grid->findBestPoint(roi_cloud, global_obstacles_, target)) {
            // 1. 存入结果 (Tool Frame)
            geometry_msgs::msg::Pose tool_pose;
            transformPose(target, tool_pose);
            tool_pose.orientation.w = target.curvature;
            res->pose_list.poses.push_back(tool_pose);

            // 2. 存入可视化 (Camera Frame)
            geometry_msgs::msg::Pose cam_pose;
            cam_pose.position.x = target.x;
            cam_pose.position.y = target.y;
            cam_pose.position.z = target.z;
            cam_pose.orientation.x = target.normal_x;
            cam_pose.orientation.y = target.normal_y;
            cam_pose.orientation.z = target.normal_z;
            cam_pose.orientation.w = 0;
            visual_poses.poses.push_back(cam_pose);

            // 3. 立即将当前成功的点位添加到 global_obstacles_，用于后续网格避让
            pcl::PointXYZ pt;
            pt.x = target.x;
            pt.y = target.y;
            pt.z = target.z;
            global_obstacles_->push_back(pt);

            plan_count++;
            relaxed_count++;
          }
        }
      }

      // 恢复原始权重
      param_.ANGLE_WEIGHT = original_angle_weight;
      param_.CENTER_WEIGHT = original_center_weight;

      RCLCPP_INFO(this->get_logger(),
                  "Relaxed mode found %d additional points, total: %d",
                  relaxed_count, plan_count);
    }

    res->pose_list.header.stamp = this->get_clock()->now();
    res->pose_list.header.frame_id = info_snap.header.frame_id;

    // 【恢复发布】发布可视化结果
    pub_visual_norm_->publish(visual_poses);

    RCLCPP_INFO(this->get_logger(), "<<< Analysis Done. Planned %d points.",
                plan_count);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
  auto node = std::make_shared<NormCalcServer>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                    4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}