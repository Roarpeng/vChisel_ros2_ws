#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/pose_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <mutex>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace std::placeholders;

class ImageNormViewerNode : public rclcpp::Node {
public:
  ImageNormViewerNode() : Node("image_norm_viewer") {
    RCLCPP_INFO(this->get_logger(),
                "Starting ImageNormViewer ROS2 node (Result Only)...");

    // Subscription to captured image (the snapshot used for calc)
    captured_img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "captured_image", 10,
        std::bind(&ImageNormViewerNode::capturedImageCallback, this, _1));

    // Subscription to visual norm results (camera frame points)
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "visual_norm_result", 10,
        std::bind(&ImageNormViewerNode::poseCallback, this, _1));

    // Subscription to camera info for projection (using depth info as
    // calculation reference)
    cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera/camera/depth/camera_info", 10,
        std::bind(&ImageNormViewerNode::cameraInfoCallback, this, _1));

    window_name_ = "Norm Calculation Result";
    cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
    cv::startWindowThread();

    // Grid Parameters
    this->declare_parameter("BOX_LEN", 0.05);
    this->declare_parameter("BOX_ROW", 4);
    this->declare_parameter("BOX_COLUMN", 6);
    this->declare_parameter("XMIN", -0.15);
    this->declare_parameter("YMIN", -0.1);
    this->declare_parameter("ZMIN", 0.2);
    this->declare_parameter("ZMAX", 0.6);

    box_len_ = this->get_parameter("BOX_LEN").as_double();
    box_row_ = this->get_parameter("BOX_ROW").as_int();
    box_column_ = this->get_parameter("BOX_COLUMN").as_int();
    xmin_ = this->get_parameter("XMIN").as_double();
    ymin_ = this->get_parameter("YMIN").as_double();
    zmin_ = this->get_parameter("ZMIN").as_double();
    zmax_ = this->get_parameter("ZMAX").as_double();

    RCLCPP_INFO(this->get_logger(),
                "Result-only Viewer initialized with grid: %dx%d (cell: %.3fm)",
                box_row_, box_column_, box_len_);
  }

  ~ImageNormViewerNode() { cv::destroyAllWindows(); }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr captured_img_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_sub_;

  cv::Mat current_result_image_;
  std::vector<geometry_msgs::msg::Pose> latest_poses_;
  sensor_msgs::msg::CameraInfo current_camera_info_;
  std::mutex data_mutex_;
  bool camera_info_received_ = false;
  std::string window_name_;

  // Grid config
  double box_len_;
  int box_row_;
  int box_column_;
  double xmin_, ymin_;
  double zmin_, zmax_;

  void
  capturedImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    try {
      cv_bridge::CvImagePtr cv_ptr =
          cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      std::lock_guard<std::mutex> lock(data_mutex_);
      current_result_image_ = cv_ptr->image.clone();
      RCLCPP_INFO(this->get_logger(), "New result snapshot received: %dx%d",
                  current_result_image_.cols, current_result_image_.rows);
      updateDisplay();
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  void
  cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_camera_info_ = *msg;
    if (!camera_info_received_) {
      RCLCPP_INFO(this->get_logger(), "Camera info received: %dx%d", msg->width,
                  msg->height);
    }
    camera_info_received_ = true;
  }

  void poseCallback(const geometry_msgs::msg::PoseArray::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_poses_ = msg->poses;
    RCLCPP_INFO(this->get_logger(), "Received %zu visual points for overlay.",
                latest_poses_.size());
    updateDisplay();
  }

  cv::Point project3DTo2D(double x, double y, double z, int img_cols,
                          int img_rows) {
    if (!camera_info_received_)
      return cv::Point(-1, -1);

    double fx = current_camera_info_.k[0];
    double fy = current_camera_info_.k[4];
    double cx = current_camera_info_.k[2];
    double cy = current_camera_info_.k[5];

    // Auto-scale intrinsics if resolution mismatch (e.g. 1280x720 image vs
    // 848x480 info)
    if (current_camera_info_.width != 0 && current_camera_info_.height != 0) {
      double scale_x =
          static_cast<double>(img_cols) / current_camera_info_.width;
      double scale_y =
          static_cast<double>(img_rows) / current_camera_info_.height;
      fx *= scale_x;
      fy *= scale_y;
      cx *= scale_x;
      cy *= scale_y;
    }

    if (z < 0.001)
      return cv::Point(-1, -1);
    return cv::Point(static_cast<int>((x * fx / z) + cx),
                     static_cast<int>((y * fy / z) + cy));
  }

  void updateDisplay() {
    if (current_result_image_.empty()) {
      return;
    }

    cv::Mat display_mat = current_result_image_.clone();
    int cols = display_mat.cols;
    int rows = display_mat.rows;
    std::vector<cv::Point> all_visual_points;

    // Draw Grid first
    if (camera_info_received_) {
      drawGrid(display_mat, cols, rows, all_visual_points);
    }

    if (latest_poses_.empty()) {
      RCLCPP_WARN(this->get_logger(),
                  "updateDisplay called but latest_poses_ is empty.");
    }

    for (size_t i = 0; i < latest_poses_.size(); ++i) {
      const auto &p = latest_poses_[i];

      // Project 3D center
      cv::Point center =
          project3DTo2D(p.position.x, p.position.y, p.position.z, cols, rows);

      if (center.x >= 0 && center.x < cols && center.y >= 0 &&
          center.y < rows) {
        all_visual_points.push_back(center);

        // Draw normal point (larger circle)
        cv::circle(display_mat, center, 8, cv::Scalar(0, 255, 0), -1);

        // Project 3D vector end point for proper perspective (e.g. 5cm length)
        double vector_len = 0.05;
        cv::Point end = project3DTo2D(
            p.position.x + p.orientation.x * vector_len,
            p.position.y + p.orientation.y * vector_len,
            p.position.z + p.orientation.z * vector_len, cols, rows);

        if (end.x >= 0 && end.x < cols && end.y >= 0 && end.y < rows) {
          all_visual_points.push_back(end);
          cv::arrowedLine(display_mat, center, end, cv::Scalar(0, 0, 255), 3, 8,
                          0, 0.3);
        } else {
          // 2D fallback if 3D projection of vector end is outside
          cv::line(display_mat, center,
                   cv::Point(center.x + p.orientation.x * 30,
                             center.y + p.orientation.y * 30),
                   cv::Scalar(0, 0, 255), 3);
        }

        // Draw label
        cv::putText(display_mat, std::to_string(i),
                    cv::Point(center.x + 10, center.y - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 0), 2);
      } else {
        if (i < 5) {
          RCLCPP_WARN(this->get_logger(),
                      "Point %zu out of bounds: 3D(%f, %f, %f) -> 2D(%d, %d)",
                      i, p.position.x, p.position.y, p.position.z, center.x,
                      center.y);
        }
      }
    }

    if (!all_visual_points.empty()) {
      cv::Rect roi = cv::boundingRect(all_visual_points);
      int padding = 150;
      roi.x = std::max(0, roi.x - padding);
      roi.y = std::max(0, roi.y - padding);
      roi.width = std::min(cols - roi.x, roi.width + 2 * padding);
      roi.height = std::min(rows - roi.y, roi.height + 2 * padding);

      if (roi.width > 20 && roi.height > 20) {
        display_mat = display_mat(roi).clone();
      }
    } else if (!latest_poses_.empty()) {
      RCLCPP_ERROR(this->get_logger(),
                   "No points could be projected onto the image!");
    }

    cv::putText(display_mat, "Norm Result Area (ROI)", cv::Point(20, 40),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);

    cv::imshow(window_name_, display_mat);
    cv::waitKey(1);
  }

  void drawGrid(cv::Mat &img, int cols, int rows,
                std::vector<cv::Point> &points_for_roi) {
    cv::Scalar grid_color(0, 255, 255); // Yellow
    int grid_thickness = 1;

    // Estimate a representative Z for the grid (using average or min Z)
    double draw_z = zmin_ + 0.1; // Offset slightly from min Z for visibility
    if (!latest_poses_.empty()) {
      draw_z = 0;
      int count = 0;
      for (const auto &p : latest_poses_) {
        draw_z += p.position.z;
        count++;
      }
      draw_z /= count;
    }

    // Draw Vertical Lines (X constant)
    for (int j = 0; j <= box_column_; ++j) {
      double x = xmin_ + j * box_len_;
      cv::Point p1 = project3DTo2D(x, ymin_, draw_z, cols, rows);
      cv::Point p2 =
          project3DTo2D(x, ymin_ + box_row_ * box_len_, draw_z, cols, rows);

      if (p1.x != -1 && p2.x != -1) {
        cv::line(img, p1, p2, grid_color, grid_thickness);
        points_for_roi.push_back(p1);
        points_for_roi.push_back(p2);
      }
    }

    // Draw Horizontal Lines (Y constant)
    for (int i = 0; i <= box_row_; ++i) {
      double y = ymin_ + i * box_len_;
      cv::Point p1 = project3DTo2D(xmin_, y, draw_z, cols, rows);
      cv::Point p2 =
          project3DTo2D(xmin_ + box_column_ * box_len_, y, draw_z, cols, rows);

      if (p1.x != -1 && p2.x != -1) {
        cv::line(img, p1, p2, grid_color, grid_thickness);
        points_for_roi.push_back(p1);
        points_for_roi.push_back(p2);
      }
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageNormViewerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}