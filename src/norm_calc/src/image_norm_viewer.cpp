#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std::placeholders;

class ImageNormViewerNode : public rclcpp::Node
{
public:
    ImageNormViewerNode() : Node("image_norm_viewer")
    {
        RCLCPP_INFO(this->get_logger(), "Starting ImageNormViewer ROS2 node...");

        // Subscribe to image, camera info, and norm calculation result topics
        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/camera/color/image_raw", 10,
            std::bind(&ImageNormViewerNode::imageCallback, this, std::placeholders::_1));
        
        cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "camera/camera/depth/camera_info", 10,
            std::bind(&ImageNormViewerNode::cameraInfoCallback, this, std::placeholders::_1));
        
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "norm_calc_result", 10,
            std::bind(&ImageNormViewerNode::poseCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "ImageNormViewer node initialized.");
        
        // Store the window name to use in destructor
        window_name_ = "Image with Norm Calculation Results";
        
        // Initialize OpenCV window
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
        cv::startWindowThread(); // Important for keeping the window responsive
    }

    ~ImageNormViewerNode()
    {
        cv::destroyAllWindows();
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_sub_;
    
    cv::Mat current_image_;
    std::vector<geometry_msgs::msg::Pose> latest_poses_;
    sensor_msgs::msg::CameraInfo current_camera_info_;
    std::mutex data_mutex_;
    bool image_updated_ = false;
    bool camera_info_received_ = false;
    std::string window_name_;

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        try 
        {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
            std::lock_guard<std::mutex> lock(data_mutex_);
            current_image_ = cv_ptr->image.clone();
            image_updated_ = true;
        } 
        catch (cv_bridge::Exception& e) 
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        current_camera_info_ = *msg;
        camera_info_received_ = true;
    }

    void poseCallback(const geometry_msgs::msg::PoseArray::ConstSharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_poses_ = msg->poses;
        
        RCLCPP_INFO(this->get_logger(), "Received %zu norm calculation results", latest_poses_.size());
    }

    // Function to project 3D point to 2D image coordinates
    cv::Point project3DTo2D(double x, double y, double z) 
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!camera_info_received_ || current_image_.empty()) {
            // Return center of image as default if camera info not available
            return cv::Point(320, 240); // Default for 640x480 image
        }

        // Use camera intrinsic parameters to project 3D point to 2D
        double fx = current_camera_info_.k[0]; // focal length x
        double fy = current_camera_info_.k[4]; // focal length y
        double cx = current_camera_info_.k[2]; // principal point x
        double cy = current_camera_info_.k[5]; // principal point y

        if (z <= 0.001) z = 0.001; // Avoid division by zero or negative depth

        int u = static_cast<int>((x * fx / z) + cx);
        int v = static_cast<int>((y * fy / z) + cy);

        return cv::Point(u, v);
    }

    void displayImageWithNorms()
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        if (current_image_.empty()) {
            // If window was previously created, destroy it
            if (window_created_) {
                cv::destroyWindow(window_name_);
                window_created_ = false;
            }
            return; // Don't display anything if no image is received
        }

        // Create window if not already created
        if (!window_created_) {
            cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
            cv::startWindowThread();
            window_created_ = true;
        }

        cv::Mat display_image = current_image_.clone();
        
        // Draw norm calculation results on the image only if we have poses
        if (!latest_poses_.empty()) {
            for (size_t i = 0; i < latest_poses_.size(); ++i) 
            {
                const auto& pose = latest_poses_[i];
                
                // Project 3D position to 2D image coordinates
                cv::Point center = project3DTo2D(pose.position.x, pose.position.y, pose.position.z);
                
                // Check if the point is within the image bounds
                if (center.x >= 0 && center.x < display_image.cols && 
                    center.y >= 0 && center.y < display_image.rows) 
                {
                    // Draw position as a circle
                    cv::circle(display_image, center, 8, cv::Scalar(0, 255, 0), 2);
                    
                    // Draw normal direction as an arrow
                    // Calculate end point of the normal vector (scale factor for visibility)
                    double normal_scale = 30.0; 
                    cv::Point end_point(
                        center.x + static_cast<int>(pose.orientation.x * normal_scale),
                        center.y + static_cast<int>(pose.orientation.y * normal_scale)
                    );
                    
                    // Ensure end point is within image bounds
                    end_point.x = std::max(0, std::min(display_image.cols - 1, end_point.x));
                    end_point.y = std::max(0, std::min(display_image.rows - 1, end_point.y));
                    
                    // Draw the normal vector as an arrow
                    cv::arrowedLine(display_image, center, end_point, cv::Scalar(0, 0, 255), 2, 8, 0, 0.1);
                    
                    // Add text label
                    std::string label = "P" + std::to_string(i);
                    cv::putText(display_image, label, 
                               cv::Point(center.x + 10, center.y), 
                               cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
                }
            }
            
            // Add title indicating both image and norm results are shown
            cv::putText(display_image, "Image + Norm Results", 
                       cv::Point(10, 30), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
        } else {
            // Only show the original image without norm results
            cv::putText(display_image, "Live Camera Image", 
                       cv::Point(10, 30), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
        }
        
        // Display the image
        cv::imshow(window_name_, display_image);
        cv::waitKey(1);
    }

private:
    bool window_created_ = false; // Track if window has been created
    
public:
    void run()
    {
        rclcpp::Rate rate(30); // 30 Hz display rate
        
        while (rclcpp::ok()) 
        {
            // Handle incoming messages
            rclcpp::spin_some(shared_from_this());
            
            // Update the display
            displayImageWithNorms();
            
            rate.sleep();
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageNormViewerNode>();
    
    // Run the display loop in the main thread and handle ROS callbacks periodically
    node->run();
    
    rclcpp::shutdown();
    return 0;
}