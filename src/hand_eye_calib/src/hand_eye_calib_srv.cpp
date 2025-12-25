#include <rclcpp/rclcpp.hpp>
#include "hand_eye_calib/hand_eye_calib.h"
#include "hand_eye_calib/srv/hand_eye_calib_data.hpp"
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

cv::Mat CalPose; //相机中组标定板的位姿，x(m),y,z,X,Y,Z,W
cv::Mat ToolPose;  //机械臂末端的位姿，x(mm),y,z,a(rz),b(ry),c(rx) Tool5 动轴（内旋）欧拉角
cv::Mat Homo_cam2gripper;//定义相机camera到末端grab的位姿矩阵

geometry_msgs::msg::Quaternion tempRes;

void getTheCalibData(const std::shared_ptr<hand_eye_calib::srv::HandEyeCalibData::Request> req,
                     std::shared_ptr<hand_eye_calib::srv::HandEyeCalibData::Response> res)
{
    RCLCPP_INFO(rclcpp::get_logger("hand_eye_calib_server"), "getTheCalibData");
    int num = 0;

    num = req->eye_data_list.poses.size();

    RCLCPP_INFO(rclcpp::get_logger("hand_eye_calib_server"), "input list size: %d", num);

    if (num < 5
        || num > 30
        || num != (int)req->hand_data_list.poses.size())
    {
        RCLCPP_ERROR(rclcpp::get_logger("hand_eye_calib_server"), "Error input data num! Must >=5 && <= 30");
        return;
    }

    CalPose = (cv::Mat_<double>(num, 7));
    ToolPose = (cv::Mat_<double>(num, 6));

    for (int i = 0; i < num; i++)
    {
        CalPose.at<double>(i,0) = req->eye_data_list.poses[i].position.x;
        CalPose.at<double>(i,1) = req->eye_data_list.poses[i].position.y;
        CalPose.at<double>(i,2) = req->eye_data_list.poses[i].position.z;
        CalPose.at<double>(i,3) = req->eye_data_list.poses[i].orientation.x;
        CalPose.at<double>(i,4) = req->eye_data_list.poses[i].orientation.y;
        CalPose.at<double>(i,5) = req->eye_data_list.poses[i].orientation.z;
        CalPose.at<double>(i,6) = req->eye_data_list.poses[i].orientation.w;

        ToolPose.at<double>(i,0) = req->hand_data_list.poses[i].position.x;
        ToolPose.at<double>(i,1) = req->hand_data_list.poses[i].position.y;
        ToolPose.at<double>(i,2) = req->hand_data_list.poses[i].position.z;
        ToolPose.at<double>(i,3) = req->hand_data_list.poses[i].orientation.x;
        ToolPose.at<double>(i,4) = req->hand_data_list.poses[i].orientation.y;
        ToolPose.at<double>(i,5) = req->hand_data_list.poses[i].orientation.z;
    }

    if (hand_eye_calib_calc(num, CalPose, ToolPose, Homo_cam2gripper))
    {
        for(int i = 0; i < 4; i++)
        {
            tempRes.x = Homo_cam2gripper.at<double>(i,0);
            tempRes.y = Homo_cam2gripper.at<double>(i,1);
            tempRes.z = Homo_cam2gripper.at<double>(i,2);
            tempRes.w = Homo_cam2gripper.at<double>(i,3);
            res->rt_list.push_back(tempRes);
        }
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("hand_eye_calib_server"), "Calibration failed!");
    }
}

int main(int argc, char** argv)
{   
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("hand_eye_calib_server");

    RCLCPP_INFO(node->get_logger(), "Ready to calibration.");
    auto service = node->create_service<hand_eye_calib::srv::HandEyeCalibData>("hand_eye_calib", getTheCalibData);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}