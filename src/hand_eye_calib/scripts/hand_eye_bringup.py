#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rclpy
from rclpy.node import Node
from hand_eye_calib.srv import HandEyeCalibData
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
import time
import copy

class HandEyeBringup(Node):
    def __init__(self):
        super().__init__('hand_eye_bringup')
        self.real_camera_pose = None
        self.subscription = self.create_subscription(
            PoseStamped,
            '/aruco_single/pose',
            self.camera_callback,
            10)
        self.client = self.create_client(HandEyeCalibData, 'hand_eye_calib')
        
    def camera_callback(self, msg):
        self.real_camera_pose = msg.pose

    def start_calib(self, camPoses, toolPoses):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        req = HandEyeCalibData.Request()
        req.eye_data_list = camPoses
        req.hand_data_list = toolPoses
        
        self.get_logger().info("hand eye calib start")
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        try:
            response = future.result()
            self.get_logger().info("hand eye calib End")
            print(response)
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = HandEyeBringup()

    CalPose = PoseArray()
    ToolPose = PoseArray()
    temp_hand_pose = Pose()

    # Wait for first camera pose
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.real_camera_pose is not None:
            break
        node.get_logger().info('等待相机姿态话题数据到位 ...')
        time.sleep(1)

    dataCount = 0
    CamSuccess = False
    ToolSuccess = True

    while rclpy.ok():
        if ToolSuccess:
            print(f"当前完成标定数据组数：{dataCount}")
            command = input("指令: r 记录一组相机数据, c 计算并保存, q 退出: ").strip()
            
            if command == "r":
                CalPose.poses.append(node.real_camera_pose)
                print(node.real_camera_pose)
                CamSuccess = True
            elif command == 'c':
                node.start_calib(CalPose, ToolPose)
                break
            elif command == 'q':
                break
            else:
                node.get_logger().info("输入错误！")
                CamSuccess = False

        if CamSuccess:
            getToolData = input("请输入当前工具坐标XYZABC,用逗号分割。q 退出: ").strip()
            if getToolData == 'q':
                break
            else:
                try:
                    toolData = getToolData.split(",")
                    if len(toolData) == 6:
                        temp_hand_pose.position.x = float(toolData[0])
                        temp_hand_pose.position.y = float(toolData[1])
                        temp_hand_pose.position.z = float(toolData[2])
                        temp_hand_pose.orientation.x = float(toolData[3])
                        temp_hand_pose.orientation.y = float(toolData[4])
                        temp_hand_pose.orientation.z = float(toolData[5])
                        print(temp_hand_pose)
                        ToolPose.poses.append(copy.deepcopy(temp_hand_pose))
                        ToolSuccess = True
                        dataCount += 1
                        CamSuccess = False # Reset for next round
                    else:
                        node.get_logger().info(f"输入格式错误！输入长度：{len(toolData)}")
                        ToolSuccess = False
                except Exception as e:
                    node.get_logger().info(f"输入格式错误！{e}")
                    ToolSuccess = False

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()