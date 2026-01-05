#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
系统状态桥接节点
System Status Bridge Node

功能：
1. 聚合所有子系统状态
2. 发布统一的系统状态话题
3. 提供WebSocket接口（通过rosbridge）
4. 处理报警和故障恢复
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from datetime import datetime


class BridgeNode(Node):
    def __init__(self):
        super().__init__('bridge_node')

        # 系统状态
        self.plc_connected = False
        self.camera_connected = False
        self.current_recipe = 'default'
        self.operation_state = 'IDLE'
        self.points_processed = 0
        self.cycles_completed = 0
        self.calib_mode_active = False
        self.calib_points_captured = 0
        self.calib_quality_score = 0
        self.last_error = ''
        self.last_error_time = ''

        # 发布系统状态
        self.status_pub = self.create_publisher(
            String,
            'vchisel/system_status',
            10
        )

        # 发布报警
        self.alarm_pub = self.create_publisher(
            String,
            'vchisel/alarms',
            10
        )

        # 服务：获取系统状态
        self.status_srv = self.create_service(
            Trigger,
            'vchisel/get_status',
            self.handle_get_status
        )

        # 服务：清除报警
        self.clear_alarm_srv = self.create_service(
            Trigger,
            'vchisel/clear_alarms',
            self.handle_clear_alarms
        )

        # 定时器：定期发布状态
        self.status_timer = self.create_timer(1.0, self.publish_status)

        # 定时器：检查子系统状态
        self.check_timer = self.create_timer(2.0, self.check_subsystems)

        self.get_logger().info('Bridge node started')

    def publish_status(self):
        """发布系统状态"""
        status = {
            'plc_connected': self.plc_connected,
            'camera_connected': self.camera_connected,
            'current_recipe': self.current_recipe,
            'operation_state': self.operation_state,
            'points_processed': self.points_processed,
            'cycles_completed': self.cycles_completed,
            'calib_mode_active': self.calib_mode_active,
            'calib_points_captured': self.calib_points_captured,
            'calib_quality_score': self.calib_quality_score,
            'last_error': self.last_error,
            'last_error_time': self.last_error_time,
            'timestamp': datetime.now().isoformat()
        }

        msg = String()
        msg.data = str(status)
        self.status_pub.publish(msg)

    def check_subsystems(self):
        """检查子系统状态"""
        # TODO: 实际检查各个子系统的连接状态
        # 这里应该订阅各个节点的心跳或状态话题
        pass

    def publish_alarm(self, level: int, code: str, message: str, source: str):
        """发布报警"""
        alarm = {
            'level': level,
            'code': code,
            'message': message,
            'source': source,
            'timestamp': datetime.now().isoformat(),
            'acknowledged': False
        }

        msg = String()
        msg.data = str(alarm)
        self.alarm_pub.publish(msg)

        # 记录最后错误
        if level >= 2:  # ERROR or CRITICAL
            self.last_error = message
            self.last_error_time = datetime.now().isoformat()

    def handle_get_status(self, request, response):
        """处理获取状态请求"""
        status = {
            'plc_connected': self.plc_connected,
            'camera_connected': self.camera_connected,
            'current_recipe': self.current_recipe,
            'operation_state': self.operation_state,
            'points_processed': self.points_processed,
            'cycles_completed': self.cycles_completed,
            'calib_mode_active': self.calib_mode_active,
            'calib_points_captured': self.calib_points_captured,
            'calib_quality_score': self.calib_quality_score,
            'last_error': self.last_error,
            'last_error_time': self.last_error_time
        }

        response.success = True
        response.message = str(status)
        return response

    def handle_clear_alarms(self, request, response):
        """处理清除报警请求"""
        self.last_error = ''
        self.last_error_time = ''
        response.success = True
        response.message = 'Alarms cleared'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
