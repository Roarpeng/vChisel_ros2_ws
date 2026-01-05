#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
报警管理器
Alarm Manager

功能：
1. 收集和管理系统报警
2. 报警分级和过滤
3. 报警历史记录
4. PLC报警联动
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from datetime import datetime
from collections import deque


class AlarmManager(Node):
    def __init__(self):
        super().__init__('alarm_manager')

        # 参数
        self.declare_parameter('max_history', 100)
        self.max_history = self.get_parameter('max_history').get_parameter_value().integer_value

        # 报警历史
        self.alarm_history = deque(maxlen=self.max_history)
        self.active_alarms = []

        # 订阅报警话题
        self.alarm_sub = self.create_subscription(
            String,
            'vchisel/alarms',
            self.alarm_callback,
            10
        )

        # 发布报警统计
        self.stats_pub = self.create_publisher(
            String,
            'vchisel/alarm_stats',
            10
        )

        # 服务：获取报警历史
        self.history_srv = self.create_service(
            Trigger,
            'alarm/history',
            self.handle_get_history
        )

        # 服务：获取活跃报警
        self.active_srv = self.create_service(
            Trigger,
            'alarm/active',
            self.handle_get_active
        )

        # 服务：确认报警
        self.ack_srv = self.create_service(
            Trigger,
            'alarm/acknowledge',
            self.handle_acknowledge
        )

        # 服务：清除所有报警
        self.clear_srv = self.create_service(
            Trigger,
            'alarm/clear_all',
            self.handle_clear_all
        )

        # 定时器：发布统计信息
        self.stats_timer = self.create_timer(5.0, self.publish_stats)

        self.get_logger().info('Alarm manager started')

    def alarm_callback(self, msg):
        """处理报警消息"""
        try:
            alarm = eval(msg.data)  # 简化处理，实际应使用JSON
            alarm['received_at'] = datetime.now().isoformat()

            # 添加到历史
            self.alarm_history.append(alarm)

            # 如果是活跃报警（未确认且级别>=WARNING）
            if not alarm.get('acknowledged', False) and alarm.get('level', 0) >= 1:
                self.active_alarms.append(alarm)

            self.get_logger().info(f"Alarm received: [{alarm.get('code', 'UNKNOWN')}] {alarm.get('message', '')}")

        except Exception as e:
            self.get_logger().error(f'Failed to process alarm: {e}')

    def publish_stats(self):
        """发布报警统计"""
        stats = {
            'total_alarms': len(self.alarm_history),
            'active_alarms': len(self.active_alarms),
            'critical_count': sum(1 for a in self.alarm_history if a.get('level', 0) == 3),
            'error_count': sum(1 for a in self.alarm_history if a.get('level', 0) == 2),
            'warning_count': sum(1 for a in self.alarm_history if a.get('level', 0) == 1),
            'info_count': sum(1 for a in self.alarm_history if a.get('level', 0) == 0),
            'timestamp': datetime.now().isoformat()
        }

        msg = String()
        msg.data = str(stats)
        self.stats_pub.publish(msg)

    def handle_get_history(self, request, response):
        """处理获取历史请求"""
        response.success = True
        response.message = str(list(self.alarm_history))
        return response

    def handle_get_active(self, request, response):
        """处理获取活跃报警请求"""
        response.success = True
        response.message = str(self.active_alarms)
        return response

    def handle_acknowledge(self, request, response):
        """处理确认报警请求"""
        # 确认所有活跃报警
        for alarm in self.active_alarms:
            alarm['acknowledged'] = True
            alarm['acknowledged_at'] = datetime.now().isoformat()

        acknowledged_count = len(self.active_alarms)
        self.active_alarms = []

        response.success = True
        response.message = f'Acknowledged {acknowledged_count} alarms'
        return response

    def handle_clear_all(self, request, response):
        """处理清除所有报警请求"""
        self.active_alarms = []
        self.alarm_history.clear()

        response.success = True
        response.message = 'All alarms cleared'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = AlarmManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
