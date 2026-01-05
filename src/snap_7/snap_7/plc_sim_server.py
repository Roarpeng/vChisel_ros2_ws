#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PLC Simulator Server
PLC仿真服务器 - 用于无真实PLC时的测试

功能：
1. 模拟PLC数据块读写
2. 模拟机器人位姿数据
3. 模拟标定状态机
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
import struct
import random
import math


class PLCSimServer(Node):
    def __init__(self):
        super().__init__('plc_simulator')

        # 参数
        self.declare_parameter('server_ip', '127.0.0.1')
        self.declare_parameter('server_port', 102)

        # 模拟数据块 - 1024字节
        self.db_data = bytearray(1024)

        # 初始化模拟数据
        self._init_simulated_data()

        # 发布模拟状态
        self.status_pub = self.create_publisher(
            String,
            'plc_sim/status',
            10
        )

        # 服务：获取模拟状态
        self.status_srv = self.create_service(
            Trigger,
            'plc_sim/get_status',
            self.handle_get_status
        )

        # 服务：重置数据
        self.reset_srv = self.create_service(
            Trigger,
            'plc_sim/reset',
            self.handle_reset
        )

        # 定时器：更新模拟数据
        self.update_timer = self.create_timer(0.1, self.update_simulation)

        # 定时器：发布状态
        self.status_timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info(f'PLC Simulator started')
        self.get_logger().info(f'  Server IP: {self.get_parameter("server_ip").value}')
        self.get_logger().info(f'  Server Port: {self.get_parameter("server_port").value}')

    def _init_simulated_data(self):
        """初始化模拟数据"""
        # 设置一些默认值

        # 偏移64-65: plcStatus = 1 (ready)
        struct.pack_into('>h', self.db_data, 64, 1)

        # 偏移66-67: rosStatus = 0
        struct.pack_into('>h', self.db_data, 66, 0)

        # 偏移600-623: 机器人位姿 (6个REAL值)
        # X=100, Y=200, Z=300, A=0, B=0, C=0
        struct.pack_into('>f', self.db_data, 600, 100.0)  # X
        struct.pack_into('>f', self.db_data, 604, 200.0)  # Y
        struct.pack_into('>f', self.db_data, 608, 300.0)  # Z
        struct.pack_into('>f', self.db_data, 612, 0.0)    # A
        struct.pack_into('>f', self.db_data, 616, 0.0)    # B
        struct.pack_into('>f', self.db_data, 620, 0.0)    # C

        # 偏移624-625: plcCalibStatus = 0
        struct.pack_into('>h', self.db_data, 624, 0)

        # 偏移626-627: rosCalibStatus = 0
        struct.pack_into('>h', self.db_data, 626, 0)

        self.sim_time = 0.0

    def update_simulation(self):
        """更新模拟数据 - 每100ms调用"""
        self.sim_time += 0.1

        # 模拟机器人小幅运动
        # X轴缓慢正弦波动
        x = 100.0 + 5.0 * math.sin(self.sim_time * 0.5)
        y = 200.0 + 5.0 * math.cos(self.sim_time * 0.5)
        z = 300.0 + 2.0 * math.sin(self.sim_time * 0.3)

        struct.pack_into('>f', self.db_data, 600, x)
        struct.pack_into('>f', self.db_data, 604, y)
        struct.pack_into('>f', self.db_data, 608, z)

    def publish_status(self):
        """发布模拟器状态"""
        # 读取当前模拟数据
        plc_status = struct.unpack_from('>h', self.db_data, 64)[0]
        x = struct.unpack_from('>f', self.db_data, 600)[0]
        y = struct.unpack_from('>f', self.db_data, 604)[0]
        z = struct.unpack_from('>f', self.db_data, 608)[0]

        status = {
            'simulator': True,
            'plc_status': plc_status,
            'robot_pose': {'x': round(x, 2), 'y': round(y, 2), 'z': round(z, 2)},
            'sim_time': round(self.sim_time, 1)
        }

        msg = String()
        msg.data = str(status)
        self.status_pub.publish(msg)

    def handle_get_status(self, request, response):
        """处理获取状态请求"""
        response.success = True
        response.message = f'PLC Simulator running. Sim time: {self.sim_time:.1f}s'
        return response

    def handle_reset(self, request, response):
        """处理重置请求"""
        self._init_simulated_data()
        response.success = True
        response.message = 'Simulation data reset'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PLCSimServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
