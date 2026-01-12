#!/usr/bin/env python3
"""
Foxglove参数动态配置辅助脚本
用于在运行时动态调整norm_calc节点的参数
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters, GetParameters, ListParameters
import json
import sys


class FoxgloveParamHelper(Node):
    """参数配置辅助节点"""
    
    def __init__(self):
        super().__init__('foxglove_param_helper')
        
        # 创建服务客户端
        self.set_params_client = self.create_client(
            SetParameters, '/norm_calc/set_parameters'
        )
        self.get_params_client = self.create_client(
            GetParameters, '/norm_calc/get_parameters'
        )
        self.list_params_client = self.create_client(
            ListParameters, '/norm_calc/list_parameters'
        )
        
        self.get_logger().info('Foxglove参数辅助节点已启动')
    
    def list_all_parameters(self):
        """列出所有可配置的参数"""
        if not self.list_params_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('list_parameters服务不可用')
            return []
        
        request = ListParameters.Request()
        future = self.list_params_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            return future.result().result.names
        else:
            self.get_logger().error('获取参数列表失败')
            return []
    
    def get_parameter(self, param_name):
        """获取参数值"""
        if not self.get_params_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('get_parameters服务不可用')
            return None
        
        request = GetParameters.Request()
        request.names = [param_name]
        future = self.get_params_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None and len(future.result().values) > 0:
            return future.result().values[0]
        else:
            self.get_logger().error(f'获取参数 {param_name} 失败')
            return None
    
    def set_parameter(self, param_name, param_value):
        """设置参数值"""
        if not self.set_params_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('set_parameters服务不可用')
            return False
        
        # 根据值类型创建参数
        if isinstance(param_value, bool):
            param = Parameter()
            param.name = param_name
            param.value.type = ParameterType.PARAMETER_BOOL
            param.value.bool_value = param_value
        elif isinstance(param_value, int):
            param = Parameter()
            param.name = param_name
            param.value.type = ParameterType.PARAMETER_INTEGER
            param.value.integer_value = param_value
        elif isinstance(param_value, float):
            param = Parameter()
            param.name = param_name
            param.value.type = ParameterType.PARAMETER_DOUBLE
            param.value.double_value = param_value
        elif isinstance(param_value, str):
            param = Parameter()
            param.name = param_name
            param.value.type = ParameterType.PARAMETER_STRING
            param.value.string_value = param_value
        elif isinstance(param_value, list):
            # 处理数组类型
            if all(isinstance(x, float) for x in param_value):
                param = Parameter()
                param.name = param_name
                param.value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
                param.value.double_array_value = param_value
            elif all(isinstance(x, int) for x in param_value):
                param = Parameter()
                param.name = param_name
                param.value.type = ParameterType.PARAMETER_INTEGER_ARRAY
                param.value.integer_array_value = param_value
            else:
                self.get_logger().error(f'不支持的数组类型: {type(param_value)}')
                return False
        else:
            self.get_logger().error(f'不支持的参数类型: {type(param_value)}')
            return False
        
        request = SetParameters.Request()
        request.parameters = [param]
        future = self.set_params_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            for result in future.result().results:
                if result.successful:
                    self.get_logger().info(f'参数 {param_name} 设置成功')
                    return True
                else:
                    self.get_logger().error(f'参数 {param_name} 设置失败: {result.reason}')
                    return False
        else:
            self.get_logger().error('设置参数失败')
            return False
    
    def export_parameters_to_json(self, filename='foxglove_params_export.json'):
        """导出所有参数到JSON文件"""
        params = self.list_all_parameters()
        param_dict = {}
        
        for param_name in params:
            param_value = self.get_parameter(param_name)
            if param_value is not None:
                param_dict[param_name] = {
                    'type': ParameterType.PARAMETER_TYPE_STRING_TO_STR.get(
                        param_value.type, 'unknown'
                    ),
                    'value': self._param_value_to_python(param_value)
                }
        
        with open(filename, 'w') as f:
            json.dump(param_dict, f, indent=2)
        
        self.get_logger().info(f'参数已导出到 {filename}')
        return param_dict
    
    def _param_value_to_python(self, param_value):
        """将ROS参数值转换为Python原生类型"""
        if param_value.type == ParameterType.PARAMETER_BOOL:
            return param_value.bool_value
        elif param_value.type == ParameterType.PARAMETER_INTEGER:
            return param_value.integer_value
        elif param_value.type == ParameterType.PARAMETER_DOUBLE:
            return param_value.double_value
        elif param_value.type == ParameterType.PARAMETER_STRING:
            return param_value.string_value
        elif param_value.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
            return list(param_value.double_array_value)
        elif param_value.type == ParameterType.PARAMETER_INTEGER_ARRAY:
            return list(param_value.integer_array_value)
        else:
            return None


def main():
    rclpy.init()
    
    helper = FoxgloveParamHelper()
    
    if len(sys.argv) > 1:
        command = sys.argv[1]
        
        if command == 'list':
            params = helper.list_all_parameters()
            print("可配置的参数:")
            for param in params:
                print(f"  - {param}")
        
        elif command == 'get' and len(sys.argv) > 2:
            param_name = sys.argv[2]
            value = helper.get_parameter(param_name)
            if value is not None:
                python_value = helper._param_value_to_python(value)
                print(f"{param_name} = {python_value}")
        
        elif command == 'set' and len(sys.argv) > 3:
            param_name = sys.argv[2]
            param_value_str = sys.argv[3]
            
            # 尝试解析参数值
            try:
                # 尝试JSON解析（支持数组和复杂类型）
                param_value = json.loads(param_value_str)
            except json.JSONDecodeError:
                # 简单字符串
                param_value = param_value_str
            
            success = helper.set_parameter(param_name, param_value)
            if success:
                print(f"参数 {param_name} 设置成功")
            else:
                print(f"参数 {param_name} 设置失败")
        
        elif command == 'export':
            filename = sys.argv[2] if len(sys.argv) > 2 else 'foxglove_params_export.json'
            helper.export_parameters_to_json(filename)
        
        else:
            print("用法:")
            print("  python3 foxglove_param_helper.py list              # 列出所有参数")
            print("  python3 foxglove_param_helper.py get <param_name>  # 获取参数值")
            print("  python3 foxglove_param_helper.py set <name> <value> # 设置参数值")
            print("  python3 foxglove_param_helper.py export [filename]  # 导出参数到JSON")
    else:
        print("Foxglove参数辅助工具")
        print("用法:")
        print("  python3 foxglove_param_helper.py list              # 列出所有参数")
        print("  python3 foxglove_param_helper.py get <param_name>  # 获取参数值")
        print("  python3 foxglove_param_helper.py set <name> <value> # 设置参数值")
        print("  python3 foxglove_param_helper.py export [filename]  # 导出参数到JSON")
    
    helper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()