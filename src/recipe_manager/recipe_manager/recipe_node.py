#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
配方管理节点
Recipe Manager Node

功能：
1. 配方CRUD操作（创建、读取、更新、删除）
2. 配方激活/切换
3. 版本管理和自动快照
4. 动态参数更新到 norm_calc 节点
"""

import os
import sys
import yaml
import shutil
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Any

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import ParameterValue, ParameterType

from std_srvs.srv import Trigger


class RecipeManagerNode(Node):
    def __init__(self):
        super().__init__('recipe_manager_node')

        # 参数
        self.declare_parameter('recipe_dir', os.path.expanduser('~/.vchisel/recipes'))
        self.declare_parameter('backup_dir', os.path.expanduser('~/.vchisel/recipes/backups'))
        self.declare_parameter('default_recipe', 'default')
        self.declare_parameter('max_backups', 10)

        self.recipe_dir = self.get_parameter('recipe_dir').get_parameter_value().string_value
        self.backup_dir = self.get_parameter('backup_dir').get_parameter_value().string_value
        self.default_recipe = self.get_parameter('default_recipe').get_parameter_value().string_value
        self.max_backups = self.get_parameter('max_backups').get_parameter_value().integer_value

        # 确保目录存在
        Path(self.recipe_dir).mkdir(parents=True, exist_ok=True)
        Path(self.backup_dir).mkdir(parents=True, exist_ok=True)

        # 当前激活的配方
        self.active_recipe = None
        self.active_recipe_data = None

        # 服务：列出配方
        self.list_srv = self.create_service(
            Trigger,
            'recipe/list',
            self.handle_list_recipes
        )

        # 服务：加载配方
        self.load_srv = self.create_service(
            Trigger,
            'recipe/load',
            self.handle_load_recipe
        )

        # 服务：保存配方
        self.save_srv = self.create_service(
            Trigger,
            'recipe/save',
            self.handle_save_recipe
        )

        # 服务：删除配方
        self.delete_srv = self.create_service(
            Trigger,
            'recipe/delete',
            self.handle_delete_recipe
        )

        # 服务：激活配方
        self.activate_srv = self.create_service(
            Trigger,
            'recipe/activate',
            self.handle_activate_recipe
        )

        # 服务：获取当前配方
        self.current_srv = self.create_service(
            Trigger,
            'recipe/current',
            self.handle_get_current_recipe
        )

        # 参数设置客户端（用于更新norm_calc参数）
        self.param_clients = {}

        # 创建默认配方（如果不存在）
        self._create_default_recipe_if_not_exists()

        # 加载默认配方
        self._load_default_recipe()

        self.get_logger().info(f'Recipe manager started. Recipe dir: {self.recipe_dir}')

    def _create_default_recipe_if_not_exists(self):
        """创建默认配方文件"""
        default_path = os.path.join(self.recipe_dir, f'{self.default_recipe}.yaml')
        if not os.path.exists(default_path):
            default_recipe = self._get_default_recipe_data()
            self._save_recipe_to_file(self.default_recipe, default_recipe)
            self.get_logger().info(f'Created default recipe: {default_path}')

    def _get_default_recipe_data(self) -> Dict[str, Any]:
        """获取默认配方数据"""
        return {
            'name': 'default',
            'description': 'Default recipe for vChisel system',
            'version': '1.0.0',
            'created_at': datetime.now().isoformat(),
            'updated_at': datetime.now().isoformat(),

            # 区域设置
            'box_len': 50.0,
            'box_row': 5,
            'box_column': 5,

            # 范围限制 (mm)
            'x_min': -500.0,
            'x_max': 500.0,
            'y_min': -500.0,
            'y_max': 500.0,
            'z_min': 0.0,
            'z_max': 1000.0,

            # 严格模式参数
            'strict_norm_th': 0.8,
            'strict_hole_dist': 10.0,
            'strict_curv_th': 0.1,

            # 宽松模式参数
            'relaxed_norm_th': 0.6,
            'relaxed_hole_dist': 20.0,
            'relaxed_curv_th': 0.2,

            # 评分权重
            'height_weight': 0.3,
            'curv_weight': 0.2,
            'angle_weight': 0.3,
            'center_weight': 0.2,

            # 凸起处理
            'protrusion_th': 5.0,
            'tip_crop_ratio': 0.1,
            'base_crop_ratio': 0.2,
        }

    def _load_default_recipe(self):
        """加载默认配方"""
        try:
            recipe_data = self._load_recipe_from_file(self.default_recipe)
            if recipe_data:
                self.active_recipe = self.default_recipe
                self.active_recipe_data = recipe_data
                self.get_logger().info(f'Loaded default recipe: {self.default_recipe}')
        except Exception as e:
            self.get_logger().warning(f'Failed to load default recipe: {e}')

    def _get_recipe_path(self, name: str) -> str:
        """获取配方文件路径"""
        return os.path.join(self.recipe_dir, f'{name}.yaml')

    def _list_all_recipes(self) -> List[Dict[str, Any]]:
        """列出所有配方"""
        recipes = []
        for filename in os.listdir(self.recipe_dir):
            if filename.endswith('.yaml') and not filename.startswith('.'):
                name = filename[:-5]  # 去掉 .yaml 后缀
                try:
                    recipe_data = self._load_recipe_from_file(name)
                    if recipe_data:
                        recipes.append({
                            'name': recipe_data.get('name', name),
                            'description': recipe_data.get('description', ''),
                            'version': recipe_data.get('version', '1.0.0'),
                            'updated_at': recipe_data.get('updated_at', ''),
                            'is_active': name == self.active_recipe
                        })
                except Exception as e:
                    self.get_logger().warning(f'Failed to load recipe {name}: {e}')
        return recipes

    def _load_recipe_from_file(self, name: str) -> Optional[Dict[str, Any]]:
        """从文件加载配方"""
        path = self._get_recipe_path(name)
        if not os.path.exists(path):
            return None

        with open(path, 'r') as f:
            return yaml.safe_load(f)

    def _save_recipe_to_file(self, name: str, data: Dict[str, Any]) -> bool:
        """保存配方到文件"""
        path = self._get_recipe_path(name)

        # 备份旧文件
        if os.path.exists(path):
            self._backup_recipe(name)

        # 更新时间戳
        data['updated_at'] = datetime.now().isoformat()

        with open(path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, allow_unicode=True)

        return True

    def _backup_recipe(self, name: str):
        """备份配方文件"""
        src_path = self._get_recipe_path(name)
        if not os.path.exists(src_path):
            return

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        backup_path = os.path.join(self.backup_dir, f'{name}_{timestamp}.yaml')

        shutil.copy2(src_path, backup_path)
        self.get_logger().debug(f'Backed up recipe {name} to {backup_path}')

        # 清理旧备份
        self._cleanup_old_backups(name)

    def _cleanup_old_backups(self, name: str):
        """清理旧备份，保留最新的N个"""
        backups = []
        for filename in os.listdir(self.backup_dir):
            if filename.startswith(f'{name}_') and filename.endswith('.yaml'):
                path = os.path.join(self.backup_dir, filename)
                backups.append((path, os.path.getmtime(path)))

        # 按时间排序，删除旧的
        backups.sort(key=lambda x: x[1], reverse=True)
        for path, _ in backups[self.max_backups:]:
            os.remove(path)
            self.get_logger().debug(f'Removed old backup: {path}')

    def _delete_recipe_file(self, name: str) -> bool:
        """删除配方文件"""
        path = self._get_recipe_path(name)
        if not os.path.exists(path):
            return False

        # 备份后删除
        self._backup_recipe(name)
        os.remove(path)
        return True

    def _apply_recipe_to_norm_calc(self, recipe_data: Dict[str, Any]) -> bool:
        """将配方参数应用到 norm_calc 节点"""
        # TODO: 实现动态参数更新
        # 这需要使用 ROS2 参数服务来更新 norm_calc 节点的参数
        self.get_logger().info(f'Applying recipe parameters to norm_calc (not implemented yet)')
        return True

    def handle_list_recipes(self, request, response):
        """处理列出配方请求"""
        try:
            recipes = self._list_all_recipes()
            response.success = True
            response.message = str({
                'recipes': recipes,
                'active_recipe': self.active_recipe,
                'count': len(recipes)
            })
        except Exception as e:
            response.success = False
            response.message = f'Failed to list recipes: {str(e)}'
        return response

    def handle_load_recipe(self, request, response):
        """处理加载配方请求"""
        # 从消息中解析配方名称
        # 注意：这里使用 Trigger 服务，实际应用中应该使用自定义服务
        # 暂时使用默认配方进行演示
        try:
            recipe_name = self.default_recipe  # TODO: 从请求中获取
            recipe_data = self._load_recipe_from_file(recipe_name)

            if recipe_data:
                response.success = True
                response.message = str(recipe_data)
            else:
                response.success = False
                response.message = f'Recipe not found: {recipe_name}'
        except Exception as e:
            response.success = False
            response.message = f'Failed to load recipe: {str(e)}'
        return response

    def handle_save_recipe(self, request, response):
        """处理保存配方请求"""
        # 暂时保存当前激活的配方
        try:
            if self.active_recipe_data:
                self._save_recipe_to_file(self.active_recipe, self.active_recipe_data)
                response.success = True
                response.message = f'Recipe saved: {self.active_recipe}'
            else:
                response.success = False
                response.message = 'No active recipe to save'
        except Exception as e:
            response.success = False
            response.message = f'Failed to save recipe: {str(e)}'
        return response

    def handle_delete_recipe(self, request, response):
        """处理删除配方请求"""
        # TODO: 从请求中获取配方名称
        response.success = False
        response.message = 'Delete requires recipe name (not implemented with Trigger service)'
        return response

    def handle_activate_recipe(self, request, response):
        """处理激活配方请求"""
        # TODO: 从请求中获取配方名称并激活
        try:
            recipe_name = self.default_recipe  # TODO: 从请求中获取
            recipe_data = self._load_recipe_from_file(recipe_name)

            if recipe_data:
                self.active_recipe = recipe_name
                self.active_recipe_data = recipe_data
                self._apply_recipe_to_norm_calc(recipe_data)

                response.success = True
                response.message = f'Recipe activated: {recipe_name}'
            else:
                response.success = False
                response.message = f'Recipe not found: {recipe_name}'
        except Exception as e:
            response.success = False
            response.message = f'Failed to activate recipe: {str(e)}'
        return response

    def handle_get_current_recipe(self, request, response):
        """处理获取当前配方请求"""
        try:
            if self.active_recipe and self.active_recipe_data:
                response.success = True
                response.message = str({
                    'name': self.active_recipe,
                    'data': self.active_recipe_data
                })
            else:
                response.success = False
                response.message = 'No active recipe'
        except Exception as e:
            response.success = False
            response.message = f'Failed to get current recipe: {str(e)}'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = RecipeManagerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
