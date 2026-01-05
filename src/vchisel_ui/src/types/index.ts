// 系统状态类型
export interface SystemStatus {
  plc_connected: boolean;
  camera_connected: boolean;
  current_recipe: string;
  operation_state: OperationState;
  points_processed: number;
  cycles_completed: number;
  calib_mode_active: boolean;
  calib_points_captured: number;
  calib_quality_score: number;
  last_error: string;
  last_error_time: string;
  timestamp: string;
}

export type OperationState = 'IDLE' | 'RUNNING' | 'ERROR' | 'RECOVERING' | 'CALIBRATING';

// 报警类型
export interface Alarm {
  level: AlarmLevel;
  code: string;
  message: string;
  source: string;
  timestamp: string;
  acknowledged: boolean;
  acknowledged_at?: string;
}

export enum AlarmLevel {
  INFO = 0,
  WARNING = 1,
  ERROR = 2,
  CRITICAL = 3,
}

// 报警统计
export interface AlarmStats {
  total_alarms: number;
  active_alarms: number;
  critical_count: number;
  error_count: number;
  warning_count: number;
  info_count: number;
  timestamp: string;
}

// 配方类型
export interface Recipe {
  name: string;
  version: string;
  description: string;
  created_at: string;
  modified_at: string;

  // 相机参数
  camera_exposure: number;
  camera_gain: number;

  // 检测参数
  detection_threshold: number;
  min_area: number;
  max_area: number;

  // 凿击参数
  chisel_force: number;
  chisel_angle: number;
  chisel_depth: number;

  // 速度参数
  approach_speed: number;
  working_speed: number;
  retreat_speed: number;
}

export interface RecipeInfo {
  name: string;
  version: string;
  description: string;
  modified_at: string;
  is_active: boolean;
}

// 标定状态
export interface CalibrationStatus {
  mode: 'idle' | 'intrinsic' | 'handeye';
  is_running: boolean;
  points_captured: number;
  min_points_required: number;
  quality_score: number;
  last_error: string;
  preview_available: boolean;
}

// 机器人位姿
export interface RobotPose {
  x: number;
  y: number;
  z: number;
  a: number;  // Rx
  b: number;  // Ry
  c: number;  // Rz
}

// ROS连接状态
export interface RosConnectionStatus {
  connected: boolean;
  url: string;
  error?: string;
}
