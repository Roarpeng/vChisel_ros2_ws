/**
 * 应用状态管理 - Zustand Store
 */

import { create } from 'zustand';
import type {
  SystemStatus,
  Alarm,
  AlarmStats,
  Recipe,
  RecipeInfo,
  RosConnectionStatus,
  CalibrationStatus
} from '@/types';

interface AppState {
  // ROS连接状态
  rosConnection: RosConnectionStatus;
  setRosConnection: (status: RosConnectionStatus) => void;

  // 系统状态
  systemStatus: SystemStatus | null;
  setSystemStatus: (status: SystemStatus) => void;

  // 报警
  alarms: Alarm[];
  alarmStats: AlarmStats | null;
  addAlarm: (alarm: Alarm) => void;
  setAlarmStats: (stats: AlarmStats) => void;
  clearAlarms: () => void;

  // 配方
  recipes: RecipeInfo[];
  currentRecipe: Recipe | null;
  setRecipes: (recipes: RecipeInfo[]) => void;
  setCurrentRecipe: (recipe: Recipe | null) => void;

  // 标定
  calibrationStatus: CalibrationStatus;
  setCalibrationStatus: (status: Partial<CalibrationStatus>) => void;

  // UI状态
  sidebarOpen: boolean;
  toggleSidebar: () => void;
  currentPage: string;
  setCurrentPage: (page: string) => void;
}

const initialCalibrationStatus: CalibrationStatus = {
  mode: 'idle',
  is_running: false,
  points_captured: 0,
  min_points_required: 10,
  quality_score: 0,
  last_error: '',
  preview_available: false,
};

export const useAppStore = create<AppState>((set) => ({
  // ROS连接
  rosConnection: { connected: false, url: 'ws://localhost:9090' },
  setRosConnection: (status) => set({ rosConnection: status }),

  // 系统状态
  systemStatus: null,
  setSystemStatus: (status) => set({ systemStatus: status }),

  // 报警
  alarms: [],
  alarmStats: null,
  addAlarm: (alarm) =>
    set((state) => ({
      alarms: [alarm, ...state.alarms].slice(0, 100), // 保留最新100条
    })),
  setAlarmStats: (stats) => set({ alarmStats: stats }),
  clearAlarms: () => set({ alarms: [] }),

  // 配方
  recipes: [],
  currentRecipe: null,
  setRecipes: (recipes) => set({ recipes }),
  setCurrentRecipe: (recipe) => set({ currentRecipe: recipe }),

  // 标定
  calibrationStatus: initialCalibrationStatus,
  setCalibrationStatus: (status) =>
    set((state) => ({
      calibrationStatus: { ...state.calibrationStatus, ...status },
    })),

  // UI
  sidebarOpen: true,
  toggleSidebar: () => set((state) => ({ sidebarOpen: !state.sidebarOpen })),
  currentPage: 'dashboard',
  setCurrentPage: (page) => set({ currentPage: page }),
}));
