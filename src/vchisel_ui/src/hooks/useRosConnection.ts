/**
 * ROS连接管理Hook
 */

import { useEffect } from 'react';
import { rosBridge } from '@/services/rosBridge';
import { useAppStore } from '@/store/appStore';
import type { SystemStatus, Alarm, AlarmStats, RosConnectionStatus } from '@/types';

export function useRosConnection() {
  const {
    rosConnection,
    setRosConnection,
    setSystemStatus,
    addAlarm,
    setAlarmStats,
  } = useAppStore();

  useEffect(() => {
    // 订阅连接状态
    const unsubConnection = rosBridge.on('connection', (data) => {
      setRosConnection(data as RosConnectionStatus);
    });

    // 订阅系统状态
    const unsubStatus = rosBridge.on('system_status', (data) => {
      setSystemStatus(data as SystemStatus);
    });

    // 订阅报警
    const unsubAlarm = rosBridge.on('alarm', (data) => {
      addAlarm(data as Alarm);
    });

    // 订阅报警统计
    const unsubStats = rosBridge.on('alarm_stats', (data) => {
      setAlarmStats(data as AlarmStats);
    });

    // 清理订阅
    return () => {
      unsubConnection();
      unsubStatus();
      unsubAlarm();
      unsubStats();
    };
  }, [setRosConnection, setSystemStatus, addAlarm, setAlarmStats]);

  const connect = (url: string) => {
    rosBridge.connect(url);
  };

  const disconnect = () => {
    rosBridge.disconnect();
  };

  return {
    ...rosConnection,
    connect,
    disconnect,
  };
}
