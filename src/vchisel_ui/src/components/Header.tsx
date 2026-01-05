/**
 * 顶部状态栏组件
 */

import { StatusIndicator } from './StatusIndicator';
import { useAppStore } from '@/store/appStore';
import { useRosConnection } from '@/hooks/useRosConnection';
import { format } from 'date-fns';

export function Header() {
  const { systemStatus, alarmStats } = useAppStore();
  const { connected } = useRosConnection();

  const getOperationStateLabel = (state: string | undefined): string => {
    const labels: Record<string, string> = {
      IDLE: '空闲',
      RUNNING: '运行中',
      ERROR: '故障',
      RECOVERING: '恢复中',
      CALIBRATING: '标定中',
    };
    return labels[state || ''] || '未知';
  };

  const getOperationStateStatus = (
    state: string | undefined
  ): 'ok' | 'warning' | 'error' | 'offline' => {
    switch (state) {
      case 'RUNNING':
        return 'ok';
      case 'CALIBRATING':
      case 'RECOVERING':
        return 'warning';
      case 'ERROR':
        return 'error';
      default:
        return 'offline';
    }
  };

  return (
    <header className="h-12 bg-bosch-gray-800 border-b border-bosch-gray-700 px-4 flex items-center justify-between">
      {/* 左侧：系统状态 */}
      <div className="flex items-center gap-6">
        {/* ROS连接 */}
        <StatusIndicator
          status={connected ? 'ok' : 'error'}
          label={connected ? 'ROS已连接' : 'ROS断开'}
          pulse={!connected}
        />

        {/* PLC连接 */}
        <StatusIndicator
          status={systemStatus?.plc_connected ? 'ok' : 'error'}
          label={systemStatus?.plc_connected ? 'PLC已连接' : 'PLC断开'}
        />

        {/* 相机连接 */}
        <StatusIndicator
          status={systemStatus?.camera_connected ? 'ok' : 'error'}
          label={systemStatus?.camera_connected ? '相机已连接' : '相机断开'}
        />

        {/* 运行状态 */}
        <StatusIndicator
          status={getOperationStateStatus(systemStatus?.operation_state)}
          label={getOperationStateLabel(systemStatus?.operation_state)}
          pulse={systemStatus?.operation_state === 'RUNNING'}
        />
      </div>

      {/* 中间：配方信息 */}
      <div className="flex items-center gap-4">
        <span className="text-sm text-bosch-gray-400">
          当前配方: <span className="text-white">{systemStatus?.current_recipe || '-'}</span>
        </span>
        <span className="text-bosch-gray-600">|</span>
        <span className="text-sm text-bosch-gray-400">
          已处理: <span className="text-white">{systemStatus?.points_processed || 0}</span> 点
        </span>
        <span className="text-bosch-gray-600">|</span>
        <span className="text-sm text-bosch-gray-400">
          周期: <span className="text-white">{systemStatus?.cycles_completed || 0}</span>
        </span>
      </div>

      {/* 右侧：报警和时间 */}
      <div className="flex items-center gap-4">
        {/* 活跃报警数 */}
        {(alarmStats?.active_alarms ?? 0) > 0 && (
          <div className="flex items-center gap-1 px-2 py-1 bg-status-error rounded text-white text-sm">
            <span>{alarmStats?.active_alarms}</span>
            <span>报警</span>
          </div>
        )}

        {/* 当前时间 */}
        <span className="text-sm text-bosch-gray-400">
          {format(new Date(), 'yyyy-MM-dd HH:mm:ss')}
        </span>
      </div>
    </header>
  );
}
