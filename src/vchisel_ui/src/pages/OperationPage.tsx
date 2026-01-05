/**
 * 运行控制页面
 */

import { useState } from 'react';
import { useAppStore } from '@/store/appStore';
import { StatusIndicator } from '@/components/StatusIndicator';
import { rosBridge } from '@/services/rosBridge';

export function OperationPage() {
  const { systemStatus } = useAppStore();
  const [loading, setLoading] = useState(false);
  const [message, setMessage] = useState('');

  const handleStart = async () => {
    setLoading(true);
    setMessage('正在启动...');
    try {
      // 调用启动服务
      await rosBridge.callService('/vchisel/start', 'std_srvs/Trigger', {});
      setMessage('启动成功');
    } catch (error) {
      setMessage(`启动失败: ${error}`);
    } finally {
      setLoading(false);
    }
  };

  const handleStop = async () => {
    setLoading(true);
    setMessage('正在停止...');
    try {
      await rosBridge.callService('/vchisel/stop', 'std_srvs/Trigger', {});
      setMessage('已停止');
    } catch (error) {
      setMessage(`停止失败: ${error}`);
    } finally {
      setLoading(false);
    }
  };

  const handleReset = async () => {
    setLoading(true);
    setMessage('正在复位...');
    try {
      await rosBridge.callService('/vchisel/reset', 'std_srvs/Trigger', {});
      setMessage('复位成功');
    } catch (error) {
      setMessage(`复位失败: ${error}`);
    } finally {
      setLoading(false);
    }
  };

  const getStateColor = (state: string | undefined) => {
    switch (state) {
      case 'RUNNING': return 'text-status-ok';
      case 'ERROR': return 'text-status-error';
      case 'CALIBRATING': return 'text-status-warning';
      default: return 'text-bosch-gray-400';
    }
  };

  return (
    <div className="p-6 space-y-6">
      <h1 className="text-2xl font-bold">运行控制</h1>

      {/* 状态概览 */}
      <div className="grid grid-cols-3 gap-4">
        <div className="card">
          <h3 className="text-sm text-bosch-gray-400 mb-2">运行状态</h3>
          <div className={`text-3xl font-bold ${getStateColor(systemStatus?.operation_state)}`}>
            {systemStatus?.operation_state || 'UNKNOWN'}
          </div>
        </div>

        <div className="card">
          <h3 className="text-sm text-bosch-gray-400 mb-2">当前配方</h3>
          <div className="text-2xl font-bold">
            {systemStatus?.current_recipe || '-'}
          </div>
        </div>

        <div className="card">
          <h3 className="text-sm text-bosch-gray-400 mb-2">连接状态</h3>
          <div className="flex items-center gap-4">
            <StatusIndicator
              status={systemStatus?.plc_connected ? 'ok' : 'error'}
              label="PLC"
            />
            <StatusIndicator
              status={systemStatus?.camera_connected ? 'ok' : 'error'}
              label="相机"
            />
          </div>
        </div>
      </div>

      {/* 控制按钮 */}
      <div className="card">
        <h3 className="text-lg font-medium mb-4">操作控制</h3>
        <div className="flex gap-4">
          <button
            onClick={handleStart}
            disabled={loading || systemStatus?.operation_state === 'RUNNING'}
            className="btn-success px-8 py-3 text-lg disabled:opacity-50"
          >
            启动
          </button>
          <button
            onClick={handleStop}
            disabled={loading || systemStatus?.operation_state !== 'RUNNING'}
            className="btn-danger px-8 py-3 text-lg disabled:opacity-50"
          >
            停止
          </button>
          <button
            onClick={handleReset}
            disabled={loading}
            className="btn-secondary px-8 py-3 text-lg disabled:opacity-50"
          >
            复位
          </button>
        </div>
        {message && (
          <p className="mt-4 text-sm text-bosch-gray-400">{message}</p>
        )}
      </div>

      {/* 统计信息 */}
      <div className="grid grid-cols-2 gap-4">
        <div className="card">
          <h3 className="text-lg font-medium mb-4">处理统计</h3>
          <div className="space-y-3">
            <div className="flex justify-between">
              <span className="text-bosch-gray-400">已处理点数</span>
              <span className="text-xl font-bold">{systemStatus?.points_processed || 0}</span>
            </div>
            <div className="flex justify-between">
              <span className="text-bosch-gray-400">完成周期</span>
              <span className="text-xl font-bold">{systemStatus?.cycles_completed || 0}</span>
            </div>
          </div>
        </div>

        <div className="card">
          <h3 className="text-lg font-medium mb-4">错误信息</h3>
          {systemStatus?.last_error ? (
            <div>
              <p className="text-status-error">{systemStatus.last_error}</p>
              <p className="text-sm text-bosch-gray-400 mt-2">
                {systemStatus.last_error_time}
              </p>
            </div>
          ) : (
            <p className="text-bosch-gray-400">无错误</p>
          )}
        </div>
      </div>
    </div>
  );
}
