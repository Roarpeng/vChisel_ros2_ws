/**
 * 设置页面
 */

import { useState } from 'react';
import { useRosConnection } from '@/hooks/useRosConnection';

export function SettingsPage() {
  const { connected, url, connect, disconnect } = useRosConnection();
  const [rosUrl, setRosUrl] = useState(url || 'ws://localhost:9090');
  const [plcIp, setPlcIp] = useState('192.168.0.1');
  const [message, setMessage] = useState('');

  const handleConnect = () => {
    connect(rosUrl);
    setMessage('正在连接...');
  };

  const handleDisconnect = () => {
    disconnect();
    setMessage('已断开连接');
  };

  return (
    <div className="p-6 space-y-6">
      <h1 className="text-2xl font-bold">系统设置</h1>

      {/* ROS连接设置 */}
      <div className="card">
        <h3 className="text-lg font-medium mb-4">ROS Bridge 连接</h3>
        <div className="flex items-center gap-4 mb-4">
          <div className="flex-1">
            <label className="block text-sm text-bosch-gray-400 mb-1">WebSocket URL</label>
            <input
              type="text"
              value={rosUrl}
              onChange={(e) => setRosUrl(e.target.value)}
              className="input-field w-full"
              placeholder="ws://localhost:9090"
            />
          </div>
          <div className="pt-6">
            {connected ? (
              <button onClick={handleDisconnect} className="btn-danger">
                断开
              </button>
            ) : (
              <button onClick={handleConnect} className="btn-success">
                连接
              </button>
            )}
          </div>
        </div>
        <div className="flex items-center gap-2">
          <div className={`w-3 h-3 rounded-full ${connected ? 'bg-status-ok' : 'bg-status-error'}`} />
          <span className="text-sm text-bosch-gray-400">
            {connected ? '已连接' : '未连接'}
          </span>
        </div>
      </div>

      {/* PLC设置 */}
      <div className="card">
        <h3 className="text-lg font-medium mb-4">PLC 设置</h3>
        <div className="grid grid-cols-3 gap-4">
          <div>
            <label className="block text-sm text-bosch-gray-400 mb-1">PLC IP地址</label>
            <input
              type="text"
              value={plcIp}
              onChange={(e) => setPlcIp(e.target.value)}
              className="input-field w-full"
            />
          </div>
          <div>
            <label className="block text-sm text-bosch-gray-400 mb-1">机架号</label>
            <input
              type="number"
              defaultValue={0}
              className="input-field w-full"
            />
          </div>
          <div>
            <label className="block text-sm text-bosch-gray-400 mb-1">插槽号</label>
            <input
              type="number"
              defaultValue={1}
              className="input-field w-full"
            />
          </div>
        </div>
        <p className="mt-4 text-sm text-bosch-gray-500">
          注意: PLC设置需要重启ROS节点才能生效
        </p>
      </div>

      {/* 相机设置 */}
      <div className="card">
        <h3 className="text-lg font-medium mb-4">相机设置</h3>
        <div className="grid grid-cols-2 gap-4">
          <div>
            <label className="block text-sm text-bosch-gray-400 mb-1">相机序列号</label>
            <input
              type="text"
              placeholder="留空自动检测"
              className="input-field w-full"
            />
          </div>
          <div>
            <label className="block text-sm text-bosch-gray-400 mb-1">帧率</label>
            <select className="input-field w-full">
              <option value="30">30 FPS</option>
              <option value="15">15 FPS</option>
              <option value="6">6 FPS</option>
            </select>
          </div>
        </div>
      </div>

      {/* 存储路径 */}
      <div className="card">
        <h3 className="text-lg font-medium mb-4">存储路径</h3>
        <div className="space-y-4">
          <div>
            <label className="block text-sm text-bosch-gray-400 mb-1">配方目录</label>
            <input
              type="text"
              defaultValue="~/.vchisel/recipes"
              className="input-field w-full"
              readOnly
            />
          </div>
          <div>
            <label className="block text-sm text-bosch-gray-400 mb-1">标定数据目录</label>
            <input
              type="text"
              defaultValue="~/.vchisel/calibrations"
              className="input-field w-full"
              readOnly
            />
          </div>
          <div>
            <label className="block text-sm text-bosch-gray-400 mb-1">日志目录</label>
            <input
              type="text"
              defaultValue="~/.ros/log"
              className="input-field w-full"
              readOnly
            />
          </div>
        </div>
      </div>

      {/* 消息 */}
      {message && (
        <div className="card bg-bosch-gray-700">
          <p className="text-sm">{message}</p>
        </div>
      )}

      {/* 系统信息 */}
      <div className="card">
        <h3 className="text-lg font-medium mb-4">系统信息</h3>
        <div className="grid grid-cols-2 gap-4 text-sm">
          <div>
            <span className="text-bosch-gray-400">应用版本:</span> 1.0.0
          </div>
          <div>
            <span className="text-bosch-gray-400">ROS版本:</span> Humble
          </div>
          <div>
            <span className="text-bosch-gray-400">平台:</span>{' '}
            {typeof window !== 'undefined' && (window as any).electronAPI?.platform || 'Web'}
          </div>
        </div>
      </div>
    </div>
  );
}
