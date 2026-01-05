/**
 * 报警页面
 */

import { useState } from 'react';
import { useAppStore } from '@/store/appStore';
import { rosBridge } from '@/services/rosBridge';
import { AlarmLevel } from '@/types';
import clsx from 'clsx';

export function AlarmsPage() {
  const { alarms, alarmStats, clearAlarms: clearLocalAlarms } = useAppStore();
  const [loading, setLoading] = useState(false);
  const [message, setMessage] = useState('');
  const [filter, setFilter] = useState<'all' | 'active' | 'critical' | 'error' | 'warning'>('all');

  const handleAcknowledge = async () => {
    setLoading(true);
    try {
      await rosBridge.acknowledgeAlarms();
      setMessage('所有报警已确认');
    } catch (error) {
      setMessage(`确认失败: ${error}`);
    } finally {
      setLoading(false);
    }
  };

  const handleClearAll = async () => {
    if (!confirm('确定要清除所有报警吗？')) return;
    setLoading(true);
    try {
      await rosBridge.clearAlarms();
      clearLocalAlarms();
      setMessage('所有报警已清除');
    } catch (error) {
      setMessage(`清除失败: ${error}`);
    } finally {
      setLoading(false);
    }
  };

  const getLevelName = (level: number) => {
    switch (level) {
      case AlarmLevel.INFO: return '信息';
      case AlarmLevel.WARNING: return '警告';
      case AlarmLevel.ERROR: return '错误';
      case AlarmLevel.CRITICAL: return '严重';
      default: return '未知';
    }
  };

  const getLevelColor = (level: number) => {
    switch (level) {
      case AlarmLevel.INFO: return 'bg-status-info';
      case AlarmLevel.WARNING: return 'bg-yellow-500';
      case AlarmLevel.ERROR: return 'bg-status-warning';
      case AlarmLevel.CRITICAL: return 'bg-status-error';
      default: return 'bg-bosch-gray-500';
    }
  };

  const filteredAlarms = alarms.filter((alarm) => {
    if (filter === 'all') return true;
    if (filter === 'active') return !alarm.acknowledged;
    if (filter === 'critical') return alarm.level === AlarmLevel.CRITICAL;
    if (filter === 'error') return alarm.level === AlarmLevel.ERROR;
    if (filter === 'warning') return alarm.level === AlarmLevel.WARNING;
    return true;
  });

  return (
    <div className="p-6 space-y-6">
      <div className="flex items-center justify-between">
        <h1 className="text-2xl font-bold">报警管理</h1>
        <div className="flex gap-2">
          <button onClick={handleAcknowledge} disabled={loading} className="btn-secondary">
            确认所有
          </button>
          <button onClick={handleClearAll} disabled={loading} className="btn-danger">
            清除所有
          </button>
        </div>
      </div>

      {/* 统计卡片 */}
      <div className="grid grid-cols-5 gap-4">
        <div className="card text-center">
          <div className="text-3xl font-bold">{alarmStats?.total_alarms || 0}</div>
          <div className="text-sm text-bosch-gray-400">总计</div>
        </div>
        <div className="card text-center">
          <div className="text-3xl font-bold text-status-error">
            {alarmStats?.active_alarms || 0}
          </div>
          <div className="text-sm text-bosch-gray-400">活跃</div>
        </div>
        <div className="card text-center">
          <div className="text-3xl font-bold text-status-error">
            {alarmStats?.critical_count || 0}
          </div>
          <div className="text-sm text-bosch-gray-400">严重</div>
        </div>
        <div className="card text-center">
          <div className="text-3xl font-bold text-status-warning">
            {alarmStats?.error_count || 0}
          </div>
          <div className="text-sm text-bosch-gray-400">错误</div>
        </div>
        <div className="card text-center">
          <div className="text-3xl font-bold text-yellow-500">
            {alarmStats?.warning_count || 0}
          </div>
          <div className="text-sm text-bosch-gray-400">警告</div>
        </div>
      </div>

      {/* 消息 */}
      {message && (
        <div className="card bg-bosch-gray-700">
          <p className="text-sm">{message}</p>
        </div>
      )}

      {/* 过滤器 */}
      <div className="flex gap-2">
        {(['all', 'active', 'critical', 'error', 'warning'] as const).map((f) => (
          <button
            key={f}
            onClick={() => setFilter(f)}
            className={clsx(
              'px-4 py-2 rounded transition-colors',
              filter === f
                ? 'bg-bosch-red text-white'
                : 'bg-bosch-gray-700 text-bosch-gray-300 hover:bg-bosch-gray-600'
            )}
          >
            {f === 'all' ? '全部' :
             f === 'active' ? '活跃' :
             f === 'critical' ? '严重' :
             f === 'error' ? '错误' : '警告'}
          </button>
        ))}
      </div>

      {/* 报警列表 */}
      <div className="card">
        <h3 className="text-lg font-medium mb-4">
          报警列表 ({filteredAlarms.length})
        </h3>
        {filteredAlarms.length === 0 ? (
          <p className="text-bosch-gray-400">暂无报警</p>
        ) : (
          <div className="space-y-2 max-h-96 overflow-y-auto">
            {filteredAlarms.map((alarm, index) => (
              <div
                key={index}
                className={clsx(
                  'p-3 rounded border-l-4',
                  alarm.acknowledged ? 'bg-bosch-gray-700 opacity-60' : 'bg-bosch-gray-700',
                  alarm.level === AlarmLevel.CRITICAL ? 'border-status-error' :
                  alarm.level === AlarmLevel.ERROR ? 'border-status-warning' :
                  alarm.level === AlarmLevel.WARNING ? 'border-yellow-500' : 'border-status-info'
                )}
              >
                <div className="flex items-start justify-between">
                  <div className="flex items-center gap-3">
                    <span className={clsx('px-2 py-0.5 rounded text-xs text-white', getLevelColor(alarm.level))}>
                      {getLevelName(alarm.level)}
                    </span>
                    <span className="font-mono text-sm text-bosch-gray-400">{alarm.code}</span>
                  </div>
                  <span className="text-xs text-bosch-gray-500">{alarm.timestamp}</span>
                </div>
                <p className="mt-2">{alarm.message}</p>
                <div className="mt-1 text-xs text-bosch-gray-500">
                  来源: {alarm.source}
                  {alarm.acknowledged && (
                    <span className="ml-4">已确认: {alarm.acknowledged_at}</span>
                  )}
                </div>
              </div>
            ))}
          </div>
        )}
      </div>
    </div>
  );
}
