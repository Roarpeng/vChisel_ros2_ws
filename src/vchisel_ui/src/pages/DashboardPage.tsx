/**
 * 仪表盘页面
 */

import { useAppStore } from '@/store/appStore';
import { StatusIndicator, ImageViewer } from '@/components';
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, ResponsiveContainer } from 'recharts';

// 模拟数据 - 实际应用中从后端获取
const mockChartData = [
  { time: '08:00', points: 120 },
  { time: '09:00', points: 145 },
  { time: '10:00', points: 132 },
  { time: '11:00', points: 168 },
  { time: '12:00', points: 89 },
  { time: '13:00', points: 156 },
  { time: '14:00', points: 178 },
];

export function DashboardPage() {
  const { systemStatus, alarmStats } = useAppStore();

  return (
    <div className="p-6 space-y-6">
      <h1 className="text-2xl font-bold">系统仪表盘</h1>

      {/* 状态卡片 */}
      <div className="grid grid-cols-4 gap-4">
        {/* PLC状态 */}
        <div className="card">
          <div className="flex items-center justify-between mb-2">
            <span className="text-bosch-gray-400 text-sm">PLC状态</span>
            <StatusIndicator
              status={systemStatus?.plc_connected ? 'ok' : 'error'}
              size="lg"
            />
          </div>
          <div className="text-xl font-bold">
            {systemStatus?.plc_connected ? '已连接' : '断开'}
          </div>
        </div>

        {/* 相机状态 */}
        <div className="card">
          <div className="flex items-center justify-between mb-2">
            <span className="text-bosch-gray-400 text-sm">相机状态</span>
            <StatusIndicator
              status={systemStatus?.camera_connected ? 'ok' : 'error'}
              size="lg"
            />
          </div>
          <div className="text-xl font-bold">
            {systemStatus?.camera_connected ? '已连接' : '断开'}
          </div>
        </div>

        {/* 已处理点数 */}
        <div className="card">
          <div className="text-bosch-gray-400 text-sm mb-2">今日处理</div>
          <div className="text-3xl font-bold text-bosch-red">
            {systemStatus?.points_processed || 0}
          </div>
          <div className="text-sm text-bosch-gray-400">凿击点</div>
        </div>

        {/* 完成周期 */}
        <div className="card">
          <div className="text-bosch-gray-400 text-sm mb-2">完成周期</div>
          <div className="text-3xl font-bold text-status-ok">
            {systemStatus?.cycles_completed || 0}
          </div>
          <div className="text-sm text-bosch-gray-400">个</div>
        </div>
      </div>

      {/* 图表和报警 */}
      <div className="grid grid-cols-3 gap-4">
        {/* 处理趋势图 */}
        <div className="card col-span-2">
          <h3 className="text-lg font-medium mb-4">处理趋势</h3>
          <div className="h-64">
            <ResponsiveContainer width="100%" height="100%">
              <LineChart data={mockChartData}>
                <CartesianGrid strokeDasharray="3 3" stroke="#424242" />
                <XAxis dataKey="time" stroke="#9E9E9E" />
                <YAxis stroke="#9E9E9E" />
                <Tooltip
                  contentStyle={{
                    backgroundColor: '#303030',
                    border: 'none',
                    borderRadius: '4px',
                  }}
                />
                <Line
                  type="monotone"
                  dataKey="points"
                  stroke="#E20015"
                  strokeWidth={2}
                  dot={{ fill: '#E20015' }}
                />
              </LineChart>
            </ResponsiveContainer>
          </div>
        </div>

        {/* 报警摘要 */}
        <div className="card">
          <h3 className="text-lg font-medium mb-4">报警统计</h3>
          <div className="space-y-4">
            <div className="flex items-center justify-between">
              <span className="text-bosch-gray-400">活跃报警</span>
              <span className="text-xl font-bold text-status-error">
                {alarmStats?.active_alarms || 0}
              </span>
            </div>
            <div className="flex items-center justify-between">
              <span className="text-bosch-gray-400">严重</span>
              <span className="text-lg font-medium text-status-error">
                {alarmStats?.critical_count || 0}
              </span>
            </div>
            <div className="flex items-center justify-between">
              <span className="text-bosch-gray-400">错误</span>
              <span className="text-lg font-medium text-status-warning">
                {alarmStats?.error_count || 0}
              </span>
            </div>
            <div className="flex items-center justify-between">
              <span className="text-bosch-gray-400">警告</span>
              <span className="text-lg font-medium text-yellow-500">
                {alarmStats?.warning_count || 0}
              </span>
            </div>
            <div className="flex items-center justify-between">
              <span className="text-bosch-gray-400">信息</span>
              <span className="text-lg font-medium text-status-info">
                {alarmStats?.info_count || 0}
              </span>
            </div>
          </div>
        </div>
      </div>

      {/* 标定状态 */}
      {systemStatus?.calib_mode_active && (
        <div className="card border-l-4 border-status-warning">
          <h3 className="text-lg font-medium mb-2">标定模式</h3>
          <div className="flex items-center gap-6">
            <div>
              <span className="text-bosch-gray-400 text-sm">采集点数</span>
              <div className="text-xl font-bold">{systemStatus.calib_points_captured}</div>
            </div>
            <div>
              <span className="text-bosch-gray-400 text-sm">质量评分</span>
              <div className="text-xl font-bold">{systemStatus.calib_quality_score}%</div>
            </div>
          </div>
        </div>
      )}

      {/* 最近错误 */}
      {systemStatus?.last_error && (
        <div className="card border-l-4 border-status-error">
          <h3 className="text-lg font-medium mb-2">最近错误</h3>
          <p className="text-status-error">{systemStatus.last_error}</p>
          <p className="text-sm text-bosch-gray-400 mt-1">{systemStatus.last_error_time}</p>
        </div>
      )}

      {/* 视觉图像显示 */}
      <div className="grid grid-cols-2 gap-4">
        <ImageViewer
          imageType="camera"
          title="相机图像"
          showInfo={true}
        />
        <ImageViewer
          imageType="annotated"
          title="法向量计算结果"
          showInfo={true}
        />
      </div>
    </div>
  );
}
