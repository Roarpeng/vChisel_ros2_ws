/**
 * 标定页面
 */

import { useState } from 'react';
import { useAppStore } from '@/store/appStore';
import { rosBridge } from '@/services/rosBridge';

export function CalibrationPage() {
  const { calibrationStatus, setCalibrationStatus } = useAppStore();
  const [loading, setLoading] = useState(false);
  const [message, setMessage] = useState('');

  const startIntrinsicCalib = async () => {
    setLoading(true);
    try {
      await rosBridge.callService('/calib/intrinsic/start', 'std_srvs/Trigger', {});
      setCalibrationStatus({ mode: 'intrinsic', is_running: true });
      setMessage('相机内参标定已启动');
    } catch (error) {
      setMessage(`启动失败: ${error}`);
    } finally {
      setLoading(false);
    }
  };

  const startHandeyeCalib = async () => {
    setLoading(true);
    try {
      await rosBridge.callService('/calib/handeye/start', 'std_srvs/Trigger', {});
      setCalibrationStatus({ mode: 'handeye', is_running: true });
      setMessage('手眼标定已启动');
    } catch (error) {
      setMessage(`启动失败: ${error}`);
    } finally {
      setLoading(false);
    }
  };

  const capturePoint = async () => {
    setLoading(true);
    try {
      const serviceName = calibrationStatus.mode === 'intrinsic'
        ? '/calib/intrinsic/capture'
        : '/calib/handeye/capture';
      const result = await rosBridge.callService<{ success: boolean; message: string }>(
        serviceName, 'std_srvs/Trigger', {}
      );
      if (result.success) {
        setCalibrationStatus({
          points_captured: calibrationStatus.points_captured + 1
        });
        setMessage('采集成功');
      } else {
        setMessage(`采集失败: ${result.message}`);
      }
    } catch (error) {
      setMessage(`采集失败: ${error}`);
    } finally {
      setLoading(false);
    }
  };

  const computeCalib = async () => {
    setLoading(true);
    try {
      const serviceName = calibrationStatus.mode === 'intrinsic'
        ? '/calib/intrinsic/compute'
        : '/calib/handeye/compute';
      const result = await rosBridge.callService<{ success: boolean; message: string }>(
        serviceName, 'std_srvs/Trigger', {}
      );
      setMessage(result.success ? '计算完成' : `计算失败: ${result.message}`);
    } catch (error) {
      setMessage(`计算失败: ${error}`);
    } finally {
      setLoading(false);
    }
  };

  const saveCalib = async () => {
    setLoading(true);
    try {
      const serviceName = calibrationStatus.mode === 'intrinsic'
        ? '/calib/intrinsic/save'
        : '/calib/handeye/save';
      const result = await rosBridge.callService<{ success: boolean; message: string }>(
        serviceName, 'std_srvs/Trigger', {}
      );
      setMessage(result.success ? '保存成功' : `保存失败: ${result.message}`);
    } catch (error) {
      setMessage(`保存失败: ${error}`);
    } finally {
      setLoading(false);
    }
  };

  const stopCalib = async () => {
    setLoading(true);
    try {
      const serviceName = calibrationStatus.mode === 'intrinsic'
        ? '/calib/intrinsic/stop'
        : '/calib/handeye/stop';
      await rosBridge.callService(serviceName, 'std_srvs/Trigger', {});
      setCalibrationStatus({ mode: 'idle', is_running: false, points_captured: 0 });
      setMessage('标定已停止');
    } catch (error) {
      setMessage(`停止失败: ${error}`);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="p-6 space-y-6">
      <h1 className="text-2xl font-bold">标定</h1>

      {/* 标定模式选择 */}
      {calibrationStatus.mode === 'idle' && (
        <div className="grid grid-cols-2 gap-6">
          <div className="card">
            <h3 className="text-lg font-medium mb-2">相机内参标定</h3>
            <p className="text-sm text-bosch-gray-400 mb-4">
              使用棋盘格标定板进行相机内参标定，包括焦距、畸变系数等参数。
            </p>
            <button
              onClick={startIntrinsicCalib}
              disabled={loading}
              className="btn-primary"
            >
              开始内参标定
            </button>
          </div>

          <div className="card">
            <h3 className="text-lg font-medium mb-2">手眼标定</h3>
            <p className="text-sm text-bosch-gray-400 mb-4">
              标定相机与机器人末端的相对位姿关系，使用ArUco标记进行自动检测。
            </p>
            <button
              onClick={startHandeyeCalib}
              disabled={loading}
              className="btn-primary"
            >
              开始手眼标定
            </button>
          </div>
        </div>
      )}

      {/* 标定进行中 */}
      {calibrationStatus.mode !== 'idle' && (
        <>
          <div className="card">
            <div className="flex items-center justify-between mb-4">
              <h3 className="text-lg font-medium">
                {calibrationStatus.mode === 'intrinsic' ? '相机内参标定' : '手眼标定'}
              </h3>
              <span className="px-3 py-1 bg-status-warning rounded text-sm">
                进行中
              </span>
            </div>

            {/* 进度 */}
            <div className="mb-6">
              <div className="flex justify-between text-sm mb-2">
                <span>采集进度</span>
                <span>{calibrationStatus.points_captured} / {calibrationStatus.min_points_required}</span>
              </div>
              <div className="w-full bg-bosch-gray-700 rounded-full h-3">
                <div
                  className="bg-bosch-red h-3 rounded-full transition-all"
                  style={{
                    width: `${Math.min(100, (calibrationStatus.points_captured / calibrationStatus.min_points_required) * 100)}%`
                  }}
                />
              </div>
            </div>

            {/* 控制按钮 */}
            <div className="flex gap-4">
              <button
                onClick={capturePoint}
                disabled={loading}
                className="btn-primary"
              >
                采集点
              </button>
              <button
                onClick={computeCalib}
                disabled={loading || calibrationStatus.points_captured < calibrationStatus.min_points_required}
                className="btn-secondary"
              >
                计算
              </button>
              <button
                onClick={saveCalib}
                disabled={loading}
                className="btn-success"
              >
                保存
              </button>
              <button
                onClick={stopCalib}
                disabled={loading}
                className="btn-danger"
              >
                停止
              </button>
            </div>
          </div>

          {/* 质量评分 */}
          {calibrationStatus.quality_score > 0 && (
            <div className="card">
              <h3 className="text-lg font-medium mb-2">标定质量</h3>
              <div className="text-4xl font-bold text-bosch-red">
                {calibrationStatus.quality_score}%
              </div>
            </div>
          )}
        </>
      )}

      {/* 消息 */}
      {message && (
        <div className="card bg-bosch-gray-700">
          <p className="text-sm">{message}</p>
        </div>
      )}
    </div>
  );
}
