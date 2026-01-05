/**
 * 视觉页面
 * 显示相机图像和法向量计算结果
 */

import { useState } from 'react';
import { ImageViewer } from '@/components';
import clsx from 'clsx';

type ViewMode = 'split' | 'camera' | 'annotated';

export function VisionPage() {
  const [viewMode, setViewMode] = useState<ViewMode>('split');

  return (
    <div className="p-6 space-y-4 h-full flex flex-col">
      <div className="flex items-center justify-between">
        <h1 className="text-2xl font-bold">视觉监控</h1>

        {/* 视图模式切换 */}
        <div className="flex gap-2">
          <button
            onClick={() => setViewMode('split')}
            className={clsx(
              'px-4 py-2 rounded transition-colors',
              viewMode === 'split'
                ? 'bg-bosch-red text-white'
                : 'bg-bosch-gray-700 text-bosch-gray-300 hover:bg-bosch-gray-600'
            )}
          >
            分屏
          </button>
          <button
            onClick={() => setViewMode('camera')}
            className={clsx(
              'px-4 py-2 rounded transition-colors',
              viewMode === 'camera'
                ? 'bg-bosch-red text-white'
                : 'bg-bosch-gray-700 text-bosch-gray-300 hover:bg-bosch-gray-600'
            )}
          >
            相机
          </button>
          <button
            onClick={() => setViewMode('annotated')}
            className={clsx(
              'px-4 py-2 rounded transition-colors',
              viewMode === 'annotated'
                ? 'bg-bosch-red text-white'
                : 'bg-bosch-gray-700 text-bosch-gray-300 hover:bg-bosch-gray-600'
            )}
          >
            法向量
          </button>
        </div>
      </div>

      {/* 图像显示区域 */}
      <div className="flex-1 min-h-0">
        {viewMode === 'split' && (
          <div className="grid grid-cols-2 gap-4 h-full">
            <ImageViewer
              imageType="camera"
              title="相机实时图像"
              className="h-full"
              showInfo={true}
            />
            <ImageViewer
              imageType="annotated"
              title="法向量计算结果"
              className="h-full"
              showInfo={true}
            />
          </div>
        )}

        {viewMode === 'camera' && (
          <ImageViewer
            imageType="camera"
            title="相机实时图像"
            className="h-full"
            showInfo={true}
          />
        )}

        {viewMode === 'annotated' && (
          <ImageViewer
            imageType="annotated"
            title="法向量计算结果"
            className="h-full"
            showInfo={true}
          />
        )}
      </div>

      {/* 状态信息 */}
      <div className="card py-2">
        <div className="flex items-center justify-between text-sm">
          <div className="flex items-center gap-4">
            <span className="text-bosch-gray-400">图像话题:</span>
            <code className="text-xs bg-bosch-gray-700 px-2 py-1 rounded">
              /vchisel/ui/camera_image
            </code>
            <code className="text-xs bg-bosch-gray-700 px-2 py-1 rounded">
              /vchisel/ui/annotated_image
            </code>
          </div>
          <div className="flex items-center gap-2">
            <span className="text-bosch-gray-400">点击图像可全屏查看</span>
          </div>
        </div>
      </div>
    </div>
  );
}
