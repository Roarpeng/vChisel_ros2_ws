/**
 * 图像查看器组件
 * 用于显示ROS图像话题中的图像
 */

import { useState, useEffect, useCallback } from 'react';
import { rosBridge, type ImageData } from '@/services/rosBridge';
import clsx from 'clsx';

interface ImageViewerProps {
  /** 图像类型: 'camera' 或 'annotated' */
  imageType: 'camera' | 'annotated';
  /** 标题 */
  title: string;
  /** 附加CSS类名 */
  className?: string;
  /** 是否显示信息覆盖层 */
  showInfo?: boolean;
}

export function ImageViewer({
  imageType,
  title,
  className,
  showInfo = true
}: ImageViewerProps) {
  const [imageData, setImageData] = useState<ImageData | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [isFullscreen, setIsFullscreen] = useState(false);

  const handleImageData = useCallback((data: unknown) => {
    try {
      setImageData(data as ImageData);
      setError(null);
    } catch (e) {
      setError(`图像解析失败: ${e}`);
    }
  }, []);

  useEffect(() => {
    const eventName = imageType === 'camera' ? 'camera_image' : 'annotated_image';
    const unsubscribe = rosBridge.on(eventName, handleImageData);

    return () => {
      unsubscribe();
    };
  }, [imageType, handleImageData]);

  const toggleFullscreen = () => {
    setIsFullscreen(!isFullscreen);
  };

  const renderContent = () => {
    if (error) {
      return (
        <div className="flex items-center justify-center h-full text-status-error">
          <span>{error}</span>
        </div>
      );
    }

    if (!imageData) {
      return (
        <div className="flex items-center justify-center h-full text-bosch-gray-400">
          <div className="text-center">
            <div className="animate-pulse mb-2">
              <svg className="w-12 h-12 mx-auto" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M4 16l4.586-4.586a2 2 0 012.828 0L16 16m-2-2l1.586-1.586a2 2 0 012.828 0L20 14m-6-6h.01M6 20h12a2 2 0 002-2V6a2 2 0 00-2-2H6a2 2 0 00-2 2v12a2 2 0 002 2z" />
              </svg>
            </div>
            <p>等待图像数据...</p>
          </div>
        </div>
      );
    }

    return (
      <div className="relative w-full h-full">
        <img
          src={`data:image/jpeg;base64,${imageData.data}`}
          alt={title}
          className="w-full h-full object-contain"
          onClick={toggleFullscreen}
        />
        {showInfo && (
          <div className="absolute bottom-0 left-0 right-0 bg-black/50 px-2 py-1 text-xs text-white">
            <div className="flex justify-between">
              <span>{imageData.width} × {imageData.height}</span>
              {imageData.normal_count !== undefined && (
                <span>法向量: {imageData.normal_count}</span>
              )}
              <span>{new Date(imageData.timestamp).toLocaleTimeString()}</span>
            </div>
          </div>
        )}
      </div>
    );
  };

  // 全屏模式
  if (isFullscreen) {
    return (
      <div
        className="fixed inset-0 z-50 bg-black flex items-center justify-center"
        onClick={toggleFullscreen}
      >
        <button
          className="absolute top-4 right-4 text-white hover:text-gray-300"
          onClick={toggleFullscreen}
        >
          <svg className="w-8 h-8" fill="none" stroke="currentColor" viewBox="0 0 24 24">
            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
          </svg>
        </button>
        {imageData && (
          <img
            src={`data:image/jpeg;base64,${imageData.data}`}
            alt={title}
            className="max-w-full max-h-full object-contain"
          />
        )}
      </div>
    );
  }

  return (
    <div className={clsx('card', className)}>
      <div className="flex items-center justify-between mb-2">
        <h3 className="text-lg font-medium">{title}</h3>
        <button
          onClick={toggleFullscreen}
          className="p-1 hover:bg-bosch-gray-700 rounded"
          title="全屏查看"
        >
          <svg className="w-5 h-5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M4 8V4m0 0h4M4 4l5 5m11-1V4m0 0h-4m4 0l-5 5M4 16v4m0 0h4m-4 0l5-5m11 5l-5-5m5 5v-4m0 4h-4" />
          </svg>
        </button>
      </div>
      <div className="bg-bosch-gray-900 rounded-lg overflow-hidden aspect-video">
        {renderContent()}
      </div>
    </div>
  );
}
