/**
 * 状态指示器组件
 */

import clsx from 'clsx';

interface StatusIndicatorProps {
  status: 'ok' | 'warning' | 'error' | 'offline';
  label?: string;
  size?: 'sm' | 'md' | 'lg';
  pulse?: boolean;
}

export function StatusIndicator({
  status,
  label,
  size = 'md',
  pulse = false,
}: StatusIndicatorProps) {
  const sizeClasses = {
    sm: 'w-2 h-2',
    md: 'w-3 h-3',
    lg: 'w-4 h-4',
  };

  const statusClasses = {
    ok: 'bg-status-ok',
    warning: 'bg-status-warning',
    error: 'bg-status-error',
    offline: 'bg-bosch-gray-500',
  };

  return (
    <div className="flex items-center gap-2">
      <div className="relative">
        <div
          className={clsx(
            'rounded-full',
            sizeClasses[size],
            statusClasses[status]
          )}
        />
        {pulse && status !== 'offline' && (
          <div
            className={clsx(
              'absolute inset-0 rounded-full animate-ping opacity-75',
              statusClasses[status]
            )}
          />
        )}
      </div>
      {label && (
        <span className="text-sm text-bosch-gray-300">{label}</span>
      )}
    </div>
  );
}
