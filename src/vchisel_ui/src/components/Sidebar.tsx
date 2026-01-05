/**
 * 侧边栏导航组件
 */

import clsx from 'clsx';
import { useAppStore } from '@/store/appStore';
import {
  HomeIcon,
  CogIcon,
  WrenchScrewdriverIcon,
  BeakerIcon,
  ExclamationTriangleIcon,
  DocumentTextIcon,
  ChevronLeftIcon,
  ChevronRightIcon,
  CameraIcon,
} from '@heroicons/react/24/outline';

interface NavItem {
  id: string;
  label: string;
  icon: React.ComponentType<{ className?: string }>;
}

const navItems: NavItem[] = [
  { id: 'dashboard', label: '仪表盘', icon: HomeIcon },
  { id: 'operation', label: '运行控制', icon: WrenchScrewdriverIcon },
  { id: 'vision', label: '视觉监控', icon: CameraIcon },
  { id: 'calibration', label: '标定', icon: BeakerIcon },
  { id: 'recipes', label: '配方管理', icon: DocumentTextIcon },
  { id: 'alarms', label: '报警', icon: ExclamationTriangleIcon },
  { id: 'settings', label: '设置', icon: CogIcon },
];

export function Sidebar() {
  const { sidebarOpen, toggleSidebar, currentPage, setCurrentPage } = useAppStore();

  return (
    <div
      className={clsx(
        'bg-bosch-gray-800 h-full flex flex-col transition-all duration-300',
        sidebarOpen ? 'w-56' : 'w-16'
      )}
    >
      {/* Logo */}
      <div className="h-16 flex items-center justify-center border-b border-bosch-gray-700">
        {sidebarOpen ? (
          <span className="text-xl font-bold text-bosch-red">vChisel</span>
        ) : (
          <span className="text-xl font-bold text-bosch-red">vC</span>
        )}
      </div>

      {/* 导航项 */}
      <nav className="flex-1 py-4">
        {navItems.map((item) => {
          const Icon = item.icon;
          const isActive = currentPage === item.id;

          return (
            <button
              key={item.id}
              onClick={() => setCurrentPage(item.id)}
              className={clsx(
                'w-full flex items-center gap-3 px-4 py-3 transition-colors',
                isActive
                  ? 'bg-bosch-red text-white'
                  : 'text-bosch-gray-300 hover:bg-bosch-gray-700 hover:text-white'
              )}
            >
              <Icon className="w-6 h-6 flex-shrink-0" />
              {sidebarOpen && (
                <span className="text-sm font-medium">{item.label}</span>
              )}
            </button>
          );
        })}
      </nav>

      {/* 折叠按钮 */}
      <button
        onClick={toggleSidebar}
        className="h-12 flex items-center justify-center border-t border-bosch-gray-700 text-bosch-gray-400 hover:text-white transition-colors"
      >
        {sidebarOpen ? (
          <ChevronLeftIcon className="w-5 h-5" />
        ) : (
          <ChevronRightIcon className="w-5 h-5" />
        )}
      </button>
    </div>
  );
}
