/**
 * 应用主布局
 */

import { Sidebar, Header } from '@/components';
import {
  DashboardPage,
  OperationPage,
  CalibrationPage,
  RecipesPage,
  AlarmsPage,
  SettingsPage,
  VisionPage
} from '@/pages';
import { useAppStore } from '@/store/appStore';
import { useRosConnection } from '@/hooks/useRosConnection';

export function App() {
  const { currentPage } = useAppStore();

  // 初始化ROS连接
  useRosConnection();

  const renderPage = () => {
    switch (currentPage) {
      case 'dashboard':
        return <DashboardPage />;
      case 'operation':
        return <OperationPage />;
      case 'vision':
        return <VisionPage />;
      case 'calibration':
        return <CalibrationPage />;
      case 'recipes':
        return <RecipesPage />;
      case 'alarms':
        return <AlarmsPage />;
      case 'settings':
        return <SettingsPage />;
      default:
        return <DashboardPage />;
    }
  };

  return (
    <div className="h-screen flex bg-bosch-gray-900">
      {/* 侧边栏 */}
      <Sidebar />

      {/* 主内容区 */}
      <div className="flex-1 flex flex-col overflow-hidden">
        {/* 顶部状态栏 */}
        <Header />

        {/* 页面内容 */}
        <main className="flex-1 overflow-auto">
          {renderPage()}
        </main>
      </div>
    </div>
  );
}

export default App;
