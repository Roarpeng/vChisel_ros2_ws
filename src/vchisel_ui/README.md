# vChisel UI

视觉凿击系统控制界面 - Electron + React + TypeScript

## 开发环境

### 前置条件

- Node.js >= 18
- npm >= 9

### 安装依赖

```bash
cd src/vchisel_ui
npm install
```

### 开发模式

```bash
# 仅启动 Web 开发服务器
npm run dev

# 启动 Electron 开发模式
npm run electron:dev
```

### 构建

```bash
# 构建 Web 应用
npm run build

# 构建 Electron 应用
npm run electron:build
```

## 功能模块

1. **仪表盘** - 系统状态概览、处理统计、报警摘要
2. **运行控制** - 启动/停止操作、手动控制
3. **标定** - 相机内参标定、手眼标定
4. **配方管理** - 配方CRUD、参数调整
5. **报警** - 报警历史、确认、过滤
6. **设置** - 系统配置、连接设置

## 与 ROS2 通信

通过 rosbridge_server WebSocket 连接，默认端口 9090。

启动 ROS2 后端：
```bash
ros2 launch vchisel_bringup full_system.launch.py
```

## 项目结构

```
src/
├── components/      # 可复用组件
├── pages/           # 页面组件
├── services/        # 服务层（ROS通信）
├── store/           # 状态管理
├── hooks/           # 自定义 Hooks
├── types/           # TypeScript 类型定义
└── assets/          # 静态资源

electron/
├── main.ts          # Electron 主进程
└── preload.ts       # 预加载脚本
```
