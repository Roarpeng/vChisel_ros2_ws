/**
 * Electron Preload Script
 * 在渲染进程中暴露安全的 API
 */

import { contextBridge, ipcRenderer } from 'electron';

// 暴露给渲染进程的 API
contextBridge.exposeInMainWorld('electronAPI', {
  getAppVersion: () => ipcRenderer.invoke('get-app-version'),

  // 可以添加更多 API
  platform: process.platform,

  // 窗口控制（如果使用无边框窗口）
  // minimize: () => ipcRenderer.send('window-minimize'),
  // maximize: () => ipcRenderer.send('window-maximize'),
  // close: () => ipcRenderer.send('window-close'),
});

// 类型声明
declare global {
  interface Window {
    electronAPI: {
      getAppVersion: () => Promise<string>;
      platform: string;
    };
  }
}
