/**
 * ROS Bridge Service
 * 处理与rosbridge_server的WebSocket通信
 */

import ROSLIB from 'roslib';
import type { SystemStatus, Recipe, RecipeInfo } from '@/types';

// ROS消息类型
interface StringMessage {
  data: string;
}

// 图像数据类型
export interface ImageData {
  type: string;
  data: string;  // base64编码的图像
  timestamp: string;
  width: number;
  height: number;
  normal_count?: number;
}

class RosBridgeService {
  private ros: ROSLIB.Ros | null = null;
  private reconnectInterval: number = 3000;
  private reconnectTimer: ReturnType<typeof setInterval> | null = null;
  private listeners: Map<string, Set<(data: unknown) => void>> = new Map();

  // 话题
  private systemStatusTopic: ROSLIB.Topic | null = null;
  private alarmsTopic: ROSLIB.Topic | null = null;
  private alarmStatsTopic: ROSLIB.Topic | null = null;
  private cameraImageTopic: ROSLIB.Topic | null = null;
  private annotatedImageTopic: ROSLIB.Topic | null = null;

  constructor() {
    this.connect();
  }

  /**
   * 连接到rosbridge
   */
  connect(url: string = 'ws://localhost:9090'): void {
    if (this.ros) {
      this.ros.close();
    }

    this.ros = new ROSLIB.Ros({ url });

    this.ros.on('connection', () => {
      console.log('Connected to rosbridge');
      this.emit('connection', { connected: true, url });
      this.setupTopics();
      if (this.reconnectTimer) {
        clearInterval(this.reconnectTimer);
        this.reconnectTimer = null;
      }
    });

    this.ros.on('error', (error) => {
      console.error('rosbridge error:', error);
      this.emit('connection', { connected: false, url, error: String(error) });
    });

    this.ros.on('close', () => {
      console.log('Connection to rosbridge closed');
      this.emit('connection', { connected: false, url });
      this.scheduleReconnect(url);
    });
  }

  /**
   * 安排重连
   */
  private scheduleReconnect(url: string): void {
    if (!this.reconnectTimer) {
      this.reconnectTimer = setInterval(() => {
        console.log('Attempting to reconnect...');
        this.connect(url);
      }, this.reconnectInterval);
    }
  }

  /**
   * 设置话题订阅
   */
  private setupTopics(): void {
    if (!this.ros) return;

    // 系统状态话题
    this.systemStatusTopic = new ROSLIB.Topic({
      ros: this.ros,
      name: '/vchisel/system_status',
      messageType: 'std_msgs/String',
    });

    this.systemStatusTopic.subscribe((msg: ROSLIB.Message) => {
      try {
        const stringMsg = msg as unknown as StringMessage;
        const status = this.parsePythonDict(stringMsg.data);
        this.emit('system_status', status);
      } catch (e) {
        console.error('Failed to parse system status:', e);
      }
    });

    // 报警话题
    this.alarmsTopic = new ROSLIB.Topic({
      ros: this.ros,
      name: '/vchisel/alarms',
      messageType: 'std_msgs/String',
    });

    this.alarmsTopic.subscribe((msg: ROSLIB.Message) => {
      try {
        const stringMsg = msg as unknown as StringMessage;
        const alarm = this.parsePythonDict(stringMsg.data);
        this.emit('alarm', alarm);
      } catch (e) {
        console.error('Failed to parse alarm:', e);
      }
    });

    // 报警统计话题
    this.alarmStatsTopic = new ROSLIB.Topic({
      ros: this.ros,
      name: '/vchisel/alarm_stats',
      messageType: 'std_msgs/String',
    });

    this.alarmStatsTopic.subscribe((msg: ROSLIB.Message) => {
      try {
        const stringMsg = msg as unknown as StringMessage;
        const stats = this.parsePythonDict(stringMsg.data);
        this.emit('alarm_stats', stats);
      } catch (e) {
        console.error('Failed to parse alarm stats:', e);
      }
    });

    // 相机图像话题
    this.cameraImageTopic = new ROSLIB.Topic({
      ros: this.ros,
      name: '/vchisel/ui/camera_image',
      messageType: 'std_msgs/String',
    });

    this.cameraImageTopic.subscribe((msg: ROSLIB.Message) => {
      try {
        const stringMsg = msg as unknown as StringMessage;
        const imageData = JSON.parse(stringMsg.data) as ImageData;
        this.emit('camera_image', imageData);
      } catch (e) {
        console.error('Failed to parse camera image:', e);
      }
    });

    // 带法向量标注的图像话题
    this.annotatedImageTopic = new ROSLIB.Topic({
      ros: this.ros,
      name: '/vchisel/ui/annotated_image',
      messageType: 'std_msgs/String',
    });

    this.annotatedImageTopic.subscribe((msg: ROSLIB.Message) => {
      try {
        const stringMsg = msg as unknown as StringMessage;
        const imageData = JSON.parse(stringMsg.data) as ImageData;
        this.emit('annotated_image', imageData);
      } catch (e) {
        console.error('Failed to parse annotated image:', e);
      }
    });
  }

  /**
   * 解析Python dict字符串为JSON对象
   */
  private parsePythonDict(pyStr: string): Record<string, unknown> {
    // 将Python布尔值转换为JSON布尔值
    let jsonStr = pyStr
      .replace(/True/g, 'true')
      .replace(/False/g, 'false')
      .replace(/None/g, 'null')
      .replace(/'/g, '"');
    return JSON.parse(jsonStr);
  }

  /**
   * 调用服务
   */
  async callService<T>(
    serviceName: string,
    serviceType: string,
    request: Record<string, unknown> = {}
  ): Promise<T> {
    return new Promise((resolve, reject) => {
      if (!this.ros) {
        reject(new Error('Not connected to rosbridge'));
        return;
      }

      const service = new ROSLIB.Service({
        ros: this.ros,
        name: serviceName,
        serviceType,
      });

      const serviceRequest = new ROSLIB.ServiceRequest(request);

      service.callService(serviceRequest, (response) => {
        resolve(response as T);
      }, (error) => {
        reject(new Error(error));
      });
    });
  }

  /**
   * 获取系统状态
   */
  async getSystemStatus(): Promise<SystemStatus> {
    const response = await this.callService<{ success: boolean; message: string }>(
      '/vchisel/get_status',
      'std_srvs/Trigger'
    );
    if (response.success) {
      return this.parsePythonDict(response.message) as unknown as SystemStatus;
    }
    throw new Error('Failed to get system status');
  }

  /**
   * 清除报警
   */
  async clearAlarms(): Promise<void> {
    await this.callService('/vchisel/clear_alarms', 'std_srvs/Trigger');
  }

  /**
   * 确认报警
   */
  async acknowledgeAlarms(): Promise<void> {
    await this.callService('/alarm/acknowledge', 'std_srvs/Trigger');
  }

  /**
   * 获取配方列表
   */
  async listRecipes(): Promise<RecipeInfo[]> {
    const response = await this.callService<{ recipes: RecipeInfo[] }>(
      '/recipe/list',
      'recipe_manager/ListRecipes'
    );
    return response.recipes;
  }

  /**
   * 加载配方
   */
  async loadRecipe(name: string): Promise<Recipe> {
    const response = await this.callService<{ success: boolean; recipe: Recipe; message: string }>(
      '/recipe/load',
      'recipe_manager/LoadRecipe',
      { name }
    );
    if (response.success) {
      return response.recipe;
    }
    throw new Error(response.message);
  }

  /**
   * 保存配方
   */
  async saveRecipe(recipe: Recipe): Promise<void> {
    const response = await this.callService<{ success: boolean; message: string }>(
      '/recipe/save',
      'recipe_manager/SaveRecipe',
      { recipe }
    );
    if (!response.success) {
      throw new Error(response.message);
    }
  }

  /**
   * 删除配方
   */
  async deleteRecipe(name: string): Promise<void> {
    const response = await this.callService<{ success: boolean; message: string }>(
      '/recipe/delete',
      'recipe_manager/DeleteRecipe',
      { name }
    );
    if (!response.success) {
      throw new Error(response.message);
    }
  }

  /**
   * 事件监听
   */
  on(event: string, callback: (data: unknown) => void): () => void {
    if (!this.listeners.has(event)) {
      this.listeners.set(event, new Set());
    }
    this.listeners.get(event)!.add(callback);

    // 返回取消订阅函数
    return () => {
      this.listeners.get(event)?.delete(callback);
    };
  }

  /**
   * 触发事件
   */
  private emit(event: string, data: unknown): void {
    this.listeners.get(event)?.forEach((callback) => callback(data));
  }

  /**
   * 断开连接
   */
  disconnect(): void {
    if (this.reconnectTimer) {
      clearInterval(this.reconnectTimer);
      this.reconnectTimer = null;
    }
    if (this.ros) {
      this.ros.close();
      this.ros = null;
    }
  }

  /**
   * 检查连接状态
   */
  isConnected(): boolean {
    return this.ros?.isConnected ?? false;
  }
}

// 单例导出
export const rosBridge = new RosBridgeService();
export default rosBridge;
