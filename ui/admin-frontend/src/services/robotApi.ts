import apiClient from './apiClient';
import type {
  RobotStatus,
  MoveDirection,
  SugarHarvestConfig,
  SugarHarvestStatus,
} from '../types';

// 后端返回的机器人状态
interface RobotStatusResponse {
  connected: boolean;
  mode: string;
  status: string;
  charge: number;
  speed: number;
  fault: string;
  fault_level: string;
  task_id: string;
  station: string;
  map_name: string;
  x: number;
  y: number;
  z: number;
  a: number;
  boom: number;
  bucket: number;
  last_update: string | null;
}

interface SimpleTaskResponse {
  success: boolean;
  task_id: string;
  message: string;
}

interface StopResponse {
  success: boolean;
  message: string;
}

export const robotApi = {
  async getStatus(): Promise<RobotStatusResponse> {
    const res = await apiClient.get<RobotStatusResponse>('/robot/status');
    return res.data;
  },

  async move(direction: MoveDirection, speed = 0.5, duration = 1.0): Promise<{ success: boolean; message: string }> {
    const res = await apiClient.post('/robot/move', { direction, speed, duration });
    return res.data;
  },

  async stop(): Promise<StopResponse> {
    const res = await apiClient.post<StopResponse>('/robot/stop');
    return res.data;
  },

  async reset(): Promise<StopResponse> {
    const res = await apiClient.post<StopResponse>('/robot/reset');
    return res.data;
  },

  async pause(): Promise<StopResponse> {
    const res = await apiClient.post<StopResponse>('/robot/pause');
    return res.data;
  },

  async resume(): Promise<StopResponse> {
    const res = await apiClient.post<StopResponse>('/robot/resume');
    return res.data;
  },

  async setServo(servoId: string, angle: number): Promise<{ success: boolean; message: string }> {
    const res = await apiClient.post('/robot/servo', { servo_id: servoId, angle });
    return res.data;
  },

  async scoop(): Promise<SimpleTaskResponse> {
    const res = await apiClient.post<SimpleTaskResponse>('/robot/scoop');
    return res.data;
  },

  async dump(): Promise<SimpleTaskResponse> {
    const res = await apiClient.post<SimpleTaskResponse>('/robot/dump');
    return res.data;
  },

  async dock(): Promise<SimpleTaskResponse> {
    const res = await apiClient.post<SimpleTaskResponse>('/robot/dock');
    return res.data;
  },

  // 导航到取货点
  async navToPick(): Promise<SimpleTaskResponse> {
    const res = await apiClient.post<SimpleTaskResponse>('/robot/nav-pick');
    return res.data;
  },

  // 导航到卸货点
  async navToDrop(): Promise<SimpleTaskResponse> {
    const res = await apiClient.post<SimpleTaskResponse>('/robot/nav-drop');
    return res.data;
  },

  async startSugarHarvest(config: SugarHarvestConfig): Promise<{ success: boolean; message: string }> {
    const res = await apiClient.post('/robot/auto_cycle/start', { config });
    return res.data;
  },

  async stopSugarHarvest(): Promise<{ success: boolean; message: string }> {
    const res = await apiClient.post('/robot/auto_cycle/stop');
    return res.data;
  },

  async getSugarHarvestStatus(): Promise<SugarHarvestStatus> {
    const res = await apiClient.get<SugarHarvestStatus>('/robot/auto_cycle/status');
    return res.data;
  },
};
