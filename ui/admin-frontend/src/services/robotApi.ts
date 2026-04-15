import apiClient from './apiClient';
import type {
  RobotStatus,
  MoveDirection,
  SugarHarvestConfig,
  SugarHarvestStatus,
} from '../types';

export const robotApi = {
  async getStatus(): Promise<RobotStatus> {
    const res = await apiClient.get<RobotStatus>('/robot/status');
    return res.data;
  },

  async move(direction: MoveDirection, speed = 0.5, duration = 1.0): Promise<{ success: boolean; message: string }> {
    const res = await apiClient.post('/robot/move', { direction, speed, duration });
    return res.data;
  },

  async stop(): Promise<{ success: boolean; message: string }> {
    const res = await apiClient.post('/robot/stop');
    return res.data;
  },

  async reset(): Promise<{ success: boolean; message: string }> {
    const res = await apiClient.post('/robot/reset');
    return res.data;
  },

  async setServo(servoId: string, angle: number): Promise<{ success: boolean; message: string }> {
    const res = await apiClient.post('/robot/servo', { servo_id: servoId, angle });
    return res.data;
  },

  async scoop(): Promise<{ success: boolean; message: string }> {
    const res = await apiClient.post('/robot/scoop');
    return res.data;
  },

  async dump(): Promise<{ success: boolean; message: string }> {
    const res = await apiClient.post('/robot/dump');
    return res.data;
  },

  async dock(): Promise<{ success: boolean; message: string }> {
    const res = await apiClient.post('/robot/dock');
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
