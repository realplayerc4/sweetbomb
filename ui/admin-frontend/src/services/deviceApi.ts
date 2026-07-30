import apiClient from './apiClient';
import type { DeviceInfo } from '../types';

export const deviceApi = {
  async getDevices(): Promise<DeviceInfo[]> {
    const res = await apiClient.get<DeviceInfo[]>('/devices/');
    return res.data;
  },

  async getDevice(deviceId: string): Promise<DeviceInfo> {
    const res = await apiClient.get<DeviceInfo>(`/devices/${deviceId}`);
    return res.data;
  },
};
