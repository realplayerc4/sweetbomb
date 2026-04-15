/**
 * WebRTC API 服务
 * 从 SW 前端复制，提供设备管理和 WebRTC 连接功能
 */

import { API_BASE } from '@/config';

export interface DeviceInfo {
  device_id: string;
  name: string;
  is_streaming: boolean;
}

export interface StreamConfig {
  sensor_id: string;
  stream_type: string;
  format: string;
  resolution: { width: number; height: number };
  framerate: number;
  enable: boolean;
}

export interface StreamRequest {
  configs: StreamConfig[];
  align_to: string;
}

export const webrtcApi = {
  /** 通用 GET 请求 */
  async get(path: string): Promise<any> {
    const res = await fetch(`${API_BASE}${path}`);
    if (!res.ok) throw new Error(`GET ${path} failed: ${res.statusText}`);
    return res.json();
  },

  /** 获取所有设备列表 */
  async getDevices(): Promise<DeviceInfo[]> {
    const res = await fetch(`${API_BASE}/devices/`);
    if (!res.ok) throw new Error('Failed to fetch devices');
    return res.json();
  },

  /** 启动设备流 */
  async startStream(deviceId: string, config: StreamRequest): Promise<void> {
    const res = await fetch(`${API_BASE}/devices/${deviceId}/stream/start/`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(config),
    });
    if (!res.ok) throw new Error('Failed to start stream');
  },

  /** 停止设备流 */
  async stopStream(deviceId: string): Promise<void> {
    const res = await fetch(`${API_BASE}/devices/${deviceId}/stream/stop/`, {
      method: 'POST',
    });
    if (!res.ok) throw new Error('Failed to stop stream');
  },

  /** 创建 WebRTC offer */
  async getWebRTCOffer(deviceId: string, streamTypes: string[]): Promise<any> {
    const res = await fetch(`${API_BASE}/webrtc/offer`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ device_id: deviceId, stream_types: streamTypes }),
    });
    if (!res.ok) throw new Error('Failed to get WebRTC offer');
    return res.json();
  },

  /** 发送 ICE 候选 */
  async sendIceCandidate(sessionId: string, candidate: RTCIceCandidate): Promise<void> {
    await fetch(`${API_BASE}/webrtc/ice-candidates`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        session_id: sessionId,
        candidate: candidate.candidate,
        sdpMid: candidate.sdpMid,
        sdpMLineIndex: candidate.sdpMLineIndex,
      }),
    });
  },

  /** 发送 WebRTC answer */
  async sendAnswer(sessionId: string, answer: RTCSessionDescriptionInit): Promise<void> {
    await fetch(`${API_BASE}/webrtc/answer/`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        session_id: sessionId,
        sdp: answer.sdp,
        type: answer.type,
      }),
    });
  },

  /** 获取 ICE 候选列表 */
  async getIceCandidates(sessionId: string): Promise<RTCIceCandidateInit[]> {
    const res = await fetch(`${API_BASE}/webrtc/sessions/${sessionId}/ice-candidates/`);
    if (!res.ok) throw new Error('Failed to get ICE candidates');
    return res.json();
  },

  /** 激活点云 */
  async activatePointCloud(deviceId: string): Promise<void> {
    const res = await fetch(`${API_BASE}/devices/${deviceId}/point_cloud/activate/`, {
      method: 'POST',
    });
    if (!res.ok) throw new Error('Failed to activate point cloud');
  },

  /** 关闭点云 */
  async deactivatePointCloud(deviceId: string): Promise<void> {
    const res = await fetch(`${API_BASE}/devices/${deviceId}/point_cloud/deactivate/`, {
      method: 'POST',
    });
    if (!res.ok) throw new Error('Failed to deactivate point cloud');
  }
};
