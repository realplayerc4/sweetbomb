
import { API_BASE } from '../config';

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

export const api = {
    async getDevices(): Promise<DeviceInfo[]> {
        const res = await fetch(`${API_BASE}/devices`);
        if (!res.ok) throw new Error('Failed to fetch devices');
        return res.json();
    },

    async startStream(deviceId: string, config: StreamRequest): Promise<void> {
        const res = await fetch(`${API_BASE}/devices/${deviceId}/stream/start`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(config),
        });
        if (!res.ok) throw new Error('Failed to start stream');
    },

    async stopStream(deviceId: string): Promise<void> {
        const res = await fetch(`${API_BASE}/devices/${deviceId}/stream/stop`, {
            method: 'POST',
        });
        if (!res.ok) throw new Error('Failed to stop stream');
    },

    async getWebRTCOffer(deviceId: string, streamTypes: string[]): Promise<any> {
        const res = await fetch(`${API_BASE}/webrtc/offer`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ device_id: deviceId, stream_types: streamTypes }),
        });
        if (!res.ok) throw new Error('Failed to get WebRTC offer');
        return res.json();
    },

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

    async sendAnswer(sessionId: string, answer: RTCSessionDescriptionInit): Promise<void> {
        await fetch(`${API_BASE}/webrtc/answer`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                session_id: sessionId,
                sdp: answer.sdp,
                type: answer.type,
            }),
        });
    },

    async getIceCandidates(sessionId: string): Promise<RTCIceCandidateInit[]> {
        const res = await fetch(`${API_BASE}/webrtc/sessions/${sessionId}/ice-candidates`);
        if (!res.ok) throw new Error('Failed to get ICE candidates');
        return res.json();
    },

    async activatePointCloud(deviceId: string): Promise<void> {
        const res = await fetch(`${API_BASE}/devices/${deviceId}/point_cloud/activate`, {
            method: 'POST'
        });
        if (!res.ok) throw new Error('Failed to activate point cloud');
    },

    async deactivatePointCloud(deviceId: string): Promise<void> {
        const res = await fetch(`${API_BASE}/devices/${deviceId}/point_cloud/deactivate`, {
            method: 'POST'
        });
        if (!res.ok) throw new Error('Failed to deactivate point cloud');
    }
};
