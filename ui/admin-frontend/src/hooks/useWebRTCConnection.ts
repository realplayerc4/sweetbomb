/**
 * WebRTC 连接 Hook
 * 从 SW 前端复制，管理 WebRTC 视频流连接
 */

import { useState, useEffect, useRef, useCallback } from 'react';
import type { DeviceInfo } from '@/services/webrtcApi';
import { webrtcApi } from '@/services/webrtcApi';

export function useWebRTCConnection() {
  const [device, setDevice] = useState<DeviceInfo | null>(null);
  const [isStreaming, setIsStreaming] = useState(false);
  const [rgbStream, setRgbStream] = useState<MediaStream | null>(null);
  const [depthStream, setDepthStream] = useState<MediaStream | null>(null);
  const [streamMetrics, setStreamMetrics] = useState<{
    rgb: { width: number; height: number; fps: number } | null;
    depth: { width: number; height: number; fps: number } | null;
  }>({ rgb: null, depth: null });
  const [error, setError] = useState<string | null>(null);

  const peerConnection = useRef<RTCPeerConnection | null>(null);
  const sessionId = useRef<string | null>(null);
  const autoStartAttempted = useRef(false);

  // Auto-connect to first available device
  useEffect(() => {
    webrtcApi.getDevices().then(devices => {
      if (devices.length > 0) {
        setDevice(devices[0]);
      }
    }).catch(err => console.error("Failed to fetch devices:", err));
  }, []);

  const startConnection = useCallback(async () => {
    if (!device) return;
    setError(null);

    try {
      // 1. Start Streams on Backend
      await webrtcApi.startStream(device.device_id, {
        configs: [
          {
            sensor_id: `${device.device_id}-sensor-0`,
            stream_type: "depth",
            format: "z16",
            resolution: { width: 640, height: 360 },
            framerate: 15,
            enable: true
          },
          {
            sensor_id: `${device.device_id}-sensor-1`,
            stream_type: "color",
            format: "rgb8",
            resolution: { width: 640, height: 360 },
            framerate: 30,
            enable: true
          }
        ],
        align_to: "color"
      });

      // 2. WebRTC Offer (with retry logic)
      let offerData: any;
      let retryCount = 0;
      const maxRetries = 5;
      const retryDelay = 500;

      while (retryCount < maxRetries) {
        try {
          offerData = await webrtcApi.getWebRTCOffer(device.device_id, ['color', 'depth']);
          console.log(`[WebRTC] Got offer after ${retryCount + 1} attempt(s)`);
          break;
        } catch (error: any) {
          retryCount++;
          if (retryCount >= maxRetries) {
            console.error(`[WebRTC] Failed to get offer after ${maxRetries} attempts`);
            throw error;
          }
          console.warn(`[WebRTC] Attempt ${retryCount} failed, retrying in ${retryDelay}ms...`);
          await new Promise(resolve => setTimeout(resolve, retryDelay));
        }
      }
      sessionId.current = offerData!.session_id;

      // 3. Create Peer Connection
      const pc = new RTCPeerConnection({
        iceServers: [{ urls: 'stun:stun.l.google.com:19302' }]
      });
      peerConnection.current = pc;

      // 4. Handle Tracks
      let trackIndex = 0;
      pc.ontrack = (event) => {
        const mid = event.transceiver.mid;
        console.log(`[WebRTC] ontrack: mid=${mid}, track=${event.track.kind}, stream_map=`, offerData.stream_map);

        let streamType: string | undefined;

        // 优先通过 stream_map 匹配
        if (mid && offerData.stream_map) {
          streamType = offerData.stream_map[mid];
        }

        // Fallback：如果 mid 匹配失败，按请求顺序分配
        if (!streamType) {
          const requestedTypes = ['color', 'depth'];
          streamType = requestedTypes[trackIndex];
          console.warn(`[WebRTC] mid="${mid}" 未在 stream_map 中找到，fallback 使用: ${streamType}`);
        }

        const newStream = new MediaStream([event.track]);
        if (streamType === 'color') {
          setRgbStream(newStream);
          setStreamMetrics(prev => ({
            ...prev,
            rgb: { width: 640, height: 360, fps: 30 }
          }));
        } else if (streamType === 'depth') {
          setDepthStream(newStream);
          setStreamMetrics(prev => ({
            ...prev,
            depth: { width: 640, height: 360, fps: 15 }
          }));
        }

        trackIndex++;
      };

      // 5. Handle ICE Candidates
      pc.onicecandidate = (event) => {
        if (event.candidate && sessionId.current) {
          webrtcApi.sendIceCandidate(sessionId.current, event.candidate);
        }
      };

      // 6. Set Remote Description
      await pc.setRemoteDescription({ type: offerData.type, sdp: offerData.sdp });

      // 7. Create Answer
      const answer = await pc.createAnswer();
      await pc.setLocalDescription(answer);
      if (sessionId.current) {
        await webrtcApi.sendAnswer(sessionId.current, answer);
      }

      // 8. Get Backend ICE Candidates
      setTimeout(async () => {
        if (sessionId.current) {
          const candidates = await webrtcApi.getIceCandidates(sessionId.current);
          for (const c of candidates) {
            await pc.addIceCandidate(new RTCIceCandidate(c));
          }
        }
      }, 1000);

      setIsStreaming(true);

    } catch (e: any) {
      console.error(e);
      setError(e.message);
      setIsStreaming(false);
    }
  }, [device]);

  const stopConnection = useCallback(async () => {
    // 1. 关闭 PeerConnection
    if (peerConnection.current) {
      peerConnection.current.getSenders().forEach(sender => {
        sender.track?.stop();
      });
      peerConnection.current.getReceivers().forEach(receiver => {
        receiver.track?.stop();
      });
      peerConnection.current.close();
      peerConnection.current = null;
    }

    // 2. 清空流状态
    setRgbStream(null);
    setDepthStream(null);
    setIsStreaming(false);
    sessionId.current = null;

    // 3. 通知后端停止
    if (device) {
      try {
        await webrtcApi.stopStream(device.device_id);
      } catch (e) {
        console.warn("停止流时后端返回错误:", e);
      }
    }
  }, [device]);

  // Auto-start streaming once device is ready
  useEffect(() => {
    let mounted = true;
    if (device && !isStreaming && !autoStartAttempted.current) {
      autoStartAttempted.current = true;
      setTimeout(() => {
        if (mounted) {
          console.log("[AutoStart] Initiating automatic stream connection...");
          startConnection();
        } else {
          autoStartAttempted.current = false;
        }
      }, 1000);
    }
    return () => {
      mounted = false;
    };
  }, [device, isStreaming, startConnection]);

  return {
    device,
    isStreaming,
    rgbStream,
    depthStream,
    streamMetrics,
    error,
    startConnection,
    stopConnection
  };
}
