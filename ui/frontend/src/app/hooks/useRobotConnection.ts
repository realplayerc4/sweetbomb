import { useState, useEffect, useRef, useCallback } from 'react';
import type { DeviceInfo } from '../services/api';
import { api } from '../services/api';
import { io, Socket } from 'socket.io-client';
import { SOCKET_URL } from '../config';

export function useRobotConnection() {
    const [device, setDevice] = useState<DeviceInfo | null>(null);
    const [isStreaming, setIsStreaming] = useState(false);
    const [rgbStream, setRgbStream] = useState<MediaStream | null>(null);
    const [depthStream, setDepthStream] = useState<MediaStream | null>(null);
    const [pointCloudData, setPointCloudData] = useState<Float32Array | null>(null);
    const [streamMetrics, setStreamMetrics] = useState<{
        rgb: { width: number; height: number; fps: number } | null;
        depth: { width: number; height: number; fps: number } | null;
        pointCount: number;
    }>({ rgb: null, depth: null, pointCount: 0 });
    const [systemStats, setSystemStats] = useState<{
        cpu_load: number;
        battery: number;
        temperature: number;
        signal: number;
        hostname: string;
        imu?: { roll: number; pitch: number; yaw: number };
    } | null>(null);
    const [error, setError] = useState<string | null>(null);

    const peerConnection = useRef<RTCPeerConnection | null>(null);
    const sessionId = useRef<string | null>(null);
    const socket = useRef<Socket | null>(null);
    const autoStartAttempted = useRef(false);
    const lastMetricsUpdateTime = useRef<number>(0);
    const lastPointCountUpdateTime = useRef<number>(0);

    // Auto-connect to first available device
    useEffect(() => {
        api.getDevices().then(devices => {
            if (devices.length > 0) {
                setDevice(devices[0]);
            }
        }).catch(err => console.error("Failed to fetch devices:", err));
    }, []);

    // Initialize Socket.IO
    useEffect(() => {
        socket.current = io(SOCKET_URL, {
            path: '/socket.io',
            transports: ['websocket'],
        });

        socket.current.on('connect', () => {
            console.log('Socket connected');
        });

        socket.current.on('metadata_update', (data: any) => {
            // Debug log to check if point cloud data is in the metadata
            if (data.metadata_streams?.depth) {
                const hasPointCloud = !!data.metadata_streams.depth.point_cloud?.vertices;
                console.log('[RobotConnection] metadata_update - depth stream has point_cloud:', hasPointCloud);
                if (!hasPointCloud) {
                    console.log('[RobotConnection] depth metadata keys:', Object.keys(data.metadata_streams.depth));
                }
            }

            if (data.system_stats) {
                setSystemStats(data.system_stats);
            }

            // Update Stream Metrics - Throttled to 1 sec
            const now = Date.now();
            if (now - lastMetricsUpdateTime.current >= 1000) {
                lastMetricsUpdateTime.current = now;
                const newMetrics = { rgb: null as any, depth: null as any, pointCount: 0 };

                if (data.metadata_streams?.color) {
                    newMetrics.rgb = {
                        width: data.metadata_streams.color.width || 0,
                        height: data.metadata_streams.color.height || 0,
                        fps: data.metadata_streams.color.frame_number ? Math.round(data.metadata_streams.color.frame_number / Math.max(1, data.metadata_streams.color.timestamp)) : 30 // Approximate or fallback
                    };
                    // Since RealSense wrapper might not easily provide exact instantaneous FPS here without historical tracking,
                    // we'll rely on our requested framerate config (30) or backend calculation if available.
                    // Assuming we fallback to 30/15 depending on requested config until backend sends explicit fps.
                    // For now, hardcode to 30/15 as configured or parse from metadata if backend adds it.
                    newMetrics.rgb.fps = 30; // using configured 30 FPS
                }

                if (data.metadata_streams?.depth) {
                    newMetrics.depth = {
                        width: data.metadata_streams.depth.width || 0,
                        height: data.metadata_streams.depth.height || 0,
                        fps: 15 // using configured 15 FPS
                    };
                }

                setStreamMetrics(prev => ({ ...prev, rgb: newMetrics.rgb || prev.rgb, depth: newMetrics.depth || prev.depth }));
            }

            // 点云数据处理
            if (data.metadata_streams?.depth?.point_cloud?.vertices) {
                try {
                    const base64Vertices = data.metadata_streams.depth.point_cloud.vertices;
                    const binaryString = window.atob(base64Vertices);
                    const len = binaryString.length;
                    const bytes = new Uint8Array(len);
                    for (let i = 0; i < len; i++) {
                        bytes[i] = binaryString.charCodeAt(i);
                    }
                    const rawVertices = new Float32Array(bytes.buffer);

                    console.log('[RobotConnection] Received REAL point cloud data:', rawVertices.length / 3, 'points');

                    // Backend already transformed coordinates to ROS coordinate system
                    // Just pass through directly - no further transformation needed
                    setPointCloudData(rawVertices);

                    // Update Point Cloud Metrics - independent throttling for point count
                    const vertexCount = rawVertices.length / 3;
                    if (now - lastPointCountUpdateTime.current >= 100) {  // 更新频率: 100ms
                        lastPointCountUpdateTime.current = now;
                        setStreamMetrics(prev => ({ ...prev, pointCount: vertexCount }));
                    }

                } catch (e) {
                    console.error("Error parsing point cloud:", e);
                }
            }
        });

        return () => {
            socket.current?.disconnect();
        };
    }, []);

    const startConnection = useCallback(async () => {
        if (!device) return;
        setError(null);

        try {
            // 1. Start Streams on Backend
            await api.startStream(device.device_id, {
                configs: [
                    {
                        sensor_id: `${device.device_id}-sensor-0`, // Assuming sensor 0 is depth, checking later
                        stream_type: "depth",
                        format: "z16",
                        resolution: { width: 1280, height: 720 },
                        framerate: 15,
                        enable: true
                    },
                    {
                        sensor_id: `${device.device_id}-sensor-1`, // Assuming sensor 1 is RGB
                        stream_type: "color",
                        format: "rgb8",
                        resolution: { width: 1280, height: 720 },
                        framerate: 30,
                        enable: true
                    }
                ],
                align_to: "color"
            });

            // 2. Activate Point Cloud
            await api.activatePointCloud(device.device_id);

            // 3. WebRTC Offer
            const offerData = await api.getWebRTCOffer(device.device_id, ['color', 'depth']);
            sessionId.current = offerData.session_id;

            // 4. Create Peer Connection
            const pc = new RTCPeerConnection({
                iceServers: [{ urls: 'stun:stun.l.google.com:19302' }]
            });
            peerConnection.current = pc;

            // 5. Handle Tracks - 使用计数器作为 fallback
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
                } else if (streamType === 'depth') {
                    setDepthStream(newStream);
                }

                trackIndex++;
            };

            // 6. Handle ICE Candidates
            pc.onicecandidate = (event) => {
                if (event.candidate && sessionId.current) {
                    api.sendIceCandidate(sessionId.current, event.candidate);
                }
            };

            // 7. Set Remote Description
            await pc.setRemoteDescription({ type: offerData.type, sdp: offerData.sdp });

            // 8. Create Answer
            const answer = await pc.createAnswer();
            await pc.setLocalDescription(answer);
            if (sessionId.current) {
                await api.sendAnswer(sessionId.current, answer);
            }

            // 9. Get Backend ICE Candidates
            setTimeout(async () => {
                if (sessionId.current) {
                    const candidates = await api.getIceCandidates(sessionId.current);
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
        // 1. 先关闭 PeerConnection 并停止所有 track
        if (peerConnection.current) {
            // 停止所有发送/接收的 track
            peerConnection.current.getSenders().forEach(sender => {
                sender.track?.stop();
            });
            peerConnection.current.getReceivers().forEach(receiver => {
                receiver.track?.stop();
            });
            peerConnection.current.close();
            peerConnection.current = null;
        }

        // 2. 立即清空前端流状态，触发视图组件清除残留帧
        setRgbStream(null);
        setDepthStream(null);
        setPointCloudData(null);
        setIsStreaming(false);
        sessionId.current = null;

        // 3. 通知后端停止（即使失败也不影响前端清理）
        if (device) {
            try {
                await api.stopStream(device.device_id);
            } catch (e) {
                console.warn("停止流时后端返回错误（可忽略）:", e);
            }
            try {
                await api.deactivatePointCloud(device.device_id);
            } catch (e) {
                console.warn("停用点云时后端返回错误（可忽略）:", e);
            }
        }
    }, [device]);

    // Auto-start streaming once device is ready
    useEffect(() => {
        let mounted = true;
        if (device && !isStreaming && !autoStartAttempted.current) {
            autoStartAttempted.current = true;
            // Add a short delay to ensure component is stably mounted
            // and bypass React 18 Strict Mode double-mount aborts.
            setTimeout(() => {
                if (mounted) {
                    console.log("[AutoStart] Initiating automatic stream connection...");
                    startConnection();
                } else {
                    autoStartAttempted.current = false; // Reset if unmounted before execution
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
        pointCloudData,
        streamMetrics,
        systemStats,
        error,
        startConnection,
        stopConnection
    };
}
