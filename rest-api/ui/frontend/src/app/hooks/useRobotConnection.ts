
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
    const [error, setError] = useState<string | null>(null);

    const peerConnection = useRef<RTCPeerConnection | null>(null);
    const sessionId = useRef<string | null>(null);
    const socket = useRef<Socket | null>(null);

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
            path: '/socket',
            transports: ['websocket'],
        });

        socket.current.on('connect', () => {
            console.log('Socket connected');
        });

        socket.current.on('metadata_update', (data: any) => {
            if (data.metadata_streams?.depth?.point_cloud?.vertices) {
                try {
                    const base64Vertices = data.metadata_streams.depth.point_cloud.vertices;
                    const binaryString = window.atob(base64Vertices);
                    const len = binaryString.length;
                    const bytes = new Uint8Array(len);
                    for (let i = 0; i < len; i++) {
                        bytes[i] = binaryString.charCodeAt(i);
                    }
                    const vertices = new Float32Array(bytes.buffer);
                    setPointCloudData(vertices);
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

            // 5. Handle Tracks
            pc.ontrack = (event) => {
                const mid = event.transceiver.mid;
                if (mid && offerData.stream_map) {
                    const streamType = offerData.stream_map[mid];
                    const newStream = new MediaStream([event.track]);

                    if (streamType === 'color') {
                        setRgbStream(newStream);
                    } else if (streamType === 'depth') {
                        setDepthStream(newStream);
                    }
                }
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
        if (!device) return;

        try {
            if (peerConnection.current) {
                peerConnection.current.close();
                peerConnection.current = null;
            }
            setRgbStream(null);
            setDepthStream(null);
            setPointCloudData(null);

            await api.stopStream(device.device_id);
            await api.deactivatePointCloud(device.device_id);
            setIsStreaming(false);
        } catch (e) {
            console.error("Error stopping stream:", e);
        }
    }, [device]);

    return {
        device,
        isStreaming,
        rgbStream,
        depthStream,
        pointCloudData,
        error,
        startConnection,
        stopConnection
    };
}
