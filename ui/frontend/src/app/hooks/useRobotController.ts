/**
 * React hook for robot control with Socket.IO real-time updates.
 */

import { useState, useEffect, useRef, useCallback } from 'react';
import { io, Socket } from 'socket.io-client';
import { SOCKET_URL } from '../config';
import { robotApi, type RobotStatus } from '../services/robotApi';

interface UseRobotControllerOptions {
    autoConnect?: boolean;
    pollInterval?: number; // Fallback polling interval if socket fails
}

interface UseRobotControllerReturn {
    // Robot status
    status: RobotStatus | null;
    isLoading: boolean;
    isConnected: boolean;

    // Actions
    move: (direction: string, speed?: number, duration?: number) => Promise<void>;
    stop: () => Promise<void>;
    reset: () => Promise<void>;
    setServo: (servoId: string, angle: number) => Promise<void>;
    scoop: () => Promise<void>;
    dump: () => Promise<void>;
    dock: () => Promise<void>;

    // Sugar harvest
    startSugarHarvest: (config: any) => Promise<void>;
    stopSugarHarvest: () => Promise<void>;
    harvestStatus: any;
    isHarvestRunning: boolean;

    // Refresh
    refreshStatus: () => Promise<void>;

    // Error handling
    error: string | null;
    clearError: () => void;
}

export function useRobotController(options: UseRobotControllerOptions = {}): UseRobotControllerReturn {
    const { autoConnect = true, pollInterval = 1000 } = options;

    // State
    const [status, setStatus] = useState<RobotStatus | null>(null);
    const [isLoading, setIsLoading] = useState(false);
    const [isConnected, setIsConnected] = useState(false);
    const [error, setError] = useState<string | null>(null);

    const [harvestStatus, setHarvestStatus] = useState<any>(null);
    const [isHarvestRunning, setIsHarvestRunning] = useState(false);

    // Refs
    const socketRef = useRef<Socket | null>(null);
    const pollIntervalRef = useRef<NodeJS.Timeout | null>(null);

    // --- Socket.IO Connection ---

    useEffect(() => {
        if (!autoConnect) return;

        socketRef.current = io(SOCKET_URL, {
            path: '/socket.io',
            transports: ['websocket'],
        });

        const socket = socketRef.current;

        socket.on('connect', () => {
            console.log('[RobotController] Socket connected');
            setIsConnected(true);
        });

        socket.on('disconnect', () => {
            console.log('[RobotController] Socket disconnected');
            setIsConnected(false);
        });

        // Behavior tree events
        socket.on('bt_event', (event: any) => {
            console.log('[RobotController] BT event:', event);
            handleBTEvent(event);
        });

        socket.on('bt_node_status', (data: any) => {
            console.log('[RobotController] BT node status:', data);
            // Could update UI with current executing node
        });

        return () => {
            socket.disconnect();
        };
    }, [autoConnect]);

    // Handle behavior tree events
    const handleBTEvent = useCallback((event: any) => {
        if (event.event_type === 'bt_started') {
            setIsHarvestRunning(true);
        } else if (event.event_type === 'bt_completed' || event.event_type === 'bt_cancelled') {
            setIsHarvestRunning(false);
        }

        // Update harvest status
        setHarvestStatus({
            is_running: isHarvestRunning,
            ...event,
        });
    }, [isHarvestRunning]);

    // --- Data Fetching ---

    const refreshStatus = useCallback(async () => {
        setIsLoading(true);
        setError(null);
        try {
            const robotStatus = await robotApi.getStatus();
            setStatus(robotStatus);
        } catch (e: any) {
            console.error('[RobotController] Failed to fetch status:', e);
            setError(e.message);
        } finally {
            setIsLoading(false);
        }
    }, []);

    const refreshHarvestStatus = useCallback(async () => {
        try {
            const status = await robotApi.getSugarHarvestStatus();
            setHarvestStatus(status);
            setIsHarvestRunning(status.is_running);
        } catch (e: any) {
            console.error('[RobotController] Failed to fetch harvest status:', e);
        }
    }, []);

    // Polling fallback
    useEffect(() => {
        if (isConnected) return; // Don't poll if socket is connected

        pollIntervalRef.current = setInterval(() => {
            refreshStatus().catch(() => {});
            refreshHarvestStatus().catch(() => {});
        }, pollInterval);

        return () => {
            if (pollIntervalRef.current) {
                clearInterval(pollIntervalRef.current);
            }
        };
    }, [isConnected, pollInterval, refreshStatus, refreshHarvestStatus]);

    // Initial fetch - wrapped in try-catch to prevent app crash
    useEffect(() => {
        refreshStatus().catch(() => {});
        refreshHarvestStatus().catch(() => {});
    }, [refreshStatus, refreshHarvestStatus]);

    // --- Actions ---

    const move = useCallback(async (direction: string, speed = 0.5, duration = 1.0) => {
        setError(null);
        try {
            await robotApi.move({ direction: direction as any, speed, duration });
            await refreshStatus();
        } catch (e: any) {
            setError(e.message);
            throw e;
        }
    }, [refreshStatus]);

    const stop = useCallback(async () => {
        setError(null);
        try {
            await robotApi.stop();
            await refreshStatus();
        } catch (e: any) {
            setError(e.message);
            throw e;
        }
    }, [refreshStatus]);

    const reset = useCallback(async () => {
        setError(null);
        try {
            await robotApi.reset();
            await refreshStatus();
        } catch (e: any) {
            setError(e.message);
            throw e;
        }
    }, [refreshStatus]);

    const setServo = useCallback(async (servoId: string, angle: number) => {
        setError(null);
        try {
            await robotApi.setServo({ servo_id: servoId, angle });
            await refreshStatus();
        } catch (e: any) {
            setError(e.message);
            throw e;
        }
    }, [refreshStatus]);

    const scoop = useCallback(async () => {
        setError(null);
        try {
            await robotApi.scoop();
            await refreshStatus();
        } catch (e: any) {
            setError(e.message);
            throw e;
        }
    }, [refreshStatus]);

    const dump = useCallback(async () => {
        setError(null);
        try {
            await robotApi.dump();
            await refreshStatus();
        } catch (e: any) {
            setError(e.message);
            throw e;
        }
    }, [refreshStatus]);
    const dock = useCallback(async () => {
        setError(null);
        try {
            await robotApi.dock();
            await refreshStatus();
        } catch (e: any) {
            setError(e.message);
            throw e;
        }
    }, [refreshStatus]);


    const startSugarHarvest = useCallback(async (config: any) => {
        setError(null);
        try {
            await robotApi.startSugarHarvest({ config });
            setIsHarvestRunning(true);
            await refreshHarvestStatus();
        } catch (e: any) {
            setError(e.message);
            throw e;
        }
    }, [refreshHarvestStatus]);

    const stopSugarHarvestFunc = useCallback(async () => {
        setError(null);
        try {
            await robotApi.stopSugarHarvest();
            setIsHarvestRunning(false);
            await refreshHarvestStatus();
        } catch (e: any) {
            setError(e.message);
            throw e;
        }
    }, [refreshHarvestStatus]);

    const clearError = useCallback(() => {
        setError(null);
    }, []);

    return {
        status,
        isLoading,
        isConnected,
        move,
        stop,
        reset,
        setServo,
        scoop,
        dump,
        dock,
        startSugarHarvest,
        stopSugarHarvest: stopSugarHarvestFunc,
        harvestStatus,
        isHarvestRunning,
        refreshStatus,
        error,
        clearError,
    };
}
