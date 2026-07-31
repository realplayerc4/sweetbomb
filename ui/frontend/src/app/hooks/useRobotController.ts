/**
 * 机器人控制 React hook，基于 Socket.IO 实时更新。
 */

import { useState, useEffect, useRef, useCallback } from 'react';
import { io, Socket } from 'socket.io-client';
import { SOCKET_URL } from '../config';
import { robotApi, type RobotStatus } from '../services/robotApi';

interface UseRobotControllerOptions {
    autoConnect?: boolean;
    pollInterval?: number; // 回退轮询间隔
}

interface UseRobotControllerReturn {
    // Robot status
    status: RobotStatus | null;
    isLoading: boolean;
    isConnected: boolean;

    // 任务状态跟踪
    pendingTaskId: string | null;  // 当前正在执行的任务ID
    isTaskRunning: boolean;        // 是否有任务正在执行

    // Actions
    move: (direction: string, speed?: number, duration?: number) => Promise<void>;
    stop: () => Promise<void>;
    pause: () => Promise<void>;
    resume: () => Promise<void>;
    navToPick: () => Promise<void>;
    navToDrop: () => Promise<void>;
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
    const { autoConnect = true, pollInterval = 500 } = options;

    // State
    const [status, setStatus] = useState<RobotStatus | null>(null);
    const [isLoading, setIsLoading] = useState(false);
    const [isConnected, setIsConnected] = useState(false);
    const [error, setError] = useState<string | null>(null);

    const [harvestStatus, setHarvestStatus] = useState<any>(null);
    const [isHarvestRunning, setIsHarvestRunning] = useState(false);

    // 任务状态跟踪
    const [pendingTaskId, setPendingTaskId] = useState<string | null>(null);
    const [isTaskRunning, setIsTaskRunning] = useState(false);

    // Refs
    const socketRef = useRef<Socket | null>(null);
    const pollIntervalRef = useRef<NodeJS.Timeout | null>(null);

    // --- Socket.IO 连接 ---

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

        // 行为树事件
        socket.on('bt_event', (event: any) => {
            console.log('[RobotController] BT event:', event);
            handleBTEvent(event);
        });

        socket.on('bt_node_status', (data: any) => {
            console.log('[RobotController] BT node status:', data);
        });

        // 监听机器人任务完成事件
        socket.on('robot_task_finish', (event: { task_id: string }) => {
            console.log('[RobotController] Robot task finished:', event);
            setPendingTaskId((prev) => {
                if (prev === event.task_id) {
                    setIsTaskRunning(false);
                    return null;
                }
                return prev;
            });
        });

        return () => {
            socket.disconnect();
        };
    }, [autoConnect]);

    // 处理行为树事件
    const handleBTEvent = useCallback((event: any) => {
        if (event.event_type === 'bt_started') {
            setIsHarvestRunning(true);
        } else if (event.event_type === 'bt_completed' || event.event_type === 'bt_cancelled') {
            setIsHarvestRunning(false);
        }

        // 更新收获状态
        setHarvestStatus({
            is_running: isHarvestRunning,
            ...event,
        });
    }, [isHarvestRunning]);

    // --- 数据获取 ---

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

    // 轮询机器人状态（始终激活）
    useEffect(() => {
        pollIntervalRef.current = setInterval(() => {
            refreshStatus().catch(() => {});
        }, pollInterval);

        return () => {
            if (pollIntervalRef.current) {
                clearInterval(pollIntervalRef.current);
            }
        };
    }, [pollInterval, refreshStatus]);

    // 初始获取 - 包裹在 try-catch 中防止应用崩溃
    useEffect(() => {
        refreshStatus().catch(() => {});
    }, [refreshStatus]);

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
            setPendingTaskId(null);
            setIsTaskRunning(false);
            await refreshStatus();
        } catch (e: any) {
            setError(e.message);
            throw e;
        }
    }, [refreshStatus]);

    const pause = useCallback(async () => {
        setError(null);
        try {
            await robotApi.pause();
            await refreshStatus();
        } catch (e: any) {
            setError(e.message);
            throw e;
        }
    }, [refreshStatus]);

    const resume = useCallback(async () => {
        setError(null);
        try {
            await robotApi.resume();
            await refreshStatus();
        } catch (e: any) {
            setError(e.message);
            throw e;
        }
    }, [refreshStatus]);

    const navToPick = useCallback(async () => {
        setError(null);
        try {
            const result = await robotApi.navToPick();
            setPendingTaskId(result.task_id);
            setIsTaskRunning(true);
            await refreshStatus();
        } catch (e: any) {
            setError(e.message);
            throw e;
        }
    }, [refreshStatus]);

    const navToDrop = useCallback(async () => {
        setError(null);
        try {
            const result = await robotApi.navToDrop();
            setPendingTaskId(result.task_id);
            setIsTaskRunning(true);
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
            const result = await robotApi.scoop();
            setPendingTaskId(result.task_id);
            setIsTaskRunning(true);
            await refreshStatus();
        } catch (e: any) {
            setError(e.message);
            throw e;
        }
    }, [refreshStatus]);

    const dump = useCallback(async () => {
        setError(null);
        try {
            const result = await robotApi.dump();
            setPendingTaskId(result.task_id);
            setIsTaskRunning(true);
            await refreshStatus();
        } catch (e: any) {
            setError(e.message);
            throw e;
        }
    }, [refreshStatus]);

    const dock = useCallback(async () => {
        setError(null);
        try {
            const result = await robotApi.dock();
            setPendingTaskId(result.task_id);
            setIsTaskRunning(true);
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
        pendingTaskId,
        isTaskRunning,
        move,
        stop,
        pause,
        resume,
        navToPick,
        navToDrop,
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
