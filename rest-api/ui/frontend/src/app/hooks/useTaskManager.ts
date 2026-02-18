/**
 * React hook for managing tasks with Socket.IO real-time updates.
 */

import { useState, useEffect, useRef, useCallback } from 'react';
import { io, Socket } from 'socket.io-client';
import { SOCKET_URL } from '../config';
import {
  taskApi,
  TaskInfo,
  TaskTypeInfo,
  TaskCreateRequest,
  TaskStatus,
  TaskEvent,
  TaskListResponse,
} from '../services/taskApi';

interface UseTaskManagerOptions {
  autoConnect?: boolean;
  deviceId?: string | null;
}

interface UseTaskManagerReturn {
  // Task types
  taskTypes: TaskTypeInfo[];
  isLoadingTypes: boolean;

  // Tasks
  tasks: TaskInfo[];
  isLoadingTasks: boolean;
  runningCount: number;
  pendingCount: number;
  canStartMore: boolean;

  // Connection status
  isConnected: boolean;

  // Actions
  refreshTasks: () => Promise<void>;
  refreshTaskTypes: () => Promise<void>;
  createTask: (request: TaskCreateRequest) => Promise<TaskInfo>;
  startTask: (taskId: string) => Promise<TaskInfo>;
  pauseTask: (taskId: string) => Promise<TaskInfo>;
  resumeTask: (taskId: string) => Promise<TaskInfo>;
  stopTask: (taskId: string) => Promise<TaskInfo>;
  deleteTask: (taskId: string) => Promise<void>;

  // Error handling
  error: string | null;
  clearError: () => void;
}

export function useTaskManager(options: UseTaskManagerOptions = {}): UseTaskManagerReturn {
  const { autoConnect = true, deviceId } = options;

  // State
  const [taskTypes, setTaskTypes] = useState<TaskTypeInfo[]>([]);
  const [isLoadingTypes, setIsLoadingTypes] = useState(false);

  const [tasks, setTasks] = useState<TaskInfo[]>([]);
  const [isLoadingTasks, setIsLoadingTasks] = useState(false);
  const [runningCount, setRunningCount] = useState(0);
  const [pendingCount, setPendingCount] = useState(0);
  const [canStartMore, setCanStartMore] = useState(true);

  const [isConnected, setIsConnected] = useState(false);
  const [error, setError] = useState<string | null>(null);

  // Refs
  const socketRef = useRef<Socket | null>(null);

  // --- Socket.IO Connection ---

  useEffect(() => {
    if (!autoConnect) return;

    socketRef.current = io(SOCKET_URL, {
      path: '/socket',
      transports: ['websocket'],
    });

    const socket = socketRef.current;

    socket.on('connect', () => {
      console.log('[TaskManager] Socket connected');
      setIsConnected(true);
    });

    socket.on('disconnect', () => {
      console.log('[TaskManager] Socket disconnected');
      setIsConnected(false);
    });

    socket.on('task_event', (event: TaskEvent) => {
      console.log('[TaskManager] Task event:', event);
      handleTaskEvent(event);
    });

    return () => {
      socket.disconnect();
    };
  }, [autoConnect]);

  // Handle task events from Socket.IO
  const handleTaskEvent = useCallback((event: TaskEvent) => {
    setTasks((prevTasks) => {
      const existingIndex = prevTasks.findIndex((t) => t.task_id === event.task_id);

      if (existingIndex >= 0) {
        // Update existing task
        const updated = [...prevTasks];
        const existing = updated[existingIndex];

        updated[existingIndex] = {
          ...existing,
          status: event.status,
          progress: event.progress ?? existing.progress,
          result: event.result ?? existing.result,
        };

        return updated;
      } else {
        // New task - fetch full info
        taskApi.getTask(event.task_id).then((task) => {
          setTasks((prev) => [...prev, task]);
        });
        return prevTasks;
      }
    });

    // Update counts
    updateCounts();
  }, []);

  // Update running/pending counts
  const updateCounts = useCallback(() => {
    setTasks((prevTasks) => {
      const running = prevTasks.filter((t) => t.status === 'running').length;
      const pending = prevTasks.filter((t) => t.status === 'pending').length;
      setRunningCount(running);
      setPendingCount(pending);
      setCanStartMore(running < 4);
      return prevTasks;
    });
  }, []);

  // --- Data Fetching ---

  const refreshTaskTypes = useCallback(async () => {
    setIsLoadingTypes(true);
    setError(null);
    try {
      const types = await taskApi.getTaskTypes();
      setTaskTypes(types);
    } catch (e: any) {
      console.error('[TaskManager] Failed to fetch task types:', e);
      setError(e.message);
    } finally {
      setIsLoadingTypes(false);
    }
  }, []);

  const refreshTasks = useCallback(async () => {
    setIsLoadingTasks(true);
    setError(null);
    try {
      const response: TaskListResponse = await taskApi.listTasks(
        deviceId ? { device_id: deviceId } : undefined
      );
      setTasks(response.tasks);
      setRunningCount(response.running_count);
      setPendingCount(response.pending_count);
      setCanStartMore(response.running_count < 4);
    } catch (e: any) {
      console.error('[TaskManager] Failed to fetch tasks:', e);
      setError(e.message);
    } finally {
      setIsLoadingTasks(false);
    }
  }, [deviceId]);

  // Initial fetch
  useEffect(() => {
    refreshTaskTypes();
    refreshTasks();
  }, [refreshTaskTypes, refreshTasks]);

  // --- Task Actions ---

  const createTask = useCallback(async (request: TaskCreateRequest): Promise<TaskInfo> => {
    setError(null);
    try {
      // Add device_id from options if not provided
      const fullRequest = {
        ...request,
        device_id: request.device_id ?? deviceId,
      };

      const task = await taskApi.createTask(fullRequest);
      setTasks((prev) => [...prev, task]);
      setPendingCount((prev) => prev + 1);
      return task;
    } catch (e: any) {
      console.error('[TaskManager] Failed to create task:', e);
      setError(e.message);
      throw e;
    }
  }, [deviceId]);

  const startTask = useCallback(async (taskId: string): Promise<TaskInfo> => {
    setError(null);
    try {
      const task = await taskApi.startTask(taskId);
      updateTaskInState(task);
      return task;
    } catch (e: any) {
      console.error('[TaskManager] Failed to start task:', e);
      setError(e.message);
      throw e;
    }
  }, []);

  const pauseTask = useCallback(async (taskId: string): Promise<TaskInfo> => {
    setError(null);
    try {
      const task = await taskApi.pauseTask(taskId);
      updateTaskInState(task);
      return task;
    } catch (e: any) {
      console.error('[TaskManager] Failed to pause task:', e);
      setError(e.message);
      throw e;
    }
  }, []);

  const resumeTask = useCallback(async (taskId: string): Promise<TaskInfo> => {
    setError(null);
    try {
      const task = await taskApi.resumeTask(taskId);
      updateTaskInState(task);
      return task;
    } catch (e: any) {
      console.error('[TaskManager] Failed to resume task:', e);
      setError(e.message);
      throw e;
    }
  }, []);

  const stopTask = useCallback(async (taskId: string): Promise<TaskInfo> => {
    setError(null);
    try {
      const task = await taskApi.stopTask(taskId);
      updateTaskInState(task);
      return task;
    } catch (e: any) {
      console.error('[TaskManager] Failed to stop task:', e);
      setError(e.message);
      throw e;
    }
  }, []);

  const deleteTask = useCallback(async (taskId: string): Promise<void> => {
    setError(null);
    try {
      await taskApi.deleteTask(taskId);
      setTasks((prev) => prev.filter((t) => t.task_id !== taskId));
      updateCounts();
    } catch (e: any) {
      console.error('[TaskManager] Failed to delete task:', e);
      setError(e.message);
      throw e;
    }
  }, []);

  // Helper to update a task in state
  const updateTaskInState = useCallback((updatedTask: TaskInfo) => {
    setTasks((prev) =>
      prev.map((t) => (t.task_id === updatedTask.task_id ? updatedTask : t))
    );
    updateCounts();
  }, [updateCounts]);

  const clearError = useCallback(() => {
    setError(null);
  }, []);

  return {
    taskTypes,
    isLoadingTypes,
    tasks,
    isLoadingTasks,
    runningCount,
    pendingCount,
    canStartMore,
    isConnected,
    refreshTasks,
    refreshTaskTypes,
    createTask,
    startTask,
    pauseTask,
    resumeTask,
    stopTask,
    deleteTask,
    error,
    clearError,
  };
}
