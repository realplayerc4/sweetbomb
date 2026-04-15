import { create } from 'zustand';
import { io, type Socket } from 'socket.io-client';
import { SOCKET_URL } from '../config';
import { taskApi } from '../services/taskApi';
import type { TaskInfo, TaskStatus, TaskCreateRequest } from '../types';

interface TaskEvent {
  event_type: string;
  task_id: string;
  task_type: string;
  status: TaskStatus;
  progress?: TaskInfo['progress'] | null;
  result?: TaskInfo['result'] | null;
  timestamp: string;
}

interface TaskStore {
  tasks: TaskInfo[];
  isLoading: boolean;
  isConnected: boolean;
  runningCount: number;
  pendingCount: number;
  completedToday: number;
  error: string | null;
  socket: Socket | null;

  connect: () => void;
  disconnect: () => void;
  refreshTasks: () => Promise<void>;
  createTask: (request: TaskCreateRequest) => Promise<TaskInfo>;
  startTask: (taskId: string) => Promise<void>;
  pauseTask: (taskId: string) => Promise<void>;
  resumeTask: (taskId: string) => Promise<void>;
  stopTask: (taskId: string) => Promise<void>;
  deleteTask: (taskId: string) => Promise<void>;
  clearError: () => void;
}

function updateCounts(tasks: TaskInfo[]) {
  const running = tasks.filter((t) => t.status === 'running').length;
  const pending = tasks.filter((t) => t.status === 'pending').length;
  const today = new Date().toDateString();
  const completed = tasks.filter(
    (t) => t.status === 'completed' && new Date(t.completed_at ?? '').toDateString() === today
  ).length;
  return { runningCount: running, pendingCount: pending, completedToday: completed };
}

export const useTaskStore = create<TaskStore>((set, get) => ({
  tasks: [
    {
      task_id: 'task-001',
      task_type: 'sugar_harvest',
      status: 'running',
      priority: 10,
      device_id: 'loader-001',
      config: {
        device_id: 'loader-001',
        priority: 10,
        max_retries: 3,
        timeout_seconds: 3600
      },
      params: {
        navigation_point: [22.5431, 108.3743],
        dump_point: [22.5445, 108.3750],
        bucket_width_m: 0.8,
        approach_offset_m: 0.5,
        max_cycles: 10,
        height_threshold_m: 0.3
      },
      created_at: new Date(Date.now() - 1800000).toISOString(),
      started_at: new Date(Date.now() - 1500000).toISOString(),
      progress: {
        current_step: 3,
        total_steps: 10,
        percentage: 30,
        message: '正在执行第3趟作业',
        elapsed_seconds: 3000
      },
      target_ton: 8.5
    },
    {
      task_id: 'task-002',
      task_type: 'sugar_harvest',
      status: 'running',
      priority: 10,
      device_id: 'loader-002',
      config: {
        device_id: 'loader-002',
        priority: 10,
        max_retries: 3,
        timeout_seconds: 3600
      },
      params: {
        navigation_point: [22.5435, 108.3747],
        dump_point: [22.5445, 108.3750],
        bucket_width_m: 0.8,
        approach_offset_m: 0.5,
        max_cycles: 10,
        height_threshold_m: 0.3
      },
      created_at: new Date(Date.now() - 2100000).toISOString(),
      started_at: new Date(Date.now() - 1800000).toISOString(),
      progress: {
        current_step: 5,
        total_steps: 10,
        percentage: 50,
        message: '正在执行第5趟作业',
        elapsed_seconds: 3600
      },
      target_ton: 8.5
    },
    {
      task_id: 'task-003',
      task_type: 'sugar_harvest',
      status: 'paused',
      priority: 5,
      device_id: 'loader-004',
      config: {
        device_id: 'loader-004',
        priority: 5,
        max_retries: 3,
        timeout_seconds: 3600
      },
      params: {
        navigation_point: [22.5440, 108.3740],
        dump_point: [22.5445, 108.3750],
        bucket_width_m: 0.8,
        approach_offset_m: 0.5,
        max_cycles: 10,
        height_threshold_m: 0.3
      },
      created_at: new Date(Date.now() - 2400000).toISOString(),
      started_at: new Date(Date.now() - 2100000).toISOString(),
      progress: {
        current_step: 2,
        total_steps: 10,
        percentage: 20,
        message: '已暂停，等待维护',
        elapsed_seconds: 1800
      },
      target_ton: 8.5
    },
    {
      task_id: 'task-004',
      task_type: 'sugar_harvest',
      status: 'running',
      priority: 5,
      device_id: 'loader-006',
      config: {
        device_id: 'loader-006',
        priority: 5,
        max_retries: 3,
        timeout_seconds: 3600
      },
      params: {
        navigation_point: [22.5438, 108.3755],
        dump_point: [22.5445, 108.3750],
        bucket_width_m: 0.8,
        approach_offset_m: 0.5,
        max_cycles: 10,
        height_threshold_m: 0.3
      },
      created_at: new Date(Date.now() - 900000).toISOString(),
      started_at: new Date(Date.now() - 600000).toISOString(),
      progress: {
        current_step: 1,
        total_steps: 10,
        percentage: 10,
        message: '正在执行第1趟作业',
        elapsed_seconds: 600
      },
      target_ton: 8.5
    }
  ],
  isLoading: false,
  isConnected: false,
  runningCount: 3,
  pendingCount: 0,
  completedToday: 0,
  error: null,
  socket: null,

  connect: () => {
    const existing = get().socket;
    if (existing) return;

    const socket = io(SOCKET_URL, {
      path: '/socket.io',
      transports: ['websocket'],
    });

    socket.on('connect', () => set({ isConnected: true }));
    socket.on('disconnect', () => set({ isConnected: false }));

    socket.on('task_event', (event: TaskEvent) => {
      set((state) => {
        const idx = state.tasks.findIndex((t) => t.task_id === event.task_id);
        let newTasks: TaskInfo[];
        if (idx >= 0) {
          newTasks = [...state.tasks];
          newTasks[idx] = {
            ...newTasks[idx],
            status: event.status,
            progress: event.progress ?? newTasks[idx].progress,
            result: event.result ?? newTasks[idx].result,
          };
        } else {
          taskApi.listTasks().then((res) => {
            set({ tasks: res.tasks, ...updateCounts(res.tasks) });
          });
          newTasks = state.tasks;
        }
        return { tasks: newTasks, ...updateCounts(newTasks) };
      });
    });

    set({ socket });
  },

  disconnect: () => {
    const socket = get().socket;
    if (socket) {
      socket.disconnect();
      set({ socket: null, isConnected: false });
    }
  },

  refreshTasks: async () => {
    set({ isLoading: true, error: null });
    try {
      const res = await taskApi.listTasks();
      set({ tasks: res.tasks, ...updateCounts(res.tasks), isLoading: false });
    } catch (e) {
      const msg = e instanceof Error ? e.message : '获取任务列表失败';
      set({ error: msg, isLoading: false });
    }
  },

  createTask: async (request) => {
    set({ error: null });
    try {
      const task = await taskApi.createTask(request);
      set((state) => {
        const newTasks = [...state.tasks, task];
        return { tasks: newTasks, ...updateCounts(newTasks) };
      });
      return task;
    } catch (e) {
      const msg = e instanceof Error ? e.message : '创建任务失败';
      set({ error: msg });
      throw new Error(msg);
    }
  },

  startTask: async (taskId) => {
    set({ error: null });
    try {
      const updated = await taskApi.startTask(taskId);
      set((state) => {
        const newTasks = state.tasks.map((t) => (t.task_id === taskId ? updated : t));
        return { tasks: newTasks, ...updateCounts(newTasks) };
      });
    } catch (e) {
      const msg = e instanceof Error ? e.message : '启动任务失败';
      set({ error: msg });
    }
  },

  pauseTask: async (taskId) => {
    set({ error: null });
    try {
      const updated = await taskApi.pauseTask(taskId);
      set((state) => {
        const newTasks = state.tasks.map((t) => (t.task_id === taskId ? updated : t));
        return { tasks: newTasks, ...updateCounts(newTasks) };
      });
    } catch (e) {
      const msg = e instanceof Error ? e.message : '暂停任务失败';
      set({ error: msg });
    }
  },

  resumeTask: async (taskId) => {
    set({ error: null });
    try {
      const updated = await taskApi.resumeTask(taskId);
      set((state) => {
        const newTasks = state.tasks.map((t) => (t.task_id === taskId ? updated : t));
        return { tasks: newTasks, ...updateCounts(newTasks) };
      });
    } catch (e) {
      const msg = e instanceof Error ? e.message : '恢复任务失败';
      set({ error: msg });
    }
  },

  stopTask: async (taskId) => {
    set({ error: null });
    try {
      const updated = await taskApi.stopTask(taskId);
      set((state) => {
        const newTasks = state.tasks.map((t) => (t.task_id === taskId ? updated : t));
        return { tasks: newTasks, ...updateCounts(newTasks) };
      });
    } catch (e) {
      const msg = e instanceof Error ? e.message : '停止任务失败';
      set({ error: msg });
    }
  },

  deleteTask: async (taskId) => {
    set({ error: null });
    try {
      await taskApi.deleteTask(taskId);
      set((state) => {
        const newTasks = state.tasks.filter((t) => t.task_id !== taskId);
        return { tasks: newTasks, ...updateCounts(newTasks) };
      });
    } catch (e) {
      const msg = e instanceof Error ? e.message : '删除任务失败';
      set({ error: msg });
    }
  },

  clearError: () => set({ error: null }),
}));
