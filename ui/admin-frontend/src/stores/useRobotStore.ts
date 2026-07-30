import { create } from 'zustand';
import { io, type Socket } from 'socket.io-client';
import { SOCKET_URL, HEARTBEAT_INTERVAL } from '../config';
import { robotApi } from '../services/robotApi';
import type { RobotState, MoveDirection, SugarHarvestConfig } from '../types';

interface RobotStore {
  status: RobotState;
  battery: number;
  position: [number, number, number];
  orientation: [number, number, number];
  imu: { roll: number; pitch: number; yaw: number };
  leftTrackSpeed: number;
  rightTrackSpeed: number;
  isConnected: boolean;
  isControllable: boolean;
  isHarvestRunning: boolean;
  harvestCycle: number;
  harvestMaxCycles: number;
  error: string | null;

  socket: Socket | null;

  connect: () => void;
  disconnect: () => void;
  refreshStatus: () => Promise<void>;
  move: (direction: MoveDirection, speed?: number, duration?: number) => Promise<void>;
  stop: () => Promise<void>;
  reset: () => Promise<void>;
  setServo: (servoId: string, angle: number) => Promise<void>;
  scoop: () => Promise<void>;
  dump: () => Promise<void>;
  dock: () => Promise<void>;
  startHarvest: (config: SugarHarvestConfig) => Promise<void>;
  stopHarvest: () => Promise<void>;
  clearError: () => void;
}

export const useRobotStore = create<RobotStore>((set, get) => ({
  status: 'idle',
  battery: 85,
  position: [0, 0, 0],
  orientation: [0, 0, 0],
  imu: { roll: 0, pitch: 0, yaw: 0 },
  leftTrackSpeed: 0,
  rightTrackSpeed: 0,
  isConnected: false,
  isControllable: true,
  isHarvestRunning: false,
  harvestCycle: 0,
  harvestMaxCycles: 10,
  error: null,
  socket: null,

  connect: () => {
    const existing = get().socket;
    if (existing) return;

    const socket = io(SOCKET_URL, {
      path: '/socket.io',
      transports: ['websocket'],
    });

    socket.on('connect', () => {
      set({ isConnected: true });
      // 连接后立即获取状态
      get().refreshStatus();
    });

    socket.on('disconnect', () => {
      set({ isConnected: false });
    });

    // 监听机器人位置更新事件
    socket.on('robot_position_update', (data: { x: number; y: number; z: number; a: number }) => {
      set({
        position: [data.x, data.y, data.z],
        orientation: [data.a, 0, 0],
        imu: { roll: 0, pitch: 0, yaw: data.a },
      });
    });

    socket.on('metadata_update', (data: { system_stats?: { battery?: number; cpu_load?: number; temperature?: number; signal?: number; imu?: { roll: number; pitch: number; yaw: number } } }) => {
      if (data.system_stats) {
        const stats = data.system_stats;
        if (stats.battery !== undefined) set((s) => ({ battery: s.status === 'idle' ? s.battery : stats.battery ?? s.battery }));
        if (stats.imu) set({ imu: stats.imu });
      }
    });

    socket.on('bt_event', (event: { event_type: string; current_cycle?: number; max_cycles?: number }) => {
      if (event.event_type === 'bt_started') {
        set({ isHarvestRunning: true });
      } else if (event.event_type === 'bt_completed' || event.event_type === 'bt_cancelled') {
        set({ isHarvestRunning: false });
      }
      if (event.current_cycle !== undefined) set({ harvestCycle: event.current_cycle });
      if (event.max_cycles !== undefined) set({ harvestMaxCycles: event.max_cycles });
    });

    // Socket 心跳
    const heartbeat = setInterval(() => {
      if (socket.connected) socket.emit('ping');
    }, HEARTBEAT_INTERVAL);

    // HTTP 定时轮询机器人状态（每 1 秒）
    const pollInterval = setInterval(() => {
      if (socket.connected) {
        get().refreshStatus();
      }
    }, 1000);

    socket.on('disconnect', () => {
      clearInterval(heartbeat);
      clearInterval(pollInterval);
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

  refreshStatus: async () => {
    try {
      const status = await robotApi.getStatus();
      set({
        status: status.status as RobotState,
        battery: status.charge,
        // 后端返回的 x, y, z 单位是 mm，a 是角度（度）
        position: [status.x, status.y, status.z],
        orientation: [status.a, 0, 0],
        imu: { roll: 0, pitch: 0, yaw: status.a },
        leftTrackSpeed: status.speed,
        rightTrackSpeed: status.speed,
        isConnected: status.connected,
      });
    } catch {
      // silently fail - status will be updated via socket
    }
  },

  move: async (direction, speed = 0.5, duration = 1.0) => {
    try {
      set({ error: null });
      await robotApi.move(direction, speed, duration);
      await get().refreshStatus();
    } catch (e) {
      const msg = e instanceof Error ? e.message : '移动指令失败';
      set({ error: msg });
    }
  },

  stop: async () => {
    try {
      set({ error: null });
      await robotApi.stop();
      await get().refreshStatus();
    } catch (e) {
      const msg = e instanceof Error ? e.message : '停止指令失败';
      set({ error: msg });
    }
  },

  reset: async () => {
    try {
      set({ error: null });
      await robotApi.reset();
      await get().refreshStatus();
    } catch (e) {
      const msg = e instanceof Error ? e.message : '重置指令失败';
      set({ error: msg });
    }
  },

  setServo: async (servoId, angle) => {
    try {
      set({ error: null });
      await robotApi.setServo(servoId, angle);
    } catch (e) {
      const msg = e instanceof Error ? e.message : '伺服控制失败';
      set({ error: msg });
    }
  },

  scoop: async () => {
    try {
      set({ error: null });
      await robotApi.scoop();
      await get().refreshStatus();
    } catch (e) {
      const msg = e instanceof Error ? e.message : '铲取指令失败';
      set({ error: msg });
    }
  },

  dump: async () => {
    try {
      set({ error: null });
      await robotApi.dump();
      await get().refreshStatus();
    } catch (e) {
      const msg = e instanceof Error ? e.message : '倾倒指令失败';
      set({ error: msg });
    }
  },

  dock: async () => {
    try {
      set({ error: null });
      await robotApi.dock();
      await get().refreshStatus();
    } catch (e) {
      const msg = e instanceof Error ? e.message : '回桩指令失败';
      set({ error: msg });
    }
  },

  startHarvest: async (config) => {
    try {
      set({ error: null });
      await robotApi.startSugarHarvest(config);
      set({ isHarvestRunning: true });
    } catch (e) {
      const msg = e instanceof Error ? e.message : '启动铲糖循环失败';
      set({ error: msg });
    }
  },

  stopHarvest: async () => {
    try {
      set({ error: null });
      await robotApi.stopSugarHarvest();
      set({ isHarvestRunning: false });
    } catch (e) {
      const msg = e instanceof Error ? e.message : '停止铲糖循环失败';
      set({ error: msg });
    }
  },

  clearError: () => set({ error: null }),
}));
