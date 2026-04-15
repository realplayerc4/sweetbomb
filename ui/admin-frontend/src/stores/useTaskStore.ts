import { create } from 'zustand';
import { robotApi } from '../services/robotApi';

/**
 * 任务状态
 * - idle: 空闲，无任务
 * - nav_to_pick: 导航到取货点（含取货）
 * - nav_to_drop: 导航到卸货点（含卸货）
 * - paused: 暂停
 * - completed: 任务完成
 * - error: 错误
 */
type TaskPhase = 'idle' | 'nav_to_pick' | 'nav_to_drop' | 'paused' | 'completed' | 'error';

interface TaskStore {
  // 任务配置
  targetKg: number;           // 目标重量 (kg)
  perCycleKg: number;         // 每次循环重量 (kg)，默认 30kg
  totalCycles: number;        // 总循环次数
  currentCycle: number;       // 当前循环次数

  // 任务状态
  phase: TaskPhase;           // 当前阶段
  isRunning: boolean;         // 是否正在运行
  completedKg: number;        // 已完成重量 (kg)

  // 错误信息
  error: string | null;

  // Actions
  setTarget: (kg: number) => void;
  start: () => Promise<void>;
  pause: () => Promise<void>;
  resume: () => Promise<void>;
  stop: () => Promise<void>;
  clearError: () => void;

  // 内部方法
  _runCycle: () => Promise<void>;
  _waitForRobotIdle: (timeoutMs: number) => Promise<boolean>;
}

export const useTaskStore = create<TaskStore>((set, get) => ({
  // 默认配置
  targetKg: 900,
  perCycleKg: 30,
  totalCycles: 30,
  currentCycle: 0,

  // 初始状态
  phase: 'idle',
  isRunning: false,
  completedKg: 0,
  error: null,

  // 设置目标重量，自动计算循环次数
  setTarget: (kg: number) => {
    const perCycle = get().perCycleKg;
    const cycles = Math.ceil(kg / perCycle);
    set({
      targetKg: kg,
      totalCycles: cycles,
      completedKg: 0,
      currentCycle: 0,
      phase: 'idle',
    });
  },

  // 启动任务
  start: async () => {
    const { isRunning, currentCycle, totalCycles } = get();
    if (isRunning) return;

    // 如果已完成所有循环
    if (currentCycle >= totalCycles) {
      set({ phase: 'completed', isRunning: false });
      return;
    }

    set({ isRunning: true, error: null });

    // 开始执行循环
    get()._runCycle();
  },

  // 暂停任务
  pause: async () => {
    const { isRunning } = get();
    if (!isRunning) return;

    try {
      await robotApi.pause();
      set({ phase: 'paused', isRunning: false });
    } catch (e) {
      const msg = e instanceof Error ? e.message : '暂停失败';
      set({ error: msg });
    }
  },

  // 恢复任务
  resume: async () => {
    const { phase } = get();
    if (phase !== 'paused') return;

    set({ isRunning: true, error: null });

    try {
      await robotApi.resume();
      get()._runCycle();
    } catch (e) {
      const msg = e instanceof Error ? e.message : '恢复失败';
      set({ error: msg, isRunning: false });
    }
  },

  // 停止任务
  stop: async () => {
    try {
      await robotApi.stop();
    } catch {
      // 忽略错误
    }
    set({
      isRunning: false,
      phase: 'idle',
      currentCycle: 0,
      completedKg: 0,
    });
  },

  clearError: () => set({ error: null }),

  // 等待机器人变为空闲状态
  _waitForRobotIdle: async (timeoutMs: number) => {
    const startTime = Date.now();

    return new Promise<boolean>((resolve) => {
      const checkInterval = setInterval(async () => {
        // 检查是否暂停或停止
        if (!get().isRunning) {
          clearInterval(checkInterval);
          resolve(false);
          return;
        }

        // 检查超时
        if (Date.now() - startTime > timeoutMs) {
          clearInterval(checkInterval);
          set({ error: '等待超时' });
          resolve(false);
          return;
        }

        // 检查机器人状态
        try {
          const status = await robotApi.getStatus();
          if (status.status === 'idle') {
            clearInterval(checkInterval);
            resolve(true);
          }
        } catch {
          // 忽略错误，继续等待
        }
      }, 500); // 每500ms检查一次
    });
  },

  // 执行循环任务
  // 简化流程：nav-pick 已包含取货，nav-drop 已包含卸货
  _runCycle: async () => {
    const { totalCycles, perCycleKg } = get();

    while (get().isRunning) {
      const cycleNum = get().currentCycle + 1;

      // 检查是否完成所有循环
      if (cycleNum > totalCycles) {
        set({ phase: 'completed', isRunning: false });
        return;
      }

      try {
        console.log(`[Task] 开始第 ${cycleNum}/${totalCycles} 次循环`);

        // 阶段1: 导航到取货点并取货（nav-pick 已包含取货动作）
        set({ phase: 'nav_to_pick' });
        console.log('[Task] 导航到取货点并取货...');
        await robotApi.navToPick();

        // 等待完成
        const pickDone = await get()._waitForRobotIdle(180000); // 最多等待3分钟
        if (!pickDone || !get().isRunning) return;

        // 阶段2: 导航到卸货点并卸货（nav-drop 已包含卸货动作）
        set({ phase: 'nav_to_drop' });
        console.log('[Task] 导航到卸货点并卸货...');
        await robotApi.navToDrop();

        // 等待完成
        const dropDone = await get()._waitForRobotIdle(180000);
        if (!dropDone || !get().isRunning) return;

        // 完成一次循环
        const newCompletedKg = get().completedKg + perCycleKg;
        const newCycle = get().currentCycle + 1;
        set({
          currentCycle: newCycle,
          completedKg: newCompletedKg,
          phase: 'idle',
        });

        console.log(`[Task] 完成第 ${newCycle}/${totalCycles} 次循环，已装载 ${newCompletedKg}kg`);

      } catch (e) {
        const msg = e instanceof Error ? e.message : '任务执行失败';
        console.error('[Task] 错误:', msg);
        set({ error: msg, phase: 'error', isRunning: false });
        return;
      }
    }
  },
}));
