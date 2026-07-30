import { create } from 'zustand';

type MonitorMode = 'single' | 'all';

interface SystemModeStore {
  monitorMode: MonitorMode;
  selectedRobot: string;
  setMonitorMode: (mode: MonitorMode) => void;
  setSelectedRobot: (id: string) => void;
}

export const useSystemModeStore = create<SystemModeStore>((set) => ({
  monitorMode: 'all',
  selectedRobot: 'loader-001',
  setMonitorMode: (mode) => set({ monitorMode: mode }),
  setSelectedRobot: (id) => set({ selectedRobot: id }),
}));
