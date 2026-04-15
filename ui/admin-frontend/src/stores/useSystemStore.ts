import { create } from 'zustand';
import { useRobotStore } from './useRobotStore';
import type { AlertInfo, AlertLevel, SystemStats, DeviceInfo, ZoneInfo, MaintenanceRecord, LoaderRegistration, DeviceWorkState } from '../types';

interface SystemStore {
  stats: SystemStats;
  devices: DeviceInfo[];
  alerts: AlertInfo[];
  zones: ZoneInfo[];
  maintenanceRecords: MaintenanceRecord[];
  currentTime: string;
  isOffline: boolean;

  setStats: (stats: Partial<SystemStats>) => void;
  setDevices: (devices: DeviceInfo[]) => void;
  registerDevice: (registration: LoaderRegistration) => void;
  updateDeviceWorkState: (deviceId: string, state: DeviceWorkState) => void;
  addAlert: (alert: Omit<AlertInfo, 'id' | 'timestamp' | 'acknowledged'>) => void;
  acknowledgeAlert: (id: string) => void;
  clearAlerts: () => void;
  addZone: (zone: Omit<ZoneInfo, 'id'>) => void;
  updateZone: (id: string, updates: Partial<ZoneInfo>) => void;
  removeZone: (id: string) => void;
  addMaintenanceRecord: (record: Omit<MaintenanceRecord, 'id'>) => void;
  setOffline: (offline: boolean) => void;
  updateTime: () => void;
}

let alertIdCounter = 0;
let zoneIdCounter = 0;
let maintenanceIdCounter = 0;

export const useSystemStore = create<SystemStore>((set) => ({
  stats: {
    total_devices: 6,
    online_devices: 4,
    running_tasks: 3,
    pending_tasks: 5,
    completed_today: 12,
    alert_count: 2,
    avg_response_time: 1.2,
    uptime_hours: 72.5,
    idle_devices: 2,
    working_devices: 3,
    fault_devices: 1,
    today_sugar_ton: 15.8,
  },
  devices: [
    { device_id: 'loader-001', name: '1号装载机', status: 'online', work_state: 'working', battery: 85, position: [22.5431, 108.3743], last_heartbeat: new Date().toISOString(), task_id: 'task-001', spec: { model: 'ZL-50', bucket_capacity: 0.85, max_speed: 0.5, bucket_width: 0.8, max_load_weight: 5.0 }, total_hours: 1280, maintenance_count: 15, energy_consumption: 12.5 },
    { device_id: 'loader-002', name: '2号装载机', status: 'online', work_state: 'working', battery: 72, position: [22.5435, 108.3747], last_heartbeat: new Date().toISOString(), task_id: 'task-002', spec: { model: 'ZL-50', bucket_capacity: 0.85, max_speed: 0.5, bucket_width: 0.8, max_load_weight: 5.0 }, total_hours: 960, maintenance_count: 10, energy_consumption: 10.2 },
    { device_id: 'loader-003', name: '3号装载机', status: 'online', work_state: 'idle', battery: 91, position: [22.5428, 108.3750], last_heartbeat: new Date().toISOString(), spec: { model: 'ZL-30', bucket_capacity: 0.6, max_speed: 0.4, bucket_width: 0.6, max_load_weight: 3.0 }, total_hours: 540, maintenance_count: 6, energy_consumption: 8.1 },
    { device_id: 'loader-004', name: '4号装载机', status: 'warning', work_state: 'fault', battery: 35, position: [22.5440, 108.3740], last_heartbeat: new Date().toISOString(), task_id: 'task-003', spec: { model: 'ZL-50', bucket_capacity: 0.85, max_speed: 0.5, bucket_width: 0.8, max_load_weight: 5.0 }, total_hours: 2100, maintenance_count: 22, energy_consumption: 15.3 },
    { device_id: 'loader-005', name: '5号装载机', status: 'offline', work_state: 'fault', battery: 0, position: [22.5420, 108.3735], last_heartbeat: new Date(Date.now() - 3600000).toISOString(), spec: { model: 'ZL-30', bucket_capacity: 0.6, max_speed: 0.4, bucket_width: 0.6, max_load_weight: 3.0 }, total_hours: 3200, maintenance_count: 35, energy_consumption: 0 },
    { device_id: 'loader-006', name: '6号装载机', status: 'online', work_state: 'working', battery: 68, position: [22.5438, 108.3755], last_heartbeat: new Date().toISOString(), task_id: 'task-004', spec: { model: 'ZL-50', bucket_capacity: 0.85, max_speed: 0.5, bucket_width: 0.8, max_load_weight: 5.0 }, total_hours: 780, maintenance_count: 8, energy_consumption: 11.0 },
  ],
  alerts: [
    { id: 'alert-1', level: 'warning' as AlertLevel, source: 'loader-004', message: '4号装载机电量低于40%，建议充电', timestamp: new Date().toISOString(), acknowledged: false, category: 'device' },
    { id: 'alert-2', level: 'error' as AlertLevel, source: 'loader-005', message: '5号装载机通信中断，已离线超过1小时', timestamp: new Date(Date.now() - 3600000).toISOString(), acknowledged: false, category: 'device' },
  ],
  zones: [
    { id: 'zone-1', name: 'A区-装卸作业区', type: 'loading' as const, bounds: [[22.5425, 108.3735], [22.5445, 108.3760]], color: '#1677ff', enabled: true },
    { id: 'zone-2', name: 'B区-糖堆存放区', type: 'sugar_pile' as const, bounds: [[22.5445, 108.3735], [22.5455, 108.3760]], color: '#52c41a', enabled: true },
    { id: 'zone-3', name: 'C区-充电维护区', type: 'charging' as const, bounds: [[22.5415, 108.3735], [22.5425, 108.3760]], color: '#fa8c16', enabled: true },
  ],
  maintenanceRecords: [
    { id: 'mt-1', device_id: 'loader-001', type: 'routine', description: '定期保养-更换液压油', date: new Date(Date.now() - 86400000 * 3).toISOString(), operator: '张工', cost: 1200 },
    { id: 'mt-2', device_id: 'loader-004', type: 'repair', description: '液压系统泄漏维修', date: new Date(Date.now() - 86400000).toISOString(), operator: '李工', cost: 3500 },
  ],
  currentTime: new Date().toLocaleString('zh-CN'),
  isOffline: false,

  setStats: (stats) => set((s) => ({ stats: { ...s.stats, ...stats } })),

  setDevices: (devices) => set({ devices }),

  registerDevice: (registration) => set((s) => {
    const newId = `loader-${String(s.devices.length + 1).padStart(3, '0')}`;
    const newDevice: DeviceInfo = {
      device_id: newId,
      name: registration.name,
      status: 'offline',
      work_state: 'idle',
      battery: 0,
      position: [22.5430, 108.3745],
      last_heartbeat: new Date().toISOString(),
      spec: {
        model: registration.model,
        bucket_capacity: registration.bucket_capacity,
        max_speed: registration.max_speed,
        bucket_width: registration.bucket_width,
        max_load_weight: registration.max_load_weight,
      },
      total_hours: 0,
      maintenance_count: 0,
      energy_consumption: 0,
    };
    return {
      devices: [...s.devices, newDevice],
      stats: { ...s.stats, total_devices: s.stats.total_devices + 1 },
    };
  }),

  updateDeviceWorkState: (deviceId, state) => set((s) => ({
    devices: s.devices.map((d) => d.device_id === deviceId ? { ...d, work_state: state } : d),
  })),

  addAlert: (alert) => {
    alertIdCounter++;
    const newAlert: AlertInfo = {
      ...alert,
      id: `alert-${alertIdCounter}`,
      timestamp: new Date().toISOString(),
      acknowledged: false,
    };
    set((s) => ({
      alerts: [newAlert, ...s.alerts].slice(0, 50),
      stats: { ...s.stats, alert_count: s.stats.alert_count + 1 },
    }));
  },

  acknowledgeAlert: (id) => set((s) => ({
    alerts: s.alerts.map((a) => (a.id === id ? { ...a, acknowledged: true } : a)),
    stats: { ...s.stats, alert_count: Math.max(0, s.stats.alert_count - 1) },
  })),

  clearAlerts: () => set((s) => ({ alerts: [], stats: { ...s.stats, alert_count: 0 } })),

  addZone: (zone) => {
    zoneIdCounter++;
    set((s) => ({ zones: [...s.zones, { ...zone, id: `zone-${zoneIdCounter}` }] }));
  },

  updateZone: (id, updates) => set((s) => ({
    zones: s.zones.map((z) => z.id === id ? { ...z, ...updates } : z),
  })),

  removeZone: (id) => set((s) => ({
    zones: s.zones.filter((z) => z.id !== id),
  })),

  addMaintenanceRecord: (record) => {
    maintenanceIdCounter++;
    set((s) => ({
      maintenanceRecords: [{ ...record, id: `mt-${maintenanceIdCounter}` }, ...s.maintenanceRecords],
    }));
  },

  setOffline: (offline) => set({ isOffline: offline }),

  updateTime: () => set({ currentTime: new Date().toLocaleString('zh-CN') }),
}));
