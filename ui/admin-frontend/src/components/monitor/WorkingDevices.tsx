import { useState, useMemo } from 'react';
import { useSystemStore } from '@/stores/useSystemStore';
import { useRobotStore } from '@/stores/useRobotStore';
import { useTaskStore } from '@/stores/useTaskStore';
import { getStatusColor } from '@/lib/utils';
import { RegisterLoaderModal } from '@/components/modals/RegisterLoaderModal';
import { DeviceDetailModal } from '@/components/modals/DeviceDetailModal';
import { Battery, Truck, Gauge, Package, ListChecks } from 'lucide-react';
import type { DeviceInfo } from '@/types';

// 扩展的设备信息（包含实时数据）
interface ExtendedDeviceInfo extends DeviceInfo {
  actualSpeed?: number;
  harvestCycle?: number;
  harvestMaxCycles?: number;
  isHarvestRunning?: boolean;
}

export function WorkingDevices() {
  const devices = useSystemStore((s) => s.devices);
  // 任务状态
  const currentCycle = useTaskStore((s) => s.currentCycle);
  const totalCycles = useTaskStore((s) => s.totalCycles);
  const isTaskRunning = useTaskStore((s) => s.isRunning);
  const taskPhase = useTaskStore((s) => s.phase);

  const [selectedDevice, setSelectedDevice] = useState<DeviceInfo | null>(null);
  const [detailOpen, setDetailOpen] = useState(false);

  // 机器人实时状态
  const robotStatus = useRobotStore((s) => s.status);
  const robotBattery = useRobotStore((s) => s.battery);
  const robotSpeed = useRobotStore((s) => s.leftTrackSpeed);
  const isConnected = useRobotStore((s) => s.isConnected);

  // 获取所有设备（包括1号装载机）
  const allDevices: ExtendedDeviceInfo[] = useMemo(() => {
    const otherDevices = devices.filter((d) => d.device_id !== 'loader-001');

    if (isConnected) {
      const isRobotWorking = robotStatus === 'moving' || robotStatus === 'scooping' || robotStatus === 'dumping' || isTaskRunning;

      const loader1: ExtendedDeviceInfo = {
        device_id: 'loader-001',
        name: '1号装载机',
        status: 'online',
        work_state: isRobotWorking ? 'working' : 'idle',
        battery: robotBattery,
        position: [0, 0],
        last_heartbeat: new Date().toISOString(),
        spec: {
          model: 'ZL-50',
          bucket_capacity: 0.85,
          max_speed: 0.5,
          bucket_width: 0.8,
          max_load_weight: 5.0,
        },
        total_hours: 1280,
        maintenance_count: 15,
        energy_consumption: 12.5,
        actualSpeed: robotSpeed * 60,
        harvestCycle: currentCycle,
        harvestMaxCycles: totalCycles,
        isHarvestRunning: isTaskRunning,
      };

      return [loader1, ...otherDevices];
    }

    return otherDevices;
  }, [devices, isConnected, robotStatus, robotBattery, robotSpeed, isTaskRunning, currentCycle, totalCycles, taskPhase]);

  const getTaskTypeName = (taskType: string) => {
    const taskTypeMap: Record<string, string> = {
      'sugar_harvest': '铲糖作业',
      'navigation': '导航任务',
      'inspection': '巡检任务',
      'charging': '充电任务'
    };
    return taskTypeMap[taskType] || taskType;
  };

  // 获取机器人状态显示文本
  const getRobotStatusText = (status: string, running: boolean, cycle: number, maxCycles: number) => {
    if (running) {
      return `任务执行中 ${cycle}/${maxCycles}`;
    }
    const statusMap: Record<string, string> = {
      'idle': '空闲',
      'moving': '移动中',
      'scooping': '铲取中',
      'dumping': '倾倒中',
      'error': '故障',
      'emergency_stop': '紧急停止'
    };
    return statusMap[status] || status;
  };

  const handleDeviceClick = (device: DeviceInfo) => {
    setSelectedDevice(device);
    setDetailOpen(true);
  };

  return (
    <div className="h-full flex flex-col">
      <div className="flex items-center justify-between mb-3">
        <div className="flex items-center gap-2">
          <div className="flex items-center gap-1.5">
            <Truck className="w-4 h-4 text-tech-green" />
            <span className="text-[12px] font-bold text-tech-green">设备列表</span>
            <span className="text-[10px] text-text-muted font-mono">{allDevices.length}</span>
          </div>
        </div>
        <RegisterLoaderModal />
      </div>

      <div className="flex-1 overflow-y-auto space-y-2">
        {allDevices.length === 0 ? (
          <div className="h-full flex items-center justify-center py-8">
            <span className="text-[14px] text-text-muted">暂无设备</span>
          </div>
        ) : (
          allDevices.map((device) => {
            const isLoader1 = device.device_id === 'loader-001';
            const speed = device.actualSpeed ?? (device.spec?.max_speed || 0) * 60;
            const capacity = isLoader1 ? 0.03 : (device.spec?.bucket_capacity || 0);

            return (
              <div
                key={device.device_id}
                onClick={() => handleDeviceClick(device)}
                className="tech-card p-3.5 cursor-pointer hover:border-cyan-500/30 transition-all group"
              >
                <div className="flex items-center justify-between mb-2">
                  <div className="flex items-center gap-2">
                    <div
                      className="status-dot shrink-0"
                      style={{
                        background: getStatusColor(device.status),
                        boxShadow: `0 0 6px ${getStatusColor(device.status)}60`,
                      }}
                    />
                    <span className="text-[14px] font-semibold text-text-primary">{device.name}</span>
                    {isLoader1 && (
                      <span className="text-[10px] px-1.5 py-0.5 rounded bg-cyan-500/20 text-cyan-400 font-bold">
                        本机
                      </span>
                    )}
                    {device.spec && (
                      <span className="text-[11px] text-text-muted font-mono">{device.spec.model}</span>
                    )}
                  </div>
                  <div className="flex items-center gap-2">
                    <div className="flex items-center gap-1">
                      <Battery
                        className="w-4 h-4"
                        style={{ color: device.battery > 40 ? '#52c41a' : device.battery > 20 ? '#fa8c16' : '#f5222d' }}
                      />
                      <span
                        className="text-[11px] font-mono font-bold"
                        style={{ color: device.battery > 40 ? '#52c41a' : device.battery > 20 ? '#fa8c16' : '#f5222d' }}
                      >
                        {device.battery}%
                      </span>
                    </div>
                  </div>
                </div>

                <div className="grid grid-cols-2 gap-2 mb-2">
                  <div className="flex items-center gap-1.5">
                    <Gauge className="w-3.5 h-3.5 text-cyan-400 shrink-0" />
                    <div>
                      <div className="text-[10px] text-text-muted">移动速度</div>
                      <div className="text-[12px] font-bold text-cyan-400 font-mono">{speed.toFixed(1)} 米/分钟</div>
                    </div>
                  </div>
                  <div className="flex items-center gap-1.5">
                    <Package className="w-3.5 h-3.5 text-tech-orange shrink-0" />
                    <div>
                      <div className="text-[10px] text-text-muted">装载量</div>
                      <div className="text-[12px] font-bold text-tech-orange font-mono">
                        {isLoader1 ? `${(capacity * 1000).toFixed(0)} kg` : `${capacity.toFixed(2)} 吨`}
                      </div>
                    </div>
                  </div>
                </div>

                {/* 1号装载机显示实时状态 */}
                {isLoader1 ? (
                  <div className="space-y-1">
                    <div className="flex items-center gap-1.5">
                      <ListChecks className="w-3.5 h-3.5 text-tech-green shrink-0" />
                      <div className="flex-1">
                        <div className="text-[10px] text-text-muted">当前状态</div>
                        <div className="text-[12px] font-bold text-tech-green">
                          {getRobotStatusText(robotStatus, device.isHarvestRunning ?? false, device.harvestCycle ?? 0, device.harvestMaxCycles ?? 0)}
                        </div>
                      </div>
                      {device.isHarvestRunning && (
                        <div className="text-[11px] text-cyan-400 font-mono">{device.harvestCycle}/{device.harvestMaxCycles}</div>
                      )}
                    </div>
                    {device.isHarvestRunning && device.harvestMaxCycles && device.harvestMaxCycles > 0 && (
                      <div className="h-1.5 bg-bg-primary/60 rounded-full overflow-hidden">
                        <div
                          className="h-full bg-gradient-to-r from-cyan-500 to-tech-blue rounded-full transition-all duration-300"
                          style={{ width: `${((device.harvestCycle ?? 0) / device.harvestMaxCycles) * 100}%` }}
                        />
                      </div>
                    )}
                  </div>
                ) : (
                  <div className="space-y-1">
                    <div className="flex items-center gap-1.5">
                      <ListChecks className="w-3.5 h-3.5 text-text-muted shrink-0" />
                      <div className="flex-1">
                        <div className="text-[10px] text-text-muted">当前任务</div>
                        <div className="text-[12px] font-bold text-text-muted">空闲</div>
                      </div>
                    </div>
                  </div>
                )}
              </div>
            );
          })
        )}
      </div>

      <DeviceDetailModal
        device={selectedDevice}
        open={detailOpen}
        onClose={() => { setDetailOpen(false); setSelectedDevice(null); }}
      />
    </div>
  );
}
