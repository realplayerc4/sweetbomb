import { useState } from 'react';
import { useSystemStore } from '@/stores/useSystemStore';
import { useTaskStore } from '@/stores/useTaskStore';
import { getStatusColor, cn } from '@/lib/utils';
import { RegisterLoaderModal } from '@/components/modals/RegisterLoaderModal';
import { DeviceDetailModal } from '@/components/modals/DeviceDetailModal';
import { Battery, Wrench, Clock, AlertTriangle, Truck, Gauge, Package, ListChecks } from 'lucide-react';
import type { DeviceInfo, DeviceWorkState } from '@/types';

export function WorkingDevices() {
  const devices = useSystemStore((s) => s.devices);
  const tasks = useTaskStore((s) => s.tasks);
  const [selectedDevice, setSelectedDevice] = useState<DeviceInfo | null>(null);
  const [detailOpen, setDetailOpen] = useState(false);

  const workingDevices = devices.filter((d) => d.work_state === 'working');

  const getDeviceTask = (deviceId: string) => {
    return tasks.find((t) => t.device_id === deviceId && (t.status === 'running' || t.status === 'paused'));
  };

  const getTaskTypeName = (taskType: string) => {
    const taskTypeMap: Record<string, string> = {
      'sugar_harvest': '铲糖作业',
      'navigation': '导航任务',
      'inspection': '巡检任务',
      'charging': '充电任务'
    };
    return taskTypeMap[taskType] || taskType;
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
            <span className="text-[12px] font-bold text-tech-green">工作中设备</span>
            <span className="text-[10px] text-text-muted font-mono">{workingDevices.length}</span>
          </div>
        </div>
        <RegisterLoaderModal />
      </div>

      <div className="flex-1 overflow-y-auto space-y-2">
        {workingDevices.length === 0 ? (
          <div className="h-full flex items-center justify-center py-8">
            <span className="text-[14px] text-text-muted">暂无工作中设备</span>
          </div>
        ) : (
          workingDevices.map((device) => {
            const deviceTask = getDeviceTask(device.device_id);
            const speed = (device.spec?.max_speed || 0) * 60; // 转换为米/分钟
            const capacity = device.spec?.bucket_capacity || 0;
            const taskProgress = deviceTask?.progress;
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
                      <div className="text-[12px] font-bold text-cyan-400 font-mono">{speed.toFixed(0)} 米/分钟</div>
                    </div>
                  </div>
                  <div className="flex items-center gap-1.5">
                    <Package className="w-3.5 h-3.5 text-tech-orange shrink-0" />
                    <div>
                      <div className="text-[10px] text-text-muted">装载量</div>
                      <div className="text-[12px] font-bold text-tech-orange font-mono">{capacity.toFixed(2)} 吨</div>
                    </div>
                  </div>
                </div>
                
                {deviceTask && (
                  <div className="space-y-1">
                    <div className="flex items-center gap-1.5">
                      <ListChecks className="w-3.5 h-3.5 text-tech-green shrink-0" />
                      <div className="flex-1">
                        <div className="text-[10px] text-text-muted">当前任务</div>
                        <div className="text-[12px] font-bold text-tech-green">{getTaskTypeName(deviceTask.task_type)}</div>
                      </div>
                      {taskProgress && (
                        <div className="text-[11px] text-cyan-400 font-mono">{taskProgress.current_step}/{taskProgress.total_steps}</div>
                      )}
                    </div>
                    {taskProgress && taskProgress.percentage > 0 && (
                      <div className="h-1.5 bg-bg-primary/60 rounded-full overflow-hidden">
                        <div
                          className="h-full bg-gradient-to-r from-cyan-500 to-tech-blue rounded-full transition-all duration-300"
                          style={{ width: `${taskProgress.percentage}%` }}
                        />
                      </div>
                    )}
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
