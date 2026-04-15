import { useState } from 'react';
import { useSystemStore } from '@/stores/useSystemStore';
import { getStatusColor, cn } from '@/lib/utils';
import { RegisterLoaderModal } from '@/components/modals/RegisterLoaderModal';
import { DeviceDetailModal } from '@/components/modals/DeviceDetailModal';
import { Battery, Wrench, Clock, AlertTriangle, Truck } from 'lucide-react';
import type { DeviceInfo, DeviceWorkState } from '@/types';

type CategoryKey = 'idle' | 'working' | 'fault' | 'charging';

const CATEGORIES: { key: CategoryKey; label: string; color: string; icon: React.ElementType }[] = [
  { key: 'idle', label: '空闲', color: '#1677ff', icon: Clock },
  { key: 'working', label: '作业中', color: '#52c41a', icon: Truck },
  { key: 'fault', label: '故障', color: '#f5222d', icon: AlertTriangle },
  { key: 'charging', label: '充电中', color: '#fa8c16', icon: Wrench },
];

export function DeviceOverview() {
  const devices = useSystemStore((s) => s.devices);
  const [selectedDevice, setSelectedDevice] = useState<DeviceInfo | null>(null);
  const [detailOpen, setDetailOpen] = useState(false);

  const categorized = {
    idle: devices.filter((d) => d.work_state === 'idle'),
    working: devices.filter((d) => d.work_state === 'working'),
    fault: devices.filter((d) => d.work_state === 'fault'),
    charging: devices.filter((d) => d.work_state === 'charging'),
  };

  const handleDeviceClick = (device: DeviceInfo) => {
    setSelectedDevice(device);
    setDetailOpen(true);
  };

  return (
    <div className="h-full flex flex-col">
      <div className="flex items-center justify-between mb-2">
        <div className="flex items-center gap-2">
          {CATEGORIES.map((cat) => {
            const Icon = cat.icon;
            const count = categorized[cat.key].length;
            return (
              <div key={cat.key} className="flex items-center gap-1">
                <Icon className="w-3 h-3" style={{ color: cat.color }} />
                <span className="text-[11px] font-bold" style={{ color: cat.color }}>{count}</span>
              </div>
            );
          })}
        </div>
        <RegisterLoaderModal />
      </div>

      <div className="flex-1 overflow-y-auto space-y-1.5">
        {CATEGORIES.map((cat) => {
          const catDevices = categorized[cat.key];
          if (catDevices.length === 0) return null;
          return (
            <div key={cat.key}>
              <div className="flex items-center gap-1.5 mb-1">
                <div className="w-1.5 h-1.5 rounded-full" style={{ background: cat.color, boxShadow: `0 0 4px ${cat.color}60` }} />
                <span className="text-[10px] font-bold tracking-wider" style={{ color: cat.color }}>{cat.label}</span>
                <span className="text-[9px] text-text-muted font-mono">{catDevices.length}</span>
              </div>
              <div className="space-y-1 ml-2.5">
                {catDevices.map((device) => (
                  <div
                    key={device.device_id}
                    onClick={() => handleDeviceClick(device)}
                    className="tech-card px-2.5 py-2 flex items-center gap-2 cursor-pointer hover:border-cyan-500/30 transition-all group"
                  >
                    <div
                      className="status-dot shrink-0"
                      style={{
                        background: getStatusColor(device.status),
                        boxShadow: `0 0 6px ${getStatusColor(device.status)}60`,
                      }}
                    />
                    <span className="text-[12px] font-semibold text-text-primary flex-1 truncate">{device.name}</span>
                    <div className="flex items-center gap-1">
                      <Battery
                        className="w-3 h-3"
                        style={{ color: device.battery > 40 ? '#52c41a' : device.battery > 20 ? '#fa8c16' : '#f5222d' }}
                      />
                      <span
                        className="text-[10px] font-mono font-bold"
                        style={{ color: device.battery > 40 ? '#52c41a' : device.battery > 20 ? '#fa8c16' : '#f5222d' }}
                      >
                        {device.battery}%
                      </span>
                    </div>
                    {device.spec && (
                      <span className="text-[9px] text-text-muted font-mono">{device.spec.model}</span>
                    )}
                  </div>
                ))}
              </div>
            </div>
          );
        })}
      </div>

      <DeviceDetailModal
        device={selectedDevice}
        open={detailOpen}
        onClose={() => { setDetailOpen(false); setSelectedDevice(null); }}
      />
    </div>
  );
}
