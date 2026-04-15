import { useSystemStore } from '@/stores/useSystemStore';
import { getStatusColor, getStatusLabel, cn, formatTime } from '@/lib/utils';
import { Battery } from 'lucide-react';

interface DeviceListProps {
  compact?: boolean;
}

export function DeviceList({ compact = false }: DeviceListProps) {
  const devices = useSystemStore((s) => s.devices);

  return (
    <div className="h-full flex flex-col">
      <div className="flex items-center justify-between mb-3">
        <span className="text-[13px] font-bold text-cyan-400 tracking-wider">设备列表</span>
        <span className="text-[11px] text-text-muted font-mono">
          {devices.filter((d) => d.status === 'online').length}/{devices.length} ONLINE
        </span>
      </div>

      <div className="flex-1 overflow-y-auto">
        <table className="w-full text-[13px]">
          <thead>
            <tr className="text-text-muted border-b border-cyan-500/15">
              <th className="text-left py-1.5 font-semibold">设备</th>
              {!compact && <th className="text-center py-1.5 font-semibold">状态</th>}
              <th className="text-center py-1.5 font-semibold">电量</th>
              {!compact && <th className="text-right py-1.5 font-semibold">最后心跳</th>}
            </tr>
          </thead>
          <tbody>
            {devices.map((device) => (
              <tr
                key={device.device_id}
                className="border-b border-cyan-500/8 hover:bg-cyan-500/5 transition-colors"
              >
                <td className="py-2">
                  <div className="flex items-center gap-2">
                    <div
                      className="status-dot shrink-0"
                      style={{
                        background: getStatusColor(device.status),
                        boxShadow: `0 0 6px ${getStatusColor(device.status)}60`,
                      }}
                    />
                    <span className="text-text-primary font-semibold">{device.name}</span>
                  </div>
                </td>
                {!compact && (
                  <td className="text-center py-2">
                    <span
                      className="px-2 py-0.5 rounded text-[11px] font-bold"
                      style={{
                        color: getStatusColor(device.status),
                        background: `${getStatusColor(device.status)}15`,
                      }}
                    >
                      {getStatusLabel(device.status)}
                    </span>
                  </td>
                )}
                <td className="py-2">
                  <div className="flex items-center justify-center gap-1">
                    <Battery
                      className="w-3.5 h-3.5"
                      style={{ color: device.battery > 40 ? '#52c41a' : device.battery > 20 ? '#fa8c16' : '#f5222d' }}
                    />
                    <span
                      className="font-mono font-bold"
                      style={{ color: device.battery > 40 ? '#52c41a' : device.battery > 20 ? '#fa8c16' : '#f5222d' }}
                    >
                      {device.battery}%
                    </span>
                  </div>
                </td>
                {!compact && (
                  <td className="text-right py-2 text-text-muted font-mono text-[11px]">
                    {formatTime(device.last_heartbeat)}
                  </td>
                )}
              </tr>
            ))}
          </tbody>
        </table>
      </div>
    </div>
  );
}
