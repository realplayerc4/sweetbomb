import { useEffect } from 'react';
import { Activity, Wifi, WifiOff, Bell, Users, User } from 'lucide-react';
import { useSystemStore } from '@/stores/useSystemStore';
import { useRobotStore } from '@/stores/useRobotStore';
import { useSystemModeStore } from '@/stores/useSystemModeStore';
import { cn } from '@/lib/utils';

export function Header() {
  const currentTime = useSystemStore((s) => s.currentTime);
  const alertCount = useSystemStore((s) => s.stats.alert_count);
  const isOffline = useSystemStore((s) => s.isOffline);
  const setOffline = useSystemStore((s) => s.setOffline);
  const isConnected = useRobotStore((s) => s.isConnected);
  const monitorMode = useSystemModeStore((s) => s.monitorMode);
  const setMonitorMode = useSystemModeStore((s) => s.setMonitorMode);
  const selectedRobot = useSystemModeStore((s) => s.selectedRobot);
  const setSelectedRobot = useSystemModeStore((s) => s.setSelectedRobot);
  const devices = useSystemStore((s) => s.devices);

  useEffect(() => {
    const handleOnline = () => setOffline(false);
    const handleOffline = () => setOffline(true);
    window.addEventListener('online', handleOnline);
    window.addEventListener('offline', handleOffline);
    return () => {
      window.removeEventListener('online', handleOnline);
      window.removeEventListener('offline', handleOffline);
    };
  }, [setOffline]);

  return (
    <header className="h-16 flex items-center justify-between px-5 border-b border-cyan-500/15 bg-bg-secondary/60 backdrop-blur-md z-50 shrink-0">
      <div className="flex items-center gap-5">
        <div className="flex items-center gap-3">
          <div className="w-10 h-10 rounded-md bg-cyan-500/15 border border-cyan-500/30 flex items-center justify-center shadow-[0_0_15px_rgba(0,212,255,0.15)]">
            <Activity className="w-5 h-5 text-cyan-400" />
          </div>
          <div>
            <h1 className="text-[16px] font-bold tracking-wide text-text-primary tech-glow-text">
              制糖基地智能调度系统
            </h1>
            <p className="text-[12px] text-text-muted tracking-[0.2em]">SUGAR PRODUCTION SMART DISPATCH</p>
          </div>
        </div>
      </div>

      <div className="flex items-center gap-4">
        <div className="flex items-center gap-1 bg-bg-primary/60 border border-cyan-500/20 rounded-md p-0.5">
          <button
            onClick={() => setMonitorMode('single')}
            className={cn(
              'flex items-center gap-1.5 px-3 py-1.5 rounded text-[12px] font-bold tracking-wider transition-all',
              monitorMode === 'single'
                ? 'bg-cyan-500/20 text-cyan-400 shadow-[0_0_8px_rgba(0,212,255,0.2)]'
                : 'text-text-muted hover:text-text-secondary'
            )}
          >
            <User className="w-4 h-4" />
            单机器人
          </button>
          <button
            onClick={() => setMonitorMode('all')}
            className={cn(
              'flex items-center gap-1.5 px-3 py-1.5 rounded text-[12px] font-bold tracking-wider transition-all',
              monitorMode === 'all'
                ? 'bg-cyan-500/20 text-cyan-400 shadow-[0_0_8px_rgba(0,212,255,0.2)]'
                : 'text-text-muted hover:text-text-secondary'
            )}
          >
            <Users className="w-4 h-4" />
            所有机器人
          </button>
        </div>

        {monitorMode === 'single' && (
          <select
            value={selectedRobot}
            onChange={(e) => setSelectedRobot(e.target.value)}
            className="h-9 bg-bg-primary/60 border border-cyan-500/20 rounded-md px-3 text-[13px] text-text-secondary focus:outline-none focus:border-cyan-500/40 appearance-none cursor-pointer min-w-[150px]"
          >
            {devices.map((d) => (
              <option key={d.device_id} value={d.device_id}>
                {d.name} ({d.status === 'online' ? '在线' : d.status === 'warning' ? '告警' : '离线'})
              </option>
            ))}
          </select>
        )}

        <div className="flex items-center gap-2 px-3 py-1.5 rounded-md bg-bg-primary/60 border border-cyan-500/15">
          {isOffline ? (
            <WifiOff className="w-4 h-4 text-tech-red" />
          ) : isConnected ? (
            <Wifi className="w-4 h-4 text-tech-green" />
          ) : (
            <WifiOff className="w-4 h-4 text-tech-red" />
          )}
          <span className={cn(
            'text-[12px] font-bold tracking-wider',
            isOffline ? 'text-tech-red' : isConnected ? 'text-tech-green' : 'text-tech-red'
          )}>
            {isOffline ? '网络离线' : isConnected ? '系统在线' : '系统离线'}
          </span>
        </div>

        <button className="relative p-2 rounded-md hover:bg-white/5 transition-colors">
          <Bell className="w-5 h-5 text-text-secondary" />
          {alertCount > 0 && (
            <span className="absolute -top-1 -right-1 w-6 h-6 bg-tech-red rounded-full text-[11px] font-bold text-white flex items-center justify-center shadow-[0_0_8px_rgba(245,34,45,0.6)]">
              {alertCount > 99 ? '99+' : alertCount}
            </span>
          )}
        </button>

        <div className="text-right">
          <div className="text-[14px] font-mono text-text-primary tracking-wider">{currentTime.split(' ')[1] || currentTime}</div>
          <div className="text-[12px] text-text-muted font-mono">{currentTime.split(' ')[0] || ''}</div>
        </div>
      </div>
    </header>
  );
}
