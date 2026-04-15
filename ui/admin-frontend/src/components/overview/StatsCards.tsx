import { Truck, ListChecks, CheckCircle2, AlertTriangle, Clock, Activity, WifiOff } from 'lucide-react';
import { useSystemStore } from '@/stores/useSystemStore';
import { useTaskStore } from '@/stores/useTaskStore';
import { cn } from '@/lib/utils';

interface StatCard {
  label: string;
  value: number | string;
  unit?: string;
  icon: React.ElementType;
  color: string;
  trend?: string;
  subText?: string;
}

export function StatsCards() {
  const stats = useSystemStore((s) => s.stats);
  const isOffline = useSystemStore((s) => s.isOffline);
  const runningCount = useTaskStore((s) => s.runningCount);
  const pendingCount = useTaskStore((s) => s.pendingCount);
  const completedToday = useTaskStore((s) => s.completedToday);

  const primaryCards: StatCard[] = [
    {
      label: '在线设备',
      value: stats.online_devices,
      unit: `/${stats.total_devices}`,
      icon: Truck,
      color: 'text-tech-green',
      trend: `${((stats.online_devices / stats.total_devices) * 100).toFixed(0)}%`,
    },
    {
      label: '运行中',
      value: runningCount,
      icon: ListChecks,
      color: 'text-cyan-400',
    },
    {
      label: '待执行',
      value: pendingCount,
      icon: Clock,
      color: 'text-tech-orange',
    },
  ];

  const secondaryCards: StatCard[] = [
    {
      label: '今日完成',
      value: completedToday,
      icon: CheckCircle2,
      color: 'text-tech-green',
      subText: `完成率 ${Math.min(100, (completedToday / Math.max(1, completedToday + pendingCount)) * 100).toFixed(0)}%`,
    },
    {
      label: '活跃告警',
      value: stats.alert_count,
      icon: AlertTriangle,
      color: 'text-tech-red',
    },
    {
      label: '今日铲糖',
      value: stats.today_sugar_ton.toFixed(1),
      unit: '吨',
      icon: Activity,
      color: 'text-cyan-400',
    },
  ];

  return (
    <div className="h-full flex flex-col gap-3">
      <div className="flex items-center gap-1.5">
        {primaryCards.map((card) => {
          const Icon = card.icon;
          return (
            <div
              key={card.label}
              className="flex-1 tech-card p-2.5 flex flex-col items-center justify-center text-center"
            >
              <Icon className={cn('w-4 h-4 mb-1', card.color)} />
              <span className={cn('text-lg font-bold font-mono tracking-tight tech-glow-number', card.color)}>
                {card.value}
                {card.unit && <span className="text-[11px] text-text-muted ml-0.5">{card.unit}</span>}
              </span>
              <span className="text-[10px] text-text-muted mt-0.5">{card.label}</span>
              {card.trend && (
                <span className="text-[9px] text-tech-green font-mono">{card.trend}</span>
              )}
            </div>
          );
        })}
      </div>

      <div className="grid grid-cols-3 gap-2">
        {secondaryCards.map((card) => {
          const Icon = card.icon;
          return (
            <div key={card.label} className="tech-card p-2.5 flex items-center gap-2">
              <div className={cn('w-7 h-7 rounded-md bg-bg-primary/60 border border-current/20 flex items-center justify-center shrink-0', card.color)}>
                <Icon className={cn('w-3.5 h-3.5', card.color)} />
              </div>
              <div className="min-w-0">
                <div className="flex items-baseline gap-1">
                  <span className={cn('text-sm font-bold font-mono', card.color)}>{card.value}</span>
                  {card.unit && <span className="text-[10px] text-text-muted">{card.unit}</span>}
                </div>
                <div className="text-[10px] text-text-muted truncate">{card.label}</div>
                {card.subText && (
                  <div className="text-[9px] text-text-muted truncate">{card.subText}</div>
                )}
              </div>
            </div>
          );
        })}
      </div>

      {isOffline && (
        <div className="flex items-center gap-2 px-3 py-2 rounded-md bg-tech-red/10 border border-tech-red/20">
          <WifiOff className="w-4 h-4 text-tech-red" />
          <span className="text-[11px] text-tech-red font-bold">网络离线 - 已切换至应急模式</span>
        </div>
      )}
    </div>
  );
}
