import { useEffect, useState } from 'react';
import { TrendingUp, Target, Timer, Weight } from 'lucide-react';
import { cn } from '@/lib/utils';

interface HarvestStat {
  label: string;
  value: string;
  unit: string;
  icon: React.ElementType;
  color: string;
  trend?: string;
}

export function HarvestStats() {
  const [stats, setStats] = useState<HarvestStat[]>([]);

  useEffect(() => {
    const updateStats = () => {
      setStats([
        {
          label: '今日铲糖量',
          value: (Math.random() * 5 + 12).toFixed(1),
          unit: '吨',
          icon: Weight,
          color: 'text-cyan-400',
          trend: `+${(Math.random() * 10 + 2).toFixed(1)}%`,
        },
        {
          label: '累计铲糖量',
          value: (Math.random() * 50 + 280).toFixed(0),
          unit: '吨',
          icon: TrendingUp,
          color: 'text-tech-green',
        },
        {
          label: '平均效率',
          value: (Math.random() * 0.3 + 1.8).toFixed(2),
          unit: '吨/h',
          icon: Target,
          color: 'text-tech-orange',
        },
        {
          label: '运行时长',
          value: (Math.random() * 2 + 6).toFixed(1),
          unit: 'h',
          icon: Timer,
          color: 'text-cyan-400',
        },
      ]);
    };

    updateStats();
    const interval = setInterval(updateStats, 5000);
    return () => clearInterval(interval);
  }, []);

  return (
    <div className="grid grid-cols-2 gap-2.5">
      {stats.map((stat) => {
        const Icon = stat.icon;
        return (
          <div key={stat.label} className="tech-card p-3">
            <div className="flex items-center justify-between mb-2">
              <span className="text-[13px] text-text-muted font-semibold tracking-wider">{stat.label}</span>
              <Icon className={cn('w-4 h-4', stat.color)} />
            </div>
            <div className="flex items-baseline gap-1">
              <span className={cn('text-xl font-bold tracking-tight tech-glow-number', stat.color)}>
                {stat.value}
              </span>
              <span className="text-[13px] text-text-muted">{stat.unit}</span>
            </div>
            {stat.trend && (
              <div className="mt-1 text-[12px] text-tech-green font-mono">{stat.trend}</div>
            )}
          </div>
        );
      })}
    </div>
  );
}
