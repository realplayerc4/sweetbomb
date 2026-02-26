import { Battery, Cpu, Thermometer, Signal, Zap, Gauge } from 'lucide-react';

interface StatusPanelProps {
  battery: number;
  cpu: number;
  temperature: number;
  signal: number;
}

export function StatusPanel({ battery, cpu, temperature, signal }: StatusPanelProps) {
  const getStatusColor = (val: number, inverse = false) => {
    if (inverse) {
      if (val > 80) return 'text-red-500';
      if (val > 60) return 'text-orange-500';
      return 'text-green-500';
    }
    if (val < 20) return 'text-red-500';
    if (val < 40) return 'text-orange-500';
    return 'text-green-500';
  };

  const metrics = [
    { label: '电池电量', value: battery, unit: '%', icon: Battery, color: getStatusColor(battery) },
    { label: 'CPU 负载', value: cpu, unit: '%', icon: Cpu, color: getStatusColor(cpu, true) },
    { label: '环境温度', value: temperature, unit: '°C', icon: Thermometer, color: getStatusColor(temperature, true) },
    { label: '信号强度', value: signal, unit: '%', icon: Signal, color: getStatusColor(signal) },
  ];

  return (
    <div className="grid grid-cols-2 gap-4">
      {metrics.map((m) => (
        <div key={m.label} className="p-4 bg-black/20 rounded-xl border border-white/5 flex flex-col gap-3 group hover:border-orange-500/30 transition-colors">
          <div className="flex items-center justify-between">
            <m.icon className={`w-4 h-4 ${m.color}`} />
            <span className={`text-lg font-bold font-mono tracking-tighter ${m.color}`}>
              {m.value.toFixed(1)}{m.unit}
            </span>
          </div>
          <div className="flex flex-col gap-1.5">
            <div className="flex justify-between text-[10px] font-bold tracking-widest text-slate-500 uppercase">
              <span>{m.label}</span>
              <span className="text-slate-600">Stable</span>
            </div>
            <div className="h-1 w-full bg-white/5 rounded-full overflow-hidden">
              <div
                className={`h-full transition-all duration-1000 ${m.color.replace('text-', 'bg-')}`}
                style={{ width: `${m.value}%` }}
              />
            </div>
          </div>
        </div>
      ))}
    </div>
  );
}
