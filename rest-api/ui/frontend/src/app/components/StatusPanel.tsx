import { Battery, Cpu, Thermometer, WifiHigh } from 'lucide-react';

interface StatusPanelProps {
  battery: number;
  cpu: number;
  temperature: number;
  signal: number;
}

export function StatusPanel({ battery, cpu, temperature, signal }: StatusPanelProps) {
  const getStatusColor = (value: number, thresholds = { low: 30, high: 70 }) => {
    if (value < thresholds.low) return 'bg-red-500';
    if (value > thresholds.high) return 'bg-green-500';
    return 'bg-yellow-500';
  };

  const getTextColor = (value: number, thresholds = { low: 30, high: 70 }) => {
    if (value < thresholds.low) return 'text-red-500';
    if (value > thresholds.high) return 'text-green-500';
    return 'text-yellow-500';
  };

  const renderBlockGraph = (value: number, color: string) => {
    const blocks = 10;
    const filledBlocks = Math.ceil((value / 100) * blocks);
    
    return (
      <div className="flex gap-1 mt-2">
        {Array.from({ length: blocks }).map((_, i) => (
          <div
            key={i}
            className={`h-8 flex-1 rounded transition-all ${
              i < filledBlocks ? color : 'bg-slate-800'
            }`}
          />
        ))}
      </div>
    );
  };

  return (
    <div className="grid grid-cols-2 gap-4">
      {/* Battery Status */}
      <div className="col-span-2 p-4 bg-slate-950/50 rounded-lg border border-slate-800">
        <div className="flex items-center justify-between mb-2">
          <div className="flex items-center gap-2">
            <Battery className={`w-5 h-5 ${getTextColor(battery)}`} />
            <span className="text-sm font-medium">电池</span>
          </div>
          <span className={`text-lg font-mono font-bold ${getTextColor(battery)}`}>{battery}%</span>
        </div>
        {renderBlockGraph(battery, getStatusColor(battery))}
      </div>

      {/* CPU Usage */}
      <div className="col-span-2 p-4 bg-slate-950/50 rounded-lg border border-slate-800">
        <div className="flex items-center justify-between mb-2">
          <div className="flex items-center gap-2">
            <Cpu className={`w-5 h-5 ${getTextColor(100 - cpu, { low: 30, high: 70 })}`} />
            <span className="text-sm font-medium">CPU负载</span>
          </div>
          <span className={`text-lg font-mono font-bold ${getTextColor(100 - cpu, { low: 30, high: 70 })}`}>{cpu}%</span>
        </div>
        {renderBlockGraph(cpu, getStatusColor(100 - cpu, { low: 30, high: 70 }))}
      </div>

      {/* Temperature */}
      <div className="p-4 bg-slate-950/50 rounded-lg border border-slate-800">
        <div className="flex items-center justify-between mb-2">
          <div className="flex items-center gap-2">
            <Thermometer className={`w-5 h-5 ${temperature > 70 ? 'text-red-500' : temperature > 40 ? 'text-yellow-500' : 'text-green-500'}`} />
            <span className="text-sm font-medium">温度</span>
          </div>
        </div>
        <div className={`text-2xl font-mono font-bold ${temperature > 70 ? 'text-red-500' : temperature > 40 ? 'text-yellow-500' : 'text-green-500'}`}>
          {temperature}°C
        </div>
        {renderBlockGraph(temperature, temperature > 70 ? 'bg-red-500' : temperature > 40 ? 'bg-yellow-500' : 'bg-green-500')}
      </div>

      {/* Signal Strength */}
      <div className="p-4 bg-slate-950/50 rounded-lg border border-slate-800">
        <div className="flex items-center justify-between mb-2">
          <div className="flex items-center gap-2">
            <WifiHigh className={`w-5 h-5 ${getTextColor(signal)}`} />
            <span className="text-sm font-medium">信号</span>
          </div>
        </div>
        <div className={`text-2xl font-mono font-bold ${getTextColor(signal)}`}>
          {signal}%
        </div>
        {renderBlockGraph(signal, getStatusColor(signal))}
      </div>
    </div>
  );
}
