import { Battery, Cpu, Thermometer, WifiHigh, type LucideIcon } from 'lucide-react';

interface StatusPanelProps {
  battery: number;
  cpu: number;
  temperature: number;
  signal: number;
}

// 单个状态卡片的通用属性
interface StatusCardProps {
  icon: LucideIcon;
  label: string;
  value: number;
  unit: string;
  color: string; // CSS 颜色值，如 '#ef4444'
}

// 颜色常量，直接使用 CSS 颜色值，绕过 Tailwind 动态类名问题
const COLOR_RED = '#ef4444';
const COLOR_YELLOW = '#eab308';
const COLOR_GREEN = '#22c55e';

// 根据数值和阈值返回对应颜色
function getColor(value: number, thresholds = { low: 30, high: 70 }): string {
  if (value < thresholds.low) return COLOR_RED;
  if (value > thresholds.high) return COLOR_GREEN;
  return COLOR_YELLOW;
}

// 温度使用反转逻辑：越高越危险
function getTempColor(temp: number): string {
  if (temp > 70) return COLOR_RED;
  if (temp > 40) return COLOR_YELLOW;
  return COLOR_GREEN;
}

// 通用小格子卡片组件
function StatusCard({ icon: Icon, label, value, unit, color }: StatusCardProps) {
  const blocks = 5;
  const filledBlocks = Math.ceil((Math.min(value, 100) / 100) * blocks);

  return (
    <div className="p-3 bg-slate-950/60 rounded-xl border border-slate-800/80 flex flex-col items-center gap-2 hover:border-slate-700 transition-colors">
      {/* 图标 + 标签 */}
      <div className="flex items-center gap-1.5">
        <Icon className="w-4 h-4" style={{ color }} />
        <span className="text-xs font-medium text-slate-400">{label}</span>
      </div>

      {/* 数值 */}
      <div className="text-xl font-mono font-bold leading-none" style={{ color }}>
        {value.toFixed(1)}
        <span className="text-xs font-normal ml-0.5">{unit}</span>
      </div>

      {/* 迷你方块进度条 */}
      <div className="flex gap-0.5 w-full">
        {Array.from({ length: blocks }).map((_, i) => (
          <div
            key={i}
            className="h-1.5 flex-1 rounded-full transition-all"
            style={{ backgroundColor: i < filledBlocks ? color : '#1e293b' }}
          />
        ))}
      </div>
    </div>
  );
}

export function StatusPanel({ battery, cpu, temperature, signal }: StatusPanelProps) {
  return (
    <div className="grid grid-cols-2 gap-2">
      <StatusCard
        icon={Battery}
        label="电池"
        value={battery}
        unit="%"
        color={getColor(battery)}
      />
      <StatusCard
        icon={Cpu}
        label="CPU负载"
        value={cpu}
        unit="%"
        // CPU负载越高越危险，反转阈值判断
        color={getColor(100 - cpu)}
      />
      <StatusCard
        icon={Thermometer}
        label="温度"
        value={temperature}
        unit="°C"
        color={getTempColor(temperature)}
      />
      <StatusCard
        icon={WifiHigh}
        label="信号"
        value={signal}
        unit="%"
        color={getColor(signal)}
      />
    </div>
  );
}
