import { Battery, Cpu, Thermometer, WifiHigh, Compass, type LucideIcon } from 'lucide-react';

interface StatusPanelProps {
  battery: number;
  cpu: number;
  temperature: number;
  signal: number;
  imu?: { roll: number; pitch: number; yaw: number };
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
const COLOR_GREEN = '#FD802E';

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

// 横向排列的单行项
function StatusItem({ icon: Icon, label, value, unit, color }: StatusCardProps) {
  return (
    <div className="flex items-center gap-3">
      <div className="flex items-center gap-1.5">
        <Icon className="w-4 h-4" style={{ color }} />
        <span className="text-xs font-medium text-slate-400">{label}</span>
      </div>
      <div className="text-sm font-mono font-bold" style={{ color }}>
        {value.toFixed(1)}
        <span className="text-xs font-normal ml-0.5">{unit}</span>
      </div>
    </div>
  );
}

export function StatusPanel({ battery, cpu, temperature, signal, imu }: StatusPanelProps) {
  return (
    <div className="flex items-center gap-8">
      <StatusItem
        icon={Battery}
        label="电池： "
        value={battery}
        unit="%"
        color={getColor(battery)}
      />
      <StatusItem
        icon={Cpu}
        label="CPU负载： "
        value={cpu}
        unit="%"
        color={getColor(100 - cpu)}
      />
      <StatusItem
        icon={Thermometer}
        label="温度： "
        value={temperature}
        unit="°C "
        color={getTempColor(temperature)}
      />
      <StatusItem
        icon={WifiHigh}
        label="信号"
        value={signal}
        unit="%"
        color={getColor(signal)}
      />

      {/* IMU 数据显示 */}
      {imu && (
        <div className="flex items-center gap-3 ml-2 pl-4 border-l border-slate-700/50">
          <div className="flex items-center gap-1.5">
            <Compass className="w-4 h-4" style={{ color: COLOR_GREEN }} />
            <span className="text-xs font-medium text-slate-400">IMU姿态</span>
          </div>
          <div className="text-xs font-mono font-bold text-orange-400 flex gap-2">
            <span>ROLL: {imu.roll.toFixed(1)}</span>
            <span>PITCH:{imu.pitch.toFixed(1)}</span>
            <span>YAW:{imu.yaw.toFixed(1)}</span>
          </div>
        </div>
      )}
    </div>
  );
}
