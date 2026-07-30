import { useState } from 'react';
import { useRobotStore } from '@/stores/useRobotStore';
import { useSystemModeStore } from '@/stores/useSystemModeStore';
import { cn } from '@/lib/utils';
import { Play, Square, Save, RotateCcw, Settings, Sliders } from 'lucide-react';

interface Parameter {
  key: string;
  label: string;
  unit: string;
  value: number;
  min: number;
  max: number;
  step: number;
  precision: number;
}

export function RobotStatusPanel() {
  const isConnected = useRobotStore((s) => s.isConnected);
  const selectedRobot = useSystemModeStore((s) => s.selectedRobot);
  const [parameters, setParameters] = useState<Parameter[]>([
    { key: 'bucket_width', label: '铲斗宽度', unit: 'm', value: 0.80, min: 0.10, max: 2.00, step: 0.01, precision: 2 },
    { key: 'target_depth', label: '目标深度', unit: 'm', value: 0.30, min: 0.01, max: 1.00, step: 0.01, precision: 2 },
    { key: 'dump_height', label: '排料高度', unit: 'm', value: 3.80, min: 0.50, max: 5.00, step: 0.01, precision: 2 },
    { key: 'forward_speed', label: '前进速度', unit: 'm/s', value: 0.20, min: 0.05, max: 1.00, step: 0.01, precision: 2 },
    { key: 'backward_distance', label: '后退距离', unit: 'm', value: 0.90, min: 0.10, max: 2.00, step: 0.01, precision: 2 },
    { key: 'recommended_height', label: '建议高度', unit: 'm', value: 0.50, min: 0.10, max: 1.50, step: 0.01, precision: 2 },
  ]);
  const [isRunning, setIsRunning] = useState(false);
  const [isSaving, setIsSaving] = useState(false);

  const handleParameterChange = (key: string, value: number) => {
    setParameters((prev) => prev.map((param) =>
      param.key === key ? { ...param, value: Number(value.toFixed(param.precision)) } : param
    ));
  };

  const handleInputChange = (key: string, e: React.ChangeEvent<HTMLInputElement>) => {
    const value = parseFloat(e.target.value);
    if (!isNaN(value)) {
      handleParameterChange(key, value);
    }
  };

  const handleSliderChange = (key: string, value: number) => {
    handleParameterChange(key, value);
  };

  const handleStart = () => {
    setIsRunning(true);
    // 这里可以添加启动控制逻辑
  };

  const handleStop = () => {
    setIsRunning(false);
    // 这里可以添加停止控制逻辑
  };

  const handleSave = async () => {
    setIsSaving(true);
    // 这里可以添加保存参数的逻辑
    setTimeout(() => setIsSaving(false), 1000);
  };

  const handleReset = () => {
    setParameters([
      { key: 'bucket_width', label: '铲斗宽度', unit: 'm', value: 0.80, min: 0.10, max: 2.00, step: 0.01, precision: 2 },
      { key: 'target_depth', label: '目标深度', unit: 'm', value: 0.30, min: 0.01, max: 1.00, step: 0.01, precision: 2 },
      { key: 'dump_height', label: '排料高度', unit: 'm', value: 3.80, min: 0.50, max: 5.00, step: 0.01, precision: 2 },
      { key: 'forward_speed', label: '前进速度', unit: 'm/s', value: 0.20, min: 0.05, max: 1.00, step: 0.01, precision: 2 },
      { key: 'backward_distance', label: '后退距离', unit: 'm', value: 0.90, min: 0.10, max: 2.00, step: 0.01, precision: 2 },
      { key: 'recommended_height', label: '建议高度', unit: 'm', value: 0.50, min: 0.10, max: 1.50, step: 0.01, precision: 2 },
    ]);
  };

  return (
    <div className="h-full flex flex-col">
      <div className="flex items-center justify-between mb-3">
        <div className="flex items-center gap-2">
          <Settings className="w-4.5 h-4.5 text-cyan-400" />
          <span className="text-[14px] font-bold text-cyan-400 tracking-wider">机器人参数控制</span>
        </div>
        <div className="flex items-center gap-2">
          <div className={cn('status-dot', isConnected ? 'status-dot-online' : 'status-dot-error')} />
          <span className={cn('text-[13px] font-bold', isConnected ? 'text-tech-green' : 'text-tech-red')}>
            {isConnected ? '已连接' : '未连接'}
          </span>
        </div>
      </div>

      <div className="flex-1 overflow-y-auto space-y-3">
        {parameters.map((param) => (
          <div key={param.key} className="tech-card p-3">
            <div className="flex items-center justify-between mb-2">
              <span className="text-[13px] text-text-muted font-semibold tracking-wider">{param.label}</span>
              <div className="flex items-center gap-2">
                <input
                  type="number"
                  value={param.value}
                  onChange={(e) => handleInputChange(param.key, e)}
                  min={param.min}
                  max={param.max}
                  step={param.step}
                  className="w-18 h-7 bg-bg-primary/60 border border-cyan-500/20 rounded px-2 text-[13px] text-cyan-400 font-mono focus:outline-none focus:border-cyan-400"
                  disabled={!isConnected}
                />
                <span className="text-[13px] text-text-muted">{param.unit}</span>
              </div>
            </div>
            <input
              type="range"
              value={param.value}
              onChange={(e) => handleSliderChange(param.key, parseFloat(e.target.value))}
              min={param.min}
              max={param.max}
              step={param.step}
              className="w-full h-1.5 bg-bg-primary/60 rounded-full appearance-none cursor-pointer"
              style={{
                background: `linear-gradient(to right, #00d4ff 0%, #00d4ff ${((param.value - param.min) / (param.max - param.min)) * 100}%, #1e293b ${((param.value - param.min) / (param.max - param.min)) * 100}%, #1e293b 100%)`
              }}
              disabled={!isConnected}
            />
            <div className="flex justify-between mt-1">
              <span className="text-[12px] text-text-muted font-mono">{param.min}</span>
              <span className="text-[12px] text-text-muted font-mono">{param.max}</span>
            </div>
          </div>
        ))}
      </div>

      <div className="mt-3 pt-3 border-t border-cyan-500/10">
        <div className="flex items-center justify-between mb-3">
          <div className="flex items-center gap-1.5">
            <Sliders className="w-4.5 h-4.5 text-cyan-400" />
            <span className="text-[13px] font-bold text-cyan-400 tracking-wider">控制操作</span>
          </div>
          <span className="text-[12px] text-text-muted font-mono">{selectedRobot}</span>
        </div>

        <div className="flex gap-2">
          <button
            onClick={handleStart}
            disabled={!isConnected || isRunning}
            className={cn(
              'flex-1 flex items-center justify-center gap-1.5 py-2.5 rounded-md font-bold text-[14px] tracking-wider transition-all',
              isConnected && !isRunning
                ? 'bg-tech-green/15 text-tech-green border border-tech-green/25 hover:bg-tech-green/25'
                : 'bg-text-muted/10 text-text-muted/50 border border-text-muted/20 cursor-not-allowed'
            )}
          >
            <Play className="w-4 h-4" />
            启动
          </button>
          <button
            onClick={handleStop}
            disabled={!isRunning}
            className={cn(
              'flex-1 flex items-center justify-center gap-1.5 py-2.5 rounded-md font-bold text-[14px] tracking-wider transition-all',
              isRunning
                ? 'bg-tech-red/15 text-tech-red border border-tech-red/25 hover:bg-tech-red/25'
                : 'bg-text-muted/10 text-text-muted/50 border border-text-muted/20 cursor-not-allowed'
            )}
          >
            <Square className="w-4 h-4" />
            停止
          </button>
        </div>

        <div className="flex gap-2 mt-2">
          <button
            onClick={handleSave}
            disabled={!isConnected || isSaving}
            className={cn(
              'flex-1 flex items-center justify-center gap-1.5 py-2 rounded-md font-bold text-[13px] tracking-wider transition-all',
              isConnected && !isSaving
                ? 'bg-cyan-500/15 text-cyan-400 border border-cyan-500/25 hover:bg-cyan-500/25'
                : 'bg-text-muted/10 text-text-muted/50 border border-text-muted/20 cursor-not-allowed'
            )}
          >
            {isSaving ? (
              <div className="w-4 h-4 border-2 border-cyan-400 border-t-transparent rounded-full animate-spin" />
            ) : (
              <Save className="w-4 h-4" />
            )}
            保存参数
          </button>
          <button
            onClick={handleReset}
            disabled={!isConnected}
            className={cn(
              'flex-1 flex items-center justify-center gap-1.5 py-2 rounded-md font-bold text-[13px] tracking-wider transition-all',
              isConnected
                ? 'bg-text-muted/15 text-text-secondary border border-text-muted/25 hover:bg-text-muted/25'
                : 'bg-text-muted/10 text-text-muted/50 border border-text-muted/20 cursor-not-allowed'
            )}
          >
            <RotateCcw className="w-4 h-4" />
            恢复默认
          </button>
        </div>
      </div>
    </div>
  );
}
