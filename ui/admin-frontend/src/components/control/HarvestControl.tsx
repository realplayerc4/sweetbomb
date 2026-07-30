import { useState, useEffect, useMemo } from 'react';
import { useRobotStore } from '@/stores/useRobotStore';
import { useSystemModeStore } from '@/stores/useSystemModeStore';
import { cn } from '@/lib/utils';
import type { SugarHarvestConfig } from '@/types';
import {
  Play, Square, Pause, RotateCcw,
  Navigation, Package, Truck, AlertTriangle,
  ChevronRight, Plus, X, Settings, Loader2,
  Zap, Clock, Target
} from 'lucide-react';

type FlowStep = 'navigate_pickup' | 'scoop' | 'navigate_dump' | 'dump';
type ControlMode = 'cycles' | 'duration';

interface CustomAction {
  id: string;
  name: string;
  after: FlowStep;
}

const FLOW_STEPS: { key: FlowStep; label: string; icon: React.ElementType; color: string }[] = [
  { key: 'navigate_pickup', label: '导航取糖点', icon: Navigation, color: '#00d4ff' },
  { key: 'scoop', label: '翻斗加载', icon: Package, color: '#52c41a' },
  { key: 'navigate_dump', label: '导航卸载点', icon: Navigation, color: '#fa8c16' },
  { key: 'dump', label: '翻斗卸载', icon: Truck, color: '#f5222d' },
];

const PRESET_ACTIONS = [
  '堆垛整理', '避障绕行', '设备自检', '路径优化',
];

export function HarvestControl() {
  const isHarvestRunning = useRobotStore((s) => s.isHarvestRunning);
  const harvestCycle = useRobotStore((s) => s.harvestCycle);
  const harvestMaxCycles = useRobotStore((s) => s.harvestMaxCycles);
  const startHarvest = useRobotStore((s) => s.startHarvest);
  const stopHarvest = useRobotStore((s) => s.stopHarvest);
  const selectedRobot = useSystemModeStore((s) => s.selectedRobot);

  const [controlMode, setControlMode] = useState<ControlMode>('cycles');
  const [targetCycles, setTargetCycles] = useState(10);
  const [targetDuration, setTargetDuration] = useState(60);
  const [priority, setPriority] = useState(5);
  const [currentStep, setCurrentStep] = useState<FlowStep>('navigate_pickup');
  const [isPaused, setIsPaused] = useState(false);
  const [customActions, setCustomActions] = useState<CustomAction[]>([]);
  const [showAddAction, setShowAddAction] = useState(false);

  const progressPercent = controlMode === 'cycles'
    ? (harvestMaxCycles > 0 ? (harvestCycle / harvestMaxCycles) * 100 : 0)
    : 50;

  useEffect(() => {
    if (!isHarvestRunning || isPaused) return;
    const steps: FlowStep[] = ['navigate_pickup', 'scoop', 'navigate_dump', 'dump'];
    let idx = steps.indexOf(currentStep);
    if (idx === -1) idx = 0;
    const timer = setInterval(() => {
      setCurrentStep(steps[(idx + 1) % steps.length]);
      idx++;
    }, 2000);
    return () => clearInterval(timer);
  }, [isHarvestRunning, isPaused, currentStep]);

  const handleStart = async () => {
    setIsPaused(false);
    const config: SugarHarvestConfig = {
      navigation_point: [1.0, 0.0],
      dump_point: [0.0, 1.0],
      bucket_width_m: 0.6,
      approach_offset_m: 0.05,
      scoop_position: 90.0,
      dump_position: 135.0,
      max_cycles: targetCycles,
      height_threshold_m: 0.20,
    };
    await startHarvest(config);
  };

  const handlePause = () => setIsPaused(true);
  const handleResume = () => setIsPaused(false);
  const handleStop = () => { stopHarvest(); setIsPaused(false); };

  const addAction = (name: string, after: FlowStep) => {
    setCustomActions([...customActions, { id: `action-${Date.now()}`, name, after }]);
    setShowAddAction(false);
  };

  const removeAction = (id: string) => setCustomActions(customActions.filter((a) => a.id !== id));

  const estimatedTotalTons = useMemo(() =>
    Number((targetCycles * 0.85).toFixed(1)),
    [targetCycles]
  );

  return (
    <div className="h-full flex flex-col">
      <div className="flex items-center justify-between mb-2 p-2 rounded-md bg-bg-primary/40 border border-cyan-500/15">
        <div className={cn('status-dot', isHarvestRunning && !isPaused ? 'status-dot-online' : isHarvestRunning && isPaused ? 'status-dot-warning' : 'status-dot-idle')} />
        <span className={cn('text-[13px] font-bold', isHarvestRunning && !isPaused ? 'text-tech-green' : isHarvestRunning && isPaused ? 'text-tech-orange' : 'text-cyan-400')}>
          {isHarvestRunning && !isPaused ? '运行中' : isHarvestRunning && isPaused ? '已暂停' : '空闲'}
        </span>
        <div className="flex items-center gap-1 text-[10px] text-text-muted font-mono">
          <Zap className="w-3 h-3" /> P{priority}
          <span className="ml-1">{selectedRobot}</span>
        </div>
      </div>

      {isHarvestRunning && (
        <div className="mb-2 space-y-1.5">
          <div className="flex justify-between text-[12px]">
            <span className="text-text-muted">循环进度</span>
            <span className="text-cyan-400 font-mono font-bold">{harvestCycle}/{harvestMaxCycles}</span>
          </div>
          <div className="h-1.5 bg-bg-primary/60 rounded-full overflow-hidden">
            <div
              className="h-full bg-gradient-to-r from-cyan-500 to-tech-blue rounded-full transition-all duration-500"
              style={{ width: `${progressPercent}%` }}
            />
          </div>
        </div>
      )}

      {!isHarvestRunning ? (
        <div className="flex-1 space-y-2.5 overflow-y-auto">
          <div>
            <div className="text-[12px] text-text-muted font-semibold tracking-wider mb-1.5">作业流程</div>
            <div className="space-y-1">
              {FLOW_STEPS.map((step, i) => {
                const Icon = step.icon;
                const isActive = currentStep === step.key && !isHarvestRunning;
                const actionsForStep = customActions.filter((a) => a.after === step.key);
                return (
                  <div key={step.key} className="flex items-center gap-1.5">
                    <div
                      className={cn(
                        'w-6 h-6 rounded-md flex items-center justify-center shrink-0 transition-all',
                        isActive ? 'shadow-[0_0_8px_rgba(0,212,255,0.4)]' : ''
                      )}
                      style={{ background: `${step.color}15`, border: `1px solid ${step.color}40` }}
                    >
                      <Icon className="w-3 h-3" style={{ color: step.color }} />
                    </div>
                    <span className={cn('text-[11px] font-semibold', isActive ? 'text-text-primary' : 'text-text-muted')}>
                      {step.label}
                    </span>
                    {actionsForStep.length > 0 && (
                      <span className="text-[9px] px-1 py-0.5 rounded bg-tech-orange/15 text-tech-orange font-mono">
                        +{actionsForStep.length}
                      </span>
                    )}
                    {i < FLOW_STEPS.length - 1 && (
                      <ChevronRight className="w-3 h-3 text-text-muted/30 ml-auto" />
                    )}
                  </div>
                );
              })}
            </div>

            <button
              onClick={() => setShowAddAction(!showAddAction)}
              className="w-full mt-1.5 flex items-center justify-center gap-1 py-1.5 rounded-md border border-dashed border-cyan-500/25 text-cyan-400/70 text-[11px] hover:border-cyan-500/50 hover:text-cyan-400 transition-colors"
            >
              <Plus className="w-3 h-3" /> 插入附加行为
            </button>

            {showAddAction && (
              <div className="p-2 rounded-md bg-bg-primary/40 border border-cyan-500/15 space-y-1.5">
                <div className="grid grid-cols-4 gap-1">
                  {PRESET_ACTIONS.map((name) => (
                    <button
                      key={name}
                      onClick={() => addAction(name, 'navigate_pickup')}
                      className="py-1 px-1.5 rounded text-[10px] text-text-secondary bg-bg-primary/60 hover:bg-cyan-500/10 hover:text-cyan-400 transition-colors text-center"
                    >
                      {name}
                    </button>
                  ))}
                </div>
              </div>
            )}

            {customActions.length > 0 && (
              <div className="space-y-1 mt-1.5">
                {customActions.map((action) => (
                  <div key={action.id} className="flex items-center gap-2 p-1.5 rounded bg-bg-primary/30 border border-tech-orange/15">
                    <div className="w-1 h-1 rounded-full bg-tech-orange" />
                    <span className="text-[11px] text-tech-orange flex-1 truncate">{action.name}</span>
                    <span className="text-[9px] text-text-muted">→ {FLOW_STEPS.find((s) => s.key === action.after)?.label}</span>
                    <button onClick={() => removeAction(action.id)} className="p-0.5 text-text-muted hover:text-tech-red">
                      <X className="w-3 h-3" />
                    </button>
                  </div>
                ))}
              </div>
            )}
          </div>

          <div>
            <div className="text-[12px] text-text-muted font-semibold tracking-wider mb-1.5">控制参数</div>
            <div className="flex items-center gap-1 mb-2">
              {([
                ['cycles', '按次数'],
                ['duration', '按时长'],
              ] as const).map(([key, label]) => (
                <button
                  key={key}
                  onClick={() => setControlMode(key)}
                  className={cn(
                    'flex-1 py-1 rounded text-[11px] font-bold transition-all',
                    controlMode === key ? 'bg-cyan-500/20 text-cyan-400' : 'bg-bg-primary/40 text-text-muted'
                  )}
                >
                  {label}
                </button>
              ))}
            </div>
            <div className="tech-card p-2.5">
              <div className="flex items-center justify-between mb-1.5">
                <span className="text-[11px] text-text-muted">
                  {controlMode === 'cycles' ? '目标循环次数' : '目标时长 (分钟)'}
                </span>
                <span className="text-[12px] font-mono text-cyan-400 font-bold">
                  {controlMode === 'cycles' ? targetCycles : targetDuration}{controlMode === 'cycles' ? ' 次' : ' min'}
                </span>
              </div>
              <input
                type="range"
                value={controlMode === 'cycles' ? targetCycles : targetDuration}
                onChange={(e) => controlMode === 'cycles'
                  ? setTargetCycles(Number(e.target.value))
                  : setTargetDuration(Number(e.target.value))
                }
                min={controlMode === 'cycles' ? 1 : 10}
                max={controlMode === 'cycles' ? 100 : 480}
                step={controlMode === 'cycles' ? 1 : 5}
                className="w-full h-1.5 bg-bg-primary/60 rounded-full appearance-none cursor-pointer"
              />
              <div className="flex justify-between mt-1 text-[9px] text-text-muted font-mono">
                <span>{controlMode === 'cycles' ? '1' : '10'}</span>
                <span>{controlMode === 'cycles' ? '100' : '480'}</span>
              </div>
            </div>
          </div>

          <div className="grid grid-cols-2 gap-2">
            <div className="tech-card p-2 text-center">
              <div className="text-[10px] text-text-muted">预计产量</div>
              <div className="text-[14px] font-bold text-cyan-400 font-mono tech-glow-number">{estimatedTotalTons} 吨</div>
            </div>
            <div className="tech-card p-2 text-center">
              <div className="text-[10px] text-text-muted">优先级</div>
              <select
                value={priority}
                onChange={(e) => setPriority(Number(e.target.value))}
                className="bg-transparent text-[14px] font-bold text-tech-orange font-mono focus:outline-none text-center w-full"
              >
                {[1, 5, 10, 20].map((v) => (
                  <option key={v} value={v} className="bg-bg-primary text-text-primary">P{v}</option>
                ))}
              </select>
            </div>
          </div>

          <button
            onClick={handleStart}
            className="w-full flex items-center justify-center gap-2 py-2.5 rounded-md bg-cyan-500/15 text-cyan-400 border border-cyan-500/25 font-bold text-[13px] tracking-wider hover:bg-cyan-500/25 transition-colors"
          >
            <Play className="w-4 h-4" /> 启动作业循环
          </button>
        </div>
      ) : (
        <div className="mt-auto space-y-2">
          <div className="flex gap-2">
            {isPaused ? (
              <button
                onClick={handleResume}
                className="flex-1 flex items-center justify-center gap-1.5 py-2 rounded-md bg-tech-green/15 text-tech-green border border-tech-green/25 font-bold text-[12px] tracking-wider hover:bg-tech-green/25 transition-colors"
              >
                <Play className="w-3.5 h-3.5" /> 恢复
              </button>
            ) : (
              <button
                onClick={handlePause}
                className="flex-1 flex items-center justify-center gap-1.5 py-2 rounded-md bg-tech-orange/15 text-tech-orange border border-tech-orange/25 font-bold text-[12px] tracking-wider hover:bg-tech-orange/25 transition-colors"
              >
                <Pause className="w-3.5 h-3.5" /> 暂停
              </button>
            )}
            <button
              onClick={handleStop}
              className="flex-1 flex items-center justify-center gap-1.5 py-2 rounded-md bg-tech-red/15 text-tech-red border border-tech-red/25 font-bold text-[12px] tracking-wider hover:bg-tech-red/25 transition-colors"
            >
              <Square className="w-3.5 h-3.5" /> 停止
            </button>
          </div>

          <div className="tech-card p-2.5 space-y-1.5">
            <div className="text-[11px] text-text-muted font-semibold tracking-wider">当前阶段</div>
            <div className="flex items-center gap-2">
              {FLOW_STEPS.map((step) => {
                const Icon = step.icon;
                const isCurrent = currentStep === step.key;
                const isPast = FLOW_STEPS.findIndex((s) => s.key === currentStep) > FLOW_STEPS.indexOf(step);
                return (
                  <div
                    key={step.key}
                    className={cn(
                      'flex-1 flex flex-col items-center gap-1',
                      isCurrent ? '' : isPast ? 'opacity-40' : 'opacity-20'
                    )}
                  >
                    <div
                      className={cn(
                        'w-7 h-7 rounded-md flex items-center justify-center transition-all',
                        isCurrent ? 'animate-pulse shadow-[0_0_8px_var(--glow)]' : ''
                      )}
                      style={{
                        background: `${step.color}${isCurrent ? '25' : '10'}`,
                        border: `1px solid ${step.color}${isCurrent ? '50' : '20'}`,
                      } as React.CSSProperties}
                    >
                      <Icon className="w-3.5 h-3.5" style={{ color: step.color }} />
                    </div>
                    <span className={cn('text-[9px]', isCurrent ? 'font-bold text-text-primary' : 'text-text-muted')}>{step.label.slice(0, 2)}</span>
                  </div>
                );
              })}
            </div>
          </div>

          <div className="tech-card p-2.5">
            <div className="flex items-center justify-between text-[11px]">
              <span className="text-text-muted flex items-center gap-1"><AlertTriangle className="w-3 h-3 text-tech-orange" /> 异常处理</span>
              <span className="text-tech-green font-mono">正常</span>
            </div>
          </div>
        </div>
      )}
    </div>
  );
}
