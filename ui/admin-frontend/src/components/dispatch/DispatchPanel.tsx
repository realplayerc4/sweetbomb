import { useTaskStore } from '@/stores/useTaskStore';
import { useRobotStore } from '@/stores/useRobotStore';
import { cn } from '@/lib/utils';
import { Zap, Bot, ArrowRight, CheckCircle2 } from 'lucide-react';

const phaseLabels: Record<string, string> = {
  'idle': '空闲',
  'nav_to_pick': '导航取货',
  'picking': '取货中',
  'nav_to_drop': '导航卸货',
  'dropping': '卸货中',
  'paused': '已暂停',
  'completed': '已完成',
  'error': '错误',
};

export function DispatchPanel() {
  const currentCycle = useTaskStore((s) => s.currentCycle);
  const totalCycles = useTaskStore((s) => s.totalCycles);
  const completedKg = useTaskStore((s) => s.completedKg);
  const targetKg = useTaskStore((s) => s.targetKg);
  const phase = useTaskStore((s) => s.phase);
  const isRunning = useTaskStore((s) => s.isRunning);

  const robotStatus = useRobotStore((s) => s.status);
  const isConnected = useRobotStore((s) => s.isConnected);

  const percentage = totalCycles > 0 ? Math.round((currentCycle / totalCycles) * 100) : 0;

  return (
    <div className="h-full flex flex-col">
      <div className="flex items-center justify-between mb-2">
        <div className="flex items-center gap-1.5">
          <Zap className="w-4 h-4 text-cyan-400" />
          <span className="text-[13px] font-bold text-cyan-400 tracking-wider">任务状态</span>
        </div>
        <div className={cn(
          'px-2 py-0.5 rounded text-[10px] font-bold',
          isConnected ? 'bg-tech-green/15 text-tech-green' : 'bg-tech-red/15 text-tech-red'
        )}>
          {isConnected ? '已连接' : '未连接'}
        </div>
      </div>

      <div className="grid grid-cols-3 gap-1.5 mb-2">
        <div className="tech-card p-2 text-center">
          <div className="text-[9px] text-text-muted">循环</div>
          <div className="text-[14px] font-bold text-cyan-400 font-mono">{currentCycle}/{totalCycles}</div>
        </div>
        <div className="tech-card p-2 text-center">
          <div className="text-[9px] text-text-muted">已装载</div>
          <div className="text-[14px] font-bold text-tech-green font-mono">{completedKg}kg</div>
        </div>
        <div className="tech-card p-2 text-center">
          <div className="text-[9px] text-text-muted">进度</div>
          <div className="text-[14px] font-bold text-tech-orange font-mono">{percentage}%</div>
        </div>
      </div>

      <div className="flex-1 overflow-y-auto space-y-2">
        <div>
          <div className="flex items-center gap-1.5 mb-1.5">
            <Bot className="w-3.5 h-3.5 text-cyan-400" />
            <span className="text-[11px] font-bold text-cyan-400">当前阶段</span>
          </div>
          <div className="tech-card p-2">
            <div className="flex items-center justify-between">
              <span className="text-[12px] text-text-primary font-semibold">
                {phaseLabels[phase] || phase}
              </span>
              {isRunning && (
                <div className="flex items-center gap-1">
                  <div className="w-2 h-2 rounded-full bg-cyan-400 animate-pulse" />
                  <span className="text-[10px] text-cyan-400">执行中</span>
                </div>
              )}
            </div>
            <div className="mt-2 h-1.5 bg-bg-primary/60 rounded-full overflow-hidden">
              <div
                className="h-full bg-gradient-to-r from-cyan-500 to-tech-blue rounded-full transition-all duration-300"
                style={{ width: `${percentage}%` }}
              />
            </div>
          </div>
        </div>

        <div>
          <div className="flex items-center gap-1.5 mb-1.5">
            <CheckCircle2 className="w-3.5 h-3.5 text-tech-green" />
            <span className="text-[11px] font-bold text-tech-green">任务统计</span>
          </div>
          <div className="space-y-1">
            <div className="flex items-center justify-between text-[11px]">
              <span className="text-text-muted">目标重量</span>
              <span className="text-cyan-400 font-mono">{targetKg} kg</span>
            </div>
            <div className="flex items-center justify-between text-[11px]">
              <span className="text-text-muted">已完成</span>
              <span className="text-tech-green font-mono">{completedKg} kg</span>
            </div>
            <div className="flex items-center justify-between text-[11px]">
              <span className="text-text-muted">剩余</span>
              <span className="text-tech-orange font-mono">{targetKg - completedKg} kg</span>
            </div>
            <div className="flex items-center justify-between text-[11px]">
              <span className="text-text-muted">每次装载</span>
              <span className="text-text-secondary font-mono">30 kg</span>
            </div>
          </div>
        </div>

        {phase === 'completed' && (
          <div className="tech-card p-2 bg-tech-green/10 border-tech-green/20">
            <div className="flex items-center gap-2">
              <CheckCircle2 className="w-4 h-4 text-tech-green" />
              <span className="text-[12px] text-tech-green font-bold">
                任务完成！共装载 {completedKg} kg
              </span>
            </div>
          </div>
        )}
      </div>
    </div>
  );
}
