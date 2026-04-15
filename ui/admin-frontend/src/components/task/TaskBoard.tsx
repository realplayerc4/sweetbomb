import { useTaskStore } from '@/stores/useTaskStore';
import { cn } from '@/lib/utils';
import { Play, Pause, Square, RotateCcw } from 'lucide-react';

const phaseLabels: Record<string, string> = {
  'idle': '空闲',
  'nav_to_pick': '取货中',
  'nav_to_drop': '卸货中',
  'paused': '已暂停',
  'completed': '已完成',
  'error': '错误',
};

export function TaskBoard() {
  const targetKg = useTaskStore((s) => s.targetKg);
  const totalCycles = useTaskStore((s) => s.totalCycles);
  const currentCycle = useTaskStore((s) => s.currentCycle);
  const completedKg = useTaskStore((s) => s.completedKg);
  const phase = useTaskStore((s) => s.phase);
  const isRunning = useTaskStore((s) => s.isRunning);
  const start = useTaskStore((s) => s.start);
  const pause = useTaskStore((s) => s.pause);
  const resume = useTaskStore((s) => s.resume);
  const stop = useTaskStore((s) => s.stop);

  const percentage = totalCycles > 0 ? Math.round((currentCycle / totalCycles) * 100) : 0;

  return (
    <div className="h-full flex flex-col">
      <div className="tech-card p-4 mb-4">
        <div className="flex items-center justify-between mb-3">
          <span className="text-[14px] font-bold text-text-primary">当前任务</span>
          <span className={cn(
            'px-2 py-1 rounded text-[11px] font-bold',
            isRunning ? 'bg-cyan-500/20 text-cyan-400' :
            phase === 'completed' ? 'bg-tech-green/20 text-tech-green' :
            phase === 'paused' ? 'bg-tech-orange/20 text-tech-orange' :
            'bg-text-muted/20 text-text-muted'
          )}>
            {phaseLabels[phase] || phase}
          </span>
        </div>

        <div className="space-y-2">
          <div className="flex justify-between text-[12px]">
            <span className="text-text-muted">目标</span>
            <span className="text-cyan-400 font-mono font-bold">{targetKg} kg</span>
          </div>
          <div className="flex justify-between text-[12px]">
            <span className="text-text-muted">已完成</span>
            <span className="text-tech-green font-mono font-bold">{completedKg} kg</span>
          </div>
          <div className="flex justify-between text-[12px]">
            <span className="text-text-muted">循环</span>
            <span className="text-tech-orange font-mono font-bold">{currentCycle}/{totalCycles}</span>
          </div>
        </div>

        <div className="mt-3">
          <div className="h-2 bg-bg-primary/60 rounded-full overflow-hidden">
            <div
              className="h-full bg-gradient-to-r from-cyan-500 to-tech-blue rounded-full transition-all duration-300"
              style={{ width: `${percentage}%` }}
            />
          </div>
          <div className="text-center text-[11px] text-text-muted mt-1">{percentage}%</div>
        </div>
      </div>

      <div className="flex items-center justify-center gap-3">
        {phase === 'paused' ? (
          <button
            onClick={resume}
            className="px-4 py-2 rounded-md bg-tech-green/15 text-tech-green border border-tech-green/25 font-bold text-[12px] hover:bg-tech-green/25 transition-colors"
          >
            <RotateCcw className="w-3.5 h-3.5 inline mr-1" />
            恢复
          </button>
        ) : (
          <button
            onClick={start}
            disabled={isRunning || phase === 'completed'}
            className="px-4 py-2 rounded-md bg-tech-green/15 text-tech-green border border-tech-green/25 font-bold text-[12px] hover:bg-tech-green/25 transition-colors disabled:opacity-50"
          >
            <Play className="w-3.5 h-3.5 inline mr-1" />
            启动
          </button>
        )}

        <button
          onClick={pause}
          disabled={!isRunning}
          className="px-4 py-2 rounded-md bg-tech-orange/15 text-tech-orange border border-tech-orange/25 font-bold text-[12px] hover:bg-tech-orange/25 transition-colors disabled:opacity-50"
        >
          <Pause className="w-3.5 h-3.5 inline mr-1" />
          暂停
        </button>

        <button
          onClick={stop}
          disabled={!isRunning && phase !== 'paused'}
          className="px-4 py-2 rounded-md bg-tech-red/15 text-tech-red border border-tech-red/25 font-bold text-[12px] hover:bg-tech-red/25 transition-colors disabled:opacity-50"
        >
          <Square className="w-3.5 h-3.5 inline mr-1" />
          终止
        </button>
      </div>
    </div>
  );
}
