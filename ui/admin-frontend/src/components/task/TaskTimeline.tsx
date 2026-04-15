import { useTaskStore } from '@/stores/useTaskStore';

const phaseLabels: Record<string, string> = {
  'idle': '空闲',
  'nav_to_pick': '取货中',
  'nav_to_drop': '卸货中',
  'paused': '已暂停',
  'completed': '已完成',
  'error': '错误',
};

export function TaskTimeline() {
  const currentCycle = useTaskStore((s) => s.currentCycle);
  const totalCycles = useTaskStore((s) => s.totalCycles);
  const completedKg = useTaskStore((s) => s.completedKg);
  const phase = useTaskStore((s) => s.phase);
  const isRunning = useTaskStore((s) => s.isRunning);

  const percentage = totalCycles > 0 ? Math.round((currentCycle / totalCycles) * 100) : 0;

  return (
    <div className="space-y-3">
      <div className="flex items-center justify-between">
        <span className="text-[12px] text-text-muted">当前阶段</span>
        <span className={`text-[12px] font-bold ${isRunning ? 'text-cyan-400' : 'text-text-muted'}`}>
          {phaseLabels[phase] || phase}
        </span>
      </div>

      <div className="flex items-center justify-between">
        <span className="text-[12px] text-text-muted">循环进度</span>
        <span className="text-[12px] font-bold text-cyan-400 font-mono">
          {currentCycle}/{totalCycles}
        </span>
      </div>

      <div className="flex items-center justify-between">
        <span className="text-[12px] text-text-muted">已完成</span>
        <span className="text-[12px] font-bold text-tech-green font-mono">
          {completedKg} kg
        </span>
      </div>

      <div className="h-2 bg-bg-primary/60 rounded-full overflow-hidden">
        <div
          className="h-full bg-gradient-to-r from-cyan-500 to-tech-blue rounded-full transition-all duration-300"
          style={{ width: `${percentage}%` }}
        />
      </div>

      <div className="text-center text-[11px] text-text-muted">
        {percentage}% 完成
      </div>
    </div>
  );
}
