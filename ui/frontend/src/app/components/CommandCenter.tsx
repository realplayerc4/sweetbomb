import { Play, Pause, RotateCcw, Power, Wifi } from 'lucide-react';

interface CommandCenterProps {
  isRunning: boolean;
  onToggleRunning: () => void;
  onReset: () => void;
}

export function CommandCenter({ isRunning, onToggleRunning, onReset }: CommandCenterProps) {
  return (
    <div className="flex items-center gap-4">
      {/* Network Status Badge */}
      <div className="hidden lg:flex items-center gap-2 px-4 py-2 bg-black/20 rounded-xl border border-white/5 mr-4">
        <Wifi className="w-4 h-4 text-green-500" />
        <span className="text-[10px] font-bold tracking-widest text-slate-400">NET: STABLE</span>
      </div>

      <div className="flex items-center gap-2 p-1.5">
        <button
          onClick={onToggleRunning}
          className="flex items-center gap-2 px-6 py-2.5 bg-[#FD802E]/10 text-[#FD802E] border border-[#FD802E]/20 rounded-xl hover:bg-[#FD802E]/20 transition-all font-bold text-xs tracking-widest"
        >
          {isRunning ? <Pause className="w-4 h-4 fill-current" /> : <Play className="w-4 h-4 fill-current" />}
          {isRunning ? '停止视觉' : '启动视觉'}
        </button>

        <button
          onClick={onReset}
          className="p-2.5 bg-[#FD802E]/10 text-[#FD802E] border border-[#FD802E]/20 rounded-xl hover:bg-[#FD802E]/20 transition-all"
          title="Reset System"
        >
          <RotateCcw className="w-4 h-4" />
        </button>
      </div>

      <div className="h-8 w-px bg-white/5 mx-2" />

      <button className="flex items-center gap-2 px-4 py-2.5 bg-[#FD802E]/10 text-[#FD802E] border border-[#FD802E]/20 rounded-xl hover:bg-[#FD802E]/20 transition-all font-bold text-xs tracking-widest">
        <Power className="w-4 h-4" />
        紧急停机
      </button>
    </div>
  );
}
