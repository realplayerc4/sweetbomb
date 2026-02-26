import { Play, Pause, RotateCcw, Power, ShieldCheck, Wifi } from 'lucide-react';

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

      <div className="flex items-center gap-2 p-1.5 bg-[#2c2c2e] rounded-2xl border border-white/5 shadow-inner">
        <button
          onClick={onToggleRunning}
          className={`flex items-center gap-2 px-6 py-2.5 rounded-xl font-bold text-xs tracking-widest transition-all ${isRunning
              ? 'bg-red-500/10 text-red-500 hover:bg-red-500/20 border border-red-500/20'
              : 'bg-orange-500 text-black hover:bg-orange-400 shadow-lg shadow-orange-500/20'
            }`}
        >
          {isRunning ? <Pause className="w-4 h-4 fill-current" /> : <Play className="w-4 h-4 fill-current" />}
          {isRunning ? '停止骨干网' : '启动骨干网'}
        </button>

        <button
          onClick={onReset}
          className="p-2.5 text-slate-400 hover:text-white hover:bg-white/5 rounded-xl transition-colors"
          title="Reset System"
        >
          <RotateCcw className="w-4 h-4" />
        </button>
      </div>

      <div className="h-8 w-px bg-white/5 mx-2" />

      <button className="flex items-center gap-2 px-4 py-2.5 bg-red-500/10 text-red-500 border border-red-500/20 rounded-xl hover:bg-red-500/20 transition-all font-bold text-xs tracking-widest">
        <Power className="w-4 h-4" />
        紧急停机
      </button>
    </div>
  );
}
