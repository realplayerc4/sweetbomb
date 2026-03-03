import { useEffect, useRef } from 'react';
import { Layers } from 'lucide-react';

interface DepthViewProps {
  isActive: boolean;
  depthStream: MediaStream | null;
  depthMetrics?: { width: number; height: number; fps: number } | null;
}

export function DepthView({ isActive, depthStream, depthMetrics }: DepthViewProps) {
  const videoRef = useRef<HTMLVideoElement>(null);

  useEffect(() => {
    if (videoRef.current) {
      if (depthStream) {
        videoRef.current.srcObject = depthStream;
      } else {
        videoRef.current.srcObject = null;
        videoRef.current.load();
      }
    }
  }, [depthStream]);

  return (
    <div className="relative w-full h-full bg-[#1c1c1e] rounded-[10px] overflow-hidden group shadow-md">
      {/* 悬浮状态胶囊 */}
      <div className="absolute top-[10px] left-1/2 -translate-x-1/2 z-10 flex items-center justify-center gap-2 bg-[#1c1c1e]/90 backdrop-blur-md px-6 py-2 rounded-full border border-[#FD802E]/60 shadow-[0_0_10px_rgba(253,128,46,0.2)]">
        <div className={`w-3 h-3 rounded-full ${isActive ? 'bg-[#FD802E] animate-pulse shadow-[0_0_8px_rgba(253,128,46,0.8)]' : 'bg-slate-500'}`} />
        <Layers className="w-3.5 h-3.5 text-[#FD802E]" />
        <span className="text-[10px] text-[#FD802E] font-bold tracking-widest uppercase font-mono">DEPTH | </span>
        {depthMetrics && isActive && (
          <span className="text-[10px] text-[#FD802E]/80 border-l border-[#FD802E]/30 pl-2 ml-1 font-mono font-bold">
            {depthMetrics.width}×{depthMetrics.height} @ {depthMetrics.fps} FPS
          </span>
        )}
      </div>

      <video
        ref={videoRef}
        autoPlay
        playsInline
        muted
        className="w-full h-full object-cover contrast-[1.2] brightness-[1.1]"
      />

      {/* Grid overlay for depth view */}
      <div className="absolute inset-0 pointer-events-none opacity-10"
        style={{ backgroundImage: 'radial-gradient(circle, white 1px, transparent 1px)', backgroundSize: '20px 20px' }}
      />

      {/* Subtle overlay for industrial look */}
      <div className="absolute inset-0 pointer-events-none border-[12px] border-black/5" />

      {/* Mode indicator badge */}
      <div className="absolute bottom-[10px] left-[10px] z-10 flex items-center gap-2 bg-black/60 backdrop-blur-md px-3 py-1.5 rounded-full border border-slate-700/50">
        <div className="w-2 h-2 rounded-full bg-green-400" />
        <span className="text-[10px] text-slate-300 font-mono uppercase">DEPTH MODE</span>
      </div>
    </div>
  );
}
