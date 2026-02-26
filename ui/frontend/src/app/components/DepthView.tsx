import { useEffect, useRef } from 'react';

interface DepthViewProps {
  isActive: boolean;
  stream: MediaStream | null;
  metrics?: { width: number; height: number; fps: number } | null;
}

export function DepthView({ isActive, stream, metrics }: DepthViewProps) {
  const videoRef = useRef<HTMLVideoElement>(null);

  useEffect(() => {
    if (videoRef.current) {
      if (stream) {
        videoRef.current.srcObject = stream;
      } else {
        videoRef.current.srcObject = null;
        videoRef.current.load();
      }
    }
  }, [stream]);

  return (
    <div className="relative w-full h-full bg-[#1c1c1e] rounded-2xl overflow-hidden group border border-white/5 shadow-md">
      {/* 悬浮状态胶囊 */}
      <div className="absolute top-4 left-4 z-10 flex items-center gap-2 bg-black/40 backdrop-blur-md px-3 py-1.5 rounded-full border border-white/10">
        <div className={`w-2.5 h-2.5 rounded-full ${isActive ? 'bg-orange-500 animate-pulse' : 'bg-slate-500'}`} />
        <span className="text-[10px] text-slate-100 font-bold tracking-widest uppercase">Depth</span>
        {metrics && isActive && (
          <span className="text-[10px] text-slate-400 border-l border-white/20 pl-2 ml-1 font-medium">
            {metrics.width}×{metrics.height} // {metrics.fps} FPS
          </span>
        )}
      </div>

      <div className="absolute top-4 right-4 z-10 flex items-center bg-black/40 backdrop-blur-md px-3 py-1.5 rounded-full border border-white/10 opacity-0 group-hover:opacity-100 transition-opacity">
        <span className="text-[10px] text-slate-300 font-bold tracking-widest uppercase">Spatial Sensor // 02</span>
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
        style={{ backgroundImage: 'radial-gradient(circle, white 1px, transparent 1px)', backgroundSize: '20px 20px' }} />
    </div>
  );
}
