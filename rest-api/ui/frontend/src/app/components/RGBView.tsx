import { useEffect, useRef } from 'react';

interface RGBViewProps {
  isActive: boolean;
  stream: MediaStream | null;
  metrics?: { width: number; height: number; fps: number } | null;
}

export function RGBView({ isActive, stream, metrics }: RGBViewProps) {
  const videoRef = useRef<HTMLVideoElement>(null);

  useEffect(() => {
    if (videoRef.current) {
      if (stream) {
        videoRef.current.srcObject = stream;
      } else {
        // 流断开时清除残留帧
        videoRef.current.srcObject = null;
        videoRef.current.load();
      }
    }
  }, [stream]);

  return (
    <div className="relative w-full h-full bg-[#1c1c1e] rounded-2xl overflow-hidden group border border-white/5 shadow-md">
      {/* 悬浮状态胶囊 */}
      <div className="absolute top-4 left-4 z-10 flex items-center gap-2 bg-black/40 backdrop-blur-md px-3 py-1.5 rounded-full border border-white/10">
        {isActive ? (
          <div className="w-2.5 h-2.5 bg-red-500 rounded-full animate-pulse shadow-[0_0_8px_rgba(239,68,68,0.5)]" />
        ) : (
          <div className="w-2.5 h-2.5 bg-slate-500 rounded-full" />
        )}
        <span className="text-[10px] text-slate-100 font-bold tracking-widest uppercase">Live</span>
        {metrics && isActive && (
          <span className="text-[10px] text-slate-400 border-l border-white/20 pl-2 ml-1 font-medium">
            {metrics.width}×{metrics.height} // {metrics.fps} FPS
          </span>
        )}
      </div>

      <div className="absolute top-4 right-4 z-10 flex items-center bg-black/40 backdrop-blur-md px-3 py-1.5 rounded-full border border-white/10 opacity-0 group-hover:opacity-100 transition-opacity">
        <span className="text-[10px] text-slate-300 font-bold tracking-widest uppercase">RGB Camera // 01</span>
      </div>

      <video
        ref={videoRef}
        autoPlay
        playsInline
        muted
        className="w-full h-full object-cover grayscale-[0.2] contrast-[1.1]"
      />

      {/* Subtle overlay for industrial look */}
      <div className="absolute inset-0 pointer-events-none border-[12px] border-black/5" />
    </div>
  );
}
