import { useEffect, useRef } from 'react';
import { Video } from 'lucide-react';

interface VideoViewProps {
  isActive: boolean;
  rgbStream: MediaStream | null;
  rgbMetrics?: { width: number; height: number; fps: number } | null;
}

export function VideoView({ isActive, rgbStream, rgbMetrics }: VideoViewProps) {
  const videoRef = useRef<HTMLVideoElement>(null);

  useEffect(() => {
    if (videoRef.current) {
      if (rgbStream) {
        videoRef.current.srcObject = rgbStream;
      } else {
        // 流断开时清除残留帧
        videoRef.current.srcObject = null;
        videoRef.current.load();
      }
    }
  }, [rgbStream]);

  return (
    <div className="relative w-full h-full bg-[#1c1c1e] rounded-[10px] overflow-hidden group shadow-md">
      {/* 悬浮状态胶囊 */}
      <div className="absolute top-[10px] left-1/2 -translate-x-1/2 z-10 flex items-center justify-center gap-2 bg-[#1c1c1e]/90 backdrop-blur-md px-6 py-2 rounded-full border border-[#FD802E]/60 shadow-[0_0_10px_rgba(253,128,46,0.2)]">
        {isActive ? (
          <div className="w-3 h-3 bg-[#FD802E] rounded-full animate-pulse shadow-[0_0_8px_rgba(253,128,46,0.8)]" />
        ) : (
          <div className="w-3 h-3 bg-slate-500 rounded-full" />
        )}
        <Video className="w-3.5 h-3.5 text-[#FD802E]" />
        <span className="text-[10px] text-[#FD802E] font-bold tracking-widest uppercase font-mono"> RGB | </span>
        {rgbMetrics && isActive && (
          <span className="text-[10px] text-[#FD802E]/80 border-l border-[#FD802E]/30 pl-2 ml-1 font-mono font-bold">
            {rgbMetrics.width}×{rgbMetrics.height} @ {rgbMetrics.fps} FPS
          </span>
        )}
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

      {/* Mode indicator badge */}
      <div className="absolute bottom-[10px] left-[10px] z-10 flex items-center gap-2 bg-black/60 backdrop-blur-md px-3 py-1.5 rounded-full border border-[#FD802E]/40 shadow-[0_0_8px_rgba(253,128,46,0.1)]">
        <div className="w-1.5 h-1.5 rounded-full bg-[#FD802E]" />
        <span className="text-[9px] text-[#FD802E] font-bold tracking-tight font-mono uppercase">RGB MODE</span>
      </div>
    </div>
  );
}
