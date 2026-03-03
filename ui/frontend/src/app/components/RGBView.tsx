import { useEffect, useRef } from 'react';
import { Video } from 'lucide-react';

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
        {metrics && isActive && (
          <span className="text-[10px] text-[#FD802E]/80 border-l border-[#FD802E]/30 pl-2 ml-1 font-mono font-bold">
            {metrics.width}×{metrics.height} @ {metrics.fps} FPS
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
    </div>
  );
}
