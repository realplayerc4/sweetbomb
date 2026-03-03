import { useEffect, useRef, useState } from 'react';
import { Video, Layers } from 'lucide-react';

interface VideoViewProps {
  isActive: boolean;
  rgbStream: MediaStream | null;
  depthStream: MediaStream | null;
  rgbMetrics?: { width: number; height: number; fps: number } | null;
  depthMetrics?: { width: number; height: number; fps: number } | null;
}

type VideoMode = 'rgb' | 'depth';

export function VideoView({ isActive, rgbStream, depthStream, rgbMetrics, depthMetrics }: VideoViewProps) {
  const videoRef = useRef<HTMLVideoElement>(null);
  const [mode, setMode] = useState<VideoMode>('rgb');

  // Update video source when mode or streams change
  useEffect(() => {
    if (videoRef.current) {
      const stream = mode === 'rgb' ? rgbStream : depthStream;
      if (stream) {
        videoRef.current.srcObject = stream;
      } else {
        videoRef.current.srcObject = null;
        videoRef.current.load();
      }
    }
  }, [mode, rgbStream, depthStream]);

  const currentMetrics = mode === 'rgb' ? rgbMetrics : depthMetrics;
  const modeLabel = mode === 'rgb' ? 'RGB' : 'Depth';

  return (
    <div className="relative w-full h-full bg-[#1c1c1e] rounded-[10px] overflow-hidden group shadow-md">
      {/* Top Status Capsule */}
      <div className="absolute top-[10px] left-1/2 -translate-x-1/2 z-10 flex items-center justify-center gap-2 bg-[#1c1c1e]/90 backdrop-blur-md px-6 py-2 rounded-full border border-[#FD802E]/60 shadow-[0_0_10px_rgba(253,128,46,0.2)]">
        {isActive ? (
          <div className="w-3 h-3 bg-[#FD802E] rounded-full animate-pulse shadow-[0_0_8px_rgba(253,128,46,0.8)]" />
        ) : (
          <div className="w-3 h-3 bg-slate-500 rounded-full" />
        )}
        {mode === 'rgb' ? (
          <Video className="w-3.5 h-3.5 text-[#FD802E]" />
        ) : (
          <Layers className="w-3.5 h-3.5 text-[#FD802E]" />
        )}
        <span className="text-[10px] text-[#FD802E] font-bold tracking-widest uppercase font-mono"> {modeLabel} | </span>
        {currentMetrics && isActive && (
          <span className="text-[10px] text-[#FD802E]/80 border-l border-[#FD802E]/30 pl-2 ml-1 font-mono font-bold">
            {currentMetrics.width}×{currentMetrics.height} @ {currentMetrics.fps} FPS
          </span>
        )}
      </div>

      {/* Mode Toggle Buttons */}
      <div className="absolute top-[10px] right-[10px] z-10 flex items-center gap-1 bg-[#1c1c1e]/90 backdrop-blur-md rounded-lg border border-slate-700/50 p-1">
        <button
          onClick={() => setMode('rgb')}
          className={`px-3 py-1.5 rounded-md text-[10px] font-bold tracking-wider uppercase transition-all ${
            mode === 'rgb'
              ? 'bg-[#FD802E] text-black shadow-[0_0_8px_rgba(253,128,46,0.5)]'
              : 'text-slate-400 hover:text-slate-300'
          }`}
        >
          RGB
        </button>
        <button
          onClick={() => setMode('depth')}
          className={`px-3 py-1.5 rounded-md text-[10px] font-bold tracking-wider uppercase transition-all ${
            mode === 'depth'
              ? 'bg-[#FD802E] text-black shadow-[0_0_8px_rgba(253,128,46,0.5)]'
              : 'text-slate-400 hover:text-slate-300'
          }`}
        >
          Depth
        </button>
      </div>

      <video
        ref={videoRef}
        autoPlay
        playsInline
        muted
        className={`w-full h-full object-cover ${
          mode === 'rgb' ? 'grayscale-[0.2] contrast-[1.1]' : 'contrast-[1.2] brightness-[1.1]'
        }`}
      />

      {/* Grid overlay for depth view */}
      {mode === 'depth' && (
        <div className="absolute inset-0 pointer-events-none opacity-10"
          style={{ backgroundImage: 'radial-gradient(circle, white 1px, transparent 1px)', backgroundSize: '20px 20px' }}
        />
      )}

      {/* Subtle overlay for industrial look */}
      <div className="absolute inset-0 pointer-events-none border-[12px] border-black/5" />

      {/* Mode indicator badge */}
      <div className="absolute bottom-[10px] left-[10px] z-10 flex items-center gap-2 bg-black/60 backdrop-blur-md px-3 py-1.5 rounded-full border border-slate-700/50">
        <div className={`w-2 h-2 rounded-full ${mode === 'rgb' ? 'bg-blue-400' : 'bg-green-400'}`} />
        <span className="text-[10px] text-slate-300 font-mono uppercase">{modeLabel} MODE</span>
      </div>
    </div>
  );
}
