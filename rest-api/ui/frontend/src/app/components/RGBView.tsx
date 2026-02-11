import { useEffect, useRef } from 'react';
import { Camera } from 'lucide-react';

interface RGBViewProps {
  isActive: boolean;
  stream: MediaStream | null;
}

export function RGBView({ isActive, stream }: RGBViewProps) {
  const videoRef = useRef<HTMLVideoElement>(null);

  useEffect(() => {
    if (videoRef.current && stream) {
      videoRef.current.srcObject = stream;
    }
  }, [stream]);

  return (
    <div className="relative w-full h-full bg-slate-950 rounded-lg overflow-hidden border border-slate-700">
      <div className="absolute top-3 left-3 z-10 flex items-center gap-2 bg-slate-900/80 backdrop-blur-sm px-3 py-1.5 rounded-md">
        <Camera className="w-4 h-4 text-blue-400" />
        <span className="text-xs text-white font-mono">RGB CAMERA</span>
        {isActive && (
          <div className="w-2 h-2 bg-red-500 rounded-full animate-pulse" />
        )}
      </div>
      <video
        ref={videoRef}
        autoPlay
        playsInline
        muted
        className="w-full h-full object-contain"
      />
    </div>
  );
}

