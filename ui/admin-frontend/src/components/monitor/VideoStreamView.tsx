/**
 * 视频流显示组件
 * 用于显示 WebRTC 视频流（RGB 或深度）
 */

import { useEffect, useRef } from 'react';
import { cn } from '@/lib/utils';

interface VideoStreamViewProps {
  stream: MediaStream | null;
  type: 'rgb' | 'depth';
  isConnected: boolean;
  metrics?: { width: number; height: number; fps: number } | null;
  deviceName?: string;
}

export function VideoStreamView({ stream, type, isConnected, metrics, deviceName }: VideoStreamViewProps) {
  const videoRef = useRef<HTMLVideoElement>(null);

  useEffect(() => {
    if (videoRef.current && stream) {
      videoRef.current.srcObject = stream;
    }
  }, [stream]);

  return (
    <div className="relative w-full h-full">
      {stream ? (
        <video
          ref={videoRef}
          autoPlay
          playsInline
          muted
          className="w-full h-full object-cover rounded"
        />
      ) : (
        <div className="w-full h-full bg-[#0a1428] rounded flex items-center justify-center">
          <div className="text-center">
            <div className="text-gray-500 text-sm">无视频流</div>
            <div className="text-cyan-400/50 text-xs mt-1">
              {type === 'rgb' ? 'RGB' : 'DEPTH'} 传感器离线
            </div>
          </div>
        </div>
      )}

      {/* 顶部标签 */}
      <div className="absolute top-2 left-1/2 -translate-x-1/2 flex items-center gap-2 bg-black/70 backdrop-blur-sm px-3 py-1 rounded-full border border-cyan-500/30">
        <div className={cn(
          'w-2 h-2 rounded-full',
          isConnected && stream ? 'bg-cyan-400 animate-pulse shadow-[0_0_6px_rgba(0,212,255,0.8)]' : 'bg-gray-500'
        )} />
        <span className="text-[10px] text-cyan-400 font-bold tracking-widest font-mono uppercase">
          {type === 'rgb' ? 'RGB' : 'DEPTH'}
        </span>
        {isConnected && stream && metrics && (
          <span className="text-[10px] text-cyan-400/70 font-mono border-l border-cyan-500/30 pl-2 ml-1">
            {metrics.width}×{metrics.height} @ {metrics.fps} FPS
          </span>
        )}
      </div>

      {/* 底部设备名 */}
      {deviceName && (
        <div className="absolute bottom-2 left-2 flex items-center gap-1.5 bg-black/60 backdrop-blur-sm px-2 py-1 rounded-full border border-cyan-500/20">
          <div className={cn(
            'w-1.5 h-1.5 rounded-full',
            isConnected && stream ? 'bg-cyan-400' : 'bg-gray-500'
          )} />
          <span className="text-[9px] text-cyan-400 font-bold tracking-tight font-mono uppercase">
            {deviceName}
          </span>
        </div>
      )}
    </div>
  );
}