import { useEffect, useRef, useState } from 'react';
import { cn } from '@/lib/utils';
import { useRobotStore } from '@/stores/useRobotStore';
import { useSystemModeStore } from '@/stores/useSystemModeStore';
import { useSystemStore } from '@/stores/useSystemStore';
import { useWebRTCConnection } from '@/hooks/useWebRTCConnection';
import { VideoStreamView } from './VideoStreamView';

interface SensorMonitorProps {
  type: 'rgb' | 'depth';
}

export function SensorMonitor({ type }: SensorMonitorProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const isConnected = useRobotStore((s) => s.isConnected);
  const monitorMode = useSystemModeStore((s) => s.monitorMode);
  const selectedRobot = useSystemModeStore((s) => s.selectedRobot);
  const devices = useSystemStore((s) => s.devices);
  const [fps, setFps] = useState(0);
  const [resolution, setResolution] = useState({ w: 0, h: 0 });
  const frameCountRef = useRef(0);
  const lastTimeRef = useRef(Date.now());

  // WebRTC 连接
  const { rgbStream, depthStream, isStreaming, streamMetrics } = useWebRTCConnection();

  const onlineDevices = devices.filter((d) => d.status === 'online' || d.status === 'warning');

  // Canvas 模拟动画
  useEffect(() => {
    // 单机器人模式且已连接时，不使用 Canvas
    if (monitorMode === 'single' && isStreaming) return;
    // 多机器人模式且第一个设备有视频流时，不使用 Canvas（第一个格子用 video）
    if (monitorMode === 'all' && isStreaming && onlineDevices.length > 0) return;

    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    let animId: number;

    const drawFrame = () => {
      const now = Date.now();
      frameCountRef.current++;

      if (now - lastTimeRef.current >= 1000) {
        setFps(frameCountRef.current);
        frameCountRef.current = 0;
        lastTimeRef.current = now;
      }

      const w = canvas.width;
      const h = canvas.height;

      ctx.fillStyle = '#0a1428';
      ctx.fillRect(0, 0, w, h);

      if (monitorMode === 'single') {
        if (type === 'rgb') {
          drawRgbSimulation(ctx, w, h, now);
        } else {
          drawDepthSimulation(ctx, w, h, now);
        }
      } else {
        // 多机器人模式：跳过第一个设备（用 video 显示）
        const otherDevices = onlineDevices.slice(1);
        drawMultiRobotViewSkipFirst(ctx, w, h, now, type, otherDevices);
      }

      setResolution({ w, h });
      animId = requestAnimationFrame(drawFrame);
    };

    if (isConnected) {
      animId = requestAnimationFrame(drawFrame);
    } else {
      ctx.fillStyle = '#0a1428';
      ctx.fillRect(0, 0, canvas.width, canvas.height);
      drawOfflineOverlay(ctx, canvas.width, canvas.height, type);
    }

    return () => {
      if (animId) cancelAnimationFrame(animId);
    };
  }, [type, isConnected, monitorMode, onlineDevices, isStreaming]);

  // 单机器人模式且已连接：使用真实视频流
  if (monitorMode === 'single' && isStreaming) {
    const stream = type === 'rgb' ? rgbStream : depthStream;
    const metrics = type === 'rgb' ? streamMetrics.rgb : streamMetrics.depth;

    return (
      <VideoStreamView
        stream={stream}
        type={type}
        isConnected={isConnected}
        metrics={metrics}
        deviceName={selectedRobot}
      />
    );
  }

  // 多机器人模式：第一个设备用真实视频流，其他用 Canvas 模拟
  if (monitorMode === 'all' && isStreaming && onlineDevices.length > 0) {
    const stream = type === 'rgb' ? rgbStream : depthStream;
    const metrics = type === 'rgb' ? streamMetrics.rgb : streamMetrics.depth;
    const otherDevices = onlineDevices.slice(1);
    const count = Math.min(onlineDevices.length, 4);
    const cols = count <= 2 ? count : 2;
    const rows = Math.ceil(count / cols);

    return (
      <div
        className="grid gap-0.5 w-full h-full rounded overflow-hidden"
        style={{ gridTemplateColumns: `repeat(${cols}, 1fr)`, gridTemplateRows: `repeat(${rows}, 1fr)` }}
      >
        {/* 第一个设备：真实视频流 */}
        <div className="relative">
          <VideoStreamView
            stream={stream}
            type={type}
            isConnected={isConnected}
            metrics={metrics}
            deviceName={onlineDevices[0].name}
          />
        </div>

        {/* 其他设备：Canvas 模拟 */}
        {otherDevices.slice(0, 3).map((device, index) => (
          <CanvasCell
            key={device.device_id}
            type={type}
            deviceName={device.name}
            timeOffset={(index + 1) * 1000}
          />
        ))}
      </div>
    );
  }

  // 默认：Canvas 模拟
  return (
    <div className="relative w-full h-full">
      <canvas
        ref={canvasRef}
        width={320}
        height={180}
        className="w-full h-full object-cover rounded"
      />

      <div className="absolute top-2 left-1/2 -translate-x-1/2 flex items-center gap-2 bg-black/70 backdrop-blur-sm px-3 py-1 rounded-full border border-cyan-500/30">
        <div className={cn(
          'w-2 h-2 rounded-full',
          isConnected ? 'bg-cyan-400 animate-pulse shadow-[0_0_6px_rgba(0,212,255,0.8)]' : 'bg-gray-500'
        )} />
        <span className="text-[10px] text-cyan-400 font-bold tracking-widest font-mono uppercase">
          {type === 'rgb' ? 'RGB' : 'DEPTH'}
        </span>
        {isConnected && fps > 0 && (
          <span className="text-[10px] text-cyan-400/70 font-mono border-l border-cyan-500/30 pl-2 ml-1">
            {resolution.w}×{resolution.h} @ {fps} FPS
          </span>
        )}
      </div>

      <div className="absolute bottom-2 left-2 flex items-center gap-1.5 bg-black/60 backdrop-blur-sm px-2 py-1 rounded-full border border-cyan-500/20">
        <div className={cn(
          'w-1.5 h-1.5 rounded-full',
          isConnected ? 'bg-cyan-400' : 'bg-gray-500'
        )} />
        <span className="text-[9px] text-cyan-400 font-bold tracking-tight font-mono uppercase">
          {monitorMode === 'single' ? selectedRobot : `${onlineDevices.length} DEVICES`}
        </span>
      </div>
    </div>
  );
}

// Canvas 单元格组件（用于多机器人视图中的模拟设备）
function CanvasCell({ type, deviceName, timeOffset }: { type: 'rgb' | 'depth'; deviceName: string; timeOffset: number }) {
  const canvasRef = useRef<HTMLCanvasElement>(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    let animId: number;

    const drawFrame = () => {
      const now = Date.now();
      const w = canvas.width;
      const h = canvas.height;

      ctx.fillStyle = '#0a1428';
      ctx.fillRect(0, 0, w, h);

      if (type === 'rgb') {
        drawRgbSimulation(ctx, w, h, now + timeOffset);
      } else {
        drawDepthSimulation(ctx, w, h, now + timeOffset);
      }

      animId = requestAnimationFrame(drawFrame);
    };

    animId = requestAnimationFrame(drawFrame);

    return () => {
      if (animId) cancelAnimationFrame(animId);
    };
  }, [type, timeOffset]);

  return (
    <div className="relative w-full h-full">
      <canvas
        ref={canvasRef}
        width={160}
        height={90}
        className="w-full h-full object-cover"
      />
      <div className="absolute bottom-0 left-0 right-0 bg-black/60 px-1 py-0.5">
        <span className="text-[8px] text-cyan-400 font-mono">{deviceName}</span>
      </div>
    </div>
  );
}

// 多机器人视图：跳过第一个设备（用 video 显示）
function drawMultiRobotViewSkipFirst(ctx: CanvasRenderingContext2D, w: number, h: number, time: number, type: string, devices: { device_id: string; name: string }[]) {
  const count = Math.min(devices.length, 3); // 最多显示 3 个（第一个用 video）
  if (count === 0) return;

  const totalSlots = count + 1; // 包括第一个 video 位置
  const cols = totalSlots <= 2 ? totalSlots : 2;
  const rows = Math.ceil(totalSlots / cols);
  const cellW = w / cols;
  const cellH = h / rows;

  for (let i = 0; i < count; i++) {
    // 跳过第一个位置（索引 0），从第二个位置开始绘制
    const slotIndex = i + 1;
    const col = slotIndex % cols;
    const row = Math.floor(slotIndex / cols);
    const x = col * cellW;
    const y = row * cellH;

    ctx.save();
    ctx.beginPath();
    ctx.rect(x, y, cellW, cellH);
    ctx.clip();

    ctx.translate(x, y);
    if (type === 'rgb') {
      drawRgbSimulation(ctx, cellW, cellH, time + i * 1000);
    } else {
      drawDepthSimulation(ctx, cellW, cellH, time + i * 1000);
    }
    ctx.restore();

    ctx.strokeStyle = 'rgba(0, 212, 255, 0.3)';
    ctx.lineWidth = 1;
    ctx.strokeRect(x, y, cellW, cellH);

    ctx.fillStyle = 'rgba(0, 0, 0, 0.6)';
    ctx.fillRect(x, y + cellH - 14, cellW, 14);
    ctx.fillStyle = 'rgba(0, 212, 255, 0.8)';
    ctx.font = '8px monospace';
    ctx.fillText(devices[i].name, x + 3, y + cellH - 4);
  }
}

function drawRgbSimulation(ctx: CanvasRenderingContext2D, w: number, h: number, time: number) {
  const t = time * 0.001;

  ctx.fillStyle = '#0d1520';
  ctx.fillRect(0, 0, w, h);

  const gridSpacing = 20;
  ctx.strokeStyle = 'rgba(0, 212, 255, 0.06)';
  ctx.lineWidth = 0.5;
  for (let x = 0; x < w; x += gridSpacing) {
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, h);
    ctx.stroke();
  }
  for (let y = 0; y < h; y += gridSpacing) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(w, y);
    ctx.stroke();
  }

  const cx = w / 2 + Math.sin(t * 0.5) * 40;
  const cy = h / 2 + Math.cos(t * 0.3) * 20;
  const gradient = ctx.createRadialGradient(cx, cy, 0, cx, cy, 80);
  gradient.addColorStop(0, 'rgba(0, 212, 255, 0.15)');
  gradient.addColorStop(0.5, 'rgba(22, 119, 255, 0.08)');
  gradient.addColorStop(1, 'transparent');
  ctx.fillStyle = gradient;
  ctx.fillRect(0, 0, w, h);

  ctx.strokeStyle = 'rgba(0, 212, 255, 0.4)';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(cx - 30, cy - 20);
  ctx.lineTo(cx + 30, cy - 20);
  ctx.lineTo(cx + 25, cy + 20);
  ctx.lineTo(cx - 25, cy + 20);
  ctx.closePath();
  ctx.stroke();

  ctx.fillStyle = 'rgba(0, 212, 255, 0.3)';
  ctx.font = '9px monospace';
  ctx.fillText(`POS: ${(cx / w * 10).toFixed(1)}, ${(cy / h * 10).toFixed(1)}`, 8, h - 8);
}

function drawDepthSimulation(ctx: CanvasRenderingContext2D, w: number, h: number, time: number) {
  const t = time * 0.001;

  ctx.fillStyle = '#0a0e14';
  ctx.fillRect(0, 0, w, h);

  const cellSize = 8;
  for (let x = 0; x < w; x += cellSize) {
    for (let y = 0; y < h; y += cellSize) {
      const dist = Math.sqrt((x - w / 2) ** 2 + (y - h / 2) ** 2);
      const wave = Math.sin(dist * 0.03 - t * 2) * 0.5 + 0.5;
      const depth = Math.max(0, Math.min(1, 1 - dist / (w * 0.5)));
      const intensity = depth * wave;

      const r = Math.floor(intensity * 50);
      const g = Math.floor(intensity * 180 + 40);
      const b = Math.floor(intensity * 255);
      ctx.fillStyle = `rgba(${r}, ${g}, ${b}, 0.6)`;
      ctx.fillRect(x, y, cellSize - 1, cellSize - 1);
    }
  }

  ctx.strokeStyle = 'rgba(0, 212, 255, 0.15)';
  ctx.lineWidth = 0.5;
  const crossSize = 20;
  const ccx = w / 2;
  const ccy = h / 2;
  ctx.beginPath();
  ctx.moveTo(ccx - crossSize, ccy);
  ctx.lineTo(ccx + crossSize, ccy);
  ctx.moveTo(ccx, ccy - crossSize);
  ctx.lineTo(ccx, ccy + crossSize);
  ctx.stroke();

  ctx.fillStyle = 'rgba(0, 212, 255, 0.4)';
  ctx.font = '9px monospace';
  ctx.fillText(`DEPTH: ${(Math.sin(t) * 0.5 + 1.5).toFixed(2)}m`, 8, h - 8);
}

function drawOfflineOverlay(ctx: CanvasRenderingContext2D, w: number, h: number, type: string) {
  ctx.fillStyle = 'rgba(0, 0, 0, 0.5)';
  ctx.fillRect(0, 0, w, h);

  ctx.fillStyle = 'rgba(100, 116, 139, 0.6)';
  ctx.font = 'bold 12px sans-serif';
  ctx.textAlign = 'center';
  ctx.fillText('无设备连接', w / 2, h / 2 - 8);
  ctx.fillStyle = 'rgba(0, 212, 255, 0.5)';
  ctx.font = '10px sans-serif';
  ctx.fillText(`${type === 'rgb' ? 'RGB' : 'DEPTH'} 传感器离线`, w / 2, h / 2 + 12);
  ctx.textAlign = 'start';
}