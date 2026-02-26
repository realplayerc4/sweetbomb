import { useState, useEffect } from 'react';
import { Bot, Terminal, Activity, Settings2, BarChart3, ShieldAlert } from 'lucide-react';
import { useRobotConnection } from './hooks/useRobotConnection';

import { RGBView } from './components/RGBView';
import { DepthView } from './components/DepthView';
import { PointCloudView } from './components/PointCloudView';
import { ControlPanel } from './components/ControlPanel';
import { StatusPanel } from './components/StatusPanel';
import { CommandCenter } from './components/CommandCenter';
import { TaskPanel } from './components/TaskPanel';
import { Toaster } from './components/ui/sonner';
import { BEVSliceView } from './components/BEVSliceView';

export default function App() {
  // Connection Hook
  const {
    device,
    isStreaming,
    rgbStream,
    depthStream,
    pointCloudData,
    streamMetrics,
    systemStats,
    error,
    startConnection,
    stopConnection
  } = useRobotConnection();

  // Control States (Local for now, until backend has control API)
  const [power, setPower] = useState(75);
  const [speed, setSpeed] = useState(50);
  const [volume, setVolume] = useState(60);
  const [sensorsEnabled, setSensorsEnabled] = useState(true);



  // Status States
  const [battery, setBattery] = useState(85);
  const [cpu, setCpu] = useState(45);
  const [temperature, setTemperature] = useState(38);
  const [signal, setSignal] = useState(92);
  const [imu, setImu] = useState({ roll: 0, pitch: 0, yaw: 0 });

  // Robot Configuration & PointCloud View Control
  const [cameraHeight, setCameraHeight] = useState(0.65);
  const [bucketHeight, setBucketHeight] = useState(0.25);
  const [tolerance, setTolerance] = useState(0.33);
  const [pcCamZ, setPcCamZ] = useState(3.0);
  const [pcCamX, setPcCamX] = useState(-5.0);

  // Toggle Streaming
  const handleToggleRunning = () => {
    if (isStreaming) {
      stopConnection();
    } else {
      startConnection();
    }
  };



  // Simulate status changes
  useEffect(() => {
    if (!isStreaming || power === 0) return;

    const interval = setInterval(() => {
      setBattery((prev) => Math.max(0, prev - 0.1));
      setCpu((prev) => Math.min(100, Math.max(20, prev + (Math.random() - 0.5) * 10)));
      setTemperature((prev) => Math.min(100, Math.max(25, prev + (Math.random() - 0.5) * 3)));
      setSignal((prev) => Math.min(100, Math.max(50, prev + (Math.random() - 0.5) * 5)));
    }, 2000);

    return () => clearInterval(interval);
  }, [isStreaming, power]);

  const handleReset = () => {
    setPower(75);
    setSpeed(50);
    setVolume(60);
    setSensorsEnabled(true);
    stopConnection();

    setBattery(85);
    setCpu(45);
    setTemperature(38);
    setSignal(92);
  };

  return (
    <div className="min-h-screen bg-[#121214] text-slate-200 font-inter selection:bg-orange-500/30">
      <Toaster position="top-right" theme="dark" />

      <div className="max-w-[1900px] mx-auto p-6 lg:p-10 flex flex-col gap-8">
        {/* Header - Industrial Card Style */}
        <header className="flex items-center justify-between p-6 bg-[#1c1c1e] rounded-2xl border border-white/5 shadow-md">
          <div className="flex items-center gap-6">
            <div className="p-4 bg-orange-500 rounded-2xl shadow-lg shadow-orange-500/20">
              <Bot className="w-8 h-8 text-black" />
            </div>
            <div className="flex flex-col">
              <h1 className="text-2xl font-bold tracking-tight text-white">制糖工程智能技术创新中心</h1>
              <p className="text-slate-500 text-sm font-medium">
                AIROS - 自主智能机器人操作系统 // ID: 250337
                {device && <span className="text-orange-500 ml-2">● {device.name}</span>}
              </p>
            </div>
          </div>

          <CommandCenter
            isRunning={isStreaming}
            onToggleRunning={handleToggleRunning}
            onReset={handleReset}
          />
        </header>

        {error && (
          <div className="bg-red-500/10 border border-red-500/50 text-red-500 p-4 rounded-lg">
            错误: {error}
          </div>
        )}

        {/* Image Views Row - RGB, Depth, Point Cloud */}
        <div className="grid grid-cols-3 gap-8 h-[350px] w-full">
          <RGBView isActive={isStreaming} stream={rgbStream} metrics={streamMetrics?.rgb} />
          <DepthView isActive={isStreaming} stream={depthStream} metrics={streamMetrics?.depth} />
          <PointCloudView
            isActive={isStreaming}
            points={pointCloudData}
            metrics={{ pointCount: streamMetrics?.pointCount }}
            camZ={pcCamZ}
            camX={pcCamX}
          />
        </div>

        {/* Bottom Area */}
        <div className="grid grid-cols-3 gap-8 min-h-[400px]">
          {/* Left Column (Col 1): BEV Canvas + Control Panel Tabs Exactly Under RGB */}
          <div className="col-span-1 flex flex-col gap-8">
            <div className="w-full h-[350px] shrink-0 bg-[#1c1c1e] rounded-2xl overflow-hidden border border-white/5 shadow-md">
              <BEVSliceView
                isActive={isStreaming}
                points={pointCloudData}
                targetHeight={bucketHeight}
                tolerance={tolerance}
                cameraHeight={cameraHeight}
              />
            </div>
          </div>

          <div className="col-span-2 grid grid-cols-2 gap-8">
            {/* System Status Panel */}
            <div className="bg-[#1c1c1e] rounded-2xl border border-white/5 shadow-md p-6 flex flex-col gap-6">
              <div className="flex items-center gap-3 mb-2">
                <BarChart3 className="w-5 h-5 text-orange-500" />
                <h2 className="text-sm font-bold tracking-widest uppercase text-slate-100">系统状态 // Telemetry</h2>
              </div>
              <StatusPanel
                battery={battery}
                cpu={cpu}
                temperature={temperature}
                signal={signal}
              />
            </div>

            {/* CommandCenter & Actions Panel */}
            <div className="bg-[#1c1c1e] rounded-2xl border border-white/5 shadow-md p-6 flex flex-col gap-6">
              <div className="flex items-center gap-3 mb-2">
                <Settings2 className="w-5 h-5 text-purple-500" />
                <h2 className="text-sm font-bold tracking-widest uppercase text-slate-100">空间配置 // Spatial Config</h2>
              </div>
              <div className="flex flex-col gap-5">
                {[
                  { label: "观测高度 (Z)", value: pcCamZ, unit: "m", min: 1.0, max: 10.0, step: 0.1, onChange: setPcCamZ },
                  { label: "观测位置 (X)", value: pcCamX, unit: "m", min: -10.0, max: 10.0, step: 0.1, onChange: setPcCamX },
                  { label: "相机高度基准", value: cameraHeight, unit: "m", min: 0.0, max: 2.0, step: 0.01, onChange: setCameraHeight },
                  { label: "铲斗高度基准", value: bucketHeight, unit: "m", min: 0.0, max: 1.0, step: 0.01, onChange: setBucketHeight },
                  { label: "切面厚度 (容差)", value: tolerance, unit: "±m", min: 0.05, max: 1.0, step: 0.01, onChange: setTolerance },
                ].map((cfg) => (
                  <div key={cfg.label} className="flex flex-col gap-2">
                    <div className="flex justify-between text-[11px] font-mono">
                      <span className="text-slate-500">{cfg.label}</span>
                      <span className="text-orange-500 font-bold">{cfg.value.toFixed(2)}{cfg.unit}</span>
                    </div>
                    <input
                      type="range"
                      min={cfg.min}
                      max={cfg.max}
                      step={cfg.step}
                      value={cfg.value}
                      onChange={(e) => cfg.onChange(parseFloat(e.target.value))}
                      className="w-full h-1 bg-white/5 rounded-lg appearance-none cursor-pointer accent-orange-500"
                    />
                  </div>
                ))}
              </div>
            </div>
          </div>
        </div>

        {/* Global Footer Status Bar */}
        <footer className="mt-4 flex items-center justify-between px-6 py-3 bg-black/20 rounded-xl border border-white/5 text-[10px] font-mono tracking-wider text-slate-500">
          <div className="flex gap-6">
            <span className="flex items-center gap-2"><Activity className="w-3 h-3" /> 延迟: 1.2ms</span>
            <span className="flex items-center gap-2 text-green-500/80"><ShieldAlert className="w-3 h-3" /> 系统状态: 标定良好</span>
          </div>
          <div>© 2026 AIROS ROBOTICS // KERNEL v4.12.0-STABLE</div>
        </footer>
      </div>
    </div>
  );
}
