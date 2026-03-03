import { useState, useEffect } from 'react';
import { Bot, Activity, ShieldAlert } from 'lucide-react';
import { useRobotConnection } from './hooks/useRobotConnection';

import { VideoView } from './components/VideoView';
import { DepthView } from './components/DepthView';
import { SliceView } from './components/SliceView';
import { CommandCenter } from './components/CommandCenter';
import { Toaster } from './components/ui/sonner';
import { MapPanel } from './components/MapPanel';
import { RobotControlPanel } from './components/RobotControlPanel';
import { BehaviorTreeViz } from './components/BehaviorTreeViz';

export default function App() {
  // Connection Hook
  const {
    device,
    isStreaming,
    rgbStream,
    depthStream,
    pointCloudData,
    streamMetrics,
    error,
    startConnection,
    stopConnection
  } = useRobotConnection();

  // Control States (Local for now, until backend has control API)
  const [power, setPower] = useState(75);



  // Status States
  const [battery, setBattery] = useState(85);
  const [cpu, setCpu] = useState(45);
  const [temperature, setTemperature] = useState(38);
  const [signal, setSignal] = useState(92);

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

  // Auto-start stream when device is discovered and not currently streaming
  useEffect(() => {
    if (device && !isStreaming && !error) {
      startConnection();
    }
  }, [device, isStreaming, error, startConnection]);

  const handleReset = () => {
    setPower(75);
    stopConnection();

    setBattery(85);
    setCpu(45);
    setTemperature(38);
    setSignal(92);
  };

  return (
    <div className="min-h-screen bg-[#121214] text-slate-200 font-inter selection:bg-orange-500/30">
      <Toaster position="top-right" theme="dark" />

      <div className="max-w-[1900px] mx-auto p-6 lg:p-10 flex flex-col gap-[30px]">
        {/* Header - Industrial Card Style */}
        <header className="flex items-center justify-between p-6 bg-transparent rounded-2xl">
          <div className="flex items-center gap-6">
            <div className="p-4 bg-[#FD802E] rounded-2xl shadow-lg shadow-[#FD802E]/20">
              <Bot className="w-8 h-8 text-black" />
            </div>
            <div className="flex flex-col">
              <h1 className="text-2xl font-bold tracking-tight text-[#FD802E]">制糖工程智能技术创新中心</h1>
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

        {/* First Row: Video(RGB) + Slice + Depth */}
        <div className="grid grid-cols-3 gap-[30px] h-[350px]">
          <VideoView
            isActive={isStreaming}
            rgbStream={rgbStream}
            rgbMetrics={streamMetrics?.rgb}
          />
          <SliceView
            isActive={isStreaming}
            pointCloudData={pointCloudData}
          />
          <DepthView
            isActive={isStreaming}
            depthStream={depthStream}
            depthMetrics={streamMetrics?.depth}
          />
        </div>

        {/* Second Row: Map + Control + BehaviorTree */}
        <div className="grid grid-cols-3 gap-[30px] min-h-[400px]">
          <MapPanel />
          <RobotControlPanel />
          <BehaviorTreeViz
            currentNode=""
            status="idle"
            cycleCount={0}
            maxCycles={10}
            sugarHeight={0.3}
            heightThreshold={0.20}
            stepHistory={[]}
          />
        </div>

        {/* Global Footer Status Bar */}
        <footer className="mt-4 flex flex-col md:flex-row items-center justify-between px-6 py-4 bg-black/20 rounded-xl text-[10px] font-mono tracking-wider text-slate-500 gap-4">
          <div className="flex flex-wrap items-center gap-6">
            <span className="flex items-center gap-2"><Activity className="w-3 h-3" /> 延迟: 1.2ms</span>
            <span className="flex items-center gap-2 text-green-500/80 mr-2"><ShieldAlert className="w-3 h-3" /> 系统状态: 标定良好</span>

            {/* System Status Metrics Moved to Footer */}
            <div className="flex items-center gap-6">
              <div className="flex items-center gap-2 whitespace-nowrap">
                <span className="text-white font-bold tracking-widest">{battery.toFixed(1)}%</span>
                <span>电池电量</span>
                <span className="text-green-500 font-medium">Stable</span>
              </div>
              <div className="flex items-center gap-2 whitespace-nowrap">
                <span className="text-white font-bold tracking-widest">{cpu.toFixed(1)}%</span>
                <span>CPU 负载</span>
                <span className="text-green-500 font-medium">Stable</span>
              </div>
              <div className="flex items-center gap-2 whitespace-nowrap">
                <span className="text-white font-bold tracking-widest">{temperature.toFixed(1)}°C</span>
                <span>环境温度</span>
                <span className="text-green-500 font-medium">Stable</span>
              </div>
              <div className="flex items-center gap-2 whitespace-nowrap">
                <span className="text-white font-bold tracking-widest">{signal.toFixed(1)}%</span>
                <span>信号强度</span>
                <span className="text-green-500 font-medium">Stable</span>
              </div>
            </div>
          </div>
          <div className="shrink-0">© 2026 AIROS ROBOTICS // KERNEL v4.12.0-STABLE</div>
        </footer>
      </div>
    </div>
  );
}
