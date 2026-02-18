import { useState, useEffect } from 'react';
import { RobotDisplay } from './components/RobotDisplay';
import { RGBView } from './components/RGBView';
import { DepthView } from './components/DepthView';
import { PointCloudView } from './components/PointCloudView';
import { ControlPanel } from './components/ControlPanel';
import { StatusPanel } from './components/StatusPanel';
import { CommandCenter } from './components/CommandCenter';
import { Card, CardContent, CardHeader, CardTitle } from './components/ui/card';
import { Tabs, TabsContent, TabsList, TabsTrigger } from './components/ui/tabs';
import { Toaster } from './components/ui/sonner';
import { Bot } from 'lucide-react';
import { useRobotConnection } from './hooks/useRobotConnection';

export default function App() {
  // Connection Hook
  const {
    device,
    isStreaming,
    rgbStream,
    depthStream,
    pointCloudData,
    error,
    startConnection,
    stopConnection
  } = useRobotConnection();

  // Control States (Local for now, until backend has control API)
  const [power, setPower] = useState(75);
  const [speed, setSpeed] = useState(50);
  const [volume, setVolume] = useState(60);
  const [sensorsEnabled, setSensorsEnabled] = useState(true);

  // Robot States
  const [armRotation, setArmRotation] = useState(0);
  const [headRotation, setHeadRotation] = useState(0);

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

  // Simulate robot movements only when streaming (or just simulating for now as feedback)
  useEffect(() => {
    if (!isStreaming || power === 0) return;

    const interval = setInterval(() => {
      setArmRotation((prev) => {
        const newRotation = prev + (Math.random() - 0.5) * 10;
        return Math.max(-30, Math.min(30, newRotation));
      });

      setHeadRotation((prev) => {
        const newRotation = prev + (Math.random() - 0.5) * 8;
        return Math.max(-25, Math.min(25, newRotation));
      });
    }, 1000);

    return () => clearInterval(interval);
  }, [isStreaming, power]);

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
    setArmRotation(0);
    setHeadRotation(0);
    setBattery(85);
    setCpu(45);
    setTemperature(38);
    setSignal(92);
  };

  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-950 via-slate-900 to-slate-950 text-white p-6">
      <Toaster />

      <div className="max-w-[1800px] mx-auto space-y-6">
        {/* Header with Command Center */}
        <div className="flex items-center justify-between mb-8">
          <div className="flex items-center gap-4">
            <div className="p-3 bg-blue-600 rounded-xl">
              <Bot className="w-8 h-8" />
            </div>
            <div>
              <h1 className="text-3xl tracking-tight">restrealsenseMonitor</h1>
              <p className="text-slate-400 text-sm">
                AIROS - 自主智能机器人操作系统
                {device ? ` - ${device.name} (${device.device_id})` : ' - 寻找设备中...'}
              </p>
            </div>
          </div>

          {/* Command Center Buttons */}
          <CommandCenter
            isRunning={isStreaming}
            onToggleRunning={handleToggleRunning}
            onReset={handleReset}
          />
        </div>

        {error && (
          <div className="bg-red-500/10 border border-red-500/50 text-red-500 p-4 rounded-lg">
            错误: {error}
          </div>
        )}

        {/* Image Views Row - RGB, Depth, Point Cloud */}
        <div className="grid grid-cols-3 gap-4 h-[280px] w-full">
          <RGBView isActive={isStreaming} stream={rgbStream} />
          <DepthView isActive={isStreaming} stream={depthStream} />
          <PointCloudView isActive={isStreaming} points={pointCloudData} />
        </div>

        {/* Main Layout */}
        <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
          {/* Robot Display */}
          <Card className="lg:col-span-2 bg-slate-900/50 border-slate-800 backdrop-blur">
            <CardContent className="p-6 h-[500px]">
              <RobotDisplay
                power={power}
                armRotation={armRotation}
                headRotation={headRotation}
              />
            </CardContent>
          </Card>

          {/* Status Panel */}
          <Card className="bg-slate-900/50 border-slate-800 backdrop-blur">
            <CardHeader>
              <CardTitle className="text-lg">系统状态</CardTitle>
            </CardHeader>
            <CardContent>
              <StatusPanel
                battery={battery}
                cpu={cpu}
                temperature={temperature}
                signal={signal}
              />
            </CardContent>
          </Card>
        </div>

        {/* Control Panel */}
        <Card className="bg-slate-900/50 border-slate-800 backdrop-blur">
          <CardHeader>
            <CardTitle className="text-lg">控制面板</CardTitle>
          </CardHeader>
          <CardContent>
            <Tabs defaultValue="controls" className="w-full">
              <TabsList className="grid w-full grid-cols-2 mb-6">
                <TabsTrigger value="controls">控制</TabsTrigger>
                <TabsTrigger value="diagnostics">诊断</TabsTrigger>
              </TabsList>

              <TabsContent value="controls" className="space-y-6">
                <ControlPanel
                  power={power}
                  speed={speed}
                  volume={volume}
                  onPowerChange={(value) => setPower(value[0])}
                  onSpeedChange={(value) => setSpeed(value[0])}
                  onVolumeChange={(value) => setVolume(value[0])}
                  sensorsEnabled={sensorsEnabled}
                  onSensorsToggle={setSensorsEnabled}
                />
              </TabsContent>

              <TabsContent value="diagnostics" className="space-y-4">
                <div className="grid grid-cols-2 gap-4 text-sm font-mono">
                  <div className="p-4 bg-slate-950/50 rounded-lg border border-slate-800">
                    <div className="text-slate-400 text-xs mb-1">运行时间</div>
                    <div className="text-blue-400">02:34:18</div>
                  </div>
                  <div className="p-4 bg-slate-950/50 rounded-lg border border-slate-800">
                    <div className="text-slate-400 text-xs mb-1">移动距离</div>
                    <div className="text-blue-400">1.2 km</div>
                  </div>
                  <div className="p-4 bg-slate-950/50 rounded-lg border border-slate-800">
                    <div className="text-slate-400 text-xs mb-1">电机状态</div>
                    <div className="text-green-400">正常</div>
                  </div>
                  <div className="p-4 bg-slate-950/50 rounded-lg border border-slate-800">
                    <div className="text-slate-400 text-xs mb-1">传感器</div>
                    <div className="text-green-400">{sensorsEnabled ? '启用' : '禁用'}</div>
                  </div>
                </div>
              </TabsContent>
            </Tabs>
          </CardContent>
        </Card>
      </div>
    </div>
  );
}
