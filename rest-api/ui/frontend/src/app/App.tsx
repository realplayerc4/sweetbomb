import { useState } from 'react';
import { Slider } from './components/ui/slider';
import { Label } from './components/ui/label';
import { Camera, Monitor, ArrowUpDown, Map } from 'lucide-react';


import { RGBView } from './components/RGBView';
import { DepthView } from './components/DepthView';
import { PointCloudView } from './components/PointCloudView';
import { BEVSliceView } from './components/BEVSliceView';
import { ControlPanel } from './components/ControlPanel';
import { StatusPanel } from './components/StatusPanel';
import { CommandCenter } from './components/CommandCenter';
import { TaskPanel } from './components/TaskPanel';
import { Card, CardContent, CardHeader, CardTitle } from './components/ui/card';
import { Tabs, TabsContent, TabsList, TabsTrigger } from './components/ui/tabs';
import { Toaster } from './components/ui/sonner';
import { Bot } from 'lucide-react';
import { useRobotConnection } from './hooks/useRobotConnection';

export default function App() {
  // Advanced PC/BEV Parameters
  const [cameraHeight, setCameraHeight] = useState(1.0); // 1.0m default
  const [bucketHeight, setBucketHeight] = useState(0.25); // 0.25m default (middle of 0.1~0.5)
  const [tolerance, setTolerance] = useState(0.05); // 5cm default
  const [pcCamZ, setPcCamZ] = useState(3.0); // Point cloud view Z height
  const [pcCamX, setPcCamX] = useState(-5.0); // Point cloud view X position

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
  } = useRobotConnection(cameraHeight);

  // Control States (Local for now, until backend has control API)
  const [power, setPower] = useState(75);
  const [speed, setSpeed] = useState(50);
  const [volume, setVolume] = useState(60);
  const [sensorsEnabled, setSensorsEnabled] = useState(true);



  // Status States (Now from backend, defaults for initial load)
  const battery = systemStats?.battery ?? 85;
  const cpu = systemStats?.cpu_load ?? 0;
  const temperature = systemStats?.temperature ?? 38;
  const signal = systemStats?.signal ?? 92;
  const imu = systemStats?.imu;

  // Toggle Streaming
  const handleToggleRunning = () => {
    if (isStreaming) {
      stopConnection();
    } else {
      startConnection();
    }
  };

  const handleReset = () => {
    setPower(75);
    setSpeed(50);
    setVolume(60);
    setSensorsEnabled(true);
    stopConnection();
  };

  return (
    <div className="min-h-screen bg-[#403D39] text-white p-6">
      <Toaster />

      <div className="max-w-[1800px] mx-auto space-y-6">
        {/* Header with Command Center */}
        <div className="flex items-center justify-between mb-8">
          <div className="flex items-center gap-4">
            <div className="p-3 bg-[#FD802E] rounded-xl shadow-[0_0_20px_rgba(253,128,46,0.3)]">
              <Bot className="w-8 h-8" />
            </div>
            <div className="flex items-baseline gap-3">
              <h1 className="text-3xl font-bold tracking-tight text-[#FD802E]">制糖工程智能技术创新中心</h1>
              <p className="text-slate-300 text-sm font-medium">
                NSFOL-自主智能机器人操作系统
                {device ? ` | 硬件ID: ${device.device_id}` : ' | 寻找设备中...'}
              </p>
            </div>
          </div>

          {/* Command Center Buttons */}
          <CommandCenter
            isRunning={isStreaming}
            onToggleRunning={handleToggleRunning}
            onReset={handleReset}
            deviceId={device?.device_id}
          />
        </div>

        {error && (
          <div className="bg-red-500/10 border border-red-500/50 text-red-500 p-4 rounded-lg">
            错误: {error}
          </div>
        )}

        {/* Image Views Row - RGB, Depth, Point Cloud */}
        <div className="grid grid-cols-3 gap-4 h-[280px] w-full mb-[20px]">
          <RGBView isActive={isStreaming} stream={rgbStream} metrics={streamMetrics?.rgb} />
          <DepthView isActive={isStreaming} stream={depthStream} metrics={streamMetrics?.depth} />
          <PointCloudView isActive={isStreaming} points={pointCloudData} metrics={{ pointCount: streamMetrics?.pointCount }} camZ={pcCamZ} camX={pcCamX} />
        </div>

        {/* Bottom Area */}
        <div className="grid grid-cols-3 gap-4 min-h-[300px]">

          {/* Left Column (Col 1): BEV Canvas + Control Panel Tabs Exactly Under RGB */}
          <div className="col-span-1 flex flex-col gap-4">

            {/* BEV Canvas */}
            <div className="w-full h-[300px] shrink-0">
              <BEVSliceView
                isActive={isStreaming}
                points={pointCloudData}
                targetHeight={bucketHeight}
                tolerance={tolerance}
                cameraHeight={cameraHeight}
              />
            </div>

            {/* Control Panel Tabs immediately below SLICE view */}
            <Card className="bg-slate-900/50 border-none backdrop-blur w-full flex-grow">
              <CardHeader>
                <CardTitle className="text-lg">控制面板</CardTitle>
              </CardHeader>
              <CardContent>
                <Tabs defaultValue="controls" className="w-full">
                  <TabsList className="grid w-full grid-cols-3 mb-6">
                    <TabsTrigger value="controls">控制</TabsTrigger>
                    <TabsTrigger value="tasks">任务</TabsTrigger>
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

                  <TabsContent value="tasks" className="space-y-4">
                    <TaskPanel deviceId={device?.device_id} />
                  </TabsContent>

                  <TabsContent value="diagnostics" className="space-y-4">
                    <div className="grid grid-cols-2 gap-4 text-sm font-mono">
                      <div className="p-4 bg-slate-950/50 rounded-lg border border-slate-800">
                        <div className="text-slate-400 text-xs mb-1">运行时间</div>
                        <div className="text-orange-400">02:34:18</div>
                      </div>
                      <div className="p-4 bg-slate-950/50 rounded-lg border border-slate-800">
                        <div className="text-slate-400 text-xs mb-1">移动距离</div>
                        <div className="text-orange-400">1.2 km</div>
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

          {/* Center Column (Col 2): BEV Settings Panel exactly under Depth */}
          <Card className="bg-slate-900/50 border-2 border-[#FD802E] backdrop-blur col-span-1 h-[300px] flex flex-col">
            <CardHeader className="pb-0 pt-2 px-8">
              <CardTitle className="text-base text-[#FD802E]">点云视觉与切片设置</CardTitle>
            </CardHeader>
            <CardContent className="space-y-2 overflow-y-auto px-10 custom-scrollbar flex-grow pb-2">

              <div className="space-y-1">
                <div className="flex items-center justify-between">
                  <Label className="flex items-center gap-2 text-sm text-[#FD802E]">
                    <Camera className="w-4 h-4" />
                    观测高度 (Z)
                  </Label>
                  <span className="text-sm font-mono text-orange-400">{pcCamZ.toFixed(1)} m</span>
                </div>
                <Slider
                  value={[pcCamZ]}
                  onValueChange={(val) => setPcCamZ(val[0])}
                  max={10.0}
                  min={0.1}
                  step={0.1}
                  className="w-full"
                />
              </div>

              <div className="space-y-1">
                <div className="flex items-center justify-between">
                  <Label className="flex items-center gap-2 text-sm text-[#FD802E]">
                    <Camera className="w-4 h-4" />
                    观测位置 (X)
                  </Label>
                  <span className="text-sm font-mono text-orange-400">{pcCamX.toFixed(1)} m</span>
                </div>
                <Slider
                  value={[pcCamX]}
                  onValueChange={(val) => setPcCamX(val[0])}
                  max={5.0}
                  min={-15.0}
                  step={0.1}
                  className="w-full"
                />
              </div>

              <div className="space-y-1 pt-1">
                <div className="flex items-center justify-between">
                  <Label className="flex items-center gap-2 text-sm text-[#FD802E]">
                    <Camera className="w-4 h-4" />
                    相机高度基准
                  </Label>
                  <span className="text-sm font-mono text-orange-400">{cameraHeight.toFixed(2)} m</span>
                </div>
                <Slider
                  value={[cameraHeight]}
                  onValueChange={(val) => setCameraHeight(val[0])}
                  max={3.0}
                  min={0.0}
                  step={0.05}
                  className="w-full"
                />
              </div>

              <div className="space-y-1">
                <div className="flex items-center justify-between">
                  <Label className="flex items-center gap-2 text-sm text-[#FD802E]">
                    <Monitor className="w-4 h-4" />
                    铲斗高度基准
                  </Label>
                  <span className="text-sm font-mono text-orange-400">{bucketHeight.toFixed(2)} m</span>
                </div>
                <Slider
                  value={[bucketHeight]}
                  onValueChange={(val) => setBucketHeight(val[0])}
                  max={0.5}
                  min={0.1}
                  step={0.01}
                  className="w-full"
                />
              </div>

              <div className="space-y-1">
                <div className="flex items-center justify-between">
                  <Label className="flex items-center gap-2 text-sm text-[#FD802E]">
                    <ArrowUpDown className="w-4 h-4" />
                    切面厚度 (容差)
                  </Label>
                  <span className="text-sm font-mono text-orange-400">±{tolerance.toFixed(2)} m</span>
                </div>
                <Slider
                  value={[tolerance]}
                  onValueChange={(val) => setTolerance(val[0])}
                  max={0.5}
                  min={0.01}
                  step={0.01}
                  className="w-full"
                />
              </div>
            </CardContent>
          </Card>

          {/* Right Column (Col 3): Map Placeholder exactly under Point Cloud */}
          <Card className="bg-slate-900/50 border-none backdrop-blur col-span-1 h-[300px] flex flex-col items-center justify-center relative overflow-hidden group">
            {/* Map Grid Background Pattern */}
            <div className="absolute inset-0 opacity-10"
              style={{ backgroundImage: 'radial-gradient(#4b5563 1px, transparent 1px)', backgroundSize: '20px 20px' }} />

            <div className="flex flex-col items-center gap-3 relative z-10 opacity-60 group-hover:opacity-100 transition-opacity px-8">
              <div className="p-4 bg-slate-800 rounded-full">
                <Map className="w-8 h-8 text-slate-400" />
              </div>
              <div className="text-sm font-medium text-slate-400">
                机器人位姿底图
              </div>
              <div className="text-xs text-slate-500 max-w-[200px] text-center">
                等待 ROS 2 Navigation/SLAM 建图话题接入...
              </div>
            </div>
          </Card>

        </div>

      </div>

      {/* Footer Status Bar 70% opacity */}
      <div className="fixed bottom-0 left-0 right-0 bg-slate-950/70 backdrop-blur-md z-50 px-6 py-3 border-t border-slate-800/50">
        <div className="max-w-[1800px] mx-auto flex items-center justify-end gap-8">
          <span className="text-sm font-bold text-[#FD802E]">系统状态</span>
          <StatusPanel
            battery={battery}
            cpu={cpu}
            temperature={temperature}
            signal={signal}
            imu={imu}
          />
        </div>
      </div>

    </div>
  );
}
