import { useState, useEffect } from 'react';
import { Bot, Activity, ShieldAlert, Settings } from 'lucide-react';
import { useRobotConnection } from './hooks/useRobotConnection';

import { RGBView } from './components/RGBView';
import { DepthView } from './components/DepthView';
import { PointCloudView } from './components/PointCloudView';
import { CommandCenter } from './components/CommandCenter';
import { Toaster } from './components/ui/sonner';
import { MapPanel } from './components/MapPanel';
import { RobotControlPanel } from './components/RobotControlPanel';
import { SugarHarvestPanel } from './components/SugarHarvestPanel';
import { BehaviorTreeViz } from './components/BehaviorTreeViz';
import { Tabs, TabsContent, TabsList, TabsTrigger } from './components/ui/tabs';

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

  // Robot Control Tab
  const [robotTab, setRobotTab] = useState<'manual' | 'auto' | 'tree'>('manual');



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

      <div className="max-w-[1900px] mx-auto p-6 lg:p-10 flex flex-col gap-8">
        {/* Header - Industrial Card Style */}
        <header className="flex items-center justify-between p-6 bg-[#1c1c1e] rounded-2xl shadow-md">
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

        {/* Image Views Row - RGB, Point Cloud Data Card */}
        <div className="flex gap-[30px] h-[350px] w-full mb-[30px]">
          <div className="flex-1">
            <RGBView isActive={isStreaming} stream={rgbStream} metrics={streamMetrics?.rgb} />
          </div>

          <div className="w-[600px]">
            <PointCloudView
              isActive={isStreaming}
              points={pointCloudData}
            />
          </div>
        </div>

        {/* Bottom Area */}
        <div className="grid grid-cols-2 gap-[30px] min-h-[400px]">
          {/* Col 1: Depth View */}
          <DepthView isActive={isStreaming} stream={depthStream} metrics={streamMetrics?.depth} />

          {/* Col 2: Map Panel */}
          <MapPanel />
        </div>

        {/* Robot Control Section */}
        <div className="bg-[#1c1c1e] rounded-2xl p-6 shadow-md">
          <div className="flex items-center gap-3 mb-6">
            <div className="p-2 bg-[#FD802E]/20 rounded-xl">
              <Settings className="w-5 h-5 text-[#FD802E]" />
            </div>
            <h2 className="text-xl font-bold text-[#FD802E]">铲糖机器人控制</h2>
          </div>

          <Tabs value={robotTab} onValueChange={(v) => setRobotTab(v as any)}>
            <TabsList className="bg-[#121214] border border-slate-700">
              <TabsTrigger value="manual" className="data-[state=active]:bg-[#FD802E] data-[state=active]:text-black">
                手动控制
              </TabsTrigger>
              <TabsTrigger value="auto" className="data-[state=active]:bg-[#FD802E] data-[state=active]:text-black">
                自主铲糖
              </TabsTrigger>
              <TabsTrigger value="tree" className="data-[state=active]:bg-[#FD802E] data-[state=active]:text-black">
                行为树状态
              </TabsTrigger>
            </TabsList>

            <div className="mt-6">
              <TabsContent value="manual" className="m-0">
                <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
                  <RobotControlPanel />
                  <div className="text-sm text-slate-400 space-y-4">
                    <h3 className="text-lg font-semibold text-slate-200">操作说明</h3>
                    <div className="space-y-2">
                      <p>• 使用<strong>虚拟摇杆</strong>或<strong>WASD</strong>键控制移动</p>
                      <p>• 点击<strong>方向按钮</strong>进行精确移动</p>
                      <p>• 拖动<strong>伺服滑块</strong>控制铲斗和翻斗</p>
                      <p>• 使用<strong>铲取/倾倒</strong>按钮快速执行动作</p>
                      <p>• <strong>紧急停止</strong>可立即停止所有运动</p>
                    </div>
                    <div className="mt-4 p-3 bg-slate-800 rounded-lg">
                      <p className="text-xs text-slate-500">键盘控制</p>
                      <p className="text-sm font-mono">W/↑ 前进 | S/↓ 后退 | A/← 左移 | D/→ 右移</p>
                    </div>
                  </div>
                </div>
              </TabsContent>

              <TabsContent value="auto" className="m-0">
                <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
                  <SugarHarvestPanel />
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
              </TabsContent>

              <TabsContent value="tree" className="m-0">
                <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
                  <div className="lg:col-span-2">
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
                  <div className="text-sm text-slate-400 space-y-4">
                    <h3 className="text-lg font-semibold text-slate-200">行为树说明</h3>
                    <div className="space-y-2">
                      <p>行为树控制铲糖自主循环流程</p>
                      <p>• <strong>序列节点 (→)</strong>: 顺序执行所有子节点</p>
                      <p>• <strong>选择节点 (?)</strong>: 依次执行直到成功</p>
                      <p>• <strong>重复节点 (↻)</strong>: 循环执行</p>
                      <p>• <strong>动作节点 (⚡)</strong>: 执行具体操作</p>
                    </div>
                    <div className="mt-4 p-3 bg-slate-800 rounded-lg">
                      <p className="text-xs text-slate-500 mb-2">循环终止条件</p>
                      <ul className="text-sm space-y-1">
                        <li>• 达到最大循环次数</li>
                        <li>• 糖堆高度 &lt; 20cm (切换推垛模式)</li>
                        <li>• 手动停止</li>
                      </ul>
                    </div>
                  </div>
                </div>
              </TabsContent>
            </div>
          </Tabs>
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
