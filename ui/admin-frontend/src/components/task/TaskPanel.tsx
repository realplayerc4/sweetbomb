import { useState } from 'react';
import { useTaskStore } from '@/stores/useTaskStore';
import { useRobotStore } from '@/stores/useRobotStore';
import { EChartsWrapper } from '@/components/charts/EChartsWrapper';
import { Play, Pause, Square, Target, BarChart2, Clock3, Loader2, CheckCircle2, AlertTriangle, RefreshCw } from 'lucide-react';
import type { EChartsOption } from 'echarts';

// 任务阶段映射
const phaseLabels: Record<string, string> = {
  'idle': '空闲',
  'nav_to_pick': '取货中',
  'nav_to_drop': '卸货中',
  'paused': '已暂停',
  'completed': '任务完成',
  'error': '错误',
};

export function TaskPanel() {
  // 任务状态
  const targetKg = useTaskStore((s) => s.targetKg);
  const perCycleKg = useTaskStore((s) => s.perCycleKg);
  const totalCycles = useTaskStore((s) => s.totalCycles);
  const currentCycle = useTaskStore((s) => s.currentCycle);
  const phase = useTaskStore((s) => s.phase);
  const isRunning = useTaskStore((s) => s.isRunning);
  const completedKg = useTaskStore((s) => s.completedKg);
  const error = useTaskStore((s) => s.error);

  // Actions
  const setTarget = useTaskStore((s) => s.setTarget);
  const start = useTaskStore((s) => s.start);
  const pause = useTaskStore((s) => s.pause);
  const resume = useTaskStore((s) => s.resume);
  const stop = useTaskStore((s) => s.stop);
  const clearError = useTaskStore((s) => s.clearError);

  // 机器人连接状态
  const isConnected = useRobotStore((s) => s.isConnected);

  // 本地状态
  const [targetInput, setTargetInput] = useState(String(targetKg));

  // 计算进度百分比
  const percentage = totalCycles > 0 ? Math.round((currentCycle / totalCycles) * 100) : 0;
  const remainingKg = targetKg - completedKg;

  // 处理目标设置
  const handleSetTarget = () => {
    const kg = parseInt(targetInput) || 900;
    setTarget(kg);
  };

  // 仪表盘配置
  const getProgressOption = (): EChartsOption => ({
    backgroundColor: 'transparent',
    series: [
      {
        type: 'gauge',
        startAngle: 180,
        endAngle: 0,
        radius: '100%',
        pointer: { show: false },
        progress: {
          show: true,
          overlap: false,
          roundCap: true,
          clip: false,
          itemStyle: { color: '#00d4ff' },
        },
        axisLine: {
          lineStyle: {
            width: 12,
            color: [[1, 'rgba(0,212,255,0.1)']],
          },
        },
        splitLine: { show: false },
        axisTick: { show: false },
        axisLabel: { show: false },
        detail: {
          valueAnimation: true,
          formatter: '{value}%',
          color: '#00d4ff',
          fontSize: 24,
          fontWeight: 'bold',
          offsetCenter: [0, 0],
        },
        data: [{ value: percentage }],
      },
    ],
  });

  return (
    <div className="h-full flex flex-col">
      {/* 目标设置 */}
      <div className="flex items-center justify-between mb-3">
        <div className="flex items-center gap-2">
          <span className="text-[12px] text-text-muted">目标:</span>
          <input
            type="number"
            value={targetInput}
            onChange={(e) => setTargetInput(e.target.value)}
            className="w-20 h-6 bg-bg-primary/60 border border-cyan-500/20 rounded px-2 text-[12px] text-cyan-400 font-mono focus:outline-none focus:border-cyan-400"
            disabled={isRunning}
          />
          <span className="text-[12px] text-text-muted">kg</span>
          <button
            onClick={handleSetTarget}
            disabled={isRunning}
            className="px-2 py-1 rounded text-[11px] bg-cyan-500/15 text-cyan-400 border border-cyan-500/25 hover:bg-cyan-500/25 transition-colors disabled:opacity-50 disabled:cursor-not-allowed"
          >
            设置
          </button>
        </div>
        <div className="text-[11px] text-text-muted">
          每次循环 <span className="text-cyan-400 font-mono">{perCycleKg}kg</span>，共 <span className="text-cyan-400 font-mono">{totalCycles}</span> 次
        </div>
      </div>

      {/* 进度显示 */}
      <div className="flex-1 flex items-center mb-4">
        <div className="flex-1 flex flex-col items-center justify-center">
          <div style={{ width: 180, height: 100 }}>
            <EChartsWrapper option={getProgressOption()} />
          </div>
          <div className="mt-2 text-center">
            <div className="text-[12px] text-cyan-400 font-bold">
              {phaseLabels[phase] || phase}
            </div>
            {isRunning && (
              <div className="flex items-center gap-1 mt-1">
                <Loader2 className="w-3 h-3 text-cyan-400 animate-spin" />
                <span className="text-[10px] text-text-muted">执行中...</span>
              </div>
            )}
          </div>
        </div>

        <div className="flex-1 space-y-3">
          <div className="space-y-2">
            <div className="flex items-center gap-2">
              <Target className="w-4 h-4 text-tech-green shrink-0" />
              <div>
                <div className="text-[12px] text-text-muted">任务目标</div>
                <div className="text-[14px] font-bold text-tech-green font-mono">
                  {targetKg} kg
                </div>
              </div>
            </div>
            <div className="flex items-center gap-2">
              <BarChart2 className="w-4 h-4 text-cyan-400 shrink-0" />
              <div>
                <div className="text-[12px] text-text-muted">已完成</div>
                <div className="text-[14px] font-bold text-cyan-400 font-mono">
                  {completedKg} kg ({currentCycle}/{totalCycles} 次)
                </div>
              </div>
            </div>
          </div>

          <div className="space-y-2">
            <div className="flex items-center gap-2">
              <Target className="w-4 h-4 text-tech-orange shrink-0" />
              <div>
                <div className="text-[12px] text-text-muted">剩余目标</div>
                <div className="text-[14px] font-bold text-tech-orange font-mono">
                  {remainingKg} kg ({totalCycles - currentCycle} 次)
                </div>
              </div>
            </div>
            <div className="flex items-center gap-2">
              <Clock3 className="w-4 h-4 text-cyan-400 shrink-0" />
              <div>
                <div className="text-[12px] text-text-muted">完成进度</div>
                <div className="text-[14px] font-bold text-cyan-400 font-mono">
                  {percentage}%
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>

      {/* 进度条 */}
      <div className="mb-3">
        <div className="h-2 bg-bg-primary/60 rounded-full overflow-hidden">
          <div
            className="h-full bg-gradient-to-r from-cyan-500 to-tech-blue rounded-full transition-all duration-300"
            style={{ width: `${percentage}%` }}
          />
        </div>
      </div>

      {/* 错误显示 */}
      {error && (
        <div className="mb-3 p-2 rounded bg-tech-red/10 border border-tech-red/25 flex items-center gap-2">
          <AlertTriangle className="w-4 h-4 text-tech-red shrink-0" />
          <span className="text-[12px] text-tech-red">{error}</span>
          <button onClick={clearError} className="ml-auto text-[11px] text-tech-red/70 hover:text-tech-red">
            关闭
          </button>
        </div>
      )}

      {/* 控制按钮 */}
      <div className="flex items-center justify-between">
        <div className="flex items-center gap-2">
          {/* 启动/恢复按钮 */}
          {phase === 'paused' ? (
            <button
              onClick={resume}
              disabled={!isConnected}
              className="px-4 py-2 rounded-md bg-tech-green/15 text-tech-green border border-tech-green/25 font-bold text-[12px] hover:bg-tech-green/25 transition-colors disabled:opacity-50 disabled:cursor-not-allowed"
            >
              <Play className="w-3.5 h-3.5 inline mr-1" />
              恢复
            </button>
          ) : (
            <button
              onClick={start}
              disabled={!isConnected || isRunning || phase === 'completed'}
              className="px-4 py-2 rounded-md bg-tech-green/15 text-tech-green border border-tech-green/25 font-bold text-[12px] hover:bg-tech-green/25 transition-colors disabled:opacity-50 disabled:cursor-not-allowed"
            >
              <Play className="w-3.5 h-3.5 inline mr-1" />
              启动
            </button>
          )}

          {/* 暂停按钮 */}
          <button
            onClick={pause}
            disabled={!isRunning}
            className="px-4 py-2 rounded-md bg-tech-orange/15 text-tech-orange border border-tech-orange/25 font-bold text-[12px] hover:bg-tech-orange/25 transition-colors disabled:opacity-50 disabled:cursor-not-allowed"
          >
            <Pause className="w-3.5 h-3.5 inline mr-1" />
            暂停
          </button>

          {/* 终止按钮 */}
          <button
            onClick={stop}
            disabled={!isRunning && phase !== 'paused'}
            className="px-4 py-2 rounded-md bg-tech-red/15 text-tech-red border border-tech-red/25 font-bold text-[12px] hover:bg-tech-red/25 transition-colors disabled:opacity-50 disabled:cursor-not-allowed"
          >
            <Square className="w-3.5 h-3.5 inline mr-1" />
            终止
          </button>
        </div>

        {/* 连接状态 */}
        <div className="flex items-center gap-1.5">
          <div
            className={`w-2 h-2 rounded-full ${isConnected ? 'bg-tech-green' : 'bg-tech-red'}`}
            style={{ boxShadow: isConnected ? '0 0 6px #52c41a' : '0 0 6px #f5222d' }}
          />
          <span className={`text-[11px] ${isConnected ? 'text-tech-green' : 'text-tech-red'}`}>
            {isConnected ? '已连接' : '未连接'}
          </span>
        </div>
      </div>

      {/* 任务完成提示 */}
      {phase === 'completed' && (
        <div className="mt-3 p-2 rounded bg-tech-green/10 border border-tech-green/25 flex items-center gap-2">
          <CheckCircle2 className="w-4 h-4 text-tech-green shrink-0" />
          <span className="text-[12px] text-tech-green">
            任务完成！已装载 {completedKg} kg
          </span>
          <button
            onClick={() => setTarget(targetKg)}
            className="ml-auto px-2 py-1 rounded text-[11px] bg-tech-green/20 text-tech-green hover:bg-tech-green/30 transition-colors"
          >
            <RefreshCw className="w-3 h-3 inline mr-1" />
            重新开始
          </button>
        </div>
      )}
    </div>
  );
}
