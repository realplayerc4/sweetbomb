import { useState } from 'react';
import { useTaskStore } from '@/stores/useTaskStore';
import { useSystemModeStore } from '@/stores/useSystemModeStore';
import { useSystemStore } from '@/stores/useSystemStore';
import { useRobotStore } from '@/stores/useRobotStore';
import { TaskCreateModal } from '@/components/modals/TaskCreateModal';
import { TaskDetailModal } from '@/components/modals/TaskDetailModal';
import { EChartsWrapper } from '@/components/charts/EChartsWrapper';
import { getStatusColor, getStatusLabel, cn, formatDuration } from '@/lib/utils';
import { User, Users, Clock, Zap, Play, Pause, Square, Send, CheckCircle2, AlertTriangle, Loader2, ListChecks, Target, BarChart2, Clock3 } from 'lucide-react';
import type { TaskInfo } from '@/types';
import type { EChartsOption } from 'echarts';

type CommandStep = 'select' | 'confirm' | 'dispatching' | 'result';

export function TaskPanel() {
  const tasks = useTaskStore((s) => s.tasks);
  const runningCount = useTaskStore((s) => s.runningCount);
  const completedToday = useTaskStore((s) => s.completedToday);
  const pendingCount = useTaskStore((s) => s.pendingCount);
  const startTask = useTaskStore((s) => s.startTask);
  const pauseTask = useTaskStore((s) => s.pauseTask);
  const resumeTask = useTaskStore((s) => s.resumeTask);
  const stopTask = useTaskStore((s) => s.stopTask);
  const monitorMode = useSystemModeStore((s) => s.monitorMode);
  const selectedRobot = useSystemModeStore((s) => s.selectedRobot);
  const setMonitorMode = useSystemModeStore((s) => s.setMonitorMode);
  const devices = useSystemStore((s) => s.devices);
  const isHarvestRunning = useRobotStore((s) => s.isHarvestRunning);
  const harvestCycle = useRobotStore((s) => s.harvestCycle);
  const harvestMaxCycles = useRobotStore((s) => s.harvestMaxCycles);

  const [selectedTask, setSelectedTask] = useState<TaskInfo | null>(null);
  const [detailOpen, setDetailOpen] = useState(false);
  const [commandStep, setCommandStep] = useState<CommandStep>('select');
  const [selectedTaskId, setSelectedTaskId] = useState<string | null>(null);
  const [dispatchResult, setDispatchResult] = useState<'success' | 'error' | null>(null);
  const [activeTaskId, setActiveTaskId] = useState<string | null>(null);

  const filteredTasks = monitorMode === 'single'
    ? tasks.filter((t) => t.device_id === selectedRobot)
    : tasks;

  const pendingTasks = tasks.filter((t) => t.status === 'pending');
  const runningTasks = tasks.filter((t) => t.status === 'running');
  const pausedTasks = tasks.filter((t) => t.status === 'paused');

  const activeTask = activeTaskId ? tasks.find((t) => t.task_id === activeTaskId) : (runningTasks[0] || pendingTasks[0] || tasks[0] || null);

  const handleTaskClick = (task: TaskInfo) => {
    setSelectedTask(task);
    setDetailOpen(true);
  };

  const handleTaskTabClick = (taskId: string) => {
    setActiveTaskId(taskId);
  };

  const handleQuickAction = async (e: React.MouseEvent, task: TaskInfo, action: 'start' | 'pause' | 'resume' | 'stop') => {
    e.stopPropagation();
    try {
      if (action === 'start') await startTask(task.task_id);
      else if (action === 'pause') await pauseTask(task.task_id);
      else if (action === 'resume') await resumeTask(task.task_id);
      else if (action === 'stop') await stopTask(task.task_id);
    } catch {
      // task action failed silently
    }
  };

  const handleQuickDispatch = (taskId: string) => {
    setSelectedTaskId(taskId);
    setCommandStep('confirm');
  };

  const handleConfirmDispatch = async () => {
    if (!selectedTaskId) return;
    setCommandStep('dispatching');
    try {
      await startTask(selectedTaskId);
      setDispatchResult('success');
      setCommandStep('result');
    } catch {
      setDispatchResult('error');
      setCommandStep('result');
    }
  };

  const handleResetCommand = () => {
    setCommandStep('select');
    setSelectedTaskId(null);
    setDispatchResult(null);
  };

  const handleControlAction = async (action: 'start' | 'pause' | 'stop') => {
    if (!activeTask) return;
    try {
      if (action === 'start' && activeTask.status === 'pending') await startTask(activeTask.task_id);
      else if (action === 'pause' && activeTask.status === 'running') await pauseTask(activeTask.task_id);
      else if (action === 'stop') await stopTask(activeTask.task_id);
    } catch {
      // action failed silently
    }
  };

  const renderQuickActions = (task: TaskInfo) => {
    switch (task.status) {
      case 'pending':
        return (
          <div className="flex items-center gap-1">
            <button onClick={(e) => handleQuickAction(e, task, 'start')} className="p-1 rounded hover:bg-white/10 text-tech-green transition-colors" title="启动">
              <Play className="w-3.5 h-3.5" />
            </button>
            <button onClick={() => handleQuickDispatch(task.task_id)} className="p-1 rounded hover:bg-white/10 text-cyan-400 transition-colors" title="快速下发">
              <Send className="w-3.5 h-3.5" />
            </button>
          </div>
        );
      case 'running':
        return (
          <div className="flex items-center gap-0.5">
            <button onClick={(e) => handleQuickAction(e, task, 'pause')} className="p-1 rounded hover:bg-white/10 text-tech-orange transition-colors" title="暂停">
              <Pause className="w-3.5 h-3.5" />
            </button>
            <button onClick={(e) => handleQuickAction(e, task, 'stop')} className="p-1 rounded hover:bg-white/10 text-tech-red transition-colors" title="终止">
              <Square className="w-3.5 h-3.5" />
            </button>
          </div>
        );
      case 'paused':
        return (
          <div className="flex items-center gap-0.5">
            <button onClick={(e) => handleQuickAction(e, task, 'resume')} className="p-1 rounded hover:bg-white/10 text-tech-green transition-colors" title="恢复">
              <Play className="w-3.5 h-3.5" />
            </button>
            <button onClick={(e) => handleQuickAction(e, task, 'stop')} className="p-1 rounded hover:bg-white/10 text-tech-red transition-colors" title="终止">
              <Square className="w-3.5 h-3.5" />
            </button>
          </div>
        );
      default:
        return null;
    }
  };

  const getProgressOption = (task: TaskInfo | null): EChartsOption => {
    const percentage = task?.progress?.percentage || 0;
    return {
      backgroundColor: 'transparent',
      series: [
        {
          type: 'gauge',
          startAngle: 180,
          endAngle: 0,
          radius: '100%',
          pointer: {
            show: false,
          },
          progress: {
            show: true,
            overlap: false,
            roundCap: true,
            clip: false,
            itemStyle: {
              color: '#00d4ff',
            },
          },
          axisLine: {
            lineStyle: {
              width: 12,
              color: [[1, 'rgba(0,212,255,0.1)']],
            },
          },
          splitLine: {
            show: false,
          },
          axisTick: {
            show: false,
          },
          axisLabel: {
            show: false,
          },
          detail: {
            valueAnimation: true,
            formatter: '{value}%',
            color: '#00d4ff',
            fontSize: 24,
            fontWeight: 'bold',
            offsetCenter: [0, 0],
          },
          data: [
            {
              value: percentage,
            },
          ],
        },
      ],
    };
  };

  return (
    <div className="h-full flex flex-col">
      <div className="flex items-center justify-between mb-3">
        <div className="flex items-center gap-2">
          <div className="flex items-center gap-3 text-[13px]">
            <span className="flex items-center gap-1 text-cyan-400">
              <Zap className="w-3.5 h-3.5" />
              <span className="font-mono font-bold">{runningCount}</span> 运行
            </span>
            <span className="flex items-center gap-1 text-tech-orange">
              <Clock className="w-3.5 h-3.5" />
              <span className="font-mono font-bold">{pendingCount}</span> 待执行
            </span>
            <span className="flex items-center gap-1 text-tech-green">
              <span className="font-mono font-bold">{completedToday}</span> 完成
            </span>
          </div>
        </div>
        <div className="flex items-center gap-3">
          <div className="flex items-center gap-2">
            <button
              onClick={() => handleControlAction('start')}
              disabled={!activeTask || activeTask.status !== 'pending'}
              className="px-3 py-1.5 rounded-md bg-tech-green/15 text-tech-green border border-tech-green/25 font-bold text-[12px] hover:bg-tech-green/25 transition-colors disabled:opacity-50 disabled:cursor-not-allowed"
            >
              <Play className="w-3.5 h-3.5 inline mr-1" />
              启动
            </button>
            <button
              onClick={() => handleControlAction('pause')}
              disabled={!activeTask || activeTask.status !== 'running'}
              className="px-3 py-1.5 rounded-md bg-tech-orange/15 text-tech-orange border border-tech-orange/25 font-bold text-[12px] hover:bg-tech-orange/25 transition-colors disabled:opacity-50 disabled:cursor-not-allowed"
            >
              <Pause className="w-3.5 h-3.5 inline mr-1" />
              暂停
            </button>
            <button
              onClick={() => handleControlAction('stop')}
              disabled={!activeTask || (activeTask.status !== 'running' && activeTask.status !== 'paused')}
              className="px-3 py-1.5 rounded-md bg-tech-red/15 text-tech-red border border-tech-red/25 font-bold text-[12px] hover:bg-tech-red/25 transition-colors disabled:opacity-50 disabled:cursor-not-allowed"
            >
              <Square className="w-3.5 h-3.5 inline mr-1" />
              终止
            </button>
          </div>
          <TaskCreateModal />
        </div>
      </div>

      {commandStep === 'select' ? (
        <div className="flex-1 flex flex-col">
          {activeTask ? (
            <div className="flex-1 flex flex-col">
              <div className="flex-1 flex items-center mb-4">
                <div className="flex-1 flex flex-col items-center justify-center">
                  <div style={{ width: 180, height: 100 }}>
                    <EChartsWrapper option={getProgressOption(activeTask)} />
                  </div>
                  <div className="mt-2 text-center">
                    <div className="text-[12px] text-text-muted">{activeTask.task_type}</div>
                    <div className="text-[11px] text-text-muted font-mono">{activeTask.task_id.slice(0, 8)}</div>
                  </div>
                </div>
                <div className="flex-1 space-y-3">
                  <div className="space-y-2">
                    <div className="flex items-center gap-2">
                      <Target className="w-4 h-4 text-tech-green shrink-0" />
                      <div>
                        <div className="text-[12px] text-text-muted">任务目标</div>
                        <div className="text-[14px] font-bold text-tech-green font-mono">
                          {activeTask.target_ton || 0} 吨
                        </div>
                      </div>
                    </div>
                    <div className="flex items-center gap-2">
                      <BarChart2 className="w-4 h-4 text-cyan-400 shrink-0" />
                      <div>
                        <div className="text-[12px] text-text-muted">已达成</div>
                        <div className="text-[14px] font-bold text-cyan-400 font-mono">
                          {((activeTask.target_ton || 0) * (activeTask.progress?.percentage || 0) / 100).toFixed(1)} 吨 ({(activeTask.progress?.percentage || 0).toFixed(0)}%)
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
                          {((activeTask.target_ton || 0) * (100 - (activeTask.progress?.percentage || 0)) / 100).toFixed(1)} 吨 ({(100 - (activeTask.progress?.percentage || 0)).toFixed(0)}%)
                        </div>
                      </div>
                    </div>
                    <div className="flex items-center gap-2">
                      <Clock3 className="w-4 h-4 text-cyan-400 shrink-0" />
                      <div>
                        <div className="text-[12px] text-text-muted">已用时间</div>
                        <div className="text-[14px] font-bold text-cyan-400 font-mono">
                          {formatDuration(activeTask.progress?.elapsed_seconds || 0)}
                        </div>
                      </div>
                    </div>
                  </div>
                </div>
              </div>
              <div className="mb-2">
                <div className="text-[12px] font-bold text-text-muted mb-2">任务标签</div>
                <div className="overflow-x-auto pb-2">
                  <div className="flex gap-2 min-w-max">
                    {filteredTasks.map((task) => (
                      <button
                        key={task.task_id}
                        onClick={() => handleTaskTabClick(task.task_id)}
                        className={cn(
                          'px-3 py-1.5 rounded-md border font-bold text-[12px] transition-colors whitespace-nowrap',
                          activeTaskId === task.task_id || (!activeTaskId && task.task_id === activeTask?.task_id)
                            ? 'bg-cyan-500/20 text-cyan-400 border-cyan-500/30'
                            : 'bg-bg-primary/60 text-text-muted border-cyan-500/15 hover:bg-bg-primary/80 hover:text-text-secondary'
                        )}
                      >
                        <span className="inline-block w-2 h-2 rounded-full mr-2" style={{ background: getStatusColor(task.status) }} />
                        {task.task_type}
                        <span className="ml-2 text-[10px] font-mono text-text-muted">{task.task_id.slice(0, 6)}</span>
                      </button>
                    ))}
                  </div>
                </div>
              </div>
              {/* <div className="space-y-2">
                <div className="text-[12px] font-bold text-text-muted">任务列表</div>
                <div className="max-h-[120px] overflow-y-auto space-y-2">
                  {filteredTasks.slice(0, 4).map((task) => (
                    <div
                      key={task.task_id}
                      onClick={() => handleTaskClick(task)}
                      className={cn(
                        'tech-card p-3 cursor-pointer transition-all group',
                        activeTaskId === task.task_id || (!activeTaskId && task.task_id === activeTask?.task_id)
                          ? 'border-cyan-500/30 bg-cyan-500/5'
                          : 'hover:border-cyan-500/30'
                      )}
                    >
                      <div className="flex items-center justify-between mb-1.5">
                        <div className="flex items-center gap-2">
                          <div
                            className="status-dot shrink-0"
                            style={{
                              background: getStatusColor(task.status),
                              boxShadow: `0 0 6px ${getStatusColor(task.status)}60`,
                            }}
                          />
                          <span className="text-[14px] font-bold" style={{ color: getStatusColor(task.status) }}>
                            {getStatusLabel(task.status)}
                          </span>
                        </div>
                        <div className="flex items-center gap-2">
                          {renderQuickActions(task)}
                          <span className="text-[12px] text-text-muted font-mono">{task.task_id.slice(0, 8)}</span>
                        </div>
                      </div>
                      <div className="flex items-center justify-between">
                        <span className="text-[14px] font-semibold text-text-primary">{task.task_type}</span>
                        {task.device_id && (
                          <span className="text-[12px] text-text-muted">{task.device_id}</span>
                        )}
                      </div>
                      {task.progress && task.progress.percentage > 0 && (
                        <div className="mt-2">
                          <div className="flex justify-between text-[12px] mb-1">
                            <span className="text-text-muted">{task.progress.message}</span>
                            <span className="text-cyan-400 font-mono">{task.progress.percentage.toFixed(0)}%</span>
                          </div>
                          <div className="h-1.5 bg-bg-primary/60 rounded-full overflow-hidden">
                            <div
                              className="h-full bg-gradient-to-r from-cyan-500 to-tech-blue rounded-full transition-all duration-300"
                              style={{ width: `${task.progress.percentage}%` }}
                            />
                          </div>
                        </div>
                      )}
                    </div>
                  ))}
                </div>
              </div> */}
            </div>
          ) : (
            <div className="flex-1 flex items-center justify-center">
              <span className="text-[14px] text-text-muted">暂无任务</span>
            </div>
          )}
        </div>
      ) : commandStep === 'confirm' && selectedTaskId ? (
        <div className="flex-1 flex flex-col items-center justify-center text-center space-y-3">
          <Send className="w-8 h-8 text-cyan-400" />
          <div>
            <div className="text-[14px] font-bold text-text-primary mb-1">确认下发任务</div>
            <div className="text-[12px] text-text-muted font-mono">{selectedTaskId.slice(0, 8)}</div>
          </div>
          <div className="flex gap-2">
            <button onClick={handleConfirmDispatch} className="px-4 py-2 rounded-md bg-cyan-500/15 text-cyan-400 border border-cyan-500/25 font-bold text-[12px] hover:bg-cyan-500/25 transition-colors">
              确认下发
            </button>
            <button onClick={handleResetCommand} className="px-4 py-2 rounded-md bg-bg-primary/60 text-text-muted border border-cyan-500/15 font-bold text-[12px] hover:text-text-secondary transition-colors">
              取消
            </button>
          </div>
        </div>
      ) : commandStep === 'dispatching' ? (
        <div className="flex-1 flex flex-col items-center justify-center text-center space-y-3">
          <Loader2 className="w-8 h-8 text-cyan-400 animate-spin" />
          <div className="text-[14px] font-bold text-text-primary">正在下发指令...</div>
        </div>
      ) : commandStep === 'result' ? (
        <div className="flex-1 flex flex-col items-center justify-center text-center space-y-3">
          {dispatchResult === 'success' ? (
            <>
              <CheckCircle2 className="w-8 h-8 text-tech-green" />
              <div className="text-[14px] font-bold text-tech-green">指令下发成功</div>
            </>
          ) : (
            <>
              <AlertTriangle className="w-8 h-8 text-tech-red" />
              <div className="text-[14px] font-bold text-tech-red">指令下发失败</div>
            </>
          )}
          <button onClick={handleResetCommand} className="px-4 py-2 rounded-md bg-cyan-500/15 text-cyan-400 border border-cyan-500/25 font-bold text-[12px] hover:bg-cyan-500/25 transition-colors">
            返回
          </button>
        </div>
      ) : null}

      <TaskDetailModal
        task={selectedTask}
        open={detailOpen}
        onClose={() => { setDetailOpen(false); setSelectedTask(null); }}
      />
    </div>
  );
}
