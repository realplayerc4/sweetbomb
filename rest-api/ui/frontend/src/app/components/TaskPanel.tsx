/**
 * Task management panel component.
 */

import { useState } from 'react';
import {
  Play,
  Pause,
  Square,
  Trash2,
  Plus,
  RefreshCw,
  CheckCircle,
  XCircle,
  Clock,
  Loader2,
  AlertCircle,
} from 'lucide-react';
import { Button } from './ui/button';
import { Card, CardContent, CardHeader, CardTitle } from './ui/card';
import { useTaskManager } from '../hooks/useTaskManager';
import type { TaskInfo, TaskStatus } from '../services/taskApi';

interface TaskPanelProps {
  deviceId?: string | null;
}

// Status badge colors
const statusColors: Record<TaskStatus, string> = {
  pending: 'bg-yellow-500/20 text-yellow-400 border-yellow-500/50',
  running: 'bg-blue-500/20 text-blue-400 border-blue-500/50',
  paused: 'bg-orange-500/20 text-orange-400 border-orange-500/50',
  completed: 'bg-green-500/20 text-green-400 border-green-500/50',
  failed: 'bg-red-500/20 text-red-400 border-red-500/50',
  stopped: 'bg-gray-500/20 text-gray-400 border-gray-500/50',
  cancelled: 'bg-gray-500/20 text-gray-400 border-gray-500/50',
};

const statusLabels: Record<TaskStatus, string> = {
  pending: '等待中',
  running: '运行中',
  paused: '已暂停',
  completed: '已完成',
  failed: '失败',
  stopped: '已停止',
  cancelled: '已取消',
};

const statusIcons: Record<TaskStatus, React.ReactNode> = {
  pending: <Clock className="w-3 h-3" />,
  running: <Loader2 className="w-3 h-3 animate-spin" />,
  paused: <Pause className="w-3 h-3" />,
  completed: <CheckCircle className="w-3 h-3" />,
  failed: <XCircle className="w-3 h-3" />,
  stopped: <Square className="w-3 h-3" />,
  cancelled: <AlertCircle className="w-3 h-3" />,
};

export function TaskPanel({ deviceId }: TaskPanelProps) {
  const {
    taskTypes,
    isLoadingTypes,
    tasks,
    isLoadingTasks,
    runningCount,
    pendingCount,
    canStartMore,
    isConnected,
    refreshTasks,
    createTask,
    startTask,
    pauseTask,
    resumeTask,
    stopTask,
    deleteTask,
    error,
    clearError,
  } = useTaskManager({ deviceId });

  const [selectedTaskType, setSelectedTaskType] = useState<string>('');
  const [isCreating, setIsCreating] = useState(false);

  // Filter tasks for current device
  const deviceTasks = deviceId
    ? tasks.filter((t) => t.device_id === deviceId)
    : tasks;

  // Group tasks by status
  const activeTasks = deviceTasks.filter(
    (t) => t.status === 'running' || t.status === 'paused'
  );
  const pendingTasks = deviceTasks.filter((t) => t.status === 'pending');
  const completedTasks = deviceTasks.filter(
    (t) => t.status === 'completed' || t.status === 'failed' || t.status === 'stopped'
  );

  // Handle create task
  const handleCreateTask = async () => {
    if (!selectedTaskType) return;

    setIsCreating(true);
    try {
      const task = await createTask({
        task_type: selectedTaskType,
        device_id: deviceId,
      });

      // Auto-start if can
      if (canStartMore) {
        await startTask(task.task_id);
      }
    } catch (e) {
      console.error('Failed to create task:', e);
    } finally {
      setIsCreating(false);
      setSelectedTaskType('');
    }
  };

  // Handle task actions
  const handleStart = async (taskId: string) => {
    try {
      await startTask(taskId);
    } catch (e) {
      console.error('Failed to start task:', e);
    }
  };

  const handlePause = async (taskId: string) => {
    try {
      await pauseTask(taskId);
    } catch (e) {
      console.error('Failed to pause task:', e);
    }
  };

  const handleResume = async (taskId: string) => {
    try {
      await resumeTask(taskId);
    } catch (e) {
      console.error('Failed to resume task:', e);
    }
  };

  const handleStop = async (taskId: string) => {
    try {
      await stopTask(taskId);
    } catch (e) {
      console.error('Failed to stop task:', e);
    }
  };

  const handleDelete = async (taskId: string) => {
    try {
      await deleteTask(taskId);
    } catch (e) {
      console.error('Failed to delete task:', e);
    }
  };

  return (
    <Card className="bg-slate-900/50 border-slate-800 backdrop-blur">
      <CardHeader>
        <div className="flex items-center justify-between">
          <CardTitle className="text-lg flex items-center gap-2">
            任务管理
            {isConnected ? (
              <span className="w-2 h-2 bg-green-500 rounded-full" title="已连接" />
            ) : (
              <span className="w-2 h-2 bg-red-500 rounded-full" title="未连接" />
            )}
          </CardTitle>
          <div className="flex items-center gap-2 text-sm text-slate-400">
            <span>运行中: {runningCount}/4</span>
            <span>等待: {pendingCount}</span>
            <Button
              variant="ghost"
              size="sm"
              onClick={() => refreshTasks()}
              disabled={isLoadingTasks}
            >
              <RefreshCw className={`w-4 h-4 ${isLoadingTasks ? 'animate-spin' : ''}`} />
            </Button>
          </div>
        </div>
      </CardHeader>
      <CardContent className="space-y-4">
        {/* Error display */}
        {error && (
          <div className="flex items-center justify-between p-3 bg-red-500/10 border border-red-500/50 rounded-lg text-red-400 text-sm">
            <span>{error}</span>
            <Button variant="ghost" size="sm" onClick={clearError}>
              <XCircle className="w-4 h-4" />
            </Button>
          </div>
        )}

        {/* Create Task */}
        <div className="flex items-center gap-2">
          <select
            value={selectedTaskType}
            onChange={(e) => setSelectedTaskType(e.target.value)}
            className="flex-1 px-3 py-2 bg-slate-950 border border-slate-700 rounded-md text-sm focus:outline-none focus:ring-2 focus:ring-blue-500"
            disabled={isLoadingTypes}
          >
            <option value="">选择任务类型...</option>
            {taskTypes.map((type) => (
              <option key={type.task_type} value={type.task_type}>
                {type.name} - {type.description}
              </option>
            ))}
          </select>
          <Button
            onClick={handleCreateTask}
            disabled={!selectedTaskType || isCreating}
            size="sm"
          >
            {isCreating ? (
              <Loader2 className="w-4 h-4 animate-spin" />
            ) : (
              <Plus className="w-4 h-4" />
            )}
            创建
          </Button>
        </div>

        {/* Active Tasks */}
        {activeTasks.length > 0 && (
          <div className="space-y-2">
            <h4 className="text-sm font-medium text-slate-300">活动任务</h4>
            {activeTasks.map((task) => (
              <TaskItem
                key={task.task_id}
                task={task}
                onPause={handlePause}
                onResume={handleResume}
                onStop={handleStop}
                canStartMore={canStartMore}
              />
            ))}
          </div>
        )}

        {/* Pending Tasks */}
        {pendingTasks.length > 0 && (
          <div className="space-y-2">
            <h4 className="text-sm font-medium text-slate-300">等待中</h4>
            {pendingTasks.map((task) => (
              <TaskItem
                key={task.task_id}
                task={task}
                onStart={handleStart}
                onDelete={handleDelete}
                canStartMore={canStartMore}
              />
            ))}
          </div>
        )}

        {/* Completed Tasks */}
        {completedTasks.length > 0 && (
          <div className="space-y-2">
            <h4 className="text-sm font-medium text-slate-300">已完成</h4>
            <div className="max-h-40 overflow-y-auto space-y-1">
              {completedTasks.slice(0, 10).map((task) => (
                <TaskItem
                  key={task.task_id}
                  task={task}
                  onDelete={handleDelete}
                  compact
                />
              ))}
            </div>
          </div>
        )}

        {/* Empty state */}
        {deviceTasks.length === 0 && !isLoadingTasks && (
          <div className="text-center py-8 text-slate-500">
            <p>暂无任务</p>
            <p className="text-xs mt-1">选择一个任务类型来创建新任务</p>
          </div>
        )}

        {/* Loading state */}
        {isLoadingTasks && deviceTasks.length === 0 && (
          <div className="flex items-center justify-center py-8">
            <Loader2 className="w-6 h-6 animate-spin text-blue-500" />
          </div>
        )}
      </CardContent>
    </Card>
  );
}

// Task item component
interface TaskItemProps {
  task: TaskInfo;
  onStart?: (taskId: string) => void;
  onPause?: (taskId: string) => void;
  onResume?: (taskId: string) => void;
  onStop?: (taskId: string) => void;
  onDelete?: (taskId: string) => void;
  canStartMore?: boolean;
  compact?: boolean;
}

function TaskItem({
  task,
  onStart,
  onPause,
  onResume,
  onStop,
  onDelete,
  canStartMore = true,
  compact = false,
}: TaskItemProps) {
  const status = task.status;

  if (compact) {
    return (
      <div className="flex items-center justify-between p-2 bg-slate-950/50 rounded border border-slate-800 text-xs">
        <div className="flex items-center gap-2">
          <span className={statusColors[status]}>{statusIcons[status]}</span>
          <span className="text-slate-300">{task.task_type}</span>
        </div>
        <div className="flex items-center gap-1">
          <span className={`px-1.5 py-0.5 rounded border text-xs ${statusColors[status]}`}>
            {statusLabels[status]}
          </span>
          {onDelete && (
            <Button
              variant="ghost"
              size="icon"
              className="h-6 w-6"
              onClick={() => onDelete(task.task_id)}
            >
              <Trash2 className="w-3 h-3" />
            </Button>
          )}
        </div>
      </div>
    );
  }

  return (
    <div className="p-3 bg-slate-950/50 rounded-lg border border-slate-800 space-y-2">
      {/* Header */}
      <div className="flex items-center justify-between">
        <div className="flex items-center gap-2">
          <span className={statusColors[status]}>{statusIcons[status]}</span>
          <span className="font-medium text-sm">{task.task_type}</span>
        </div>
        <span className={`px-2 py-1 rounded border text-xs ${statusColors[status]}`}>
          {statusLabels[status]}
        </span>
      </div>

      {/* Progress */}
      {(status === 'running' || status === 'paused') && (
        <div className="space-y-1">
          <div className="flex items-center justify-between text-xs text-slate-400">
            <span>{task.progress.message || '处理中...'}</span>
            <span>{task.progress.percentage.toFixed(1)}%</span>
          </div>
          <div className="h-1.5 bg-slate-800 rounded-full overflow-hidden">
            <div
              className="h-full bg-blue-500 transition-all duration-300"
              style={{ width: `${task.progress.percentage}%` }}
            />
          </div>
          {task.progress.estimated_remaining_seconds && (
            <div className="text-xs text-slate-500">
              预计剩余: {formatDuration(task.progress.estimated_remaining_seconds)}
            </div>
          )}
        </div>
      )}

      {/* Result */}
      {task.result && status === 'completed' && (
        <div className="text-xs text-slate-400">
          {task.result.message}
          {task.result.metrics && (
            <span className="ml-2 text-slate-500">
              ({Object.entries(task.result.metrics).map(([k, v]) => `${k}: ${v}`).join(', ')})
            </span>
          )}
        </div>
      )}

      {/* Error */}
      {task.result?.error && status === 'failed' && (
        <div className="text-xs text-red-400 bg-red-500/10 p-2 rounded">
          错误: {task.result.error}
        </div>
      )}

      {/* Actions */}
      <div className="flex items-center gap-2 pt-1">
        {status === 'pending' && onStart && (
          <Button
            size="sm"
            variant="default"
            onClick={() => onStart(task.task_id)}
            disabled={!canStartMore}
          >
            <Play className="w-3 h-3 mr-1" />
            启动
          </Button>
        )}

        {status === 'running' && onPause && (
          <Button size="sm" variant="secondary" onClick={() => onPause(task.task_id)}>
            <Pause className="w-3 h-3 mr-1" />
            暂停
          </Button>
        )}

        {status === 'paused' && onResume && (
          <Button size="sm" variant="default" onClick={() => onResume(task.task_id)}>
            <Play className="w-3 h-3 mr-1" />
            继续
          </Button>
        )}

        {(status === 'running' || status === 'paused') && onStop && (
          <Button size="sm" variant="destructive" onClick={() => onStop(task.task_id)}>
            <Square className="w-3 h-3 mr-1" />
            停止
          </Button>
        )}

        {(status === 'completed' || status === 'failed' || status === 'stopped') && onDelete && (
          <Button size="sm" variant="ghost" onClick={() => onDelete(task.task_id)}>
            <Trash2 className="w-3 h-3 mr-1" />
            删除
          </Button>
        )}
      </div>
    </div>
  );
}

// Format duration helper
function formatDuration(seconds: number): string {
  if (seconds < 60) {
    return `${Math.round(seconds)}秒`;
  }
  const minutes = Math.floor(seconds / 60);
  const secs = Math.round(seconds % 60);
  return `${minutes}分${secs}秒`;
}
