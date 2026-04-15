import { useTaskStore } from '@/stores/useTaskStore';
import { getStatusColor, formatDateTime, cn } from '@/lib/utils';
import { Play, Pause, Square, Trash2, RotateCcw } from 'lucide-react';
import type { TaskInfo, TaskStatus } from '@/types';

const statusColumns: TaskStatus[] = ['pending', 'running', 'paused', 'completed', 'failed'];

const columnLabels: Record<TaskStatus, string> = {
  pending: '待执行',
  running: '运行中',
  paused: '已暂停',
  completed: '已完成',
  failed: '失败',
  stopped: '已停止',
  cancelled: '已取消',
};

export function TaskBoard() {
  const tasks = useTaskStore((s) => s.tasks);
  const startTask = useTaskStore((s) => s.startTask);
  const pauseTask = useTaskStore((s) => s.pauseTask);
  const resumeTask = useTaskStore((s) => s.resumeTask);
  const stopTask = useTaskStore((s) => s.stopTask);
  const deleteTask = useTaskStore((s) => s.deleteTask);

  const getTasksByStatus = (status: TaskStatus) => tasks.filter((t) => t.status === status);

  const renderActions = (task: TaskInfo) => {
    const btnClass = 'p-1 rounded hover:bg-white/10 transition-colors';
    switch (task.status) {
      case 'pending':
        return (
          <button className={cn(btnClass, 'text-tech-green')} onClick={() => startTask(task.task_id)}>
            <Play className="w-3.5 h-3.5" />
          </button>
        );
      case 'running':
        return (
          <>
            <button className={cn(btnClass, 'text-tech-orange')} onClick={() => pauseTask(task.task_id)}>
              <Pause className="w-3.5 h-3.5" />
            </button>
            <button className={cn(btnClass, 'text-tech-red')} onClick={() => stopTask(task.task_id)}>
              <Square className="w-3.5 h-3.5" />
            </button>
          </>
        );
      case 'paused':
        return (
          <>
            <button className={cn(btnClass, 'text-tech-green')} onClick={() => resumeTask(task.task_id)}>
              <RotateCcw className="w-3.5 h-3.5" />
            </button>
            <button className={cn(btnClass, 'text-tech-red')} onClick={() => stopTask(task.task_id)}>
              <Square className="w-3.5 h-3.5" />
            </button>
          </>
        );
      case 'completed':
      case 'failed':
      case 'stopped':
      case 'cancelled':
        return (
          <button className={cn(btnClass, 'text-text-muted')} onClick={() => deleteTask(task.task_id)}>
            <Trash2 className="w-3.5 h-3.5" />
          </button>
        );
      default:
        return null;
    }
  };

  return (
    <div className="h-full grid grid-cols-5 gap-3">
      {statusColumns.map((status) => {
        const columnTasks = getTasksByStatus(status);
        return (
          <div key={status} className="flex flex-col min-h-0">
            <div className="flex items-center gap-2 mb-3 px-1">
              <div
                className="status-dot shrink-0"
                style={{
                  background: getStatusColor(status),
                  boxShadow: `0 0 6px ${getStatusColor(status)}60`,
                }}
              />
              <span className="text-xs font-bold tracking-wider" style={{ color: getStatusColor(status) }}>
                {columnLabels[status]}
              </span>
              <span className="text-[10px] text-text-muted font-mono ml-auto">{columnTasks.length}</span>
            </div>

            <div className="flex-1 overflow-y-auto space-y-2">
              {columnTasks.map((task) => (
                <div
                  key={task.task_id}
                  className="tech-card p-3 group"
                >
                  <div className="flex items-center justify-between mb-2">
                    <span className="text-[10px] text-text-muted font-mono truncate">{task.task_id.slice(0, 8)}</span>
                    <div className="flex items-center gap-1 opacity-0 group-hover:opacity-100 transition-opacity">
                      {renderActions(task)}
                    </div>
                  </div>
                  <div className="text-xs font-semibold text-text-primary mb-1 truncate">
                    {task.task_type}
                  </div>
                  {task.device_id && (
                    <div className="text-[10px] text-text-muted mb-2">
                      设备: {task.device_id}
                    </div>
                  )}
                  {task.progress && task.progress.percentage > 0 && (
                    <div className="space-y-1">
                      <div className="flex justify-between text-[9px]">
                        <span className="text-text-muted">{task.progress.message}</span>
                        <span className="text-tech-blue font-mono">{task.progress.percentage.toFixed(0)}%</span>
                      </div>
                      <div className="h-1 bg-bg-primary/60 rounded-full overflow-hidden">
                        <div
                          className="h-full bg-tech-blue rounded-full transition-all duration-300"
                          style={{ width: `${task.progress.percentage}%` }}
                        />
                      </div>
                    </div>
                  )}
                  <div className="text-[9px] text-text-muted font-mono mt-2">
                    {formatDateTime(task.created_at)}
                  </div>
                </div>
              ))}
              {columnTasks.length === 0 && (
                <div className="h-20 flex items-center justify-center border border-dashed border-border-tech/20 rounded-lg">
                  <span className="text-[10px] text-text-muted">暂无任务</span>
                </div>
              )}
            </div>
          </div>
        );
      })}
    </div>
  );
}
