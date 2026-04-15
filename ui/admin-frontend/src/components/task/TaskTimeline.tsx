import { useTaskStore } from '@/stores/useTaskStore';
import { getStatusColor, getStatusLabel, formatTime } from '@/lib/utils';

export function TaskTimeline() {
  const tasks = useTaskStore((s) => s.tasks);

  const sortedTasks = [...tasks]
    .filter((t) => t.started_at)
    .sort((a, b) => new Date(b.started_at!).getTime() - new Date(a.started_at!).getTime())
    .slice(0, 15);

  return (
    <div className="space-y-2">
      {sortedTasks.length === 0 ? (
        <div className="h-full flex items-center justify-center">
          <span className="text-xs text-text-muted">暂无执行记录</span>
        </div>
      ) : (
        sortedTasks.map((task) => (
          <div
            key={task.task_id}
            className="flex items-start gap-3 p-2 rounded-md hover:bg-white/3 transition-colors"
          >
            <div
              className="w-2 h-2 rounded-full mt-1.5 shrink-0"
              style={{
                background: getStatusColor(task.status),
                boxShadow: `0 0 6px ${getStatusColor(task.status)}60`,
              }}
            />
            <div className="flex-1 min-w-0">
              <div className="flex items-center gap-2">
                <span className="text-xs font-semibold text-text-primary truncate">{task.task_type}</span>
                <span
                  className="text-[9px] px-1.5 py-0.5 rounded font-bold"
                  style={{
                    color: getStatusColor(task.status),
                    background: `${getStatusColor(task.status)}15`,
                  }}
                >
                  {getStatusLabel(task.status)}
                </span>
              </div>
              <div className="text-[10px] text-text-muted font-mono mt-0.5">
                {task.started_at && formatTime(task.started_at)}
                {task.device_id && ` · ${task.device_id}`}
              </div>
            </div>
          </div>
        ))
      )}
    </div>
  );
}
