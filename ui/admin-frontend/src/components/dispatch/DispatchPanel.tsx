import { useState } from 'react';
import { useSystemStore } from '@/stores/useSystemStore';
import { useTaskStore } from '@/stores/useTaskStore';
import { useSystemModeStore } from '@/stores/useSystemModeStore';
import { ManualInterveneModal } from '@/components/modals/ManualInterveneModal';
import { cn } from '@/lib/utils';
import { Zap, Route, Users, Hand, Bot, ArrowRight, CheckCircle2, Clock } from 'lucide-react';

export function DispatchPanel() {
  const devices = useSystemStore((s) => s.devices);
  const tasks = useTaskStore((s) => s.tasks);
  const stats = useSystemStore((s) => s.stats);
  const [interveneOpen, setInterveneOpen] = useState(false);

  const pendingTasks = tasks.filter((t) => t.status === 'pending');
  const runningTasks = tasks.filter((t) => t.status === 'running');
  const idleDevices = devices.filter((d) => d.work_state === 'idle' && d.status === 'online');
  const workingDevices = devices.filter((d) => d.work_state === 'working');

  const autoAssignResult = pendingTasks.slice(0, idleDevices.length).map((task, i) => ({
    task: task.task_type,
    taskId: task.task_id.slice(0, 8),
    device: idleDevices[i]?.name || '无可用',
    deviceId: idleDevices[i]?.device_id || '',
    priority: task.priority,
  }));

  return (
    <div className="h-full flex flex-col">
      <div className="flex items-center justify-between mb-2">
        <div className="flex items-center gap-1.5">
          {/* <Zap className="w-4 h-4 text-cyan-400" />
          <span className="text-[13px] font-bold text-cyan-400 tracking-wider">智能调度</span> */}
        </div>
        <button
          onClick={() => setInterveneOpen(true)}
          className="flex items-center gap-1 px-2 py-1 rounded-md bg-tech-orange/10 border border-tech-orange/20 text-[10px] text-tech-orange font-bold hover:bg-tech-orange/20 transition-colors"
        >
          <Hand className="w-3 h-3" />
          人工干预
        </button>
      </div>

      <div className="grid grid-cols-3 gap-1.5 mb-2">
        <div className="tech-card p-2 text-center">
          <div className="text-[9px] text-text-muted">待调度</div>
          <div className="text-[14px] font-bold text-tech-orange font-mono">{pendingTasks.length}</div>
        </div>
        <div className="tech-card p-2 text-center">
          <div className="text-[9px] text-text-muted">执行中</div>
          <div className="text-[14px] font-bold text-cyan-400 font-mono">{runningTasks.length}</div>
        </div>
        <div className="tech-card p-2 text-center">
          <div className="text-[9px] text-text-muted">空闲设备</div>
          <div className="text-[14px] font-bold text-tech-green font-mono">{idleDevices.length}</div>
        </div>
      </div>

      <div className="flex-1 overflow-y-auto space-y-2">
        <div>
          <div className="flex items-center gap-1.5 mb-1.5">
            <Bot className="w-3.5 h-3.5 text-cyan-400" />
            <span className="text-[11px] font-bold text-cyan-400">自动派车建议</span>
          </div>
          {autoAssignResult.length > 0 ? (
            <div className="space-y-1.5">
              {autoAssignResult.map((item) => (
                <div key={item.taskId} className="tech-card p-2 flex items-center gap-2">
                  <div className="flex-1 min-w-0">
                    <div className="flex items-center gap-1.5">
                      <span className="text-[11px] font-semibold text-text-primary truncate">{item.task}</span>
                      <span className="text-[9px] text-text-muted font-mono">{item.taskId}</span>
                    </div>
                    <div className="flex items-center gap-1 mt-0.5">
                      <span className="text-[10px] text-text-muted">指派</span>
                      <ArrowRight className="w-2.5 h-2.5 text-cyan-400" />
                      <span className="text-[10px] text-cyan-400 font-semibold">{item.device}</span>
                    </div>
                  </div>
                  <span className={cn(
                    'text-[9px] px-1.5 py-0.5 rounded font-bold',
                    item.priority >= 10 ? 'bg-tech-red/15 text-tech-red' : item.priority >= 5 ? 'bg-tech-orange/15 text-tech-orange' : 'bg-cyan-500/15 text-cyan-400'
                  )}>
                    P{item.priority}
                  </span>
                </div>
              ))}
            </div>
          ) : (
            <div className="text-[11px] text-text-muted text-center py-3">
              {pendingTasks.length === 0 ? '无待调度任务' : '无空闲设备可分配'}
            </div>
          )}
        </div>

        <div>
          <div className="flex items-center gap-1.5 mb-1.5">
            <Route className="w-3.5 h-3.5 text-tech-green" />
            <span className="text-[11px] font-bold text-tech-green">当前调度状态</span>
          </div>
          <div className="space-y-1">
            {workingDevices.slice(0, 4).map((d) => (
              <div key={d.device_id} className="flex items-center gap-2 text-[11px]">
                <div className="status-dot" style={{ background: '#52c41a', boxShadow: '0 0 6px rgba(82,196,26,0.6)' }} />
                <span className="text-text-primary font-semibold">{d.name}</span>
                <span className="text-text-muted">→</span>
                <span className="text-text-muted font-mono">{d.task_id?.slice(0, 8) || '-'}</span>
                <span className="ml-auto text-[9px] text-text-muted font-mono">{d.battery}%</span>
              </div>
            ))}
            {workingDevices.length === 0 && (
              <div className="text-[10px] text-text-muted text-center py-2">无作业设备</div>
            )}
          </div>
        </div>
      </div>

      <ManualInterveneModal open={interveneOpen} onClose={() => setInterveneOpen(false)} />
    </div>
  );
}
