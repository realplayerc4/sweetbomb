import { useState } from 'react';
import { Select, Button, message, Tag } from 'antd';
import { TechModal } from './TechModal';
import { useSystemStore } from '@/stores/useSystemStore';
import { useTaskStore } from '@/stores/useTaskStore';
import { useSystemModeStore } from '@/stores/useSystemModeStore';
import { Hand, ArrowRight, AlertTriangle } from 'lucide-react';
import { cn } from '@/lib/utils';
import type { TaskInfo, DeviceInfo, DispatchAssignment } from '@/types';

interface ManualInterveneModalProps {
  open: boolean;
  onClose: () => void;
  task?: TaskInfo | null;
}

export function ManualInterveneModal({ open, onClose, task }: ManualInterveneModalProps) {
  const devices = useSystemStore((s) => s.devices);
  const updateDeviceWorkState = useSystemStore((s) => s.updateDeviceWorkState);
  const tasks = useTaskStore((s) => s.tasks);
  const [selectedDevice, setSelectedDevice] = useState<string | null>(null);
  const [selectedTask, setSelectedTask] = useState<string | null>(task?.task_id || null);
  const [assignments, setAssignments] = useState<DispatchAssignment[]>([]);

  const idleDevices = devices.filter((d) => d.work_state === 'idle' && d.status === 'online');
  const workingDevices = devices.filter((d) => d.work_state === 'working');
  const pendingTasks = tasks.filter((t) => t.status === 'pending');

  const handleAssign = () => {
    if (!selectedDevice || !selectedTask) {
      message.warning('请选择设备和任务');
      return;
    }
    const device = devices.find((d) => d.device_id === selectedDevice);
    const t = tasks.find((tk) => tk.task_id === selectedTask);
    if (!device || !t) return;

    const newAssignment: DispatchAssignment = {
      task_id: selectedTask,
      device_id: selectedDevice,
      priority: t.priority,
      reason: '人工干预指派',
    };
    setAssignments((prev) => [...prev, newAssignment]);
    updateDeviceWorkState(selectedDevice, 'working');
    message.success(`已将 ${device.name} 指派给任务 ${selectedTask.slice(0, 8)}`);
    setSelectedDevice(null);
    setSelectedTask(null);
  };

  const handleRemoveAssignment = (taskId: string) => {
    const assignment = assignments.find((a) => a.task_id === taskId);
    if (assignment) {
      updateDeviceWorkState(assignment.device_id, 'idle');
    }
    setAssignments((prev) => prev.filter((a) => a.task_id !== taskId));
  };

  return (
    <TechModal
      title="人工干预调度"
      open={open}
      onCancel={onClose}
      footer={null}
      width={580}
    >
      <div className="space-y-4 py-2">
        <div className="p-3 rounded-md border border-tech-orange/20 bg-tech-orange/5">
          <div className="flex items-center gap-1.5 mb-1">
            <AlertTriangle className="w-3.5 h-3.5 text-tech-orange" />
            <span className="text-[12px] font-bold text-tech-orange">人工干预模式</span>
          </div>
          <span className="text-[11px] text-text-secondary">手动调整自动调度结果，可重新指派设备或修改任务分配。修改后将覆盖系统自动调度决策。</span>
        </div>

        <div>
          <div className="text-[12px] text-text-muted font-semibold mb-2">当前设备状态</div>
          <div className="grid grid-cols-2 gap-2">
            <div className="tech-card p-2">
              <div className="text-[10px] text-tech-green mb-1">空闲设备</div>
              <div className="space-y-1">
                {idleDevices.map((d) => (
                  <div key={d.device_id} className="flex items-center gap-2 text-[11px]">
                    <div className="status-dot" style={{ background: '#52c41a', boxShadow: '0 0 6px rgba(82,196,26,0.6)' }} />
                    <span className="text-text-primary">{d.name}</span>
                    <span className="text-text-muted font-mono ml-auto">{d.battery}%</span>
                  </div>
                ))}
                {idleDevices.length === 0 && <span className="text-[10px] text-text-muted">无空闲设备</span>}
              </div>
            </div>
            <div className="tech-card p-2">
              <div className="text-[10px] text-cyan-400 mb-1">作业中设备</div>
              <div className="space-y-1">
                {workingDevices.map((d) => (
                  <div key={d.device_id} className="flex items-center gap-2 text-[11px]">
                    <div className="status-dot" style={{ background: '#1677ff', boxShadow: '0 0 6px rgba(22,119,255,0.6)' }} />
                    <span className="text-text-primary">{d.name}</span>
                    <span className="text-text-muted font-mono ml-auto">{d.task_id?.slice(0, 6) || '-'}</span>
                  </div>
                ))}
                {workingDevices.length === 0 && <span className="text-[10px] text-text-muted">无作业设备</span>}
              </div>
            </div>
          </div>
        </div>

        <div>
          <div className="text-[12px] text-text-muted font-semibold mb-2">手动指派</div>
          <div className="flex gap-2 items-end">
            <div className="flex-1">
              <label className="text-[10px] text-text-muted block mb-1">选择设备</label>
              <Select
                value={selectedDevice}
                onChange={setSelectedDevice}
                className="w-full"
                placeholder="选择空闲设备"
                options={idleDevices.map((d) => ({ value: d.device_id, label: `${d.name} (${d.battery}%)` }))}
              />
            </div>
            <ArrowRight className="w-4 h-4 text-text-muted shrink-0 mb-1.5" />
            <div className="flex-1">
              <label className="text-[10px] text-text-muted block mb-1">选择任务</label>
              <Select
                value={selectedTask}
                onChange={setSelectedTask}
                className="w-full"
                placeholder="选择待执行任务"
                options={pendingTasks.map((t) => ({ value: t.task_id, label: `${t.task_type} (${t.task_id.slice(0, 8)})` }))}
              />
            </div>
            <Button type="primary" onClick={handleAssign} disabled={!selectedDevice || !selectedTask}>
              指派
            </Button>
          </div>
        </div>

        {assignments.length > 0 && (
          <div>
            <div className="text-[12px] text-text-muted font-semibold mb-2">干预记录</div>
            <div className="space-y-2">
              {assignments.map((a) => {
                const device = devices.find((d) => d.device_id === a.device_id);
                const t = tasks.find((tk) => tk.task_id === a.task_id);
                return (
                  <div key={a.task_id} className="tech-card p-2.5 flex items-center gap-3">
                    <Hand className="w-4 h-4 text-tech-orange shrink-0" />
                    <div className="flex-1 min-w-0">
                      <div className="text-[12px] text-text-primary">
                        <span className="font-semibold">{device?.name || a.device_id}</span>
                        <ArrowRight className="w-3 h-3 inline mx-1 text-text-muted" />
                        <span className="font-mono">{a.task_id.slice(0, 8)}</span>
                      </div>
                      <div className="text-[10px] text-text-muted">{a.reason}</div>
                    </div>
                    <Tag color="orange" bordered={false} className="text-[10px]">P{a.priority}</Tag>
                    <button
                      onClick={() => handleRemoveAssignment(a.task_id)}
                      className="p-1 rounded hover:bg-white/10 text-text-muted hover:text-tech-red transition-colors"
                    >
                      ×
                    </button>
                  </div>
                );
              })}
            </div>
          </div>
        )}
      </div>
    </TechModal>
  );
}
