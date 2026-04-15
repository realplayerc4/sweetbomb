import { Tabs, Tag, Descriptions, Progress, Button } from 'antd';
import { TechModal } from './TechModal';
import { useTaskStore } from '@/stores/useTaskStore';
import { useSystemStore } from '@/stores/useSystemStore';
import { getStatusColor, getStatusLabel, formatDateTime, cn } from '@/lib/utils';
import { Play, Pause, Square, RotateCcw, AlertTriangle } from 'lucide-react';
import type { TaskInfo } from '@/types';

interface TaskDetailModalProps {
  task: TaskInfo | null;
  open: boolean;
  onClose: () => void;
}

export function TaskDetailModal({ task, open, onClose }: TaskDetailModalProps) {
  const startTask = useTaskStore((s) => s.startTask);
  const pauseTask = useTaskStore((s) => s.pauseTask);
  const resumeTask = useTaskStore((s) => s.resumeTask);
  const stopTask = useTaskStore((s) => s.stopTask);
  const devices = useSystemStore((s) => s.devices);

  if (!task) return null;

  const assignedDevices = task.assigned_devices
    ? devices.filter((d) => task.assigned_devices!.includes(d.device_id))
    : task.device_id
      ? devices.filter((d) => d.device_id === task.device_id)
      : [];

  const renderActions = () => {
    switch (task.status) {
      case 'pending':
        return (
          <Button type="primary" icon={<Play className="w-3.5 h-3.5" />} onClick={() => startTask(task.task_id)}>
            启动任务
          </Button>
        );
      case 'running':
        return (
          <div className="flex gap-2">
            <Button icon={<Pause className="w-3.5 h-3.5" />} onClick={() => pauseTask(task.task_id)}>暂停</Button>
            <Button danger icon={<Square className="w-3.5 h-3.5" />} onClick={() => stopTask(task.task_id)}>终止</Button>
          </div>
        );
      case 'paused':
        return (
          <div className="flex gap-2">
            <Button type="primary" icon={<RotateCcw className="w-3.5 h-3.5" />} onClick={() => resumeTask(task.task_id)}>恢复</Button>
            <Button danger icon={<Square className="w-3.5 h-3.5" />} onClick={() => stopTask(task.task_id)}>终止</Button>
          </div>
        );
      default:
        return null;
    }
  };

  return (
    <TechModal
      title={`任务详情 - ${task.task_id.slice(0, 8)}`}
      open={open}
      onCancel={onClose}
      footer={renderActions()}
      width={580}
    >
      <div className="py-2">
        <Tabs
          defaultActiveKey="info"
          items={[
            {
              key: 'info',
              label: '基本信息',
              children: (
                <div className="space-y-4">
                  <div className="flex items-center gap-3">
                    <div
                      className="status-dot shrink-0"
                      style={{
                        background: getStatusColor(task.status),
                        boxShadow: `0 0 8px ${getStatusColor(task.status)}80`,
                        width: 12,
                        height: 12,
                      }}
                    />
                    <span className="text-[16px] font-bold" style={{ color: getStatusColor(task.status) }}>
                      {getStatusLabel(task.status)}
                    </span>
                    <Tag color={task.priority >= 10 ? 'red' : task.priority >= 5 ? 'orange' : 'blue'} bordered={false}>
                      P{task.priority}
                    </Tag>
                  </div>

                  <div className="tech-card p-3 space-y-2">
                    <DetailRow label="任务ID" value={task.task_id} mono />
                    <DetailRow label="任务类型" value={task.task_type} />
                    <DetailRow label="主设备" value={task.device_id || '自动分配'} />
                    <DetailRow label="创建时间" value={formatDateTime(task.created_at)} mono />
                    {task.started_at && <DetailRow label="开始时间" value={formatDateTime(task.started_at)} mono />}
                    {task.completed_at && <DetailRow label="完成时间" value={formatDateTime(task.completed_at)} mono />}
                    {task.scoop_point && <DetailRow label="铲糖点" value={task.scoop_point} />}
                    {task.dump_point && <DetailRow label="卸糖点" value={task.dump_point} />}
                    {task.target_ton != null && <DetailRow label="目标量" value={`${task.target_ton} 吨`} />}
                  </div>

                  {assignedDevices.length > 0 && (
                    <div>
                      <div className="text-[12px] text-text-muted font-semibold mb-2">分配装载机</div>
                      <div className="flex flex-wrap gap-2">
                        {assignedDevices.map((d) => (
                          <div key={d.device_id} className="tech-card px-3 py-1.5 flex items-center gap-2">
                            <div className="status-dot" style={{ background: getStatusColor(d.status), boxShadow: `0 0 6px ${getStatusColor(d.status)}60` }} />
                            <span className="text-[12px] text-text-primary font-semibold">{d.name}</span>
                            <span className="text-[11px] text-text-muted font-mono">{d.battery}%</span>
                          </div>
                        ))}
                      </div>
                    </div>
                  )}
                </div>
              ),
            },
            {
              key: 'progress',
              label: '执行进度',
              children: (
                <div className="space-y-4">
                  {task.progress ? (
                    <>
                      <div className="tech-card p-4">
                        <div className="flex justify-between text-[13px] mb-2">
                          <span className="text-text-secondary">{task.progress.message}</span>
                          <span className="text-cyan-400 font-mono font-bold text-[16px]">{task.progress.percentage.toFixed(1)}%</span>
                        </div>
                        <Progress
                          percent={task.progress.percentage}
                          strokeColor="#00d4ff"
                          trailColor="rgba(0,212,255,0.1)"
                          showInfo={false}
                          size="default"
                        />
                        <div className="flex justify-between text-[11px] text-text-muted mt-2 font-mono">
                          <span>步骤 {task.progress.current_step}/{task.progress.total_steps}</span>
                          {task.progress.estimated_remaining_seconds != null && (
                            <span>预计剩余 {task.progress.estimated_remaining_seconds}s</span>
                          )}
                        </div>
                      </div>

                      <div className="grid grid-cols-2 gap-3">
                        <div className="tech-card p-3 text-center">
                          <div className="text-[10px] text-text-muted">已用时间</div>
                          <div className="text-[14px] font-bold text-cyan-400 font-mono">{Math.floor(task.progress.elapsed_seconds / 60)}m {task.progress.elapsed_seconds % 60}s</div>
                        </div>
                        <div className="tech-card p-3 text-center">
                          <div className="text-[10px] text-text-muted">剩余时间</div>
                          <div className="text-[14px] font-bold text-tech-orange font-mono">
                            {task.progress.estimated_remaining_seconds != null
                              ? `${Math.floor(task.progress.estimated_remaining_seconds / 60)}m ${task.progress.estimated_remaining_seconds % 60}s`
                              : '计算中...'}
                          </div>
                        </div>
                      </div>
                    </>
                  ) : (
                    <div className="text-center py-8 text-text-muted text-[13px]">任务尚未开始</div>
                  )}

                  {task.result && (
                    <div className="tech-card p-3">
                      <div className="text-[13px] text-text-muted font-semibold mb-2">执行结果</div>
                      <div className={cn('text-[14px] font-bold', task.result.success ? 'text-tech-green' : 'text-tech-red')}>
                        {task.result.success ? '执行成功' : '执行失败'}
                      </div>
                      {task.result.message && (
                        <p className="text-[13px] text-text-secondary mt-1">{task.result.message}</p>
                      )}
                    </div>
                  )}

                  {task.status === 'failed' && (
                    <div className="p-3 rounded-md border border-tech-red/20 bg-tech-red/5 flex items-center gap-2">
                      <AlertTriangle className="w-4 h-4 text-tech-red shrink-0" />
                      <span className="text-[12px] text-tech-red">任务执行异常，请检查设备状态或进行人工干预</span>
                    </div>
                  )}
                </div>
              ),
            },
          ]}
        />
      </div>
    </TechModal>
  );
}

function DetailRow({ label, value, mono }: { label: string; value: string; mono?: boolean }) {
  return (
    <div className="flex items-center justify-between">
      <span className="text-[12px] text-text-muted">{label}</span>
      <span className={cn('text-[13px] text-text-primary', mono && 'font-mono')}>{value}</span>
    </div>
  );
}
