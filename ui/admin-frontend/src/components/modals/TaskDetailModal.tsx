import { TechModal } from './TechModal';
import { formatDateTime, cn } from '@/lib/utils';
import { CheckCircle2, Clock, Package } from 'lucide-react';

interface TaskDetailModalProps {
  task: any | null;
  open: boolean;
  onClose: () => void;
}

export function TaskDetailModal({ task, open, onClose }: TaskDetailModalProps) {
  if (!task) return null;

  return (
    <TechModal
      title={`任务详情`}
      open={open}
      onCancel={onClose}
      footer={null}
      width={480}
    >
      <div className="py-4 space-y-4">
        <div className="tech-card p-4 space-y-3">
          <div className="flex items-center justify-between">
            <div className="flex items-center gap-2">
              <Package className="w-4 h-4 text-cyan-400" />
              <span className="text-[14px] font-bold text-text-primary">任务目标</span>
            </div>
            <span className="text-[16px] font-bold text-cyan-400 font-mono">{task.targetKg || 900} kg</span>
          </div>

          <div className="flex items-center justify-between">
            <div className="flex items-center gap-2">
              <CheckCircle2 className="w-4 h-4 text-tech-green" />
              <span className="text-[14px] font-bold text-text-primary">已完成</span>
            </div>
            <span className="text-[16px] font-bold text-tech-green font-mono">{task.completedKg || 0} kg</span>
          </div>

          <div className="flex items-center justify-between">
            <div className="flex items-center gap-2">
              <Clock className="w-4 h-4 text-tech-orange" />
              <span className="text-[14px] font-bold text-text-primary">进度</span>
            </div>
            <span className="text-[16px] font-bold text-tech-orange font-mono">
              {task.currentCycle || 0}/{task.totalCycles || 30} 次
            </span>
          </div>
        </div>

        <div className="text-[12px] text-text-muted text-center">
          每次循环装载 30 kg
        </div>
      </div>
    </TechModal>
  );
}
