import { useState } from 'react';
import { InputNumber, message } from 'antd';
import { TechModal } from '@/components/modals/TechModal';
import { useTaskStore } from '@/stores/useTaskStore';
import { useRobotStore } from '@/stores/useRobotStore';
import { Plus, Target, Package } from 'lucide-react';

export function TaskCreateModal() {
  const [open, setOpen] = useState(false);
  const [targetKg, setTargetKg] = useState<number>(900);

  const setTarget = useTaskStore((s) => s.setTarget);
  const currentTarget = useTaskStore((s) => s.targetKg);
  const isConnected = useRobotStore((s) => s.isConnected);

  const handleCreate = () => {
    if (!targetKg || targetKg <= 0) {
      message.error('请输入有效的目标重量');
      return;
    }
    setTarget(targetKg);
    message.success(`任务目标已设置为 ${targetKg} kg`);
    setOpen(false);
  };

  return (
    <>
      <button
        onClick={() => setOpen(true)}
        disabled={!isConnected}
        className="flex items-center gap-2 px-3 py-2 rounded-lg bg-cyan-500/15 text-cyan-400 border border-cyan-500/25 font-bold text-[12px] tracking-wider hover:bg-cyan-500/25 transition-colors disabled:opacity-50 disabled:cursor-not-allowed"
      >
        <Plus className="w-4 h-4" />
        设置目标
      </button>

      <TechModal
        title="设置任务目标"
        open={open}
        onOk={handleCreate}
        onCancel={() => setOpen(false)}
        okText="确认"
        cancelText="取消"
        width={400}
      >
        <div className="space-y-4 py-4">
          <div>
            <label className="text-[13px] text-text-muted block mb-2 font-semibold flex items-center gap-1">
              <Target className="w-3.5 h-3.5 text-cyan-400" />
              目标重量 (kg)
            </label>
            <InputNumber
              value={targetKg}
              onChange={(v) => setTargetKg(v || 900)}
              min={30}
              max={10000}
              step={30}
              className="w-full"
              placeholder="输入目标重量"
              addonAfter="kg"
            />
          </div>

          <div className="p-3 rounded-md border border-cyan-500/20 bg-cyan-500/5">
            <div className="flex items-center gap-1.5 mb-2">
              <Package className="w-4 h-4 text-cyan-400" />
              <span className="text-[12px] font-bold text-cyan-400">任务说明</span>
            </div>
            <div className="space-y-1 text-[11px] text-text-muted">
              <div>• 每次循环装载 <span className="text-cyan-400 font-mono">30 kg</span></div>
              <div>• 目标 <span className="text-cyan-400 font-mono">{targetKg} kg</span> = <span className="text-cyan-400 font-mono">{Math.ceil(targetKg / 30)}</span> 次循环</div>
              <div>• 执行流程：导航取货 → 取货 → 导航卸货 → 卸货</div>
            </div>
          </div>

          {currentTarget > 0 && (
            <div className="text-[11px] text-text-muted text-center">
              当前目标: <span className="text-cyan-400 font-mono">{currentTarget} kg</span>
            </div>
          )}
        </div>
      </TechModal>
    </>
  );
}