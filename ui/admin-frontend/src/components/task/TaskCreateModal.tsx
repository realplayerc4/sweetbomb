import { useState, useMemo } from 'react';
import { Plus, Sparkles, Clock, Target, Bot } from 'lucide-react';
import { useTaskStore } from '@/stores/useTaskStore';
import { useSystemModeStore } from '@/stores/useSystemModeStore';
import { useSystemStore } from '@/stores/useSystemStore';
import { Modal, Select, InputNumber, TimePicker } from 'antd';
import type { TaskCreateRequest } from '@/types';
import dayjs from 'dayjs';

const AVG_CYCLE_TIME_MIN = 3.5;
const BUCKET_CAPACITY_TON = 0.85;

export function TaskCreateModal() {
  const [open, setOpen] = useState(false);
  const [taskType, setTaskType] = useState('sugar_harvest');
  const [targetAmount, setTargetAmount] = useState<number | null>(10);
  const [startTime, setStartTime] = useState<dayjs.Dayjs | null>(null);
  const [endTime, setEndTime] = useState<dayjs.Dayjs | null>(null);
  const [priority, setPriority] = useState(5);
  const [loading, setLoading] = useState(false);
  const createTask = useTaskStore((s: { createTask: any; }) => s.createTask);
  const monitorMode = useSystemModeStore((s: { monitorMode: any; }) => s.monitorMode);
  const selectedRobot = useSystemModeStore((s: { selectedRobot: any; }) => s.selectedRobot);
  const devices = useSystemStore((s: { devices: any; }) => s.devices);

  const availableRobots = useMemo(() =>
    devices.filter((d: { status: string; battery: number; }) => d.status === 'online' && d.battery > 30),
    [devices]
  );

  const lowBatteryRobots = devices.filter((d: { status: string; battery: number; }) => d.status === 'online' && d.battery <= 30);
  const chargingRobots = devices.filter((d: { status: string; battery: number; }) => d.status === 'offline' || (d.status === 'warning' && d.battery < 20));

  const aiMaxCapacity = useMemo(() => {
    if (!startTime || !endTime) return null;
    const durationMin = endTime.diff(startTime, 'minute', true);
    if (durationMin <= 0) return null;
    const totalCycles = Math.floor(durationMin / AVG_CYCLE_TIME_MIN);
    return Number((totalCycles * availableRobots.length * BUCKET_CAPACITY_TON).toFixed(1));
  }, [startTime, endTime, availableRobots]);

  const handleCreate = async () => {
    setLoading(true);
    try {
      const request: TaskCreateRequest = {
        task_type: taskType,
        device_id: monitorMode === 'single' ? selectedRobot : null,
        config: { priority: priority as 1 | 5 | 10 | 20 },
        params: {
          target_ton: targetAmount,
          start_time: startTime?.format('HH:mm'),
          end_time: endTime?.format('HH:mm'),
          max_capacity: aiMaxCapacity,
          assigned_robots: monitorMode === 'all' ? availableRobots.map((r: { device_id: any; }) => r.device_id) : [selectedRobot],
        },
      };
      await createTask(request);
      setOpen(false);
      resetForm();
    } catch {
    } finally {
      setLoading(false);
    }
  };

  const resetForm = () => {
    setTaskType('sugar_harvest');
    setTargetAmount(10);
    setStartTime(null);
    setEndTime(null);
    setPriority(5);
  };

  return (
    <>
      <button
        onClick={() => setOpen(true)}
        className="flex items-center gap-2 px-3 py-2 rounded-lg bg-cyan-500/15 text-cyan-400 border border-cyan-500/25 font-bold text-[12px] tracking-wider hover:bg-cyan-500/25 transition-colors"
      >
        <Plus className="w-4 h-4" />
        创建任务
      </button>

      <Modal
        title={<span className="text-cyan-400 font-bold text-[14px]">创建新任务</span>}
        open={open}
        onOk={handleCreate}
        onCancel={() => { setOpen(false); resetForm(); }}
        confirmLoading={loading}
        okText="发布任务"
        cancelText="取消"
        width={520}
        className="[&_.ant-modal-content]:bg-[#0d1b36] [&_.ant-modal-content]:border [&_.ant-modal-content]:border-cyan-500/20 [&_.ant-modal-header]:bg-transparent [&_.ant-modal-body]:bg-transparent [&_.ant-modal-footer]:bg-transparent [&_.ant-modal-title]:text-cyan-400"
      >
        <div className="space-y-4 py-2">
          <div>
            <label className="text-[12px] text-text-muted block mb-1 font-semibold">任务类型</label>
            <Select
              value={taskType}
              onChange={setTaskType}
              className="w-full"
              options={[
                { value: 'sugar_harvest', label: '铲糖作业' },
                { value: 'navigation', label: '导航任务' },
                { value: 'inspection', label: '巡检任务' },
                { value: 'charging', label: '充电任务' },
              ]}
            />
          </div>

          <div>
            <label className="text-[12px] text-text-muted block mb-1 font-semibold">
              执行模式
              <span className="ml-2 px-1.5 py-0.5 rounded bg-cyan-500/15 text-cyan-400 text-[10px] font-bold">
                {monitorMode === 'single' ? `单机 · ${selectedRobot}` : `多机 · ${availableRobots.length}台可用`}
              </span>
            </label>
            <div className="grid grid-cols-2 gap-2 mt-1">
              <div className="tech-card p-2">
                <div className="text-[10px] text-tech-green mb-0.5">可用设备</div>
                <div className="text-[14px] font-bold text-tech-green font-mono">{availableRobots.length} 台</div>
              </div>
              <div className="tech-card p-2">
                <div className="text-[10px] text-tech-orange mb-0.5">低电量/充电中</div>
                <div className="text-[14px] font-bold text-tech-orange font-mono">{lowBatteryRobots.length + chargingRobots.length} 台</div>
              </div>
            </div>
          </div>

          {taskType === 'sugar_harvest' && (
            <>
              <div className="grid grid-cols-2 gap-3">
                <div>
                  <label className="text-[12px] text-text-muted block mb-1 font-semibold flex items-center gap-1">
                    <Clock className="w-3 h-3 text-cyan-400" /> 开始时间
                  </label>
                  <TimePicker
                    value={startTime}
                    onChange={(v) => setStartTime(v)}
                    format="HH:mm"
                    minuteStep={5}
                    className="w-full"
                    placeholder="选择开始时间"
                  />
                </div>
                <div>
                  <label className="text-[12px] text-text-muted block mb-1 font-semibold flex items-center gap-1">
                    <Clock className="w-3 h-3 text-cyan-400" /> 结束时间
                  </label>
                  <TimePicker
                    value={endTime}
                    onChange={(v) => setEndTime(v)}
                    format="HH:mm"
                    minuteStep={5}
                    className="w-full"
                    placeholder="选择结束时间"
                  />
                </div>
              </div>

              <div>
                <label className="text-[12px] text-text-muted block mb-1 font-semibold flex items-center gap-1">
                  <Target className="w-3 h-3 text-cyan-400" /> 铲糖目标量（吨）
                </label>
                <InputNumber
                  value={targetAmount}
                  onChange={(v) => setTargetAmount(v)}
                  min={0.5}
                  max={aiMaxCapacity || 999}
                  step={0.5}
                  precision={1}
                  className="w-full"
                  placeholder="输入目标量"
                  addonAfter="吨"
                />
              </div>

              <div className="p-3 rounded-md border border-cyan-500/20 bg-cyan-500/5">
                <div className="flex items-center gap-1.5 mb-2">
                  <Sparkles className="w-4 h-4 text-cyan-400" />
                  <span className="text-[13px] font-bold text-cyan-400">AI 辅助计算</span>
                </div>
                {aiMaxCapacity !== null ? (
                  <div className="space-y-1.5">
                    <div className="flex justify-between text-[11px]">
                      <span className="text-text-muted">理论最大铲糖量</span>
                      <span className="text-cyan-400 font-mono font-bold text-[14px] tech-glow-number">{aiMaxCapacity} 吨</span>
                    </div>
                    <div className="flex justify-between text-[11px]">
                      <span className="text-text-muted">单次循环时长</span>
                      <span className="text-text-secondary font-mono">{AVG_CYCLE_TIME_MIN} min</span>
                    </div>
                    <div className="flex justify-between text-[11px]">
                      <span className="text-text-muted">单次装载量</span>
                      <span className="text-text-secondary font-mono">{BUCKET_CAPACITY_TON} 吨</span>
                    </div>
                    <div className="flex justify-between text-[11px]">
                      <span className="text-text-muted">预计循环次数</span>
                      <span className="text-text-secondary font-mono">
                        {startTime && endTime ? Math.floor(endTime.diff(startTime, 'minute', true) / AVG_CYCLE_TIME_MIN) : '-'}
                      </span>
                    </div>
                    {targetAmount && targetAmount > aiMaxCapacity && (
                      <div className="mt-2 p-2 rounded bg-tech-red/10 border border-tech-red/20">
                        <span className="text-[11px] text-tech-red">目标量超出AI计算上限，建议调整为 {aiMaxCapacity} 吨以内</span>
                      </div>
                    )}
                  </div>
                ) : (
                  <span className="text-[11px] text-text-muted">请先选择时间段以获取AI计算结果</span>
                )}
              </div>
            </>
          )}

          <div>
            <label className="text-[12px] text-text-muted block mb-1 font-semibold">优先级</label>
            <Select
              value={priority}
              onChange={setPriority}
              className="w-full"
              options={[
                { value: 1, label: '低 (1)' },
                { value: 5, label: '中 (5)' },
                { value: 10, label: '高 (10)' },
                { value: 20, label: '紧急 (20)' },
              ]}
            />
          </div>
        </div>
      </Modal>
    </>
  );
}
