import { useState, useMemo } from 'react';
import { Select, InputNumber, TimePicker, Upload, message } from 'antd';
import { TechModal } from './TechModal';
import { useTaskStore } from '@/stores/useTaskStore';
import { useSystemModeStore } from '@/stores/useSystemModeStore';
import { useSystemStore } from '@/stores/useSystemStore';
import { Plus, Sparkles, Clock, Target, Bot, Camera, Upload as UploadIcon } from 'lucide-react';
import type { TaskCreateRequest } from '@/types';
import dayjs from 'dayjs';
import { cn } from '@/lib/utils';

const AVG_CYCLE_TIME_MIN = 3.5;
const BUCKET_CAPACITY_TON = 0.85;

export function TaskCreateModal() {
  const [open, setOpen] = useState(false);
  const [taskType, setTaskType] = useState('sugar_harvest');
  const [targetAmount, setTargetAmount] = useState<number | null>(10);
  const [startTime, setStartTime] = useState<dayjs.Dayjs | null>(null);
  const [endTime, setEndTime] = useState<dayjs.Dayjs | null>(null);
  const [priority, setPriority] = useState(5);
  const [recommendedCount, setRecommendedCount] = useState<number>(3);
  const [scoopPointPhoto, setScoopPointPhoto] = useState<string | null>(null);
  const [dumpPointPhoto, setDumpPointPhoto] = useState<string | null>(null);
  const [scoopPointName, setScoopPointName] = useState('A区铲糖点');
  const [dumpPointName, setDumpPointName] = useState('B区卸糖点');
  const [loading, setLoading] = useState(false);
  const [errors, setErrors] = useState<Record<string, string>>({});

  const createTask = useTaskStore((s) => s.createTask);
  const monitorMode = useSystemModeStore((s) => s.monitorMode);
  const selectedRobot = useSystemModeStore((s) => s.selectedRobot);
  const devices = useSystemStore((s) => s.devices);

  const availableRobots = useMemo(() =>
    devices.filter((d) => d.status === 'online' && d.work_state === 'idle' && d.battery > 30),
    [devices]
  );

  const lowBatteryRobots = devices.filter((d) => d.status === 'online' && d.battery <= 30);
  const workingRobots = devices.filter((d) => d.work_state === 'working');

  const aiMaxCapacity = useMemo(() => {
    if (!startTime || !endTime) return null;
    const durationMin = endTime.diff(startTime, 'minute', true);
    if (durationMin <= 0) return null;
    const totalCycles = Math.floor(durationMin / AVG_CYCLE_TIME_MIN);
    return Number((totalCycles * recommendedCount * BUCKET_CAPACITY_TON).toFixed(1));
  }, [startTime, endTime, recommendedCount]);

  const recommendedLoaderCount = useMemo(() => {
    if (!targetAmount || !aiMaxCapacity || aiMaxCapacity <= 0) return recommendedCount;
    const count = Math.ceil(targetAmount / (aiMaxCapacity / recommendedCount));
    return Math.min(count, availableRobots.length);
  }, [targetAmount, aiMaxCapacity, recommendedCount, availableRobots.length]);

  const validate = (): boolean => {
    const newErrors: Record<string, string> = {};
    if (!targetAmount || targetAmount <= 0) newErrors.targetAmount = '请设定目标铲糖吨数';
    if (!startTime) newErrors.startTime = '请选择开始时间';
    if (!endTime) newErrors.endTime = '请选择结束时间';
    if (startTime && endTime && endTime.isBefore(startTime)) newErrors.endTime = '结束时间必须晚于开始时间';
    if (!scoopPointName.trim()) newErrors.scoopPoint = '请输入铲糖点名称';
    if (!dumpPointName.trim()) newErrors.dumpPoint = '请输入卸糖点名称';
    setErrors(newErrors);
    return Object.keys(newErrors).length === 0;
  };

  const handleCreate = async () => {
    if (!validate()) return;
    setLoading(true);
    try {
      const assignedDevices = monitorMode === 'all'
        ? availableRobots.slice(0, recommendedCount).map((r) => r.device_id)
        : [selectedRobot];

      const request: TaskCreateRequest = {
        task_type: taskType,
        device_id: monitorMode === 'single' ? selectedRobot : null,
        config: { priority: priority as 1 | 5 | 10 | 20 },
        params: {
          target_ton: targetAmount,
          start_time: startTime?.format('HH:mm'),
          end_time: endTime?.format('HH:mm'),
          max_capacity: aiMaxCapacity,
          assigned_robots: assignedDevices,
          scoop_point: scoopPointName,
          dump_point: dumpPointName,
          scoop_point_photo: scoopPointPhoto,
          dump_point_photo: dumpPointPhoto,
          recommended_loader_count: recommendedCount,
        },
      };
      await createTask(request);
      message.success('任务创建成功');
      setOpen(false);
      resetForm();
    } catch {
      message.error('任务创建失败');
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
    setRecommendedCount(3);
    setScoopPointPhoto(null);
    setDumpPointPhoto(null);
    setScoopPointName('A区铲糖点');
    setDumpPointName('B区卸糖点');
    setErrors({});
  };

  const handlePhotoUpload = (type: 'scoop' | 'dump', file: File) => {
    const reader = new FileReader();
    reader.onload = (e) => {
      const result = e.target?.result as string;
      if (type === 'scoop') setScoopPointPhoto(result);
      else setDumpPointPhoto(result);
    };
    reader.readAsDataURL(file);
    return false;
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

      <TechModal
        title="制定新任务"
        open={open}
        onOk={handleCreate}
        onCancel={() => { setOpen(false); resetForm(); }}
        confirmLoading={loading}
        okText="发布任务"
        cancelText="取消"
        width={600}
      >
        <div className="space-y-4 py-2 max-h-[65vh] overflow-y-auto">
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
                {monitorMode === 'single' ? `单机 · ${selectedRobot}` : `多机 · ${availableRobots.length}台空闲`}
              </span>
            </label>
            <div className="grid grid-cols-3 gap-2 mt-1">
              <div className="tech-card p-2">
                <div className="text-[10px] text-tech-green mb-0.5">空闲设备</div>
                <div className="text-[14px] font-bold text-tech-green font-mono">{availableRobots.length} 台</div>
              </div>
              <div className="tech-card p-2">
                <div className="text-[10px] text-cyan-400 mb-0.5">作业中</div>
                <div className="text-[14px] font-bold text-cyan-400 font-mono">{workingRobots.length} 台</div>
              </div>
              <div className="tech-card p-2">
                <div className="text-[10px] text-tech-orange mb-0.5">低电量/充电中</div>
                <div className="text-[14px] font-bold text-tech-orange font-mono">{lowBatteryRobots.length} 台</div>
              </div>
            </div>
          </div>

          {taskType === 'sugar_harvest' && (
            <>
              <div className="grid grid-cols-2 gap-3">
                <div>
                  <label className="text-[12px] text-text-muted block mb-1 font-semibold flex items-center gap-1">
                    <Clock className="w-3 h-3 text-cyan-400" /> 开始时间 *
                  </label>
                  <TimePicker
                    value={startTime}
                    onChange={(v) => { setStartTime(v); if (errors.startTime) setErrors((p) => ({ ...p, startTime: '' })); }}
                    format="HH:mm"
                    minuteStep={5}
                    className="w-full"
                    placeholder="选择开始时间"
                    status={errors.startTime ? 'error' : undefined}
                  />
                  {errors.startTime && <span className="text-[11px] text-tech-red mt-1 block">{errors.startTime}</span>}
                </div>
                <div>
                  <label className="text-[12px] text-text-muted block mb-1 font-semibold flex items-center gap-1">
                    <Clock className="w-3 h-3 text-cyan-400" /> 结束时间 *
                  </label>
                  <TimePicker
                    value={endTime}
                    onChange={(v) => { setEndTime(v); if (errors.endTime) setErrors((p) => ({ ...p, endTime: '' })); }}
                    format="HH:mm"
                    minuteStep={5}
                    className="w-full"
                    placeholder="选择结束时间"
                    status={errors.endTime ? 'error' : undefined}
                  />
                  {errors.endTime && <span className="text-[11px] text-tech-red mt-1 block">{errors.endTime}</span>}
                </div>
              </div>

              <div>
                <label className="text-[12px] text-text-muted block mb-1 font-semibold flex items-center gap-1">
                  <Target className="w-3 h-3 text-cyan-400" /> 铲糖目标量（吨）*
                </label>
                <InputNumber
                  value={targetAmount}
                  onChange={(v) => { setTargetAmount(v); if (errors.targetAmount) setErrors((p) => ({ ...p, targetAmount: '' })); }}
                  min={0.5}
                  max={aiMaxCapacity || 999}
                  step={0.5}
                  precision={1}
                  className="w-full"
                  placeholder="输入目标量"
                  addonAfter="吨"
                  status={errors.targetAmount ? 'error' : undefined}
                />
                {errors.targetAmount && <span className="text-[11px] text-tech-red mt-1 block">{errors.targetAmount}</span>}
              </div>

              <div>
                <label className="text-[12px] text-text-muted block mb-1 font-semibold flex items-center gap-1">
                  <Bot className="w-3 h-3 text-cyan-400" /> 推荐装载机数量
                </label>
                <InputNumber
                  value={recommendedCount}
                  onChange={(v) => v != null && setRecommendedCount(v)}
                  min={1}
                  max={availableRobots.length || 1}
                  step={1}
                  className="w-full"
                />
                <span className="text-[10px] text-text-muted mt-1 block">系统根据目标量自动推荐，可手动调整</span>
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
                    {targetAmount && aiMaxCapacity && targetAmount > aiMaxCapacity && (
                      <div className="mt-2 p-2 rounded bg-tech-red/10 border border-tech-red/20">
                        <span className="text-[11px] text-tech-red">目标量超出AI计算上限，建议调整为 {aiMaxCapacity} 吨以内</span>
                      </div>
                    )}
                  </div>
                ) : (
                  <span className="text-[11px] text-text-muted">请先选择时间段以获取AI计算结果</span>
                )}
              </div>

              <div className="grid grid-cols-2 gap-3">
                <div>
                  <label className="text-[12px] text-text-muted block mb-1 font-semibold flex items-center gap-1">
                    <Camera className="w-3 h-3 text-cyan-400" /> 铲糖点 *
                  </label>
                  <input
                    type="text"
                    value={scoopPointName}
                    onChange={(e) => { setScoopPointName(e.target.value); if (errors.scoopPoint) setErrors((p) => ({ ...p, scoopPoint: '' })); }}
                    className="w-full h-8 bg-bg-primary/60 border border-cyan-500/20 rounded px-2 text-[13px] text-text-secondary focus:outline-none focus:border-cyan-400 mb-1.5"
                    placeholder="输入铲糖点名称"
                  />
                  <Upload
                    beforeUpload={(file) => handlePhotoUpload('scoop', file)}
                    showUploadList={false}
                    accept="image/*"
                  >
                    <div className="tech-card p-2 flex items-center justify-center gap-1 cursor-pointer hover:border-cyan-500/30 transition-colors">
                      {scoopPointPhoto ? (
                        <img src={scoopPointPhoto} alt="铲糖点" className="w-full h-16 object-cover rounded" />
                      ) : (
                        <>
                          <UploadIcon className="w-3 h-3 text-text-muted" />
                          <span className="text-[10px] text-text-muted">上传铲糖点照片</span>
                        </>
                      )}
                    </div>
                  </Upload>
                  {errors.scoopPoint && <span className="text-[11px] text-tech-red mt-1 block">{errors.scoopPoint}</span>}
                </div>
                <div>
                  <label className="text-[12px] text-text-muted block mb-1 font-semibold flex items-center gap-1">
                    <Camera className="w-3 h-3 text-cyan-400" /> 卸糖点 *
                  </label>
                  <input
                    type="text"
                    value={dumpPointName}
                    onChange={(e) => { setDumpPointName(e.target.value); if (errors.dumpPoint) setErrors((p) => ({ ...p, dumpPoint: '' })); }}
                    className="w-full h-8 bg-bg-primary/60 border border-cyan-500/20 rounded px-2 text-[13px] text-text-secondary focus:outline-none focus:border-cyan-400 mb-1.5"
                    placeholder="输入卸糖点名称"
                  />
                  <Upload
                    beforeUpload={(file) => handlePhotoUpload('dump', file)}
                    showUploadList={false}
                    accept="image/*"
                  >
                    <div className="tech-card p-2 flex items-center justify-center gap-1 cursor-pointer hover:border-cyan-500/30 transition-colors">
                      {dumpPointPhoto ? (
                        <img src={dumpPointPhoto} alt="卸糖点" className="w-full h-16 object-cover rounded" />
                      ) : (
                        <>
                          <UploadIcon className="w-3 h-3 text-text-muted" />
                          <span className="text-[10px] text-text-muted">上传卸糖点照片</span>
                        </>
                      )}
                    </div>
                  </Upload>
                  {errors.dumpPoint && <span className="text-[11px] text-tech-red mt-1 block">{errors.dumpPoint}</span>}
                </div>
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
      </TechModal>
    </>
  );
}
