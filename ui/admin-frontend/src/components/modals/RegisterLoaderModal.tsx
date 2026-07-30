import { useState } from 'react';
import { Input, InputNumber, Select, Form, message } from 'antd';
import { TechModal } from './TechModal';
import { useSystemStore } from '@/stores/useSystemStore';
import { Plus, Truck } from 'lucide-react';
import type { LoaderRegistration } from '@/types';

const MODELS = [
  { value: 'ZL-30', label: 'ZL-30 (小型装载机)' },
  { value: 'ZL-50', label: 'ZL-50 (中型装载机)' },
  { value: 'ZL-80', label: 'ZL-80 (大型装载机)' },
];

export function RegisterLoaderModal() {
  const [open, setOpen] = useState(false);
  const [loading, setLoading] = useState(false);
  const [form, setForm] = useState<LoaderRegistration>({
    name: '',
    model: 'ZL-50',
    bucket_capacity: 0.85,
    max_speed: 0.5,
    bucket_width: 0.8,
    max_load_weight: 5.0,
  });
  const [errors, setErrors] = useState<Partial<Record<keyof LoaderRegistration, string>>>({});
  const registerDevice = useSystemStore((s) => s.registerDevice);

  const validate = (): boolean => {
    const newErrors: Partial<Record<keyof LoaderRegistration, string>> = {};
    if (!form.name.trim()) newErrors.name = '请输入装载机名称';
    if (!form.model) newErrors.model = '请选择型号';
    if (form.bucket_capacity <= 0) newErrors.bucket_capacity = '铲斗容量必须大于0';
    if (form.max_speed <= 0) newErrors.max_speed = '移动速度必须大于0';
    if (form.bucket_width <= 0) newErrors.bucket_width = '铲斗宽度必须大于0';
    if (form.max_load_weight <= 0) newErrors.max_load_weight = '最大载重必须大于0';
    setErrors(newErrors);
    return Object.keys(newErrors).length === 0;
  };

  const handleRegister = async () => {
    if (!validate()) return;
    setLoading(true);
    try {
      registerDevice(form);
      message.success(`${form.name} 注册成功`);
      setOpen(false);
      resetForm();
    } catch {
      message.error('注册失败');
    } finally {
      setLoading(false);
    }
  };

  const resetForm = () => {
    setForm({ name: '', model: 'ZL-50', bucket_capacity: 0.85, max_speed: 0.5, bucket_width: 0.8, max_load_weight: 5.0 });
    setErrors({});
  };

  const updateField = <K extends keyof LoaderRegistration>(key: K, value: LoaderRegistration[K]) => {
    setForm((prev) => ({ ...prev, [key]: value }));
    if (errors[key]) setErrors((prev) => ({ ...prev, [key]: undefined }));
  };

  return (
    <>
      <button
        onClick={() => setOpen(true)}
        className="flex items-center gap-2 px-3 py-2 rounded-lg bg-cyan-500/15 text-cyan-400 border border-cyan-500/25 font-bold text-[12px] tracking-wider hover:bg-cyan-500/25 transition-colors"
      >
        <Plus className="w-4 h-4" />
        注册装载机
      </button>

      <TechModal
        title="注册装载机"
        open={open}
        onOk={handleRegister}
        onCancel={() => { setOpen(false); resetForm(); }}
        confirmLoading={loading}
        okText="确认注册"
        cancelText="取消"
      >
        <div className="space-y-4 py-2">
          <div>
            <label className="text-[12px] text-text-muted block mb-1 font-semibold">装载机名称 *</label>
            <Input
              value={form.name}
              onChange={(e) => updateField('name', e.target.value)}
              placeholder="例如：7号装载机"
              status={errors.name ? 'error' : undefined}
            />
            {errors.name && <span className="text-[11px] text-tech-red mt-1 block">{errors.name}</span>}
          </div>

          <div>
            <label className="text-[12px] text-text-muted block mb-1 font-semibold">设备型号 *</label>
            <Select
              value={form.model}
              onChange={(v) => updateField('model', v)}
              className="w-full"
              options={MODELS}
            />
            {errors.model && <span className="text-[11px] text-tech-red mt-1 block">{errors.model}</span>}
          </div>

          <div className="grid grid-cols-2 gap-3">
            <div>
              <label className="text-[12px] text-text-muted block mb-1 font-semibold">铲斗容量 (吨) *</label>
              <InputNumber
                value={form.bucket_capacity}
                onChange={(v) => v != null && updateField('bucket_capacity', v)}
                min={0.1}
                max={10}
                step={0.05}
                precision={2}
                className="w-full"
                status={errors.bucket_capacity ? 'error' : undefined}
              />
              {errors.bucket_capacity && <span className="text-[11px] text-tech-red mt-1 block">{errors.bucket_capacity}</span>}
            </div>
            <div>
              <label className="text-[12px] text-text-muted block mb-1 font-semibold">最大移动速度 (m/s) *</label>
              <InputNumber
                value={form.max_speed}
                onChange={(v) => v != null && updateField('max_speed', v)}
                min={0.05}
                max={5}
                step={0.05}
                precision={2}
                className="w-full"
                status={errors.max_speed ? 'error' : undefined}
              />
              {errors.max_speed && <span className="text-[11px] text-tech-red mt-1 block">{errors.max_speed}</span>}
            </div>
          </div>

          <div className="grid grid-cols-2 gap-3">
            <div>
              <label className="text-[12px] text-text-muted block mb-1 font-semibold">铲斗宽度 (m) *</label>
              <InputNumber
                value={form.bucket_width}
                onChange={(v) => v != null && updateField('bucket_width', v)}
                min={0.1}
                max={3}
                step={0.01}
                precision={2}
                className="w-full"
                status={errors.bucket_width ? 'error' : undefined}
              />
              {errors.bucket_width && <span className="text-[11px] text-tech-red mt-1 block">{errors.bucket_width}</span>}
            </div>
            <div>
              <label className="text-[12px] text-text-muted block mb-1 font-semibold">最大载重 (吨) *</label>
              <InputNumber
                value={form.max_load_weight}
                onChange={(v) => v != null && updateField('max_load_weight', v)}
                min={0.5}
                max={20}
                step={0.5}
                precision={1}
                className="w-full"
                status={errors.max_load_weight ? 'error' : undefined}
              />
              {errors.max_load_weight && <span className="text-[11px] text-tech-red mt-1 block">{errors.max_load_weight}</span>}
            </div>
          </div>

          <div className="p-3 rounded-md border border-cyan-500/20 bg-cyan-500/5">
            <div className="flex items-center gap-1.5 mb-2">
              <Truck className="w-4 h-4 text-cyan-400" />
              <span className="text-[13px] font-bold text-cyan-400">注册预览</span>
            </div>
            <div className="grid grid-cols-2 gap-2 text-[11px]">
              <div className="flex justify-between"><span className="text-text-muted">名称</span><span className="text-text-secondary">{form.name || '-'}</span></div>
              <div className="flex justify-between"><span className="text-text-muted">型号</span><span className="text-text-secondary">{form.model}</span></div>
              <div className="flex justify-between"><span className="text-text-muted">铲斗容量</span><span className="text-cyan-400 font-mono">{form.bucket_capacity} 吨</span></div>
              <div className="flex justify-between"><span className="text-text-muted">移动速度</span><span className="text-cyan-400 font-mono">{form.max_speed} m/s</span></div>
              <div className="flex justify-between"><span className="text-text-muted">铲斗宽度</span><span className="text-cyan-400 font-mono">{form.bucket_width} m</span></div>
              <div className="flex justify-between"><span className="text-text-muted">最大载重</span><span className="text-cyan-400 font-mono">{form.max_load_weight} 吨</span></div>
            </div>
          </div>
        </div>
      </TechModal>
    </>
  );
}
