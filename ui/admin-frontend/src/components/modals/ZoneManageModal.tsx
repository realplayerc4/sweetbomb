import { useState } from 'react';
import { Select, Switch, Input, Button, message, Tag } from 'antd';
import { TechModal } from './TechModal';
import { useSystemStore } from '@/stores/useSystemStore';
import { Plus, Trash2, MapPin, Shield, Edit3 } from 'lucide-react';
import { cn } from '@/lib/utils';
import type { ZoneInfo, ZoneType } from '@/types';

const ZONE_TYPES: { value: ZoneType; label: string; color: string }[] = [
  { value: 'sugar_pile', label: '糖堆存放区', color: '#52c41a' },
  { value: 'loading', label: '装卸作业区', color: '#1677ff' },
  { value: 'unloading', label: '卸载区', color: '#00d4ff' },
  { value: 'charging', label: '充电维护区', color: '#fa8c16' },
  { value: 'restricted', label: '禁区', color: '#f5222d' },
  { value: 'fence', label: '电子围栏', color: '#9254de' },
];

export function ZoneManageModal() {
  const [open, setOpen] = useState(false);
  const zones = useSystemStore((s) => s.zones);
  const addZone = useSystemStore((s) => s.addZone);
  const updateZone = useSystemStore((s) => s.updateZone);
  const removeZone = useSystemStore((s) => s.removeZone);

  const [newZoneName, setNewZoneName] = useState('');
  const [newZoneType, setNewZoneType] = useState<ZoneType>('sugar_pile');
  const [editingId, setEditingId] = useState<string | null>(null);

  const handleAddZone = () => {
    if (!newZoneName.trim()) {
      message.warning('请输入区域名称');
      return;
    }
    const typeInfo = ZONE_TYPES.find((t) => t.value === newZoneType);
    addZone({
      name: newZoneName,
      type: newZoneType,
      bounds: [[22.5430, 108.3740], [22.5440, 108.3750]],
      color: typeInfo?.color || '#64748b',
      enabled: true,
    });
    setNewZoneName('');
    message.success('区域已添加');
  };

  const handleRemoveZone = (id: string) => {
    removeZone(id);
    message.success('区域已删除');
  };

  const handleToggleZone = (id: string, enabled: boolean) => {
    updateZone(id, { enabled });
  };

  return (
    <>
      <button
        onClick={() => setOpen(true)}
        className="flex items-center gap-1.5 px-2 py-1.5 rounded-md bg-bg-primary/60 border border-cyan-500/20 text-[11px] text-text-secondary hover:text-cyan-400 hover:border-cyan-500/40 transition-colors"
      >
        <MapPin className="w-3 h-3" />
        区域管理
      </button>

      <TechModal
        title="料场与作业区域管理"
        open={open}
        onCancel={() => setOpen(false)}
        footer={null}
        width={620}
      >
        <div className="space-y-4 py-2">
          <div className="p-3 rounded-md border border-cyan-500/20 bg-cyan-500/5">
            <div className="text-[12px] font-bold text-cyan-400 mb-2">添加新区域</div>
            <div className="flex gap-2">
              <Input
                value={newZoneName}
                onChange={(e) => setNewZoneName(e.target.value)}
                placeholder="区域名称"
                className="flex-1"
              />
              <Select
                value={newZoneType}
                onChange={setNewZoneType}
                className="w-36"
                options={ZONE_TYPES.map((t) => ({ value: t.value, label: t.label }))}
              />
              <Button type="primary" icon={<Plus className="w-3.5 h-3.5" />} onClick={handleAddZone}>
                添加
              </Button>
            </div>
          </div>

          <div className="space-y-2">
            <div className="text-[12px] font-bold text-text-muted">现有区域 ({zones.length})</div>
            {zones.map((zone) => {
              const typeInfo = ZONE_TYPES.find((t) => t.value === zone.type);
              return (
                <div
                  key={zone.id}
                  className={cn(
                    'tech-card p-3 flex items-center gap-3 transition-all',
                    !zone.enabled && 'opacity-50'
                  )}
                >
                  <div
                    className="w-3 h-3 rounded-sm shrink-0"
                    style={{ background: zone.color, boxShadow: `0 0 6px ${zone.color}60` }}
                  />
                  <div className="flex-1 min-w-0">
                    <div className="flex items-center gap-2">
                      <span className="text-[13px] font-semibold text-text-primary truncate">{zone.name}</span>
                      <Tag color={zone.color} bordered={false} className="text-[10px]">
                        {typeInfo?.label || zone.type}
                      </Tag>
                    </div>
                  </div>
                  <div className="flex items-center gap-2">
                    <Switch
                      size="small"
                      checked={zone.enabled}
                      onChange={(v) => handleToggleZone(zone.id, v)}
                    />
                    <button
                      onClick={() => handleRemoveZone(zone.id)}
                      className="p-1 rounded hover:bg-white/10 text-text-muted hover:text-tech-red transition-colors"
                    >
                      <Trash2 className="w-3.5 h-3.5" />
                    </button>
                  </div>
                </div>
              );
            })}
            {zones.length === 0 && (
              <div className="text-center py-6 text-text-muted text-[13px]">暂无区域配置</div>
            )}
          </div>

          <div className="p-3 rounded-md border border-tech-red/20 bg-tech-red/5">
            <div className="flex items-center gap-1.5 mb-1">
              <Shield className="w-3.5 h-3.5 text-tech-red" />
              <span className="text-[12px] font-bold text-tech-red">禁区与电子围栏</span>
            </div>
            <span className="text-[11px] text-text-secondary">设置禁区后，装载机将自动规避该区域。电子围栏可限制装载机活动范围。</span>
          </div>
        </div>
      </TechModal>
    </>
  );
}
