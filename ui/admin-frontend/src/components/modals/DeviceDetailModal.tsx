import { useState } from 'react';
import { Tabs, Tag, Descriptions, Progress, Button, Table, message } from 'antd';
import { TechModal } from './TechModal';
import { useSystemStore } from '@/stores/useSystemStore';
import { getStatusColor, getStatusLabel, formatDateTime, cn } from '@/lib/utils';
import { Battery, Clock, Wrench, Zap, Thermometer, Activity, Truck } from 'lucide-react';
import type { DeviceInfo, MaintenanceRecord } from '@/types';

interface DeviceDetailModalProps {
  device: DeviceInfo | null;
  open: boolean;
  onClose: () => void;
}

export function DeviceDetailModal({ device, open, onClose }: DeviceDetailModalProps) {
  const maintenanceRecords = useSystemStore((s) => s.maintenanceRecords);
  const addMaintenanceRecord = useSystemStore((s) => s.addMaintenanceRecord);

  if (!device) return null;

  const deviceMaintenance = maintenanceRecords.filter((r) => r.device_id === device.device_id);

  const batteryColor = device.battery > 60 ? '#52c41a' : device.battery > 30 ? '#fa8c16' : '#f5222d';
  const workStateMap: Record<string, { label: string; color: string }> = {
    idle: { label: '空闲', color: '#1677ff' },
    working: { label: '作业中', color: '#52c41a' },
    fault: { label: '故障', color: '#f5222d' },
    charging: { label: '充电中', color: '#fa8c16' },
  };
  const ws = workStateMap[device.work_state] || { label: device.work_state, color: '#64748b' };

  const maintenanceColumns = [
    { title: '类型', dataIndex: 'type', key: 'type', width: 80, render: (v: string) => {
      const typeMap: Record<string, { label: string; color: string }> = {
        routine: { label: '日常保养', color: '#1677ff' },
        repair: { label: '故障维修', color: '#f5222d' },
        inspection: { label: '巡检', color: '#52c41a' },
      };
      const t = typeMap[v] || { label: v, color: '#64748b' };
      return <Tag color={t.color} bordered={false}>{t.label}</Tag>;
    }},
    { title: '描述', dataIndex: 'description', key: 'description' },
    { title: '操作人', dataIndex: 'operator', key: 'operator', width: 80 },
    { title: '日期', dataIndex: 'date', key: 'date', width: 120, render: (v: string) => formatDateTime(v) },
    { title: '费用', dataIndex: 'cost', key: 'cost', width: 80, render: (v: number) => v ? `¥${v}` : '-' },
  ];

  return (
    <TechModal
      title={`设备详情 - ${device.name}`}
      open={open}
      onCancel={onClose}
      footer={null}
      width={640}
    >
      <div className="py-2">
        <Tabs
          defaultActiveKey="status"
          items={[
            {
              key: 'status',
              label: '设备状态',
              children: (
                <div className="space-y-4">
                  <div className="grid grid-cols-3 gap-3">
                    <div className="tech-card p-3 text-center">
                      <Battery className={cn('w-5 h-5 mx-auto mb-1')} style={{ color: batteryColor }} />
                      <div className="text-lg font-bold font-mono" style={{ color: batteryColor }}>{device.battery}%</div>
                      <div className="text-[10px] text-text-muted">电量</div>
                    </div>
                    <div className="tech-card p-3 text-center">
                      <Activity className="w-5 h-5 mx-auto mb-1" style={{ color: ws.color }} />
                      <div className="text-lg font-bold" style={{ color: ws.color }}>{ws.label}</div>
                      <div className="text-[10px] text-text-muted">工作状态</div>
                    </div>
                    <div className="tech-card p-3 text-center">
                      <Zap className="w-5 h-5 mx-auto mb-1 text-tech-orange" />
                      <div className="text-lg font-bold font-mono text-tech-orange">{device.energy_consumption ?? 0} kWh</div>
                      <div className="text-[10px] text-text-muted">能耗</div>
                    </div>
                  </div>

                  <div className="tech-card p-3 space-y-2">
                    <div className="flex justify-between text-[13px]">
                      <span className="text-text-muted">设备ID</span>
                      <span className="text-text-primary font-mono">{device.device_id}</span>
                    </div>
                    <div className="flex justify-between text-[13px]">
                      <span className="text-text-muted">连接状态</span>
                      <span style={{ color: getStatusColor(device.status) }}>{getStatusLabel(device.status)}</span>
                    </div>
                    <div className="flex justify-between text-[13px]">
                      <span className="text-text-muted">当前任务</span>
                      <span className="text-text-primary font-mono">{device.task_id || '无'}</span>
                    </div>
                    <div className="flex justify-between text-[13px]">
                      <span className="text-text-muted">累计工时</span>
                      <span className="text-cyan-400 font-mono">{device.total_hours ?? 0} h</span>
                    </div>
                    <div className="flex justify-between text-[13px]">
                      <span className="text-text-muted">最后心跳</span>
                      <span className="text-text-secondary font-mono text-[11px]">{formatDateTime(device.last_heartbeat)}</span>
                    </div>
                  </div>
                </div>
              ),
            },
            {
              key: 'spec',
              label: '设备参数',
              children: device.spec ? (
                <div className="space-y-3">
                  <div className="tech-card p-3 space-y-2">
                    <div className="flex justify-between text-[13px]">
                      <span className="text-text-muted">型号</span>
                      <span className="text-text-primary font-bold">{device.spec.model}</span>
                    </div>
                    <div className="flex justify-between text-[13px]">
                      <span className="text-text-muted">铲斗容量</span>
                      <span className="text-cyan-400 font-mono">{device.spec.bucket_capacity} 吨</span>
                    </div>
                    <div className="flex justify-between text-[13px]">
                      <span className="text-text-muted">最大速度</span>
                      <span className="text-cyan-400 font-mono">{device.spec.max_speed} m/s</span>
                    </div>
                    <div className="flex justify-between text-[13px]">
                      <span className="text-text-muted">铲斗宽度</span>
                      <span className="text-cyan-400 font-mono">{device.spec.bucket_width} m</span>
                    </div>
                    <div className="flex justify-between text-[13px]">
                      <span className="text-text-muted">最大载重</span>
                      <span className="text-cyan-400 font-mono">{device.spec.max_load_weight} 吨</span>
                    </div>
                  </div>
                </div>
              ) : (
                <div className="text-center py-8 text-text-muted text-[13px]">暂无设备参数</div>
              ),
            },
            {
              key: 'maintenance',
              label: '维保记录',
              children: (
                <div>
                  <div className="flex justify-between items-center mb-3">
                    <span className="text-[12px] text-text-muted">共 {deviceMaintenance.length} 条记录</span>
                    <Button
                      type="primary"
                      size="small"
                      onClick={() => {
                        addMaintenanceRecord({
                          device_id: device.device_id,
                          type: 'routine',
                          description: '定期巡检',
                          date: new Date().toISOString(),
                          operator: '系统',
                        });
                        message.success('已添加巡检记录');
                      }}
                    >
                      添加记录
                    </Button>
                  </div>
                  <Table
                    dataSource={deviceMaintenance}
                    columns={maintenanceColumns}
                    rowKey="id"
                    size="small"
                    pagination={false}
                    scroll={{ y: 200 }}
                  />
                </div>
              ),
            },
          ]}
        />
      </div>
    </TechModal>
  );
}
