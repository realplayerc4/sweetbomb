import { useState } from 'react';
import { Select, Tabs, Tag, Table, Button, message } from 'antd';
import { TechModal } from './TechModal';
import { useSystemStore } from '@/stores/useSystemStore';
import { useTaskStore } from '@/stores/useTaskStore';
import { EChartsWrapper } from '@/components/charts/EChartsWrapper';
import { FileText, Download, TrendingUp } from 'lucide-react';
import type { EChartsOption } from 'echarts';
import { useMemo } from 'react';
import { cn } from '@/lib/utils';

type ReportType = 'daily' | 'weekly' | 'monthly';

const REPORT_DATA: Record<ReportType, {
  summary: { totalTon: number; taskCount: number; avgEfficiency: number; avgDuration: number; completionRate: number; alertCount: number };
  trend: { dates: string[]; tons: number[]; tasks: number[] };
  deviceRanking: { name: string; ton: number; tasks: number; hours: number }[];
}> = {
  daily: {
    summary: { totalTon: 15.8, taskCount: 28, avgEfficiency: 2.1, avgDuration: 12.5, completionRate: 85.7, alertCount: 2 },
    trend: {
      dates: ['06:00', '08:00', '10:00', '12:00', '14:00', '16:00', '18:00'],
      tons: [0.5, 2.1, 3.5, 2.8, 3.2, 2.5, 1.2],
      tasks: [1, 4, 6, 5, 5, 4, 3],
    },
    deviceRanking: [
      { name: '1号机', ton: 4.2, tasks: 7, hours: 6.5 },
      { name: '2号机', ton: 3.8, tasks: 6, hours: 5.8 },
      { name: '3号机', ton: 3.5, tasks: 6, hours: 5.2 },
      { name: '6号机', ton: 2.8, tasks: 5, hours: 4.5 },
      { name: '4号机', ton: 1.5, tasks: 4, hours: 3.0 },
    ],
  },
  weekly: {
    summary: { totalTon: 98.5, taskCount: 158, avgEfficiency: 1.95, avgDuration: 14.2, completionRate: 89.9, alertCount: 11 },
    trend: {
      dates: ['周一', '周二', '周三', '周四', '周五', '周六', '周日'],
      tons: [14.2, 15.8, 16.1, 14.5, 13.8, 12.5, 11.6],
      tasks: [22, 25, 26, 23, 22, 20, 20],
    },
    deviceRanking: [
      { name: '1号机', ton: 22.5, tasks: 42, hours: 38 },
      { name: '2号机', ton: 20.8, tasks: 38, hours: 35 },
      { name: '3号机', ton: 19.2, tasks: 35, hours: 32 },
      { name: '6号机', ton: 18.5, tasks: 22, hours: 28 },
      { name: '4号机', ton: 10.2, tasks: 13, hours: 15 },
    ],
  },
  monthly: {
    summary: { totalTon: 420.5, taskCount: 633, avgEfficiency: 1.88, avgDuration: 13.8, completionRate: 93.0, alertCount: 44 },
    trend: {
      dates: ['第1周', '第2周', '第3周', '第4周'],
      tons: [98.5, 105.2, 112.8, 104.0],
      tasks: [158, 165, 172, 138],
    },
    deviceRanking: [
      { name: '1号机', ton: 85.2, tasks: 168, hours: 152 },
      { name: '2号机', ton: 78.5, tasks: 152, hours: 140 },
      { name: '3号机', ton: 72.8, tasks: 140, hours: 128 },
      { name: '6号机', ton: 68.5, tasks: 88, hours: 112 },
      { name: '4号机', ton: 45.2, tasks: 52, hours: 65 },
    ],
  },
};

export function ReportModal() {
  const [open, setOpen] = useState(false);
  const [reportType, setReportType] = useState<ReportType>('daily');

  const data = REPORT_DATA[reportType];
  const s = data.summary;

  const trendOption = useMemo<EChartsOption>(() => ({
    backgroundColor: 'transparent',
    tooltip: {
      trigger: 'axis',
      backgroundColor: 'rgba(13,27,54,0.95)',
      borderColor: 'rgba(0,212,255,0.3)',
      textStyle: { color: '#cbd5e1', fontSize: 11 },
    },
    legend: {
      textStyle: { color: '#64748b', fontSize: 10 },
      top: 0,
    },
    grid: { left: 40, right: 40, top: 30, bottom: 24 },
    xAxis: {
      type: 'category',
      data: data.trend.dates,
      axisLine: { lineStyle: { color: 'rgba(0,212,255,0.15)' } },
      axisLabel: { color: '#64748b', fontSize: 10 },
    },
    yAxis: [
      {
        type: 'value',
        name: '吨',
        axisLine: { show: false },
        splitLine: { lineStyle: { color: 'rgba(0,212,255,0.06)' } },
        axisLabel: { color: '#64748b', fontSize: 10 },
      },
      {
        type: 'value',
        name: '任务数',
        axisLine: { show: false },
        splitLine: { show: false },
        axisLabel: { color: '#64748b', fontSize: 10 },
      },
    ],
    series: [
      {
        name: '铲糖量(吨)',
        type: 'bar',
        data: data.trend.tons,
        barWidth: '40%',
        itemStyle: {
          borderRadius: [3, 3, 0, 0],
          color: { type: 'linear', x: 0, y: 0, x2: 0, y2: 1, colorStops: [{ offset: 0, color: '#00d4ff' }, { offset: 1, color: 'rgba(0,212,255,0.2)' }] },
        },
      },
      {
        name: '任务数',
        type: 'line',
        yAxisIndex: 1,
        data: data.trend.tasks,
        smooth: true,
        lineStyle: { color: '#52c41a', width: 2 },
        itemStyle: { color: '#52c41a' },
      },
    ],
  }), [data]);

  const rankingColumns = [
    { title: '排名', key: 'rank', width: 50, render: (_: unknown, __: unknown, idx: number) => (
      <span className={cn('font-mono font-bold', idx < 3 ? 'text-cyan-400' : 'text-text-muted')}>{idx + 1}</span>
    )},
    { title: '设备', dataIndex: 'name', key: 'name' },
    { title: '铲糖量(吨)', dataIndex: 'ton', key: 'ton', render: (v: number) => <span className="text-cyan-400 font-mono">{v}</span> },
    { title: '任务数', dataIndex: 'tasks', key: 'tasks', render: (v: number) => <span className="font-mono">{v}</span> },
    { title: '工时(h)', dataIndex: 'hours', key: 'hours', render: (v: number) => <span className="font-mono">{v}</span> },
  ];

  return (
    <>
      <button
        onClick={() => setOpen(true)}
        className="flex items-center gap-1.5 px-2 py-1.5 rounded-md bg-bg-primary/60 border border-cyan-500/20 text-[11px] text-text-secondary hover:text-cyan-400 hover:border-cyan-500/40 transition-colors"
      >
        <FileText className="w-3 h-3" />
        数据报表
      </button>

      <TechModal
        title="数据统计与报表"
        open={open}
        onCancel={() => setOpen(false)}
        footer={null}
        width={700}
      >
        <div className="py-2">
          <div className="flex items-center justify-between mb-4">
            <div className="flex items-center gap-1 bg-bg-primary/60 border border-cyan-500/20 rounded p-0.5">
              {([['daily', '日报'], ['weekly', '周报'], ['monthly', '月报']] as const).map(([key, label]) => (
                <button
                  key={key}
                  onClick={() => setReportType(key)}
                  className={cn(
                    'px-3 py-1.5 rounded text-[12px] font-bold tracking-wider transition-all',
                    reportType === key ? 'bg-cyan-500/20 text-cyan-400' : 'text-text-muted hover:text-text-secondary'
                  )}
                >
                  {label}
                </button>
              ))}
            </div>
            <Button icon={<Download className="w-3.5 h-3.5" />} size="small">
              导出报表
            </Button>
          </div>

          <div className="grid grid-cols-3 gap-3 mb-4">
            <div className="tech-card p-3 text-center">
              <div className="text-[10px] text-text-muted">总铲糖量</div>
              <div className="text-lg font-bold text-cyan-400 font-mono tech-glow-number">{s.totalTon}</div>
              <div className="text-[10px] text-text-muted">吨</div>
            </div>
            <div className="tech-card p-3 text-center">
              <div className="text-[10px] text-text-muted">任务总量</div>
              <div className="text-lg font-bold text-tech-green font-mono">{s.taskCount}</div>
              <div className="text-[10px] text-text-muted">个</div>
            </div>
            <div className="tech-card p-3 text-center">
              <div className="text-[10px] text-text-muted">完成率</div>
              <div className="text-lg font-bold text-cyan-400 font-mono tech-glow-number">{s.completionRate}%</div>
              <div className="text-[10px] text-text-muted">同比+5.2%</div>
            </div>
          </div>

          <div className="grid grid-cols-3 gap-3 mb-4">
            <div className="tech-card p-2.5 text-center">
              <div className="text-[10px] text-text-muted">平均效率</div>
              <div className="text-[14px] font-bold text-tech-orange font-mono">{s.avgEfficiency} 吨/h</div>
            </div>
            <div className="tech-card p-2.5 text-center">
              <div className="text-[10px] text-text-muted">平均时长</div>
              <div className="text-[14px] font-bold text-text-primary font-mono">{s.avgDuration} min</div>
            </div>
            <div className="tech-card p-2.5 text-center">
              <div className="text-[10px] text-text-muted">告警次数</div>
              <div className="text-[14px] font-bold text-tech-red font-mono">{s.alertCount}</div>
            </div>
          </div>

          <div className="tech-card p-3 mb-4">
            <div className="text-[11px] text-text-muted font-semibold mb-2">产量趋势</div>
            <div style={{ height: 200 }}>
              <EChartsWrapper option={trendOption} />
            </div>
          </div>

          <div className="tech-card p-3">
            <div className="text-[11px] text-text-muted font-semibold mb-2">设备排名</div>
            <Table
              dataSource={data.deviceRanking}
              columns={rankingColumns}
              rowKey="name"
              size="small"
              pagination={false}
            />
          </div>
        </div>
      </TechModal>
    </>
  );
}
