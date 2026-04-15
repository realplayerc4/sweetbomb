import { useState, useMemo } from 'react';
import { EChartsWrapper } from '@/components/charts/EChartsWrapper';
import { cn } from '@/lib/utils';
import type { EChartsOption } from 'echarts';

type TimeDimension = 'today' | 'week' | 'month';

const MOCK_DATA: Record<TimeDimension, {
  taskTypes: { name: string; value: number; color: string }[];
  robotDistribution: { name: string; value: number; color: string }[];
  durationBuckets: { label: string; count: number }[];
  metrics: {
    totalTasks: number;
    completedTasks: number;
    avgDuration: number;
    loadBalance: number;
    interruptCount: number;
    avgPerRun: number;
  };
}> = {
  today: {
    taskTypes: [
      { name: '铲糖作业', value: 18, color: '#00d4ff' },
      { name: '导航任务', value: 5, color: '#52c41a' },
      { name: '巡检任务', value: 3, color: '#fa8c16' },
      { name: '充电任务', value: 2, color: '#f5222d' },
    ],
    robotDistribution: [
      { name: '1号机', value: 7, color: '#00d4ff' },
      { name: '2号机', value: 6, color: '#1677ff' },
      { name: '3号机', value: 6, color: '#52c41a' },
      { name: '4号机', value: 5, color: '#fa8c16' },
      { name: '5号机', value: 3, color: '#f5222d' },
      { name: '6号机', value: 1, color: '#9254de' },
    ],
    durationBuckets: [
      { label: '<5min', count: 8 },
      { label: '5-15min', count: 12 },
      { label: '15-30min', count: 5 },
      { label: '>30min', count: 3 },
    ],
    metrics: {
      totalTasks: 28,
      completedTasks: 24,
      avgDuration: 12.5,
      loadBalance: 87,
      interruptCount: 2,
      avgPerRun: 0.85,
    },
  },
  week: {
    taskTypes: [
      { name: '铲糖作业', value: 95, color: '#00d4ff' },
      { name: '导航任务', value: 28, color: '#52c41a' },
      { name: '巡检任务', value: 21, color: '#fa8c16' },
      { name: '充电任务', value: 14, color: '#f5222d' },
    ],
    robotDistribution: [
      { name: '1号机', value: 42, color: '#00d4ff' },
      { name: '2号机', value: 38, color: '#1677ff' },
      { name: '3号机', value: 35, color: '#52c41a' },
      { name: '4号机', value: 22, color: '#fa8c16' },
      { name: '5号机', value: 13, color: '#f5222d' },
      { name: '6号机', value: 8, color: '#9254de' },
    ],
    durationBuckets: [
      { label: '<5min', count: 42 },
      { label: '5-15min', count: 68 },
      { label: '15-30min', count: 32 },
      { label: '>30min', count: 16 },
    ],
    metrics: {
      totalTasks: 158,
      completedTasks: 142,
      avgDuration: 14.2,
      loadBalance: 82,
      interruptCount: 11,
      avgPerRun: 0.92,
    },
  },
  month: {
    taskTypes: [
      { name: '铲糖作业', value: 380, color: '#00d4ff' },
      { name: '导航任务', value: 112, color: '#52c41a' },
      { name: '巡检任务', value: 85, color: '#fa8c16' },
      { name: '充电任务', value: 56, color: '#f5222d' },
    ],
    robotDistribution: [
      { name: '1号机', value: 168, color: '#00d4ff' },
      { name: '2号机', value: 152, color: '#1677ff' },
      { name: '3号机', value: 140, color: '#52c41a' },
      { name: '4号机', value: 88, color: '#fa8c16' },
      { name: '5号机', value: 52, color: '#f5222d' },
      { name: '6号机', value: 33, color: '#9254de' },
    ],
    durationBuckets: [
      { label: '<5min', count: 168 },
      { label: '5-15min', count: 272 },
      { label: '15-30min', count: 128 },
      { label: '>30min', count: 65 },
    ],
    metrics: {
      totalTasks: 633,
      completedTasks: 589,
      avgDuration: 13.8,
      loadBalance: 79,
      interruptCount: 44,
      avgPerRun: 0.89,
    },
  },
};

export function TaskStatistics() {
  const [dimension, setDimension] = useState<TimeDimension>('today');
  const [chartTab, setChartTab] = useState<'type' | 'robot'>('type');
  const data = MOCK_DATA[dimension];

  const pieOption = useMemo<EChartsOption>(() => ({
    backgroundColor: 'transparent',
    tooltip: {
      trigger: 'item',
      backgroundColor: 'rgba(13,27,54,0.95)',
      borderColor: 'rgba(0,212,255,0.3)',
      textStyle: { color: '#cbd5e1', fontSize: 12 },
      formatter: '{b}: {c} ({d}%)',
    },
    legend: {
      orient: 'vertical',
      left: 'left',
      top: 'middle',
      textStyle: { color: '#64748b', fontSize: 10 },
      itemWidth: 10,
      itemHeight: 8,
      itemGap: 8,
    },
    series: [{
      type: 'pie',
      radius: ['40%', '70%'],
      center: ['55%', '50%'],
      avoidLabelOverlap: true,
      itemStyle: { borderRadius: 4, borderColor: '#0a1428', borderWidth: 2 },
      label: { show: false },
      emphasis: {
        label: { show: true, fontSize: 12, fontWeight: 'bold', color: '#fff' },
        itemStyle: { shadowBlur: 10, shadowColor: 'rgba(0,0,0,0.3)' },
      },
      data: chartTab === 'type' ? data.taskTypes : data.robotDistribution,
    }],
  }), [data, chartTab]);

  const barOption = useMemo<EChartsOption>(() => ({
    backgroundColor: 'transparent',
    tooltip: {
      trigger: 'axis',
      backgroundColor: 'rgba(13,27,54,0.95)',
      borderColor: 'rgba(0,212,255,0.3)',
      textStyle: { color: '#cbd5e1', fontSize: 11 },
    },
    grid: { left: 36, right: 16, top: 12, bottom: 24 },
    xAxis: {
      type: 'category',
      data: data.durationBuckets.map((b) => b.label),
      axisLine: { lineStyle: { color: 'rgba(0,212,255,0.15)' } },
      axisLabel: { color: '#64748b', fontSize: 10 },
    },
    yAxis: {
      type: 'value',
      axisLine: { show: false },
      splitLine: { lineStyle: { color: 'rgba(0,212,255,0.06)' } },
      axisLabel: { color: '#64748b', fontSize: 10 },
    },
    series: [{
      type: 'bar',
      data: data.durationBuckets.map((b) => b.count),
      barWidth: '50%',
      itemStyle: {
        borderRadius: [3, 3, 0, 0],
        color: {
          type: 'linear',
          x: 0, y: 0, x2: 0, y2: 1,
          colorStops: [
            { offset: 0, color: '#00d4ff' },
            { offset: 1, color: 'rgba(0,212,255,0.2)' },
          ],
        },
      },
    }],
  }), [data]);

  const m = data.metrics;

  return (
    <div className="h-full flex flex-col">
      <div className="flex items-center justify-between mb-2">
        <div className="flex items-center gap-1 bg-bg-primary/60 border border-cyan-500/20 rounded p-0.5">
          {([['today', '本日'], ['week', '本周'], ['month', '本月']] as const).map(([key, label]) => (
            <button
              key={key}
              onClick={() => setDimension(key)}
              className={cn(
                'px-2 py-1 rounded text-[11px] font-bold tracking-wider transition-all',
                dimension === key ? 'bg-cyan-500/20 text-cyan-400' : 'text-text-muted hover:text-text-secondary'
              )}
            >
              {label}
            </button>
          ))}
        </div>
        <div className="flex items-center gap-1 bg-bg-primary/60 border border-cyan-500/20 rounded p-0.5">
          <button
            onClick={() => setChartTab('type')}
            className={cn('px-2 py-1 rounded text-[10px] font-bold transition-all', chartTab === 'type' ? 'bg-cyan-500/20 text-cyan-400' : 'text-text-muted hover:text-text-secondary')}
          >
            任务类型
          </button>
          <button
            onClick={() => setChartTab('robot')}
            className={cn('px-2 py-1 rounded text-[10px] font-bold transition-all', chartTab === 'robot' ? 'bg-cyan-500/20 text-cyan-400' : 'text-text-muted hover:text-text-secondary')}
          >
            机器人分配
          </button>
        </div>
      </div>

      <div className="grid grid-cols-6 gap-1.5 mb-2">
        <StatBox label="任务总量" value={String(m.totalTasks)} unit="个" />
        <StatBox label="已完成" value={String(m.completedTasks)} unit="个" highlight="green" />
        <StatBox label="平均时长" value={m.avgDuration.toFixed(1)} unit="min" />
        <StatBox label="负载均衡" value={m.loadBalance.toFixed(0)} unit="%" highlight="cyan" />
        <StatBox label="中断次数" value={String(m.interruptCount)} unit="次" highlight={m.interruptCount > 5 ? 'red' : undefined} />
        <StatBox label="平均单次" value={m.avgPerRun.toFixed(2)} unit="吨" />
      </div>

      <div className="grid grid-cols-2 gap-2 flex-1 min-h-0">
        <div className="tech-card p-2">
          <div className="text-[10px] text-text-muted font-semibold mb-1">{chartTab === 'type' ? '任务类型占比' : '机器人分配比例'}</div>
          <EChartsWrapper option={pieOption} />
        </div>
        <div className="tech-card p-2">
          <div className="text-[10px] text-text-muted font-semibold mb-1">执行时长分布</div>
          <EChartsWrapper option={barOption} />
        </div>
      </div>
    </div>
  );
}

function StatBox({ label, value, unit, highlight }: { label: string; value: string; unit?: string; highlight?: 'green' | 'red' | 'cyan' }) {
  const colorMap = { green: 'text-tech-green', red: 'text-tech-red', cyan: 'text-cyan-400' };
  const color = highlight ? (colorMap[highlight] || 'text-cyan-400') : 'text-text-primary';
  return (
    <div className="tech-card p-1.5 text-center">
      <div className="text-[9px] text-text-muted">{label}</div>
      <div className={cn('text-[14px] font-bold font-mono tech-glow-number', color)}>{value}</div>
      <div className="text-[9px] text-text-muted">{unit}</div>
    </div>
  );
}
