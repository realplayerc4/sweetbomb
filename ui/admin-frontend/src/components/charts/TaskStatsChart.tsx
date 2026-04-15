import { useMemo } from 'react';
import { EChartsWrapper } from './EChartsWrapper';
import { useTaskStore } from '@/stores/useTaskStore';
import type { EChartsOption } from 'echarts';

export function TaskStatsChart() {
  const tasks = useTaskStore((s) => s.tasks);

  const option = useMemo<EChartsOption>(() => {
    const statusCounts: Record<string, number> = {
      pending: 0,
      running: 0,
      paused: 0,
      completed: 0,
      failed: 0,
      stopped: 0,
    };
    tasks.forEach((t) => {
      if (statusCounts[t.status] !== undefined) statusCounts[t.status]++;
    });

    return {
      backgroundColor: 'transparent',
      tooltip: {
        trigger: 'axis',
        backgroundColor: 'rgba(13,27,54,0.95)',
        borderColor: 'rgba(22,119,255,0.3)',
        textStyle: { color: '#cbd5e1', fontSize: 11 },
      },
      grid: { left: 40, right: 16, top: 16, bottom: 30 },
      xAxis: {
        type: 'category',
        data: ['待执行', '运行中', '已暂停', '已完成', '失败', '已停止'],
        axisLine: { lineStyle: { color: 'rgba(22,119,255,0.2)' } },
        axisLabel: { color: '#64748b', fontSize: 10 },
      },
      yAxis: {
        type: 'value',
        axisLine: { show: false },
        splitLine: { lineStyle: { color: 'rgba(22,119,255,0.08)' } },
        axisLabel: { color: '#64748b', fontSize: 10 },
      },
      series: [
        {
          type: 'bar',
          data: [
            { value: statusCounts.pending, itemStyle: { color: '#fa8c16' } },
            { value: statusCounts.running, itemStyle: { color: '#1677ff' } },
            { value: statusCounts.paused, itemStyle: { color: '#fa8c16', opacity: 0.6 } },
            { value: statusCounts.completed, itemStyle: { color: '#52c41a' } },
            { value: statusCounts.failed, itemStyle: { color: '#f5222d' } },
            { value: statusCounts.stopped, itemStyle: { color: '#64748b' } },
          ],
          barWidth: '50%',
          itemStyle: { borderRadius: [3, 3, 0, 0] },
        },
      ],
    };
  }, [tasks]);

  return <EChartsWrapper option={option} />;
}
