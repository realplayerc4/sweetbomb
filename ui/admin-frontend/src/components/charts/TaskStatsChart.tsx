import { useMemo } from 'react';
import { EChartsWrapper } from './EChartsWrapper';
import { useTaskStore } from '@/stores/useTaskStore';
import type { EChartsOption } from 'echarts';

export function TaskStatsChart() {
  const currentCycle = useTaskStore((s) => s.currentCycle);
  const totalCycles = useTaskStore((s) => s.totalCycles);
  const completedKg = useTaskStore((s) => s.completedKg);
  const targetKg = useTaskStore((s) => s.targetKg);

  const option = useMemo<EChartsOption>(() => {
    const percentage = totalCycles > 0 ? Math.round((currentCycle / totalCycles) * 100) : 0;
    const remaining = targetKg - completedKg;

    return {
      backgroundColor: 'transparent',
      tooltip: {
        trigger: 'item',
        backgroundColor: 'rgba(13,27,54,0.95)',
        borderColor: 'rgba(22,119,255,0.3)',
        textStyle: { color: '#cbd5e1', fontSize: 11 },
      },
      series: [
        {
          type: 'pie',
          radius: ['60%', '80%'],
          center: ['50%', '50%'],
          avoidLabelOverlap: false,
          label: {
            show: true,
            position: 'center',
            formatter: `{value|${percentage}%}\n{label|完成}`,
            rich: {
              value: { color: '#00d4ff', fontSize: 18, fontWeight: 'bold' },
              label: { color: '#64748b', fontSize: 11, padding: [4, 0, 0, 0] },
            },
          },
          labelLine: { show: false },
          data: [
            {
              value: completedKg,
              name: '已完成',
              itemStyle: { color: '#52c41a' }
            },
            {
              value: remaining,
              name: '剩余',
              itemStyle: { color: 'rgba(22,119,255,0.3)' }
            },
          ],
        },
      ],
    };
  }, [currentCycle, totalCycles, completedKg, targetKg]);

  return <EChartsWrapper option={option} />;
}