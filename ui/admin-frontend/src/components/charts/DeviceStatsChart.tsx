import { useMemo } from 'react';
import { EChartsWrapper } from './EChartsWrapper';
import { useSystemStore } from '@/stores/useSystemStore';
import type { EChartsOption } from 'echarts';

export function DeviceStatsChart() {
  const devices = useSystemStore((s) => s.devices);

  const option = useMemo<EChartsOption>(() => {
    const statusCounts: Record<string, number> = { online: 0, offline: 0, warning: 0, error: 0 };
    devices.forEach((d) => {
      if (statusCounts[d.status] !== undefined) statusCounts[d.status]++;
    });

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
          radius: ['45%', '70%'],
          center: ['50%', '50%'],
          avoidLabelOverlap: false,
          itemStyle: { borderRadius: 4, borderColor: '#0a1428', borderWidth: 2 },
          label: {
            show: true,
            color: '#cbd5e1',
            fontSize: 10,
            formatter: '{b}\n{c}台',
          },
          labelLine: { lineStyle: { color: 'rgba(22,119,255,0.3)' } },
          data: [
            { value: statusCounts.online, name: '在线', itemStyle: { color: '#52c41a' } },
            { value: statusCounts.warning, name: '告警', itemStyle: { color: '#fa8c16' } },
            { value: statusCounts.offline, name: '离线', itemStyle: { color: '#64748b' } },
            { value: statusCounts.error, name: '故障', itemStyle: { color: '#f5222d' } },
          ],
        },
      ],
    };
  }, [devices]);

  return <EChartsWrapper option={option} />;
}
