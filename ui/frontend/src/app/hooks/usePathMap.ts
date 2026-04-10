/** usePathMap Hook - 按需加载 pathMap 站点数据 */

import { useState, useCallback } from 'react';
import {
  getPathMap,
  updatePickStation as apiUpdatePickStation,
  type PathMapData,
  type PathNode,
  type PickStation,
  type DropStation,
} from '../services/pathMapApi';

export function usePathMap() {
  const [nodes, setNodes] = useState<PathNode[]>([]);
  const [pickStations, setPickStations] = useState<PickStation[]>([]);
  const [dropStations, setDropStations] = useState<DropStation[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  /** 按需加载 pathMap 数据 */
  const loadPathMap = useCallback(async () => {
    setIsLoading(true);
    setError(null);
    try {
      const data: PathMapData = await getPathMap();
      setNodes(data.nodes);
      setPickStations(data.pick_stations);
      setDropStations(data.drop_stations);
      return data;
    } catch (err) {
      const msg = err instanceof Error ? err.message : '加载 pathMap 失败';
      setError(msg);
      return null;
    } finally {
      setIsLoading(false);
    }
  }, []);

  /** 更新取货站参数 */
  const updatePickStation = useCallback(
    async (id: number, data: { ox?: number; oy?: number; r?: number; station_num?: number }) => {
      try {
        await apiUpdatePickStation(id, data);
        // 重新加载获取最新数据
        await loadPathMap();
        return true;
      } catch (err) {
        console.error('更新取货站失败:', err);
        return false;
      }
    },
    [loadPathMap]
  );

  return {
    nodes,
    pickStations,
    dropStations,
    isLoading,
    error,
    loadPathMap,
    updatePickStation,
  };
}