/**
 * useMap Hook - 地图数据加载
 */

import { useState, useEffect, useCallback, useRef } from 'react';
import {
  getMapList,
  getMapImage,
  getMapInfo,
  type MapInfo,
  type MapListResponse,
  type MapDetailInfo,
} from '../services/mapApi';

interface UseMapListOptions {
  autoLoad?: boolean;
}

export function useMapList(options: UseMapListOptions = {}) {
  const { autoLoad = true } = options;
  const [maps, setMaps] = useState<MapInfo[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const fetchMaps = useCallback(async () => {
    setIsLoading(true);
    setError(null);
    try {
      const data: MapListResponse = await getMapList();
      setMaps(data.maps);
    } catch (err) {
      setError(err instanceof Error ? err.message : '获取地图列表失败');
    } finally {
      setIsLoading(false);
    }
  }, []);

  useEffect(() => {
    if (autoLoad) {
      fetchMaps();
    }
  }, [autoLoad, fetchMaps]);

  return {
    maps,
    isLoading,
    error,
    refresh: fetchMaps,
  };
}

export function useMapImage(mapName: string | null, thetaDeg: number = 0) {
  const [imageUrl, setImageUrl] = useState<string | null>(null);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const objectUrlRef = useRef<string | null>(null);

  const loadImage = useCallback(
    async (options: { refresh?: boolean; pointSize?: number; dpi?: number } = {}) => {
      if (!mapName) return;

      // 清理之前的 Object URL
      if (objectUrlRef.current) {
        URL.revokeObjectURL(objectUrlRef.current);
        objectUrlRef.current = null;
      }

      setIsLoading(true);
      setError(null);

      try {
        const blob = await getMapImage(mapName, {
          thetaDeg,
          refresh: options.refresh,
          pointSize: options.pointSize,
          dpi: options.dpi,
        });
        const objectUrl = URL.createObjectURL(blob);
        objectUrlRef.current = objectUrl;
        setImageUrl(objectUrl);
      } catch (err) {
        setError(err instanceof Error ? err.message : '加载地图图片失败');
      } finally {
        setIsLoading(false);
      }
    },
    [mapName, thetaDeg]
  );

  useEffect(() => {
    if (mapName) {
      loadImage();
    }
    return () => {
      if (objectUrlRef.current) {
        URL.revokeObjectURL(objectUrlRef.current);
        objectUrlRef.current = null;
      }
    };
  }, [mapName, thetaDeg, loadImage]);

  return {
    imageUrl,
    isLoading,
    error,
    loadImage,
  };
}

export function useMapInfo(mapName: string | null, thetaDeg: number = 0) {
  const [info, setInfo] = useState<MapDetailInfo | null>(null);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const fetchInfo = useCallback(async () => {
    if (!mapName) return;

    setIsLoading(true);
    setError(null);
    try {
      const data = await getMapInfo(mapName, thetaDeg);
      setInfo(data);
    } catch (err) {
      setError(err instanceof Error ? err.message : '获取地图信息失败');
    } finally {
      setIsLoading(false);
    }
  }, [mapName, thetaDeg]);

  useEffect(() => {
    if (mapName) {
      fetchInfo();
    }
  }, [mapName, thetaDeg, fetchInfo]);

  return {
    info,
    isLoading,
    error,
    refresh: fetchInfo,
  };
}