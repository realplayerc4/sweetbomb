import { useState, useEffect, useCallback, useRef } from 'react';
import { API_BASE } from '../config';

export interface MapInfo {
  filename: string;
  name: string;
  size_bytes: number;
  modified_time: number;
  has_cached_png: boolean;
  png_url: string | null;
}

export interface MapListResponse {
  total: number;
  maps: MapInfo[];
  cache_dir?: string;
}

// 地图详细信息（包含旋转后的边界）
export interface MapDetailInfo {
  filename: string;
  name: string;
  resolution: number;
  origin: { x: number; y: number };
  grid_size: { width: number; height: number };
  bounds: { min_x: number; min_y: number; max_x: number; max_y: number };
  rotated_bounds: { min_x: number; min_y: number; max_x: number; max_y: number };
  png_bounds: { min_x: number; min_y: number; max_x: number; max_y: number };
  img_width_px: number;
  img_height_px: number;
  point_count: number;
  png_url: string;
  // 像素偏移量（装饰元素占用的空间）
  offset: { x_left: number; y_top: number; y_bottom: number };
  // 数据区域像素尺寸
  data_area_px: { width: number; height: number };
  // axes实际显示的数据范围
  axes_range: { min_x: number; max_x: number; min_y: number; max_y: number };
}

interface UseMapOptions {
  autoLoad?: boolean;
}

export function useMapList(options: UseMapOptions = {}) {
  const { autoLoad = true } = options;
  const [maps, setMaps] = useState<MapInfo[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const fetchMaps = useCallback(async () => {
    setIsLoading(true);
    setError(null);
    try {
      const response = await fetch(`${API_BASE}/map/`);
      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }
      const data: MapListResponse = await response.json();
      setMaps(data.maps);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to fetch maps');
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

  const loadImage = useCallback(async (
    options: {
      refresh?: boolean;
      pointSize?: number;
      dpi?: number;
    } = {}
  ) => {
    if (!mapName) return;

    // Clean up previous object URL
    if (objectUrlRef.current) {
      URL.revokeObjectURL(objectUrlRef.current);
      objectUrlRef.current = null;
    }

    setIsLoading(true);
    setError(null);

    try {
      const params = new URLSearchParams();
      if (options.refresh) params.append('refresh', 'true');
      if (options.pointSize !== undefined) params.append('point_size', options.pointSize.toString());
      if (options.dpi !== undefined) params.append('dpi', options.dpi.toString());
      if (thetaDeg !== 0) params.append('theta', thetaDeg.toString()); // 传递角度值

      const queryString = params.toString() ? `?${params.toString()}` : '';
      const url = `${API_BASE}/map/${mapName}.png${queryString}`;

      const response = await fetch(url);
      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const blob = await response.blob();
      const objectUrl = URL.createObjectURL(blob);
      objectUrlRef.current = objectUrl;
      setImageUrl(objectUrl);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to load map image');
    } finally {
      setIsLoading(false);
    }
  }, [mapName, thetaDeg]);

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
      const params = new URLSearchParams();
      if (thetaDeg !== 0) params.append('theta', thetaDeg.toString());
      const queryString = params.toString() ? `?${params.toString()}` : '';
      const response = await fetch(`${API_BASE}/map/${mapName}/info${queryString}`);
      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }
      const data = await response.json();
      setInfo(data);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to fetch map info');
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
