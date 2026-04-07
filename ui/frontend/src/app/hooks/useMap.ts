import { useState, useEffect, useCallback, useRef } from 'react';
import { API_BASE } from '../config';

export interface MapInfo {
  name: string;
  resolution: number;
  origin_x: number;
  origin_y: number;
  grid_width: number;
  grid_height: number;
  total_points: number;
}

export interface MapListResponse {
  maps: MapInfo[];
  cache_dir: string;
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

export function useMapImage(mapName: string | null) {
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
  }, [mapName]);

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
  }, [mapName, loadImage]);

  return {
    imageUrl,
    isLoading,
    error,
    loadImage,
  };
}

export function useMapInfo(mapName: string | null) {
  const [info, setInfo] = useState<MapInfo | null>(null);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const fetchInfo = useCallback(async () => {
    if (!mapName) return;

    setIsLoading(true);
    setError(null);
    try {
      const response = await fetch(`${API_BASE}/map/${mapName}/info`);
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
  }, [mapName]);

  useEffect(() => {
    if (mapName) {
      fetchInfo();
    }
  }, [mapName, fetchInfo]);

  return {
    info,
    isLoading,
    error,
    refresh: fetchInfo,
  };
}
