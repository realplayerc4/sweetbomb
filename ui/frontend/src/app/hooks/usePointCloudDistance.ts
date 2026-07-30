import { useState, useEffect, useCallback } from 'react';
import { api } from '../services/api';

interface PointCloudDistanceData {
  move_distance: number;
  material_distance: number;
  timestamp: string;
}

interface UsePointCloudDistanceReturn {
  moveDistance: number | null;
  materialDistance: number | null;
  isLoading: boolean;
  error: string | null;
  refresh: () => void;
}

/**
 * Hook to fetch move_distance from backend REST API
 * This replaces the WebSocket-based advanceDistance with REST API polling
 */
export function usePointCloudDistance(
  deviceId: string | null,
  pollInterval: number = 200  // 200ms default for real-time updates
): UsePointCloudDistanceReturn {
  const [data, setData] = useState<PointCloudDistanceData | null>(null);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const fetchDistance = useCallback(async () => {
    if (!deviceId) {
      setData(null);
      return;
    }

    setIsLoading(true);
    setError(null);

    try {
      const response = await api.get(`/devices/${deviceId}/point_cloud/move_distance`);
      setData(response.data);
    } catch (err: any) {
      console.warn('[usePointCloudDistance] Failed to fetch:', err.message);
      setError(err.message || 'Failed to fetch distance');
      // Don't clear data on error, keep last known value
    } finally {
      setIsLoading(false);
    }
  }, [deviceId]);

  // Polling effect
  useEffect(() => {
    if (!deviceId) {
      setData(null);
      return;
    }

    // Initial fetch
    fetchDistance();

    // Setup polling interval
    const intervalId = setInterval(fetchDistance, pollInterval);

    return () => {
      clearInterval(intervalId);
    };
  }, [deviceId, pollInterval, fetchDistance]);

  return {
    moveDistance: data?.move_distance ?? null,
    materialDistance: data?.material_distance ?? null,
    isLoading,
    error,
    refresh: fetchDistance,
  };
}
