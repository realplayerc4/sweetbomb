/**
 * 地图 API 服务
 * 提供地图图片和元数据获取功能
 */

import { API_BASE } from '../config';

// ==================== 类型定义 ====================

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

// ==================== API 函数 ====================

/** 获取地图列表 */
export async function getMapList(): Promise<MapListResponse> {
  const res = await fetch(`${API_BASE}/map/`);
  if (!res.ok) {
    throw new Error(`获取地图列表失败: ${res.status}`);
  }
  return res.json();
}

/** 获取地图图片 (PNG Blob) */
export async function getMapImage(
  mapName: string,
  options: {
    thetaDeg?: number;
    refresh?: boolean;
    pointSize?: number;
    dpi?: number;
  } = {}
): Promise<Blob> {
  const params = new URLSearchParams();
  if (options.thetaDeg !== undefined && options.thetaDeg !== 0) {
    params.append('theta', options.thetaDeg.toString());
  }
  if (options.refresh) {
    params.append('refresh', 'true');
  }
  if (options.pointSize !== undefined) {
    params.append('point_size', options.pointSize.toString());
  }
  if (options.dpi !== undefined) {
    params.append('dpi', options.dpi.toString());
  }

  const queryString = params.toString() ? `?${params.toString()}` : '';
  const url = `${API_BASE}/map/${mapName}.png${queryString}`;

  const res = await fetch(url);
  if (!res.ok) {
    throw new Error(`获取地图图片失败: ${res.status}`);
  }
  return res.blob();
}

/** 获取地图详细信息 */
export async function getMapInfo(
  mapName: string,
  thetaDeg: number = 0
): Promise<MapDetailInfo> {
  const params = new URLSearchParams();
  if (thetaDeg !== 0) {
    params.append('theta', thetaDeg.toString());
  }
  const queryString = params.toString() ? `?${params.toString()}` : '';

  const res = await fetch(`${API_BASE}/map/${mapName}/info${queryString}`);
  if (!res.ok) {
    throw new Error(`获取地图信息失败: ${res.status}`);
  }
  return res.json();
}

/** 清除地图缓存 */
export async function clearMapCache(): Promise<{ success: boolean; message: string }> {
  const res = await fetch(`${API_BASE}/map/cache/clear`, { method: 'POST' });
  if (!res.ok) {
    throw new Error(`清除地图缓存失败: ${res.status}`);
  }
  return res.json();
}
