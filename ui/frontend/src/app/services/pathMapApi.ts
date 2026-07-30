/** PathMap API 客户端 */

import { API_BASE } from '../config';

// ---- TypeScript 接口 ----

export interface PathNode {
  id: number;
  x: number;
  y: number;
}

export interface Segment {
  id: number;
  type: string;
  begin_node: number;
  end_node: number;
  nav_type: string;
  max_speed: number;
  move_direction: string;
  ox: number | null;
  oy: number | null;
  direction: string | null;
}

export interface PickStationPosition {
  station_id: number;  // 站号 (如 101, 102...)
  x: number;           // 坐标 mm
  y: number;           // 坐标 mm
}

export interface PickStation {
  id: number;
  connect_node: number;
  start_id: number;
  ox: number;
  oy: number;
  R: number;  // 半径 (mm) - API 返回大写 R
  station_num: number;
  max_speed: number;
  // 预计算的所有取货位坐标
  positions: PickStationPosition[];
}

export interface DropStation {
  id: number;
  connect_node: number;
  node: number;
  boom_pos: number;
  bucket_pos: number;
  bucket_out_pos: number;
  // 预计算的坐标
  x: number | null;
  y: number | null;
}

export interface ChargeStation {
  id: number;
  node: number;  // 充电节点 ID，需要通过 nodes 查找坐标
  vehicle_th: number;  // 车体角度
  ip: string;  // 充电桩 IP
  port: number;  // 充电桩端口
}

export interface PathMapData {
  nodes: PathNode[];
  segments: Segment[];
  pick_stations: PickStation[];
  drop_stations: DropStation[];
  charge_stations: ChargeStation[];
}

// ---- API 函数 ----

/** 获取完整 pathMap 数据 */
export async function getPathMap(): Promise<PathMapData> {
  const res = await fetch(`${API_BASE}/path_map/`);
  if (!res.ok) throw new Error(`获取 pathMap 失败: ${res.status}`);
  return res.json();
}

/** 更新取货站参数 */
export async function updatePickStation(
  id: number,
  data: { ox?: number; oy?: number; r?: number; station_num?: number }
): Promise<{ success: boolean }> {
  const res = await fetch(`${API_BASE}/path_map/pick_station/${id}`, {
    method: 'PUT',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(data),
  });
  if (!res.ok) throw new Error(`更新取货站失败: ${res.status}`);
  return res.json();
}

/** 更新放货站参数 */
export async function updateDropStation(
  id: number,
  data: { node?: number; boom_pos?: number; bucket_pos?: number; bucket_out_pos?: number }
): Promise<{ success: boolean }> {
  const res = await fetch(`${API_BASE}/path_map/drop_station/${id}`, {
    method: 'PUT',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(data),
  });
  if (!res.ok) throw new Error(`更新放货站失败: ${res.status}`);
  return res.json();
}