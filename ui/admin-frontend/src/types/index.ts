export type RobotState = 'idle' | 'moving' | 'scooping' | 'dumping' | 'error' | 'emergency_stop';

export type MoveDirection = 'forward' | 'backward' | 'left' | 'right' | 'rotate_left' | 'rotate_right' | 'stop';

export type TaskStatus = 'pending' | 'running' | 'paused' | 'completed' | 'failed' | 'stopped' | 'cancelled';

export type TaskPriority = 1 | 5 | 10 | 20;

export type DeviceStatus = 'online' | 'offline' | 'warning' | 'error';

export type AlertLevel = 'info' | 'warning' | 'error' | 'critical';

export type DeviceWorkState = 'idle' | 'working' | 'fault' | 'charging';

export type ZoneType = 'sugar_pile' | 'loading' | 'unloading' | 'charging' | 'restricted' | 'fence';

export interface ServoState {
  name: string;
  current_angle: number;
  target_angle: number;
  is_moving: boolean;
}

export interface RobotStatus {
  state: RobotState;
  battery_level: number;
  current_position: [number, number, number];
  orientation: [number, number, number];
  imu?: { roll: number; pitch: number; yaw: number };
  left_track_speed: number;
  right_track_speed: number;
  servos: Record<string, ServoState>;
  timestamp: string;
}

export interface LoaderSpec {
  model: string;
  bucket_capacity: number;
  max_speed: number;
  bucket_width: number;
  max_load_weight: number;
}

export interface DeviceInfo {
  device_id: string;
  name: string;
  status: DeviceStatus;
  work_state: DeviceWorkState;
  battery: number;
  position: [number, number];
  last_heartbeat: string;
  task_id?: string;
  spec?: LoaderSpec;
  total_hours?: number;
  maintenance_count?: number;
  energy_consumption?: number;
}

export interface TaskProgress {
  current_step: number;
  total_steps: number;
  percentage: number;
  message: string;
  elapsed_seconds: number;
  estimated_remaining_seconds?: number | null;
}

export interface TaskResult {
  success: boolean;
  data?: Record<string, unknown>;
  message?: string;
  error?: string | null;
}

export interface TaskInfo {
  task_id: string;
  task_type: string;
  status: TaskStatus;
  priority: TaskPriority;
  device_id?: string | null;
  config: TaskConfig;
  params: Record<string, unknown>;
  created_at: string;
  started_at?: string | null;
  completed_at?: string | null;
  progress: TaskProgress;
  result?: TaskResult | null;
  assigned_devices?: string[];
  scoop_point?: string;
  dump_point?: string;
  target_ton?: number;
}

export interface TaskConfig {
  device_id?: string;
  priority?: TaskPriority;
  max_retries?: number;
  timeout_seconds?: number | null;
  params?: Record<string, unknown>;
}

export interface TaskTypeInfo {
  task_type: string;
  name: string;
  description: string;
  category: string;
  requires_device: boolean;
}

export interface TaskCreateRequest {
  task_type: string;
  device_id?: string | null;
  config?: TaskConfig | null;
  params?: Record<string, unknown>;
}

export interface AlertInfo {
  id: string;
  level: AlertLevel;
  source: string;
  message: string;
  timestamp: string;
  acknowledged: boolean;
  category?: 'collision' | 'speed' | 'zone' | 'device' | 'system' | 'network';
}

export interface SystemStats {
  total_devices: number;
  online_devices: number;
  running_tasks: number;
  pending_tasks: number;
  completed_today: number;
  alert_count: number;
  avg_response_time: number;
  uptime_hours: number;
  idle_devices: number;
  working_devices: number;
  fault_devices: number;
  today_sugar_ton: number;
}

export interface Waypoint {
  name: string;
  pos: [number, number, number];
  created_at?: string;
}

export interface SugarHarvestConfig {
  navigation_point: [number, number];
  dump_point: [number, number];
  bucket_width_m?: number;
  approach_offset_m?: number;
  scoop_position?: number;
  dump_position?: number;
  max_cycles?: number;
  height_threshold_m?: number;
}

export interface SugarHarvestStatus {
  is_running: boolean;
  current_cycle?: number;
  max_cycles?: number;
  sugar_height?: number;
  height_threshold?: number;
}

export interface ChartDataPoint {
  name: string;
  value: number;
}

export interface TimeSeriesPoint {
  time: string;
  value: number;
}

export interface ZoneInfo {
  id: string;
  name: string;
  type: ZoneType;
  bounds: [[number, number], [number, number]];
  color: string;
  enabled: boolean;
}

export interface DispatchAssignment {
  task_id: string;
  device_id: string;
  priority: TaskPriority;
  reason: string;
}

export interface MaintenanceRecord {
  id: string;
  device_id: string;
  type: 'routine' | 'repair' | 'inspection';
  description: string;
  date: string;
  operator: string;
  cost?: number;
}

export interface ReportConfig {
  type: 'daily' | 'weekly' | 'monthly';
  date_range: [string, string];
  metrics: string[];
}

export interface LoaderRegistration {
  name: string;
  model: string;
  bucket_capacity: number;
  max_speed: number;
  bucket_width: number;
  max_load_weight: number;
}
