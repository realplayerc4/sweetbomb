# Admin-Frontend (SWNFP) 架构文档

> 后台管理前端 - 糖厂机器人车队管理系统

---

## 概述

**访问地址:** `http://192.168.3.101:5174/`

**技术栈:**
- React 19 + TypeScript
- Vite 7 (构建工具)
- Ant Design 6 (UI 组件库)
- Zustand 5 (状态管理)
- Socket.IO (实时通信)
- ECharts (图表)
- Leaflet (地图)
- Tailwind CSS 4 (样式)

---

## 目录结构

```
src/
├── App.tsx                 # 主入口，配置 Ant Design 暗色主题
├── main.tsx               # React 根渲染
├── index.css              # 全局样式 (Tailwind + 自定义 CSS)
├── config.ts              # API 配置常量
├── assets/                # 静态资源
├── components/
│   ├── charts/           # 图表组件 (ECharts)
│   │   ├── EChartsWrapper.tsx
│   │   ├── TaskStatsChart.tsx
│   │   └── DeviceStatsChart.tsx
│   ├── control/          # 机器人控制组件
│   │   └── HarvestControl.tsx
│   ├── dispatch/         # 调度组件
│   │   └── DispatchPanel.tsx
│   ├── layout/           # 布局组件
│   │   ├── DashboardLayout.tsx
│   │   ├── Header.tsx
│   │   ├── FixedModule.tsx
│   │   └── DraggableModule.tsx
│   ├── map/              # 地图可视化
│   │   ├── MapMonitor.tsx
│   │   └── MapOverview.tsx
│   ├── modals/           # 模态对话框
│   │   ├── DeviceDetailModal.tsx
│   │   ├── ManualInterveneModal.tsx
│   │   ├── RegisterLoaderModal.tsx
│   │   ├── ReportModal.tsx
│   │   ├── TaskCreateModal.tsx
│   │   ├── TaskDetailModal.tsx
│   │   ├── TechModal.tsx
│   │   └── ZoneManageModal.tsx
│   ├── monitor/          # 监控组件
│   │   ├── DeviceList.tsx
│   │   ├── DeviceOverview.tsx
│   │   ├── RobotStatusPanel.tsx
│   │   ├── SensorMonitor.tsx
│   │   ├── VideoStreamView.tsx  ⭐ 新增
│   │   └── WorkingDevices.tsx
│   ├── overview/         # 仪表盘概览组件
│   │   ├── AlertPanel.tsx
│   │   ├── HarvestForecast.tsx
│   │   ├── HarvestStats.tsx
│   │   └── StatsCards.tsx
│   └── task/             # 任务管理
│       ├── TaskBoard.tsx
│       ├── TaskCreateModal.tsx
│       ├── TaskPanel.tsx
│       └── TaskTimeline.tsx
├── hooks/                # 自定义 React Hooks
│   └── useWebRTCConnection.ts  ⭐ 新增
├── lib/                  # 工具函数
│   └── utils.ts          # cn() 类名合并工具
├── services/             # API 服务层
│   ├── apiClient.ts      # Axios 实例
│   ├── deviceApi.ts
│   ├── robotApi.ts
│   ├── taskApi.ts
│   └── webrtcApi.ts      ⭐ 新增
├── stores/               # Zustand 状态管理
│   ├── useRobotStore.ts
│   ├── useSystemModeStore.ts
│   ├── useSystemStore.ts
│   └── useTaskStore.ts
└── types/                # TypeScript 类型定义
    └── index.ts
```

---

## 配置文件

### config.ts

```typescript
const isDev = import.meta.env?.DEV;
const API_PORT = '8000';
const HOST = isDev
  ? `${window.location.protocol}//${window.location.hostname}:${API_PORT}`
  : window.location.origin;

export const API_BASE = `${HOST}/api`;    // API 基础路径
export const SOCKET_URL = HOST;            // Socket.IO 连接地址

export const REFRESH_INTERVAL = 2000;      // 刷新间隔 (ms)
export const HEARTBEAT_INTERVAL = 30000;   // 心跳间隔 (ms)
export const MAX_RECONNECT_ATTEMPTS = 5;   // 最大重连次数
```

---

## 状态管理 (Zustand Stores)

### useRobotStore - 机器人控制状态

| 状态字段 | 类型 | 说明 |
|---------|------|------|
| `status` | RobotState | 机器人状态 (idle/moving/scooping/dumping/error) |
| `battery` | number | 电池电量 |
| `position` | [number, number, number] | 当前位置 |
| `orientation` | [number, number, number] | 方向 |
| `imu` | { roll, pitch, yaw } | IMU 数据 |
| `leftTrackSpeed` | number | 左履带速度 |
| `rightTrackSpeed` | number | 右履带速度 |
| `isConnected` | boolean | Socket 连接状态 |
| `isControllable` | boolean | 是否可控 |
| `isHarvestRunning` | boolean | 铲糖循环是否运行中 |
| `harvestCycle` | number | 当前循环次数 |
| `harvestMaxCycles` | number | 最大循环次数 |
| `error` | string \| null | 错误信息 |

**主要 Actions:**
- `connect()` / `disconnect()` - Socket.IO 连接管理
- `refreshStatus()` - 刷新机器人状态
- `move(direction, speed?, duration?)` - 移动控制
- `stop()` / `reset()` - 停止/重置
- `setServo(servoId, angle)` - 伺服控制
- `scoop()` / `dump()` / `dock()` - 铲取/倾倒/回桩
- `startHarvest(config)` / `stopHarvest()` - 自动铲糖循环

### useTaskStore - 任务管理状态

| 状态字段 | 类型 | 说明 |
|---------|------|------|
| `tasks` | TaskInfo[] | 任务列表 |
| `isLoading` | boolean | 加载状态 |
| `isConnected` | boolean | Socket 连接状态 |
| `runningCount` | number | 运行中任务数 |
| `pendingCount` | number | 待执行任务数 |
| `completedToday` | number | 今日完成任务数 |

**主要 Actions:**
- `createTask(request)` - 创建任务
- `startTask(taskId)` / `pauseTask(taskId)` / `resumeTask(taskId)` / `stopTask(taskId)` - 任务控制
- `deleteTask(taskId)` - 删除任务
- `refreshTasks()` - 刷新任务列表

### useSystemStore - 系统全局状态

| 状态字段 | 类型 | 说明 |
|---------|------|------|
| `stats` | SystemStats | 系统统计 |
| `devices` | DeviceInfo[] | 设备列表 |
| `alerts` | AlertInfo[] | 告警列表 |
| `zones` | ZoneInfo[] | 区域信息 |
| `maintenanceRecords` | MaintenanceRecord[] | 维护记录 |
| `currentTime` | Date | 当前时间 |
| `isOffline` | boolean | 离线状态 |

### useSystemModeStore - UI 模式状态

| 状态字段 | 类型 | 说明 |
|---------|------|------|
| `monitorMode` | 'single' \| 'all' | 监控模式 (单机器人/全部) |
| `selectedRobot` | string | 当前选中的机器人 |

---

## API 服务层

### apiClient.ts - Axios 实例

```typescript
import axios from 'axios';
import { API_BASE } from '../config';

const apiClient = axios.create({
  baseURL: API_BASE,
  timeout: 10000,
  headers: { 'Content-Type': 'application/json' },
});

// 错误拦截器
apiClient.interceptors.response.use(
  (response) => response,
  (error) => {
    const message = error.response?.data?.detail || error.message;
    return Promise.reject(new Error(message));
  }
);

export default apiClient;
```

### robotApi.ts - 机器人控制 API

| 方法 | 端点 | 说明 |
|------|------|------|
| `getStatus()` | GET /robot/status | 获取机器人状态 |
| `move(direction, speed, duration)` | POST /robot/move | 移动控制 |
| `stop()` | POST /robot/stop | 停止 |
| `reset()` | POST /robot/reset | 重置 |
| `setServo(servoId, angle)` | POST /robot/servo | 伺服控制 |
| `scoop()` | POST /robot/scoop | 铲取 |
| `dump()` | POST /robot/dump | 倾倒 |
| `dock()` | POST /robot/dock | 回桩 |
| `startSugarHarvest(config)` | POST /robot/auto_cycle/start | 启动铲糖循环 |
| `stopSugarHarvest()` | POST /robot/auto_cycle/stop | 停止铲糖循环 |
| `getSugarHarvestStatus()` | GET /robot/auto_cycle/status | 获取铲糖状态 |

### taskApi.ts - 任务管理 API

| 方法 | 端点 | 说明 |
|------|------|------|
| `getTaskTypes()` | GET /tasks/types | 获取任务类型 |
| `listTasks(filters?)` | GET /tasks | 获取任务列表 |
| `createTask(request)` | POST /tasks | 创建任务 |
| `startTask(taskId)` | POST /tasks/{id}/start | 启动任务 |
| `pauseTask(taskId)` | POST /tasks/{id}/pause | 暂停任务 |
| `resumeTask(taskId)` | POST /tasks/{id}/resume | 恢复任务 |
| `stopTask(taskId)` | POST /tasks/{id}/stop | 停止任务 |
| `deleteTask(taskId)` | DELETE /tasks/{id} | 删除任务 |

### deviceApi.ts - 设备管理 API

| 方法 | 端点 | 说明 |
|------|------|------|
| `getDevices()` | GET /devices | 获取设备列表 |
| `getDevice(deviceId)` | GET /devices/{id} | 获取设备详情 |

### webrtcApi.ts - WebRTC 视频流 API ⭐ 新增

| 方法 | 端点 | 说明 |
|------|------|------|
| `getDevices()` | GET /devices | 获取流设备列表 |
| `startStream(deviceId, configs)` | POST /devices/{id}/stream/start | 启动流 |
| `stopStream(deviceId)` | POST /devices/{id}/stream/stop | 停止流 |
| `getWebRTCOffer(deviceId, streamTypes)` | POST /webrtc/offer | 获取 WebRTC Offer |
| `sendAnswer(sessionId, answer)` | POST /webrtc/answer | 发送 Answer |
| `sendIceCandidate(sessionId, candidate)` | POST /webrtc/ice-candidates | 发送 ICE Candidate |
| `getIceCandidates(sessionId)` | GET /webrtc/sessions/{id}/ice-candidates | 获取 ICE Candidates |
| `activatePointCloud(deviceId)` | POST /devices/{id}/point_cloud/activate | 激活点云 |
| `deactivatePointCloud(deviceId)` | POST /devices/{id}/point_cloud/deactivate | 停用点云 |

---

## 自定义 Hooks

### useWebRTCConnection.ts ⭐ 新增

WebRTC 视频流连接 Hook，管理完整的 WebRTC 连接生命周期。

**返回值:**
```typescript
{
  device: DeviceInfo | null;           // 当前设备
  isStreaming: boolean;                // 是否正在推流
  rgbStream: MediaStream | null;       // RGB 视频流
  depthStream: MediaStream | null;     // 深度视频流
  streamMetrics: {
    rgb: { width, height, fps } | null;
    depth: { width, height, fps } | null;
  };
  error: string | null;                // 错误信息
  startConnection: () => Promise<void>; // 启动连接
  stopConnection: () => Promise<void>;  // 停止连接
}
```

**使用示例:**
```typescript
const { rgbStream, depthStream, isStreaming, streamMetrics } = useWebRTCConnection();

// 在组件中使用
{isStreaming && rgbStream && (
  <video autoPlay playsInline srcObject={rgbStream} />
)}
```

---

## 核心组件

### SensorMonitor.tsx - 传感器监控

支持两种模式:
1. **单机器人模式** - 显示真实 WebRTC 视频流
2. **多机器人模式** - 第一个设备用真实视频流，其他用 Canvas 模拟

**Props:**
```typescript
interface SensorMonitorProps {
  type: 'rgb' | 'depth';  // 传感器类型
}
```

### VideoStreamView.tsx - 视频流显示 ⭐ 新增

纯视频显示组件，用于渲染 MediaStream。

**Props:**
```typescript
interface VideoStreamViewProps {
  stream: MediaStream | null;
  type: 'rgb' | 'depth';
  isConnected: boolean;
  metrics?: { width: number; height: number; fps: number } | null;
  deviceName?: string;
}
```

### MapMonitor.tsx - 地图监控

基于 Leaflet 的地图可视化组件，显示设备位置和区域。

### TaskPanel.tsx - 任务面板

任务列表展示，支持进度条、快速操作按钮。

### HarvestControl.tsx - 铲糖控制

自动铲糖循环控制面板，显示流程步骤。

---

## Socket.IO 事件

### 监听事件

| 事件 | 数据 | 说明 |
|------|------|------|
| `connect` | - | Socket 连接成功 |
| `disconnect` | - | Socket 断开 |
| `metadata_update` | { system_stats, metadata_streams } | 系统状态和流元数据 |
| `bt_event` | { event_type, current_cycle, max_cycles } | 行为树事件 |
| `task_event` | { event_type, task_id, status, progress } | 任务事件 |
| `robot_task_finish` | { task_id } | 机器人任务完成 |

### 发送事件

| 事件 | 说明 |
|------|------|
| `ping` | 心跳 |

---

## 类型定义 (types/index.ts)

### 核心类型

```typescript
// 机器人状态
export type RobotState = 'idle' | 'moving' | 'scooping' | 'dumping' | 'error' | 'emergency_stop';

// 任务状态
export type TaskStatus = 'pending' | 'running' | 'paused' | 'completed' | 'failed' | 'stopped' | 'cancelled';

// 设备状态
export type DeviceStatus = 'online' | 'offline' | 'warning' | 'error';

// 告警级别
export type AlertLevel = 'info' | 'warning' | 'error' | 'critical';

// 设备信息
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
}

// 任务信息
export interface TaskInfo {
  task_id: string;
  task_type: string;
  status: TaskStatus;
  priority: TaskPriority;
  device_id?: string;
  config: TaskConfig;
  params: Record<string, unknown>;
  created_at: string;
  started_at?: string;
  completed_at?: string;
  progress: TaskProgress;
  result?: TaskResult;
}

// 铲糖配置
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
```

---

## 与 SW 前端的差异

| 特性 | Admin-Frontend (SWNFP) | SW Frontend |
|------|------------------------|-------------|
| UI 框架 | Ant Design 6 | shadcn/ui + Radix |
| 状态管理 | Zustand | React Hooks |
| 布局 | 18 列固定网格 | 自适应布局 |
| 路由 | 无 (单页) | 无 (单页) |
| 用途 | 后台管理、多设备监控 | 机器人操控、实时控制 |

---

## 开发命令

```bash
# 进入目录
cd /home/jetson/sweetbomb/ui/admin-frontend

# 安装依赖
npm install

# 开发模式
npm run dev

# 构建生产版本
npm run build

# 预览生产版本
npm run preview
```

---

## 服务管理

Admin-Frontend 作为 systemd 服务运行:

```bash
# 查看状态
systemctl status sweetbomb-admin-frontend

# 重启服务
sudo systemctl restart sweetbomb-admin-frontend

# 查看日志
journalctl -u sweetbomb-admin-frontend -f
```

---

*Version: v1.0*
*Last Updated: 2026-04-15*
