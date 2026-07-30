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
│   │   └── FixedModule.tsx
│   ├── map/              # 地图可视化
│   │   └── MapMonitor.tsx
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
│   │   ├── RobotStatusPanel.tsx
│   │   ├── SensorMonitor.tsx
│   │   └── WorkingDevices.tsx
│   ├── overview/         # 仪表盘概览组件
│   │   ├── AlertPanel.tsx
│   │   ├── HarvestStats.tsx
│   │   └── StatsCards.tsx
│   └── task/             # 任务管理
│       ├── TaskBoard.tsx
│       ├── TaskPanel.tsx
│       └── TaskTimeline.tsx
├── hooks/                # 自定义 React Hooks
│   ├── useMap.ts         # 地图数据加载
│   └── usePathMap.ts     # 站点数据加载
├── lib/                  # 工具函数
│   └── utils.ts          # cn() 类名合并工具
├── services/             # API 服务层
│   ├── apiClient.ts      # Axios 实例
│   ├── mapApi.ts         # 地图 API
│   ├── pathMapApi.ts     # 站点 API
│   ├── robotApi.ts       # 机器人控制 API
│   └── taskApi.ts        # 任务 API
├── stores/               # Zustand 状态管理
│   ├── useRobotStore.ts  # 机器人状态
│   ├── useSystemModeStore.ts  # UI 模式
│   ├── useSystemStore.ts # 系统全局状态
│   └── useTaskStore.ts   # 任务状态
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
```

---

## 状态管理 (Zustand Stores)

### useRobotStore - 机器人控制状态

| 状态字段 | 类型 | 说明 |
|---------|------|------|
| `status` | RobotState | 机器人状态 (idle/moving/scooping/dumping/error) |
| `battery` | number | 电池电量 |
| `position` | [number, number, number] | 当前位置 (x, y, z) mm |
| `orientation` | [number, number, number] | 方向 (a, 0, 0) |
| `leftTrackSpeed` | number | 左履带速度 (m/s) |
| `rightTrackSpeed` | number | 右履带速度 (m/s) |
| `isConnected` | boolean | Socket 连接状态 |
| `isHarvestRunning` | boolean | 铲糖循环是否运行中 |
| `harvestCycle` | number | 当前循环次数 |
| `harvestMaxCycles` | number | 最大循环次数 |

**主要 Actions:**
- `connect()` / `disconnect()` - Socket.IO 连接管理
- `refreshStatus()` - 刷新机器人状态（每秒轮询）
- `move(direction, speed?, duration?)` - 移动控制
- `stop()` / `pause()` / `resume()` - 停止/暂停/恢复
- `scoop()` / `dump()` / `dock()` - 铲取/倾倒/回桩
- `navToPick()` / `navToDrop()` - 导航到取货点/卸货点

### useTaskStore - 任务管理状态 ⭐ 重构

| 状态字段 | 类型 | 说明 |
|---------|------|------|
| `targetKg` | number | 目标重量 (kg)，默认 900kg |
| `perCycleKg` | number | 每次循环重量，默认 30kg |
| `totalCycles` | number | 总循环次数 |
| `currentCycle` | number | 当前循环次数 |
| `phase` | TaskPhase | 当前阶段 |
| `isRunning` | boolean | 是否正在运行 |
| `completedKg` | number | 已完成重量 (kg) |
| `error` | string \| null | 错误信息 |

**任务阶段 (TaskPhase):**
| 阶段 | 说明 |
|------|------|
| `idle` | 空闲 |
| `nav_to_pick` | 取货中（导航+取货） |
| `nav_to_drop` | 卸货中（导航+卸货） |
| `paused` | 已暂停 |
| `completed` | 任务完成 |
| `error` | 错误 |

**主要 Actions:**
- `setTarget(kg)` - 设置目标重量，自动计算循环次数
- `start()` - 启动任务循环
- `pause()` - 暂停任务
- `resume()` - 恢复任务
- `stop()` - 停止并重置任务

### useSystemStore - 系统全局状态

| 状态字段 | 类型 | 说明 |
|---------|------|------|
| `stats` | SystemStats | 系统统计 |
| `devices` | DeviceInfo[] | 设备列表 (Mock 数据) |
| `alerts` | AlertInfo[] | 告警列表 |
| `zones` | ZoneInfo[] | 区域信息 |

### useSystemModeStore - UI 模式状态

| 状态字段 | 类型 | 说明 |
|---------|------|------|
| `monitorMode` | 'single' \| 'all' | 监控模式 |
| `selectedRobot` | string | 当前选中的机器人 |

---

## API 服务层

### robotApi.ts - 机器人控制 API

| 方法 | 端点 | 说明 |
|------|------|------|
| `getStatus()` | GET /robot/status | 获取机器人状态 |
| `move(direction, speed, duration)` | POST /robot/move | 移动控制 |
| `stop()` | POST /robot/stop | 停止 |
| `pause()` | POST /robot/pause | 暂停 |
| `resume()` | POST /robot/resume | 恢复 |
| `scoop()` | POST /robot/scoop | 铲取 |
| `dump()` | POST /robot/dump | 倾倒 |
| `dock()` | POST /robot/dock | 回桩 |
| `navToPick()` | POST /robot/nav-pick | 导航到取货点 (含取货) |
| `navToDrop()` | POST /robot/nav-drop | 导航到卸货点 (含卸货) |

### mapApi.ts - 地图 API

| 方法 | 端点 | 说明 |
|------|------|------|
| `getMapList()` | GET /map/list | 获取地图列表 |
| `getMapImage(name, theta)` | GET /map/{name}/image | 获取地图图片 |
| `getMapInfo(name, theta)` | GET /map/{name}/info | 获取地图信息 |

### pathMapApi.ts - 站点 API

| 方法 | 端点 | 说明 |
|------|------|------|
| `getPathMap()` | GET /path_map | 获取站点数据 |
| `updatePickStation(id, data)` | PUT /path_map/pick_station/{id} | 更新取货站 |

---

## 任务系统架构 ⭐

### 任务循环流程

```
┌─────────────────────────────────────────────────────────────┐
│                      任务循环流程                            │
├─────────────────────────────────────────────────────────────┤
│  目标: 900 kg  →  30 次循环 (每次 30kg)                      │
│                                                             │
│  每次循环:                                                   │
│  1. nav_to_pick   → POST /api/robot/nav-pick (含取货)       │
│     等待 status === 'idle' (最多3分钟)                       │
│                                                             │
│  2. nav_to_drop   → POST /api/robot/nav-drop (含卸货)       │
│     等待 status === 'idle' (最多3分钟)                       │
│                                                             │
│  3. 完成一次循环 → currentCycle++, completedKg += 30kg      │
│                                                             │
│  重复直到 currentCycle >= totalCycles                       │
└─────────────────────────────────────────────────────────────┘
```

### 数据来源

| 数据 | 1号装载机 | 2-6号装载机 |
|------|-----------|-------------|
| 状态 | 真实 (useRobotStore) | Mock (useSystemStore) |
| 电量 | 真实 | Mock |
| 速度 | 真实 (m/min) | Mock |
| 位置 | 真实 (地图显示) | Mock |

---

## 核心组件

### TaskPanel.tsx - 任务面板 ⭐

任务控制主界面，包含：
- 目标设置 (kg)
- 进度显示 (仪表盘 + 进度条)
- 控制按钮 (启动/暂停/终止)
- 实时状态显示

### WorkingDevices.tsx - 设备列表

显示所有设备，1号装载机标记为"本机"：
- 真实数据：速度 (m/min)、电量、状态
- 装载量：1号 = 30kg，其他 = 0.85吨

### MapMonitor.tsx - 地图监控

基于后端地图图片的可视化：
- 显示机器人实时位置
- 显示站点标记 (取货站/卸货站/充电站)
- 支持缩放、拖拽、偏移校准

### StatsCards.tsx - 统计卡片

显示关键指标：
- 在线设备数
- 任务进度
- 已装载重量

---

## Socket.IO 事件

### 监听事件

| 事件 | 数据 | 说明 |
|------|------|------|
| `connect` | - | Socket 连接成功 |
| `disconnect` | - | Socket 断开 |
| `metadata_update` | { system_stats } | 系统状态更新 |
| `bt_event` | { event_type, current_cycle, max_cycles } | 行为树事件 |
| `robot_position_update` | { x, y, z, a } | 机器人位置更新 |

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

// 任务阶段
export type TaskPhase = 'idle' | 'nav_to_pick' | 'nav_to_drop' | 'paused' | 'completed' | 'error';

// 设备状态
export type DeviceStatus = 'online' | 'offline' | 'warning' | 'error';

// 设备工作状态
export type DeviceWorkState = 'idle' | 'working' | 'fault' | 'maintenance';
```

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

*Version: v2.0*
*Last Updated: 2026-04-15*
