# CLAUDE.md

> AI 行为准则与上下文规则

---

## Always Read (必读规则)

在开始任何任务之前，必须阅读以下文件以获取完整上下文：

### Priority 1 - 项目上下文

```
memory-bank/project-overview.md  → 项目目标、非目标、核心实体
memory-bank/architecture.md      → 系统架构、模块职责
memory-bank/tech-stack.md        → 技术选型决策
```

### Priority 2 - 变更历史

```
docs/changelog.md                → 版本变更记录
memory-bank/progress.md          → 项目进度
```

---

## Module Boundaries (模块边界)

### 后端模块 (app/)

| 模块 | 职责 | 允许修改 |
|------|------|----------|
| `api/endpoints/` | API 路由定义 | 是 |
| `services/` | 业务逻辑 | 是 |
| `models/` | 数据模型 | 是 |
| `core/` | 核心配置 | 是 |

### 前端模块 (ui/)

#### SW 前端 (ui/frontend/src/app/) - 机器人操控

| 模块 | 职责 | 允许修改 |
|------|------|----------|
| `components/` | React 组件 | 是 |
| `hooks/` | 状态 Hooks | 是 |
| `services/` | API 调用 | 是 |

#### SWNFP 前端 (ui/admin-frontend/src/) - 后台管理

| 模块 | 职责 | 允许修改 |
|------|------|----------|
| `components/` | Dashboard 组件 | 是 |
| `stores/` | Zustand 状态管理 | 是 |
| `services/` | API 调用 | 是 |

### 文档模块

| 模块 | 职责 | 修改规则 |
|------|------|----------|
| `memory-bank/` | AI 上下文 | 功能变更时更新 |
| `docs/changelog.md` | 变更日志 | 每次提交时更新 |

---

## Naming Conventions (命名约定)

| 类型 | 约定 | 示例 |
|------|------|------|
| API 路由 | snake_case | `/api/device_info` |
| Python 变量 | snake_case | `device_count` |
| Python 类 | PascalCase | `DeviceManager` |
| React 组件 | PascalCase | `TaskPanel.tsx` |
| React Hooks | camelCase + use | `useTaskManager.ts` |
| 常量 | UPPER_SNAKE | `MAX_RETRIES` |

---

## Code Style (代码风格)

### Python

- 使用 ruff 进行 linting
- 遵循 PEP 8 规范
- 优先使用 async/await

### TypeScript/React

- 使用 ESLint + Prettier
- 优先使用函数组件 + Hooks
- 明确的类型定义，避免 any

---

## Error Handling (错误处理)

- API 端点: 返回标准 HTTP 状态码 + 错误详情
- 服务层: 抛出自定义异常，由路由层捕获
- 前端: 显示用户友好的错误消息

---

## Systemd Services (开机自启动)

| Port | Service | Type | 用途 |
|------|---------|------|------|
| 5173 | sweetbomb-frontend | Vite (Frontend) | 机器人操控 (SW) |
| 5174 | sweetbomb-admin-frontend | Vite (Admin Frontend) | 后台管理 (SWNFP) |
| 8000 | sweetbomb-backend | FastAPI (Backend) | 统一后端 |

**安装自启动：**
```bash
cd /home/jetson/sweetbomb
sudo ./install-service.sh
```

**常用命令：**
```bash
# 启动所有服务
systemctl start sweetbomb-backend sweetbomb-frontend sweetbomb-admin-frontend
# 停止所有服务
systemctl stop sweetbomb-backend sweetbomb-frontend sweetbomb-admin-frontend
# 重启所有服务
systemctl restart sweetbomb-backend sweetbomb-frontend sweetbomb-admin-frontend
# 查看状态
systemctl status sweetbomb-backend sweetbomb-frontend sweetbomb-admin-frontend
# 查看日志
journalctl -u sweetbomb-backend -u sweetbomb-frontend -u sweetbomb-admin-frontend -f
```

**服务特性：**
- `Restart=always` - 进程崩溃自动重启
- `RestartSec=3` - 重启间隔 3 秒
- `After=network.target` - 等待网络后启动

---

## TCP Protocol (机器人通讯)

**端口:** 9090

**消息格式:**
```
{MessageType=xxx\nField=value\n}
```

**任务类型:**
| Type | 说明 |
|------|------|
| pick | 铲取 |
| drop | 倾倒 |
| charge | 回桩充电 |
| allPick | 导航到取货点 |
| allDrop | 导航到卸货点 |
| cancelTask | 取消任务 |
| pauseTask | 暂停任务 |
| pauseCancel | 取消暂停 |

**回复规则:** 只有 `taskFinish` 需要回复下位机，其他消息不回复。

---

## Point Cloud Analysis (点云分析)

**API 端点:**
- `GET /api/devices/{device_id}/point_cloud/move_distance` - 获取前进距离
- `GET /api/devices/{device_id}/point_cloud/analysis` - 获取分析结果
- `POST /api/devices/{device_id}/point_cloud/settings` - 更新分析参数

**计算逻辑:**
- `material_distance` = 最近物料点的 X 坐标 - camera_to_teeth
- `move_distance` = material_distance - 0.05m (接近偏移)

**相机距离发送:**
- 每 250ms 发送 `{MessageType=cameraCheck\nDistance=xxxx\n}` 到下位机

---

*Version: v2.1*
*Last Updated: 2026-04-15*