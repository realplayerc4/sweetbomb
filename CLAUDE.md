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

### Priority 2 - API 规范

```
rest-api/spec.md                 → API 契约、组件规范
rest-api/knowledge.md            → 技术知识、配置参考
```

### Priority 3 - 变更历史

```
rest-api/changelog.md            → 版本变更记录
memory-bank/progress.md          → 项目进度
```

---

## Module Boundaries (模块边界)

### 后端模块 (rest-api/)

| 模块 | 职责 | 允许修改 |
|------|------|----------|
| `routers/` | API 路由定义 | 是 |
| `services/` | 业务逻辑 | 是 |
| `models/` | 数据模型 | 是 |
| `tests/` | 单元测试 | 是 |

### 前端模块 (rest-api/ui/)

| 模块 | 职责 | 允许修改 |
|------|------|----------|
| `components/` | React 组件 | 是 |
| `hooks/` | 状态 Hooks | 是 |
| `services/` | API 调用 | 是 |

### 文档模块

| 模块 | 职责 | 修改规则 |
|------|------|----------|
| `memory-bank/` | AI 上下文 | 功能变更时更新 |
| `docs/agent/` | Agent 配置 | 只读 |
| `rest-api/spec.md` | API 规范 | 新增 API 时更新 |
| `rest-api/knowledge.md` | 技术知识 | 发现新知识时更新 |
| `rest-api/changelog.md` | 变更日志 | 每次提交时更新 |

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

## Workflow (工作流程)

### 新功能开发

```
1. 阅读 memory-bank/project-overview.md 确认符合目标
2. 阅读 rest-api/spec.md 了解 API 规范
3. 实现代码
4. 更新 rest-api/spec.md (如有 API 变更)
5. 更新 rest-api/changelog.md
6. 更新 memory-bank/progress.md
```

### Bug 修复

```
1. 阅读 rest-api/knowledge.md 查找相关解决方案
2. 定位问题代码
3. 实现修复
4. 更新 rest-api/changelog.md
5. 如发现新知识，更新 rest-api/knowledge.md
```

### 架构变更

```
1. 阅读 memory-bank/architecture.md 理解当前架构
2. 设计变更方案
3. 更新 memory-bank/architecture.md
4. 更新 rest-api/spec.md (如涉及 API)
5. 更新 rest-api/changelog.md
```

---

## Agent Roles (Agent 角色)

当使用 Multi-Agent 协作时，参考 `docs/agent/README.md`：

| 角色 | 对应文档 | 职责 |
|------|----------|------|
| 立法者 | spec.md | 定义规范 |
| 智库专家 | knowledge.md | 维护知识 |
| 执行官 | README.md | 落地规范 |
| 史官 | changelog.md | 记录变更 |

---

## Code Style (代码风格)

### Python

- 使用 ruff 进行 linting
- 使用 mypy 进行类型检查
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

## PM2 Services

| Port | Name | Type |
|------|------|------|
| 5173 | sweetbomb-5173 | Vite (Frontend) |
| 8000 | sweetbomb-8000 | FastAPI (Backend) |

**Terminal Commands:**
```bash
pm2 start ecosystem.config.cjs   # First time
pm2 start all                    # After first time
pm2 stop all / pm2 restart all
pm2 start {name} / pm2 stop {name}
pm2 logs / pm2 status / pm2 monit
pm2 save                         # Save process list
pm2 resurrect                    # Restore saved list
```

---

*Version: v1.0*
*Last Updated: 2025-02-24*
