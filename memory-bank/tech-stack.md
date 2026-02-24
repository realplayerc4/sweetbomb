# Tech Stack

> 技术选型与版本要求

---

## Backend Stack

| 技术 | 版本 | 用途 |
|------|------|------|
| Python | 3.10+ | 运行时环境 |
| FastAPI | 0.115+ | Web 框架 |
| Uvicorn | 0.30+ | ASGI 服务器 |
| pyrealsense2 | 2.56+ | RealSense SDK |
| aiortc | 1.6+ | WebRTC 实现 |
| python-socketio | 5.11+ | WebSocket |
| Pydantic | 2.0+ | 数据验证 |

---

## Frontend Stack

| 技术 | 版本 | 用途 |
|------|------|------|
| Node.js | 18+ | 运行时环境 |
| React | 18+ | UI 框架 |
| TypeScript | 5.0+ | 类型系统 |
| Vite | 5.0+ | 构建工具 |
| TailwindCSS | 3.4+ | 样式框架 |
| Three.js | 0.160+ | 3D 渲染 |
| Socket.IO Client | 4.7+ | 实时通信 |

---

## Hardware Requirements

| 要求 | 规格 |
|------|------|
| 设备 | Intel RealSense D400 系列 |
| 接口 | USB 3.0+ |
| 系统 | Ubuntu 20.04/22.04 |
| 内存 | 最低 4GB |

---

## Development Tools

| 工具 | 用途 |
|------|------|
| ruff | Python Linter |
| mypy | 类型检查 |
| pytest | 单元测试 |
| ESLint | JS/TS Linter |

---

## Decision Records

### DR-001: 选择 WebRTC 而非 MJPEG

**原因**:
- WebRTC 延迟 < 100ms，MJPEG 通常 > 200ms
- WebRTC 支持双向通信，便于扩展
- 浏览器原生支持，无需插件

### DR-002: 选择 FastAPI 而非 Flask

**原因**:
- 原生异步支持，适合 I/O 密集场景
- 自动生成 OpenAPI 文档
- Pydantic 集成，类型安全

### DR-003: 选择 Three.js 而非原生 WebGL

**原因**:
- 更高层次的抽象，开发效率高
- 社区活跃，文档完善
- 内置相机控制、着色器支持

---

*Version: v1.0*
*Last Updated: 2025-02-24*
