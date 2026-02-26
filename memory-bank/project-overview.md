# Project Overview

> 项目目标、非目标与核心实体定义

---

## Project Identity

**Name**: RealSense Web Monitor
**Type**: Hardware Monitoring Platform
**Tech Stack**: FastAPI + React + WebRTC + RealSense

---

## Goals

1. **实时监控**: 提供 RealSense 设备的实时视频流监控能力
2. **参数配置**: 支持在线调整传感器参数（曝光、增益、激光功率等）
3. **低延迟传输**: 基于 WebRTC 实现毫秒级视频传输
4. **3D 可视化**: 集成 Three.js 提供点云实时渲染
5. **任务管理**: 支持可扩展的后台任务系统（如对象检测、数据采集）

---

## Non-Goals

1. **不支持的设备**: 非 Intel RealSense D400 系列设备
2. **不提供的功能**:
   - 录制与回放（非实时场景）
   - 多设备同步采集
   - 云端部署（仅本地/局域网）
3. **不涉及领域**:
   - 移动端原生应用
   - 边缘计算设备优化

---

## Core Entities

| 实体 | 描述 | 关键属性 |
|------|------|----------|
| **Device** | RealSense 设备 | id, name, serial, firmware |
| **Sensor** | 传感器组件 | id, name, options |
| **Stream** | 数据流 | type (RGB/Depth), resolution, fps |
| **Task** | 后台任务 | id, type, status, progress |

---

## Success Metrics

- 视频流延迟 < 100ms
- API 响应时间 < 50ms (P99)
- 支持 2 路并发视频流
- 支持最多 4 个并发后台任务

---

## Stakeholders

| 角色 | 关注点 |
|------|--------|
| 开发者 | API 文档、代码可维护性 |
| 运维人员 | 部署简单性、日志完整 |
| 终端用户 | 界面响应、视频流畅度 |

---

*Version: v1.1 (Industrial Redesign)*
*Last Updated: 2026-02-26*
