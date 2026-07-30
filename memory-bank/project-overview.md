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
6. **行为树控制**: 实现铲糖自动化任务的完整行为树系统

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

## Behavior Tree System

### 铲糖任务行为树结构

```
RepeatNode (SugarHarvestMainLoop, max_count=cycles)
└── SequenceNode (FullHarvestCycle)
    ├── NavigateToSugarPoint          [导航到取糖点]
    ├── CheckShovelFlat               [检查车铲是否放平]
    ├── AnalyzeSugarDistance          [分析糖堆距离和高度]
    └── SelectorNode (HeightCheck)
        ├── 路径 A: 高度足够 → SequenceNode
        │   ├── CalculateApproachDistance  [计算前进距离]
        │   ├── ScoopAndReturn             [连贯动作：前进→翻转→倒退]
        │   └── DumpAndReturn              [连贯动作：导航A点→举升→导航B点→倾倒→倒退→归零]
        └── 路径 B: 高度不足 → ReturnToHome [回桩]
```

### 黑板数据流

```yaml
NodeContext.blackboard:
├── nav_point_position       (NavigateToSugarPoint 记录，ScoopAndReturn 使用)
├── distance_analysis        (AnalyzeSugarDistance 记录)
├── approach_distance        (CalculateApproachDistance 记录，ScoopAndReturn 使用)
└── dump_point_a_final       (DumpAndReturn 内部使用)
```

---

## Success Metrics

| 指标 | 当前值 | 目标值 |
|------|--------|--------|
| 视频延迟 | ~80ms | < 100ms |
| API P99 | ~30ms | < 50ms |
| 并发任务 | 4 | 4 |
| 测试覆盖率 | 60% | 80% |

---

## Stakeholders

| 角色 | 关注点 |
|------|--------|
| 开发者 | API 文档、代码可维护性 |
| 运维人员 | 部署简单性、日志完整 |
| 终端用户 | 界面响应、视频流畅度 |

---

*Version: v1.2 (Behavior Tree Enhanced)*
*Last Updated: 2026-03-03*
