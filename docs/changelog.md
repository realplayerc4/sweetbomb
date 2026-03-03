# 变更日志 (Changelog)

> 本文档记录所有显著变更、功能更新和问题修复。
> 遵循 [Keep a Changelog](https://keepachangelog.com/) 格式和 [Semantic Versioning](https://semver.org/)。

---

## 变更类型标签

| 标签 | 说明 |
|------|------|
| `[Added]` | 新功能 |
| `[Changed]` | 现有功能的变更 |
| `[Deprecated]` | 即将移除的功能 |
| `[Removed]` | 移除的功能 |
| `[Fixed]` | 问题修复 |
| `[Security]` | 安全修复 |
| `[Docs]` | 文档更新 |
| `[Refactor]` | 代码重构 |

---

## [Unreleased]

### Planned

- [ ] 铲糖任务成功率统计与优化建议
- [ ] 远程节点管理功能上线
- [ ] 多语言支持 (i18n)
- [ ] 数据导出功能 (CSV/JSON)

---

## [1.2.0] - 2026-03-03

### [Added]

- **铲糖行为树完整实现**
  - `DumpAndReturn` 连贯动作节点（导航A点→举升→导航B点→倾倒→倒退→归零）
  - `ScoopAndReturn` 连贯动作节点（前进→翻转铲子→倒退）
  - `AnalyzeSugarDistance` 距离分析节点
  - `CheckShovelFlat` 车铲平整度检查节点
  - `ReturnToHome` 回桩充电节点

- **按钮确认安全机制**
  - 倾倒动作需要前端按钮确认
  - 30秒超时保护
  - Socket.IO 事件通知

- **IMU 数据处理**
  - 3D 姿态估计可视化
  - Roll/Pitch 实时计算
  - 重力向量可视化

- **3x2 完美网格布局**
  - 仪表盘黄金比例布局
  - 视觉中心偏移（+8px 黄金胶囊偏移）
  - 字重加粗优化
  - 品牌色 `#FD802E` 注入

### [Changed]

- **点云渲染优化**
  - 粒子大小调整为 4.5
  - 探测范围扩展至 6m
  - 使用 `NormalBlending` 避免 `AdditiveBlending` 过曝
  - `OrthographicCamera` 俯视图严格锁定视锥
  - `frustumCulled: false` 解决视角拉远点云消失问题

- **倾倒流程重构**
  - 废弃单一卸载点配置 (`dump_point`)
  - 引入双卸载点设计 (`dump_point_a` 和 `dump_point_b`)
  - A点：等待/归零位置，B点：倾倒位置
  - 新增位置保护：到达A点才能举升，到达B点才能倾倒
  - 新增归零保护：回到A点才能归零电机，防止空中归零导致物料掉落

- **结构优化**
  - 工程目录全链路物理分离
  - 前端组件模块化重构
  - 状态管理集中化

### [Fixed]

- 修复 Socket.IO 连接 403 错误
- 修复点云在俯视角下消失问题
- 修复铲糖任务高度阈值判断逻辑
- 修复倾倒动作安全确认超时问题

### [Refactor]

- `bt_nodes.py` 核心节点重设计
- `bt_scoop_action.py` 铲糖动作优化
- `bt_dump_action.py` 倾倒动作重构
- 前端组件解耦

### [Docs]

- 新增行为树规范文档
- 更新 API 接口文档
- 完善铲糖任务配置说明

---

## [1.1.0] - 2026-02-20

### [Added]

- **行为树基础框架**
  - `BTNode` 基类设计
  - `ActionNode` 动作节点
  - `ConditionNode` 条件节点
  - `SequenceNode` 序列节点
  - `SelectorNode` 选择节点
  - `RepeatNode` 重复节点
  - `DecoratorNode` 装饰器节点
  - `InverterNode` 反转节点

- **3D 点云可视化**
  - Three.js 集成
  - 深度图转点云
  - 实时渲染
  - 性能优化

- **任务管理系统**
  - 任务状态机
  - 并发控制
  - 进度追踪

### [Changed]

- 前端架构重构
- 组件库升级
- 状态管理优化

### [Fixed]

- WebRTC 连接稳定性
- 内存泄漏问题

---

## [1.0.0] - 2026-02-10

### [Added]

- **基础架构**
  - FastAPI 后端框架
  - React 前端框架
  - WebRTC 视频流
  - Socket.IO 实时通信

- **RealSense 集成**
  - 设备发现与枚举
  - 流启动/停止
  - 传感器选项配置
  - IMU 数据读取

- **Web 界面**
  - 视频流显示
  - 设备配置面板
  - 参数调节滑块
  - 状态监控

### [Security]

- CORS 配置
- 输入验证
- 错误信息脱敏

---

## 版本历史

```
v1.2.0 (2026-03-03)  铲糖行为树完整实现，倾倒流程重构，UI 黄金版本
v1.1.0 (2026-02-20)  行为树基础框架，3D 点云可视化，任务管理系统
v1.0.0 (2026-02-10)  基础架构，RealSense 集成，Web 界面
```

---

## 废弃功能

| 版本 | 功能 | 替代方案 | 移除日期 |
|------|------|----------|----------|
| 1.2.0 | `dump_point` 配置 | `dump_point_a` + `dump_point_b` | 待定 |

---

*文档版本: v1.2*
*最后更新: 2026-03-03*
