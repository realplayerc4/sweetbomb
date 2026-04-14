# Progress

> 项目进度与里程碑记录

---

## Current Status

**Phase**: 机器人通讯协议完善与系统守护化 (Phase 7)
**Last Milestone**: TCP 通讯协议完善、点云分析回调修复、systemd 自启动配置
**Next Milestone**: 前端点云参数同步、cameraCheck 距离发送验证

---

## Completed Milestones

### 2026-04-14: 系统守护化配置

- [x] **[systemd 服务]**: 创建 sweetbomb-backend/frontend 服务文件
- [x] **[install-service.sh]**: 自动安装脚本，配置开机自启动
- [x] **[文档更新]**: CLAUDE.md、README.md 移除 PM2，改用 systemd
- [x] **[清理]**: 删除 PM2 相关命令文件和配置

### 2026-04-14: TCP 通讯协议完善

- [x] **[按钮互锁]**: pick/drop/charge 按钮互锁，taskFinish 解锁
- [x] **[暂停任务]**: pauseTask/pauseCancel 消息类型实现
- [x] **[导航按钮]**: allPick/allDrop 任务端点
- [x] **[TCP 回复规则]**: 仅 taskFinish 回复下位机，其他不回复
- [x] **[举升范围]**: 修正为 60-124 度

### 2026-04-14: 点云分析修复

- [x] **[回调修复]**: stream_controller.py 调用 analysis_result_callback
- [x] **[API 端点]**: 新增 `/settings` API 同步前端参数
- [x] **[参数存储]**: rs_manager 存储 camera_to_teeth 等参数
- [x] **[序列化修复]**: numpy.float32 转 Python float
- [x] **[标签修改]**: "铲齿深度" → "相机高度"

### 2026-04-10: 车辆速度实时显示

- [x] **[机器人面板]**: "设定速度"改为"车辆速度"，使用 status.speed 实时更新

### 2026-04-08: 实时遥测数据显示优化

- [x] **[机器人面板]**: 添加举升/旋转柱状条实时显示
  - 举升范围 10-290mm，旋转范围 -50-80mm
  - 石墨橙色填充，按百分比显示
  - 底部数值保留一位小数 + mm 单位
- [x] **[地图面板]**: X, Y, A 坐标实时显示于状态胶囊
- [x] **[界面简化]**: 移除 Mode 显示，电量字体加大
- [x] **[数据频率]**: 轮询间隔优化为 500ms

### 2026-02-27: 仪表盘黄金版 1.2.0 (Phase 6 - Final Polish)

- [x] **[排版升级]**: 实现 3x2 完美网格布局，统一 30px 响应式间距。
- [x] **[视觉精修]**: 全局胶囊 10px 黄金偏移与字重加粗，修复 Slice View 图标色彩失配。
- [x] **[性能/数据]**: 调优粒子大小至 4.5，拓展探测距离至 6m，并完整解耦空间配置与地图组件。
- [x] **[脚部增强]**: 页脚状态栏实现物理量动态渲染与美化。

### 2026-02-26: 工程目录重构与全链路分离 (Phase 6 - v2)

- [x] **[应用层独立]**: 彻底打散 `rest-api` 包裹层，将 `app/` 和 `tests/` 提升至项目根目录。
- [x] **[运维层归档]**: 创建 `deploy/` 目录，收容所有分布在根目录的部署与远程修复脚本。
- [x] **[规范对齐]**: 将 API 规范 (`spec.md`, `knowledge.md`) 整合至 `docs/api/`。
- [x] **[Bug 修复]**: 彻底解决 Three.js 点云粒子由于深度测试 (`depthWrite`) 冲突导致的不显示问题。
- [x] **[环境治理]**: 修正 `.gitignore` 并清洗历史提交，剔除误录入的 `venv` 等巨型依赖包。

### 2026-02-26: 视觉控制台与数据拓展 (Phase 5)

- [x] 后台集成 `psutil` 并抓取真实硬件 IMU 数据流
- [x] 基于数学解算实时 Roll、Pitch 姿态
- [x] 重写 WebGL Shader 实现正交鸟瞰投影 (BEV Context)
- [x] 应用极简工业风 (Formant-style) UI 视觉重构
- [x] `localStorage` 并入控制面板，实现参数缓存持久化

### 2025-02-24: 项目结构重组

- [x] 创建 memory-bank 记忆库结构
- [x] 整合文档层次结构
- [x] 迁移 .agent 到 docs/agent

### 2025-02-24: 任务管理系统

- [x] 后端任务 API (CRUD + 生命周期控制)
- [x] 任务基类与注册机制
- [x] 对象检测任务实现
- [x] 前端任务面板 UI

### 2025-02-18: 核心功能

- [x] 设备发现与管理
- [x] WebRTC 视频流
- [x] Socket.IO 元数据推送
- [x] 点云 3D 可视化

---

## In Progress

### 待完成

- [ ] 前端 SliceView 修改参数时调用 settings API 同步后端
- [ ] cameraCheck 距离发送验证（250ms 间隔）
- [ ] material_distance 计算参数调整（camera_to_teeth 当前 1.044m）

---

## Planned

- [ ] 前端参数同步到后端（实时生效）
- [ ] 多设备支持优化
- [ ] 录制功能 (非实时)

---

## Blockers

*当前无阻塞项*

---

## Metrics

| 指标 | 当前值 | 目标值 |
|------|--------|--------|
| TCP 延迟 | ~50ms | < 100ms |
| API P99 | ~30ms | < 50ms |
| 点云分析频率 | 4Hz | 4Hz |

---

*Last Updated: 2026-04-14*
