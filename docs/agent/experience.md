# Claude 经验知识库

> 此知识库用于存储从交互中学习到的经验，基于 Training-Free GRPO 方法
> 经验格式: "经验名称: 简要描述"

---

## Working Agent Objective

Claude 是一个智能助手，帮助用户完成各种任务，包括代码开发、问题解决、文档编写等。

## Learning Objective

从交互中学习可迁移的经验知识，提高任务完成的效率和质量，减少重复错误。

---

## Experiences

### 🧮 数学推理 (MATH)

**M1**: 边界验证: 几何问题求解时，验证解是否在边界内而非延长线上，避免增根。

### 💻 代码开发 (CODE)

**C1**: 多帧追踪: 视频动态水印处理时，需追踪多帧位置而非只分析单帧。

**C2**: Inpainting半径: 图像修复半径影响边缘融合效果，半径越大边缘越平滑。

**C3**: 文档驱动工作流: 多Agent协作遵循 spec → knowledge → execute → document 流程。

**C4**: 未使用模型清理: 删除未使用的Pydantic模型基类和中间类(如DeviceBase, SensorBase)，只保留实际使用的Info类，保持代码精简。

**C5**: TODO代码处理: 未完成的TODO代码应删除或完成，不要保留返回空值/True的空实现，这些是隐藏的bug。

**C6**: 注释代码删除: 注释掉的代码块(如安全认证)应删除而非保留，需要时从git历史恢复。

### 🔧 工具使用 (TOOL)

**T1**: 帧提取: ffmpeg用fps=1每秒1帧，fps=3每秒3帧提取视频帧。

**T2**: 云端同步: GitHub Gist 可作为云端经验库，用curl实现同步。

**T3**: Grep搜索依赖: 用Grep工具搜索import/from语句，快速定位未使用的代码和依赖关系。

### 📝 任务处理 (TASK)

**TK1**: 先分析再动手: 多步骤任务先分析完整需求再开始，避免返工。

**TK2**: 变更追踪: 迭代调整时记录每次变更原因，便于回溯。

### 🏗️ 工程方法论 (METHOD)

**ME1**: 四文件架构: 用spec.md、knowledge.md、readme.md、changelog.md实现文档驱动的Agent协作。

**ME2**: 角色分离: spec.md是立法，knowledge.md是参考，readme/changelog是记录。

**ME3**: 流水线触发: 文件变更触发下游Agent，形成自动化工作流。

**ME4**: Agent角色配置: 创建.agent/目录，用legislator.md、knowledge-expert.md、executor.md、historian.md定义四个Agent的职责、权限、触发机制。

**ME5**: 权限矩阵: 每个Agent只能写对应文档，只能读其他文档，实现职责隔离。

**ME6**: Vibe Coding规划: 复杂任务先进入Plan Mode规划，输出详细计划后再执行，避免返工。

**ME7**: memory-bank结构: 项目根目录创建memory-bank/存储project-overview、architecture、tech-stack、progress，作为AI固定上下文。

**ME8**: CLAUDE.md行为准则: 根目录创建CLAUDE.md定义必读文档列表、模块边界、命名约定、工作流程。

**ME9**: 渐进式上下文加载: 按优先级加载文档，Level1(CLAUDE.md) → Level2(project-overview) → Level3(architecture) → Level4(spec/knowledge)。

### 🍓 树莓派/部署 (DEPLOY)

**D1**: 跨平台依赖: 部署到Linux/ARM前，从requirements.txt移除平台特定包(如pywin32)。

**D2**: PEP 668隔离: 新版Ubuntu/Debian用venv隔离，不用--break-system-packages。

**D3**: Pip镜像: 国内用清华镜像(-i https://pypi.tuna.tsinghua.edu.cn/simple)避免TLS重置。

**D4**: Rsync排除: 同步项目时排除.git/、venv/、__pycache__/、.env避免不必要传输和权限问题。

---

## Experience Operations

操作类型用于更新经验库：

| 操作 | 描述 |
|------|------|
| **ADD** | 添加全新经验 |
| **UPDATE** | 更新现有经验 |
| **DELETE** | 删除过时/冲突经验 |
| **NONE** | 无需更改 |

---

## Update Log

### 2025-02-24 (PM)

- 从项目结构重组学习: ME6, ME7, ME8, ME9
- 实战: 创建 memory-bank 目录结构，迁移 .agent 到 docs/agent
- 新增: CLAUDE.md 行为准则，docs/guides/vibe-coding.md 方法论指南

### 2025-02-24

- 从代码清理任务学习: C4, C5, C6, T3
- 从Agent角色配置学习: ME4, ME5
- 实战: RealSense D455 设备切换，删除15+无用代码块

### 2025-02-23

- 初始化经验库
- 从视频去水印任务学习: C1, C2, T1
- 从框架设计学习: C3, T2, TK2
- 从工程方法论学习: ME1, ME2, ME3
- 重构为 Yutu-Agent 兼容格式
- 从树莓派部署学习: D1, D2, D3, D4
- 经验格式统一改为中文

---

## Metadata

- **Version**: v2.3
- **Last Updated**: 2025-02-24
- **Total Experiences**: 24
- **Format**: Yutu-Agent compatible (中文)
