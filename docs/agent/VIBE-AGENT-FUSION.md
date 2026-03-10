# Vibe Agent 融合架构

> 多Agent协作系统 + Vibe Coding 方法论 的缝合产物

---

## 核心融合理念

```
┌─────────────────────────────────────────────────────────────────┐
│                     Vibe Coding 上下文层                         │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │                    memory-bank/                          │   │
│  │  project-overview ──────► 目标边界 (Goals/Non-Goals)    │   │
│  │  architecture ──────────► 模块职责 (Module Boundaries)  │   │
│  │  tech-stack ────────────► 技术决策 (Decision Records)   │   │
│  │  progress ──────────────► 进度追踪 (Milestones)         │   │
│  └─────────────────────────────────────────────────────────┘   │
└───────────────────────────┬─────────────────────────────────────┘
                            │ 上下文注入
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│                     Multi-Agent 协作层                           │
│                                                                 │
│   ┌─────────────┐    触发     ┌─────────────┐                  │
│   │   立法者    │ ──────────► │   执行官    │                  │
│   │ legislator  │             │  executor   │                  │
│   │             │             │             │                  │
│   │ 写: spec.md │             │ 写:README.md│                  │
│   │ 读: memory- │             │ 读: memory- │                  │
│   │     bank/*  │             │     bank/*  │                  │
│   └──────┬──────┘             └──────┬──────┘                  │
│          │                           │                          │
│          │ 查询              通知变更 │                          │
│          ▼                           ▼                          │
│   ┌─────────────┐             ┌─────────────┐                  │
│   │  智库专家   │◄────────────│    史官     │                  │
│   │ knowledge-  │   问题反馈   │  historian  │                  │
│   │  expert     │             │             │                  │
│   │             │             │             │                  │
│   │ 写:knowledge│             │ 写:changelog│                  │
│   │ 读: memory- │             │ 读: memory- │                  │
│   │     bank/*  │             │     bank/*  │                  │
│   └─────────────┘             └─────────────┘                  │
│                                                                 │
└───────────────────────────┬─────────────────────────────────────┘
                            │ 执行输出
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│                       代码/项目层                                │
│                   rest-api/ + ui/                               │
└─────────────────────────────────────────────────────────────────┘
```

---

## 融合后的文档层次

```
ros2realsense/
│
├── CLAUDE.md                    # AI 行为准则 (融合入口)
│   └── 定义: 必读规则 + Agent触发规则 + 上下文加载顺序
│
├── memory-bank/                 # 固定上下文层 (Vibe Coding 核心)
│   ├── project-overview.md      # 所有 Agent 共享: 目标边界
│   ├── architecture.md          # 所有 Agent 共享: 架构约束
│   ├── tech-stack.md            # 所有 Agent 共享: 技术决策
│   └── progress.md              # 所有 Agent 共享: 进度追踪
│
├── docs/
│   └── agent/                   # Agent 角色定义 (Multi-Agent 核心)
│       ├── README.md            # 协作系统概述
│       ├── legislator.md        # 立法者: 依赖 memory-bank 确认边界
│       ├── knowledge-expert.md  # 智库专家: 依赖 memory-bank 提取知识
│       ├── executor.md          # 执行官: 依赖 memory-bank 落地规范
│       ├── historian.md         # 史官: 依赖 memory-bank 记录变更
│       └── experience.md        # 经验库: 跨任务学习
│
└── rest-api/                    # 模块级文档 (Agent 工作区)
    ├── spec.md                  # 立法者写入: API 规范
    ├── knowledge.md             # 智库专家写入: 技术知识
    ├── README.md                # 执行官写入: 使用说明
    └── changelog.md             # 史官写入: 变更记录
```

---

## Agent 行为准则 (融合版)

### 立法者 (Legislator)

```yaml
角色: 规范定义者
输入:
  - 必读: memory-bank/project-overview.md (确认目标边界)
  - 必读: memory-bank/architecture.md (确认模块边界)
  - 可选: rest-api/knowledge.md (技术参考)
输出:
  - 写入: rest-api/spec.md
  - 触发: 通知执行官、史官
约束:
  - 新规范必须符合 project-overview 中的 Goals
  - 不能定义 Non-Goals 中的功能
  - 规范变更需更新 architecture.md
```

### 智库专家 (Knowledge Expert)

```yaml
角色: 知识维护者
输入:
  - 必读: memory-bank/tech-stack.md (技术决策背景)
  - 必读: memory-bank/architecture.md (架构约束)
  - 参考: docs/agent/experience.md (历史经验)
输出:
  - 写入: rest-api/knowledge.md
  - 更新: docs/agent/experience.md (新经验)
约束:
  - 知识必须与 tech-stack 决策一致
  - 新经验需记录到 experience.md
```

### 执行官 (Executor)

```yaml
角色: 规范落地者
输入:
  - 必读: memory-bank/project-overview.md (理解目标)
  - 必读: rest-api/spec.md (遵循规范)
  - 参考: rest-api/knowledge.md (技术参考)
输出:
  - 写入: rest-api/README.md
  - 实现: 代码变更
  - 触发: 通知史官
约束:
  - 实现必须严格遵循 spec.md
  - 偏离规范需先请求立法者更新
```

### 史官 (Historian)

```yaml
角色: 变更记录者
输入:
  - 必读: memory-bank/progress.md (当前进度)
  - 接收: 所有 Agent 的变更通知
输出:
  - 写入: rest-api/changelog.md
  - 更新: memory-bank/progress.md
约束:
  - 每次变更必须记录
  - 里程碑完成需更新 progress.md
```

---

## 触发规则 (融合版)

```yaml
# Vibe Coding 规划触发
plan_mode_trigger:
  conditions:
    - 任务涉及多文件变更
    - 任务影响架构
    - 任务超出当前 spec 边界
  actions:
    - 进入 Plan Mode
    - 输出详细计划
    - 等待用户确认

# memory-bank 变更触发
memory_bank_change:
  project_overview:
    notify: [legislator, executor]  # 目标变更影响规范和实现
  architecture:
    notify: [all_agents]            # 架构变更影响所有 Agent
  tech_stack:
    notify: [knowledge_expert]      # 技术变更影响知识库
  progress:
    notify: [historian]             # 进度更新由史官维护

# 模块文档变更触发 (保持原有逻辑)
spec_md_change:
  source: rest-api/spec.md
  notify: [executor, historian]

knowledge_md_change:
  source: rest-api/knowledge.md
  notify: [historian]  # 可能需要更新 experience

readme_md_change:
  source: rest-api/README.md
  notify: [historian]

# 代码提交触发
code_commit:
  notify: [historian]
  condition: 提交信息包含 [fix] 或 [feat]
  update_experience: true
```

---

## 上下文加载流程 (融合版)

```
用户发起任务
      │
      ▼
┌─────────────────┐
│  CLAUDE.md      │ ◄── Level 1: 行为准则
│  确定任务类型   │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ memory-bank/    │ ◄── Level 2: 固定上下文
│ project-overview│     确认目标边界
│ architecture    │     确认模块边界
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ 确定需要的 Agent│ ◄── Level 3: Agent 选择
│ 加载角色配置    │     读取 docs/agent/{role}.md
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ 模块级文档      │ ◄── Level 4: 工作区上下文
│ spec.md         │     (按需加载)
│ knowledge.md    │
└────────┬────────┘
         │
         ▼
    执行任务
         │
         ▼
┌─────────────────┐
│ 更新相关文档    │ ◄── Level 5: 输出
│ 触发下游 Agent  │
└─────────────────┘
```

---

## 融合后的工作流示例

### 场景: 新增 API 端点

```
1. 用户请求: "添加 /api/record 端点"

2. CLAUDE.md 匹配: 新功能开发 → 进入规划流程

3. 加载上下文:
   - memory-bank/project-overview.md
     → 检查: 是否符合 Goals?
     → 发现: "录制功能" 在 Non-Goals 中 ❌

4. 输出:
   "此功能在 project-overview.md 的 Non-Goals 中定义为不支持:
    - 录制与回放（非实时场景）
    如需添加，请先更新 project-overview.md"

5. 用户确认更新目标后:
   - 立法者: 在 spec.md 定义新端点规范
   - 智库专家: 在 knowledge.md 补充录制技术知识
   - 执行官: 实现代码，更新 README.md
   - 史官: 记录 changelog，更新 progress.md
```

### 场景: Bug 修复

```
1. 用户报告: "WebRTC 连接经常断开"

2. 加载上下文:
   - memory-bank/architecture.md → 确认 webrtc_manager 模块职责
   - rest-api/knowledge.md → 查找 WebRTC 相关知识

3. 发现 knowledge.md 有相关条目:
   "6.2 WebRTC 连接失败: 检查 ICE 候选、防火墙、编解码器"

4. 执行修复

5. 更新文档:
   - 史官: changelog.md 记录 [Fixed]
   - 智库专家: 如发现新知识，更新 knowledge.md 和 experience.md
```

---

## 融合优势

| 维度 | 原 Multi-Agent | 原 Vibe Coding | 融合后 |
|------|---------------|----------------|--------|
| 上下文管理 | 每次重新说明 | 文档固定 | memory-bank 提供固定基线 |
| 角色职责 | 有定义 | 无 | Agent 角色明确 |
| 规划流程 | 无 | Plan Mode | Agent 触发规划 |
| 变更追踪 | changelog | progress | 两者结合 |
| 经验积累 | 无 | experience.md | 跨任务学习 |
| 触发机制 | 文件级 | 无 | 多级触发 |

---

## 快速参考卡

```
┌────────────────────────────────────────────────────────────┐
│                    Vibe Agent 速查表                        │
├────────────────────────────────────────────────────────────┤
│  开始任务前必读:                                            │
│    1. CLAUDE.md           → 行为准则                        │
│    2. memory-bank/*       → 项目上下文                      │
│    3. docs/agent/{role}   → 当前角色定义                    │
│    4. rest-api/spec.md    → API 规范                       │
├────────────────────────────────────────────────────────────┤
│  完成任务后必写:                                            │
│    • 立法者 → spec.md                                      │
│    • 智库专家 → knowledge.md, experience.md                │
│    • 执行官 → README.md, 代码                              │
│    • 史官 → changelog.md, progress.md                      │
├────────────────────────────────────────────────────────────┤
│  触发规则:                                                  │
│    • spec.md 变更 → 通知 executor, historian               │
│    • memory-bank 变更 → 通知所有 Agent                     │
│    • 代码提交 → 通知 historian                             │
└────────────────────────────────────────────────────────────┘
```

---

*Version: v1.0*
*Created: 2025-02-24*
*融合: Multi-Agent System + Vibe Coding Methodology*
