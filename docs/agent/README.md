# Multi-Agent 协作系统

> 文档驱动的自动化协作框架 (融合 Vibe Coding 方法论)

---

## 系统架构

```
                    ┌─────────────────────────────────────┐
                    │            用户指令                  │
                    └─────────────────┬───────────────────┘
                                      │
                                      ▼
┌─────────────────────────────────────────────────────────────────────┐
│                     Vibe Coding 上下文层                             │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                      memory-bank/                            │   │
│  │  project-overview ──► 目标边界    architecture ──► 模块职责  │   │
│  │  tech-stack ────────► 技术决策    progress ────────► 进度追踪│   │
│  └─────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────┬───────────────────────────────┘
                                      │ 上下文注入
                                      ▼
┌─────────────────────────────────────────────────────────────────────┐
│                         Agent 协作层                                 │
│                                                                     │
│   ┌─────────────┐                         ┌─────────────┐          │
│   │   立法者    │ ─────── 触发 ─────────▶ │   执行官    │          │
│   │ legislator  │                         │  executor   │          │
│   │  spec.md    │                         │  readme.md  │          │
│   └──────┬──────┘                         └──────┬──────┘          │
│          │                                       │                  │
│          │ 查询/参考                    通知变更 │                  │
│          ▼                                       ▼                  │
│   ┌─────────────┐                         ┌─────────────┐          │
│   │  智库专家   │◀─────── 问题反馈 ───────│    史官     │          │
│   │ knowledge-  │                         │  historian  │          │
│   │  expert     │                         │changelog.md │          │
│   │knowledge.md │                         │             │          │
│   │experience.md│                         │ progress.md │          │
│   └─────────────┘                         └─────────────┘          │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
                                      │
                                      ▼
                    ┌─────────────────────────────────────┐
                    │            代码/项目                 │
                    └─────────────────────────────────────┘
```

---

## 核心文件

### 上下文层 (memory-bank/)

所有 Agent 共享的固定上下文：

| 文件 | 作用 |
|------|------|
| `project-overview.md` | 项目目标、非目标、核心实体 |
| `architecture.md` | 系统架构、模块职责 |
| `tech-stack.md` | 技术选型决策 |
| `progress.md` | 进度与里程碑 |

### 角色层 (docs/agent/)

| 文件 | 作用 |
|------|------|
| `legislator.md` | 立法者角色定义 |
| `knowledge-expert.md` | 智库专家角色定义 |
| `executor.md` | 执行官角色定义 |
| `historian.md` | 史官角色定义 |
| `experience.md` | 跨任务经验库 |
| `VIBE-AGENT-FUSION.md` | 融合架构详细说明 |

### 工作区层 (rest-api/)

| 文件 | 负责角色 | 职责 |
|------|----------|------|
| `spec.md` | 立法者 | 定义规范 |
| `knowledge.md` | 智库专家 | 维护知识 |
| `README.md` | 执行官 | 使用说明 |
| `changelog.md` | 史官 | 变更记录 |

---

## 角色清单

| 角色 | 文件 | 对应文档 | 必读上下文 | 职责 |
|------|------|----------|------------|------|
| 立法者 | `legislator.md` | `spec.md` | project-overview, architecture | 定义规范、制定规则 |
| 智库专家 | `knowledge-expert.md` | `knowledge.md` | tech-stack, experience | 维护知识、提供参考 |
| 执行官 | `executor.md` | `README.md` | project-overview, spec | 落地规范、维护状态 |
| 史官 | `historian.md` | `changelog.md` | progress, all docs | 记录变更、追踪版本 |

---

## 协作工作流

### 场景 1: 新增功能

```
1. 加载上下文: memory-bank/project-overview.md → 确认符合目标
2. 立法者: 在 spec.md 定义新功能规范
3. 触发 → 执行官: 更新 README.md 使用说明
4. 触发 → 史官: 在 changelog.md 记录 [Added]，更新 progress.md
5. 参考 → 智库专家: 补充技术背景 (如需要)
```

### 场景 2: 问题修复

```
1. 加载上下文: memory-bank/tech-stack.md → 理解技术背景
2. 智库专家: 查找 knowledge.md 相关知识
3. 发现问题 → 智库专家: 更新 FAQ
4. 修复完成 → 史官: 在 changelog.md 记录 [Fixed]
5. 新经验 → 智库专家: 更新 experience.md
```

### 场景 3: 架构变更

```
1. 加载上下文: memory-bank/architecture.md → 理解当前架构
2. 立法者: 更新 spec.md 和 architecture.md
3. 触发 → 执行官: 同步 README.md 目录结构
4. 触发 → 史官: 在 changelog.md 记录 [Changed]
5. 触发 → 智库专家: 更新 knowledge.md 技术栈文档
```

---

## 触发规则

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
    notify: [legislator, executor]
  architecture:
    notify: [all_agents]
  tech_stack:
    notify: [knowledge_expert]
  progress:
    notify: [historian]

# 模块文档变更触发
spec_md_change:
  source: rest-api/spec.md
  notify: [executor, historian]

knowledge_md_change:
  source: rest-api/knowledge.md
  notify: [historian]

readme_md_change:
  source: rest-api/README.md
  notify: [historian]

# 代码提交触发
code_commit:
  notify: [historian]
  update_experience: true
```

---

## 文件权限矩阵

| 角色 | memory-bank/* | spec.md | knowledge.md | README.md | changelog.md | experience.md |
|------|---------------|---------|--------------|-----------|--------------|---------------|
| 立法者 | R | **RW** | R | R | R | R |
| 智库专家 | R | R | **RW** | R | R | **RW** |
| 执行官 | R | R | R | **RW** | R | R |
| 史官 | **RW** (progress) | R | R | R | **RW** | R |

---

## 上下文加载顺序

```
Level 1: CLAUDE.md              → AI 行为准则
Level 2: memory-bank/*          → 项目固定上下文
Level 3: docs/agent/{role}.md   → 当前角色定义
Level 4: rest-api/spec.md       → API 规范
Level 5: rest-api/knowledge.md  → 技术知识 (按需)
```

---

## 使用方式

### 启动特定角色

```
# 作为立法者工作
1. 阅读 memory-bank/project-overview.md 确认目标边界
2. 阅读 memory-bank/architecture.md 确认模块边界
3. 阅读 docs/agent/legislator.md 遵循角色约束
4. 操作 rest-api/spec.md

# 作为史官记录变更
1. 阅读 memory-bank/progress.md 了解当前进度
2. 阅读 docs/agent/historian.md 遵循记录模板
3. 操作 rest-api/changelog.md 和 memory-bank/progress.md
```

### 协作检查清单

- [ ] 开始任务前是否阅读了 memory-bank 相关文档？
- [ ] 立法者变更 spec.md 后，是否通知执行官？
- [ ] 执行官更新 README.md 后，是否通知史官？
- [ ] 史官记录是否包含来源标注？
- [ ] 发现新经验是否更新 experience.md？

---

## 相关文档

- **融合架构详解**: `docs/agent/VIBE-AGENT-FUSION.md`
- **Vibe Coding 指南**: `docs/guides/vibe-coding.md`
- **AI 行为准则**: `CLAUDE.md`

---

## 版本历史

| 版本 | 日期 | 变更 |
|------|------|------|
| v2.0 | 2025-02-24 | 融合 Vibe Coding 方法论，新增 memory-bank 上下文层 |
| v1.0 | 2025-02-24 | 初始创建四角色系统 |

---

*系统版本: v2.0*
