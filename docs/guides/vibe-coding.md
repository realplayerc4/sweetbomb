# Vibe Coding 方法论

> 规划驱动 + 上下文固定 + 模块化的 AI 协作开发范式

---

## Core Principles

### 1. 规划优先 (Plan First)

在编写代码之前，必须完成规划：

```
1. 理解需求 → 明确目标与非目标
2. 探索代码 → 了解现有架构
3. 设计方案 → 输出实施计划
4. 用户确认 → 获取批准后执行
```

### 2. 上下文固定 (Fixed Context)

通过文档建立稳定的上下文基线：

```
memory-bank/          → 项目级上下文
├── project-overview  → 目标边界
├── architecture      → 模块职责
├── tech-stack        → 技术决策
└── progress          → 进度追踪
```

### 3. 模块化协作 (Modular Collaboration)

每个模块独立维护文档：

```
rest-api/
├── spec.md       → API 契约
├── knowledge.md  → 技术知识
├── README.md     → 使用说明
└── changelog.md  → 变更记录
```

---

## Documentation Hierarchy

```
┌─────────────────────────────────────────────────┐
│                   CLAUDE.md                      │  ← AI 行为准则
│              (AI Behavior Rules)                 │
└────────────────────────┬────────────────────────┘
                         │
         ┌───────────────┼───────────────┐
         ▼               ▼               ▼
┌─────────────┐  ┌─────────────┐  ┌─────────────┐
│ memory-bank │  │   Module    │  │    Docs     │
│  (Context)  │  │   Specs     │  │   (Guides)  │
└─────────────┘  └─────────────┘  └─────────────┘
```

---

## Workflow Patterns

### Pattern 1: Feature Development

```yaml
trigger: 新功能请求
steps:
  1. 读取 memory-bank/project-overview.md → 确认符合目标
  2. 读取 memory-bank/architecture.md → 确认模块边界
  3. 读取 rest-api/spec.md → 了解 API 规范
  4. 设计方案并输出计划
  5. 用户确认后实现
  6. 更新相关文档
```

### Pattern 2: Bug Fix

```yaml
trigger: Bug 报告
steps:
  1. 读取 rest-api/knowledge.md → 查找已知问题
  2. 定位问题代码
  3. 实现修复
  4. 更新 rest-api/changelog.md
  5. 如发现新知识 → 更新 knowledge.md
```

### Pattern 3: Architecture Change

```yaml
trigger: 架构调整
steps:
  1. 读取 memory-bank/architecture.md
  2. 设计变更方案
  3. 更新 architecture.md
  4. 更新相关 spec.md
  5. 通知所有模块更新
```

---

## Multi-Agent Roles

基于四角色协作模型：

| 角色 | 职责 | 输出文档 |
|------|------|----------|
| 立法者 (Legislator) | 定义规范 | spec.md |
| 智库专家 (Knowledge Expert) | 维护知识 | knowledge.md |
| 执行官 (Executor) | 落地规范 | README.md |
| 史官 (Historian) | 记录变更 | changelog.md |

详见 `docs/agent/README.md`

---

## Best Practices

### 1. 文档即代码

- 文档与代码同步更新
- 使用版本控制追踪变更
- 文档变更触发下游通知

### 2. 渐进式上下文加载

```
Level 1: CLAUDE.md          → 行为准则
Level 2: project-overview   → 项目边界
Level 3: architecture       → 架构细节
Level 4: spec/knowledge     → 模块规范
```

### 3. 触发式更新

```yaml
triggers:
  spec_change:
    notify: [executor, historian]
  code_commit:
    notify: [historian]
  bug_fix:
    update: [knowledge, changelog]
```

---

## Anti-Patterns

### 避免的做法

1. **跳过规划**: 直接写代码不先理解上下文
2. **文档滞后**: 代码改了文档没更新
3. **过度抽象**: 为假设的未来需求设计
4. **重复上下文**: 每次都重新解释项目背景

### 推荐的做法

1. **先读后写**: 阅读相关文档再开始工作
2. **增量更新**: 小步快跑，频繁更新文档
3. **单一职责**: 每个文档只负责一件事
4. **可追溯**: 变更都有 changelog 记录

---

## Experience Library

从交互中学习的经验存储在 `docs/agent/experience.md`：

- 代码开发经验 (CODE)
- 工具使用经验 (TOOL)
- 任务处理经验 (TASK)
- 工程方法论 (METHOD)
- 部署运维经验 (DEPLOY)

---

## References

- Multi-Agent 协作系统: `docs/agent/README.md`
- API 规范: `rest-api/spec.md`
- 技术知识: `rest-api/knowledge.md`
- 经验库: `docs/agent/experience.md`

---

*Version: v1.0*
*Last Updated: 2025-02-24*
