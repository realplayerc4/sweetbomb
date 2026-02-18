# 项目改进建议

本文档记录了代码审查中发现的问题和建议的改进项。

## 前端组件 (Frontend Components)

### DepthView.tsx / RGBView.tsx

#### 高优先级
- [ ] **修复 Tailwind CSS 类名拼写错误** (DepthView.tsx:38)
  - 当前: `hover:scale-135`
  - 应改为: `hover:scale-[1.35]` 或其他有效的 scale 值

- [ ] **移除重复的 transform 样式**
  - 当前: className 包含 `transition-transform`，同时有内联样式 `transform: 'scale(1.25)'`
  - 建议: 统一使用一种方式，推荐使用 Tailwind 类名

- [ ] **统一两个组件的样式行为**
  - RGBView.tsx 缺少 hover 效果
  - 建议: 确保两个组件的行为一致

#### 中优先级
- [ ] **利用 `isActive` prop 控制视频状态**
  - 当前: `isActive` 仅用于显示指示灯
  - 建议: 可以在 `isActive=false` 时暂停视频播放以节省资源

## React Hooks

### useRobotConnection.ts

#### 高优先级
- [ ] **修复 ICE 候选者的竞态条件** (lines 152-159)
  ```typescript
  // 当前使用 setTimeout 等待 1 秒，不可靠
  setTimeout(async () => { ... }, 1000);

  // 建议使用 Promise 或事件驱动的方式
  ```

- [ ] **在设置新流前清理旧的 MediaStream tracks**
  ```typescript
  // 在 startConnection 开始时添加:
  rgbStream?.getTracks().forEach(track => track.stop());
  depthStream?.getTracks().forEach(track => track.stop());
  ```

#### 中优先级
- [ ] **为 `metadata_update` 添加 TypeScript 类型定义** (line 40)
  ```typescript
  // 当前: (data: any)
  // 建议定义接口:
  interface MetadataStreams {
    depth?: {
      point_cloud?: {
        vertices: string; // base64 encoded
      };
    };
  }

  interface MetadataUpdate {
    metadata_streams?: MetadataStreams;
  }

  socket.current.on('metadata_update', (data: MetadataUpdate) => { ... });
  ```

- [ ] **改进 track 跟踪逻辑** (lines 104-132)
  - 当前 fallback 逻辑可能因 track 到达顺序不同而失败
  - 建议: 基于 track kind 或其他更可靠的标识符

#### 低优先级
- [ ] **使用 `unknown` 替代 `any` 类型** (line 163)
  ```typescript
  // 当前: catch (e: any)
  // 建议: catch (e: unknown)
  ```

## .gitignore

- [ ] **添加 `node_modules/` 到忽略列表**
- [ ] **移除多余的空行** (lines 38-39)
- [ ] **验证 `.config/` 是否应该被忽略**

## 通用建议

### 安全性
- [ ] 检查 WebRTC 相关代码是否有潜在的 XSS 或注入漏洞
- [ ] 验证 Socket.IO 连接的身份验证机制

### 性能
- [ ] 考虑对点云数据进行节流或采样，避免过快更新
- [ ] 添加性能监控和错误边界

### 可维护性
- [ ] 添加单元测试
- [ ] 考虑将常量提取到配置文件
- [ ] 添加更详细的 JSDoc 注释
