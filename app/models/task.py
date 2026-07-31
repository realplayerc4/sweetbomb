"""任务系统数据模型。"""

from enum import Enum
from typing import Any, Dict, List, Optional
from datetime import datetime
from pydantic import BaseModel, Field


class TaskStatus(str, Enum):
    """任务生命周期状态。"""
    PENDING = "pending"
    RUNNING = "running"
    PAUSED = "paused"
    COMPLETED = "completed"
    FAILED = "failed"
    STOPPED = "stopped"
    CANCELLED = "cancelled"


class TaskPriority(int, Enum):
    """任务优先级。"""
    LOW = 1
    NORMAL = 5
    HIGH = 10
    URGENT = 20


class TaskConfig(BaseModel):
    """任务基础配置。"""
    device_id: Optional[str] = None
    priority: TaskPriority = TaskPriority.NORMAL
    max_retries: int = 0
    timeout_seconds: Optional[int] = None
    params: Dict[str, Any] = Field(default_factory=dict)


class TaskCreateRequest(BaseModel):
    """创建新任务的请求体。"""
    task_type: str
    device_id: Optional[str] = None
    config: Optional[TaskConfig] = None
    params: Dict[str, Any] = Field(default_factory=dict)


class TaskUpdateRequest(BaseModel):
    """更新任务的请求体。"""
    priority: Optional[TaskPriority] = None
    params: Optional[Dict[str, Any]] = None


class TaskProgress(BaseModel):
    """任务进度信息。"""
    current_step: int = 0
    total_steps: int = 0
    percentage: float = 0.0
    message: str = ""
    elapsed_seconds: float = 0.0
    estimated_remaining_seconds: Optional[float] = None


class TaskResult(BaseModel):
    """已完成任务的结果。"""
    success: bool
    data: Dict[str, Any] = Field(default_factory=dict)
    message: str = ""
    error: Optional[str] = None
    metrics: Dict[str, Any] = Field(default_factory=dict)


class TaskInfo(BaseModel):
    """任务的完整信息。"""
    task_id: str
    task_type: str
    status: TaskStatus = TaskStatus.PENDING
    priority: TaskPriority = TaskPriority.NORMAL
    device_id: Optional[str] = None
    config: TaskConfig = Field(default_factory=TaskConfig)
    params: Dict[str, Any] = Field(default_factory=dict)

    # 时间戳
    created_at: datetime = Field(default_factory=datetime.now)
    started_at: Optional[datetime] = None
    completed_at: Optional[datetime] = None

    # 进度追踪
    progress: TaskProgress = Field(default_factory=TaskProgress)

    # 结果（完成后填充）
    result: Optional[TaskResult] = None

    class Config:
        from_attributes = True


class TaskTypeInfo(BaseModel):
    """可用任务类型的信息。"""
    task_type: str
    name: str
    description: str
    category: str = "general"
    requires_device: bool = True
    params_schema: Dict[str, Any] = Field(default_factory=dict)


class TaskListResponse(BaseModel):
    """列出任务的响应。"""
    tasks: List[TaskInfo]
    total: int
    running_count: int
    pending_count: int


class TaskEvent(BaseModel):
    """任务更新的 Socket.IO 事件负载。"""
    event_type: str  # created, started, progress, paused, resumed, stopped, completed, failed
    task_id: str
    task_type: str
    status: TaskStatus
    progress: Optional[TaskProgress] = None
    result: Optional[TaskResult] = None
    timestamp: datetime = Field(default_factory=datetime.now)
