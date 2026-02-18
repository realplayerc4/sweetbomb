"""Task system data models."""

from enum import Enum
from typing import Any, Dict, List, Optional
from datetime import datetime
from pydantic import BaseModel, Field


class TaskStatus(str, Enum):
    """Task lifecycle status."""
    PENDING = "pending"
    RUNNING = "running"
    PAUSED = "paused"
    COMPLETED = "completed"
    FAILED = "failed"
    STOPPED = "stopped"
    CANCELLED = "cancelled"


class TaskPriority(int, Enum):
    """Task priority levels."""
    LOW = 1
    NORMAL = 5
    HIGH = 10
    URGENT = 20


class TaskConfig(BaseModel):
    """Base configuration for a task."""
    device_id: Optional[str] = None
    priority: TaskPriority = TaskPriority.NORMAL
    max_retries: int = 0
    timeout_seconds: Optional[int] = None
    params: Dict[str, Any] = Field(default_factory=dict)


class TaskCreateRequest(BaseModel):
    """Request body for creating a new task."""
    task_type: str
    device_id: Optional[str] = None
    config: Optional[TaskConfig] = None
    params: Dict[str, Any] = Field(default_factory=dict)


class TaskUpdateRequest(BaseModel):
    """Request body for updating a task."""
    priority: Optional[TaskPriority] = None
    params: Optional[Dict[str, Any]] = None


class TaskProgress(BaseModel):
    """Task progress information."""
    current_step: int = 0
    total_steps: int = 0
    percentage: float = 0.0
    message: str = ""
    elapsed_seconds: float = 0.0
    estimated_remaining_seconds: Optional[float] = None


class TaskResult(BaseModel):
    """Result of a completed task."""
    success: bool
    data: Dict[str, Any] = Field(default_factory=dict)
    message: str = ""
    error: Optional[str] = None
    metrics: Dict[str, Any] = Field(default_factory=dict)


class TaskInfo(BaseModel):
    """Complete information about a task."""
    task_id: str
    task_type: str
    status: TaskStatus = TaskStatus.PENDING
    priority: TaskPriority = TaskPriority.NORMAL
    device_id: Optional[str] = None
    config: TaskConfig = Field(default_factory=TaskConfig)
    params: Dict[str, Any] = Field(default_factory=dict)

    # Timestamps
    created_at: datetime = Field(default_factory=datetime.now)
    started_at: Optional[datetime] = None
    completed_at: Optional[datetime] = None

    # Progress tracking
    progress: TaskProgress = Field(default_factory=TaskProgress)

    # Result (populated when completed)
    result: Optional[TaskResult] = None

    class Config:
        from_attributes = True


class TaskTypeInfo(BaseModel):
    """Information about an available task type."""
    task_type: str
    name: str
    description: str
    category: str = "general"
    requires_device: bool = True
    params_schema: Dict[str, Any] = Field(default_factory=dict)


class TaskListResponse(BaseModel):
    """Response for listing tasks."""
    tasks: List[TaskInfo]
    total: int
    running_count: int
    pending_count: int


class TaskEvent(BaseModel):
    """Socket.IO event payload for task updates."""
    event_type: str  # created, started, progress, paused, resumed, stopped, completed, failed
    task_id: str
    task_type: str
    status: TaskStatus
    progress: Optional[TaskProgress] = None
    result: Optional[TaskResult] = None
    timestamp: datetime = Field(default_factory=datetime.now)
