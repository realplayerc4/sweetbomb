"""Task API endpoints."""

from fastapi import APIRouter, Depends, HTTPException, Query
from typing import List, Optional

from app.models.task import (
    TaskCreateRequest,
    TaskInfo,
    TaskListResponse,
    TaskStatus,
    TaskTypeInfo,
)
from app.services.task_manager import TaskManager
from app.services.tasks.registry import TaskRegistry


router = APIRouter()


def get_task_manager() -> TaskManager:
    """Dependency to get TaskManager instance."""
    return TaskManager.get_instance()


def get_task_registry() -> TaskRegistry:
    """Dependency to get TaskRegistry instance."""
    return TaskRegistry.get_instance()


# --- Task Type Endpoints ---

@router.get("/types", response_model=List[TaskTypeInfo])
async def get_task_types(
    registry: TaskRegistry = Depends(get_task_registry)
):
    """
    Get all available task types.

    Returns a list of task types that can be created, including their
    names, descriptions, and parameter schemas.
    """
    return registry.list_type_infos()


@router.get("/types/{task_type}", response_model=TaskTypeInfo)
async def get_task_type_info(
    task_type: str,
    registry: TaskRegistry = Depends(get_task_registry)
):
    """
    Get detailed information about a specific task type.

    Args:
        task_type: The task type identifier

    Returns:
        TaskTypeInfo with name, description, and parameter schema
    """
    info = registry.get_type_info(task_type)
    if info is None:
        raise HTTPException(status_code=404, detail=f"Task type not found: {task_type}")
    return info


# --- Task CRUD Endpoints ---

@router.post("/", response_model=TaskInfo, status_code=201)
async def create_task(
    request: TaskCreateRequest,
    manager: TaskManager = Depends(get_task_manager)
):
    """
    Create a new task.

    The task will be created in PENDING status and must be started
    using the /tasks/{task_id}/start endpoint.

    Args:
        request: Task creation request with task_type, device_id, and params

    Returns:
        TaskInfo for the created task
    """
    try:
        return manager.create_task(request)
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.get("/", response_model=TaskListResponse)
async def list_tasks(
    status: Optional[TaskStatus] = Query(None, description="Filter by status"),
    device_id: Optional[str] = Query(None, description="Filter by device ID"),
    task_type: Optional[str] = Query(None, description="Filter by task type"),
    manager: TaskManager = Depends(get_task_manager)
):
    """
    List all tasks with optional filtering.

    Args:
        status: Filter by task status
        device_id: Filter by device ID
        task_type: Filter by task type

    Returns:
        TaskListResponse with list of tasks and counts
    """
    return manager.list_tasks(status=status, device_id=device_id, task_type=task_type)


@router.get("/{task_id}", response_model=TaskInfo)
async def get_task(
    task_id: str,
    manager: TaskManager = Depends(get_task_manager)
):
    """
    Get detailed information about a specific task.

    Args:
        task_id: The task ID

    Returns:
        TaskInfo with full task details including progress and result
    """
    task = manager.get_task(task_id)
    if task is None:
        raise HTTPException(status_code=404, detail=f"Task not found: {task_id}")
    return task


@router.delete("/{task_id}", status_code=204)
async def delete_task(
    task_id: str,
    manager: TaskManager = Depends(get_task_manager)
):
    """
    Delete a task.

    Note: Running tasks cannot be deleted. Stop the task first.

    Args:
        task_id: The task ID to delete
    """
    try:
        manager.delete_task(task_id)
    except ValueError as e:
        if "not found" in str(e):
            raise HTTPException(status_code=404, detail=str(e))
        raise HTTPException(status_code=400, detail=str(e))


# --- Task Control Endpoints ---

@router.post("/{task_id}/start", response_model=TaskInfo)
async def start_task(
    task_id: str,
    manager: TaskManager = Depends(get_task_manager)
):
    """
    Start a pending task.

    The task will begin execution. At most 4 tasks can run concurrently.

    Args:
        task_id: The task ID to start

    Returns:
        Updated TaskInfo with RUNNING status
    """
    try:
        if not manager.can_start_more_tasks():
            raise HTTPException(
                status_code=429,
                detail="Maximum concurrent tasks reached (4). Wait for a task to complete."
            )
        return await manager.start_task(task_id)
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.post("/{task_id}/pause", response_model=TaskInfo)
async def pause_task(
    task_id: str,
    manager: TaskManager = Depends(get_task_manager)
):
    """
    Pause a running task.

    The task can be resumed later using /tasks/{task_id}/resume.

    Args:
        task_id: The task ID to pause

    Returns:
        Updated TaskInfo with PAUSED status
    """
    try:
        return await manager.pause_task(task_id)
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.post("/{task_id}/resume", response_model=TaskInfo)
async def resume_task(
    task_id: str,
    manager: TaskManager = Depends(get_task_manager)
):
    """
    Resume a paused task.

    Args:
        task_id: The task ID to resume

    Returns:
        Updated TaskInfo with RUNNING status
    """
    try:
        return await manager.resume_task(task_id)
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.post("/{task_id}/stop", response_model=TaskInfo)
async def stop_task(
    task_id: str,
    manager: TaskManager = Depends(get_task_manager)
):
    """
    Stop a running or paused task.

    The task cannot be resumed after stopping. Use this to cancel
    a task that is no longer needed.

    Args:
        task_id: The task ID to stop

    Returns:
        Updated TaskInfo with STOPPED status
    """
    try:
        return await manager.stop_task(task_id)
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
