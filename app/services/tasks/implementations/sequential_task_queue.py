import asyncio
import logging
from typing import Any, Dict, List, Optional

from app.models.task import TaskResult, TaskStatus, TaskCreateRequest
from app.services.tasks.base_task import BaseTask
from app.services.tasks.registry import register_task

logger = logging.getLogger(__name__)

@register_task
class SequentialTaskQueue(BaseTask):
    """
    Task that executes a list of sub-tasks in sequence.
    """
    task_type = "sequential_queue"
    name = "顺序任务队列"
    description = "按顺序执行多个子任务，如果其中一个任务失败，队列将停止执行"
    category = "general"
    requires_device = False

    params_schema = {
        "type": "object",
        "properties": {
            "sub_tasks": {
                "type": "array",
                "items": {
                    "type": "object",
                    "properties": {
                        "task_type": {"type": "string"},
                        "params": {"type": "object"},
                        "device_id": {"type": "string"}
                    },
                    "required": ["task_type"]
                },
                "minItems": 1,
                "description": "要按顺序执行的任务列表"
            }
        },
        "required": ["sub_tasks"]
    }

    def __init__(self, task_id: str, config, params: Dict[str, Any], device_id: Optional[str] = None):
        super().__init__(task_id, config, params, device_id)
        self.sub_tasks_configs = params.get("sub_tasks", [])
        self._current_sub_task_id = None
        self._current_sub_task_idx = -1

    def validate(self) -> bool:
        if not self.sub_tasks_configs:
            raise ValueError("sub_tasks list cannot be empty")
        return True

    async def run(self) -> TaskResult:
        from app.services.task_manager import TaskManager
        manager = TaskManager.get_instance()
        
        total = len(self.sub_tasks_configs)
        results = []
        
        self.update_progress(0, total_steps=total, message=f"开始执行顺序任务队列 (共 {total} 个子任务)...")
        
        for i, config in enumerate(self.sub_tasks_configs):
            self._current_sub_task_idx = i
            await self.async_check_paused()
            
            if self._stop_requested:
                break
                
            task_type = config.get("task_type")
            task_params = config.get("params", {})
            task_device_id = config.get("device_id") or self.device_id
            
            self.update_progress(
                percentage=(i / total) * 100,
                message=f"正在准备第 {i+1}/{total} 个子任务: {task_type}"
            )
            
            try:
                # Create sub-task
                request = TaskCreateRequest(
                    task_type=task_type,
                    device_id=task_device_id,
                    params=task_params
                )
                sub_task_info = manager.create_task(request)
                self._current_sub_task_id = sub_task_info.task_id
                
                # Start sub-task
                await manager.start_task(self._current_sub_task_id)
                
                # Wait for completion
                while True:
                    await asyncio.sleep(0.5)
                    await self.async_check_paused()
                    
                    if self._stop_requested:
                        await manager.stop_task(self._current_sub_task_id)
                        break
                        
                    current_info = manager.get_task(self._current_sub_task_id)
                    if not current_info:
                        break
                        
                    if current_info.status in [TaskStatus.COMPLETED, TaskStatus.FAILED, TaskStatus.STOPPED, TaskStatus.CANCELLED]:
                        results.append({
                            "task_type": task_type,
                            "status": current_info.status,
                            "result": current_info.result.model_dump() if current_info.result else None
                        })
                        
                        if current_info.status != TaskStatus.COMPLETED:
                            return TaskResult(
                                success=False,
                                message=f"任务队列在第 {i+1} 个任务 ({task_type}) 失败/中断后停止",
                                data={"completed_results": results}
                            )
                        break
                        
                    # Propagate sub-task progress to main progress
                    sub_p = current_info.progress.percentage
                    overall_p = ((i + (sub_p / 100)) / total) * 100
                    self.update_progress(
                        percentage=overall_p,
                        message=f"任务队列进度 {i+1}/{total}: {current_info.progress.message}"
                    )
                    
            except Exception as e:
                logger.error(f"Error in SequentialTaskQueue sub-task {i}: {e}")
                return TaskResult(
                    success=False,
                    message=f"任务队列在第 {i+1} 个子任务运行时发生异常",
                    error=str(e),
                    data={"completed_results": results}
                )
        
        if self._stop_requested:
            return TaskResult(success=False, message="任务队列已人为中途停止", data={"completed_results": results})
            
        return TaskResult(
            success=True,
            message=f"任务队列成功完成所有 {total} 个任务",
            data={"results": results}
        )

    def teardown(self):
        logger.info(f"Cleaning up task queue {self.task_id}")
        # Note: If main task is torn down, we should ideally ensure sub-tasks are cleaned up if still running
        pass
