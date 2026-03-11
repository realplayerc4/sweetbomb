import asyncio
import logging
from typing import Any, Dict, Optional, Tuple

from app.models.task import TaskResult, TaskStatus
from app.services.tasks.base_task import BaseTask
from app.services.tasks.registry import register_task
from app.services.waypoint_manager import WaypointManager

logger = logging.getLogger(__name__)

@register_task
class NavigationTask(BaseTask):
    """
    Task for moving the robot to a specific coordinate or waypoint.
    Currently runs in SIMULATION mode.
    """
    task_type = "navigation"
    name = "路径点导航"
    description = "驱动机器人移动到指定的坐标点或预设路径点"
    category = "navigation"
    requires_device = False # In simulation, we don't strictly need a realsense device

    params_schema = {
        "type": "object",
        "properties": {
            "target_name": {"type": "string", "description": "目标路径点名称"},
            "target_pos": {
                "type": "array", 
                "items": {"type": "number"}, 
                "minItems": 3, 
                "maxItems": 3,
                "description": "目标坐标 [x, y, z]"
            },
            "speed": {"type": "number", "default": 0.5, "description": "移动速度 (m/s)"}
        }
    }

    def __init__(self, task_id: str, config, params: Dict[str, Any], device_id: Optional[str] = None):
        super().__init__(task_id, config, params, device_id)
        self.target_name = params.get("target_name")
        self.target_pos = params.get("target_pos")
        self.speed = params.get("speed", 0.5)
        self.wp_manager = WaypointManager.get_instance()
        
        # Current simulated position (hardcoded for now, would come from robot base)
        self.current_pos = [0.0, 0.0, 0.0]

    def validate(self) -> bool:
        if not self.target_name and not self.target_pos:
            raise ValueError("Either target_name or target_pos must be provided")
        
        if self.target_name:
            wp = self.wp_manager.get_by_name(self.target_name)
            if not wp:
                raise ValueError(f"Waypoint '{self.target_name}' not found")
            self.target_pos = wp.pos
            
        return True

    async def run(self) -> TaskResult:
        self.update_progress(0, message=f"正在规划前往 {self.target_name or self.target_pos} 的路径...")
        await asyncio.sleep(1.0) # Planning delay
        
        # Calculate distance
        start_pos = self.current_pos
        dest_pos = self.target_pos
        
        dist = sum((a - b)**2 for a, b in zip(start_pos, dest_pos))**0.5
        duration = dist / self.speed if self.speed > 0 else 5.0
        steps = 20
        step_duration = duration / steps
        
        logger.info(f"Navigation task {self.task_id} started. Distance: {dist:.2f}m, Duration: {duration:.2f}s")
        
        for i in range(steps):
            await self.async_check_paused()
            
            # Interpolate position
            ratio = (i + 1) / steps
            self.current_pos = [
                start_pos[j] + (dest_pos[j] - start_pos[j]) * ratio 
                for j in range(3)
            ]
            
            self.update_progress(
                percentage=ratio * 100,
                message=f"正在移动... 当前位置: [{self.current_pos[0]:.2f}, {self.current_pos[1]:.2f}, {self.current_pos[2]:.2f}]"
            )
            
            await asyncio.sleep(step_duration)
            
            if self._stop_requested:
                break
                
        if self._stop_requested:
            return TaskResult(success=False, message="导航任务已中途人工停止", data={"final_pos": self.current_pos})
            
        return TaskResult(
            success=True, 
            message="导航成功到达目标点", 
            data={"target": self.target_name or self.target_pos, "final_pos": self.current_pos}
        )

@register_task
class ReturnToOriginTask(NavigationTask):
    """
    Specialized task for returning to the origin (0, 0, 0).
    """
    task_type = "return_to_origin"
    name = "回原点"
    description = "驱动机器人返回预设的原点位置 (0, 0, 0)"
    category = "navigation"
    
    def __init__(self, task_id: str, config, params: Dict[str, Any], device_id: Optional[str] = None):
        # Override params to enforce origin
        params["target_name"] = "Origin"
        params["target_pos"] = [0.0, 0.0, 0.0]
        super().__init__(task_id, config, params, device_id)
