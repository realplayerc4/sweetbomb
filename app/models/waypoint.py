from pydantic import BaseModel, Field
from typing import List, Tuple
from datetime import datetime

class Waypoint(BaseModel):
    """机器人航点模型。"""
    name: str
    pos: Tuple[float, float, float] = Field(default=(0.0, 0.0, 0.0)) # x, y, z
    created_at: datetime = Field(default_factory=datetime.now)

class WaypointList(BaseModel):
    """航点列表，用于便捷序列化。"""
    waypoints: List[Waypoint] = Field(default_factory=list)
