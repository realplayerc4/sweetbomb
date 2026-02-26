from pydantic import BaseModel, Field
from typing import List, Tuple
from datetime import datetime

class Waypoint(BaseModel):
    """Robotic waypoint model."""
    name: str
    pos: Tuple[float, float, float] = Field(default=(0.0, 0.0, 0.0)) # x, y, z
    created_at: datetime = Field(default_factory=datetime.now)

class WaypointList(BaseModel):
    """List of waypoints for easy serialization."""
    waypoints: List[Waypoint] = Field(default_factory=list)
