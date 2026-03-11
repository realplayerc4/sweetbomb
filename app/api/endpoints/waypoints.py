from fastapi import APIRouter, Depends, HTTPException
from typing import List

from app.models.waypoint import Waypoint
from app.services.waypoint_manager import WaypointManager

router = APIRouter()

def get_waypoint_manager() -> WaypointManager:
    return WaypointManager.get_instance()

@router.get("/", response_model=List[Waypoint])
async def list_waypoints(manager: WaypointManager = Depends(get_waypoint_manager)):
    """List all saved waypoints."""
    return manager.get_all()

@router.post("/", response_model=Waypoint)
async def create_waypoint(waypoint: Waypoint, manager: WaypointManager = Depends(get_waypoint_manager)):
    """Create or update a waypoint."""
    try:
        manager.add_waypoint(waypoint)
        return waypoint
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))

@router.delete("/{name}", status_code=204)
async def delete_waypoint(name: str, manager: WaypointManager = Depends(get_waypoint_manager)):
    """Delete a waypoint by name."""
    if not manager.remove_waypoint(name):
        raise HTTPException(status_code=404, detail=f"Waypoint '{name}' not found")
    return None
