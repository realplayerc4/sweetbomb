"""API Router Configuration.

This module aggregates all API endpoint routers.
"""

from fastapi import APIRouter

from app.api.endpoints import (
    devices,
    map as map_endpoint,
    options,
    path_map,
    point_cloud,
    robot,
    sensors,
    streams,
    webrtc,
)

# Create main API router
api_router = APIRouter()

# Include all endpoint routers
api_router.include_router(devices.router, prefix="/devices", tags=["devices"])
api_router.include_router(sensors.router, prefix="/devices/{device_id}/sensors", tags=["sensors"])
api_router.include_router(options.router, prefix="/devices/{device_id}/sensors/{sensor_id}/options", tags=["options"])
api_router.include_router(streams.router, prefix="/devices/{device_id}/stream", tags=["streams"])
api_router.include_router(point_cloud.router, prefix="/devices/{device_id}/point_cloud", tags=["point_cloud"])
api_router.include_router(webrtc.router, prefix="/webrtc", tags=["webrtc"])
api_router.include_router(map_endpoint.router, prefix="/map", tags=["map"])
api_router.include_router(path_map.router, prefix="/path_map", tags=["path_map"])
api_router.include_router(robot.router, prefix="/robot", tags=["robot"])
