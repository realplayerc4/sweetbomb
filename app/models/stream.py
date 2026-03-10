from pydantic import BaseModel
from typing import List, Optional


class Resolution(BaseModel):
    width: int
    height: int


class StreamConfig(BaseModel):
    stream_type: str
    format: str
    resolution: Resolution
    framerate: int
    sensor_id: str
    enable: bool = True


class StreamStart(BaseModel):
    configs: List[StreamConfig]
    align_to: Optional[str] = None
    apply_filters: bool = False


class StreamStatus(BaseModel):
    device_id: str
    is_streaming: bool
    active_streams: List[str] = []
    framerate: Optional[float] = None
    duration: Optional[float] = None


class PointCloudStatus(BaseModel):
    device_id: str
    is_active: bool
