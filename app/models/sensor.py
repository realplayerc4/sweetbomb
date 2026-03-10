from pydantic import BaseModel
from typing import List

from app.models.option import OptionInfo


class SupportedStreamProfile(BaseModel):
    stream_type: str
    resolutions: List[tuple[int, int]]
    fps: List[int]
    formats: List[str]


class SensorInfo(BaseModel):
    sensor_id: str
    name: str
    type: str
    supported_stream_profiles: List[SupportedStreamProfile] = []
    options: List[OptionInfo] = []
