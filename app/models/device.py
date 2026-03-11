from pydantic import BaseModel
from typing import List, Optional


class DeviceInfo(BaseModel):
    device_id: str
    name: str
    serial_number: str
    firmware_version: Optional[str] = None
    physical_port: Optional[str] = None
    usb_type: Optional[str] = None
    product_id: Optional[str] = None
    sensors: List[str] = []
    is_streaming: bool = False
