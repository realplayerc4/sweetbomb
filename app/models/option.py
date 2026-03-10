from pydantic import BaseModel
from typing import Any, Optional, Union


class OptionUpdate(BaseModel):
    value: Any


class OptionInfo(BaseModel):
    option_id: str
    name: str
    description: Optional[str] = None
    current_value: Any
    default_value: Any
    min_value: Optional[Union[float, int]] = None
    max_value: Optional[Union[float, int]] = None
    step: Optional[Union[float, int]] = None
    units: Optional[str] = None
    read_only: bool = False
