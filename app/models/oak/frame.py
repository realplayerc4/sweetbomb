"""OAK 帧数据模型。"""

from typing import Optional
from dataclasses import dataclass
from enum import Enum
import numpy as np


class OAKFrameType(Enum):
    """OAK 帧类型。"""
    COLOR = "color"
    LEFT = "left"
    RIGHT = "right"
    DEPTH = "depth"
    DISPARITY = "disparity"
    RECTIFIED_LEFT = "rectified_left"
    RECTIFIED_RIGHT = "rectified_right"


@dataclass
class OAKFrame:
    """OAK 帧数据。"""
    data: np.ndarray  # 帧数据
    frame_type: OAKFrameType  # 帧类型
    timestamp: float  # 时间戳
    sequence_num: int  # 序列号
    stream_name: str  # 流名称
    width: int  # 宽度
    height: int  # 高度

    def to_dict(self) -> dict:
        """转换为字典。"""
        return {
            "frame_type": self.frame_type.value,
            "timestamp": self.timestamp,
            "sequence_num": self.sequence_num,
            "stream_name": self.stream_name,
            "width": self.width,
            "height": self.height,
        }
