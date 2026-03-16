"""OAK 管道配置模型。"""

from enum import Enum
from typing import Optional, List, Dict, Any
from dataclasses import dataclass, field


class OAKResolution(Enum):
    """OAK 支持的分辨率。"""
    THE_1080_P = (1920, 1080)
    THE_720_P = (1280, 720)
    THE_480_P = (640, 480)
    THE_400_P = (640, 400)
    THE_240_P = (320, 240)

    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height


class OAKCameraType(Enum):
    """OAK 相机类型。"""
    COLOR = "color"           # RGB 彩色相机
    MONO = "mono"             # 单色相机（左/右）
    DEPTH = "depth"           # 深度相机（立体深度）


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
class OAKCameraConfig:
    """单个相机配置。"""
    camera_type: OAKCameraType
    resolution: OAKResolution = OAKResolution.THE_1080_P
    fps: int = 30
    # 手动曝光控制
    manual_exposure: bool = False
    exposure_us: Optional[int] = None  # 曝光时间 (微秒)
    iso: Optional[int] = None  # ISO 增益
    # 对焦控制 (仅彩色相机)
    manual_focus: bool = False
    lens_position: Optional[int] = None  # 镜头位置 (0-255)
    # 其他选项
    sharpness: Optional[int] = None  # 锐度
    brightness: Optional[int] = None  # 亮度
    contrast: Optional[int] = None  # 对比度
    saturation: Optional[int] = None  # 饱和度 (仅彩色相机)

    def to_dict(self) -> Dict[str, Any]:
        """转换为字典。"""
        return {
            "camera_type": self.camera_type.value,
            "resolution": f"{self.resolution.width}x{self.resolution.height}",
            "fps": self.fps,
            "manual_exposure": self.manual_exposure,
            "exposure_us": self.exposure_us,
            "iso": self.iso,
            "manual_focus": self.manual_focus,
            "lens_position": self.lens_position,
        }


@dataclass
class OAKStreamConfig:
    """输出流配置。"""
    stream_name: str
    frame_type: OAKFrameType
    # 发布配置
    publish_enabled: bool = True
    # 输出队列大小
    queue_size: int = 4
    # 阻塞模式
    blocking: bool = False
    # 预处理选项
    color_map: Optional[str] = None  # 深度可视化色图 (jet, turbo, etc.)
    normalize_depth: bool = True  # 深度归一化


@dataclass
class OAKPipelineConfig:
    """OAK 管道完整配置。"""
    # 设备 MxId (可选，None 表示使用第一个可用设备)
    device_mxid: Optional[str] = None
    # USB 速度模式
    usb_speed: str = "SUPER_PLUS"  # SUPER, SUPER_PLUS, HIGH
    # 相机配置
    cameras: List[OAKCameraConfig] = field(default_factory=list)
    # 输出流配置
    streams: List[OAKStreamConfig] = field(default_factory=list)
    # 深度计算配置
    depth_enabled: bool = True
    depth_align_to_color: bool = True  # 深度对齐到彩色相机
    depth_subpixel: bool = True  # 子像素精度
    depth_extended_disparity: bool = False  # 扩展视差范围
    depth_left_right_check: bool = True  # 左右一致性检查
    median_filter: str = "KERNEL_7x7"  # 中值滤波核大小
    # 点云配置
    point_cloud_enabled: bool = False
    point_cloud_rate: int = 5  # 点云发布频率 (Hz)
    # 性能配置
    max_latency_ms: int = 100  # 最大延迟
    xlink_chunk_size: int = 0  # XLink 分块大小 (0 = 默认)

    def validate(self) -> List[str]:
        """验证配置，返回错误列表。"""
        errors = []

        # 检查相机配置
        has_color = any(c.camera_type == OAKCameraType.COLOR for c in self.cameras)
        has_mono = any(c.camera_type == OAKCameraType.MONO for c in self.cameras)

        if self.depth_enabled and not has_mono:
            errors.append("深度计算需要至少一个单目相机（左/右）")

        if self.depth_align_to_color and not has_color:
            errors.append("深度对齐到彩色相机需要彩色相机配置")

        # 检查 USB 速度
        valid_usb_speeds = ["LOW", "FULL", "HIGH", "SUPER", "SUPER_PLUS"]
        if self.usb_speed not in valid_usb_speeds:
            errors.append(f"无效 USB 速度: {self.usb_speed}")

        return errors

    def to_dict(self) -> Dict[str, Any]:
        """转换为字典。"""
        return {
            "device_mxid": self.device_mxid,
            "usb_speed": self.usb_speed,
            "cameras": [c.to_dict() for c in self.cameras],
            "depth_enabled": self.depth_enabled,
            "depth_align_to_color": self.depth_align_to_color,
            "point_cloud_enabled": self.point_cloud_enabled,
        }
