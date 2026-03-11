"""
pyrealsense2 库的 Mock 实现（用于测试）

本模块实现了 rs_manager.py 中使用的所有类和函数，
使得测试可以在没有真实 RealSense 硬件的情况下运行。

Mock 实现的功能包括：
- 设备上下文管理
- 设备和传感器模拟
- 流配置和帧数据处理
- 深度滤波器模拟
- 点云处理模拟
"""
from unittest.mock import MagicMock
import enum
import numpy as np

# 库中使用的枚举
class stream(enum.Enum):
    """数据流类型枚举"""
    depth = 1
    color = 2
    infrared = 3
    infrared2 = 4
    fisheye = 5
    gyro = 6
    accel = 7
    gpio = 8
    pose = 9
    confidence = 10
    motion = 11

class format(enum.Enum):
    """数据格式枚举"""
    z16 = 1
    y8 = 2
    rgb8 = 3
    bgr8 = 4
    rgba8 = 5
    bgra8 = 6
    yuyv = 7
    raw8 = 8
    raw10 = 9
    raw16 = 10
    xyz32f = 11
    any = 12
    motion_xyz32f = 13

class camera_info(enum.Enum):
    """相机信息枚举"""
    name = 1
    serial_number = 2
    firmware_version = 3
    physical_port = 4
    debug_op_code = 5
    advanced_mode = 6
    product_id = 7
    product_line = 8
    usb_type_descriptor = 9
    product_line_usb_type = 10
    recommended_firmware_version = 11
    count = 12

# 定义选项系统中使用的枚举
class option(enum.Enum):
    """传感器选项枚举"""
    backlight_compensation = 1
    brightness = 2
    contrast = 3
    exposure = 4
    gain = 5
    gamma = 6
    hue = 7
    saturation = 8
    sharpness = 9
    white_balance = 10
    enable_auto_exposure = 11
    enable_auto_white_balance = 12
    visual_preset = 13
    laser_power = 14
    accuracy = 15
    motion_range = 16
    filter_option = 17
    confidence_threshold = 18
    emitter_enabled = 19
    frames_queue_size = 20
    total_frame_drops = 21
    auto_exposure_mode = 22
    power_line_frequency = 23
    asic_temperature = 24
    error_polling_enabled = 25
    projector_temperature = 26
    output_trigger_enabled = 27
    count = 28

    # 重写 name 属性用于字符串表示
    @property
    def name(self):
        return self._name_

# Mock option_range
class option_range:
    """选项范围模拟类"""
    def __init__(self, min_val=0, max_val=100, default=50, step=1):
        self.min = min_val
        self.max = max_val
        self.default = default
        self.step = step

# Mock rs.context 类
class context:
    """RealSense 上下文模拟类"""
    def __init__(self):
        self.devices = []

    def add_device(self, device):
        """添加设备到上下文"""
        self.devices.append(device)

# Mock rs.device 类
class device:
    """RealSense 设备模拟类"""
    def __init__(self, serial_number="1234", name="Test Device"):
        self.serial = serial_number
        self.name = name
        self.sensors = []
        self._info = {
            camera_info.serial_number: serial_number,
            camera_info.name: name,
            camera_info.firmware_version: "1.0.0",
            camera_info.physical_port: "USB1",
            camera_info.usb_type_descriptor: "3.0",
            camera_info.product_id: "0123"
        }

    def get_info(self, info_type):
        """获取设备信息"""
        if info_type in self._info:
            return self._info[info_type]
        raise RuntimeError(f"Info {info_type} not available")

    def add_sensor(self, sensor):
        """添加传感器到设备"""
        self.sensors.append(sensor)

# Mock 传感器基类
class sensor:
    """传感器基类模拟"""
    def __init__(self, name="Generic Sensor"):
        self.name = name
        self._info = {
            camera_info.name: name
        }
        self._options = {
            option:backlight_compensation: 0,
            option.brightness: 50,
            option.contrast: 50,
            option.exposure: 33,
            option.gain: 64
        }
        self._option_ranges = {
            option.backlight_compensation: option_range(0, 1, 0, 1),
            option.brightness: option_range(0, 100, 50, 1),
            option.contrast: option_range(0, 100, 50, 1),
            option.exposure: option_range(1, 66, 33, 1),
            option.gain: option_range(0, 128, 64, 1)
        }
        self._option_read_only = set()
        self._profiles = []

    def get_info(self, info_type):
        """获取传感器信息"""
        value = next(iter(self._info.values()))
        for key in self._info:
            if key.name == info_type.name:
                value = self._info[key]
                break
        return value

    def get_supported_options(self):
        """获取支持的选项列表"""
        return list(self._options.keys())

    def get_option(self, option_type):
        """获取选项值"""
        if option_type in self._options:
            return self._options[option_type]
        raise RuntimeError(f"Option {option_type} not supported")

    def set_option(self, option_type, value):
        """设置选项值"""
        if option_type in self._option_read_only:
            raise RuntimeError(f"Option {option_type} is read-only")
        if option_type in self._options:
            opt_range = self._option_ranges[option_type]
            if value < opt_range.min or value > opt_range.max:
                raise RuntimeError(f"Value {value} out of range [{opt_range.min}, {opt_range.max}]")
            self._options[option_type] = value
        else:
            raise RuntimeError(f"Option {option_type} not supported")

    def get_option_range(self, option_type):
        """获取选项范围"""
        if option_type in self._option_ranges:
            return self._option_ranges[option_type]
        raise RuntimeError(f"Option range {option_type} not available")

    def get_option_description(self, option_type):
        """获取选项描述"""
        return f"Description for {option_type}"

    def is_option_read_only(self, option_type):
        """检查选项是否只读"""
        return option_type in self._option_read_only

    def get_stream_profiles(self):
        """获取流配置文件列表"""
        return self._profiles

    def add_profile(self, profile):
        """添加流配置文件"""
        self._profiles.append(profile)

    def is_depth_sensor(self):
        """检查是否为深度传感器"""
        return False

    def is_color_sensor(self):
        """检查是否为颜色传感器"""
        return False

    def is_motion_sensor(self):
        """检查是否为运动传感器"""
        return False

    def is_fisheye_sensor(self):
        """检查是否为鱼眼传感器"""
        return False

# Mock 深度传感器
class depth_sensor(sensor):
    """深度传感器模拟类"""
    def __init__(self, name="Depth Sensor"):
        super().__init__(name)
        # 添加深度传感器特定选项
        self._options[option.laser_power] = 10
        self._options[option.accuracy] = 2
        self._option_ranges[option.laser_power] = option_range(0, 100, 10, 1)
        self._option_ranges[option.accuracy] = option_range(1, 3, 2, 1)

    def is_depth_sensor(self):
        return True

# Mock 颜色传感器
class color_sensor(sensor):
    """颜色传感器模拟类"""
    def __init__(self, name="Color Sensor"):
        super().__init__(name)
        # 添加颜色传感器特定选项
        self._options[option.saturation] = 64
        self._options[option.white_balance] = 4600
        self._option_ranges[option.saturation] = option_range(0, 100, 64, 1)
        self._option_ranges[option.white_balance] = option_range(2800, 6500, 4600, 10)

    def is_color_sensor(self):
        return True

# Mock 运动传感器
class motion_sensor(sensor):
    """运动传感器模拟类"""
    def __init__(self, name="Motion Sensor"):
        super().__init__(name)
        # 添加运动传感器特定选项
        self._options[option.motion_range] = 16
        self._option_ranges[option.motion_range] = option_range(0, 32, 16, 1)

    def is_motion_sensor(self):
        return True

# Mock 鱼眼传感器
class fisheye_sensor(sensor):
    """鱼眼传感器模拟类"""
    def __init__(self, name="Fisheye Sensor"):
        super().__init__(name)

    def is_fisheye_sensor(self):
        return True

# Mock 流配置文件
class stream_profile:
    """流配置文件模拟类"""
    def __init__(self, stream_type=stream.depth, format=format.z16, index=0):
        self._stream_type = stream_type
        self._format = format
        self._index = index

    def stream_type(self):
        """获取流类型"""
        return self._stream_type

    def format(self):
        """获取数据格式"""
        return self._format

    def index(self):
        """获取流索引"""
        return self._index

    def is_video_stream_profile(self):
        """检查是否为视频流配置文件"""
        return isinstance(self, video_stream_profile)

    def as_video_stream_profile(self):
        """转换为视频流配置文件"""
        if self.is_video_stream_profile():
            return self
        raise RuntimeError("Not a video stream profile")

# Mock 视频流配置文件
class video_stream_profile(stream_profile):
    """视频流配置文件模拟类"""
    def __init__(self, stream_type=stream.depth, format=format.z16, width=640, height=480, fps=30, index=0):
        super().__init__(stream_type, format, index)
        self._width = width
        self._height = height
        self._fps = fps

    def width(self):
        """获取宽度"""
        return self._width

    def height(self):
        """获取高度"""
        return self._height

    def fps(self):
        """获取帧率"""
        return self._fps

# Mock 运动流配置文件
class motion_stream_profile(stream_profile):
    """运动流配置文件模拟类"""
    def __init__(self, stream_type=stream.gyro, format=format.motion_xyz32f, fps=200, index=0):
        super().__init__(stream_type, format, index)
        self._fps = fps

    def fps(self):
        """获取帧率"""
        return self._fps

# Mock 帧数据
class frame:
    """帧数据基类模拟"""
    def __init__(self, width=640, height=480, timestamp=12345, frame_number=1):
        self._width = width
        self._height = height
        self._timestamp = timestamp
        self._frame_number = frame_number
        self._data = np.random.randint(0, 255, (height, width, 3), dtype=np.uint8)

    def get_width(self):
        """获取宽度"""
        return self._width

    def get_height(self):
        """获取高度"""
        return self._height

    def get_timestamp(self):
        """获取时间戳"""
        return self._timestamp

    def get_frame_number(self):
        """获取帧号"""
        return self._frame_number

    def get_data(self):
        """获取数据"""
        return self._data

    def is_video_frame(self):
        """检查是否为视频帧"""
        return True

    def is_depth_frame(self):
        """检查是否为深度帧"""
        return isinstance(self, depth_frame)

    def is_color_frame(self):
        """检查是否为颜色帧"""
        return isinstance(self, color_frame)

    def is_motion_frame(self):
        """检查是否为运动帧"""
        return isinstance(self, motion_frame)

    def as_depth_frame(self):
        """转换为深度帧"""
        if self.is_depth_frame():
            return self
        raise RuntimeError("Not a depth frame) {}

    def as_color_frame(self):
        """转换为颜色帧"""
        if self.is_color_frame():
            return self
        raise RuntimeError("Not a color frame")

    def as_motion_frame(self):
        """转换为运动帧"""
        if self.is.is_motion_frame():
            return self
        raise RuntimeError("Not a motion frame")

# Mock 深度帧
class depth_frame(frame):
    """深度帧模拟数据类"""
    def __init__(self, width=640, height=480, timestamp=12345, frame_number=1):
        super().__init__(width, height, timestamp, frame_number)
        # 生成随机深度数据（16位）
        self._depth_data = np.random.randint(0, 65535, (height, width), dtype=np.uint16)

    def get_distance(self, x, y):
        """获取指定坐标的距离值（米）"""
        return self._depth_data[y, x] / 1000.0  # 转换为米

# Mock 颜色帧
class color_frame(frame):
    """颜色帧模拟数据类"""
    def __init__(self, width=640, height=480, timestamp=12345, frame_number=1):
        super().__init__(width, height, timestamp, frame_number)
        # 生成随机颜色数据（RGB）
        self._data = np.random.randint(0, 255, (height, width, 3), dtype=np.uint8)

)

    @property
    def data(self):
        """获取帧数据"""
        return self._data

# Mock 运动帧
class motion_frame(frame):
    """运动帧模拟数据类"""
    def __init__(self, timestamp=12345, frame_number=1):
        super().__init__(0, 0, timestamp, frame_number)

    def get_motion_data(self):
        """获取运动数据"""
        class motion_data:
            def __init__(self):
                self.x = np.random.random()
                self.y = np.random.random()
                self.z = np.random.random()
        return motion_data()

# Mock 帧集合
class frameset:
    """帧集合模拟类"""
    def __init__(self):
        self.frames = {}

    def add_frame(self, stream_type, frame):
        """添加帧到集合"""
        self.frames[stream_type] = frame

    def get_depth_frame(self):
        """获取深度帧"""
        if stream.depth in self.frames:
            return self.frames[stream.depth]
        return None

    def get_color_frame(self):
        """获取颜色帧"""
        if stream.color in self.frames:
            return self.frames[stream.color.color_frame:
            return self.frames[stream.color]
        return None

    def get_infrared_frame(self, index=1):
        """获取红外帧"""
        stream_key = stream.infrared if index == 1 else stream.infrared2
        if stream_key in self.frames:
            return self.frames[stream_key]
        return None

    def get_motion_frame(self):
        """获取运动帧"""
        if stream.motion in self.frames:
            return self.frames[stream.motion]
        return None

    def first_or_default(self, stream_type):
        """获取指定流类型的第一个帧或默认值"""
        if stream_type in self.frames:
            return self.frames[stream_type]
        return None

    def __iter__(self):
        """迭代器"""
        return iter(self.frames.values())

# Mock 管道
class pipeline:
    """RealSense 管道模拟类"""
    def __init__(self, context=None):
        self.ctx = context if context else MagicMock()
        self.profile = None
        self.active = False
        self.device_id = None
        self.config = None
        self.frameset = frameset()

    def start(self, config=None):
        """启动管道"""
        self.active = True
        self.config = config
        if config:
            self.device_id = config.device_id

        # 创建 mock 管道配置文件
        self.profile = pipeline_profile(self.device_id)

        # 基于配置创建 mock 帧
        if config and config.streams:
            for stream_cfg in config.streams:
                stream_type = stream_cfg["stream"]
                frame_obj = None

                if stream_type == stream.depth:
                    frame_obj = depth_frame(stream_cfg.get("width", 640), stream_cfg.get("height", 480))
                elif stream_type == stream.color:
                    frame_obj = color_frame(stream_cfg.get("width", 640), stream_cfg.get("height", 480))
                elif stream_type in [stream.gyro, stream.accel]:
                    frame_obj = motion_frame()
                else:
                    frame_obj = frame(stream_cfg.get("width", 640), stream_cfg.get("height", 480))

                self.frameset.add_frame(stream_type, frame_obj)

        return self.profile

    def stop(self):
        """停止管道"""
        self.active = False
        self.profile = None

    def wait_for_frames(self, timeout_ms=5000):
        """等待新帧"""
        if not self.active:
            raise RuntimeError("Pipeline not started")
        # 返回帧集合的副本以模拟新的帧集合
        fs = frameset()
        for stream_type, frame in self.frameset.frames.items():
            fs.add_frame(stream_type, frame)
        return fs

# Mock 管道配置文件
class pipeline_profile:
    """管道配置文件模拟类"""
    def __init__(self, device_id=None):
        self.device_id = device_id

    def get_device(self):
        """获取设备"""
        return device(self.device_id)

    def get_streams(self):
        """获取流列表"""
        return []

# Mock 配置
class config:
    """管道配置模拟类"""
    def __init__(self):
        self.device_id = None
        self.streams = []

    def enable_device(self, device_id):
        """启用设备"""
        self.device_id = device_id

    def enable_stream(self, stream_type, width=0, height=0, format=format.any, framerate=0):
        """启用流"""
        self.streams.append({
            "stream": stream_type,
            "width": width,
            "height": height,
            "format": format,
            "framerate": framerate
        })

    def disable_stream(self, stream_type):
        """禁用流"""
        self.streams = [s for s in self.streams if s["stream"] != stream_type]

    def resolve(self, pipeline):
        """解析配置"""
        return pipeline_profile(self.device_id)

# Mock 对齐处理器
class align:
    """帧对齐处理器模拟类"""
    def __init__(self, stream_type):
        self.stream_type = stream_type

    def process(self, frameset):
        """处理帧对齐"""
        return frameset

# Mock 点云处理器
class pointcloud:
    """点云处理器模拟类"""
    def __init__(self):
        pass

    def calculate(self, depth_frame):
        """从深度帧计算点云"""
        return points()

    def map_to(self, frame, mapped_frame):
        """映射纹理到点云"""
        return points()

# Mock 点云数据
class points:
    """点云数据模拟类"""
    def __init__(self):
        pass

    def get_vertices(self):
        """获取顶点坐标"""
        return np.random.random((100, 3)).astype(np.float32)

    def get_texture_coordinates(self):
        """获取纹理坐标"""
        return np.random.random((100, 2)).astype(np.float32)

# Mock 深度颜色化器
class colorizer:
    """深度帧颜色化器模拟类"""
    def __init__(self):
        pass

    def colorize(self, depth_frame):
        """将深度帧转换为彩色帧"""
        width = depth_frame.get_width()
        height = depth_frame.get_height()
        # 创建深度帧的颜色化版本
        colorized = color_frame(width, height, depth_frame.get_timestamp(), depth_frame.get_frame_number())
        # 生成蓝色到红色的热力图外观
        colorized._data = np.zeros((height, width, 3), dtype=np.uint8)
        depth_data = colorized._depth_data.astype(float) / 65535.0  # 归一化到 0-1
        colorized._data[:,:,0] = (1.0 - depth_data) * 255  # 红色（远）
        colorized._data[:,:,2] = depth_data * 255  # 蓝色（近）
        return colorized

# Mock 孔洞填充滤波器
class hole_filling_filter:
    """孔洞填充滤波器模拟类"""
    def __init__(self):
        pass

    def process(self, frame):
        """填充深度帧中的孔洞"""
        return frame

# Mock 降采样滤波器
class decimation_filter:
    """降采样滤波器模拟类"""
    def __init__(self):
        pass

    def process(self, frame):
        """对帧进行降采样"""
        return frame

# Mock 时间滤波器
class temporal_filter:
    """时间滤波器模拟类"""
    def __init__(self):
        pass

    def process(self, frame):
        """应用时间滤波"""
        return frame

# Mock 空间滤波器
class spatial_filter:
    """空间滤波器模拟类"""
    def __init__(self):
        pass

    def process(self, frame):
        """应用空间滤波"""
        return frame

# Mock 视差变换
class disparity_transform:
    """视差变换模拟类"""
    def __init__(self, transform_to_disparity=True):
        self.transform_to_disparity = transform_to_disparity fanc

    def process(self, frame):
        """转换帧"""
        return frame

# Mock 异常类型
class error(Exception):
    """RealSense 基础异常"""
    pass

class camera_disconnected_error(error):
    """相机断开连接异常"""
    pass

class invalid_value_error(error):
    """无效值异常"""
    pass

class wrong_api_call_sequence_error(error):
    """错误的 API 调用顺序异常"""
    pass

class not_implemented_error(error):
    """未实现异常"""
    pass

# 辅助函数：设置 mock 设备
def create_mock_device(serial_number, name, with_depth=True, with_color=True, with_motion=False, with_fisheye=False):
    """
    创建具有指定传感器的 mock 设备
    """
    mock_device = device(serial_number, name)

    if with_depth:
        depth_sensor_obj = depth_sensor("Depth Sensor")
        # 添加深度流配置文件
        for res in [(640, 480), (1280, 720)]:
            for fps in [30, 60]:
                depth_sensor_obj.add_profile(video_stream_profile(
                    stream.depth, format.z16, res[0], res[1], fps
                ))
        mock_device.add_sensor(depth_sensor_obj)

    if with_color:
        color_sensor_obj = color_sensor("RGB Camera")
        # 添加颜色流配置文件
        for res in [(640, 480), (1280, 720), (1920, 1080)]:
            for fps in [30, 60]:
                for fmt in [format.rgb8, format.bgr8]:
                    color_sensor_obj.add_profile(video_stream_profile(
                        stream.color, fmt, res[0], res[1], fps
                    ))
        mock_device.add_sensor(color_sensor_obj)

    if with_motion:
        motion_sensor_obj = motion_sensor("Motion Module")
        # 添加运动流配置文件
        for fps in [200, 400]:
            motion_sensor_obj.add_profile(motion_stream_profile(stream.gyro, format.motion_xyz32f, fps))
            motion_sensor_obj.add_profile(motion_stream_profile(stream.accel, format.motion_xyz32f, fps))
        mock_device.add_sensor(motion_sensor_obj)

    if with_fisheye:
        fisheye_sensor_obj = fisheye_sensor("Fisheye Camera")
        # 添加鱼眼流配置文件
        for res in [(640, 480), (1280, 720)]:
            for fps in [30, 60]:
                fisheye_sensor_obj.add_profile(video_stream_profile(
                    stream.fisheye, format.y8, res[0], res[1], fps
                ))
        mock_device.add_sensor(fisheye_sensor_obj)

    return mock_device

def setup_mock_context():
    """
    设置包含多个设备的 mock 上下文
    """
    ctx = context()

    # 添加一些 mock 设备
    device1 = create_mock_device("1234", "RealSense D435", with_depth=True, with_color=True)
    device2 = create_mock_device("5678", "RealSense D415", with_depth=True, with_color=True)
    device3 = create_mock_device("9012", "RealSense T265", with_fisheye=True, with_motion=True)

    ctx.add_device(device1)
    ctx.add_device(device2)
    ctx.add_device(device3)

    return ctx
