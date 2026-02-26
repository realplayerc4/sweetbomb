"""传感器参数控制模块。

从 rs_manager 拆分而来，负责传感器信息查询与参数的读写操作。
"""

import threading
from typing import Any, Dict, List

import pyrealsense2 as rs

from app.core.errors import RealSenseError
from app.models.option import OptionInfo
from app.models.sensor import SensorInfo, SupportedStreamProfile


class SensorControl:
    """管理 RealSense 传感器参数的读取与设置。"""

    def __init__(
        self,
        devices: Dict[str, rs.device],
        lock: threading.Lock,
        refresh_fn,
    ):
        # 共享设备字典和锁；refresh_fn 用于在设备未找到时触发刷新
        self.devices = devices
        self.lock = lock
        self._refresh_devices = refresh_fn

    def _ensure_device(self, device_id: str) -> None:
        """确保设备存在，若不在则尝试刷新。"""
        if device_id not in self.devices:
            self._refresh_devices()
            if device_id not in self.devices:
                raise RealSenseError(
                    status_code=404, detail=f"Device {device_id} not found"
                )

    def _parse_sensor_index(self, device_id: str, sensor_id: str) -> int:
        """从 sensor_id 字符串中解析传感器索引。"""
        try:
            sensor_index = int(sensor_id.split("-")[-1])
            if sensor_index < 0 or sensor_index >= len(self.devices[device_id].sensors):
                raise RealSenseError(
                    status_code=404, detail=f"Sensor {sensor_id} not found"
                )
            return sensor_index
        except (ValueError, IndexError):
            raise RealSenseError(
                status_code=404, detail=f"Invalid sensor ID format: {sensor_id}"
            )

    def get_sensors(self, device_id: str) -> List[SensorInfo]:
        """获取设备的所有传感器信息。"""
        self._ensure_device(device_id)
        dev = self.devices[device_id]
        sensors = []

        for i, sensor in enumerate(dev.sensors):
            sensor_id = f"{device_id}-sensor-{i}"
            try:
                name = sensor.get_info(rs.camera_info.name)
            except RuntimeError:
                name = f"Sensor {i}"

            sensor_type = sensor.name
            stream_profiles_list = self._build_stream_profiles(sensor)
            options = self.get_sensor_options(device_id, sensor_id)

            sensor_info = SensorInfo(
                sensor_id=sensor_id,
                name=name,
                type=sensor_type,
                supported_stream_profiles=stream_profiles_list,
                options=options,
            )
            sensors.append(sensor_info)

        return sensors

    def get_sensor(self, device_id: str, sensor_id: str) -> SensorInfo:
        """根据 ID 获取特定传感器。"""
        sensors = self.get_sensors(device_id)
        for sensor in sensors:
            if sensor.sensor_id == sensor_id:
                return sensor
        raise RealSenseError(status_code=404, detail=f"Sensor {sensor_id} not found")

    def get_sensor_options(self, device_id: str, sensor_id: str) -> List[OptionInfo]:
        """获取传感器的所有可用选项。"""
        self._ensure_device(device_id)
        dev = self.devices[device_id]
        sensor_index = self._parse_sensor_index(device_id, sensor_id)
        sensor = dev.sensors[sensor_index]

        options = []
        for option in sensor.get_supported_options():
            try:
                opt_name = option.name
                current_value = sensor.get_option(option)
                option_range = sensor.get_option_range(option)

                option_info = OptionInfo(
                    option_id=opt_name,
                    name=opt_name.replace("_", " ").title(),
                    description=sensor.get_option_description(option),
                    current_value=current_value,
                    default_value=option_range.default,
                    min_value=option_range.min,
                    max_value=option_range.max,
                    step=option_range.step,
                    read_only=sensor.is_option_read_only(option),
                )
                options.append(option_info)
            except RuntimeError:
                # 某些选项在某些状态下不可读，直接跳过
                pass

        return options

    def get_sensor_option(
        self, device_id: str, sensor_id: str, option_id: str
    ) -> OptionInfo:
        """获取传感器的某个特定选项。

        先从批量获取中查找，若因 RuntimeError 被跳过，则直接通过
        pyrealsense2 枚举单独读取目标 option，以提升可靠性。
        """
        options = self.get_sensor_options(device_id, sensor_id)
        for option in options:
            if option.option_id == option_id:
                return option

        # 批量获取中被跳过的情况，尝试单独读取
        if device_id not in self.devices:
            raise RealSenseError(
                status_code=404, detail=f"Device {device_id} not found"
            )

        dev = self.devices[device_id]
        try:
            sensor_index = int(sensor_id.split("-")[-1])
            sensor = dev.sensors[sensor_index]
        except (ValueError, IndexError):
            raise RealSenseError(
                status_code=404, detail=f"Sensor {sensor_id} not found"
            )

        for opt in sensor.get_supported_options():
            if opt.name == option_id:
                try:
                    current_value = sensor.get_option(opt)
                    option_range = sensor.get_option_range(opt)
                    return OptionInfo(
                        option_id=opt.name,
                        name=opt.name.replace("_", " ").title(),
                        description=sensor.get_option_description(opt),
                        current_value=current_value,
                        default_value=option_range.default,
                        min_value=option_range.min,
                        max_value=option_range.max,
                        step=option_range.step,
                        read_only=sensor.is_option_read_only(opt),
                    )
                except RuntimeError as e:
                    raise RealSenseError(
                        status_code=500,
                        detail=f"Option {option_id} exists but cannot be read: {str(e)}",
                    )

        raise RealSenseError(status_code=404, detail=f"Option {option_id} not found")

    def set_sensor_option(
        self, device_id: str, sensor_id: str, option_id: str, value: Any
    ) -> bool:
        """设置传感器的某个选项值。"""
        self._ensure_device(device_id)
        dev = self.devices[device_id]
        sensor_index = self._parse_sensor_index(device_id, sensor_id)
        sensor = dev.sensors[sensor_index]

        option_value = None
        for option in sensor.get_supported_options():
            if option.name == option_id:
                option_value = option
                break

        if option_value is None:
            raise RealSenseError(
                status_code=404, detail=f"Option {option_id} not found"
            )

        # 校验值范围
        option_range = sensor.get_option_range(option_value)
        if value < option_range.min or value > option_range.max:
            raise RealSenseError(
                status_code=400,
                detail=f"Value {value} is out of range [{option_range.min}, {option_range.max}] for option {option_id}",
            )

        try:
            sensor.set_option(option_value, value)
            return True
        except RuntimeError as e:
            raise RealSenseError(
                status_code=500, detail=f"Failed to set option: {str(e)}"
            )

    @staticmethod
    def _build_stream_profiles(sensor) -> List[SupportedStreamProfile]:
        """从传感器构建支持的流配置列表。"""
        profiles = sensor.get_stream_profiles()
        supported: dict = {}

        for profile in profiles:
            if profile.is_video_stream_profile():
                video_profile = profile.as_video_stream_profile()
                fmt = str(profile.format()).split(".")[1]
                width, height = video_profile.width(), video_profile.height()
                fps = video_profile.fps()
            else:
                fmt = "combined_motion"
                width, height = 640, 480
                fps = 30

            stream_type = profile.stream_type().name
            if profile.stream_type() == rs.stream.infrared:
                stream_index = profile.stream_index()
                if stream_index == 0:
                    continue
                else:
                    stream_type = f"{profile.stream_type().name}-{stream_index}"

            if stream_type not in supported:
                supported[stream_type] = {
                    "stream_type": stream_type,
                    "resolutions": [],
                    "fps": [],
                    "formats": [],
                }

            resolution = (width, height)
            if resolution not in supported[stream_type]["resolutions"]:
                supported[stream_type]["resolutions"].append(resolution)
            if fps not in supported[stream_type]["fps"]:
                supported[stream_type]["fps"].append(fps)
            if fmt not in supported[stream_type]["formats"]:
                supported[stream_type]["formats"].append(fmt)

        return [
            SupportedStreamProfile(
                stream_type=data["stream_type"],
                resolutions=data["resolutions"],
                fps=data["fps"],
                formats=data["formats"],
            )
            for data in supported.values()
        ]
