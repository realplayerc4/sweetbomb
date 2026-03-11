"""
假设备设置模块

本模块提供用于测试的假 RealSense 设备创建功能。
用于在测试环境中模拟真实设备的行为。
"""
from .pyrealsense_mock import context, create_mock_device

def setup_fake_devices():
    """
    创建一组假 RealSense 设备用于测试
    返回包含所有 mock 对象的字典
    """
    # 创建 mock 上下文
    mock_context = context()

    # 创建两个带有深度和颜色传感器的假设备
    fake_device1 = create_mock_device(
        "device1", "Test Device 1", with_depth=True, with_color=True
    )
    fake_device2 = create_mock_device(
        "device2", "Test Device 2", with_depth=True, with_color=True
    )

    # 将设备添加到上下文
    mock_context.add_device(fake_device1)
    mock_context.add_device(fake_device2)

    # 提取传感器以便在测试中轻松访问
    depth_sensors = []
    color_sensors = []

    # 从设备 1 收集传感器
    for i, sensor in enumerate(fake_device1.sensors):
        if sensor.is_depth_sensor():
            depth_sensors.append(sensor)
        elif sensor.is_color_sensor():
            color_sensors.append(sensor)

    # 从设备 2 收集传感器
    for i, sensor in enumerate(fake_device2.sensors):
        if sensor.is_depth_sensor():
            depth_sensors.append(sensor)
        elif sensor.is_color_sensor():
            color_sensors.append(sensor)

    # 在字典中返回所有 mock 对象
    return {
        "context": mock_context,
        "devices": [fake_device1, fake_device2],
        "depth_sensors": depth_sensors,
        "color_sensors": color_sensors,
    }
