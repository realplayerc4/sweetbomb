#!/usr/bin/env python3
"""
测试 Intel RealSense 相机
"""

import sys
import time
import numpy as np

def check_pyrealsense():
    """检查 pyrealsense2 是否安装"""
    try:
        import pyrealsense2 as rs
        print(f"✅ pyrealsense2 已安装，版本: {rs.__version__}")
        return True
    except ImportError:
        print("❌ pyrealsense2 未安装")
        print("安装命令: pip3 install pyrealsense2")
        return False

def check_opencv():
    """检查 OpenCV 是否安装"""
    try:
        import cv2
        print(f"✅ OpenCV 已安装，版本: {cv2.__version__}")
        return True
    except ImportError:
        print("⚠️ OpenCV 未安装（可选，用于显示深度图）")
        return False

def list_devices():
    """列出所有 RealSense 设备"""
    import pyrealsense2 as rs

    print("\n=== 搜索 RealSense 设备 ===")
    context = rs.context()
    devices = context.query_devices()

    if len(devices) == 0:
        print("❌ 未找到 RealSense 设备")
        print("提示: 请检查 USB 连接，RealSense 需要 USB 3.0 接口")
        return None

    print(f"✅ 找到 {len(devices)} 个设备:")
    for i, dev in enumerate(devices):
        print(f"\n  设备 {i+1}:")
        print(f"    名称: {dev.get_info(rs.camera_info.name)}")
        print(f"    序列号: {dev.get_info(rs.camera_info.serial_number)}")
        print(f"    固件: {dev.get_info(rs.camera_info.firmware_version)}")

    return devices

def test_basic_streaming():
    """测试基本视频流"""
    import pyrealsense2 as rs

    print("\n=== 测试基本视频流 ===")

    pipeline = rs.pipeline()
    config = rs.config()

    # 尝试启用彩色流和深度流
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    try:
        pipeline_profile = pipeline.start(config)
        print("✅ 视频流已启动")

        # 获取一些帧
        for i in range(10):
            frames = pipeline.wait_for_frames(timeout_ms=5000)
            if frames:
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()

                if color_frame and depth_frame:
                    print(f"  帧 {i+1}: 彩色 {color_frame.get_width()}x{color_frame.get_height()}, "
                          f"深度 {depth_frame.get_width()}x{depth_frame.get_height()}")

        # 测试深度对齐
        print("\n  测试深度对齐...")
        align = rs.align(rs.stream.color)
        frames = pipeline.wait_for_frames(timeout_ms=5000)
        aligned_frames = align.process(frames)
        aligned_depth = aligned_frames.get_depth_frame()
        print(f"✅ 对齐后的深度分辨率: {aligned_depth.get_width()}x{aligned_depth.get_height()}")

        pipeline.stop()
        print("✅ 视频流已停止")
        return True

    except Exception as e:
        print(f"❌ 视频流测试失败: {e}")
        try:
            pipeline.stop()
        except:
            pass
        return False

def test_hardware_metadata():
    """测试硬件元数据"""
    import pyrealsense2 as rs

    print("\n=== 测试硬件元数据 ===")

    try:
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        pipeline_profile = pipeline.start(config)

        device = pipeline_profile.get_device()
        serial_number = device.get_info(rs.camera_info.serial_number)

        # 尝试获取传感器信息
        sensors = device.query_sensors()
        print(f"✅ 找到 {len(sensors)} 个传感器:")
        for i, sensor in enumerate(sensors):
            print(f"  传感器 {i+1}: {sensor.get_info(rs.camera_info.name)}")
            for profile in sensor.get_stream_profiles():
                if profile.stream_type() == rs.stream.color:
                    vprofile = profile.as_video_stream_profile()
                    if vprofile.width() == 640 and vprofile.height() == 480:
                        print(f"    支持 640x480 @ {vprofile.fps()}fps")

        pipeline.stop()
        return True

    except Exception as e:
        print(f"❌ 硬件元数据测试失败: {e}")
        return False

def main():
    """主函数"""
    print("="*60)
    print("Intel RealSense 相机测试脚本")
    print("="*60)

    # 检查依赖
    if not check_pyrealsense():
        sys.exit(1)
    check_opencv()

    # 列出设备
    devices = list_devices()
    if not devices:
        print("\n⚠️  未检测到 RealSense 设备，无法继续测试")
        print("常见原因:")
        print("  1. USB 连接问题 - RealSense 需要 USB 3.0 接口")
        print("  2. 驱动未加载 - 尝试运行: sudo modprobe uvcvideo")
        print("  3. 设备固件过旧 - 使用 RealSense Viewer 更新固件")
        sys.exit(1)

    # 运行测试
    print("\n" + "="*60)
    print("开始功能测试")
    print("="*60)

    results = []

    results.append(("基本视频流", test_basic_streaming()))
    results.append(("硬件元数据", test_hardware_metadata()))

    # 总结
    print("\n" + "="*60)
    print("测试总结")
    print("="*60)

    for name, passed in results:
        status = "✅ 通过" if passed else "❌ 失败"
        print(f"  {name}: {status}")

    all_passed = all(r[1] for r in results)

    if all_passed:
        print("\n✅ 所有测试通过! RealSense 相机工作正常。")
    else:
        print("\n⚠️  部分测试失败，请检查错误信息。")

if __name__ == "__main__":
    main()
